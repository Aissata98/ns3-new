#ifndef EDC_CLOSURE_PHY_H
#define EDC_CLOSURE_PHY_H

// Local ns-3 extension. The installed AquaSim library and historic binaries are
// not patched. This retains AquaSim's channel/collision machinery, and replaces
// its advance-charged energy hooks with elapsed, finite, shared-owner accounting.
#include "closure_energy.h"
#include "ns3/aqua-sim-phy-cmn.h"
#include "ns3/aqua-sim-net-device.h"
#include "ns3/aqua-sim-energy-model.h"
#include "ns3/aqua-sim-header.h"
#include "ns3/core-module.h"
#include <functional>

namespace ns3 {
// Simulator-only power provenance; PacketTags are not additional wire bytes.
// A receiver cannot decode a frame whose emitting device lost power midway.
class ReviewedSupplyTag : public Tag {
 public:
  static TypeId GetTypeId() { static TypeId id=TypeId("ns3::ReviewedSupplyTag").SetParent<Tag>().AddConstructor<ReviewedSupplyTag>(); return id; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 28; }
  void Serialize(TagBuffer b) const override { b.WriteU32(node); b.WriteU64(startNs); b.WriteU64(endNs); b.WriteU64(outageOrdinal); }
  void Deserialize(TagBuffer b) override { node=b.ReadU32(); startNs=b.ReadU64(); endNs=b.ReadU64(); outageOrdinal=b.ReadU64(); }
  void Print(std::ostream& out) const override { out << node << ':' << startNs << ':' << endNs; }
  uint32_t node=0; uint64_t startNs=0, endNs=0, outageOrdinal=0;
};
NS_OBJECT_ENSURE_REGISTERED(ReviewedSupplyTag);
class ReviewedReceiveSupplyTag : public ReviewedSupplyTag {
 public:
  static TypeId GetTypeId() { static TypeId id=TypeId("ns3::ReviewedReceiveSupplyTag").SetParent<ReviewedSupplyTag>().AddConstructor<ReviewedReceiveSupplyTag>(); return id; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
};
NS_OBJECT_ENSURE_REGISTERED(ReviewedReceiveSupplyTag);

// Local driver completion metadata only: never a wire ACK or receiver oracle.
class ReviewedLocalTxTag : public Tag {
 public:
  static TypeId GetTypeId() { static TypeId id=TypeId("ns3::ReviewedLocalTxTag").SetParent<Tag>().AddConstructor<ReviewedLocalTxTag>(); return id; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 13; }
  void Serialize(TagBuffer b) const override { b.WriteU64(token); b.WriteU8(attempt); b.WriteU16(fragment); b.WriteU16(fragments); }
  void Deserialize(TagBuffer b) override { token=b.ReadU64(); attempt=b.ReadU8(); fragment=b.ReadU16(); fragments=b.ReadU16(); }
  void Print(std::ostream& out) const override { out << token << ':' << unsigned(attempt) << ':' << fragment << '/' << fragments; }
  uint64_t token=0; uint8_t attempt=0; uint16_t fragment=0, fragments=0;
};
NS_OBJECT_ENSURE_REGISTERED(ReviewedLocalTxTag);

class ReviewedAquaSimPhy : public AquaSimPhyCmn {
 public:
  using LocalTxOutcome = std::function<void(const ReviewedLocalTxTag&, bool, double)>;
  void SetLocalTxOutcome(LocalTxOutcome callback) { m_localTxOutcome=std::move(callback); }
  static TypeId GetTypeId() {
    static TypeId id = TypeId("ns3::ReviewedAquaSimPhy")
        .SetParent<AquaSimPhyCmn>().AddConstructor<ReviewedAquaSimPhy>();
    return id;
  }
  void SetLedger(std::shared_ptr<closure_energy::PowerLedger> ledger) {
    NS_ABORT_MSG_IF(!ledger || m_ledger, "A PHY requires one immutable owner-ledger binding");
    m_ledger = std::move(ledger);
    Supplies()[GetNetDevice()->GetNode()->GetId()] = Supply{m_ledger,{}};
    Sync(); ScheduleBoundary();
  }
  std::shared_ptr<closure_energy::PowerLedger> Ledger() const { return m_ledger; }
  std::uint64_t RejectedHalfDuplexTx() const { return m_rejectedTx; }
  std::uint64_t RejectedHalfDuplexRx() const { return m_rejectedRx; }
  std::uint64_t ReceivedWaveforms() const { return m_received; }
  std::uint64_t FailedReceiveFlags() const { return m_failedFlags; }
  std::uint64_t RejectedEmissionSupply() const { return m_supplyFailures; }
  static bool EmissionSupplyValid(const ReviewedSupplyTag& tag) {
    auto it=Supplies().find(tag.node);
    if(it==Supplies().end()) return false;
    auto source=it->second.ledger.lock();
    if(!source || tag.outageOrdinal>it->second.outages.size()) return false;
    source->Advance(Simulator::Now().GetSeconds());
    const double end=static_cast<double>(tag.endNs)*1e-9;
    if(source->DepletedAt()+1e-9<end) return false;
    // The ordinal distinguishes power-off after TX at the very same ns-3
    // timestamp from an earlier completed power-off/power-on cycle.
    return tag.outageOrdinal==it->second.outages.size() ||
           it->second.outages.at(tag.outageOrdinal)+1e-9>=end;
  }

  void Sync() {
    if (!m_ledger) return;
    m_ledger->Advance(Simulator::Now().GetSeconds());
    if (EM()) EM()->SetEnergy(m_ledger->Budget()->Remaining());
    if (m_ledger->Budget()->Remaining() <= 0) {
      m_PoweredOn = false;
      SetDeviceStatus(DISABLE);
    }
  }
  std::uint64_t AddExternalLoad(double duration_s, double watts) {
    NS_ABORT_MSG_IF(!m_ledger, "Unbound reviewed energy ledger");
    auto id = m_ledger->AddLoad(Simulator::Now().GetSeconds(), duration_s, watts);
    Sync(); ScheduleBoundary(); return id;
  }
  void StopExternalLoad(std::uint64_t id) {
    NS_ABORT_MSG_IF(!m_ledger, "Unbound reviewed energy ledger");
    m_ledger->StopLoad(Simulator::Now().GetSeconds(), id);
    Sync(); ScheduleBoundary();
  }
  void SetBasePower(double watts) {
    NS_ABORT_MSG_IF(!m_ledger, "Unbound reviewed energy ledger");
    m_ledger->SetBasePower(Simulator::Now().GetSeconds(), watts);
    Sync(); ScheduleBoundary();
  }
  bool PktTransmit(Ptr<Packet> packet, int channel = 0) override {
    NS_ABORT_MSG_IF(!m_ledger, "Reviewed PHY used without physical energy inputs");
    Sync();
    const double now = Simulator::Now().GetSeconds();
    if (!Alive() || m_ledger->TxActive(now) || m_ledger->RxActive(now)) {
      ++m_rejectedTx;
      RestoreRadioStatus();
      NotifyLocalTx(packet, false, now);
      return false;
    }
    // The MAC may already have changed its status to SEND. The ledger, rather
    // than that overwritten flag, decides whether another waveform is active.
    SetDeviceStatus(SEND);
    auto inspect=packet->Copy();
    AquaSimPacketStamp stamp; AquaSimHeader header;
    inspect->RemoveHeader(stamp); inspect->PeekHeader(header);
    ReviewedSupplyTag tag, previous;
    packet->RemovePacketTag(previous);
    tag.node=GetNetDevice()->GetNode()->GetId();
    tag.outageOrdinal=Supplies().at(tag.node).outages.size();
    tag.startNs=Simulator::Now().GetNanoSeconds();
    tag.endNs=(Simulator::Now()+header.GetTxTime()).GetNanoSeconds();
    packet->AddPacketTag(tag);
    const bool accepted=AquaSimPhyCmn::PktTransmit(packet, channel);
    NotifyLocalTx(packet, accepted, accepted ? double(tag.endNs)*1e-9 : now);
    return accepted;
  }
  void SignalCacheCallback(Ptr<Packet> packet) override {
    Sync();
    if (!Alive()) return; // A depleted receiver cannot finish a pending frame.
    ReviewedSupplyTag tag;
    if (!packet->PeekPacketTag(tag)) { ++m_supplyFailures; return; }
    if(!EmissionSupplyValid(tag)) { ++m_supplyFailures; return; }
    ReviewedReceiveSupplyTag receiverTag;
    if(!packet->PeekPacketTag(receiverTag) || !EmissionSupplyValid(receiverTag))
      { ++m_supplyFailures; return; }
    AquaSimPhyCmn::SignalCacheCallback(packet);
  }
  void UpdateIdleEnergy() override {
    // The base constructor schedules one call at t=1 s. No base energy hook is
    // invoked: otherwise physical activity would be charged twice.
    Sync(); ScheduleBoundary();
  }
  void PowerOff() override {
    if (!m_ledger) { AquaSimPhyCmn::PowerOff(); return; }
    const double now=Simulator::Now().GetSeconds();
    if(m_ledger->Awake())
      Supplies()[GetNetDevice()->GetNode()->GetId()].outages.push_back(now);
    m_ledger->SetAwake(now, false);
    m_PoweredOn = false;
    SetDeviceStatus(SLEEP);
    Sync(); ScheduleBoundary();
  }
  void PowerOn() override {
    if (!m_ledger) { AquaSimPhyCmn::PowerOn(); return; }
    Sync();
    if (m_ledger->Budget()->Remaining() <= 0) return;
    m_ledger->SetAwake(Simulator::Now().GetSeconds(), true);
    m_PoweredOn = true;
    SetDeviceStatus(NIDLE);
    ScheduleBoundary();
  }
  void StatusShift(double) override {
    // Acoustic reception is nonpreemptible in the reviewed half-duplex model.
    // PktTransmit enforces this even if a MAC requests a RX-to-TX status shift.
    Sync();
  }
 protected:
  Ptr<Packet> PrevalidateIncomingPkt(Ptr<Packet> packet) override {
    NS_ABORT_MSG_IF(!m_ledger, "Reviewed PHY used without physical energy inputs");
    Sync();
    const double now = Simulator::Now().GetSeconds();
    if (!Alive() || m_ledger->TxActive(now)) {
      ++m_rejectedRx;
      RestoreRadioStatus();
      return nullptr;
    }
    // Some legacy MAC/PHY callbacks reset the device flag. Reconstruct it from
    // actual unfinished reception intervals before AquaSim's collision test.
    SetDeviceStatus(m_ledger->RxActive(now) ? RECV : NIDLE);
    // AquaSimChannel reuses AquaSimHeader::TxTime for propagation delay.
    // Restore emitter waveform duration before the PHY energy/collision hooks;
    // propagation is already represented by the channel's scheduled arrival.
    ReviewedSupplyTag emitter;
    if(!packet->PeekPacketTag(emitter) || emitter.endNs<=emitter.startNs)
      { ++m_supplyFailures; return nullptr; }
    AquaSimPacketStamp stamp; AquaSimHeader header;
    packet->RemoveHeader(stamp); packet->RemoveHeader(header);
    header.SetTxTime(NanoSeconds(emitter.endNs-emitter.startNs));
    packet->AddHeader(header); packet->AddHeader(stamp);
    ReviewedReceiveSupplyTag receive, previous;
    packet->RemovePacketTag(previous);
    receive.node=GetNetDevice()->GetNode()->GetId();
    receive.startNs=Simulator::Now().GetNanoSeconds();
    receive.endNs=(Simulator::Now()+header.GetTxTime()).GetNanoSeconds();
    receive.outageOrdinal=Supplies().at(receive.node).outages.size();
    packet->AddPacketTag(receive);
    return AquaSimPhyCmn::PrevalidateIncomingPkt(packet);
  }
  void UpdateTxEnergy(Time duration) override {
    const double now = Simulator::Now().GetSeconds();
    NS_ABORT_MSG_IF(!m_ledger || !m_ledger->BeginTx(now, duration.GetSeconds()),
                    "PHY admitted an infeasible half-duplex transmission");
    Sync(); ScheduleBoundary();
  }
  void UpdateRxEnergy(Time duration, bool error) override {
    const double now = Simulator::Now().GetSeconds();
    NS_ABORT_MSG_IF(!m_ledger || !m_ledger->BeginRx(now, duration.GetSeconds()),
                    "PHY admitted an infeasible reception");
    ++m_received;
    if (error) ++m_failedFlags;
    Sync(); ScheduleBoundary();
  }
  void EnergyDeplete() override {
    Sync(); m_PoweredOn = false;
    SetDeviceStatus(DISABLE);
  }
  void DoDispose() override {
    if(GetNetDevice() && GetNetDevice()->GetNode()) Supplies().erase(GetNetDevice()->GetNode()->GetId());
    m_boundary.Cancel(); m_ledger.reset();
    AquaSimPhyCmn::DoDispose();
  }
 private:
  void NotifyLocalTx(Ptr<const Packet> packet, bool accepted, double endAt) {
    ReviewedLocalTxTag tag;
    if(m_localTxOutcome && packet->PeekPacketTag(tag)) m_localTxOutcome(tag,accepted,endAt);
  }
  LocalTxOutcome m_localTxOutcome;
  struct Supply { std::weak_ptr<closure_energy::PowerLedger> ledger; std::vector<double> outages; };
  static std::map<uint32_t,Supply>& Supplies() { static std::map<uint32_t,Supply> records; return records; }
  bool Alive() const { return m_ledger && m_ledger->Awake() && m_ledger->Budget()->Remaining() > 0; }
  void SetDeviceStatus(TransStatus status) {
    if (!GetNetDevice() || m_settingStatus) return;
    // AquaSim's setter calls MAC::PowerOff again if its previous status was
    // SLEEP. Break that legacy feedback recursion while writing the status.
    // The ledger still blocks all sends when asleep or exhausted.
    m_settingStatus = true;
    const bool powered = m_PoweredOn;
    m_PoweredOn = true;
    GetNetDevice()->SetTransmissionStatus(status);
    m_PoweredOn = powered;
    m_settingStatus = false;
  }
  void RestoreRadioStatus() {
    if (!GetNetDevice() || !m_ledger) return;
    const double now = Simulator::Now().GetSeconds();
    SetDeviceStatus(!Alive() ? DISABLE : m_ledger->TxActive(now) ? SEND : m_ledger->RxActive(now) ? RECV : NIDLE);
  }
  void AtBoundary() { Sync(); RestoreRadioStatus(); ScheduleBoundary(); }
  void ScheduleBoundary() {
    m_boundary.Cancel();
    if (!m_ledger || m_ledger->Budget()->Remaining() <= 0) return;
    const double now = Simulator::Now().GetSeconds();
    const double next = m_ledger->NextBoundary(now);
    if (next > now && std::isfinite(next))
      m_boundary = Simulator::Schedule(std::max(TimeStep(1),Seconds(next - now)),
                                     &ReviewedAquaSimPhy::AtBoundary, this);
  }
  EventId m_boundary;
  bool m_settingStatus = false;
  std::shared_ptr<closure_energy::PowerLedger> m_ledger;
  std::uint64_t m_rejectedTx = 0, m_rejectedRx = 0, m_received = 0, m_failedFlags = 0;
  std::uint64_t m_supplyFailures = 0;
};
NS_OBJECT_ENSURE_REGISTERED(ReviewedAquaSimPhy);
} // namespace ns3
#endif
