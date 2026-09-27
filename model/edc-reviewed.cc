/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/aqua-sim-ng-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "external_workload_replay.h"
#include "closure_phy.h"
#include "closure_ingress.h"
#include "closure_mission.h"
#include "closure_mission_mobility.h"
#include "closure_adaptive.h"
#include "closure_optical.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <unistd.h>
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AuvBasedEdcUwlsn");

// ── EdcHeader: packet header for the real multi-hop BSN→DDN chain ─────────────
// 41 bytes: packet identity/routing/timestamp plus class, hop count, attempt,
// logical-record size, fragment metadata, and previous-hop node ID.
class EdcHeader : public Header
{
  public:
    static TypeId
    GetTypeId()
    {
        static TypeId t = TypeId("ns3::EdcHeader")
                              .SetParent<Header>()
                              .SetGroupName("AuvEdc")
                              .AddConstructor<EdcHeader>();
        return t;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    void Print(std::ostream& os) const override
    {
        os << "EDC id=" << m_id << " src=" << m_srcId << " dst=" << m_dstId
           << " hops=" << (int)m_hops;
    }
    bool IsControl() const { return m_reviewed && m_leg == 255; }
    bool HasWindowMetadata() const { return m_reviewed && !IsControl() && m_windowMetadata; }
    uint32_t GetSerializedSize() const override { return IsControl() ? 61 : m_reviewed ? 51+(HasWindowMetadata()?24:0) : 41; }
    void Serialize(Buffer::Iterator i) const override
    {
        i.WriteHtonU64(m_id);
        i.WriteHtonU32(m_srcId);
        i.WriteHtonU32(m_dstId);
        i.WriteHtonU32(m_nextHopId);
        i.WriteHtonU32(m_createAtMs);
        i.WriteU8(m_prio | (m_reviewed ? 0x80 : 0) | (HasWindowMetadata()?0x40:0));
        i.WriteU8(m_hops);
        i.WriteU8(m_attempt);
        i.WriteHtonU32(m_payloadBytes);
        i.WriteHtonU16(m_fragmentBytes);
        i.WriteHtonU16(m_fragmentIndex);
        i.WriteHtonU16(m_fragmentCount);
        i.WriteHtonU32(m_previousHopId);
        if (m_reviewed)
          {
            i.WriteU8(m_ack ? 1 : 0);
            i.WriteU8(m_leg);
            i.WriteHtonU64(m_token);
            if (IsControl())
              {
                i.WriteU8(m_controlVersion); i.WriteU8(m_controlKind);
                i.WriteHtonU64(m_controlExpiryNs);
              }
            else if(HasWindowMetadata())
              {
                i.WriteHtonU64(m_windowNumericId); i.WriteHtonU64(m_windowPayloadBytes);
                i.WriteHtonU32(m_windowChunkIndex); i.WriteHtonU32(m_windowChunkCount);
              }
          }
    }
    uint32_t Deserialize(Buffer::Iterator i) override
    {
        m_id          = i.ReadNtohU64();
        m_srcId       = i.ReadNtohU32();
        m_dstId       = i.ReadNtohU32();
        m_nextHopId   = i.ReadNtohU32();
        m_createAtMs  = i.ReadNtohU32();
        m_prio        = i.ReadU8();
        m_hops        = i.ReadU8();
        m_attempt     = i.ReadU8();
        m_payloadBytes = i.ReadNtohU32();
        m_fragmentBytes = i.ReadNtohU16();
        m_fragmentIndex = i.ReadNtohU16();
        m_fragmentCount = i.ReadNtohU16();
        m_previousHopId = i.ReadNtohU32();
        m_reviewed = (m_prio & 0x80) != 0;
        m_windowMetadata = m_reviewed && (m_prio & 0x40) != 0;
        m_prio &= m_reviewed ? 0x3f : 0x7f;
        if (m_reviewed)
          {
            m_ack = i.ReadU8() != 0;
            m_leg = i.ReadU8();
            m_token = i.ReadNtohU64();
            if (IsControl())
              {
                m_controlVersion=i.ReadU8(); m_controlKind=i.ReadU8();
                m_controlExpiryNs=i.ReadNtohU64();
              }
            else if(HasWindowMetadata())
              {
                m_windowNumericId=i.ReadNtohU64(); m_windowPayloadBytes=i.ReadNtohU64();
                m_windowChunkIndex=i.ReadNtohU32(); m_windowChunkCount=i.ReadNtohU32();
              }
          }
        return GetSerializedSize();
    }

    uint64_t m_id{0};
    uint32_t m_srcId{0};        // originating BSN NS3 node index
    uint32_t m_dstId{0};        // acoustic endpoint NS3 node index
    uint32_t m_nextHopId{0};    // intended forwarding node ID
    uint32_t m_createAtMs{0};   // packet creation time in ms
    uint8_t  m_prio{1};
    uint8_t  m_hops{0};
    uint8_t  m_attempt{1};
    uint32_t m_payloadBytes{0};
    uint16_t m_fragmentBytes{0};
    uint16_t m_fragmentIndex{0};
    uint16_t m_fragmentCount{1};
    uint32_t m_previousHopId{0};
    bool m_reviewed{false};
    bool m_ack{false};
    uint8_t m_leg{0};
    uint64_t m_token{0};
    uint8_t m_controlVersion{1}, m_controlKind{0};
    uint64_t m_controlExpiryNs{0};
    bool m_windowMetadata{false};
    uint64_t m_windowNumericId{0}, m_windowPayloadBytes{0};
    uint32_t m_windowChunkIndex{0}, m_windowChunkCount{0};
};
NS_OBJECT_ENSURE_REGISTERED(EdcHeader);

// Isolated transport slice. This is not a complete closure-validation build:
// reverse ACKs, complete PHY-state power, durable ingress and feasible motion
// remain separate gates. State is shared by every app on one physical node.
struct ClosureTransportState
{
    using RxIdentity = std::tuple<uint64_t, uint64_t, uint32_t, uint16_t, uint8_t>;
    std::map<RxIdentity, bool> receiveOutcomes;
    double nextTxAvailableS = 0.0;
    double maxAdmissionWaitS = 0.0;
    uint64_t txReservations = 0;
    uint64_t txRangeRejected = 0;
    uint64_t uniqueApplicationRx = 0;
    uint64_t duplicateRxCallbacks = 0;
    uint64_t obsoleteTxCancelled = 0;

    double Reserve(double requestedAt, double duration)
    {
        if (!std::isfinite(requestedAt) || requestedAt < 0.0 ||
            !std::isfinite(duration) || duration <= 0.0)
          throw std::invalid_argument("invalid closure transmit reservation");
        const double start = std::max(requestedAt, nextTxAvailableS);
        if (!std::isfinite(start + duration))
          throw std::invalid_argument("closure transmit reservation overflow");
        nextTxAvailableS = start + duration;
        maxAdmissionWaitS = std::max(maxAdmissionWaitS, start - requestedAt);
        ++txReservations;
        return start;
    }
};

// ── EdcForwardingApp: one instance per sensor node (BSN/DRN/DDN) ──────────────
// Real packet flow through AquaSim-NG acoustic channel.
// Greedy geographic forwarding: always advance toward the destination using the
// farthest reachable neighbour in the correct direction.
class EdcForwardingApp : public Application
{
  public:
    using DdnCb = std::function<void(uint64_t, uint32_t, uint8_t, uint8_t,
                                     uint8_t, uint32_t, uint32_t)>;
    using EnergyCb = std::function<void(uint8_t, uint32_t, double, uint32_t, bool)>;
    using ReceptionCb = std::function<bool(uint32_t, double)>;
    using ReviewedCb = std::function<void(const EdcHeader&)>;
    using ReviewedActiveCb = std::function<bool(const EdcHeader&)>;
    using LocalTxOutcomeCb = ReviewedAquaSimPhy::LocalTxOutcome;
    using ControlReceiveCb = std::function<void(const EdcHeader&, const std::vector<uint8_t>&)>;
    using ControlTraceCb = std::function<void(const std::string&, const EdcHeader&, uint32_t, const std::string&)>;
    using HopCb = std::function<void(uint64_t, uint32_t, uint8_t, uint32_t,
                                     uint8_t, uint32_t, uint32_t, uint32_t, double)>;

    static TypeId
    GetTypeId()
    {
        static TypeId t = TypeId("ns3::EdcForwardingApp")
                              .SetParent<Application>()
                              .SetGroupName("AuvEdc")
                              .AddConstructor<EdcForwardingApp>();
        return t;
    }

    // myId          NS3 node index of this node
    // myPosition    3-D coordinate (m)
    // txRange       effective tx range (adaptive: >= stated range, ensures connectivity)
    // finalDstId    acoustic endpoint NS3 node index (DDN baseline / DRN contribution)
    // finalDstX     x of finalDst
    // allPositions  node-ID ordered positions of all static acoustic nodes
    // isEndpoint    true → this node IS the acoustic endpoint; call m_cb on DATA arrival
    // callbackDstId DDN NS3 index passed to m_cb (self for DDN, paired DDN for wired DRN)
    // cb            invoked when packet arrives: (pktId, srcBsn, prio, hops, ddnNs3Id)
    void Setup(uint32_t myId, const Vector& myPosition, double txRange,
               uint32_t finalDstId, double finalDstX,
               const std::vector<Vector>& allPositions,
               bool isEndpoint, uint32_t callbackDstId, DdnCb cb)
    {
        m_myId          = myId;
        m_myPosition    = myPosition;
        m_range         = txRange;
        m_finalDstId    = finalDstId;
        m_finalDstX     = finalDstX;
        m_allPositions  = allPositions;
        m_isEndpoint    = isEndpoint;
        m_cbDstId       = callbackDstId;
        m_cb            = std::move(cb);
    }

    void SetEnergyCallback(EnergyCb cb) { m_energyCb = std::move(cb); }
    void SetReceptionCallback(ReceptionCb cb) { m_receptionCb = std::move(cb); }
    void SetRelayEnabled(bool enabled) { m_relayEnabled = enabled; }
    void SetRelayCandidateCount(uint32_t count) { m_relayCandidateCount = count; }
    void SetHopCallback(HopCb cb) { m_hopCb = std::move(cb); }
    void SetClosureTransport(std::shared_ptr<ClosureTransportState> state)
    {
        m_closureTransport = std::move(state);
        // Explicit point-to-point senders cannot recruit an unplanned relay
        // when their chosen endpoint is outside the fixed hardware range.
        if (m_closureTransport && !m_relayEnabled) m_relayCandidateCount = 0;
    }
    static uint32_t RunClosureSelfTests();
    static uint32_t RunReviewedFrameSelfTests();
    static uint32_t RunControlSelfTests();
    void SetReviewedCallbacks(ReviewedCb delivered, ReviewedActiveCb active)
    {
        m_reviewedCb = std::move(delivered);
        m_reviewedActive = std::move(active);
    }
    void SetLocalTxOutcome(LocalTxOutcomeCb callback) { m_localTxOutcome=std::move(callback); }
    void SetControlTransport(uint32_t maxPayload, uint32_t maxInflight,
                             ControlReceiveCb receive, ControlTraceCb trace)
    {
        m_controlMaxPayload=maxPayload; m_controlMaxInflight=maxInflight;
        m_controlReceive=std::move(receive); m_controlTrace=std::move(trace);
    }
    bool SendControlPayload(EdcHeader h, const std::vector<uint8_t>& bytes)
    {
        if (!h.IsControl() || h.m_controlVersion!=1 || h.m_controlKind==0 ||
            h.m_ack || bytes.empty() || bytes.size()>m_controlMaxPayload ||
            bytes.size()!=h.m_payloadBytes || !m_closureTransport) return false;
        const uint64_t fragments=(bytes.size()+m_maxFragmentPayloadBytes-1)/m_maxFragmentPayloadBytes;
        if (fragments==0 || fragments>65535) return false;
        if (!ReserveControlSource(h)) return false;
        h.m_srcId=m_myId; h.m_hops=1; h.m_attempt=1;
        h.m_fragmentCount=static_cast<uint16_t>(fragments);
        for(uint32_t f=0; f<fragments; ++f)
          {
            const uint32_t offset=f*m_maxFragmentPayloadBytes;
            h.m_fragmentIndex=static_cast<uint16_t>(f);
            h.m_fragmentBytes=static_cast<uint16_t>(std::min<uint32_t>(m_maxFragmentPayloadBytes,
                static_cast<uint32_t>(bytes.size())-offset));
            QueueClosureFrame(Create<Packet>(bytes.data()+offset,h.m_fragmentBytes),h);
          }
        return true;
    }
    double SendReviewed(EdcHeader h)
    {
        // The optional LOW descriptor belongs to DATA. A real ACK retains
        // token identity and its explicitly sized body, without a copied descriptor.
        if(h.m_ack) h.m_windowMetadata=false;
        NS_ABORT_MSG_IF(!m_closureTransport || !h.m_reviewed,
                        "Reviewed frame requires shared closure transport");
        const uint32_t fragments = (h.m_payloadBytes + m_maxFragmentPayloadBytes - 1) /
                                   m_maxFragmentPayloadBytes;
        NS_ABORT_MSG_IF(fragments == 0 || fragments > 65535, "Invalid reviewed fragmentation");
        h.m_srcId = m_myId;
        h.m_hops = 1;
        h.m_fragmentCount = static_cast<uint16_t>(fragments);
        for (uint32_t f = 0; f < fragments; ++f)
          {
            h.m_fragmentIndex = static_cast<uint16_t>(f);
            h.m_fragmentBytes = static_cast<uint16_t>(std::min(
                m_maxFragmentPayloadBytes, h.m_payloadBytes - f * m_maxFragmentPayloadBytes));
            QueueClosureFrame(Create<Packet>(h.m_fragmentBytes), h);
          }
        return m_closureTransport->nextTxAvailableS;
    }
    void SetFragmentation(uint32_t payloadBytes, double bitRate)
    {
        m_maxFragmentPayloadBytes = std::max(1u, std::min(payloadBytes, 65535u));
        m_bitRate = std::max(1.0, bitRate);
    }

    // Called by GeneratePacket for BSN nodes.
    double SendPkt(uint64_t pktId, uint8_t prio, uint32_t payloadBytes, uint8_t attempt)
    {
        if (!m_socket) return Simulator::Now().GetSeconds();
        const uint32_t fragmentCount32 =
            (payloadBytes + m_maxFragmentPayloadBytes - 1) /
            m_maxFragmentPayloadBytes;
        if (fragmentCount32 > 65535)
          {
            NS_FATAL_ERROR("logical acoustic record requires more than 65535 fragments");
          }
        double offsetSeconds = 0.0;
        for (uint32_t fragmentIndex = 0; fragmentIndex < fragmentCount32;
             ++fragmentIndex)
          {
            const uint32_t sentBytes = fragmentIndex * m_maxFragmentPayloadBytes;
            const uint16_t fragmentBytes = static_cast<uint16_t>(std::min(
                m_maxFragmentPayloadBytes, payloadBytes - sentBytes));
            EdcHeader h;
            h.m_id         = pktId;
            h.m_srcId      = m_myId;
            h.m_dstId      = m_finalDstId;
            h.m_prio       = prio;
            h.m_attempt    = attempt;
            h.m_payloadBytes = payloadBytes;
            h.m_fragmentBytes = fragmentBytes;
            h.m_fragmentIndex = static_cast<uint16_t>(fragmentIndex);
            h.m_fragmentCount = static_cast<uint16_t>(fragmentCount32);
            h.m_hops       = 1;
            h.m_createAtMs = static_cast<uint32_t>(Simulator::Now().GetMilliSeconds());
            h.m_nextHopId = BestNextHopId(m_finalDstId, m_finalDstX);
            h.m_previousHopId = m_myId;
            auto pkt = Create<Packet>(fragmentBytes);
            if (m_closureTransport)
              QueueClosureFrame(pkt, h);
            else
              Simulator::Schedule(Seconds(offsetSeconds),
                                  &EdcForwardingApp::TransmitFrame, this, pkt, h);
            offsetSeconds +=
                (fragmentBytes + h.GetSerializedSize()) * 8.0 / m_bitRate;
          }
        return m_closureTransport ? m_closureTransport->nextTxAvailableS
                                  : Simulator::Now().GetSeconds() + offsetSeconds;
    }

  private:
    bool ReserveControlSource(const EdcHeader& h)
    {
        const uint64_t now=static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
        if (!m_controlMaxInflight || h.m_token==0 || h.m_controlExpiryNs<=now) return false;
        for(auto it=m_controlSourceExpiry.begin();it!=m_controlSourceExpiry.end();)
          if(it->second<=now) it=m_controlSourceExpiry.erase(it); else ++it;
        // This is a local software reservation, not a remote receipt. Keep it
        // through the advertised lifetime even if transmission finishes early.
        // It bounds admitted control bodies without inventing an ACK or a
        // PHY completion callback for a best-effort control message.
        if(m_controlSourceExpiry.size()>=m_controlMaxInflight ||
           m_controlSourceExpiry.count(h.m_token)) return false;
        m_controlSourceExpiry.emplace(h.m_token,h.m_controlExpiryNs);
        return true;
    }

    void QueueClosureFrame(Ptr<Packet> pkt, EdcHeader h)
    {
        const double now = Simulator::Now().GetSeconds();
        const double duration = (h.m_fragmentBytes + h.GetSerializedSize()) * 8.0 / m_bitRate;
        const double start = m_closureTransport->Reserve(now, duration);
        Simulator::Schedule(Seconds(start - now),
                            &EdcForwardingApp::TransmitFrame, this, pkt, h);
    }

    void TransmitFrame(Ptr<Packet> pkt, EdcHeader h)
    {
        if (h.m_reviewed && m_reviewedActive && !m_reviewedActive(h))
          {
            ++m_closureTransport->obsoleteTxCancelled;
            if (h.IsControl() && m_controlTrace)
              m_controlTrace("control_fragment_rejected",h,m_myId,"expired_or_local_sender_unavailable");
            ReportLocalTxRejected(h);
            return;
          }
        if (m_closureTransport)
          {
            // Re-evaluate a moving endpoint when the reserved handoff starts.
            // A disconnected hop stays disconnected; do not enlarge its range.
            h.m_nextHopId = BestNextHopId(h.m_dstId, PositionOf(h.m_dstId).x);
            // Upstream DATA/ACKs may use static relays; the three offload legs
            // remain point-to-point and cannot silently add a relay topology.
            if (h.m_reviewed && h.m_leg != 1 && h.m_nextHopId != h.m_dstId)
              h.m_nextHopId = m_myId;
            h.m_previousHopId = m_myId;
            if (h.m_nextHopId == m_myId || h.m_nextHopId >= m_allPositions.size() ||
                CalculateDistance(PositionOf(h.m_nextHopId), PositionOf(m_myId)) > m_range)
              {
                ++m_closureTransport->txRangeRejected;
                if (h.IsControl() && m_controlTrace)
                  m_controlTrace("control_fragment_rejected",h,m_myId,"fixed_range_or_endpoint");
                ReportLocalTxRejected(h);
                return;
              }
          }
        const double distance =
            CalculateDistance(PositionOf(h.m_nextHopId), PositionOf(m_myId));
        if (m_energyCb && !h.IsControl())
          m_energyCb(h.m_prio, h.m_fragmentBytes + (h.GetSerializedSize()-41),
                     distance, m_myId, true);
        if (m_hopCb && !h.IsControl())
          m_hopCb(h.m_id, h.m_srcId, h.m_prio, h.m_fragmentBytes, h.m_attempt,
                  m_myId, h.m_nextHopId, h.m_dstId, distance);
        if (h.IsControl() && m_controlTrace)
          m_controlTrace("control_fragment_handoff",h,m_myId,"application_to_mac_not_phy_completion");
        if (h.m_reviewed && !h.m_ack && !h.IsControl() && m_localTxOutcome)
          {
            ReviewedLocalTxTag tag, previous;
            tag.token=h.m_token; tag.attempt=h.m_attempt;
            tag.fragment=h.m_fragmentIndex; tag.fragments=h.m_fragmentCount;
            pkt->RemovePacketTag(previous); pkt->AddPacketTag(tag);
          }
        pkt->AddHeader(h);
        if (!SendToNode(pkt, h.m_nextHopId))
          {
            if (h.IsControl() && m_controlTrace)
              m_controlTrace("control_fragment_rejected",h,m_myId,"device_admission");
            ReportLocalTxRejected(h);
          }
    }

    void StartApplication() override
    {
        auto tid = TypeId::LookupByName("ns3::PacketSocketFactory");
        m_socket  = Socket::CreateSocket(GetNode(), tid);
        m_socket->Bind();
        m_socket->SetRecvCallback(MakeCallback(&EdcForwardingApp::OnRecv, this));
        Ptr<AquaSimNetDevice> device =
            DynamicCast<AquaSimNetDevice>(GetNode()->GetDevice(0));
        device->GetRouting()->TraceConnect(
            "PacketReceived", "",
            MakeCallback(&EdcForwardingApp::OnRoutingDelivered, this));
    }

    void StopApplication() override
    {
        if (m_socket) { m_socket->Close(); m_socket = nullptr; }
    }

    void OnRecv(Ptr<Socket> sock)
    {
        Ptr<Packet> pkt;
        while ((pkt = sock->Recv()))
        {
            ProcessReceivedPacket(pkt);
        }
    }

    void OnRoutingDelivered(std::string, Ptr<const Packet> packet)
    {
        Ptr<Packet> copy = packet->Copy();
        AquaSimHeader ash;
        if (copy->RemoveHeader(ash) == 0) return;
        ProcessReceivedPacket(copy);
    }

    void ProcessReceivedPacket(Ptr<Packet> pkt)
    {
        const uint64_t physicalPacketUid = pkt->GetUid();
        EdcHeader h;
        if (pkt->RemoveHeader(h) == 0) return;
        if (h.m_nextHopId != m_myId) return;
        if (h.m_previousHopId >= m_allPositions.size()) return;
        const double hopDistance =
            CalculateDistance(PositionOf(m_myId), PositionOf(h.m_previousHopId));
        bool firstApplicationCallback = true;
        if (m_closureTransport)
          {
            // Packet::Copy preserves UID across socket/routing callbacks.
            // Include logical identity, upstream node, fragment and attempt so
            // a new application retransmission is charged and sampled anew.
            const ClosureTransportState::RxIdentity key{
                physicalPacketUid, h.m_id, h.m_previousHopId,
                h.m_fragmentIndex, h.m_attempt};
            auto observed = m_closureTransport->receiveOutcomes.find(key);
            bool accepted;
            if (observed == m_closureTransport->receiveOutcomes.end())
              {
                ++m_closureTransport->uniqueApplicationRx;
                if (m_energyCb && !h.IsControl())
                  m_energyCb(h.m_prio, h.m_fragmentBytes + (h.GetSerializedSize()-41),
                             hopDistance, m_myId, false);
                accepted = !m_receptionCb || m_receptionCb(
                    h.m_fragmentBytes + (h.m_reviewed ? h.GetSerializedSize() : 0), hopDistance);
                m_closureTransport->receiveOutcomes.emplace(key, accepted);
              }
            else
              {
                ++m_closureTransport->duplicateRxCallbacks;
                firstApplicationCallback = false;
                accepted = observed->second;
              }
            if (!accepted) return;
          }
        else if (m_receptionCb && !m_receptionCb(h.m_fragmentBytes, hopDistance)) return;
        if (h.m_reviewed)
          {
            if (!firstApplicationCallback) return;
            ProcessReviewedFrame(pkt, h);
            return;
          }
        uint64_t seenKey = h.m_id * 1315423911ULL ^
            (static_cast<uint64_t>(h.m_attempt) << 24) ^ h.m_fragmentIndex;
        if (m_seen.count(seenKey)) return;
        m_seen.insert(seenKey);
        if (m_energyCb && !m_closureTransport)
          m_energyCb(h.m_prio, h.m_fragmentBytes,
                     hopDistance,
                     m_myId, false);
        h.m_hops++;
        if (m_isEndpoint && h.m_dstId == m_myId)
          {
            const uint64_t assemblyKey = h.m_id * 257ULL + h.m_attempt;
            auto& received = m_receivedFragments[assemblyKey];
            received.insert(h.m_fragmentIndex);
            if (received.size() == h.m_fragmentCount)
              {
                m_receivedFragments.erase(assemblyKey);
                m_cb(h.m_id, h.m_srcId, h.m_prio, h.m_hops, h.m_attempt,
                     h.m_payloadBytes, m_cbDstId);
              }
            return;
          }
        if (!m_relayEnabled || h.m_dstId == m_myId) return;
        h.m_nextHopId = BestNextHopId(h.m_dstId, PositionOf(h.m_dstId).x);
        h.m_previousHopId = m_myId;
        if (m_closureTransport)
          {
            QueueClosureFrame(pkt, h);
            return;
          }
        if (m_energyCb)
          m_energyCb(h.m_prio, h.m_fragmentBytes,
                     CalculateDistance(PositionOf(h.m_nextHopId), PositionOf(m_myId)),
                     m_myId, true);
        if (m_hopCb)
          m_hopCb(h.m_id, h.m_srcId, h.m_prio, h.m_fragmentBytes, h.m_attempt,
                  m_myId, h.m_nextHopId, h.m_dstId,
                  CalculateDistance(PositionOf(h.m_nextHopId), PositionOf(m_myId)));
        pkt->AddHeader(h);
        SendToNode(pkt, h.m_nextHopId);
    }

    void ProcessReviewedFrame(Ptr<Packet> pkt, EdcHeader h)
    {
        if (h.IsControl()) { ProcessControlFrame(pkt,h); return; }
        if (!m_reviewedCb) return;
        const auto key = std::make_tuple(h.m_token, h.m_ack, h.m_attempt, h.m_fragmentIndex);
        const auto assembly = std::make_tuple(h.m_token, h.m_ack, h.m_attempt);
        if (!m_reviewedSeen.insert(key).second)
          {
            // A genuinely new physical copy of completed DATA is re-ACKed;
            // duplicate socket/routing callbacks were filtered above.
            if (!h.m_ack && h.m_dstId == m_myId && m_reviewedCompleted.count(assembly))
              m_reviewedCb(h);
            return;
          }
        ++h.m_hops;
        if (h.m_dstId == m_myId)
          {
            auto& received = m_reviewedFragments[assembly];
            received.insert(h.m_fragmentIndex);
            if (received.size() == h.m_fragmentCount)
              {
                m_reviewedFragments.erase(assembly);
                m_reviewedCompleted.insert(assembly);
                m_reviewedCb(h);
              }
            return;
          }
        if (h.m_leg == 1) QueueClosureFrame(pkt, h);
    }

    struct ControlAssembly
    {
        EdcHeader header;
        std::vector<uint8_t> bytes;
        std::set<uint16_t> received;
        bool complete=false;
    };

    void ProcessControlFrame(Ptr<Packet> pkt, const EdcHeader& h)
    {
        const uint64_t now=static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
        auto reject=[&](const char* reason) {
          if(m_controlTrace) m_controlTrace("control_receive_rejected",h,m_myId,reason);
        };
        if(!m_controlReceive || !m_controlMaxPayload || !m_controlMaxInflight ||
           !h.IsControl() || h.m_controlVersion!=1 || h.m_controlKind==0 || h.m_ack ||
           h.m_dstId!=m_myId || h.m_srcId!=h.m_previousHopId || h.m_attempt!=1 ||
           h.m_controlExpiryNs<=now || h.m_payloadBytes==0 || h.m_payloadBytes>m_controlMaxPayload)
          { reject("version_endpoint_expiry_or_payload_limit"); return; }
        const uint64_t count=(uint64_t(h.m_payloadBytes)+m_maxFragmentPayloadBytes-1)/m_maxFragmentPayloadBytes;
        const uint64_t offset=uint64_t(h.m_fragmentIndex)*m_maxFragmentPayloadBytes;
        if(count>65535 || h.m_fragmentCount!=count || h.m_fragmentIndex>=count ||
           offset>=h.m_payloadBytes || h.m_fragmentBytes!=std::min<uint64_t>(m_maxFragmentPayloadBytes,h.m_payloadBytes-offset) ||
           pkt->GetSize()!=h.m_fragmentBytes)
          { reject("malformed_control_fragment"); return; }
        // Bounded receiver-local state, including completed identities until
        // their on-wire expiry. No source pending-map or queue is consulted.
        for(auto it=m_controlAssemblies.begin();it!=m_controlAssemblies.end();)
          if(it->second.header.m_controlExpiryNs<=now) it=m_controlAssemblies.erase(it); else ++it;
        const auto key=std::make_pair(h.m_srcId,h.m_token);
        auto found=m_controlAssemblies.find(key);
        if(found==m_controlAssemblies.end())
          {
            if(m_controlAssemblies.size()>=m_controlMaxInflight)
              { reject("receiver_reassembly_slot_limit"); return; }
            ControlAssembly a; a.header=h; a.bytes.resize(h.m_payloadBytes);
            found=m_controlAssemblies.emplace(key,std::move(a)).first;
          }
        auto& a=found->second;
        if(a.header.m_id!=h.m_id || a.header.m_controlKind!=h.m_controlKind ||
           a.header.m_controlExpiryNs!=h.m_controlExpiryNs ||
           a.header.m_payloadBytes!=h.m_payloadBytes || a.header.m_fragmentCount!=h.m_fragmentCount)
          { reject("inconsistent_control_fragment_identity"); return; }
        if(a.complete || !a.received.insert(h.m_fragmentIndex).second) return;
        pkt->CopyData(a.bytes.data()+offset,h.m_fragmentBytes);
        if(a.received.size()!=h.m_fragmentCount) return;
        a.complete=true;
        auto body=std::move(a.bytes);
        a.received.clear();
        m_controlReceive(h,body);
    }

    // Greedy: farthest reachable node in the direction of destX.
    uint32_t BestNextHopId(uint32_t destinationId, double destX) const
    {
        const Vector myPosition = PositionOf(m_myId);
        if (destinationId < m_allPositions.size() &&
            CalculateDistance(PositionOf(destinationId), myPosition) <= m_range + 0.1)
          return destinationId;
        double dir     = (destX >= myPosition.x) ? 1.0 : -1.0;
        uint32_t best  = m_myId;
        double bestDist = 0.0;
        for (uint32_t id = 0; id < m_allPositions.size(); ++id)
        {
            if (id == m_myId) continue;
            // Intermediate upstream relays are static BSN/DRN/DDN devices.
            // A sink or the moving AUV may still be an explicit destination
            // through the direct-reachability return above, but never a greedy
            // waypoint in the bottom access chain.
            if (id >= m_relayCandidateCount) continue;
            const Vector candidate = PositionOf(id);
            double nx = candidate.x;
            double delta = nx - myPosition.x;
            if (std::abs(delta) < 0.01)   continue; // self
            if (dir * delta <= 0.0)        continue; // wrong dir
            double d = CalculateDistance(candidate, myPosition);
            if (d > m_range + 0.1)         continue; // out of range
            if (d > bestDist) { bestDist = d; best = id; }
        }
        return best;
    }

    void ReportLocalTxRejected(const EdcHeader& h)
    {
        if (!h.m_reviewed || h.m_ack || h.IsControl() || !m_localTxOutcome) return;
        ReviewedLocalTxTag tag;
        tag.token=h.m_token; tag.attempt=h.m_attempt;
        tag.fragment=h.m_fragmentIndex; tag.fragments=h.m_fragmentCount;
        m_localTxOutcome(tag,false,Simulator::Now().GetSeconds());
    }

    bool SendToNode(Ptr<Packet> pkt, uint32_t nodeId)
    {
        if (nodeId == m_myId || nodeId >= NodeList::GetNNodes()) return false;
        Ptr<Node> target = NodeList::GetNode(nodeId);
        if (!target || target->GetNDevices() == 0) return false;
        Ptr<AquaSimNetDevice> sourceDevice =
            DynamicCast<AquaSimNetDevice>(GetNode()->GetDevice(0));
        return sourceDevice->Send(pkt, target->GetDevice(0)->GetAddress(), 0);
    }

    Vector PositionOf(uint32_t nodeId) const
    {
        if (nodeId < NodeList::GetNNodes())
          {
            Ptr<Node> node = NodeList::GetNode(nodeId);
            if (node)
              {
                Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
                if (mobility) return mobility->GetPosition();
              }
          }
        return nodeId < m_allPositions.size() ? m_allPositions[nodeId]
                                               : Vector(0.0, 0.0, 0.0);
    }

    uint32_t m_myId{0};
    Vector   m_myPosition;
    double   m_range{100.0};
    uint32_t m_finalDstId{0};
    double   m_finalDstX{0.0};
    std::vector<Vector> m_allPositions;
    bool     m_isEndpoint{false};
    bool     m_relayEnabled{true};
    uint32_t m_relayCandidateCount{std::numeric_limits<uint32_t>::max()};
    uint32_t m_cbDstId{0};
    DdnCb    m_cb;
    EnergyCb m_energyCb;
    ReceptionCb m_receptionCb;
    HopCb m_hopCb;
    Ptr<Socket> m_socket;
    std::unordered_set<uint64_t> m_seen;
    std::unordered_map<uint64_t, std::unordered_set<uint16_t>> m_receivedFragments;
    uint32_t m_maxFragmentPayloadBytes{4096};
    double m_bitRate{25000.0};
    std::shared_ptr<ClosureTransportState> m_closureTransport;
    ReviewedCb m_reviewedCb;
    ReviewedActiveCb m_reviewedActive;
    LocalTxOutcomeCb m_localTxOutcome;
    ControlReceiveCb m_controlReceive;
    ControlTraceCb m_controlTrace;
    uint32_t m_controlMaxPayload=0, m_controlMaxInflight=0;
    std::map<uint64_t,uint64_t> m_controlSourceExpiry;
    std::map<std::pair<uint32_t,uint64_t>,ControlAssembly> m_controlAssemblies;
    std::set<std::tuple<uint64_t, bool, uint8_t, uint16_t>> m_reviewedSeen;
    std::map<std::tuple<uint64_t, bool, uint8_t>, std::set<uint16_t>> m_reviewedFragments;
    std::set<std::tuple<uint64_t, bool, uint8_t>> m_reviewedCompleted;
};
NS_OBJECT_ENSURE_REGISTERED(EdcForwardingApp);

uint32_t
EdcForwardingApp::RunControlSelfTests()
{
    uint32_t checks=0;
    auto check=[&](bool value,const char* label) {
      NS_ABORT_MSG_IF(!value,"Control transport self-test failed: " << label); ++checks;
    };
    EdcHeader control; control.m_reviewed=true; control.m_leg=255; control.m_controlKind=2;
    control.m_id=(uint64_t(1)<<48)+9; control.m_token=7; control.m_controlExpiryNs=123456789012345;
    auto encoded=Create<Packet>(3); encoded->AddHeader(control); EdcHeader decoded;
    encoded->RemoveHeader(decoded);
    check(control.GetSerializedSize()==61 && decoded.IsControl() && decoded.m_controlVersion==1 &&
          decoded.m_controlKind==2 && decoded.m_id==control.m_id &&
          decoded.m_controlExpiryNs==control.m_controlExpiryNs && encoded->GetSize()==3,
          "versioned control header preserves full nonce and expiry without changing body");
    EdcHeader low; low.m_reviewed=true; low.m_leg=4; low.m_windowMetadata=true;
    low.m_windowNumericId=(uint64_t(1)<<50)+3; low.m_windowPayloadBytes=(uint64_t(1)<<40)+17;
    low.m_windowChunkIndex=70000; low.m_windowChunkCount=70001;
    auto descriptor=Create<Packet>(32); descriptor->AddHeader(low); EdcHeader received;
    descriptor->RemoveHeader(received);
    check(low.GetSerializedSize()==75 && received.HasWindowMetadata() && received.m_prio==low.m_prio &&
          received.m_windowNumericId==low.m_windowNumericId && received.m_windowPayloadBytes==low.m_windowPayloadBytes &&
          received.m_windowChunkIndex==70000 && received.m_windowChunkCount==70001 && descriptor->GetSize()==32,
          "24-byte optional LOW descriptor preserves 64-bit window identity and 32-bit chunk fields");
    EdcHeader plain; EdcHeader data; data.m_reviewed=true; data.m_leg=2;
    check(plain.GetSerializedSize()==41 && data.GetSerializedSize()==51,
          "legacy and descriptor-free reviewed DATA wire lengths remain unchanged");

    auto app=CreateObject<EdcForwardingApp>(); app->m_myId=7;
    app->SetFragmentation(4,6000); app->SetClosureTransport(std::make_shared<ClosureTransportState>());
    uint32_t deliveries=0,rejections=0; std::vector<uint8_t> delivered;
    app->SetControlTransport(16,2,[&](const EdcHeader&,const std::vector<uint8_t>& body) {
      ++deliveries; delivered=body;
    },[&](const std::string& event,const EdcHeader&,uint32_t,const std::string&) {
      if(event=="control_receive_rejected") ++rejections;
    });
    EdcHeader h=control; h.m_srcId=3; h.m_previousHopId=3; h.m_dstId=7;
    h.m_token=1; h.m_attempt=1; h.m_payloadBytes=10; h.m_fragmentCount=3;
    h.m_controlExpiryNs=1000000000;
    const std::vector<uint8_t> body{0,1,2,3,4,5,6,7,8,9};
    auto fragment=[&](uint16_t index) {
      h.m_fragmentIndex=index; const uint32_t offset=uint32_t(index)*4;
      h.m_fragmentBytes=static_cast<uint16_t>(std::min<uint32_t>(4,10-offset));
      return Create<Packet>(body.data()+offset,h.m_fragmentBytes);
    };
    auto middle=fragment(1); app->ProcessControlFrame(middle,h); app->ProcessControlFrame(middle->Copy(),h);
    check(deliveries==0 && app->m_controlAssemblies.at({3,1}).received.size()==1,
          "partial and duplicate control fragments cannot invoke receiver callback");
    auto first=fragment(0); app->ProcessControlFrame(first,h);
    check(deliveries==0,"control receiver waits for all encoded body fragments");
    auto last=fragment(2); app->ProcessControlFrame(last,h);
    check(deliveries==1 && delivered==body,"out-of-order control reassembly delivers exact bytes causally once");
    app->ProcessControlFrame(last->Copy(),h);
    check(deliveries==1,"complete duplicate control does not repeat receiver callback");
    h.m_controlKind=1; app->ProcessControlFrame(last->Copy(),h); h.m_controlKind=2;
    check(rejections==1 && deliveries==1,"same token with inconsistent kind cannot replace prior control body");
    h.m_token=2; h.m_controlVersion=2; app->ProcessControlFrame(last->Copy(),h); h.m_controlVersion=1;
    check(rejections==2,"unknown control wire version is rejected");
    h.m_controlExpiryNs=0; app->ProcessControlFrame(last->Copy(),h); h.m_controlExpiryNs=1000000000;
    check(rejections==3,"expired control is rejected using its received wire deadline");
    app->ProcessControlFrame(Create<Packet>(1),h);
    check(rejections==4,"actual body size must match declared fragment size");
    h.m_payloadBytes=17; app->ProcessControlFrame(last->Copy(),h); h.m_payloadBytes=10;
    check(rejections==5,"oversized received body is rejected without truncation or allocation");
    auto pending=fragment(0); app->ProcessControlFrame(pending,h);
    h.m_token=3; app->ProcessControlFrame(pending->Copy(),h);
    check(rejections==6 && app->m_controlAssemblies.size()==2,
          "finite receiver slots include incomplete and completed identities");
    app->m_controlAssemblies.at({3,1}).header.m_controlExpiryNs=0;
    app->ProcessControlFrame(pending->Copy(),h);
    check(app->m_controlAssemblies.size()==2 && app->m_controlAssemblies.count({3,3})==1,
          "expired receiver-local state releases a bounded slot");
    h.m_payloadBytes=17;
    check(!app->SendControlPayload(h,std::vector<uint8_t>(17)) &&
          app->m_closureTransport->txReservations==0,
          "source rejects oversize body before any MAC handoff reservation");
    h.m_token=11; h.m_controlExpiryNs=1000000000;
    check(app->ReserveControlSource(h) && !app->ReserveControlSource(h),
          "source control token owns at most one local lifetime reservation");
    h.m_token=12; const bool second=app->ReserveControlSource(h); h.m_token=13;
    check(second && !app->ReserveControlSource(h) && app->m_controlSourceExpiry.size()==2,
          "source in-flight control body reservations obey the same explicit finite slot cap");
    app->m_controlSourceExpiry.at(11)=0;
    check(app->ReserveControlSource(h) && app->m_controlSourceExpiry.size()==2,
          "expired local source reservation releases capacity without receiver knowledge");
    h.m_token=14; h.m_controlExpiryNs=0;
    check(!app->ReserveControlSource(h),"source never reserves an already-expired control");
    Simulator::Destroy();
    return checks;
}

uint32_t
EdcForwardingApp::RunClosureSelfTests()
{
    uint32_t checks = 0;
    auto check = [&](bool value, const char* label) {
        NS_ABORT_MSG_IF(!value, "Closure transport self-test failed: " << label);
        ++checks;
    };
    {
        ClosureTransportState state;
        check(state.Reserve(0.0, 2.0) == 0.0, "first reservation starts immediately");
        check(state.Reserve(0.0, 1.0) == 2.0, "shared transmitter serializes contention");
        check(state.nextTxAvailableS == 3.0, "serialized finish time");
        check(state.Reserve(10.0, 0.5) == 10.0, "idle gap does not add a delay");
        check(state.maxAdmissionWaitS == 2.0 && state.txReservations == 3,
              "queue diagnostics count exact reservations");
        bool rejected = false;
        try { state.Reserve(0.0, -1.0); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected && state.txReservations == 3, "invalid duration leaves state unchanged");
    }
    {
        auto state = std::make_shared<ClosureTransportState>();
        auto endpoint = CreateObject<EdcForwardingApp>();
        auto otherApp = CreateObject<EdcForwardingApp>();
        uint32_t charges = 0, draws = 0, deliveries = 0;
        bool accept = false;
        const std::vector<Vector> positions{Vector(0, 0, 0), Vector(10, 0, 0)};
        auto delivered = [&](uint64_t, uint32_t, uint8_t, uint8_t,
                             uint8_t, uint32_t, uint32_t) { ++deliveries; };
        endpoint->Setup(1, positions[1], 100.0, 1, 10.0, positions, true, 1, delivered);
        otherApp->Setup(1, positions[1], 100.0, 1, 10.0, positions, false, 1, delivered);
        otherApp->SetRelayEnabled(false);
        for (auto app : {endpoint, otherApp})
          {
            app->SetClosureTransport(state);
            app->SetEnergyCallback([&](uint8_t, uint32_t, double, uint32_t, bool tx) {
                NS_ABORT_MSG_IF(tx, "RX-only test unexpectedly transmitted");
                ++charges;
            });
            app->SetReceptionCallback([&](uint32_t, double) { ++draws; return accept; });
          }
        auto frame = [](uint8_t attempt) {
            EdcHeader h;
            h.m_id = 42; h.m_srcId = 0; h.m_dstId = 1; h.m_nextHopId = 1;
            h.m_previousHopId = 0; h.m_prio = 1; h.m_attempt = attempt;
            h.m_payloadBytes = 64; h.m_fragmentBytes = 64; h.m_fragmentCount = 1;
            auto packet = Create<Packet>(64); packet->AddHeader(h); return packet;
        };
        auto failed = frame(1);
        check(failed->Copy()->GetUid() == failed->GetUid(), "packet copies preserve physical UID");
        endpoint->ProcessReceivedPacket(failed->Copy());
        endpoint->ProcessReceivedPacket(failed->Copy());
        check(charges == 1 && draws == 1 && deliveries == 0,
              "failed intended RX is charged and sampled once despite duplicate callback");
        check(state->uniqueApplicationRx == 1 && state->duplicateRxCallbacks == 1,
              "failed-reception duplicate accounting");
        accept = true;
        auto success = frame(2);
        otherApp->ProcessReceivedPacket(success->Copy());
        endpoint->ProcessReceivedPacket(success->Copy());
        endpoint->ProcessReceivedPacket(success->Copy());
        check(charges == 2 && draws == 2 && deliveries == 1,
              "shared cache does not let an unrelated app consume the endpoint delivery");
        check(state->uniqueApplicationRx == 2 && state->duplicateRxCallbacks == 3,
              "success callback duplicates are shared across apps");
        endpoint->ProcessReceivedPacket(frame(2));
        check(charges == 3 && draws == 3 && deliveries == 1,
              "new physical duplicate pays RX before logical duplicate suppression");
        endpoint->ProcessReceivedPacket(frame(3));
        check(charges == 4 && draws == 4 && deliveries == 2,
              "new retry attempt is a separate physical reception");
    }
    {
        auto a = CreateObject<EdcForwardingApp>();
        auto b = CreateObject<EdcForwardingApp>();
        auto state = std::make_shared<ClosureTransportState>();
        const std::vector<Vector> positions{Vector(0, 0, 0), Vector(10, 0, 0), Vector(20, 0, 0)};
        for (auto app : {a, b})
          {
            app->Setup(0, positions[0], 11.0, 2, 20.0, positions, false, 0,
                       [](uint64_t, uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t) {});
            app->SetRelayEnabled(false);
            app->SetClosureTransport(state);
            app->SetFragmentation(64, 840.0);
          }
        check(a->BestNextHopId(2, 20.0) == 0,
              "point-to-point out-of-range endpoint cannot recruit an undeclared relay");
        check(a->BestNextHopId(1, 10.0) == 1, "in-range endpoint remains reachable");
        EdcHeader h; h.m_fragmentBytes = 64; h.m_dstId = 2;
        a->QueueClosureFrame(Create<Packet>(64), h);
        b->QueueClosureFrame(Create<Packet>(64), h);
        check(state->txReservations == 2 && std::abs(state->nextTxAvailableS - 2.0) < 1e-12,
              "two apps share one serialized application-wire admission clock");
        // No network or PHY is run in this test. Destroy only these queued test
        // callbacks so they cannot escape into a later self-test/execution.
        Simulator::Destroy();
    }
    return checks;
}

uint32_t
EdcForwardingApp::RunReviewedFrameSelfTests()
{
    uint32_t checks = 0;
    auto check = [&](bool ok, const char* label) {
        NS_ABORT_MSG_IF(!ok, "Reviewed frame self-test failed: " << label);
        ++checks;
    };
    EdcHeader legacy;
    auto oldPacket = Create<Packet>(12); oldPacket->AddHeader(legacy);
    check(oldPacket->GetSize() == 53, "legacy header remains exactly 41 bytes");
    EdcHeader decodedOld; oldPacket->RemoveHeader(decodedOld);
    check(!decodedOld.m_reviewed && oldPacket->GetSize() == 12, "legacy header roundtrip");
    EdcHeader h;
    h.m_reviewed = true; h.m_id = 9; h.m_token = 123456789012ULL;
    h.m_leg = 2; h.m_prio = 2; h.m_srcId = 0; h.m_dstId = 1;
    h.m_nextHopId = 1; h.m_previousHopId = 0;
    h.m_fragmentBytes = 12; h.m_payloadBytes = 12;
    auto packet = Create<Packet>(12); packet->AddHeader(h);
    check(packet->GetSize() == 63, "reviewed header is charged on wire");
    EdcHeader decoded; auto copy = packet->Copy(); copy->RemoveHeader(decoded);
    check(decoded.m_reviewed && !decoded.m_ack && decoded.m_leg == 2 &&
          decoded.m_token == h.m_token && decoded.m_prio == 2,
          "reviewed leg identity roundtrip");
    auto endpoint = CreateObject<EdcForwardingApp>();
    auto state = std::make_shared<ClosureTransportState>();
    const std::vector<Vector> positions{Vector(0, 0, 0), Vector(10, 0, 0)};
    endpoint->Setup(1, positions[1], 100, 1, 10, positions, true, 1,
                   [](uint64_t, uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint32_t) {});
    endpoint->SetClosureTransport(state);
    uint32_t dataCallbacks = 0, ackCallbacks = 0, charges = 0;
    endpoint->SetReviewedCallbacks([&](const EdcHeader& frame) {
        if (frame.m_ack) ++ackCallbacks; else ++dataCallbacks;
    }, [](const EdcHeader&) { return true; });
    endpoint->SetEnergyCallback([&](uint8_t, uint32_t bytes, double, uint32_t, bool tx) {
        check(!tx && bytes == 22, "energy callback includes reviewed extension bytes");
        ++charges;
    });
    endpoint->ProcessReceivedPacket(packet->Copy());
    endpoint->ProcessReceivedPacket(packet->Copy());
    check(dataCallbacks == 1 && charges == 1, "dual callback neither re-delivers nor re-ACKs");
    auto physicalDuplicate = Create<Packet>(12); physicalDuplicate->AddHeader(h);
    endpoint->ProcessReceivedPacket(physicalDuplicate);
    check(dataCallbacks == 2 && charges == 2, "new physical DATA duplicate requests another ACK");
    h.m_ack = true;
    auto ack = Create<Packet>(12); ack->AddHeader(h);
    endpoint->ProcessReceivedPacket(ack);
    check(dataCallbacks == 2 && ackCallbacks == 1, "ACK and DATA have separate reassembly identities");
    endpoint->SetReviewedCallbacks([](const EdcHeader&) {},
                                   [](const EdcHeader&) { return false; });
    endpoint->TransmitFrame(Create<Packet>(12), h);
    check(state->obsoleteTxCancelled == 1 && charges == 3,
          "locally obsolete frame is cancelled before energy or device handoff");
    return checks;
}

// ─────────────────────────────────────────────────────────────────────────────

namespace
{

enum class NodeRole
{
  BSN,
  DRN,
  DDN,
  SINK,
  AUV
};

enum class ProtocolMode
{
  BASELINE,
  CONTRIBUTION,
  PURE_ACOUSTIC,
  MADCS_HP
};

enum class Priority
{
  LOW = 0,
  MEDIUM = 1,
  HIGH = 2
};

static closure_ingress::Priority
ReviewedCablePriority(Priority priority)
{
  // The main wire convention numbers LOW first; the ingress scheduler numbers
  // HIGH first. A numeric cast silently reverses the two service classes.
  switch (priority)
    {
    case Priority::HIGH: return closure_ingress::Priority::High;
    case Priority::MEDIUM: return closure_ingress::Priority::Medium;
    case Priority::LOW: return closure_ingress::Priority::Low;
    }
  NS_FATAL_ERROR("Invalid main priority at the finite cable boundary");
  return closure_ingress::Priority::Low; // Unreachable, satisfies return analysis.
}

struct LogicalNode
{
  uint32_t id;
  NodeRole role;
  Vector position;
  double range;
};

struct DataPacket
{
  uint64_t id;
  uint32_t sourceBsn;
  double createdAt;
  double ddnArrivalAt;
  Priority priority;
  uint32_t sizeBytes;
  bool burst = false;   // part of a correlated alarm burst (stress test)
  // Immutable producer-local completed-window descriptor, 24 B inside the
  // declared record metadata allowance. Chunk index is zero-based; H/M are 0.
  uint64_t windowNumericId = 0, windowPayloadBytes = 0;
  uint32_t windowChunkIndex = 0, windowChunkCount = 0;
};

struct ReviewedVerticalState
{
  DataPacket packet;
  EdcHeader lastReceivedHeader;
  uint32_t ddnIndex = 0;
  bool gatewayCommitted = false, cableConfirmed = false;
};

// One lifetime-bounded acoustic leg. Receiver commitment and sender custody
// confirmation are separate facts; a retained sender copy is not a new record.
struct ReviewedLegState
{
  DataPacket packet;
  uint64_t token = 0;
  uint8_t leg = 0;
  uint32_t sender = 0, receiver = 0, ddnIndex = 0, sinkIndex = 0;
  uint8_t attempts = 0;
  bool arrived = false, confirmed = false, terminal = false, expired = false;
  double arrivalAt = -1.0;
  std::set<uint16_t> localFragmentOutcomes;
  uint32_t localExpectedFragments = 0;
  double localLastTxEndS = 0.0;
  bool localTxRejected = false, localCompletionScheduled = false;
  EventId timeout;
};

struct PendingAcousticPacket
{
  DataPacket packet;
  uint32_t ddnIndex;
  uint8_t attempts = 0;
  EventId timeout;
};

struct PendingDirectPacket
{
  DataPacket packet;
  uint32_t ddnIndex;
  uint32_t sinkNodeId;
  uint8_t attempts = 0;
  EventId timeout;
};

struct PendingMobilePacket
{
  DataPacket packet;
  uint32_t ddnIndex;
  uint8_t attempts = 0;
  EventId timeout;
};

struct PendingAuvSurfacePacket
{
  DataPacket packet;
  uint32_t sinkIndex;
  uint8_t attempts = 0;
  EventId timeout;
};

struct PendingOpticalPacket
{
  DataPacket packet;
  uint32_t ddnIndex;
  double txDistance;
  double successProbability;
  double startedAt = 0.0;
  double durationSeconds = 0.0;
  double chargedSeconds = 0.0;
  bool reviewedWire = false, reviewedReceived = false;
  uint32_t reviewedAttempts = 0;
  uint64_t reviewedNonce = 0;
};

struct ReviewedOpticalReceiverSession
{
  uint64_t nonce=0, creditBytes=0, startupLoad=0, receiveLoad=0;
  uint32_t gateway=0;
  double readyAt=0, until=0;
};
struct ReviewedOpticalSenderSession
{
  uint64_t nonce=0, creditBytes=0, activePacket=0;
  double readyAt=0, until=0;
  EventId timeout;
};

struct Stats
{
  uint64_t generated = 0;
  uint64_t gatewayOriginated = 0;
  uint64_t reachedDdn = 0;
  uint64_t collectedByAuv = 0;
  uint64_t deliveredToSink = 0;
  uint64_t directCriticalToSink = 0;
  uint64_t bufferDropped = 0;
  uint64_t bufferDroppedHigh = 0;
  uint64_t bufferDroppedMedium = 0;
  uint64_t bufferDroppedLow = 0;
  uint64_t directMediumToSink = 0;
  uint64_t directLowToSink = 0;
  uint64_t opticalTransfers = 0;
  uint64_t opticalAttemptedPackets = 0;
  uint64_t opticalLostPackets = 0;
  uint64_t opticalContactSamples = 0;
  uint64_t acousticTransfers = 0;
  uint64_t acousticTxBytes = 0;
  double acousticTxAirtimeS = 0.0;
  uint64_t acousticSourceAttempts = 0;
  uint64_t acousticRetransmissions = 0;
  uint64_t acousticRetryExhausted = 0;
  uint64_t acousticAcks = 0;
  uint64_t directSourceAttempts = 0;
  uint64_t directRetransmissions = 0;
  uint64_t directCyclesExhausted = 0;
  uint64_t directAcks = 0;
  uint64_t mobileSourceAttempts = 0;
  uint64_t mobileRetransmissions = 0;
  uint64_t mobileContactFailures = 0;
  uint64_t mobileAcks = 0;
  uint64_t auvSurfaceSourceAttempts = 0;
  uint64_t auvSurfaceRetransmissions = 0;
  uint64_t auvSurfaceContactFailures = 0;
  uint64_t auvSurfaceAcks = 0;
  uint64_t deadlineExpired = 0;
  uint64_t deadlineExpiredHigh = 0;
  uint64_t deadlineExpiredMedium = 0;
  uint64_t deadlineExpiredLow = 0;
  uint64_t highGenerated = 0;
  uint64_t mediumGenerated = 0;
  uint64_t lowGenerated = 0;
  // Correlated alarm-burst stress test (HIGH class).
  uint64_t burstGenerated = 0;
  uint64_t burstDelivered = 0;
  double   burstDelaySum = 0.0;
  uint64_t burstDeadlineMiss = 0;
  uint64_t highDelivered = 0;
  uint64_t mediumDelivered = 0;
  uint64_t lowDelivered = 0;
  uint64_t highDeliveredBytes = 0;
  uint64_t mediumDeliveredBytes = 0;
  uint64_t lowDeliveredBytes = 0;
  uint64_t highDeadlineMiss = 0;
  uint64_t mediumDeadlineMiss = 0;
  uint64_t lowDeadlineMiss = 0;
  double highDelaySum = 0.0;
  double mediumDelaySum = 0.0;
  double lowDelaySum = 0.0;
  double energyConsumedJ = 0.0;
  double highEnergyJ = 0.0;
  double mediumEnergyJ = 0.0;
  double lowEnergyJ = 0.0;
  double bsnEnergyJ = 0.0;
  double drnEnergyJ = 0.0;
  double ddnEnergyJ = 0.0;
  double auvEnergyJ = 0.0;
  double auvOperationalEnergyJ = 0.0;
  double sinkEnergyJ = 0.0;
  double delaySum = 0.0;
  // Surface delivery at the common endpoint (AUV-to-sink or direct EOM-to-sink).
  uint64_t surfaceDelivered = 0;
  double   surfaceDelaySum  = 0.0;
  double   opticalSuccessProbSum = 0.0;
  double   opticalSnrDbSum = 0.0;
  uint64_t maxDdnBuffered = 0;
  uint64_t maxAuvBuffered = 0;
  double ddnQueuePacketSeconds = 0.0;
  double auvQueuePacketSeconds = 0.0;
  double queueObservationSeconds = 0.0;
  // Acoustic-chain-only delay: BSN→DDN hop (comparable to Ahmed 2022's reported 1-9 s).
  // Separating this from full E2E allows fair cross-paper comparison.
  uint64_t acousticDelivered = 0;
  double   acousticDelaySum  = 0.0;
};

// Reviewer diagnostics are observational: they consume no RNG values and do
// not schedule simulation events. Occupancy/contact integrals are sampled at
// the existing AUV scheduler ticks, while transfer/admission counts are exact.
struct GatewayDiagnostics
{
  bool contactActive = false;
  uint64_t contactCount = 0;
  double contactSeconds = 0.0;
  double contactStartTime = -1.0;
  uint64_t contactStartQueued = 0;
  uint64_t contactAttempts = 0;
  uint64_t contactCollected = 0;
  uint64_t opticalAttempts = 0;
  uint64_t opticalCollected = 0;
  uint64_t opticalLostPer = 0;
  uint64_t opticalLostContact = 0;
  uint64_t opticalBytes = 0;
  uint64_t mediumAdmissions = 0;
  uint64_t mediumTokensCharged = 0;
  uint64_t mediumTokensChargedExpired = 0;
  uint64_t mediumPurgedBeforeAdmission = 0;
  uint64_t mediumExpiredBeforeAttempt = 0;
  double mediumEligibleSeconds = 0.0;
  double mediumTokenBlockedSeconds = 0.0;
  double mediumHighBlockedSeconds = 0.0;
  double mediumContactDeferredSeconds = 0.0;
  double queueRecordSeconds = 0.0;
  uint64_t maxGatewayQueued = 0;
};

class AuvEdcExperiment
{
public:
  void Configure(int argc, char* argv[]);
  void Run();
  bool RunHardwareSelfTests();
  bool RunClosureTransportSelfTests();
  bool RunReviewedSelfTests();
  bool RunReviewedIngressSelfTests();
  bool RunReviewedOpticalSelfTests();

private:
  void ScaleRoles();
  void BuildNs3Topology();
  void BuildLogicalTopology();
  void ScheduleTraffic();
  void GenerateReplayPacket(edc_replay::Event event);
  void GenerateReviewedReplayBatch(std::vector<edc_replay::Event> events);
  void GenerateReviewedReplayPacket(edc_replay::Event event, uint64_t windowNumericId,
      uint64_t windowPayloadBytes, uint32_t windowChunkIndex, uint32_t windowChunkCount);
  closure_ingress::Config ReviewedIngressConfiguration() const;
  void ReviewedIngressInitialize();
  void ReviewedIngressSchedule(uint32_t ddnIndex);
  void ReviewedIngressTick(uint32_t ddnIndex, double eventTime);
  void ReviewedIngressRefresh(double now);
  void ReviewedIngressRelease(uint32_t ddnIndex, uint64_t id,
                              closure_ingress::Removal reason, bool freedSpace = true);
  void ReviewedIngressTrace(uint32_t ddnIndex, const closure_ingress::Event& event) const;
  void ReviewedIngressEnergy(uint32_t ddnIndex, const closure_ingress::EnergyEvent& event);
  void ReviewedIngressActivity(uint32_t ddnIndex, closure_ingress::EnergyEvent event);
  void ReviewedIngressBeginLoad(uint32_t ddnIndex, closure_ingress::EnergyEvent event);
  void ReviewedIngressEndLoad(uint32_t ddnIndex, std::string owner, uint64_t loadId);
  bool ReviewedIngressPowered(uint32_t ddnIndex);
  double ReviewedIngressPoweredUntil(uint32_t ddnIndex, const std::string& owner, double end);
  void ReviewedIngressSummary() const;
  void ReviewedVerticalReceive(const EdcHeader& header);
  bool ReviewedVerticalAdmission(uint32_t ddnIndex, const closure_ingress::Record& record, double time);
  void ReviewedVerticalAcknowledge(uint64_t recordId);
  uint64_t ReviewedIngressSourceHeld() const;
  uint64_t ReviewedIngressArchiveOverflow() const;
  uint64_t ReviewedIngressSourceUnavailable() const;
  void GeneratePacket();
  void EnqueueAtDdn(uint32_t ddnIndex, DataPacket packet);
  // Real-forwarding callback: called by EdcForwardingApp when packet reaches DDN.
  void RealEnqueueAtDdn(uint64_t pktId, uint32_t srcBsnNs3Id,
                        uint8_t prio, uint8_t hops, uint8_t attempt,
                        uint32_t payloadBytes,
                        uint32_t ddnNs3Id);
  void StartAcousticAttempt(uint64_t packetId);
  void AcousticAttemptTimeout(uint64_t packetId, uint8_t attempt);
  void QueueDirectAcoustic(uint32_t ddnIndex, const DataPacket& packet);
  void StartDirectAttempt(uint64_t packetId);
  void StartNextDirectPacket(uint32_t ddnIndex);
  void FinishDirectPacket(uint32_t ddnIndex, uint64_t packetId);
  void ExpireDirectPacket(uint64_t packetId, const std::string& reason);
  void DirectAttemptTimeout(uint64_t packetId, uint8_t attempt);
  void RealDeliverToSink(uint64_t pktId, uint32_t srcBsnNs3Id,
                         uint8_t prio, uint8_t hops, uint8_t attempt,
                         uint32_t payloadBytes, uint32_t unused);
  void QueueMobileAcoustic(uint32_t ddnIndex, const DataPacket& packet);
  void StartNextMobilePacket(uint32_t ddnIndex);
  void StartMobileAcousticAttempt(uint64_t packetId);
  void MobileAcousticAttemptTimeout(uint64_t packetId, uint8_t attempt);
  void FinishMobilePacket(uint32_t ddnIndex, uint64_t packetId, bool requeue);
  void RealCollectByAuv(uint64_t pktId, uint32_t srcDdnNs3Id,
                        uint8_t prio, uint8_t hops, uint8_t attempt,
                        uint32_t payloadBytes, uint32_t unused);
  void QueueAuvSurface(const DataPacket& packet, uint32_t sinkIndex);
  void StartAuvSurfaceAttempt(uint64_t packetId);
  void AuvSurfaceAttemptTimeout(uint64_t packetId, uint8_t attempt);
  void RealAuvDeliverToSink(uint64_t pktId, uint32_t srcAuvNs3Id,
                            uint8_t prio, uint8_t hops, uint8_t attempt,
                            uint32_t payloadBytes, uint32_t unused);
  void StartOpticalTransfer(uint32_t ddnIndex,
                            const DataPacket& packet,
                            double distance,
                            double successProbability);
  void TryStartOpticalTransfer(uint32_t ddnIndex);
  void ReviewedOpticalInitialize();
  void ReviewedOpticalWake(uint32_t gateway);
  void ReviewedOpticalReceiveWake(uint32_t sender,uint32_t receiver,uint64_t nonce,const std::vector<uint8_t>& body);
  void ReviewedOpticalReceiveAck(uint32_t sender,uint32_t receiver,uint64_t nonce,const std::vector<uint8_t>& body);
  void ReviewedOpticalReady(uint64_t nonce);
  void ReviewedOpticalClose(uint64_t nonce);
  void ReviewedOpticalSend(uint32_t gateway,uint64_t nonce);
  void ReviewedOpticalArrival(uint32_t gateway,uint64_t nonce,uint64_t packetId,uint32_t attempt,std::vector<uint8_t> descriptor);
  void ReviewedOpticalTimeout(uint32_t gateway,uint64_t nonce,uint64_t packetId,uint32_t attempt);
  void ReviewedOpticalAbandon(uint64_t packetId,const std::string& reason);
  uint64_t ReviewedOpticalReservedMemoryBytes() const;
  bool ReviewedOpticalReceiverBusy() const;
  uint64_t ReviewedOpticalUnreceivedPending() const;
  double ReviewedOpticalServicePredictionSeconds(const std::vector<uint32_t>& payloads) const;
  void ReviewedOpticalSummary() const;
  void CompleteOpticalTransfer(uint64_t packetId);
  void ChargeOpticalAttemptEnergy(PendingOpticalPacket& pending,
                                  double observedAt,
                                  bool completed = false);
  void SettlePendingOpticalEnergy(double observedAt);
  NodeRole NodeRoleFromNs3Id(uint32_t nodeId) const;
  void BuildForwardingApps();
  uint32_t ReviewedUpstreamReceiver(uint32_t ddnIndex) const;
  void StartReviewedLeg(const DataPacket& packet, uint8_t leg, uint32_t sender,
                        uint32_t receiver, uint32_t ddnIndex, uint32_t sinkIndex);
  void SendReviewedAttempt(uint64_t token);
  void ReviewedTimeout(uint64_t token, uint8_t attempt);
  void ReviewedLocalTxOutcome(const ReviewedLocalTxTag& tag, uint32_t nodeId,
                              bool accepted, double completionAt);
  void ReviewedLocalAttemptComplete(uint64_t token, uint8_t attempt);
  using ReviewedControlHandler = std::function<void(uint32_t,uint32_t,uint64_t,const std::vector<uint8_t>&)>;
  uint64_t SendReviewedControl(uint32_t sender, uint32_t receiver, uint8_t kind,
                               uint64_t nonce, const std::vector<uint8_t>& payload);
  void SetReviewedControlHandler(uint8_t kind, ReviewedControlHandler handler);
  void ReceiveReviewedControl(const EdcHeader& header, const std::vector<uint8_t>& payload);
  void TraceReviewedControl(const std::string& event, const EdcHeader& header,
                             uint32_t node, const std::string& reason);
  void ReviewedArrival(const EdcHeader& header);
  bool TryReviewedUpstreamAdmission(uint32_t ddnIndex, DataPacket packet);
  void EndReviewedLeg(uint64_t token, bool confirmed, bool expired);
  bool ReviewedFrameActive(const EdcHeader& header, uint32_t transmittingNode) const;
  uint64_t ReviewedCommittedPending(uint8_t leg = 0) const;
  void AuvTick();
  void ReviewedMissionInitialize();
  void ReviewedMissionAdvance(double now);
  void ReviewedMissionRecoveryComplete();
  void ReviewedMissionDecision();
  void ReviewedMissionSummary() const;
  Vector ReviewedMissionVelocity(double time) const;
  bool ReviewedMissionReserve(const DataPacket& packet);
  void ReviewedMissionCommit(uint64_t id, uint32_t station = std::numeric_limits<uint32_t>::max());
  void ReviewedMissionRelease(uint64_t id);
  bool ReviewedMissionCommunicationsAvailable() const;
  bool ReviewedMissionAdaptivePlan(double now, const closure_mission::Pose& current, double guardedW);
  closure_adaptive::ServicePrediction ReviewedMissionServicePrediction() const;
  closure_adaptive::ChunkDescriptor ReviewedMissionChunk(const DataPacket& packet, uint32_t station) const;
  void ReviewedTelemetryInitialize();
  void ReviewedTelemetrySend(uint32_t station);
  void ReviewedTelemetryReceive(uint32_t sender, uint32_t receiver, uint64_t sequence,
                                const std::vector<uint8_t>& bytes);
  void ReviewedTelemetryTrace(const std::string& event, uint32_t station, uint64_t sequence,
                              double observedAt, uint64_t bytes, uint64_t windows,
                              const std::string& reason) const;
  Vector GetAuvPosition(double time) const;
  double AuvInstantaneousSpeed(double time) const;
  double AuvPeakSpeed() const;
  double AuvOperationalEnergy(double start, double duration) const;
  uint32_t NearestDdn(double x) const;
  uint32_t AssociatedDrn(uint32_t bsnIndex) const;
  uint32_t DdnForDrn(uint32_t drnIndex) const;
  uint32_t PacketSizeBytes(Priority priority) const;
  double DeadlineSeconds(Priority priority) const;
  bool DeadlineExpired(const DataPacket& packet) const;
  void RecordDeadlineExpiry(const DataPacket& packet);
  void AccountGeneratedPriority(Priority priority);
  void AccountDeliveredPriority(Priority priority, double delay, bool burst, uint32_t payloadBytes);
  void InjectAlarmBurst();
  void ChargeNodeRoleEnergy(NodeRole role, double joules);
  void ChargeGlobalEnergy(NodeRole role, double joules);
  void ChargePacketEnergy(Priority priority, double joules, NodeRole role);
  void ChargePacketEnergySplit(Priority priority,
                               double joules,
                               NodeRole firstRole,
                               double firstShare,
                               NodeRole secondRole);
  Priority AssignPriority(uint64_t packetId) const;
  double OpticalExtinctionCoefficient() const;
  double OpticalPacketSuccessProbability(uint32_t ddnIndex,
                                         double distance,
                                         uint32_t packetBytes,
                                         double& snrDb,
                                         double& ber,
                                         double& pointingGain);
  double AcousticPacketSuccessProbability(double distance,
                                          uint32_t packetBytes) const;
  double AcousticTxEnergyJ(uint32_t packetBytes, double distance) const;
  double AcousticRxEnergyJ(uint32_t packetBytes) const;
  double AcousticRecordDurationS(uint32_t payloadBytes) const;
  static double QFunction(double x);
  bool EnqueueContribPacket(uint32_t ddnIndex, const DataPacket& packet,
                            bool atFront = false);
  uint64_t DdnQueueBytes(uint32_t ddnIndex) const;
  uint64_t DdnQueueRecords(uint32_t ddnIndex) const;
  std::pair<uint64_t, uint64_t> DdnResidentStorage(uint32_t ddnIndex,
                                                 uint64_t excludeId = 0) const;
  uint64_t RecordStorageBytes(const DataPacket& packet) const;
  void ObserveStorage(uint32_t ddnIndex);
  void RecordBufferDrop(uint32_t ddnIndex, const DataPacket& packet,
                        const std::string& reason);
  void WriteStorageHeader() const;
  void WriteStorage(const std::string& event, uint32_t ddnIndex,
                    const DataPacket* packet = nullptr,
                    const std::string& reason = "") const;
  bool UsesPriorityQueues() const
  {
    return m_protocol == ProtocolMode::CONTRIBUTION ||
           (m_protocol == ProtocolMode::PURE_ACOUSTIC && m_matchHybridQueues);
  }
  static double SinusoidalPathStretch(double contactDepth, double spacing);
  static std::string NormalizeName(std::string value);
  bool PopNextDdnPacket(uint32_t ddnIndex, DataPacket& packet);
  void RequeueFrontDdnPacket(uint32_t ddnIndex, const DataPacket& packet);
  bool InRange(const Vector& a, const Vector& b, double range) const;
  void PrintSummary() const;
  void SampleMetrics();
  void WriteMetricsHeader() const;
  void TracePacket(const std::string& event,
                   const DataPacket& packet,
                   int32_t ddnIndex = -1,
                   int32_t sinkIndex = -1,
                   uint32_t attempt = 0,
                   double distance = -1.0,
                   const std::string& outcome = "") const;
  void WritePacketTraceHeader() const;
  double MediumFallbackRefillRate() const;
  void ObserveGatewayDiagnostics(double now, const Vector& auvPos);
  void WriteGatewayDiagnostics(const std::string& event, int32_t ddnIndex = -1) const;
  void WriteGatewayDiagnosticsHeader() const;
  void ObservePhyEnergy(bool terminal = false);
  void WritePhyEnergyHeader() const;
  void WritePhyEnergy(const std::string& event, int32_t nodeIndex = -1) const;
  void BindReviewedPhyEnergy();
  void FinalizeReviewedPhyEnergy();
  void TraceAcousticHop(uint64_t packetId,
                        uint32_t sourceId,
                        uint8_t priority,
                        uint32_t payloadBytes,
                        uint8_t attempt,
                        uint32_t fromNode,
                        uint32_t toNode,
                        uint32_t destinationNode,
                        double distance) const;
  uint64_t BufferedAtDdn() const;
  bool HasHighBacklog() const;
  bool HasMediumBacklog() const;

  // Underwater acoustic channel model.
  // All closure transport parameters are opt-in. Negative power/range defaults
  // deliberately require an explicit numerical contract, not a vendor guess.
  bool m_closureMode = false;
  bool m_reviewedMission = false;
  std::string m_reviewedMissionPolicy = "static", m_reviewedMissionCsv;
  double m_reviewedMissionOperationW = -1, m_reviewedMissionRecoveryS = -1;
  double m_reviewedMissionRecoveryJ = -1, m_reviewedMissionServiceS = -1;
  double m_reviewedMissionFinalServiceS = -1;
  uint64_t m_reviewedMissionCarryBytes = 0, m_reviewedMissionCarryPeak = 0;
  uint64_t m_reviewedMissionCarryRefusals = 0;
  std::unique_ptr<closure_mission::Mission> m_reviewedMissionModel;
  closure_mission::RouteConstraints m_reviewedMissionConstraints;
  closure_mission::Pose m_reviewedMissionHome;
  struct ReviewedMissionStep { closure_mission::ContinuousLeg leg; std::string kind; int32_t station = -1; };
  std::deque<ReviewedMissionStep> m_reviewedMissionSteps;
  std::optional<closure_mission::ContinuousLeg> m_reviewedMissionReservedReturn;
  std::vector<std::pair<uint32_t,double>> m_reviewedMissionVisitOrder;
  std::size_t m_reviewedMissionVisitCursor = 0;
  bool m_reviewedMissionReturning = false, m_reviewedMissionRecoveryScheduled = false;
  std::string m_reviewedMissionLastKind = "initial_submerged_rendezvous";
  uint64_t m_reviewedMissionLegsStarted = 0, m_reviewedMissionGatewayVisits = 0;
  uint64_t m_reviewedMissionSurfaceServices = 0;
  double m_reviewedTelemetryPeriodS = -1, m_reviewedTelemetryMaxAgeS = -1;
  double m_reviewedMissionMaxRevisitS = -1;
  std::string m_reviewedTelemetryCsv;
  closure_adaptive::ReceivedReports m_reviewedTelemetryReports;
  closure_adaptive::Coverage m_reviewedMissionCoverage;
  std::vector<uint64_t> m_reviewedTelemetrySequences;
  std::map<uint64_t,closure_adaptive::ChunkDescriptor> m_reviewedMissionOnboardChunks;
  std::map<std::string,closure_mission::ContinuousLeg> m_reviewedMissionGeometryCache;
  std::optional<closure_mission::Pose> m_reviewedMissionCanonicalEnd;
  int32_t m_reviewedMissionLastStation = -1;
  uint64_t m_reviewedTelemetrySent = 0, m_reviewedTelemetryReceived = 0;
  uint64_t m_reviewedTelemetryObsolete = 0, m_reviewedTelemetryInvalid = 0;
  uint64_t m_reviewedMissionAdaptiveDecisions = 0, m_reviewedMissionAdaptiveNoFeasible = 0;
  uint64_t m_reviewedMissionCoverageViolations = 0;
  uint64_t m_reviewedMissionReservedMemoryBytes = 0;
  bool m_reviewedPhyEnergy = false;
  std::vector<Ptr<ReviewedAquaSimPhy>> m_reviewedPhys;
  std::vector<std::shared_ptr<closure_energy::PowerLedger>> m_reviewedLedgers;
  std::array<double, 5> m_reviewedBatteryJ{{-1,-1,-1,-1,-1}};
  std::array<double, 5> m_reviewedBaseW{{-1,-1,-1,-1,0}};
  double m_reviewedIdlePowerW = -1, m_reviewedSleepPowerW = -1;
  std::string m_reviewedEnergyJson;
  bool m_reviewedTransport = false;
  // Explicit numerical engineering contract; no inferred vendor RAM, cable or power.
  bool m_reviewedIngress = false, m_reviewedIngressSelfTest = false;
  bool m_reviewedVerticalIngress = false;
  std::map<uint64_t, ReviewedVerticalState> m_reviewedVertical;
  uint64_t m_reviewedVerticalCommitToken = 0, m_reviewedVerticalConfirmToken = 0;
  uint64_t m_reviewedIngressArchiveBytes = 0;
  uint64_t m_reviewedIngressSourceRamBytes = 0, m_reviewedIngressGatewayRamBytes = 0;
  double m_reviewedIngressBitRate = -1.0, m_reviewedIngressWireBitsPerByte = -1.0;
  uint32_t m_reviewedIngressDataHeaderBytes = std::numeric_limits<uint32_t>::max();
  uint32_t m_reviewedIngressAckBytes = 0, m_reviewedIngressMaxAttempts = 0;
  double m_reviewedIngressPropagationS = -1.0, m_reviewedIngressDurableWriteS = -1.0;
  double m_reviewedIngressTurnaroundS = -1.0, m_reviewedIngressRetryS = -1.0;
  double m_reviewedIngressAckTimeoutS = -1.0, m_reviewedIngressRetentionS = -1.0;
  double m_reviewedIngressSourceTxW = -1.0, m_reviewedIngressSourceRxW = -1.0;
  double m_reviewedIngressGatewayTxW = -1.0, m_reviewedIngressGatewayRxW = -1.0;
  double m_reviewedIngressSourceEnergyJ = 0.0, m_reviewedIngressGatewayEnergyJ = 0.0;
  double m_reviewedIngressSourceBudgetJ = -1.0, m_reviewedIngressSourceBaseW = -1.0;
  double m_reviewedIngressSourceBaseEnergyJ = 0.0;
  std::vector<std::shared_ptr<closure_energy::PowerLedger>> m_reviewedIngressSourceLedgers;
  std::vector<double> m_reviewedIngressAckTxEnd, m_reviewedIngressAckRxEnd;
  std::string m_reviewedIngressCsv;
  std::vector<std::unique_ptr<closure_ingress::Link>> m_reviewedIngressLinks;
  std::vector<closure_ingress::State> m_reviewedIngressStates;
  std::vector<EventId> m_reviewedIngressEvents;
  std::vector<uint64_t> m_reviewedIngressSequences;
  std::vector<bool> m_reviewedIngressBusy, m_reviewedIngressCreditPending;
  std::vector<double> m_reviewedIngressClock;
  uint64_t m_reviewedIngressAdmissionId = 0;
  bool m_reviewedSelfTest = false;
  uint32_t m_reviewedAckPayloadBytes = 0;
  uint32_t m_reviewedMaxAttempts = 0;
  uint64_t m_reviewedNextToken = 1;
  std::map<uint64_t, ReviewedLegState> m_reviewedLegs;
  std::map<std::pair<uint64_t, uint8_t>, uint64_t> m_reviewedLegTokens;
  std::vector<Ptr<EdcForwardingApp>> m_reviewedApps;
  uint64_t m_reviewedDataArrivals = 0, m_reviewedDuplicateData = 0;
  uint64_t m_reviewedAckSent = 0, m_reviewedConfirmed = 0;
  uint64_t m_reviewedUnconfirmedReceived = 0, m_reviewedRetryDropped = 0;
  uint64_t m_reviewedUpstreamExpired = 0;
  uint64_t m_reviewedUpstreamAdmissionId = 0;
  uint64_t m_reviewedUpstreamAdmissionRejected = 0;
  uint64_t m_reviewedUpstreamCompleteReceipts = 0;
  std::set<uint64_t> m_reviewedUpstreamReceivedIds;
  uint64_t m_reviewedStaleFrames = 0;
  uint64_t m_reviewedLateRecovered = 0;
  uint64_t m_reviewedPhysicalAttemptsSettled = 0, m_reviewedLocalRejectedAttempts = 0;
  uint32_t m_reviewedControlMaxPayloadBytes=0, m_reviewedControlMaxInflight=16;
  double m_reviewedControlLifetimeS=0.0;
  uint64_t m_reviewedControlNextToken=1;
  uint64_t m_reviewedControlEnqueued=0, m_reviewedControlReceived=0, m_reviewedControlRejected=0;
  uint64_t m_reviewedControlFragmentHandoffs=0, m_reviewedControlFragmentRejections=0;
  uint64_t m_reviewedControlPayloadBytesEnqueued=0, m_reviewedControlAppWireBytes=0;
  std::map<uint8_t,ReviewedControlHandler> m_reviewedControlHandlers;
  bool m_closureFullValidation = false;
  bool m_closureTransportSelfTest = false;
  double m_closureAcousticTxPowerW = -1.0;
  double m_closureAcousticRxPowerW = -1.0;
  double m_closureBsnRangeM = -1.0;
  double m_closureDrnRangeM = -1.0;
  double m_closureGatewayRangeM = -1.0;
  double m_closureAuvRangeM = -1.0;
  double m_closureSinkRangeM = -1.0;
  std::string m_closureRetrySemantics = "unset";
  std::vector<std::shared_ptr<ClosureTransportState>> m_closureNodeTransport;
  // Absorption: Thorp (1967), "Analytic description of the low-frequency attenuation
  //   coefficient," JASA 42(1), α(f) in dB/km, f in kHz.
  // Spreading: k=1.5 (practical), Urick (1983), "Principles of Underwater Sound," 3rd ed.
  // Power margin: WHOI Micromodem 2 source level ~185 dB re µPa @ 1 m (Woods Hole
  //   Oceanographic Institution, 2012); ambient + thermal noise ~133 dB re µPa in 25 kHz BW
  //   at 500 m depth (Wenz curves, moderate shipping) → PM = 185 − 133 = 52 dB.
  double m_acousticFreqKhz  = 10.0;  // center frequency (kHz) — typical UWSN modem band
  std::string m_acousticMac = "aloha";
  double m_txPowerMarginDb  = 52.0;  // transmit power margin at 1 m ref distance (dB)
  double m_macLoadPenaltyDb =  0.0;  // ignored legacy CLI; contention comes from AquaSim MAC events
  uint32_t m_acousticMaxRetransmissions = 3;
  double m_acousticAckTimeout = 12.0;
  double m_acousticRetryBackoff = 1.0;
  // Independent streams keep arrivals/classes identical when a protocol knob
  // changes how many channel/retry draws are consumed (paired-seed design).
  Ptr<UniformRandomVariable> m_priorityRng;
  Ptr<UniformRandomVariable> m_acousticReceptionRng;
  Ptr<UniformRandomVariable> m_opticalRng;
  Ptr<UniformRandomVariable> m_retryRng;
  Ptr<ExponentialRandomVariable> m_arrivalRng;  // Poisson arrivals: exp inter-arrival times

  double m_simStop = 10500.0;  // legacy CLI default; paper runs override it
  double m_trafficStop = -1.0; // negative => generate until simStop; otherwise leave a drain interval
  uint32_t m_totalNodes = 126;
  uint32_t m_sinks = 5;
  std::string m_sinkPlacement = "midgap";
  // Default -1 selects the fixed 12,600-m pipeline in Configure().
  double m_pipelineLength = -1.0;
  // Both mobile architectures monitor a pipeline at 1000 m depth.
  // BASELINE: DDNs sit at pipeline depth (1000 m) — AUV dives to 1000 m.
  // CONTRIBUTION: DDNs lifted 500 m by EOM cable → DDN at 500 m depth.
  //   → AUV dives only to 500 m (shorter cycle, more frequent contacts).
  //   → UWOC eligibility uses the configurable optical gate (30 m nominally).
  double m_depth = 1000.0;
  double m_ddnEomOffset = 500.0;  // EOM cable length: DDN lifted from 1000 m to 500 m
  double m_trafficLoad = 1.0;
  // Application payload sizes. The on-wire acoustic packet additionally carries
  // the 41-byte EdcHeader above. --packetSize remains a legacy override: a
  // positive value forces the same payload size for every class.
  uint32_t m_packetSize = 0;
  uint32_t m_highPacketSize = 64;
  uint32_t m_mediumPacketSize = 256;
  uint32_t m_lowPacketSize = 1024;
  double m_channelBandwidth = 25000.0; // occupied acoustic bandwidth (Hz)
  double m_acousticBitRate = 31200.0;  // AquaSim modulation symbol/bit rate (bit/s)
  uint32_t m_acousticFramePayloadBytes = 4096;
  double m_acousticSpeed = 1500.0;
  double m_bsnRange = 100.0;
  double m_drnRange = 250.0;
  double m_ddnRange = 400.0;
  double m_ddnCollectionRange = 400.0;
  double m_opticalRange = 30.0;  // geometric contact gate; PHY success comes from the UWOC model below
  std::string m_opticalWaterType = "clear-ocean";
  // WP1 UWOC model. Beer-Lambert attenuation c=a+b uses m^-1 coefficients
  // representative of blue-green operation: very-clear ocean 0.056, clear ocean
  // 0.151, coastal 0.398, turbid harbor 2.195. The link budget is calibrated so
  // clear-ocean water reaches BER=1e-5 at 68.5 m, then the same budget is reused
  // for all water types and distances.
  double m_opticalExtCoeffOverride = -1.0;
  double m_opticalReferenceExtCoeff = 0.151;
  double m_opticalReferenceRange = 68.5;
  double m_opticalReferenceSnr = 9.10;  // OOK: BER=Q(sqrt(2*SNR)) ~= 1e-5
  double m_opticalDataRateBps = 1000000.0;
  double m_opticalDivergenceDeg = 30.0;
  double m_opticalPointingSigma = 0.75; // Rayleigh radial jitter scale, metres
  double m_opticalPointingCoherenceSeconds = 0.0;
  bool m_disableOpticalLoss = false;
  double m_sinkRange = 500.0;
  // Operational speed envelope: 2--5 kn (3.704--9.260 km/h). The nominal
  // 4 kn value matches common survey operation rather than the superseded
  // 20--45 km/h optimization domain.
  double m_auvSpeedKmh = 7.408;
  double m_auvMaxSpeedKmh = 9.260;
  // Optional extension: preserve legacy trajectory and RNG/event ordering,
  // but enforce its geometric peak speed and integrate P(actual speed).
  bool m_auvInstantaneousPower = false;
  double m_motionSpacing = 1.0;
  double m_motionContactDepth = 0.0;
  double m_motionHorizontalSpeed = 0.0;
  // Least-squares fit to the configured 3/4/5 kn input-power reference points
  // (168/413/781 W), with v expressed in m/s:
  // P_AUV(v) = P0 + k v^3 = 5.23 + 45.78 v^3 [W].
  double m_auvFixedPowerW = 5.23;
  double m_auvPropulsionCoeff = 45.78;
  double m_auvDepth = 500.0;   // legacy CLI value; trajectory depth is protocol-derived
  double m_auvAmplitude = 0.0; // paper path is 2-D (x-z); set >0 for lateral oscillation
  double m_tick = 1.0;
  double m_sampleInterval = 10.0;
  double m_metricSampleStart = 0.0;
  double m_helloTimeout = 5.0;
  uint32_t m_ddnContactPacketsPerTick = 0;
  uint32_t m_sinkContactPacketsPerTick = 0;
  double m_processingDelay = 0.02;
  // Comparative normalized packet-energy coefficients.
  double m_acousticEnergyPerByteMeter = 0.000002;   // J/(byte*m)
  double m_acousticReceiveEnergyPerByte = 0.00005;
  double m_opticalEnergyPerByteMeter = 0.00000002;  // J/(byte*m)
  std::string m_energyModel = "normalized";
  double m_acousticRxPowerW = 0.8;
  bool m_reviewedOpticalControl = false;
  bool m_reviewedOpticalSelfTest = false;
  bool m_reviewedCompactTrace = false;
  double m_reviewedOpticalStartupS=0, m_reviewedOpticalListenS=0, m_reviewedOpticalPollS=0;
  double m_reviewedOpticalGatewayStartupW=0, m_reviewedOpticalVehicleStartupW=0;
  double m_reviewedOpticalEfficiency=0, m_reviewedOpticalFrameSuccess=-1;
  double m_reviewedOpticalAckWaitS=0, m_reviewedOpticalTurnaroundS=0, m_reviewedOpticalControlPredictionS=0;
  uint32_t m_reviewedOpticalMaxAttempts=0, m_reviewedOpticalReceiptEntries=0;
  uint64_t m_reviewedOpticalCreditBytes=0, m_reviewedOpticalNextNonce=1;
  std::optional<ReviewedOpticalReceiverSession> m_reviewedOpticalReceiver;
  std::map<uint32_t,ReviewedOpticalSenderSession> m_reviewedOpticalSenders;
  std::map<uint32_t,double> m_reviewedOpticalNextPoll;
  std::map<uint64_t,std::pair<uint32_t,double>> m_reviewedOpticalReceipts;
  uint64_t m_reviewedOpticalWakeSent=0,m_reviewedOpticalWakeReceived=0,m_reviewedOpticalAckReceived=0;
  uint64_t m_reviewedOpticalDuplicates=0,m_reviewedOpticalCreditRejected=0,m_reviewedOpticalCopyAbandoned=0;
  uint64_t m_reviewedOpticalDataWireBytes=0;
  double m_opticalTxPowerW = 15.0;
  double m_opticalRxPowerW = 10.0;
  double m_controlMessageEnergyJ = 0.02;
  double m_directAlarmEnergyJ = 0.03;
  // Fallback timeout: HIGH is hard-deadline emergency traffic. MEDIUM is
  // aging-aware soft real-time traffic and can use acoustic fallback only when
  // the admitted fallback load remains below rhoMax. LOW stays AUV/UWOC
  // best-effort in the Paper-A policy.
  double m_highFallbackTimeout = 0.0;
  double m_mediumFallbackTimeout = 600.0;
  double m_mediumFallbackRhoMax = 0.0;
  double m_mediumFallbackBurstSeconds = 10.0;
  double m_mediumFallbackTokens = 0.0;
  // Negative retains the submitted rho-limited formula. A nonnegative value
  // deliberately replaces that admission budget, permitting service headroom
  // above the mean MEDIUM arrival rate without changing generated traffic.
  double m_mediumFallbackRefillRate = -1.0;
  bool m_purgeExpiredMediumBeforeAdmission = false;
  double m_lowFallbackTimeout = 900.0;
  double m_lowFallbackRhoMax = 0.0;
  double m_lowFallbackBurstSeconds = 10.0;
  double m_lowFallbackTokens = 0.0;
  double m_highDeadline = 120.0;
  double m_mediumDeadline = 600.0;
  double m_lowDeadline = 30000.0; // LOW is best effort; exceeds one slow patrol
  // Correlated alarm-burst stress test: inject m_alarmBurstSize HIGH alarms at
  // one DDN simultaneously at m_alarmBurstTime (models a leak/anchor-strike event
  // hitting the shared acoustic bypass at once). 0 = disabled.
  uint32_t m_alarmBurstSize = 0;
  double   m_alarmBurstTime = 5000.0;
  bool m_completePatrolInSimTime = false;
  bool m_tracePositions = false;
  bool m_enableCriticalDirect = true;
  bool m_enableHighFallback = true;
  bool m_enableMediumFallback = true;
  bool m_lowPayloadAtGateway = false;
  std::string m_auvCollectionMedium = "auto";
  bool m_includeMobilityIdleEnergy = true;
  double m_nodeIdlePowerW = 0.0005;
  // Finite per-DDN buffer capacity (packets, total across priority queues).
  // 0 = unbounded (legacy). Overflow uses priority-aware drop.
  uint32_t m_ddnBufferCapacity = 0;
  // Logical application allocation, not total MCU RAM or sensor flash.
  // The independent record limit remains active when nonzero. Both limits
  // disabled means the legacy unbounded queue, a sizing control only.
  uint64_t m_ddnBufferCapacityBytes = 0;
  uint32_t m_ddnRecordOverheadBytes = 0;
  std::string m_ddnByteBudgetScope = "queue";
  bool m_hardwareSelfTest = false;
  std::string m_storageCsv;
  std::vector<uint64_t> m_storageHighWaterBytes;
  std::vector<uint64_t> m_storageHighWaterRecords;
  std::vector<uint64_t> m_residentHighWaterBytes;
  std::vector<uint64_t> m_residentHighWaterRecords;
  // Overflow policy: "priority" (default, alarm-preserving preemption) or
  // "fifo" (plain drop-tail, no priority protection — ablation only).
  std::string m_ddnBufferPolicy = "priority";
  // Optional direct-reporting control: reuse the hybrid per-class queues,
  // strict non-preemptive service, and overflow policy while retaining the
  // same direct acoustic delivery path. Disabled to preserve legacy runs.
  bool m_matchHybridQueues = false;
  std::string m_protocolName = "baseline";
  ProtocolMode m_protocol = ProtocolMode::BASELINE;
  std::string m_metricsCsv;
  std::string m_packetTraceCsv;
  std::string m_workloadReplayCsv;
  std::string m_diagnosticsCsv;
  double m_diagnosticsInterval = 60.0;
  double m_nextDiagnosticsTime = 0.0;
  std::vector<GatewayDiagnostics> m_gatewayDiagnostics;
  // AquaSim's internal battery gates PHY operation independently of the
  // manuscript's communication/mission energy ledger. Negative leaves the
  // library default untouched; a positive override applies to every device.
  double m_phyInitialEnergyJ = -1.0;
  std::string m_phyEnergyCsv;
  double m_phyEnergyInterval = 300.0;
  double m_nextPhyEnergyTime = 0.0;
  std::vector<Ptr<AquaSimEnergyModel>> m_phyEnergyModels;
  std::vector<double> m_phyFirstDepletedAt;
  std::string m_animXml = "animation.xml";
  bool m_enableAnimation = true;

  uint32_t m_numBsn = 100;
  uint32_t m_numDrn = 20;
  uint32_t m_numDdn = 6;
  uint32_t m_nextBsn = 0;
  uint64_t m_nextPacketId = 1;

  std::vector<LogicalNode> m_bsns;
  std::vector<LogicalNode> m_drns;
  std::vector<LogicalNode> m_ddns;
  std::vector<LogicalNode> m_sinkNodes;
  std::deque<DataPacket> m_auvBuffer;
  std::vector<std::deque<DataPacket>> m_ddnBuffers;
  std::vector<std::deque<DataPacket>> m_ddnHighBuffers;
  std::vector<std::deque<DataPacket>> m_ddnMediumBuffers;
  std::vector<std::deque<DataPacket>> m_ddnLowBuffers;
  Stats m_stats;

  NodeContainer m_allNodes;
  Ptr<Node> m_auvNode;
  Ptr<MobilityModel> m_auvMobility;
  // One forwarding app per BSN; indexed in BSN order (ns3 node id = index).
  std::vector<Ptr<EdcForwardingApp>> m_bsnFwdApps;
  std::vector<Ptr<EdcForwardingApp>> m_ddnSinkApps;
  std::vector<Ptr<EdcForwardingApp>> m_ddnAuvApps;
  std::vector<Ptr<EdcForwardingApp>> m_auvSinkApps;
  // Packet creation times keyed by packet ID (needed for E2E delay in real forwarding).
  std::unordered_map<uint64_t, PendingAcousticPacket> m_pendingAcoustic;
  std::unordered_map<uint64_t, PendingDirectPacket> m_pendingDirect;
  std::unordered_map<uint64_t, PendingMobilePacket> m_pendingMobile;
  std::unordered_map<uint64_t, PendingAuvSurfacePacket> m_pendingAuvSurface;
  std::unordered_map<uint64_t, PendingOpticalPacket> m_pendingOptical;
  std::vector<bool> m_opticalActive;
  std::vector<double> m_opticalPointingLastDraw;
  std::vector<double> m_opticalPointingRadialOffset;
  std::vector<std::deque<uint64_t>> m_directWaiting;
  std::vector<bool> m_directActive;
  std::vector<std::deque<uint64_t>> m_mobileWaiting;
  std::vector<bool> m_mobileActive;
};

void
AuvEdcExperiment::Configure(int argc, char* argv[])
{
  CommandLine cmd;
  cmd.AddValue("reviewedControlMaxPayloadBytes", "Explicit maximum encoded control body; 0 disables controls, no truncation, at most 1 MiB.", m_reviewedControlMaxPayloadBytes);
  cmd.AddValue("reviewedControlLifetimeS", "Explicit positive control transport lifetime from send, carried in the wire header; no implicit ACK/retry.", m_reviewedControlLifetimeS);
  cmd.AddValue("reviewedControlMaxInflight", "Finite source lifetime reservations and separate receiver-local reassembly/dedup slots retained until on-wire expiry; software allocation, not manufacturer memory.", m_reviewedControlMaxInflight);
  cmd.AddValue("reviewedMission", "Continuous fixed-3-kn static mission with one shared finite PHY/vehicle battery; engineering route envelope, not hardware certification.", m_reviewedMission);
  cmd.AddValue("reviewedMissionPolicy", "static or adaptive: same continuous fixed-speed geometry and finite mission; adaptive uses transported causal reports.", m_reviewedMissionPolicy);
  cmd.AddValue("reviewedMissionMaxRevisitS", "Explicit engineering maximum station revisit interval; same hard coverage rule for static/adaptive; negative preserves old static QA only.", m_reviewedMissionMaxRevisitS);
  cmd.AddValue("reviewedTelemetryPeriodS", "Explicit best-effort acoustic report period per gateway in adaptive mode.", m_reviewedTelemetryPeriodS);
  cmd.AddValue("reviewedTelemetryMaxAgeS", "Maximum age from original observation; stale or unknown reports receive no predicted useful bytes.", m_reviewedTelemetryMaxAgeS);
  cmd.AddValue("reviewedTelemetryCsv", "Required adaptive report/decision causal trace path.", m_reviewedTelemetryCsv);
  cmd.AddValue("reviewedMissionOperationW", "Explicit whole-vehicle operation baseline at 3 kn, W; additional interfaces must not already be included.", m_reviewedMissionOperationW);
  cmd.AddValue("reviewedMissionRecoveryS", "Explicit positive external recovery-handling duration, s; not a modeled ascent or deck recovery.", m_reviewedMissionRecoveryS);
  cmd.AddValue("reviewedMissionRecoveryJ", "Explicit positive recovery-handling vehicle energy allowance, J.", m_reviewedMissionRecoveryJ);
  cmd.AddValue("reviewedMissionServiceS", "Explicit minimum regular offload service duration, s; rounded up to moving 100-m-radius circles.", m_reviewedMissionServiceS);
  cmd.AddValue("reviewedMissionFinalServiceS", "Explicit terminal home offload service allowance, s; no presumed packet success.", m_reviewedMissionFinalServiceS);
  cmd.AddValue("reviewedMissionCarryBytes", "Explicit finite whole-record AUV carry allocation including optical reservation and unacknowledged surface copies, bytes.", m_reviewedMissionCarryBytes);
  cmd.AddValue("reviewedMissionCsv", "Required exact trajectory decision and custody summary CSV; no new network measurements are implied.", m_reviewedMissionCsv);
  cmd.AddValue("reviewedIngress", "Opt-in finite dedicated logger-to-gateway ingress; requires reviewedTransport and external replay.", m_reviewedIngress);
  cmd.AddValue("reviewedVerticalIngress", "Opt-in HIGH/MED sharing the existing finite logger cable, priority nonpreemptive; upstream acoustic ACK only after durable gateway admission and cable ACK.", m_reviewedVerticalIngress);
  cmd.AddValue("reviewedIngressSelfTest", "Deterministic finite-ingress integration checks; no network run.", m_reviewedIngressSelfTest);
  cmd.AddValue("reviewedIngressArchiveBytes", "Explicit per-logger persistent archive allocation, retained through mission end, bytes.", m_reviewedIngressArchiveBytes);
  cmd.AddValue("reviewedIngressSourceRamBytes", "Explicit per-logger whole-chunk staging RAM allocation, bytes; not total firmware RAM.", m_reviewedIngressSourceRamBytes);
  cmd.AddValue("reviewedIngressGatewayRamBytes", "Explicit per-gateway dedicated-cable staging RAM allocation, bytes.", m_reviewedIngressGatewayRamBytes);
  cmd.AddValue("reviewedIngressBitRate", "Explicit dedicated cable bit rate, bit/s; not acoustic service rate.", m_reviewedIngressBitRate);
  cmd.AddValue("reviewedIngressWireBitsPerByte", "Explicit serial framing bits per byte, e.g. a declared 8N1 engineering choice.", m_reviewedIngressWireBitsPerByte);
  cmd.AddValue("reviewedIngressDataHeaderBytes", "Explicit extra DATA wire header bytes; zero must be explicitly supplied.", m_reviewedIngressDataHeaderBytes);
  cmd.AddValue("reviewedIngressAckBytes", "Explicit positive serialized cable ACK/NACK/credit size, bytes.", m_reviewedIngressAckBytes);
  cmd.AddValue("reviewedIngressPropagationS", "Explicit one-way cable propagation delay, seconds.", m_reviewedIngressPropagationS);
  cmd.AddValue("reviewedIngressDurableWriteS", "Explicit gateway durable-admission service delay, seconds per chunk.", m_reviewedIngressDurableWriteS);
  cmd.AddValue("reviewedIngressTurnaroundS", "Explicit turnaround before reverse cable ACK/NACK, seconds.", m_reviewedIngressTurnaroundS);
  cmd.AddValue("reviewedIngressRetryS", "Positive source retry backoff after cable NACK, seconds.", m_reviewedIngressRetryS);
  cmd.AddValue("reviewedIngressAckTimeoutS", "Positive extra timeout after a lost cable ACK, seconds; reliable cable ACK is the declared current model.", m_reviewedIngressAckTimeoutS);
  cmd.AddValue("reviewedIngressMaxAttempts", "Finite DATA attempts per explicit serialized-credit epoch, including initial attempt.", m_reviewedIngressMaxAttempts);
  cmd.AddValue("reviewedIngressRetentionS", "Archive retention boundary, seconds from mission start; must cover simStop.", m_reviewedIngressRetentionS);
  cmd.AddValue("reviewedIngressSourceTxW", "Explicit source electrical cable DATA-TX active power, W; engineering input.", m_reviewedIngressSourceTxW);
  cmd.AddValue("reviewedIngressSourceRxW", "Explicit source electrical cable ACK/credit-RX active power, W.", m_reviewedIngressSourceRxW);
  cmd.AddValue("reviewedIngressGatewayTxW", "Explicit gateway electrical cable ACK/credit-TX active power, W.", m_reviewedIngressGatewayTxW);
  cmd.AddValue("reviewedIngressGatewayRxW", "Explicit gateway electrical cable DATA-RX active power, W.", m_reviewedIngressGatewayRxW);
  cmd.AddValue("reviewedIngressCsv", "Finite ingress identity, timing, ownership and per-owner active-energy event CSV.", m_reviewedIngressCsv);
  cmd.AddValue("reviewedIngressSourceBudgetJ", "Explicit usable per-source logger battery budget J; separate from gateway PHY battery.", m_reviewedIngressSourceBudgetJ);
  cmd.AddValue("reviewedIngressSourceBaseW", "Explicit continuous acquisition/logger/archive baseline W per source, including completed-window production.", m_reviewedIngressSourceBaseW);
  cmd.AddValue("reviewedTransport", "Opt-in actual reverse acoustic ACKs and lifetime-bounded leg custody; requires closureMode.", m_reviewedTransport);
  cmd.AddValue("reviewedSelfTest", "Deterministic reviewed header/custody tests; no traffic campaign.", m_reviewedSelfTest);
  cmd.AddValue("reviewedAckPayloadBytes", "Explicit ACK payload bytes, additional to the 51-byte reviewed wire header; no implicit vendor frame.", m_reviewedAckPayloadBytes);
  cmd.AddValue("reviewedMaxAttempts", "Explicit lifetime maximum DATA attempts per acoustic leg, including initial attempt (1--254).", m_reviewedMaxAttempts);
  cmd.AddValue("closureMode", "Opt-in PARTIAL transport slice; does not certify full physical closure.", m_closureMode);
  cmd.AddValue("closureFullValidation", "Request full closure validation; always rejected by this incomplete transport slice.", m_closureFullValidation);
  cmd.AddValue("closureTransportSelfTest", "Run deterministic transport-slice tests, without a network campaign.", m_closureTransportSelfTest);
  cmd.AddValue("closureAcousticTxPowerW", "Explicit numerical acoustic TX input power W, no vendor default.", m_closureAcousticTxPowerW);
  cmd.AddValue("closureAcousticRxPowerW", "Explicit numerical intended application-arrival RX power W, no vendor default.", m_closureAcousticRxPowerW);
  cmd.AddValue("closureBsnRangeM", "Fixed BSN acoustic transmit range m; no connectivity enlargement.", m_closureBsnRangeM);
  cmd.AddValue("closureDrnRangeM", "Fixed relay acoustic transmit range m; no connectivity enlargement.", m_closureDrnRangeM);
  cmd.AddValue("closureGatewayRangeM", "Fixed gateway acoustic transmit range m, including direct and mobile links.", m_closureGatewayRangeM);
  cmd.AddValue("closureAuvRangeM", "Fixed AUV acoustic transmit/offload range m.", m_closureAuvRangeM);
  cmd.AddValue("closureSinkRangeM", "Fixed surface-device acoustic transmit range m.", m_closureSinkRangeM);
  cmd.AddValue("closureRetrySemantics", "Must explicitly be legacy-endpoint-cycle: BSN leg bounded; direct/AUV cycles may restart until deadline/contact/horizon.", m_closureRetrySemantics);
  cmd.AddValue("simStop", "Simulation stop time in seconds.", m_simStop);
  cmd.AddValue("trafficStop",
               "Stop generating new packets at this time; negative uses simStop. Use < simStop for drain-period evaluation.",
               m_trafficStop);
  cmd.AddValue("nodes", "Nominal horizontal topology positions; EOM modes add one elevated DDN endpoint per collection site.", m_totalNodes);
  cmd.AddValue("startBsnIndex",
               "Diagnostic: BSN index used for the first generated packet.",
               m_nextBsn);
  cmd.AddValue("sinks", "Number of surface sinks.", m_sinks);
  cmd.AddValue("sinkPlacement",
               "Surface-sink placement: midgap (historical), ddn (above selected DDNs), or uniform.",
               m_sinkPlacement);
  cmd.AddValue("trafficLoad", "Aggregate network offered load in packets/s.", m_trafficLoad);
  cmd.AddValue("workloadReplayCsv", "Explicit matched workload; replaces the random generator. LOW records originate at a separate gateway-connected instrument.", m_workloadReplayCsv);
  cmd.AddValue("packetSize", "Legacy payload-size override in bytes; 0 uses class-specific sizes.", m_packetSize);
  cmd.AddValue("highPacketSize", "HIGH application payload size in bytes.", m_highPacketSize);
  cmd.AddValue("mediumPacketSize", "MEDIUM application payload size in bytes.", m_mediumPacketSize);
  cmd.AddValue("lowPacketSize", "LOW application payload size in bytes.", m_lowPacketSize);
  cmd.AddValue("lowPayloadAtGateway",
               "Generate LOW bulk records at their collection gateway instead of sending the full record over the bottom acoustic access chain.",
               m_lowPayloadAtGateway);
  cmd.AddValue("channelBandwidth", "Deprecated acoustic occupied-bandwidth metadata; the applied PHY rate is acousticBitRate.", m_channelBandwidth);
  cmd.AddValue("acousticBitRate", "Acoustic PHY bit rate in bit/s. Paper uses 31.2 kbit/s.", m_acousticBitRate);
  cmd.AddValue("acousticFramePayloadBytes",
               "Maximum application payload per acoustic fragment.",
               m_acousticFramePayloadBytes);
  cmd.AddValue("pipelineLength", "Pipeline length in meters. Paper uses 12600 m.", m_pipelineLength);
  cmd.AddValue("depth", "Pipeline deployment depth in meters.", m_depth);
  cmd.AddValue("ddnEomOffset",
               "Vertical EOM offset: contribution DDN depth is pipeline depth minus this value.",
               m_ddnEomOffset);
  cmd.AddValue("opticalRange",
               "Maximum DDN-AUV distance allowing short-range UWOC transfer in contribution mode.",
               m_opticalRange);
  cmd.AddValue("opticalWaterType",
               "UWOC water type: very-clear, clear-ocean, coastal, turbid-harbor, or Jerlov labels I/IB/II/III/1C/3C.",
               m_opticalWaterType);
  cmd.AddValue("opticalExtCoeff",
               "Optional UWOC extinction coefficient c=a+b in 1/m. Negative uses opticalWaterType.",
               m_opticalExtCoeffOverride);
  cmd.AddValue("opticalReferenceRange",
               "UWOC calibration range in metres for clear-ocean BER=1e-5.",
               m_opticalReferenceRange);
  cmd.AddValue("opticalDataRateBps",
               "UWOC payload data rate during AUV-DDN contact.",
               m_opticalDataRateBps);
  cmd.AddValue("opticalDivergenceDeg",
               "UWOC transmitter divergence angle used for pointing-loss footprint.",
               m_opticalDivergenceDeg);
  cmd.AddValue("opticalPointingSigma",
               "Rayleigh radial pointing jitter scale in metres; 0 disables pointing loss.",
               m_opticalPointingSigma);
  cmd.AddValue("opticalPointingCoherenceSeconds",
               "Hold one pointing-offset draw for this many seconds at each gateway; 0 redraws per transfer.",
               m_opticalPointingCoherenceSeconds);
  cmd.AddValue("disableOpticalLoss",
               "Debug switch: restore ideal optical links in contribution mode.",
               m_disableOpticalLoss);
  cmd.AddValue("auvSpeedKmh", "AUV path speed in km/h. Final Paper-A envelope is 2--5 kn (3.704--9.260 km/h).", m_auvSpeedKmh);
  cmd.AddValue("auvInstantaneousPower", "Integrate cubic power using actual sinusoidal-trajectory speed and enforce geometric peak speed (default false preserves legacy).", m_auvInstantaneousPower);
  cmd.AddValue("auvMaxSpeedKmh", "Maximum admissible AUV speed in km/h; default is 5 kn.", m_auvMaxSpeedKmh);
  cmd.AddValue("auvFixedPowerW", "Fixed AUV hotel/propulsion-intercept power P0 in P=P0+k*v^3 [W].", m_auvFixedPowerW);
  cmd.AddValue("auvPropulsionCoeff", "Cubic AUV propulsion coefficient k in P=P0+k*v^3 [W/(m/s)^3].", m_auvPropulsionCoeff);
  cmd.AddValue("auvDepth",
               "Legacy compatibility value; the sinusoidal path contact depth is derived from protocol and EOM lift.",
               m_auvDepth);
  cmd.AddValue("auvAmplitude", "Lateral amplitude of the sinusoidal AUV path in meters.", m_auvAmplitude);
  cmd.AddValue("completePatrolInSimTime",
               "Non-paper convenience option: scale AUV speed so it can complete one back-and-forth patrol during simStop.",
               m_completePatrolInSimTime);
  cmd.AddValue("tick", "AUV collection scheduler period in seconds.", m_tick);
  cmd.AddValue("sampleInterval", "CSV metrics sampling interval in seconds.", m_sampleInterval);
  cmd.AddValue("metricSampleStart",
               "Time of the first periodic metrics sample in seconds.",
               m_metricSampleStart);
  cmd.AddValue("helloTimeout", "Hello message timeout in seconds. Paper uses 5 s.", m_helloTimeout);
  cmd.AddValue("acousticEnergyPerByteMeter",
               "Abstract acoustic TX energy coefficient in J/(byte*m).",
               m_acousticEnergyPerByteMeter);
  cmd.AddValue("opticalEnergyPerByteMeter",
               "Abstract optical TX energy coefficient in J/(byte*m).",
               m_opticalEnergyPerByteMeter);
  cmd.AddValue("energyModel",
               "Communication energy model: normalized (legacy coefficients) or hardware (time-at-power modem model).",
               m_energyModel);
  cmd.AddValue("acousticRxPowerW",
               "Acoustic receive power used by the hardware energy model.",
               m_acousticRxPowerW);
  cmd.AddValue("opticalTxPowerW",
               "Optical emitter input power used by the hardware energy model.",
               m_opticalTxPowerW);
  cmd.AddValue("reviewedOpticalControl","Explicit acoustic wake/credit and acoustic confirmation around timed one-way optical bursts.",m_reviewedOpticalControl);
  cmd.AddValue("reviewedOpticalSelfTest","Optical custody/control deterministic integration checks, no network run.",m_reviewedOpticalSelfTest);
  cmd.AddValue("reviewedCompactTrace","Omit only per-fragment acoustic_hop_tx lines; retain all record outcome events.",m_reviewedCompactTrace);
  cmd.AddValue("reviewedOpticalStartupS","Declared optical startup latency [s], not a measured manufacturer value.",m_reviewedOpticalStartupS);
  cmd.AddValue("reviewedOpticalListenS","Finite advertised optical receiver session [s], including startup.",m_reviewedOpticalListenS);
  cmd.AddValue("reviewedOpticalPollS","Minimum interval between wake requests to the same gateway [s].",m_reviewedOpticalPollS);
  cmd.AddValue("reviewedOpticalGatewayStartupW","Gateway emitter startup input power [W].",m_reviewedOpticalGatewayStartupW);
  cmd.AddValue("reviewedOpticalVehicleStartupW","AUV receiver startup input power [W].",m_reviewedOpticalVehicleStartupW);
  cmd.AddValue("reviewedOpticalEfficiency","Declared optical DC supply efficiency (0,1].",m_reviewedOpticalEfficiency);
  cmd.AddValue("reviewedOpticalFrameSuccess","Abstract whole-DATA-frame success probability inside certified contact, not a calibrated UV BER.",m_reviewedOpticalFrameSuccess);
  cmd.AddValue("reviewedOpticalAckWaitS","Finite gateway wait for actual acoustic receipt ACK [s].",m_reviewedOpticalAckWaitS);
  cmd.AddValue("reviewedOpticalTurnaroundS","Nonzero optical burst inter-record/processing interval [s].",m_reviewedOpticalTurnaroundS);
  cmd.AddValue("reviewedOpticalControlPredictionS","Predeclared nominal acoustic control transaction prediction [s]; not a guaranteed stochastic latency.",m_reviewedOpticalControlPredictionS);
  cmd.AddValue("reviewedOpticalMaxAttempts","Lifetime DATA attempt limit per gateway-held record.",m_reviewedOpticalMaxAttempts);
  cmd.AddValue("reviewedOpticalReceiptEntries","Finite AUV optical receipt cache slots, 64 B each, deducted from total carry allocation.",m_reviewedOpticalReceiptEntries);
  cmd.AddValue("reviewedOpticalCreditBytes","Maximum receiver-advertised resident-byte grant per contact session.",m_reviewedOpticalCreditBytes);
  cmd.AddValue("opticalRxPowerW",
               "Optical receiver power used by the hardware energy model.",
               m_opticalRxPowerW);
  cmd.AddValue("auvCollectionMedium",
               "AUV-to-gateway collection medium: auto, acoustic, or optical.",
               m_auvCollectionMedium);
  cmd.AddValue("ddnContactPacketsPerTick",
               "Maximum packets transferred from one DDN to the AUV per AUV scheduler tick.",
               m_ddnContactPacketsPerTick);
  cmd.AddValue("sinkContactPacketsPerTick",
               "Maximum packets transferred from the AUV to a sink per AUV scheduler tick.",
               m_sinkContactPacketsPerTick);
  cmd.AddValue("highFallbackTimeout",
               "Contribution mode: seconds before HIGH packets bypass AUV collection and use direct acoustic delivery.",
               m_highFallbackTimeout);
  cmd.AddValue("mediumFallbackTimeout",
               "Contribution mode: seconds before MEDIUM packets become eligible for aging-aware direct acoustic fallback.",
               m_mediumFallbackTimeout);
  cmd.AddValue("directFallbackTimeout",
               "Backward-compatible alias for mediumFallbackTimeout.",
               m_mediumFallbackTimeout);
  cmd.AddValue("mediumFallbackRhoMax",
               "Contribution mode: maximum fraction of aggregate packet rate admitted to HIGH+MEDIUM direct fallback; 0 disables MEDIUM fallback.",
               m_mediumFallbackRhoMax);
  cmd.AddValue("rhoMax",
               "Alias for mediumFallbackRhoMax, used by the Paper-A optimizer decision vector.",
               m_mediumFallbackRhoMax);
  cmd.AddValue("mediumFallbackBurstSeconds",
               "Token-bucket burst allowance for MEDIUM fallback admission.",
               m_mediumFallbackBurstSeconds);
  cmd.AddValue("mediumFallbackRefillRate",
               "MEDIUM token refill in records/s; -1 preserves the legacy rho-limited formula, nonnegative explicitly replaces that budget.",
               m_mediumFallbackRefillRate);
  cmd.AddValue("purgeExpiredMediumBeforeAdmission",
               "Optional policy ablation: discard an otherwise admissible expired MEDIUM candidate without spending its fallback token; false preserves submitted behavior.",
               m_purgeExpiredMediumBeforeAdmission);
  cmd.AddValue("lowFallbackTimeout",
               "Experimental: seconds before LOW packets become eligible for opportunistic direct acoustic fallback.",
               m_lowFallbackTimeout);
  cmd.AddValue("lowFallbackRhoMax",
               "Experimental: total admitted normalized acoustic load cap allowing LOW fallback after HIGH/MEDIUM reservations; 0 disables LOW fallback.",
               m_lowFallbackRhoMax);
  cmd.AddValue("lowFallbackBurstSeconds",
               "Experimental token-bucket burst allowance for LOW fallback admission.",
               m_lowFallbackBurstSeconds);
  cmd.AddValue("highDeadline",
               "Deadline in seconds for HIGH packets when computing miss ratio.",
               m_highDeadline);
  cmd.AddValue("mediumDeadline",
               "Deadline in seconds for MEDIUM packets when computing miss ratio.",
               m_mediumDeadline);
  cmd.AddValue("lowDeadline",
               "Deadline in seconds for LOW packets when computing miss ratio.",
               m_lowDeadline);
  cmd.AddValue("alarmBurstSize",
               "Correlated HIGH alarms injected simultaneously at one DDN (0=off, stress test).",
               m_alarmBurstSize);
  cmd.AddValue("alarmBurstTime",
               "Time in seconds at which the correlated alarm burst is injected.",
               m_alarmBurstTime);
  cmd.AddValue("metricsCsv", "Optional CSV file for sampled metrics.", m_metricsCsv);
  cmd.AddValue("packetTraceCsv",
               "Optional per-packet event CSV used for analytical calibration.",
               m_packetTraceCsv);
  cmd.AddValue("diagnosticsCsv", "Optional bounded per-gateway reviewer diagnostics CSV.",
               m_diagnosticsCsv);
  cmd.AddValue("diagnosticsInterval", "Per-gateway diagnostic snapshot interval in seconds.",
               m_diagnosticsInterval);
  cmd.AddValue("phyInitialEnergyJ",
               "Internal AquaSim battery reserve per device in joules; -1 preserves library default, positive applies equally to every node without changing the manuscript energy ledger.",
               m_phyInitialEnergyJ);
  cmd.AddValue("phyEnergyCsv", "Optional read-only internal PHY energy telemetry CSV.",
               m_phyEnergyCsv);
  cmd.AddValue("phyEnergyInterval", "All-node internal PHY energy snapshot interval in seconds.",
               m_phyEnergyInterval);
  cmd.AddValue("animXml",
               "Optional NetAnim XML output path. Empty string disables NetAnim output.",
               m_animXml);
  cmd.AddValue("enableAnimation",
               "Enable NetAnim XML generation.",
               m_enableAnimation);
  cmd.AddValue("tracePositions", "Print AUV position at each tick.", m_tracePositions);
  cmd.AddValue("enableCriticalDirect",
               "In contribution mode, allow high-priority packets to be sent directly from DDN to sink when the AUV is distant.",
               m_enableCriticalDirect);
  cmd.AddValue("enableHighFallback",
               "Enable the age-triggered HIGH acoustic fallback path when direct fallback is enabled.",
               m_enableHighFallback);
  cmd.AddValue("enableMediumFallback",
               "Enable token-budgeted MEDIUM acoustic fallback when direct fallback is enabled.",
               m_enableMediumFallback);
  cmd.AddValue("includeMobilityIdleEnergy",
               "Include AUV mobility energy and node idle/hibernation energy in energyConsumedJ.",
               m_includeMobilityIdleEnergy);
  cmd.AddValue("nodeIdlePowerW",
               "Idle/listening power charged per nominal submerged position in watts.",
               m_nodeIdlePowerW);
  cmd.AddValue("ddnBufferCapacity",
               "Finite per-DDN buffer capacity in packets (0 = unbounded). Overflow drops with priority preemption.",
               m_ddnBufferCapacity);
  cmd.AddValue("ddnBufferCapacityBytes", "Per-gateway logical application allocation in bytes (0 disables byte limit); scope selected by ddnByteBudgetScope, independent of record limit.", m_ddnBufferCapacityBytes);
  cmd.AddValue("ddnRecordOverheadBytes", "Assumed stored metadata bytes per queued record, in addition to payload (not a measured MCU footprint).", m_ddnRecordOverheadBytes);
  cmd.AddValue("storageCsv", "Optional per-record gateway storage/drop trace and occupancy snapshots.", m_storageCsv);
  cmd.AddValue("ddnByteBudgetScope", "Byte budget scope: queue (legacy application queue) or resident (unique gateway-owned records including pending transmissions).", m_ddnByteBudgetScope);
  cmd.AddValue("hardwareSelfTest", "Run deterministic storage/motion extension self-tests and exit without simulating.", m_hardwareSelfTest);
  cmd.AddValue("ddnBufferPolicy",
               "DDN overflow policy: 'priority' (default, alarm-preserving) or 'fifo' (drop-tail ablation, no priority protection).",
               m_ddnBufferPolicy);
  cmd.AddValue("matchHybridQueues",
               "For pure-acoustic reporting, reuse the hybrid per-class queues, strict priority service, and overflow policy.",
               m_matchHybridQueues);
  cmd.AddValue("protocol", "Protocol mode: baseline, contribution, pure-acoustic, or madcs-hp.", m_protocolName);
  cmd.AddValue("acousticFreqKhz",
               "Center frequency for Thorp absorption model (kHz). Default 10 kHz.",
               m_acousticFreqKhz);
  cmd.AddValue("acousticMac",
               "AquaSim MAC: aloha (default, ACK/backoff) or broadcast (legacy).",
               m_acousticMac);
  cmd.AddValue("txPowerMarginDb",
               "Acoustic modem link margin at 1 m reference distance (dB). "
               "Represents source level minus integrated noise floor in channel bandwidth.",
               m_txPowerMarginDb);
  cmd.AddValue("macLoadPenaltyDb",
               "Deprecated compatibility option; ignored because AquaSim MAC events model contention.",
               m_macLoadPenaltyDb);
  cmd.AddValue("acousticMaxRetransmissions",
               "Maximum source retransmissions after the initial BSN acoustic attempt.",
               m_acousticMaxRetransmissions);
  cmd.AddValue("acousticAckTimeout",
               "Seconds to wait for endpoint delivery confirmation.",
               m_acousticAckTimeout);
  cmd.AddValue("acousticRetryBackoff",
               "Base seconds of uniform retry backoff after an ACK timeout.",
               m_acousticRetryBackoff);
  // Multi-seed support: use --RngRun=1..5 and average results across runs.
  // ns-3 RngSeedManager controls the global pseudo-random stream.
  uint32_t rngRun  = 1;
  uint32_t rngSeed = 1;
  cmd.AddValue("RngRun",  "Random number generator run index (1–N for averaging).", rngRun);
  cmd.AddValue("RngSeed", "Random number generator seed.",                           rngSeed);
  cmd.AddValue("reviewedPhyEnergy", "Opt-in finite elapsed PHY waveform energy and half duplex; not full physical validation.", m_reviewedPhyEnergy);
  const char* energyRoles[] = {"Bsn", "Drn", "Gateway", "Surface", "Auv"};
  for (unsigned i=0; i<5; ++i)
    {
      cmd.AddValue(std::string("reviewed")+energyRoles[i]+"BatteryJ", "Explicit finite per-device electrical budget J; no inferred vendor capacity.", m_reviewedBatteryJ[i]);
      if (i<4) cmd.AddValue(std::string("reviewed")+energyRoles[i]+"BaseW", "Explicit non-radio device baseline W, excludes radio and separately metered payloads.", m_reviewedBaseW[i]);
    }
  cmd.AddValue("reviewedIdlePowerW", "Explicit acoustic listening power W.", m_reviewedIdlePowerW);
  cmd.AddValue("reviewedSleepPowerW", "Explicit acoustic sleep power W; default policy is awake.", m_reviewedSleepPowerW);
  cmd.AddValue("reviewedEnergyJson", "Terminal authoritative per-owner waveform energy JSON; legacy metric energy remains diagnostic.", m_reviewedEnergyJson);
  cmd.Parse(argc, argv);
  NS_ABORT_MSG_IF(m_reviewedControlMaxPayloadBytes &&
      (!m_reviewedTransport || !m_reviewedPhyEnergy || m_reviewedControlMaxPayloadBytes>1048576 ||
       !std::isfinite(m_reviewedControlLifetimeS) || m_reviewedControlLifetimeS<=0 ||
       m_reviewedControlLifetimeS>1e6 || m_reviewedControlMaxInflight==0 || m_reviewedControlMaxInflight>65535),
      "Controls require reviewed physical transport, explicit positive bounded payload/lifetime and finite receive slots");
  if (m_reviewedPhyEnergy)
    {
      NS_ABORT_MSG_IF(!m_reviewedTransport || m_reviewedEnergyJson.empty(), "Reviewed PHY energy requires reviewed transport and an explicit JSON output");
      for (unsigned i=0; i<5; ++i)
        NS_ABORT_MSG_IF(!std::isfinite(m_reviewedBatteryJ[i]) || m_reviewedBatteryJ[i]<=0 ||
                        !std::isfinite(m_reviewedBaseW[i]) || m_reviewedBaseW[i]<0,
                        "Reviewed PHY requires finite positive per-role battery and explicit nonnegative baseline power");
      NS_ABORT_MSG_IF(!std::isfinite(m_reviewedIdlePowerW) || m_reviewedIdlePowerW<0 ||
                      !std::isfinite(m_reviewedSleepPowerW) || m_reviewedSleepPowerW<0,
                      "Reviewed PHY requires explicit listening and sleep powers");
    }
  NS_ABORT_MSG_IF(m_reviewedTransport && (!m_closureMode ||
      m_reviewedAckPayloadBytes == 0 || m_reviewedAckPayloadBytes > 65535 ||
      m_reviewedMaxAttempts == 0 || m_reviewedMaxAttempts > 254),
      "Reviewed transport requires closureMode, explicit ACK payload 1--65535 bytes and lifetime attempts 1--254");
  NS_ABORT_MSG_IF(m_closureFullValidation,
                  "CLOSURE_NOT_READY: authoritative PHY-state energy, durable ingress, finite battery/route and adaptive-policy integration require independent validation");
  RngSeedManager::SetSeed(rngSeed);
  RngSeedManager::SetRun(rngRun);

  if (m_protocolName == "baseline")
    {
      m_protocol = ProtocolMode::BASELINE;
    }
  else if (m_protocolName == "contribution")
    {
      m_protocol = ProtocolMode::CONTRIBUTION;
      if (m_depth <= m_ddnEomOffset)
        {
          m_depth = m_ddnEomOffset * 2.0;
        }
    }
  else if (m_protocolName == "pure-acoustic" || m_protocolName == "pure_acoustic")
    {
      m_protocol = ProtocolMode::PURE_ACOUSTIC;
      m_protocolName = "pure-acoustic";
      if (m_depth <= m_ddnEomOffset)
        {
          m_depth = m_ddnEomOffset * 2.0;
        }
    }
  else if (m_protocolName == "madcs-hp" || m_protocolName == "madcs_hp" ||
           m_protocolName == "madcs" || m_protocolName == "clmd-tdma" ||
           m_protocolName == "clmd_tdma" || m_protocolName == "clmd")
    {
      m_protocol = ProtocolMode::MADCS_HP;
      m_protocolName = "madcs-hp";
    }
  else
    {
      NS_FATAL_ERROR("protocol must be 'baseline', 'contribution', 'pure-acoustic', or 'madcs-hp'");
    }

  m_auvCollectionMedium = NormalizeName(m_auvCollectionMedium);
  if (m_auvCollectionMedium == "auto")
    {
      m_auvCollectionMedium =
          (m_protocol == ProtocolMode::CONTRIBUTION) ? "optical" : "acoustic";
    }
  if (m_protocol == ProtocolMode::PURE_ACOUSTIC)
    {
      m_auvCollectionMedium = "none";
    }
  else if (m_auvCollectionMedium != "acoustic" &&
           m_auvCollectionMedium != "optical")
    {
      NS_FATAL_ERROR("auvCollectionMedium must be auto, acoustic, or optical");
    }

  m_energyModel = NormalizeName(m_energyModel);
  if (m_energyModel != "normalized" && m_energyModel != "hardware")
    {
      NS_FATAL_ERROR("energyModel must be normalized or hardware");
    }

  if (m_closureMode)
    {
      for (double value : {m_closureAcousticTxPowerW, m_closureAcousticRxPowerW,
                           m_closureBsnRangeM, m_closureDrnRangeM,
                           m_closureGatewayRangeM, m_closureAuvRangeM,
                           m_closureSinkRangeM})
        NS_ABORT_MSG_IF(!std::isfinite(value) || value <= 0.0,
                        "Closure transport requires explicit finite positive TX/RX powers and all five role ranges");
      NS_ABORT_MSG_IF(m_energyModel != "hardware",
                      "Closure transport requires hardware numerical energy accounting");
      NS_ABORT_MSG_IF(!std::isfinite(m_acousticFreqKhz) ||
                      m_acousticFreqKhz < 20.0 || m_acousticFreqKhz > 34.0,
                      "Closure MF transport requires an explicit 20--34 kHz absorption-frequency proxy; no hardware waveform calibration is claimed");
      const std::vector<double> supportedRates{100.0, 200.0, 400.0, 900.0,
                                              3000.0, 3500.0, 6000.0, 9000.0};
      NS_ABORT_MSG_IF(std::find(supportedRates.begin(), supportedRates.end(),
                               m_acousticBitRate) == supportedRates.end(),
                      "Closure MF transport requires an explicitly selected documented service-rate scale; no MDFT waveform reproduction is claimed");
      NS_ABORT_MSG_IF(m_closureRetrySemantics != (m_reviewedTransport
                         ? "reviewed-leg-lifetime" : "legacy-endpoint-cycle"),
                      "Explicit retry semantics must match mode: reviewed-leg-lifetime or legacy-endpoint-cycle");
      m_bsnRange = m_closureBsnRangeM;
      m_drnRange = m_closureDrnRangeM;
      m_ddnRange = m_closureGatewayRangeM;
      m_ddnCollectionRange = m_closureGatewayRangeM;
      m_sinkRange = m_closureAuvRangeM;
      m_acousticRxPowerW = m_closureAcousticRxPowerW;
    }

  m_ddnBufferPolicy = NormalizeName(m_ddnBufferPolicy);
  m_ddnByteBudgetScope = NormalizeName(m_ddnByteBudgetScope);
  if (m_ddnByteBudgetScope != "queue" && m_ddnByteBudgetScope != "resident")
    NS_FATAL_ERROR("ddnByteBudgetScope must be queue or resident");
  if (m_ddnBufferPolicy != "priority" && m_ddnBufferPolicy != "fifo")
    {
      NS_FATAL_ERROR("ddnBufferPolicy must be priority or fifo");
    }
  if (m_nodeIdlePowerW < 0.0)
    {
      NS_FATAL_ERROR("nodeIdlePowerW must be non-negative");
    }
  if (m_matchHybridQueues && m_protocol != ProtocolMode::PURE_ACOUSTIC)
    {
      NS_FATAL_ERROR("matchHybridQueues is only valid with protocol=pure-acoustic");
    }
  if ((m_ddnBufferCapacityBytes > 0 || !m_storageCsv.empty()) &&
      !(UsesPriorityQueues() &&
        (m_protocol == ProtocolMode::PURE_ACOUSTIC || m_auvCollectionMedium == "optical")))
    {
      NS_FATAL_ERROR("Storage extension supports contribution/optical and pure-acoustic/matchHybridQueues=1 only; legacy mobile-waiting modes are not byte-bounded");
    }
  if (m_auvInstantaneousPower &&
      (m_completePatrolInSimTime || m_auvAmplitude != 0.0))
    {
      NS_FATAL_ERROR("auvInstantaneousPower requires fixed commanded speed (completePatrolInSimTime=0) and the validated 2-D path (auvAmplitude=0)");
    }

  // Evaluated configurations share a fixed 12,600-m pipeline. Changing the
  // logical node count therefore changes spacing, not monitored length.
  if (m_pipelineLength <= 0.0)
    {
      m_pipelineLength = 12600.0; // Table 1: fixed network dimension
    }

  if (m_trafficLoad <= 0.0)
    {
      NS_FATAL_ERROR("trafficLoad must be positive");
    }
  if (m_packetSize > 0)
    {
      m_highPacketSize = m_packetSize;
      m_mediumPacketSize = m_packetSize;
      m_lowPacketSize = m_packetSize;
    }
  if (m_highPacketSize == 0 || m_mediumPacketSize == 0 || m_lowPacketSize == 0)
    {
      NS_FATAL_ERROR("class payload sizes must be positive");
    }
  if (m_auvSpeedKmh <= 0.0 || m_auvMaxSpeedKmh <= 0.0 ||
      m_auvSpeedKmh > m_auvMaxSpeedKmh)
    {
      NS_FATAL_ERROR("auvSpeedKmh must be in (0, auvMaxSpeedKmh]; default maximum is 5 kn");
    }
  if (m_auvFixedPowerW < 0.0 || m_auvPropulsionCoeff <= 0.0)
    {
      NS_FATAL_ERROR("AUV power parameters require auvFixedPowerW >= 0 and auvPropulsionCoeff > 0");
    }
  if (m_acousticRxPowerW <= 0.0 || m_opticalTxPowerW <= 0.0 ||
      m_opticalRxPowerW <= 0.0)
    {
      NS_FATAL_ERROR("hardware communication powers must be positive");
    }
  if (m_channelBandwidth <= 0.0 || m_acousticBitRate <= 0.0)
    {
      NS_FATAL_ERROR("channelBandwidth and acousticBitRate must be positive");
    }
  if (m_acousticFramePayloadBytes == 0 || m_acousticFramePayloadBytes > 65535)
    {
      NS_FATAL_ERROR("acousticFramePayloadBytes must be in [1,65535]");
    }
  if (m_acousticAckTimeout <= 0.0 || m_acousticRetryBackoff < 0.0 ||
      m_acousticMaxRetransmissions > 254)
    {
      NS_FATAL_ERROR("invalid acoustic retry/ACK configuration");
    }
  if (m_trafficStop < 0.0)
    {
      m_trafficStop = m_simStop;
    }
  if (m_trafficStop > m_simStop)
    {
      NS_FATAL_ERROR("trafficStop must be <= simStop");
    }
  if (m_sampleInterval <= 0.0 || m_metricSampleStart < 0.0 ||
      m_metricSampleStart > m_simStop)
    {
      NS_FATAL_ERROR("invalid metrics sampling schedule");
    }
  if (m_opticalRange <= 0.0)
    {
      NS_FATAL_ERROR("opticalRange must be positive");
    }
  if (m_opticalReferenceRange <= 0.0 || m_opticalReferenceExtCoeff <= 0.0 ||
      m_opticalReferenceSnr <= 0.0)
    {
      NS_FATAL_ERROR("optical calibration parameters must be positive");
    }
  if (m_opticalDataRateBps <= 0.0)
    {
      NS_FATAL_ERROR("opticalDataRateBps must be positive");
    }
  if (m_opticalDivergenceDeg <= 0.0 || m_opticalDivergenceDeg >= 180.0)
    {
      NS_FATAL_ERROR("opticalDivergenceDeg must be in (0, 180)");
    }
  if (m_opticalPointingSigma < 0.0 || m_opticalPointingCoherenceSeconds < 0.0)
    {
      NS_FATAL_ERROR("optical pointing parameters must be non-negative");
    }
  if (m_mediumFallbackRhoMax < 0.0 || m_mediumFallbackRhoMax > 1.0 ||
      m_lowFallbackRhoMax < 0.0 || m_lowFallbackRhoMax > 1.0)
    {
      NS_FATAL_ERROR("fallback rho caps must be fractions in [0,1]");
    }
  if (m_mediumFallbackBurstSeconds <= 0.0)
    {
      NS_FATAL_ERROR("mediumFallbackBurstSeconds must be positive");
    }
  if (!std::isfinite(m_mediumFallbackRefillRate) ||
      (m_mediumFallbackRefillRate < 0.0 && m_mediumFallbackRefillRate != -1.0))
    {
      NS_FATAL_ERROR("mediumFallbackRefillRate must be -1 (legacy) or nonnegative");
    }
  if (!std::isfinite(m_diagnosticsInterval) || m_diagnosticsInterval <= 0.0)
    {
      NS_FATAL_ERROR("diagnosticsInterval must be finite and positive");
    }
  if (!std::isfinite(m_phyInitialEnergyJ) ||
      (m_phyInitialEnergyJ != -1.0 && m_phyInitialEnergyJ <= 0.0))
    {
      NS_FATAL_ERROR("phyInitialEnergyJ must be -1 (library default) or positive");
    }
  if (!std::isfinite(m_phyEnergyInterval) || m_phyEnergyInterval <= 0.0)
    {
      NS_FATAL_ERROR("phyEnergyInterval must be finite and positive");
    }
  if (m_lowFallbackRhoMax < 0.0)
    {
      NS_FATAL_ERROR("lowFallbackRhoMax must be non-negative");
    }
  if (m_lowFallbackBurstSeconds <= 0.0)
    {
      NS_FATAL_ERROR("lowFallbackBurstSeconds must be positive");
    }
  (void)OpticalExtinctionCoefficient(); // validates opticalWaterType / override early
  if (m_totalNodes < 12)
    {
      NS_FATAL_ERROR("nodes must be at least 12 to keep BSN/DRN/DDN and 5 sinks meaningful");
    }
  if (m_sinks < 1)  
    {
      NS_FATAL_ERROR("sinks must be at least 1");
    }
  m_sinkPlacement = NormalizeName(m_sinkPlacement);
  if (m_sinkPlacement != "midgap" && m_sinkPlacement != "ddn" &&
      m_sinkPlacement != "uniform" && m_sinkPlacement != "hybrid")
    {
      NS_FATAL_ERROR("sinkPlacement must be midgap, ddn, uniform, or hybrid");
    }
  m_acousticMac = NormalizeName(m_acousticMac);
  if (m_acousticMac != "aloha" && m_acousticMac != "broadcast")
    {
      NS_FATAL_ERROR("acousticMac must be aloha or broadcast");
    }

  NS_ABORT_MSG_IF(m_reviewedVerticalIngress && (!m_reviewedIngress || !m_reviewedTransport || !m_reviewedPhyEnergy),
      "Reviewed vertical ingress requires the shared finite ingress and authoritative PHY owners");
  NS_ABORT_MSG_IF(m_reviewedVerticalIngress &&
      (!std::isfinite(m_reviewedIngressDurableWriteS) || m_reviewedIngressDurableWriteS <= 0),
      "Vertical confirmation requires an explicit positive durable gateway-admission service time");
  if (m_reviewedIngress)
    {
      NS_ABORT_MSG_IF(!m_reviewedTransport || !m_reviewedPhyEnergy || m_workloadReplayCsv.empty() ||
          !UsesPriorityQueues() || !m_lowPayloadAtGateway || m_alarmBurstSize > 0 ||
          m_ddnBufferCapacityBytes == 0 || m_ddnByteBudgetScope != "resident" ||
          m_ddnRecordOverheadBytes != 64 || m_reviewedIngressCsv.empty() ||
          m_reviewedIngressRetentionS < m_simStop ||
          !std::isfinite(m_reviewedIngressSourceBudgetJ) || m_reviewedIngressSourceBudgetJ <= 0 ||
          !std::isfinite(m_reviewedIngressSourceBaseW) || m_reviewedIngressSourceBaseW < 0 ||
          m_reviewedIngressDataHeaderBytes == std::numeric_limits<uint32_t>::max(),
          "Reviewed ingress requires reviewed PHY/shared gateway battery, finite source battery/base power, replay, matched queues, resident finite outbox +64 B, explicit timing/storage/powers/CSV and mission-long archive retention");
      try
        {
          closure_ingress::Callbacks cb;
          cb.on_admitted = [](const closure_ingress::Record&, double) { return false; };
          closure_ingress::Link validate(ReviewedIngressConfiguration(), cb);
        }
      catch (const std::exception& error) { NS_FATAL_ERROR("Invalid reviewed ingress contract: " << error.what()); }
    }
  if (m_reviewedMission)
    {
      NS_ABORT_MSG_IF(m_reviewedMissionPolicy != "static" && m_reviewedMissionPolicy != "adaptive",
          "Reviewed mission policy must be static or adaptive");
      if (m_reviewedMissionPolicy == "adaptive")
        {
          NS_ABORT_MSG_IF(!m_reviewedIngress || !m_reviewedOpticalControl || m_protocol != ProtocolMode::CONTRIBUTION ||
              m_reviewedControlMaxPayloadBytes < closure_adaptive::wire::reportHeaderBytes ||
              m_reviewedTelemetryCsv.empty(),
              "Adaptive mission requires finite metadata-carrying ingress and actual bounded acoustic control transport");
          for (double value : {m_reviewedTelemetryPeriodS, m_reviewedTelemetryMaxAgeS,
                               m_reviewedMissionMaxRevisitS})
            NS_ABORT_MSG_IF(!std::isfinite(value) || value <= 0,
                            "Adaptive telemetry period, observation age and coverage interval require explicit positive values");
        }
      NS_ABORT_MSG_IF(!std::isfinite(m_reviewedMissionMaxRevisitS) || m_reviewedMissionMaxRevisitS == 0,
                      "Mission revisit interval must be positive, or negative for archived static-QA compatibility only");
      NS_ABORT_MSG_IF(m_reviewedOpticalControl && m_reviewedMissionMaxRevisitS <= 0,
                      "Controlled optical missions require the same explicit positive coverage interval for static and adaptive policies");
      NS_ABORT_MSG_IF(!m_reviewedPhyEnergy || !m_reviewedTransport ||
          (m_protocol != ProtocolMode::CONTRIBUTION && m_protocol != ProtocolMode::PURE_ACOUSTIC) ||
          (m_protocol != ProtocolMode::PURE_ACOUSTIC && m_auvCollectionMedium != "optical") || m_reviewedMissionCarryBytes == 0 ||
          m_reviewedMissionCsv.empty(),
          "Reviewed mission requires authoritative PHY energy, reviewed transport, matched contribution/direct mode, optical collection, explicit finite carry and trajectory CSV");
      for (double value : {m_reviewedMissionOperationW, m_reviewedMissionRecoveryS,
                            m_reviewedMissionRecoveryJ, m_reviewedMissionServiceS,
                            m_reviewedMissionFinalServiceS})
        NS_ABORT_MSG_IF(!std::isfinite(value) || value <= 0,
                        "Reviewed mission requires explicit positive operation, service and recovery allowances");
      NS_ABORT_MSG_IF(std::abs(m_reviewedBatteryJ[4] - 67.0 * 3600000.0) > 1e-5 ||
          std::abs(m_auvSpeedKmh / 3.6 - closure_mission::fixedThreeKnotsMps) > 1e-10 ||
          m_completePatrolInSimTime || m_auvAmplitude != 0 ||
          std::abs(m_depth - 1000.0) > 1e-9 || std::abs(m_ddnEomOffset - 500.0) > 1e-9,
          "Reviewed mission preserves 1000-m pipe/500-m gateways, finite 67-kWh vehicle, fixed 3-kn speed, and unchanged observation horizon");
      NS_ABORT_MSG_IF(m_reviewedBaseW[4] != 0,
                      "AUV baseline is owned by reviewed mission through the same PHY ledger; reviewedAuvBaseW must be zero initially");
    }
  ScaleRoles();
}

void
AuvEdcExperiment::ScaleRoles()
{
  // Interleaved role rule: four BSNs per five positions and a DDN at every
  // 25th position; arbitrary N is counted explicitly below. The N=126 branch
  // preserves the historical 100/20/6 mapping.
  //   BASELINE:     DDN at pipeline depth (1000 m), all acoustic forwarding.
  //   CONTRIBUTION: DDN lifted to ddnDepth (500 m) via vertical EOM cable above its
  //                 paired DRN (co-located in x, at pipeline depth).  Relay DRNs at
  //                 i%5 positions forward acoustically toward the nearest paired DRN.
  // Surface-sink count and placement are explicit scenario parameters.
  if (m_totalNodes == 126)
    {
      m_numBsn = 100;
      m_numDrn = 20;
      m_numDdn = 6;
      return;
    }

  // Simulate the interleaved count to get exact numbers for arbitrary N.
  m_numBsn = 0; m_numDrn = 0; m_numDdn = 0;
  for (uint32_t i = 0; i < m_totalNodes; ++i)
    {
      if      (i % 25 == 0) m_numDdn++;
      else if (i % 5  == 0) m_numDrn++;
      else                   m_numBsn++;
    }
  if (m_numDdn == 0) m_numDdn = 1;
  // m_sinks is an explicit deployment parameter. Never derive or overwrite it
  // here: doing so silently ignored --sinks and coupled infrastructure cost to
  // DDN density. Sink-count feasibility is evaluated as a separate scenario.
}

void
AuvEdcExperiment::BuildNs3Topology()
{
  // CONTRIBUTION adds one paired DRN per DDN position (co-located in x), so the
  // actual sensor-node count may exceed m_totalNodes.  Always allocate from the
  // real sizes computed in BuildLogicalTopology().
  uint32_t sensorNodes = m_numBsn + m_numDrn + m_numDdn;
  m_allNodes.Create(sensorNodes + m_sinks + 1);
  m_auvNode = m_allNodes.Get(sensorNodes + m_sinks);

  PacketSocketHelper socketHelper;
  socketHelper.Install(m_allNodes);

  // AquaSimModulation otherwise defaults to 10 ksymbols/s, silently diverging
  // from the 31.2-kbit/s PHY specified in the paper. ALOHA hop ACKs are disabled:
  // this experiment has one bounded endpoint-confirmation/retry mechanism, and
  // stacking it on the MAC's unbounded per-hop queues creates retry storms.
  Config::SetDefault("ns3::AquaSimModulation::SPS",
                     UintegerValue(static_cast<uint32_t>(std::llround(m_acousticBitRate))));
  if (m_acousticMac == "aloha")
    {
      Config::SetDefault("ns3::AquaSimAloha::AckOn", IntegerValue(0));
    }

  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  // RangePropagation filters physical receptions by the transmitting PHY's
  // TransRange before propagation and MAC collision processing.  Spatially
  // separated links therefore form distinct interference neighbourhoods.
  channel.SetPropagation("ns3::AquaSimRangePropagation");

  AquaSimHelper asHelper = AquaSimHelper::Default();
  if (m_reviewedPhyEnergy)
    asHelper.SetPhy("ns3::ReviewedAquaSimPhy", "Frequency", DoubleValue(m_acousticFreqKhz));
  if (m_phyInitialEnergyJ > 0.0)
    asHelper.SetEnergyModel("ns3::AquaSimEnergyModel", "InitialEnergy",
                            DoubleValue(m_phyInitialEnergyJ));
  asHelper.SetChannel(channel.Create());
  asHelper.SetMac(m_acousticMac == "aloha" ? "ns3::AquaSimAloha"
                                            : "ns3::AquaSimBroadcastMac");
  asHelper.SetRouting("ns3::AquaSimStaticRouting");

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();

  uint32_t nodeId = 0;
  for (const auto& node : m_bsns)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice>();
      asHelper.Create(m_allNodes.Get(nodeId++), dev);
      dev->GetPhy()->SetTransRange(node.range);
      positions->Add(node.position);
    }
  for (const auto& node : m_drns)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice>();
      asHelper.Create(m_allNodes.Get(nodeId++), dev);
      dev->GetPhy()->SetTransRange(node.range);
      positions->Add(node.position);
    }
  for (const auto& node : m_ddns)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice>();
      asHelper.Create(m_allNodes.Get(nodeId++), dev);
      // Gateways must reach the surface endpoint across the 500-m vertical
      // handoff while retaining the shorter bottom-network routing gate.
      dev->GetPhy()->SetTransRange(m_closureMode ? m_closureGatewayRangeM
                                               : std::max(node.range, m_sinkRange));
      positions->Add(node.position);
    }
  for (const auto& node : m_sinkNodes)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice>();
      asHelper.Create(m_allNodes.Get(nodeId++), dev);
      dev->GetPhy()->SetTransRange(m_closureMode ? m_closureSinkRangeM : node.range);
      positions->Add(node.position);
    }

  Ptr<AquaSimNetDevice> auvDev = CreateObject<AquaSimNetDevice>();
  asHelper.Create(m_auvNode, auvDev);
  auvDev->GetPhy()->SetTransRange(m_closureMode ? m_closureAuvRangeM
                  : std::max(m_ddnCollectionRange, m_sinkRange) + 1.0);
  positions->Add(GetAuvPosition(0.0));

  mobility.SetPositionAllocator(positions);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  if (m_reviewedMission)
    {
      NodeContainer fixed;
      for (uint32_t i=0;i+1<m_allNodes.GetN();++i) fixed.Add(m_allNodes.Get(i));
      mobility.Install(fixed);
      auto analytic=CreateObject<ReviewedMissionMobility>();
      analytic->SetPosition(GetAuvPosition(0.0));
      m_auvNode->AggregateObject(analytic);
      m_auvMobility=analytic;
    }
  else
    {
      mobility.Install(m_allNodes);
      m_auvMobility=m_auvNode->GetObject<ConstantPositionMobilityModel>();
    }

  // The application selects one next hop within its geometric routing gate.
  // Configure the AquaSim static router so that this destination is forwarded
  // directly to itself, avoiding AquaSimRoutingDummy's flooding behavior.
  // A seed is intentionally reused across paired policies. A seed-only
  // pathname lets concurrent processes truncate each other's route tables
  // during initialization. Use a process-private, atomically created file;
  // its random pathname is unrelated to ns-3's random-number streams.
  char routeTemplate[] = "/tmp/edc-static-routes-XXXXXX";
  const int routeFd = ::mkstemp(routeTemplate);
  if (routeFd < 0) NS_FATAL_ERROR("Cannot create private static-routing table");
  const std::string routeFile(routeTemplate);
  {
    std::FILE* routes = ::fdopen(routeFd, "w");
    if (!routes)
      {
        ::close(routeFd);
        std::remove(routeFile.c_str());
        NS_FATAL_ERROR("Cannot open private static-routing table");
      }
    for (uint32_t i = 0; i < m_allNodes.GetN(); ++i)
      {
        Ptr<AquaSimNetDevice> source =
            DynamicCast<AquaSimNetDevice>(m_allNodes.Get(i)->GetDevice(0));
        int sourceAddress = AquaSimAddress::ConvertFrom(source->GetAddress()).GetAsInt();
        for (uint32_t j = 0; j < m_allNodes.GetN(); ++j)
          {
            if (i == j) continue;
            Ptr<AquaSimNetDevice> destination =
                DynamicCast<AquaSimNetDevice>(m_allNodes.Get(j)->GetDevice(0));
            int destinationAddress =
                AquaSimAddress::ConvertFrom(destination->GetAddress()).GetAsInt();
            std::fprintf(routes, "%d:%d:%d\n", sourceAddress,
                         destinationAddress, destinationAddress);
          }
      }
    if (std::fclose(routes) != 0)
      {
        std::remove(routeFile.c_str());
        NS_FATAL_ERROR("Cannot flush private static-routing table");
      }
  }
  for (uint32_t i = 0; i < m_allNodes.GetN(); ++i)
    {
      Ptr<AquaSimNetDevice> device =
          DynamicCast<AquaSimNetDevice>(m_allNodes.Get(i)->GetDevice(0));
      Ptr<AquaSimStaticRouting> routing =
          DynamicCast<AquaSimStaticRouting>(device->GetRouting());
      routing->SetRouteTable(const_cast<char*>(routeFile.c_str()));
    }
  // AquaSimStaticRouting::SetRouteTable synchronously calls ReadRouteTable
  // and closes the file before returning. Every node now owns its routes.
  if (std::remove(routeFile.c_str()) != 0)
    NS_LOG_WARN("Could not remove private static-routing table " << routeFile);

  if (m_reviewedPhyEnergy) BindReviewedPhyEnergy();
  if (m_reviewedMission)
    {
      ReviewedMissionInitialize();
      DynamicCast<ReviewedMissionMobility>(m_auvMobility)->SetEvaluators(
        [this](double t){return GetAuvPosition(t);},
        [this](double t){return ReviewedMissionVelocity(t);});
    }
  // Install real forwarding applications on every sensor node.
  BuildForwardingApps();
  ReviewedOpticalInitialize();
  if (m_closureMode)
    {
      m_closureNodeTransport.resize(m_allNodes.GetN());
      for (uint32_t i = 0; i < m_allNodes.GetN(); ++i)
        {
          auto state = std::make_shared<ClosureTransportState>();
          m_closureNodeTransport[i] = state;
          Ptr<Node> node = m_allNodes.Get(i);
          for (uint32_t a = 0; a < node->GetNApplications(); ++a)
            {
              auto app = DynamicCast<EdcForwardingApp>(node->GetApplication(a));
              if (app) app->SetClosureTransport(state);
            }
        }
    }
  if (!m_phyEnergyCsv.empty())
    {
      m_phyEnergyModels.reserve(m_allNodes.GetN());
      for (uint32_t i = 0; i < m_allNodes.GetN(); ++i)
        {
          auto device = DynamicCast<AquaSimNetDevice>(m_allNodes.Get(i)->GetDevice(0));
          auto energy = device->EnergyModel();
          if (!energy) NS_FATAL_ERROR("Missing internal AquaSim energy model for node " << i);
          m_phyEnergyModels.push_back(energy);
        }
      m_phyFirstDepletedAt.assign(m_allNodes.GetN(), -1.0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BuildForwardingApps — creates one EdcForwardingApp per sensor node and wires
// them for greedy geographic multi-hop forwarding.
//
// Adaptive range: we scale each node's effective TX range to be at least 10%
// greater than the inter-node spacing.  This guarantees chain connectivity for
// any node count while preserving the paper's relative range ordering.
// ─────────────────────────────────────────────────────────────────────────────
void
AuvEdcExperiment::BuildForwardingApps()
{
  m_bsnFwdApps.clear();
  m_ddnSinkApps.clear();
  m_ddnAuvApps.clear();
  m_auvSinkApps.clear();

  // Collect 3-D positions in ns-3 node-ID order. Keeping co-located nodes
  // distinct lets the packet socket target one physical next-hop device and
  // makes PER/energy use the true vertical as well as horizontal distance.
  std::vector<Vector> allPositions;
  allPositions.reserve(m_bsns.size() + m_drns.size() + m_ddns.size() +
                       m_sinkNodes.size() + 1);
  for (const auto& n : m_bsns) allPositions.push_back(n.position);
  for (const auto& n : m_drns) allPositions.push_back(n.position);
  for (const auto& n : m_ddns) allPositions.push_back(n.position);
  for (const auto& n : m_sinkNodes) allPositions.push_back(n.position);
  allPositions.push_back(m_auvMobility->GetPosition());

  bool hasEomGateway = (m_protocol == ProtocolMode::CONTRIBUTION ||
                        m_protocol == ProtocolMode::PURE_ACOUSTIC);
  const uint32_t staticRelayCount = m_numBsn + m_numDrn + m_numDdn;

  // Legacy mode treats declared BSN/DRN ranges as lower bounds and applies a
  // role-count reachability rule. Closure mode uses the exact explicit role
  // ranges, allowing physical disconnection instead of repairing the topology.
  // Packet success and energy use the actual geometric hop distance.
  double bsnStep = (m_numBsn > 1) ? m_pipelineLength / (m_numBsn - 1) : m_pipelineLength;
  double drnStep = (m_numDrn > 1) ? m_pipelineLength / (m_numDrn - 1) : m_pipelineLength;
  // The mean-spacing enlargement is legacy-only.
  double effBsn = m_closureMode ? m_bsnRange : std::max(m_bsnRange, bsnStep * 1.1);
  double effDrn = m_closureMode ? m_drnRange : std::max(m_drnRange, drnStep * 1.1);
  double effDdn  = m_ddnRange;
  for (uint32_t i = 0; i < m_numBsn; ++i)
    DynamicCast<AquaSimNetDevice>(m_allNodes.Get(i)->GetDevice(0))->GetPhy()->SetTransRange(effBsn);
  for (uint32_t j = 0; j < m_numDrn; ++j)
    DynamicCast<AquaSimNetDevice>(m_allNodes.Get(m_numBsn + j)->GetDevice(0))->GetPhy()->SetTransRange(effDrn);
  for (uint32_t k = 0; k < m_numDdn; ++k)
    DynamicCast<AquaSimNetDevice>(m_allNodes.Get(m_numBsn + m_numDrn + k)->GetDevice(0))->GetPhy()->SetTransRange(effDdn);

  EdcForwardingApp::DdnCb ddnCb =
      [this](uint64_t id, uint32_t src, uint8_t prio, uint8_t hops,
             uint8_t attempt, uint32_t payloadBytes, uint32_t ddnNs3)
  {
      this->RealEnqueueAtDdn(id, src, prio, hops, attempt, payloadBytes, ddnNs3);
  };
  EdcForwardingApp::EnergyCb energyCb =
      [this](uint8_t prio, uint32_t payloadBytes, double distance,
             uint32_t nodeId, bool isTx)
  {
      const uint32_t wireBytes = payloadBytes + EdcHeader().GetSerializedSize();
      const double joules = isTx
          ? AcousticTxEnergyJ(wireBytes, distance)
          : AcousticRxEnergyJ(wireBytes);
      ChargePacketEnergy(static_cast<Priority>(prio), joules,
                         NodeRoleFromNs3Id(nodeId));
      if (isTx)
        {
          m_stats.acousticTransfers++;
          m_stats.acousticTxBytes += wireBytes;
          m_stats.acousticTxAirtimeS += wireBytes * 8.0 / m_acousticBitRate;
        }
  };
  EdcForwardingApp::DdnCb surfaceCb =
      [this](uint64_t id, uint32_t src, uint8_t prio, uint8_t hops,
             uint8_t attempt, uint32_t payloadBytes, uint32_t unused)
  {
      if (m_pendingDirect.count(id))
        this->RealDeliverToSink(id, src, prio, hops, attempt, payloadBytes, unused);
      else
        this->RealAuvDeliverToSink(id, src, prio, hops, attempt, payloadBytes, unused);
  };
  EdcForwardingApp::ReceptionCb receptionCb =
      [this](uint32_t payloadBytes, double distance)
  {
      return m_acousticReceptionRng->GetValue() <=
          AcousticPacketSuccessProbability(distance, payloadBytes);
  };
  EdcForwardingApp::DdnCb auvCb =
      [this](uint64_t id, uint32_t src, uint8_t prio, uint8_t hops,
             uint8_t attempt, uint32_t payloadBytes, uint32_t unused)
  {
      this->RealCollectByAuv(id, src, prio, hops, attempt, payloadBytes, unused);
  };
  EdcForwardingApp::HopCb hopCb =
      [this](uint64_t id, uint32_t src, uint8_t prio, uint32_t payloadBytes,
             uint8_t attempt, uint32_t from, uint32_t to, uint32_t dst,
             double distance)
  {
      this->TraceAcousticHop(id, src, prio, payloadBytes, attempt,
                             from, to, dst, distance);
  };

  if (m_reviewedTransport)
    {
      // A single app per physical device owns reviewed DATA and ACK routing,
      // including the BSN return endpoint and the wired DRN's acoustic port.
      for (uint32_t id = 0; id < allPositions.size(); ++id)
        {
          const NodeRole role = NodeRoleFromNs3Id(id);
          const double range = role == NodeRole::BSN ? m_closureBsnRangeM :
              role == NodeRole::DRN ? m_closureDrnRangeM :
              role == NodeRole::DDN ? m_closureGatewayRangeM :
              role == NodeRole::SINK ? m_closureSinkRangeM : m_closureAuvRangeM;
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(id, allPositions[id], range, id, allPositions[id].x,
                     allPositions, true, id, ddnCb);
          app->SetEnergyCallback(energyCb);
          app->SetReceptionCallback(receptionCb);
          app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
          app->SetRelayCandidateCount(staticRelayCount);
          app->SetHopCallback(hopCb);
          app->SetReviewedCallbacks(
              [this](const EdcHeader& h) { ReviewedArrival(h); },
              [this, id](const EdcHeader& h) { return ReviewedFrameActive(h, id); });
          if (m_reviewedPhyEnergy)
            app->SetLocalTxOutcome([this,id](const ReviewedLocalTxTag& tag, bool accepted, double endAt) {
              ReviewedLocalTxOutcome(tag,id,accepted,endAt);
            });
          app->SetControlTransport(m_reviewedControlMaxPayloadBytes,m_reviewedControlMaxInflight,
              [this](const EdcHeader& h,const std::vector<uint8_t>& bytes) { ReceiveReviewedControl(h,bytes); },
              [this](const std::string& event,const EdcHeader& h,uint32_t node,const std::string& reason) {
                TraceReviewedControl(event,h,node,reason);
              });
          m_allNodes.Get(id)->AddApplication(app);
          app->SetStartTime(Seconds(0.0));
          app->SetStopTime(Seconds(m_simStop + 1.0));
          m_reviewedApps.push_back(app);
        }
      return;
    }

  // ── BSN apps ──────────────────────────────────────────────────────────────
  for (uint32_t i = 0; i < m_numBsn; ++i)
    {
      uint32_t drnIdx = AssociatedDrn(i);
      uint32_t ddnIdx = hasEomGateway
                            ? DdnForDrn(drnIdx)
                            : NearestDdn(m_bsns[i].position.x);

      uint32_t finalDstNs3; // acoustic endpoint NS3 node index
      double   finalDstX;
      if (hasEomGateway)
        {
          // The packet's ultimate acoustic endpoint is the DRN physically
          // paired with the selected DDN, not merely the BSN's nearest relay.
          // Intermediate BSN/DRN apps read this destination from the header.
          uint32_t pairedDrnIdx = 0;
          double bestDistance = std::numeric_limits<double>::max();
          for (uint32_t k = 0; k < m_numDrn; ++k)
            {
              const double distance = std::abs(
                  m_drns[k].position.x - m_ddns[ddnIdx].position.x);
              if (distance < bestDistance)
                {
                  bestDistance = distance;
                  pairedDrnIdx = k;
                }
            }
          finalDstNs3 = m_numBsn + pairedDrnIdx;
          finalDstX   = m_drns[pairedDrnIdx].position.x;
        }
      else
        {
          // BSN routes acoustically all the way to the DDN
          finalDstNs3 = m_numBsn + m_numDrn + ddnIdx;
          finalDstX   = m_ddns[ddnIdx].position.x;
        }

      auto app = CreateObject<EdcForwardingApp>();
      app->Setup(i,                     // myId = BSN NS3 index
                 m_bsns[i].position,
                 effBsn,
                 finalDstNs3, finalDstX,
                 allPositions,
                 false,                 // BSN is a relay, not the endpoint
                 0,                     // callbackDstId unused for relays
                 ddnCb);
      app->SetEnergyCallback(energyCb);
      app->SetReceptionCallback(receptionCb);
      app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
      app->SetRelayCandidateCount(staticRelayCount);
      app->SetHopCallback(hopCb);
      m_allNodes.Get(i)->AddApplication(app);
      app->SetStartTime(Seconds(0.0));
      app->SetStopTime(Seconds(m_simStop + 1.0));
      m_bsnFwdApps.push_back(app);
    }

  // ── DRN apps ──────────────────────────────────────────────────────────────
  for (uint32_t j = 0; j < m_numDrn; ++j)
    {
      uint32_t ddnIdx   = DdnForDrn(j);
      uint32_t ddnNs3Id = m_numBsn + m_numDrn + ddnIdx;
      uint32_t drnNs3Id = m_numBsn + j;

      // EOM modes: paired DRN (co-located with DDN, same x) = acoustic endpoint → EOM.
      //               relay DRN = forwards acoustically toward the nearest paired DRN.
      // BASELINE:     all DRNs are relays toward the DDN.
      bool isPaired   = hasEomGateway &&
                        (std::abs(m_drns[j].position.x - m_ddns[ddnIdx].position.x) < 1.0);
      bool isEndpoint = isPaired;

      uint32_t finalDstNs3;
      double   finalDstX;
      if (!hasEomGateway)
        {
          finalDstNs3 = m_numBsn + m_numDrn + ddnIdx;
          finalDstX   = m_ddns[ddnIdx].position.x;
        }
      else if (isPaired)
        {
          finalDstNs3 = drnNs3Id;            // endpoint = self
          finalDstX   = m_drns[j].position.x;
        }
      else
        {
          // Relay: target the paired DRN co-located with the nearest DDN
          double   ddnX       = m_ddns[ddnIdx].position.x;
          uint32_t pairedNs3  = m_numBsn;
          double   bestD      = std::numeric_limits<double>::max();
          for (uint32_t k = 0; k < m_numDrn; ++k)
            {
              double d = std::abs(m_drns[k].position.x - ddnX);
              if (d < bestD) { bestD = d; pairedNs3 = m_numBsn + k; }
            }
          finalDstNs3 = pairedNs3;
          finalDstX   = ddnX;
        }

      auto app = CreateObject<EdcForwardingApp>();
      app->Setup(drnNs3Id,
                 m_drns[j].position,
                 effDrn,
                 finalDstNs3, finalDstX,
                 allPositions,
                 isEndpoint,
                 ddnNs3Id,             // wired: callback delivers to this DDN
                 ddnCb);
      app->SetEnergyCallback(energyCb);
      app->SetReceptionCallback(receptionCb);
      app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
      app->SetRelayCandidateCount(staticRelayCount);
      app->SetHopCallback(hopCb);
      m_allNodes.Get(drnNs3Id)->AddApplication(app);
      app->SetStartTime(Seconds(0.0));
      app->SetStopTime(Seconds(m_simStop + 1.0));
    }

  // ── DDN apps (non-EOM modes only: acoustic endpoint) ──────────────────────
  // EOM DDNs receive via callback from their paired DRN (no acoustic app).
  if (!hasEomGateway)
    {
      for (uint32_t k = 0; k < m_numDdn; ++k)
        {
          uint32_t ddnNs3Id = m_numBsn + m_numDrn + k;
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(ddnNs3Id,
                     m_ddns[k].position,
                     effDdn,
                     ddnNs3Id, m_ddns[k].position.x, // finalDst = self
                     allPositions,
                     true,        // IS the acoustic endpoint
                     ddnNs3Id,    // callback passes its own ns3 id
                     ddnCb);
          app->SetEnergyCallback(energyCb);
          app->SetReceptionCallback(receptionCb);
          app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
          m_allNodes.Get(ddnNs3Id)->AddApplication(app);
          app->SetStartTime(Seconds(0.0));
          app->SetStopTime(Seconds(m_simStop + 1.0));
        }
    }

  // Acoustic mobile collection: actual AquaSim DDN->AUV packets. The
  // moving AUV position is read from its MobilityModel at every TX/RX event.
  if (m_protocol != ProtocolMode::PURE_ACOUSTIC &&
      m_auvCollectionMedium == "acoustic")
    {
      const uint32_t auvNodeId = m_numBsn + m_numDrn + m_numDdn + m_sinks;
      for (uint32_t k = 0; k < m_numDdn; ++k)
        {
          const uint32_t ddnNodeId = m_numBsn + m_numDrn + k;
          Ptr<AquaSimNetDevice> ddnDevice =
              DynamicCast<AquaSimNetDevice>(m_allNodes.Get(ddnNodeId)->GetDevice(0));
          ddnDevice->GetPhy()->SetTransRange(m_closureMode ? m_closureGatewayRangeM
              : std::max(effDdn, m_ddnCollectionRange) + 1.0);
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(ddnNodeId, m_ddns[k].position,
                     m_closureMode ? m_closureGatewayRangeM : m_ddnCollectionRange + 1.0,
                     auvNodeId, m_auvMobility->GetPosition().x,
                     allPositions, false, 0, auvCb);
          app->SetEnergyCallback(energyCb);
          app->SetReceptionCallback(receptionCb);
          app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
          app->SetRelayEnabled(false);
          m_allNodes.Get(ddnNodeId)->AddApplication(app);
          app->SetStartTime(Seconds(0.0));
          app->SetStopTime(Seconds(m_simStop + 1.0));
          m_ddnAuvApps.push_back(app);
        }

      auto auvApp = CreateObject<EdcForwardingApp>();
      auvApp->Setup(auvNodeId, m_auvMobility->GetPosition(),
                    m_closureMode ? m_closureAuvRangeM : m_ddnCollectionRange + 1.0,
                    auvNodeId, m_auvMobility->GetPosition().x,
                    allPositions, true, 0, auvCb);
      auvApp->SetEnergyCallback(energyCb);
      auvApp->SetReceptionCallback(receptionCb);
      auvApp->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
      m_auvNode->AddApplication(auvApp);
      auvApp->SetStartTime(Seconds(0.0));
      auvApp->SetStopTime(Seconds(m_simStop + 1.0));
    }

  // Contribution direct bypass: one sender app per DDN targets its nearest
  // surface sink; sink apps observe actual AquaSim delivery events.
  if (m_protocol == ProtocolMode::CONTRIBUTION ||
      m_protocol == ProtocolMode::PURE_ACOUSTIC)
    {
      for (uint32_t k = 0; k < m_numDdn; ++k)
        {
          uint32_t ddnNodeId = m_numBsn + m_numDrn + k;
          uint32_t nearestSink = 0;
          double bestDistance = std::numeric_limits<double>::max();
          for (uint32_t s = 0; s < m_sinkNodes.size(); ++s)
            {
              double dx = m_sinkNodes[s].position.x - m_ddns[k].position.x;
              double dz = m_sinkNodes[s].position.z - m_ddns[k].position.z;
              double distance = std::hypot(dx, dz);
              if (distance < bestDistance)
                { bestDistance = distance; nearestSink = s; }
            }
          uint32_t sinkNodeId = m_numBsn + m_numDrn + m_numDdn + nearestSink;
          Ptr<AquaSimNetDevice> ddnDevice =
              DynamicCast<AquaSimNetDevice>(m_allNodes.Get(ddnNodeId)->GetDevice(0));
          const double directRange = m_closureMode ? m_closureGatewayRangeM
                                                   : bestDistance + 1.0;
          ddnDevice->GetPhy()->SetTransRange(directRange);
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(ddnNodeId, m_ddns[k].position, directRange,
                     sinkNodeId, m_sinkNodes[nearestSink].position.x,
                     allPositions, false, 0, surfaceCb);
          app->SetEnergyCallback(energyCb);
          app->SetReceptionCallback(receptionCb);
          app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
          app->SetRelayEnabled(false);
          m_allNodes.Get(ddnNodeId)->AddApplication(app);
          app->SetStartTime(Seconds(0.0));
          app->SetStopTime(Seconds(m_simStop + 1.0));
          m_ddnSinkApps.push_back(app);
        }
    }

  // Every protocol uses real sink DATA-arrival events. The same endpoint callback
  // dispatches DDN bypass packets and AUV-carried packets by pending state.
  for (uint32_t s = 0; s < m_sinkNodes.size(); ++s)
    {
      const uint32_t sinkNodeId = m_numBsn + m_numDrn + m_numDdn + s;
      auto sinkApp = CreateObject<EdcForwardingApp>();
      sinkApp->Setup(sinkNodeId, m_sinkNodes[s].position,
                     m_closureMode ? m_closureSinkRangeM : m_sinkRange + 1.0,
                     sinkNodeId, m_sinkNodes[s].position.x,
                     allPositions, true, 0, surfaceCb);
      sinkApp->SetEnergyCallback(energyCb);
      sinkApp->SetReceptionCallback(receptionCb);
      sinkApp->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
      m_allNodes.Get(sinkNodeId)->AddApplication(sinkApp);
      sinkApp->SetStartTime(Seconds(0.0));
      sinkApp->SetStopTime(Seconds(m_simStop + 1.0));
    }

  if (m_protocol != ProtocolMode::PURE_ACOUSTIC)
    {
      const uint32_t auvNodeId = m_numBsn + m_numDrn + m_numDdn + m_sinks;
      for (uint32_t s = 0; s < m_sinkNodes.size(); ++s)
        {
          const uint32_t sinkNodeId = m_numBsn + m_numDrn + m_numDdn + s;
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(auvNodeId, m_auvMobility->GetPosition(),
                     m_closureMode ? m_closureAuvRangeM : m_sinkRange + 1.0,
                     sinkNodeId, m_sinkNodes[s].position.x,
                     allPositions, false, 0, surfaceCb);
          app->SetEnergyCallback(energyCb);
          app->SetReceptionCallback(receptionCb);
          app->SetFragmentation(m_acousticFramePayloadBytes, m_acousticBitRate);
          app->SetRelayEnabled(false);
          m_auvNode->AddApplication(app);
          app->SetStartTime(Seconds(0.0));
          app->SetStopTime(Seconds(m_simStop + 1.0));
          m_auvSinkApps.push_back(app);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reviewed acoustic DATA/ACK transaction integration.
void
AuvEdcExperiment::SetReviewedControlHandler(uint8_t kind, ReviewedControlHandler handler)
{
  NS_ABORT_MSG_IF(kind==0 || !handler || m_reviewedControlHandlers.count(kind),
                  "Control kind requires one nonempty receiver callback");
  m_reviewedControlHandlers.emplace(kind,std::move(handler));
}

uint64_t
AuvEdcExperiment::SendReviewedControl(uint32_t sender,uint32_t receiver,uint8_t kind,
                                      uint64_t nonce,const std::vector<uint8_t>& payload)
{
  EdcHeader h; h.m_reviewed=true; h.m_leg=255; h.m_controlVersion=1; h.m_controlKind=kind;
  h.m_srcId=sender; h.m_dstId=receiver; h.m_id=nonce; h.m_prio=0; h.m_attempt=1;
  h.m_payloadBytes=static_cast<uint32_t>(std::min<size_t>(payload.size(),UINT32_MAX));
  if(!m_reviewedPhyEnergy || !m_reviewedControlMaxPayloadBytes || kind==0 ||
     payload.empty() || payload.size()>m_reviewedControlMaxPayloadBytes ||
     sender==receiver || sender>=m_reviewedApps.size() || receiver>=m_reviewedApps.size() ||
     m_reviewedControlNextToken==0)
    { ++m_reviewedControlRejected; TraceReviewedControl("control_rejected",h,sender,"invalid_sender_endpoint_body_or_disabled"); return 0; }
  h.m_token=m_reviewedControlNextToken++;
  h.m_controlExpiryNs=static_cast<uint64_t>((Simulator::Now()+Seconds(m_reviewedControlLifetimeS)).GetNanoSeconds());
  if(!m_reviewedApps[sender]->SendControlPayload(h,payload))
    { ++m_reviewedControlRejected; TraceReviewedControl("control_rejected",h,sender,"local_fragmentation_or_body_limit"); return 0; }
  ++m_reviewedControlEnqueued;
  m_reviewedControlPayloadBytesEnqueued+=payload.size();
  TraceReviewedControl("control_enqueued",h,sender,"body_serialized_best_effort_no_ack");
  return h.m_token;
}

void
AuvEdcExperiment::ReceiveReviewedControl(const EdcHeader& h,const std::vector<uint8_t>& payload)
{
  const auto handler=m_reviewedControlHandlers.find(h.m_controlKind);
  if(!h.IsControl() || h.m_controlVersion!=1 || h.m_ack || payload.size()!=h.m_payloadBytes ||
     h.m_controlExpiryNs<=static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()) ||
     handler==m_reviewedControlHandlers.end())
    { ++m_reviewedControlRejected; TraceReviewedControl("control_rejected",h,h.m_dstId,"expired_malformed_or_unregistered_kind"); return; }
  ++m_reviewedControlReceived;
  TraceReviewedControl("control_received",h,h.m_dstId,"complete_body_from_actual_receiver_reassembly");
  handler->second(h.m_srcId,h.m_dstId,h.m_id,payload);
}

void
AuvEdcExperiment::TraceReviewedControl(const std::string& event,const EdcHeader& h,
                                       uint32_t node,const std::string& reason)
{
  if(event=="control_fragment_handoff")
    { ++m_reviewedControlFragmentHandoffs; m_reviewedControlAppWireBytes+=h.m_fragmentBytes+h.GetSerializedSize(); }
  if(event=="control_fragment_rejected" || event=="control_receive_rejected")
    ++m_reviewedControlFragmentRejections;
  if(m_packetTraceCsv.empty()) return;
  std::ofstream out(m_packetTraceCsv,std::ios::app);
  out << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ',' << event << ','
      << h.m_id << ',' << h.m_srcId << ",255," << h.m_payloadBytes << ",-1,-1,1,-1.000000,"
      << "kind=" << unsigned(h.m_controlKind) << ";token=" << h.m_token << ";node=" << node
      << ";dst=" << h.m_dstId << ";fragment=" << h.m_fragmentIndex << ";count=" << h.m_fragmentCount
      << ";fragmentBytes=" << h.m_fragmentBytes << ";headerBytes=" << h.GetSerializedSize()
      << ";reason=" << reason << '\n';
}

uint32_t
AuvEdcExperiment::ReviewedUpstreamReceiver(uint32_t ddnIndex) const
{
  if (m_protocol != ProtocolMode::CONTRIBUTION && m_protocol != ProtocolMode::PURE_ACOUSTIC)
    return m_numBsn + m_numDrn + ddnIndex;
  uint32_t best = 0;
  double distance = std::numeric_limits<double>::max();
  for (uint32_t j = 0; j < m_drns.size(); ++j)
    {
      const double d = std::abs(m_drns[j].position.x - m_ddns[ddnIndex].position.x);
      if (d < distance) { best = j; distance = d; }
    }
  return m_numBsn + best;
}

void
AuvEdcExperiment::StartReviewedLeg(const DataPacket& packet, uint8_t leg,
    uint32_t sender, uint32_t receiver, uint32_t ddnIndex, uint32_t sinkIndex)
{
  const auto key = std::make_pair(packet.id, leg);
  // Lifetime state never resets on a contact/cycle retry.
  if (m_reviewedLegTokens.count(key)) return;
  ReviewedLegState state;
  state.packet = packet; state.token = m_reviewedNextToken++;
  state.leg = leg; state.sender = sender; state.receiver = receiver;
  state.ddnIndex = ddnIndex; state.sinkIndex = sinkIndex;
  m_reviewedLegTokens.emplace(key, state.token);
  m_reviewedLegs.emplace(state.token, state);
  SendReviewedAttempt(state.token);
}

bool
AuvEdcExperiment::ReviewedFrameActive(const EdcHeader& h, uint32_t transmittingNode) const
{
  if(h.IsControl())
    return h.m_controlVersion==1 && h.m_controlKind!=0 && !h.m_ack &&
      h.m_srcId==transmittingNode &&
      h.m_controlExpiryNs>static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()) &&
      (!m_reviewedMission || transmittingNode!=m_numBsn+m_numDrn+m_numDdn+m_sinks ||
       ReviewedMissionCommunicationsAvailable());
  auto it = m_reviewedLegs.find(h.m_token);
  if (it == m_reviewedLegs.end()) return false;
  const auto& s = it->second;
  if (s.packet.id != h.m_id || s.leg != h.m_leg) return false;
  if (m_reviewedMission && transmittingNode == m_numBsn+m_numDrn+m_numDdn+m_sinks &&
      !ReviewedMissionCommunicationsAvailable()) return false;
  // Only the source can cancel its own obsolete DATA reservations. Relays and
  // ACK emitters do not receive an instantaneous oracle of source timeout or
  // ACK state. Already in-flight DATA/ACKs complete and pay their real cost.
  if (transmittingNode != s.sender || h.m_ack) return true;
  return !s.terminal && h.m_attempt == s.attempts;
}

void
AuvEdcExperiment::SendReviewedAttempt(uint64_t token)
{
  auto& s = m_reviewedLegs.at(token);
  if (s.terminal) return;
  if (DeadlineExpired(s.packet)) { EndReviewedLeg(token, false, true); return; }
  if (s.attempts >= m_reviewedMaxAttempts) { EndReviewedLeg(token, false, false); return; }
  ++s.attempts;
  s.localFragmentOutcomes.clear();
  s.localExpectedFragments=(s.packet.sizeBytes + m_acousticFramePayloadBytes - 1) /
                           m_acousticFramePayloadBytes;
  s.localLastTxEndS=Simulator::Now().GetSeconds();
  s.localTxRejected=false; s.localCompletionScheduled=false;
  const bool retry = s.attempts > 1;
  if (s.leg == 1)
    { ++m_stats.acousticSourceAttempts; m_stats.acousticRetransmissions += retry; }
  else if (s.leg == 2)
    { ++m_stats.directSourceAttempts; m_stats.directRetransmissions += retry; }
  else if (s.leg == 3)
    { ++m_stats.mobileSourceAttempts; m_stats.mobileRetransmissions += retry; }
  else
    { ++m_stats.auvSurfaceSourceAttempts; m_stats.auvSurfaceRetransmissions += retry; }
  EdcHeader h;
  h.m_reviewed = true; h.m_token = token; h.m_leg = s.leg;
  h.m_id = s.packet.id; h.m_prio = static_cast<uint8_t>(s.packet.priority);
  h.m_payloadBytes = s.packet.sizeBytes; h.m_dstId = s.receiver; h.m_attempt = s.attempts;
  if(s.packet.priority==Priority::LOW && s.packet.windowChunkCount>0)
    {
      h.m_windowMetadata=true; h.m_windowNumericId=s.packet.windowNumericId;
      h.m_windowPayloadBytes=s.packet.windowPayloadBytes;
      h.m_windowChunkIndex=s.packet.windowChunkIndex; h.m_windowChunkCount=s.packet.windowChunkCount;
    }
  const double completeAt = m_reviewedApps.at(s.sender)->SendReviewed(h);
  TracePacket("reviewed_data_tx", s.packet, s.ddnIndex, s.sinkIndex,
              s.attempts, -1.0, "leg=" + std::to_string(s.leg));
  const double now = Simulator::Now().GetSeconds();
  // PHY-enabled mode waits for actual local waveform outcomes, including MAC
  // queue delay and its own framing. App handoff completion is not TX completion.
  // This initial timer is only a hard record-lifetime safeguard for a lower MAC
  // stall/drop that does not report a physical or application admission outcome.
  const double expiryAt = s.packet.createdAt + DeadlineSeconds(s.packet.priority) + 1e-6;
  const double timeoutAt = m_reviewedPhyEnergy ? expiryAt :
                          std::min(completeAt + m_acousticAckTimeout, expiryAt);
  s.timeout = Simulator::Schedule(Seconds(std::max(1e-6, timeoutAt - now)),
      &AuvEdcExperiment::ReviewedTimeout, this, token, s.attempts);
}

void
AuvEdcExperiment::ReviewedLocalTxOutcome(const ReviewedLocalTxTag& tag, uint32_t nodeId,
                                         bool accepted, double completionAt)
{
  auto it=m_reviewedLegs.find(tag.token);
  if(it==m_reviewedLegs.end()) return;
  auto& s=it->second;
  // Only the local source's current DATA attempt can advance its timer. Relay
  // events, old attempts and terminal transactions convey no remote knowledge.
  if(!m_reviewedPhyEnergy || s.terminal || s.sender!=nodeId || s.attempts!=tag.attempt ||
     tag.fragments!=s.localExpectedFragments || tag.fragment>=tag.fragments ||
     s.localCompletionScheduled || !std::isfinite(completionAt)) return;
  if(!s.localFragmentOutcomes.insert(tag.fragment).second) return;
  s.localLastTxEndS=std::max(s.localLastTxEndS,std::max(completionAt,Simulator::Now().GetSeconds()));
  s.localTxRejected|=!accepted;
  if(s.localFragmentOutcomes.size()!=s.localExpectedFragments) return;
  // A final-index range refusal can arrive before earlier MAC-queued frames
  // start. Settle ALL fragment outcomes and their latest physical end first.
  s.localCompletionScheduled=true;
  s.timeout.Cancel();
  const double now=Simulator::Now().GetSeconds();
  const double expiryAt=s.packet.createdAt+DeadlineSeconds(s.packet.priority)+1e-6;
  if(s.localLastTxEndS>=expiryAt)
    s.timeout=Simulator::Schedule(Seconds(std::max(1e-6,expiryAt-now)),
        &AuvEdcExperiment::ReviewedTimeout,this,tag.token,tag.attempt);
  else
    s.timeout=Simulator::Schedule(Seconds(std::max(0.0,s.localLastTxEndS-now)),
        &AuvEdcExperiment::ReviewedLocalAttemptComplete,this,tag.token,tag.attempt);
}

void
AuvEdcExperiment::ReviewedLocalAttemptComplete(uint64_t token, uint8_t attempt)
{
  auto it=m_reviewedLegs.find(token);
  if(it==m_reviewedLegs.end()) return;
  auto& s=it->second;
  if(s.terminal || s.attempts!=attempt || !s.localCompletionScheduled) return;
  ++m_reviewedPhysicalAttemptsSettled;
  m_reviewedLocalRejectedAttempts+=s.localTxRejected;
  TracePacket("reviewed_local_tx_complete",s.packet,s.ddnIndex,s.sinkIndex,
      attempt,-1.0,"leg="+std::to_string(s.leg)+";fragments="+
      std::to_string(s.localExpectedFragments)+";rejected="+(s.localTxRejected?"1":"0"));
  const double now=Simulator::Now().GetSeconds();
  const double expiryAt=s.packet.createdAt+DeadlineSeconds(s.packet.priority)+1e-6;
  // A known local refusal cannot deliver this complete attempt. Retry only
  // after all other local fragments have settled, still under the same cap.
  // Otherwise the unchanged explicit ACK grace starts at physical TX completion.
  const double wait=s.localTxRejected ? 1e-6 : m_acousticAckTimeout;
  s.timeout=Simulator::Schedule(Seconds(std::max(1e-6,std::min(now+wait,expiryAt)-now)),
      &AuvEdcExperiment::ReviewedTimeout,this,token,attempt);
}

void
AuvEdcExperiment::ReviewedTimeout(uint64_t token, uint8_t attempt)
{
  auto& s = m_reviewedLegs.at(token);
  if (s.terminal || s.attempts != attempt) return;
  if (DeadlineExpired(s.packet) || s.attempts >= m_reviewedMaxAttempts)
    { EndReviewedLeg(token, false, DeadlineExpired(s.packet)); return; }
  const double backoff = m_acousticRetryBackoff * m_retryRng->GetValue(0.5, 1.5);
  s.timeout = Simulator::Schedule(Seconds(backoff),
      &AuvEdcExperiment::SendReviewedAttempt, this, token);
}

bool
AuvEdcExperiment::TryReviewedUpstreamAdmission(uint32_t ddnIndex, DataPacket packet)
{
  // This acknowledges acceptance into the authoritative application queue,
  // not a validated flash write or finite DRN-to-gateway cable transaction.
  packet.ddnArrivalAt = Simulator::Now().GetSeconds();
  bool accepted = false;
  if (UsesPriorityQueues())
    {
      NS_ABORT_MSG_IF(m_reviewedUpstreamAdmissionId != 0, "Nested upstream queue admission");
      m_reviewedUpstreamAdmissionId = packet.id;
      accepted = EnqueueContribPacket(ddnIndex, packet);
      m_reviewedUpstreamAdmissionId = 0;
    }
  else if (m_ddnBufferCapacity == 0 || m_ddnBuffers.at(ddnIndex).size() < m_ddnBufferCapacity)
    {
      m_ddnBuffers.at(ddnIndex).push_back(packet);
      accepted = true;
    }
  if (!accepted)
    {
      ++m_reviewedUpstreamAdmissionRejected;
      TracePacket("reviewed_upstream_admission_rejected", packet, ddnIndex, -1,
                  0, -1.0, "source_retains_retry_custody_no_ack");
      return false;
    }
  ++m_stats.reachedDdn;
  ++m_stats.acousticDelivered;
  m_stats.acousticDelaySum += packet.ddnArrivalAt - packet.createdAt;
  return true;
}

void
AuvEdcExperiment::ReviewedArrival(const EdcHeader& h)
{
  auto it = m_reviewedLegs.find(h.m_token);
  if (it == m_reviewedLegs.end()) { ++m_reviewedStaleFrames; return; }
  auto& s = it->second;
  if(!h.m_ack && (h.HasWindowMetadata()!=(s.packet.windowChunkCount>0) ||
      (h.HasWindowMetadata() && (h.m_windowNumericId!=s.packet.windowNumericId ||
       h.m_windowPayloadBytes!=s.packet.windowPayloadBytes ||
       h.m_windowChunkIndex!=s.packet.windowChunkIndex || h.m_windowChunkCount!=s.packet.windowChunkCount ||
       h.m_windowNumericId==0 || h.m_windowPayloadBytes==0 || h.m_windowChunkIndex>=h.m_windowChunkCount))))
    { ++m_reviewedStaleFrames; return; }
  if (s.packet.id != h.m_id || s.leg != h.m_leg ||
      h.m_attempt == 0 || h.m_attempt > s.attempts ||
      h.m_payloadBytes != (h.m_ack ? m_reviewedAckPayloadBytes : s.packet.sizeBytes) ||
      h.m_srcId != (h.m_ack ? s.receiver : s.sender) ||
      h.m_dstId != (h.m_ack ? s.sender : s.receiver))
    { ++m_reviewedStaleFrames; return; }
  if (h.m_ack)
    {
      // A valid delayed ACK for an earlier attempt confirms the same lifetime
      // token, including when a newer DATA attempt has already been scheduled.
      if (!s.arrived || s.terminal) { ++m_reviewedStaleFrames; return; }
      EndReviewedLeg(s.token, true, false);
      return;
    }
  const bool verticalCommit = !h.m_ack && m_reviewedVerticalCommitToken == h.m_token;
  const bool verticalConfirm = !h.m_ack && m_reviewedVerticalConfirmToken == h.m_token;
  if (s.leg == 1 && !verticalCommit && !verticalConfirm)
    {
      ++m_reviewedUpstreamCompleteReceipts;
      m_reviewedUpstreamReceivedIds.insert(s.packet.id);
      TracePacket("reviewed_upstream_data_received", s.packet, s.ddnIndex, -1,
                  h.m_attempt, -1.0, "complete_data_before_queue_admission");
    }
  if (DeadlineExpired(s.packet)) { ++m_reviewedStaleFrames; return; }
  if (s.leg == 1 && m_reviewedVerticalIngress && !verticalCommit && !verticalConfirm)
    {
      const auto local = m_reviewedVertical.find(s.packet.id);
      if (local == m_reviewedVertical.end() || !local->second.cableConfirmed)
        { ReviewedVerticalReceive(h); return; }
      // A duplicate acoustic DATA may be acknowledged only using the actual
      // cable confirmation already received at this local bottom endpoint.
    }
  if (!s.arrived)
    {
      // A complete waveform alone does not transfer custody. A rejected
      // incoming record remains source-owned and does not generate an ACK.
      if (s.leg == 1 && !TryReviewedUpstreamAdmission(s.ddnIndex, s.packet)) return;
      // Source exhaustion cannot teleport an in-flight frame out of the
      // channel. A later, still-in-deadline receiver arrival is real delivery,
      // but never resurrects the terminated sender transaction. Reclassify
      // its provisional undelivered outcome once, preserving conservation.
      if (s.terminal)
        {
          NS_ABORT_MSG_IF(s.expired || s.confirmed, "Invalid reviewed late-recovery state");
          if (s.leg == 1) --m_stats.acousticRetryExhausted;
          else --m_reviewedRetryDropped;
          ++m_reviewedLateRecovered;
          ++m_reviewedUnconfirmedReceived;
        }
      s.arrived = true;
      s.arrivalAt = Simulator::Now().GetSeconds();
      ++m_reviewedDataArrivals;
      TracePacket("reviewed_data_arrived", s.packet, s.ddnIndex, s.sinkIndex,
                  h.m_attempt, -1.0, "leg=" + std::to_string(s.leg));
      if (s.leg == 1)
        { /* The authoritative queue accepted this record above. */ }
      else if (s.leg == 3)
        { ++m_stats.collectedByAuv; m_auvBuffer.push_back(s.packet); }
      else
        {
          const double delay = s.arrivalAt - s.packet.createdAt;
          ++m_stats.surfaceDelivered; ++m_stats.deliveredToSink;
          m_stats.surfaceDelaySum += delay; m_stats.delaySum += delay;
          if (s.leg == 2)
            {
              if (s.packet.priority == Priority::HIGH) ++m_stats.directCriticalToSink;
              else if (s.packet.priority == Priority::MEDIUM) ++m_stats.directMediumToSink;
              else ++m_stats.directLowToSink;
            }
          AccountDeliveredPriority(s.packet.priority, delay, s.packet.burst, s.packet.sizeBytes);
        }
    }
  else if (!verticalConfirm) ++m_reviewedDuplicateData;
  // Durable gateway commitment updates logical receiver custody immediately,
  // but is not local sender knowledge. Wait for the serialized cable ACK.
  if (verticalCommit) return;
  EdcHeader ack = h;
  ack.m_ack = true; ack.m_dstId = s.sender;
  ack.m_payloadBytes = m_reviewedAckPayloadBytes;
  ++m_reviewedAckSent;
  TracePacket("reviewed_ack_tx", s.packet, s.ddnIndex, s.sinkIndex,
              h.m_attempt, -1.0, "leg=" + std::to_string(s.leg));
  m_reviewedApps.at(s.receiver)->SendReviewed(ack);
}

void
AuvEdcExperiment::EndReviewedLeg(uint64_t token, bool confirmed, bool expired)
{
  auto& s = m_reviewedLegs.at(token);
  if (s.terminal) return;
  NS_ABORT_MSG_IF(confirmed && !s.arrived, "ACK cannot confirm data never received");
  s.timeout.Cancel();
  s.terminal = true; s.confirmed = confirmed; s.expired = expired;
  if (confirmed)
    {
      ++m_reviewedConfirmed;
      if (s.leg == 1) ++m_stats.acousticAcks;
      else if (s.leg == 2) ++m_stats.directAcks;
      else if (s.leg == 3) ++m_stats.mobileAcks;
      else ++m_stats.auvSurfaceAcks;
    }
  else if (s.arrived) ++m_reviewedUnconfirmedReceived;
  else if (expired)
    {
      RecordDeadlineExpiry(s.packet);
      if (s.leg == 1) ++m_reviewedUpstreamExpired;
    }
  else if (s.leg == 1) ++m_stats.acousticRetryExhausted;
  else ++m_reviewedRetryDropped;
  TracePacket(confirmed ? "reviewed_ack_received" : "reviewed_leg_terminal",
      s.packet, s.ddnIndex, s.sinkIndex, s.attempts, -1.0,
      confirmed ? "confirmed" : s.arrived ? "received_but_unconfirmed" :
      expired ? "deadline_expired" : "lifetime_attempts_exhausted");
  // Receiver commitment is already recorded at DATA arrival. Only now does
  // the sender release its retained copy; a failed ACK never duplicates a
  // delivered packet or turns an already committed record into a drop.
  if (s.leg == 1) m_pendingAcoustic.erase(s.packet.id);
  else if (s.leg == 2)
    { m_pendingDirect.erase(s.packet.id); FinishDirectPacket(s.ddnIndex, s.packet.id); }
  else if (s.leg == 3) FinishMobilePacket(s.ddnIndex, s.packet.id, false);
  else
    { ReviewedMissionRelease(s.packet.id); m_pendingAuvSurface.erase(s.packet.id); }
}

uint64_t
AuvEdcExperiment::ReviewedCommittedPending(uint8_t leg) const
{
  uint64_t count = 0;
  for (const auto& entry : m_reviewedLegs)
    if (!entry.second.terminal && entry.second.arrived &&
        (leg == 0 || entry.second.leg == leg)) ++count;
  return count;
}

// RealEnqueueAtDdn — callback invoked when a packet genuinely arrives at a DDN
// (or a wired DRN in contribution mode) through the AquaSim acoustic channel.
// ─────────────────────────────────────────────────────────────────────────────
void
AuvEdcExperiment::RealEnqueueAtDdn(uint64_t pktId,
                                    uint32_t srcBsnNs3Id,
                                    uint8_t  prio,
                                    uint8_t  hops,
                                    uint8_t  attempt,
                                    uint32_t payloadBytes,
                                    uint32_t ddnNs3Id)
{
  uint32_t ddnIdx = ddnNs3Id - m_numBsn - m_numDrn;
  if (ddnIdx >= m_numDdn) return; // sanity guard

  auto pending = m_pendingAcoustic.find(pktId);
  if (pending == m_pendingAcoustic.end() || pending->second.attempts != attempt)
    return; // late duplicate or ACK for an already completed logical packet

  pending->second.timeout.Cancel();
  DataPacket packet = pending->second.packet;
  m_pendingAcoustic.erase(pending);
  m_stats.acousticAcks++;
  TracePacket("upstream_ack", packet, static_cast<int32_t>(ddnIdx), -1,
              attempt, -1.0, "delivered");

  // Charge a 16-byte acknowledgement energy term at the endpoint and source.
  // The acknowledgement is idealized: no reverse packet, delay, or loss is
  // simulated. DATA loss/retry is handled by AquaSim and the source timeout.
  constexpr uint32_t ACK_BYTES = 16;
  const double ackDistance = std::abs(m_bsns[srcBsnNs3Id].position.x -
                                      m_ddns[ddnIdx].position.x);
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticTxEnergyJ(ACK_BYTES, ackDistance),
                     NodeRole::DDN);
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticRxEnergyJ(ACK_BYTES),
                     NodeRole::BSN);

  EnqueueAtDdn(ddnIdx, packet);
}

void
AuvEdcExperiment::StartAcousticAttempt(uint64_t packetId)
{
  auto it = m_pendingAcoustic.find(packetId);
  if (it == m_pendingAcoustic.end()) return;
  PendingAcousticPacket& pending = it->second;
  if (m_reviewedTransport)
    {
      StartReviewedLeg(pending.packet, 1, pending.packet.sourceBsn,
                       ReviewedUpstreamReceiver(pending.ddnIndex), pending.ddnIndex, 0);
      return;
    }
  pending.attempts++;
  m_stats.acousticSourceAttempts++;
  if (pending.attempts > 1) m_stats.acousticRetransmissions++;

  const DataPacket& packet = pending.packet;
  TracePacket("upstream_tx", packet, static_cast<int32_t>(pending.ddnIndex), -1,
              pending.attempts, -1.0, "attempt");
  const double sourceTxComplete = m_bsnFwdApps.at(packet.sourceBsn)->SendPkt(
      packet.id, static_cast<uint8_t>(packet.priority),
      packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + (m_closureMode
                  ? std::max(0.0, sourceTxComplete - Simulator::Now().GetSeconds())
                  : AcousticRecordDurationS(packet.sizeBytes))),
      &AuvEdcExperiment::AcousticAttemptTimeout, this, packetId, pending.attempts);
}

void
AuvEdcExperiment::AcousticAttemptTimeout(uint64_t packetId, uint8_t attempt)
{
  auto it = m_pendingAcoustic.find(packetId);
  if (it == m_pendingAcoustic.end() || it->second.attempts != attempt) return;
  if (it->second.attempts <= m_acousticMaxRetransmissions)
    {
      const double backoff = m_acousticRetryBackoff * m_retryRng->GetValue(0.5, 1.5);
      Simulator::Schedule(Seconds(backoff),
                          &AuvEdcExperiment::StartAcousticAttempt, this, packetId);
      return;
    }

  m_stats.acousticRetryExhausted++;
  TracePacket("upstream_exhausted", it->second.packet,
              static_cast<int32_t>(it->second.ddnIndex), -1,
              it->second.attempts, -1.0, "dropped");
  m_pendingAcoustic.erase(it);
}

NodeRole
AuvEdcExperiment::NodeRoleFromNs3Id(uint32_t nodeId) const
{
  if (nodeId < m_numBsn) return NodeRole::BSN;
  if (nodeId < m_numBsn + m_numDrn) return NodeRole::DRN;
  if (nodeId < m_numBsn + m_numDrn + m_numDdn) return NodeRole::DDN;
  if (nodeId < m_numBsn + m_numDrn + m_numDdn + m_sinks) return NodeRole::SINK;
  return NodeRole::AUV;
}

void
AuvEdcExperiment::QueueDirectAcoustic(uint32_t ddnIndex, const DataPacket& packet)
{
  if (m_pendingDirect.count(packet.id)) return;
  uint32_t nearestSink = 0;
  double bestDistance = std::numeric_limits<double>::max();
  for (uint32_t s = 0; s < m_sinkNodes.size(); ++s)
    {
      double dx = m_sinkNodes[s].position.x - m_ddns[ddnIndex].position.x;
      double dz = m_sinkNodes[s].position.z - m_ddns[ddnIndex].position.z;
      double distance = std::hypot(dx, dz);
      if (distance < bestDistance)
        { bestDistance = distance; nearestSink = s; }
    }
  uint32_t sinkNodeId = m_numBsn + m_numDrn + m_numDdn + nearestSink;
  m_pendingDirect.emplace(packet.id,
      PendingDirectPacket{packet, ddnIndex, sinkNodeId, 0, EventId()});
  m_directWaiting[ddnIndex].push_back(packet.id);
  StartNextDirectPacket(ddnIndex);
}

void
AuvEdcExperiment::StartNextDirectPacket(uint32_t ddnIndex)
{
  if (m_directActive[ddnIndex] || m_directWaiting[ddnIndex].empty()) return;
  m_directActive[ddnIndex] = true;
  StartDirectAttempt(m_directWaiting[ddnIndex].front());
}

void
AuvEdcExperiment::FinishDirectPacket(uint32_t ddnIndex, uint64_t packetId)
{
  if (m_reviewedIngress)
    {
      const auto token = m_reviewedLegTokens.find({packetId, 2});
      const bool receiverCommitted = token != m_reviewedLegTokens.end() &&
          m_reviewedLegs.at(token->second).arrived;
      // This hook releases the sender's physical outbox copy at ACK/local
      // termination, not at receiver DATA arrival. A late receiver recovery
      // does not re-create or re-release this sender allocation.
      ReviewedIngressRelease(ddnIndex, packetId, receiverCommitted
          ? closure_ingress::Removal::Forwarded : closure_ingress::Removal::Dropped);
    }
  if (!m_directWaiting[ddnIndex].empty() &&
      m_directWaiting[ddnIndex].front() == packetId)
    m_directWaiting[ddnIndex].pop_front();
  else
    {
      auto& queue = m_directWaiting[ddnIndex];
      queue.erase(std::remove(queue.begin(), queue.end(), packetId), queue.end());
    }
  m_directActive[ddnIndex] = false;
  StartNextDirectPacket(ddnIndex);
}

void
AuvEdcExperiment::StartDirectAttempt(uint64_t packetId)
{
  auto it = m_pendingDirect.find(packetId);
  if (it == m_pendingDirect.end()) return;
  if (m_reviewedTransport)
    {
      const auto& p = it->second;
      StartReviewedLeg(p.packet, 2, m_numBsn + m_numDrn + p.ddnIndex,
          p.sinkNodeId, p.ddnIndex, p.sinkNodeId - m_numBsn - m_numDrn - m_numDdn);
      return;
    }
  if (DeadlineExpired(it->second.packet))
    {
      ExpireDirectPacket(packetId, "before_attempt");
      return;
    }
  PendingDirectPacket& pending = it->second;
  pending.attempts++;
  m_stats.directSourceAttempts++;
  if (pending.attempts > 1) m_stats.directRetransmissions++;
  const DataPacket& packet = pending.packet;
  const uint32_t firstSinkNodeId = m_numBsn + m_numDrn + m_numDdn;
  const int32_t sinkIndex = static_cast<int32_t>(pending.sinkNodeId - firstSinkNodeId);
  const double directDistance = CalculateDistance(
      m_ddns[pending.ddnIndex].position, m_sinkNodes.at(sinkIndex).position);
  TracePacket("direct_tx", packet, static_cast<int32_t>(pending.ddnIndex), sinkIndex,
              pending.attempts, directDistance, "attempt");
  const double sourceTxComplete = m_ddnSinkApps.at(pending.ddnIndex)->SendPkt(
      packet.id, static_cast<uint8_t>(packet.priority),
      packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + (m_closureMode
                  ? std::max(0.0, sourceTxComplete - Simulator::Now().GetSeconds())
                  : AcousticRecordDurationS(packet.sizeBytes))),
      &AuvEdcExperiment::DirectAttemptTimeout, this, packetId, pending.attempts);
}

void
AuvEdcExperiment::ExpireDirectPacket(uint64_t packetId,
                                     const std::string& reason)
{
  auto it = m_pendingDirect.find(packetId);
  if (it == m_pendingDirect.end()) return;
  it->second.timeout.Cancel();
  const uint32_t ddnIndex = it->second.ddnIndex;
  const uint32_t firstSinkNodeId = m_numBsn + m_numDrn + m_numDdn;
  const int32_t sinkIndex = static_cast<int32_t>(
      it->second.sinkNodeId - firstSinkNodeId);
  const double directDistance = CalculateDistance(
      m_ddns[ddnIndex].position, m_sinkNodes.at(sinkIndex).position);
  const DataPacket packet = it->second.packet;
  if (!m_diagnosticsCsv.empty() && packet.priority == Priority::MEDIUM &&
      reason == "before_attempt")
    m_gatewayDiagnostics.at(ddnIndex).mediumExpiredBeforeAttempt++;
  TracePacket("deadline_expired", packet, static_cast<int32_t>(ddnIndex),
              sinkIndex, it->second.attempts, directDistance, reason);
  m_pendingDirect.erase(it);
  RecordDeadlineExpiry(packet);
  FinishDirectPacket(ddnIndex, packetId);
}

void
AuvEdcExperiment::DirectAttemptTimeout(uint64_t packetId, uint8_t attempt)
{
  auto it = m_pendingDirect.find(packetId);
  if (it == m_pendingDirect.end() || it->second.attempts != attempt) return;
  if (DeadlineExpired(it->second.packet))
    {
      ExpireDirectPacket(packetId, "after_attempt");
      return;
    }
  if (it->second.attempts <= m_acousticMaxRetransmissions)
    {
      double backoff = m_acousticRetryBackoff * m_retryRng->GetValue(0.5, 1.5);
      Simulator::Schedule(Seconds(backoff),
                          &AuvEdcExperiment::StartDirectAttempt, this, packetId);
      return;
    }
  m_stats.directCyclesExhausted++;
  const uint32_t firstSinkNodeId = m_numBsn + m_numDrn + m_numDdn;
  const int32_t sinkIndex = static_cast<int32_t>(it->second.sinkNodeId - firstSinkNodeId);
  const double directDistance = CalculateDistance(
      m_ddns[it->second.ddnIndex].position, m_sinkNodes.at(sinkIndex).position);
  DataPacket packet = it->second.packet;
  uint32_t ddnIndex = it->second.ddnIndex;
  const bool expired = DeadlineExpired(packet);
  TracePacket(expired ? "deadline_expired" : "direct_cycle_exhausted", packet,
              static_cast<int32_t>(ddnIndex), sinkIndex,
              it->second.attempts, directDistance,
              expired ? "dropped" : "requeued");
  m_pendingDirect.erase(it);
  if (expired)
    RecordDeadlineExpiry(packet);
  else
    RequeueFrontDdnPacket(ddnIndex, packet);
  FinishDirectPacket(ddnIndex, packetId);
}

void
AuvEdcExperiment::RealDeliverToSink(uint64_t pktId,
                                    uint32_t srcBsnNs3Id,
                                    uint8_t prio,
                                    uint8_t hops,
                                    uint8_t attempt,
                                    uint32_t payloadBytes,
                                    uint32_t)
{
  auto it = m_pendingDirect.find(pktId);
  if (it == m_pendingDirect.end() || it->second.attempts != attempt) return;
  it->second.timeout.Cancel();
  DataPacket packet = it->second.packet;
  uint32_t ddnIndex = it->second.ddnIndex;
  uint32_t sinkNodeId = it->second.sinkNodeId;
  m_pendingDirect.erase(it);
  m_stats.directAcks++;

  constexpr uint32_t ACK_BYTES = 16;
  const uint32_t firstSinkNodeId = m_numBsn + m_numDrn + m_numDdn;
  if (sinkNodeId < firstSinkNodeId || sinkNodeId >= firstSinkNodeId + m_sinks)
    return;
  const uint32_t sinkIndex = sinkNodeId - firstSinkNodeId;
  const double ackDistance =
      CalculateDistance(m_sinkNodes[sinkIndex].position, m_ddns[ddnIndex].position);
  (void)srcBsnNs3Id; // For direct packets the header source is the sending DDN.
  TracePacket("direct_ack", packet, static_cast<int32_t>(ddnIndex),
              static_cast<int32_t>(sinkIndex), attempt, ackDistance, "delivered");
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticTxEnergyJ(ACK_BYTES, ackDistance),
                     NodeRole::SINK);
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticRxEnergyJ(ACK_BYTES),
                     NodeRole::DDN);

  double e2eDelay = Simulator::Now().GetSeconds() - packet.createdAt;
  m_stats.deliveredToSink++;
  m_stats.surfaceDelivered++;
  m_stats.surfaceDelaySum += e2eDelay;
  m_stats.delaySum += e2eDelay;
  if (packet.priority == Priority::HIGH) m_stats.directCriticalToSink++;
  else if (packet.priority == Priority::MEDIUM) m_stats.directMediumToSink++;
  else m_stats.directLowToSink++;
  AccountDeliveredPriority(packet.priority, e2eDelay, packet.burst, packet.sizeBytes);
  FinishDirectPacket(ddnIndex, pktId);
}

void
AuvEdcExperiment::QueueMobileAcoustic(uint32_t ddnIndex, const DataPacket& packet)
{
  if (m_pendingMobile.count(packet.id)) return;
  m_pendingMobile.emplace(packet.id,
      PendingMobilePacket{packet, ddnIndex, 0, EventId()});
  m_mobileWaiting[ddnIndex].push_back(packet.id);
  StartNextMobilePacket(ddnIndex);
}

void
AuvEdcExperiment::StartNextMobilePacket(uint32_t ddnIndex)
{
  if (m_mobileActive[ddnIndex] || m_mobileWaiting[ddnIndex].empty()) return;
  m_mobileActive[ddnIndex] = true;
  StartMobileAcousticAttempt(m_mobileWaiting[ddnIndex].front());
}

void
AuvEdcExperiment::StartMobileAcousticAttempt(uint64_t packetId)
{
  auto it = m_pendingMobile.find(packetId);
  if (it == m_pendingMobile.end()) return;
  PendingMobilePacket& pending = it->second;
  if (m_reviewedTransport)
    {
      StartReviewedLeg(pending.packet, 3, m_numBsn + m_numDrn + pending.ddnIndex,
          m_numBsn + m_numDrn + m_numDdn + m_sinks, pending.ddnIndex, 0);
      return;
    }
  const double distance = CalculateDistance(
      m_ddns[pending.ddnIndex].position, m_auvMobility->GetPosition());
  if (distance > m_ddnCollectionRange)
    {
      m_stats.mobileContactFailures++;
      TracePacket("mobile_acoustic_contact_end", pending.packet,
                  static_cast<int32_t>(pending.ddnIndex), -1,
                  pending.attempts, distance, "requeued");
      FinishMobilePacket(pending.ddnIndex, packetId, true);
      return;
    }

  pending.attempts++;
  m_stats.mobileSourceAttempts++;
  if (pending.attempts > 1) m_stats.mobileRetransmissions++;
  TracePacket("mobile_acoustic_tx", pending.packet,
              static_cast<int32_t>(pending.ddnIndex), -1,
              pending.attempts, distance, "attempt");
  const double sourceTxComplete = m_ddnAuvApps.at(pending.ddnIndex)->SendPkt(
      pending.packet.id, static_cast<uint8_t>(pending.packet.priority),
      pending.packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + (m_closureMode
                  ? std::max(0.0, sourceTxComplete - Simulator::Now().GetSeconds())
                  : AcousticRecordDurationS(pending.packet.sizeBytes))),
      &AuvEdcExperiment::MobileAcousticAttemptTimeout,
      this, packetId, pending.attempts);
}

void
AuvEdcExperiment::MobileAcousticAttemptTimeout(uint64_t packetId, uint8_t attempt)
{
  auto it = m_pendingMobile.find(packetId);
  if (it == m_pendingMobile.end() || it->second.attempts != attempt) return;
  const double distance = CalculateDistance(
      m_ddns[it->second.ddnIndex].position, m_auvMobility->GetPosition());
  if (it->second.attempts <= m_acousticMaxRetransmissions &&
      distance <= m_ddnCollectionRange)
    {
      const double backoff = m_acousticRetryBackoff * m_retryRng->GetValue(0.5, 1.5);
      Simulator::Schedule(Seconds(backoff),
                          &AuvEdcExperiment::StartMobileAcousticAttempt,
                          this, packetId);
      return;
    }

  m_stats.mobileContactFailures++;
  TracePacket("mobile_acoustic_timeout", it->second.packet,
              static_cast<int32_t>(it->second.ddnIndex), -1,
              it->second.attempts, distance, "requeued");
  FinishMobilePacket(it->second.ddnIndex, packetId, true);
}

void
AuvEdcExperiment::FinishMobilePacket(uint32_t ddnIndex,
                                     uint64_t packetId,
                                     bool requeue)
{
  auto pending = m_pendingMobile.find(packetId);
  if (pending != m_pendingMobile.end())
    {
      pending->second.timeout.Cancel();
      if (requeue) RequeueFrontDdnPacket(ddnIndex, pending->second.packet);
      m_pendingMobile.erase(pending);
    }
  if (m_reviewedIngress && !requeue)
    {
      const auto token = m_reviewedLegTokens.find({packetId, 3});
      const bool receiverCommitted = token != m_reviewedLegTokens.end() &&
          m_reviewedLegs.at(token->second).arrived;
      ReviewedIngressRelease(ddnIndex, packetId, receiverCommitted
          ? closure_ingress::Removal::Forwarded : closure_ingress::Removal::Dropped);
    }
  auto& queue = m_mobileWaiting[ddnIndex];
  if (!queue.empty() && queue.front() == packetId) queue.pop_front();
  else queue.erase(std::remove(queue.begin(), queue.end(), packetId), queue.end());
  m_mobileActive[ddnIndex] = false;
  StartNextMobilePacket(ddnIndex);
}

void
AuvEdcExperiment::RealCollectByAuv(uint64_t pktId,
                                    uint32_t srcDdnNs3Id,
                                    uint8_t prio,
                                    uint8_t,
                                    uint8_t attempt,
                                    uint32_t,
                                    uint32_t)
{
  auto it = m_pendingMobile.find(pktId);
  if (it == m_pendingMobile.end() || it->second.attempts != attempt) return;
  const uint32_t expectedDdnNodeId = m_numBsn + m_numDrn + it->second.ddnIndex;
  if (srcDdnNs3Id != expectedDdnNodeId) return;
  DataPacket packet = it->second.packet;
  const uint32_t ddnIndex = it->second.ddnIndex;
  const double distance = CalculateDistance(
      m_ddns[ddnIndex].position, m_auvMobility->GetPosition());
  m_stats.mobileAcks++;
  m_stats.collectedByAuv++;
  m_auvBuffer.push_back(packet);
  constexpr uint32_t ACK_BYTES = 16;
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticTxEnergyJ(ACK_BYTES, distance),
                     NodeRole::AUV);
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticRxEnergyJ(ACK_BYTES),
                     NodeRole::DDN);
  TracePacket("mobile_acoustic_ack", packet, static_cast<int32_t>(ddnIndex),
              -1, attempt, distance, "collected");
  FinishMobilePacket(ddnIndex, pktId, false);
}

void
AuvEdcExperiment::QueueAuvSurface(const DataPacket& packet, uint32_t sinkIndex)
{
  if (!m_pendingAuvSurface.empty() || sinkIndex >= m_sinkNodes.size())
    {
      m_auvBuffer.push_front(packet);
      return;
    }
  m_pendingAuvSurface.emplace(packet.id,
      PendingAuvSurfacePacket{packet, sinkIndex, 0, EventId()});
  StartAuvSurfaceAttempt(packet.id);
}

void
AuvEdcExperiment::StartAuvSurfaceAttempt(uint64_t packetId)
{
  auto it = m_pendingAuvSurface.find(packetId);
  if (it == m_pendingAuvSurface.end()) return;
  PendingAuvSurfacePacket& pending = it->second;
  if (m_reviewedTransport)
    {
      StartReviewedLeg(pending.packet, 4, m_numBsn + m_numDrn + m_numDdn + m_sinks,
          m_numBsn + m_numDrn + m_numDdn + pending.sinkIndex, 0, pending.sinkIndex);
      return;
    }
  const double distance = CalculateDistance(
      m_auvMobility->GetPosition(), m_sinkNodes[pending.sinkIndex].position);
  if (distance > m_sinkRange)
    {
      m_stats.auvSurfaceContactFailures++;
      TracePacket("auv_sink_contact_end", pending.packet, -1,
                  static_cast<int32_t>(pending.sinkIndex), pending.attempts,
                  distance, "requeued");
      m_auvBuffer.push_front(pending.packet);
      m_pendingAuvSurface.erase(it);
      return;
    }

  pending.attempts++;
  m_stats.auvSurfaceSourceAttempts++;
  if (pending.attempts > 1) m_stats.auvSurfaceRetransmissions++;
  TracePacket("auv_sink_tx", pending.packet, -1,
              static_cast<int32_t>(pending.sinkIndex), pending.attempts,
              distance, "attempt");
  const double sourceTxComplete = m_auvSinkApps.at(pending.sinkIndex)->SendPkt(
      pending.packet.id, static_cast<uint8_t>(pending.packet.priority),
      pending.packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + (m_closureMode
                  ? std::max(0.0, sourceTxComplete - Simulator::Now().GetSeconds())
                  : AcousticRecordDurationS(pending.packet.sizeBytes))),
      &AuvEdcExperiment::AuvSurfaceAttemptTimeout,
      this, packetId, pending.attempts);
}

void
AuvEdcExperiment::AuvSurfaceAttemptTimeout(uint64_t packetId, uint8_t attempt)
{
  auto it = m_pendingAuvSurface.find(packetId);
  if (it == m_pendingAuvSurface.end() || it->second.attempts != attempt) return;
  const double distance = CalculateDistance(
      m_auvMobility->GetPosition(), m_sinkNodes[it->second.sinkIndex].position);
  if (it->second.attempts <= m_acousticMaxRetransmissions &&
      distance <= m_sinkRange)
    {
      const double backoff = m_acousticRetryBackoff * m_retryRng->GetValue(0.5, 1.5);
      Simulator::Schedule(Seconds(backoff),
                          &AuvEdcExperiment::StartAuvSurfaceAttempt,
                          this, packetId);
      return;
    }

  m_stats.auvSurfaceContactFailures++;
  DataPacket packet = it->second.packet;
  const bool expired = DeadlineExpired(packet);
  TracePacket(expired ? "deadline_expired" : "auv_sink_timeout", packet, -1,
              static_cast<int32_t>(it->second.sinkIndex), it->second.attempts,
              distance, expired ? "dropped" : "requeued");
  if (expired)
    RecordDeadlineExpiry(packet);
  else
    m_auvBuffer.push_front(packet);
  m_pendingAuvSurface.erase(it);
}

void
AuvEdcExperiment::RealAuvDeliverToSink(uint64_t pktId,
                                       uint32_t srcAuvNs3Id,
                                       uint8_t prio,
                                       uint8_t,
                                       uint8_t attempt,
                                       uint32_t,
                                       uint32_t)
{
  auto it = m_pendingAuvSurface.find(pktId);
  if (it == m_pendingAuvSurface.end() || it->second.attempts != attempt) return;
  const uint32_t expectedAuvNodeId = m_numBsn + m_numDrn + m_numDdn + m_sinks;
  if (srcAuvNs3Id != expectedAuvNodeId) return;
  it->second.timeout.Cancel();
  DataPacket packet = it->second.packet;
  const uint32_t sinkIndex = it->second.sinkIndex;
  const double distance = CalculateDistance(
      m_auvMobility->GetPosition(), m_sinkNodes[sinkIndex].position);
  m_pendingAuvSurface.erase(it);
  m_stats.auvSurfaceAcks++;

  constexpr uint32_t ACK_BYTES = 16;
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticTxEnergyJ(ACK_BYTES, distance),
                     NodeRole::SINK);
  ChargePacketEnergy(static_cast<Priority>(prio),
                     AcousticRxEnergyJ(ACK_BYTES),
                     NodeRole::AUV);
  const double e2eDelay = Simulator::Now().GetSeconds() - packet.createdAt;
  m_stats.surfaceDelivered++;
  m_stats.surfaceDelaySum += e2eDelay;
  m_stats.deliveredToSink++;
  m_stats.delaySum += e2eDelay;
  AccountDeliveredPriority(packet.priority, e2eDelay, packet.burst, packet.sizeBytes);
  TracePacket("auv_sink_ack", packet, -1, static_cast<int32_t>(sinkIndex),
              attempt, distance, "delivered");
}

#include "reviewed_optical_service.inc"

void
AuvEdcExperiment::StartOpticalTransfer(uint32_t ddnIndex,
                                       const DataPacket& packet,
                                       double distance,
                                       double successProbability)
{
  if (m_pendingOptical.count(packet.id)) return;
  if (m_reviewedPhyEnergy)
    {
      const auto gateway=m_numBsn+m_numDrn+ddnIndex;
      const auto auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
      m_reviewedPhys.at(gateway)->Sync(); m_reviewedPhys.at(auv)->Sync();
      if(m_reviewedLedgers.at(gateway)->Budget()->Remaining()<=0 ||
         m_reviewedLedgers.at(auv)->Budget()->Remaining()<=0)
        { ReviewedMissionRelease(packet.id); RequeueFrontDdnPacket(ddnIndex,packet); return; }
    }
  m_opticalActive.at(ddnIndex) = true;
  const double txDuration =
      static_cast<double>(packet.sizeBytes * 8) / m_opticalDataRateBps;
  if (m_reviewedPhyEnergy)
    {
      m_reviewedPhys.at(m_numBsn+m_numDrn+ddnIndex)->AddExternalLoad(
        std::max(txDuration,1e-9),m_opticalTxPowerW);
      m_reviewedPhys.at(m_numBsn+m_numDrn+m_numDdn+m_sinks)->AddExternalLoad(
        std::max(txDuration,1e-9),m_opticalRxPowerW);
    }
  m_pendingOptical.emplace(packet.id,
      PendingOpticalPacket{packet, ddnIndex, distance, successProbability,
                          Simulator::Now().GetSeconds(),
                          std::max(txDuration, 1e-9), 0.0});
  Simulator::Schedule(Seconds(std::max(txDuration, 1e-9)),
                      &AuvEdcExperiment::CompleteOpticalTransfer,
                      this, packet.id);
}

void
AuvEdcExperiment::ChargeOpticalAttemptEnergy(PendingOpticalPacket& pending,
                                             double observedAt,
                                             bool completed)
{
  if (pending.reviewedWire) return; // Authoritative elapsed PHY-owner loads already account this service.
  if (pending.durationSeconds <= 0.0) return;
  // The scheduled emitter and receiver window remains active for the frame,
  // including PER failures and contact-ending failures. This does not model
  // a modem's measured early-abort, wakeup, acquisition or listening behavior.
  // Completed callbacks settle the full scheduled frame; a terminal horizon
  // settles only elapsed time. Tracking the already charged interval makes
  // repeated finalization and partial-then-complete settlement idempotent.
  const double elapsed = completed ? pending.durationSeconds
      : std::min(pending.durationSeconds,
                 std::max(0.0, observedAt - pending.startedAt));
  const double delta = std::max(0.0, elapsed - pending.chargedSeconds);
  if (delta <= 0.0) return;
  pending.chargedSeconds += delta;
  if (m_energyModel == "hardware")
    {
      ChargePacketEnergy(pending.packet.priority, m_opticalTxPowerW * delta,
                         NodeRole::DDN);
      ChargePacketEnergy(pending.packet.priority, m_opticalRxPowerW * delta,
                         NodeRole::AUV);
    }
  else
    {
      const double fullFrameEnergyJ = 0.001 + pending.packet.sizeBytes *
          pending.txDistance * m_opticalEnergyPerByteMeter;
      ChargePacketEnergySplit(pending.packet.priority,
                              fullFrameEnergyJ * delta / pending.durationSeconds,
                              NodeRole::DDN, 0.5, NodeRole::AUV);
    }
}

void
AuvEdcExperiment::SettlePendingOpticalEnergy(double observedAt)
{
  for (auto& entry : m_pendingOptical)
    ChargeOpticalAttemptEnergy(entry.second, observedAt);
}

void
AuvEdcExperiment::TryStartOpticalTransfer(uint32_t ddnIndex)
{
  if (m_reviewedOpticalControl) { ReviewedOpticalWake(ddnIndex); return; }
  if (m_reviewedMission && !m_pendingOptical.empty()) return;
  if (ddnIndex >= m_numDdn || m_opticalActive.at(ddnIndex)) return;
  if (!ReviewedMissionCommunicationsAvailable()) return;
  const double now = Simulator::Now().GetSeconds();
  const Vector auvPos = GetAuvPosition(now);
  const double distance = CalculateDistance(auvPos, m_ddns[ddnIndex].position);
  if (distance > m_opticalRange) return;

  DataPacket packet;
  if (!PopNextDdnPacket(ddnIndex, packet)) return;
  if (!ReviewedMissionReserve(packet))
    {
      RequeueFrontDdnPacket(ddnIndex, packet);
      return;
    }
  double snrDb = 0.0;
  double ber = 0.0;
  double pointingGain = 1.0;
  const double successProbability = OpticalPacketSuccessProbability(
      ddnIndex, distance, packet.sizeBytes, snrDb, ber, pointingGain);
  m_stats.opticalContactSamples++;
  m_stats.opticalSuccessProbSum += successProbability;
  m_stats.opticalSnrDbSum += snrDb;
  m_stats.opticalAttemptedPackets++;
  if (!m_diagnosticsCsv.empty())
    {
      auto& diagnostic = m_gatewayDiagnostics.at(ddnIndex);
      diagnostic.opticalAttempts++;
      diagnostic.contactAttempts++;
    }
  TracePacket("optical_tx", packet, static_cast<int32_t>(ddnIndex), -1, 1,
              distance, "attempt");
  StartOpticalTransfer(ddnIndex, packet, distance, successProbability);
}

void
AuvEdcExperiment::CompleteOpticalTransfer(uint64_t packetId)
{
  auto it = m_pendingOptical.find(packetId);
  if (it == m_pendingOptical.end()) return;
  ChargeOpticalAttemptEnergy(it->second, Simulator::Now().GetSeconds(), true);
  PendingOpticalPacket pending = it->second;
  m_pendingOptical.erase(it);
  m_opticalActive.at(pending.ddnIndex) = false;
  const Vector rxPosition = GetAuvPosition(Simulator::Now().GetSeconds());
  const double rxDistance = CalculateDistance(
      rxPosition, m_ddns[pending.ddnIndex].position);
  bool poweredThroughFrame = true;
  if (m_reviewedPhyEnergy)
    {
      const auto gateway=m_numBsn+m_numDrn+pending.ddnIndex;
      const auto auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
      m_reviewedPhys.at(gateway)->Sync(); m_reviewedPhys.at(auv)->Sync();
      const double end=pending.startedAt+pending.durationSeconds;
      poweredThroughFrame=m_reviewedLedgers.at(gateway)->DepletedAt()+1e-9>=end &&
                          m_reviewedLedgers.at(auv)->DepletedAt()+1e-9>=end;
    }
  const bool contactHeld = rxDistance <= m_opticalRange && poweredThroughFrame;
  // The distance to a fixed gateway is speed-Lipschitz. This conservative
  // whole-frame bound rules out leaving/reentering the gate between endpoints;
  // it is not merely an end-position sample of a curved moving contact.
  const bool continuousContact = !m_reviewedMission ||
      (pending.txDistance + rxDistance + closure_mission::fixedThreeKnotsMps * pending.durationSeconds) / 2.0 <= m_opticalRange + 1e-9;
  const bool received = contactHeld && continuousContact && ReviewedMissionCommunicationsAvailable() &&
      m_opticalRng->GetValue() <= pending.successProbability;
  if (!received)
    {
      ReviewedMissionRelease(pending.packet.id);
      m_stats.opticalLostPackets++;
      if (!m_diagnosticsCsv.empty())
        {
          auto& diagnostic = m_gatewayDiagnostics.at(pending.ddnIndex);
          if (contactHeld && continuousContact) diagnostic.opticalLostPer++;
          else diagnostic.opticalLostContact++;
        }
      TracePacket("optical_loss", pending.packet,
                  static_cast<int32_t>(pending.ddnIndex), -1, 1,
                  rxDistance, contactHeld && continuousContact ? "requeued_per" : "requeued_contact_end");
      RequeueFrontDdnPacket(pending.ddnIndex, pending.packet);
      return;
    }

  m_auvBuffer.push_back(pending.packet);
  ReviewedMissionCommit(pending.packet.id, pending.ddnIndex);
  ReviewedIngressRelease(pending.ddnIndex, pending.packet.id,
                          closure_ingress::Removal::Forwarded);
  m_stats.collectedByAuv++;
  m_stats.opticalTransfers++;
  if (!m_diagnosticsCsv.empty())
    {
      auto& diagnostic = m_gatewayDiagnostics.at(pending.ddnIndex);
      diagnostic.opticalCollected++;
      diagnostic.contactCollected++;
      diagnostic.opticalBytes += pending.packet.sizeBytes;
    }
  TracePacket("optical_rx", pending.packet,
              static_cast<int32_t>(pending.ddnIndex), -1, 1,
              rxDistance, "collected");
  // Serialize the optical link: only after this packet's RX event may the
  // next queued packet begin, using the AUV's then-current 3-D position.
  TryStartOpticalTransfer(pending.ddnIndex);
}

void
AuvEdcExperiment::BuildLogicalTopology()
{
  m_bsns.clear();
  m_drns.clear();
  m_ddns.clear();
  m_sinkNodes.clear();

  if (m_protocol == ProtocolMode::BASELINE || m_protocol == ProtocolMode::MADCS_HP)
    {
      // Faithful Ahmed 2022 topology (Sec. III-A, Eq. 1): x = i * dm, i=0..N-1.
      // dm = L/N gives exact 100 m spacing for N=126 (12600/126=100 m) so DDNs
      // fall at exact multiples of 2500 m matching the paper's octet rule.
      const double dm = (m_totalNodes > 0)
                            ? m_pipelineLength / static_cast<double>(m_totalNodes)
                            : 100.0;
      for (uint32_t i = 0; i < m_totalNodes; ++i)
        {
          double x = static_cast<double>(i) * dm;
          if (i % 25 == 0)
            m_ddns.push_back({static_cast<uint32_t>(m_ddns.size()),
                               NodeRole::DDN, Vector(x, 0.0, m_depth), m_ddnRange});
          else if (i % 5 == 0)
            m_drns.push_back({static_cast<uint32_t>(m_drns.size()),
                               NodeRole::DRN, Vector(x, 0.0, m_depth), m_drnRange});
          else
            m_bsns.push_back({static_cast<uint32_t>(m_bsns.size()),
                               NodeRole::BSN, Vector(x, 0.0, m_depth), m_bsnRange});
        }
      m_numBsn = static_cast<uint32_t>(m_bsns.size());
      m_numDrn = static_cast<uint32_t>(m_drns.size());
      m_numDdn = static_cast<uint32_t>(m_ddns.size());
      // m_pipelineLength already set to 12600 m in Configure() — do NOT override.
    }
  else
    {
      // EOM topology — same interleaved pattern as BASELINE.
      // At each DDN position (i%25==0): DRN at pipeline depth (1000 m) as the acoustic
      // collector, plus DDN at ddnDepth (500 m) as the AUV-facing gateway connected via
      // vertical EOM cable.  Relay DRNs (i%5==0) forward via multi-hop toward the paired
      // DRN of their nearest DDN.
      //
      //  Pipeline (1000 m):  BSN BSN BSN BSN [relay-DRN] ... [relay-DRN] [paired-DRN]
      //                                                                     | EOM cable
      //  ddnDepth (500 m):                                                [DDN] → AUV
      double ddnDepth = std::max(0.0, m_depth - m_ddnEomOffset);
      const double dm = (m_totalNodes > 0)
                            ? m_pipelineLength / static_cast<double>(m_totalNodes)
                            : 100.0;
      for (uint32_t i = 0; i < m_totalNodes; ++i)
        {
          double x = static_cast<double>(i) * dm;
          if (i % 25 == 0)
            {
              // EOM station: paired DRN at pipeline depth + DDN above via cable
              m_drns.push_back({static_cast<uint32_t>(m_drns.size()),
                                 NodeRole::DRN, Vector(x, 0.0, m_depth),   m_drnRange});
              m_ddns.push_back({static_cast<uint32_t>(m_ddns.size()),
                                 NodeRole::DDN, Vector(x, 0.0, ddnDepth),  m_ddnRange});
            }
          else if (i % 5 == 0)
            m_drns.push_back({static_cast<uint32_t>(m_drns.size()),
                               NodeRole::DRN, Vector(x, 0.0, m_depth),    m_drnRange});
          else
            m_bsns.push_back({static_cast<uint32_t>(m_bsns.size()),
                               NodeRole::BSN, Vector(x, 0.0, m_depth),    m_bsnRange});
        }
      m_numBsn = static_cast<uint32_t>(m_bsns.size());
      m_numDrn = static_cast<uint32_t>(m_drns.size());
      m_numDdn = static_cast<uint32_t>(m_ddns.size());
    }

  {
      // Place each sink at the midpoint of an evenly-chosen DDN inter-node gap so
      // that the AUV always has a reachable sink when it ascends to the surface.
      // When numSinks == numGaps (baseline: 5 sinks, 5 gaps) every gap gets a sink.
      // When numSinks < numGaps (contribution with many DDNs) sinks are distributed
      // uniformly across the pipeline.
      std::vector<double> hybridPositions;
      if (m_sinkPlacement == "hybrid" && !m_ddns.empty())
        {
          for (uint32_t k = 0; k < m_numDdn; ++k)
            {
              hybridPositions.push_back(m_ddns[k].position.x);
              if (k + 1 < m_numDdn)
                hybridPositions.push_back(
                    0.5 * (m_ddns[k].position.x + m_ddns[k + 1].position.x));
            }
          if (m_sinks > hybridPositions.size())
            NS_FATAL_ERROR("hybrid sinkPlacement supports at most 2*numDdn-1 sinks");
        }
      for (uint32_t i = 0; i < m_sinks; ++i)
        {
          // Sink at midpoint of a DDN gap = where the AUV sinusoidal path surfaces.
          // With only 1 DDN (numGaps==0) the AUV surfaces at ddnSpan/2 from the DDN.
          double ddnSpan = (m_numDdn > 1)
                               ? m_ddns.back().position.x - m_ddns.front().position.x
                               : m_pipelineLength;
          uint32_t numGaps = (m_numDdn > 1) ? m_numDdn - 1 : 0;
          double x = (m_numDdn > 0 ? m_ddns.front().position.x : 0.0) + ddnSpan / 2.0;
          if (m_sinkPlacement == "ddn" && m_numDdn > 0)
            {
              uint32_t ddnIdx = (m_sinks > 1)
                  ? static_cast<uint32_t>(std::round(
                        i * static_cast<double>(m_numDdn - 1) /
                        static_cast<double>(m_sinks - 1)))
                  : m_numDdn / 2;
              ddnIdx = std::min(ddnIdx, m_numDdn - 1);
              x = m_ddns[ddnIdx].position.x;
            }
          else if (m_sinkPlacement == "hybrid" && !hybridPositions.empty())
            {
              uint32_t positionIdx = (m_sinks > 1)
                  ? static_cast<uint32_t>(std::round(
                        i * static_cast<double>(hybridPositions.size() - 1) /
                        static_cast<double>(m_sinks - 1)))
                  : static_cast<uint32_t>(hybridPositions.size() / 2);
              positionIdx = std::min(positionIdx,
                                     static_cast<uint32_t>(hybridPositions.size() - 1));
              x = hybridPositions[positionIdx];
            }
          else if (m_sinkPlacement == "uniform")
            {
              x = (m_sinks > 1)
                  ? i * m_pipelineLength / static_cast<double>(m_sinks - 1)
                  : 0.5 * m_pipelineLength;
            }
          else if (numGaps > 0)
            {
              uint32_t gapIdx = (m_sinks > 1)
                                    ? static_cast<uint32_t>(std::round(
                                          i * static_cast<double>(numGaps - 1) /
                                          static_cast<double>(m_sinks - 1)))
                                    : numGaps / 2;
              gapIdx = std::min(gapIdx, numGaps - 1);
              x = 0.5 * (m_ddns[gapIdx].position.x + m_ddns[gapIdx + 1].position.x);
            }
          m_sinkNodes.push_back({i, NodeRole::SINK, Vector(x, 0.0, 0.0),
                                 m_closureMode ? m_closureSinkRangeM : m_sinkRange});
        }
    }

  m_ddnBuffers.assign(m_numDdn, std::deque<DataPacket>());
  m_ddnHighBuffers.assign(m_numDdn, std::deque<DataPacket>());
  m_ddnMediumBuffers.assign(m_numDdn, std::deque<DataPacket>());
  m_ddnLowBuffers.assign(m_numDdn, std::deque<DataPacket>());
  m_directWaiting.assign(m_numDdn, std::deque<uint64_t>());
  m_directActive.assign(m_numDdn, false);
  m_mobileWaiting.assign(m_numDdn, std::deque<uint64_t>());
  m_mobileActive.assign(m_numDdn, false);
  m_opticalActive.assign(m_numDdn, false);
  m_opticalPointingLastDraw.assign(m_numDdn, -1.0);
  m_opticalPointingRadialOffset.assign(m_numDdn, 0.0);
  m_gatewayDiagnostics.assign(m_numDdn, GatewayDiagnostics());
  m_storageHighWaterBytes.assign(m_numDdn, 0);
  m_storageHighWaterRecords.assign(m_numDdn, 0);
  m_residentHighWaterBytes.assign(m_numDdn, 0);
  m_residentHighWaterRecords.assign(m_numDdn, 0);
  if (m_auvInstantaneousPower)
    {
      const double span = m_numDdn > 1
          ? m_ddns.back().position.x - m_ddns.front().position.x : m_pipelineLength;
      m_motionSpacing = m_numDdn > 1 ? span / (m_numDdn - 1) : span;
      m_motionContactDepth =
          (m_protocol == ProtocolMode::CONTRIBUTION || m_protocol == ProtocolMode::PURE_ACOUSTIC)
              ? std::max(0.0, m_depth - m_ddnEomOffset) : m_depth;
      m_motionHorizontalSpeed = (m_auvSpeedKmh / 3.6) /
          SinusoidalPathStretch(m_motionContactDepth, m_motionSpacing);
    }
  if (m_auvInstantaneousPower && AuvPeakSpeed() * 3.6 > m_auvMaxSpeedKmh + 1e-9)
    {
      NS_FATAL_ERROR("Actual sinusoidal peak speed " << AuvPeakSpeed() * 3.6
                     << " km/h exceeds auvMaxSpeedKmh=" << m_auvMaxSpeedKmh);
    }
}

void
AuvEdcExperiment::ScheduleTraffic()
{
  if (m_reviewedIngress) ReviewedIngressInitialize();
  if (m_workloadReplayCsv.empty())
    {
      Simulator::Schedule(Seconds(0.5), &AuvEdcExperiment::GeneratePacket, this);
    }
  else
    {
      NS_ABORT_MSG_IF(!UsesPriorityQueues() || !m_lowPayloadAtGateway || m_alarmBurstSize > 0,
                      "Replay requires matched priority queues, gateway inspection ingress, and no additional alarm injection");
      edc_replay::ValidationOptions options;
      options.stop_s = std::min(m_trafficStop, m_simStop);
      options.source_bsn_count = m_numBsn;
      options.ddn_count = m_numDdn;
      options.low_source_is_gateway_instrument = true;
      try
        {
          const auto workload = edc_replay::LoadCsv(m_workloadReplayCsv, options);
          if (m_reviewedIngress)
            {
              // Exogenous events remain private producer input. Only events
              // released at THIS instant reach the producer callback; no route
              // controller is given the Workload or its future-window index.
              std::map<double, std::vector<edc_replay::Event>> batches;
              for (const auto& event : workload.events) batches[event.release_s].push_back(event);
              for (const auto& batch : batches)
                {
                  Time release = Seconds(batch.first);
                  if (release.GetSeconds() < batch.first) release += NanoSeconds(1);
                  Simulator::Schedule(release, &AuvEdcExperiment::GenerateReviewedReplayBatch,
                                      this, batch.second);
                }
            }
          else for (const auto& event : workload.events)
            Simulator::Schedule(Seconds(event.release_s), &AuvEdcExperiment::GenerateReplayPacket, this, event);
          std::cout << "WORKLOAD_REPLAY records=" << workload.events.size()
                    << " windows=" << workload.windows.size()
                    << " path=" << m_workloadReplayCsv
                    << " ingress=" << (m_reviewedIngress ? "finite-dedicated-cable" : "ideal-boundary")
                    << " sourceBsnSentinel=" << std::numeric_limits<uint32_t>::max() << '\n';
        }
      catch (const std::exception& error)
        {
          NS_FATAL_ERROR("Invalid workload replay: " << error.what());
        }
    }
  if (m_alarmBurstSize > 0)
    Simulator::Schedule(Seconds(m_alarmBurstTime), &AuvEdcExperiment::InjectAlarmBurst, this);
  Simulator::Schedule(Seconds(0.0), &AuvEdcExperiment::AuvTick, this);
  if (!m_metricsCsv.empty())
    {
      WriteMetricsHeader();
      Simulator::Schedule(Seconds(m_metricSampleStart),
                          &AuvEdcExperiment::SampleMetrics, this);
    }
  if (!m_packetTraceCsv.empty())
    {
      WritePacketTraceHeader();
    }
  if (!m_diagnosticsCsv.empty()) WriteGatewayDiagnosticsHeader();
  if (!m_phyEnergyCsv.empty()) WritePhyEnergyHeader();
  if (!m_storageCsv.empty()) WriteStorageHeader();
}

closure_ingress::Config
AuvEdcExperiment::ReviewedIngressConfiguration() const
{
  closure_ingress::Config c;
  c.source_archive_capacity_bytes = m_reviewedIngressArchiveBytes;
  c.gateway_outbox_capacity_bytes = m_ddnBufferCapacityBytes;
  c.source_staging_ram_capacity_bytes = m_reviewedIngressSourceRamBytes;
  c.gateway_staging_ram_capacity_bytes = m_reviewedIngressGatewayRamBytes;
  c.record_overhead_bytes = m_ddnRecordOverheadBytes;
  c.bit_rate_bps = m_reviewedIngressBitRate;
  c.wire_bits_per_byte = m_reviewedIngressWireBitsPerByte;
  c.data_header_bytes = m_reviewedIngressDataHeaderBytes;
  c.ack_wire_bytes = m_reviewedIngressAckBytes;
  c.one_way_propagation_s = m_reviewedIngressPropagationS;
  c.durable_admission_delay_s = m_reviewedIngressDurableWriteS;
  c.turnaround_s = m_reviewedIngressTurnaroundS;
  c.retry_backoff_s = m_reviewedIngressRetryS;
  c.failed_ack_timeout_s = m_reviewedIngressAckTimeoutS;
  c.max_attempts_per_credit_epoch = m_reviewedIngressMaxAttempts;
  c.archive_retention_end_s = m_reviewedIngressRetentionS;
  c.source_budget_j = m_reviewedIngressSourceBudgetJ;
  c.source_acquisition_base_w = m_reviewedIngressSourceBaseW;
  c.source_tx_w = m_reviewedIngressSourceTxW;
  c.source_rx_w = m_reviewedIngressSourceRxW;
  c.gateway_tx_w = m_reviewedIngressGatewayTxW;
  c.gateway_rx_w = m_reviewedIngressGatewayRxW;
  return c;
}

void
AuvEdcExperiment::ReviewedVerticalReceive(const EdcHeader& header)
{
  auto& leg = m_reviewedLegs.at(header.m_token);
  NS_ABORT_MSG_IF(!m_reviewedVerticalIngress || leg.leg != 1 || leg.packet.priority == Priority::LOW ||
                  leg.receiver != ReviewedUpstreamReceiver(leg.ddnIndex),
                  "Vertical input must be complete HIGH/MED DATA at the paired bottom relay");
  auto pending = m_reviewedVertical.find(leg.packet.id);
  if (pending != m_reviewedVertical.end())
    {
      pending->second.lastReceivedHeader = header;
      TracePacket("reviewed_vertical_duplicate_wait", leg.packet, leg.ddnIndex, -1,
                  header.m_attempt, -1.0, "one_finite_local_copy_no_premature_ack");
      return;
    }
  const double now = Simulator::Now().GetSeconds();
  const uint32_t i = leg.ddnIndex;
  ReviewedVerticalState state;
  state.packet = leg.packet; state.lastReceivedHeader = header; state.ddnIndex = i;
  m_reviewedVertical.emplace(leg.packet.id, state);
  closure_ingress::Record record{leg.packet.id, ++m_reviewedIngressSequences.at(i),
      "report:"+std::to_string(leg.packet.id), leg.packet.sizeBytes,
      ReviewedCablePriority(leg.packet.priority), leg.packet.createdAt, leg.packet.createdAt};
  record.source_ready_s = now; record.source_node = leg.packet.sourceBsn;
  // The paired relay and logger share the declared local finite archive and
  // cable endpoint. The logger supplies this cable; the DRN radio keeps its own
  // existing PHY battery. No second vertical server or new power supply exists.
  m_reviewedIngressBusy[i] = true; m_reviewedIngressClock[i] = now;
  const auto accepted = m_reviewedIngressLinks.at(i)->Submit(record, now, true);
  m_reviewedIngressBusy[i] = false;
  if (accepted != closure_ingress::SubmitResult::Accepted)
    {
      m_reviewedVertical.erase(leg.packet.id);
      ++m_reviewedUpstreamAdmissionRejected;
      TracePacket("reviewed_vertical_local_backpressure", leg.packet, i, -1,
                  header.m_attempt, -1.0, "upstream_sender_retains_copy_no_ack");
    }
  else TracePacket("reviewed_vertical_queued", leg.packet, i, -1, header.m_attempt, -1.0,
                   "shared_nonpreemptive_priority_cable_no_upstream_ack");
  ReviewedIngressTick(i, now);
}

bool
AuvEdcExperiment::ReviewedVerticalAdmission(uint32_t i, const closure_ingress::Record& record, double time)
{
  auto found = m_reviewedVertical.find(record.id);
  NS_ABORT_MSG_IF(found == m_reviewedVertical.end() || found->second.ddnIndex != i ||
                  std::abs(Simulator::Now().GetSeconds()-time) > 1e-6,
                  "Vertical durable admission requires an existing bounded local transaction");
  auto& local = found->second;
  if (local.gatewayCommitted) return true;
  auto& leg = m_reviewedLegs.at(local.lastReceivedHeader.m_token);
  if (DeadlineExpired(leg.packet)) return false;
  NS_ABORT_MSG_IF(m_reviewedVerticalCommitToken != 0, "Nested vertical durable admission");
  m_reviewedVerticalCommitToken = leg.token;
  ReviewedArrival(local.lastReceivedHeader);
  m_reviewedVerticalCommitToken = 0;
  local.gatewayCommitted = leg.arrived;
  return local.gatewayCommitted;
}

void
AuvEdcExperiment::ReviewedVerticalAcknowledge(uint64_t id)
{
  auto found = m_reviewedVertical.find(id);
  if (found == m_reviewedVertical.end() || found->second.cableConfirmed) return;
  auto& local = found->second;
  NS_ABORT_MSG_IF(!local.gatewayCommitted, "Cable ACK cannot confirm an uncommitted gateway record");
  local.cableConfirmed = true;
  // Called only after the kernel's finite reverse ACK actually reached the
  // bottom endpoint with both electrical owners available throughout activity.
  NS_ABORT_MSG_IF(m_reviewedVerticalConfirmToken != 0, "Nested vertical confirmation");
  m_reviewedVerticalConfirmToken = local.lastReceivedHeader.m_token;
  ReviewedArrival(local.lastReceivedHeader);
  m_reviewedVerticalConfirmToken = 0;
}

void
AuvEdcExperiment::ReviewedIngressInitialize()
{
  NS_ABORT_MSG_IF(!m_reviewedIngressLinks.empty(), "Reviewed ingress initialized twice");
  const auto config = ReviewedIngressConfiguration();
  if (m_reviewedVerticalIngress)
    for (uint32_t i=0; i<m_numDdn; ++i)
      {
        const uint32_t receiver=ReviewedUpstreamReceiver(i);
        NS_ABORT_MSG_IF(receiver<m_numBsn || receiver-m_numBsn>=m_drns.size() || i>=m_ddns.size(),
                        "Shared vertical cable has no existing paired DRN endpoint");
        const auto& bottom=m_drns[receiver-m_numBsn].position;
        const auto& top=m_ddns[i].position;
        NS_ABORT_MSG_IF(std::abs(bottom.x-top.x)>1e-6 || std::abs(bottom.y-top.y)>1e-6 ||
                        std::abs(bottom.z-1000.0)>1e-6 || std::abs(top.z-500.0)>1e-6,
                        "Shared vertical ingress preserves the existing 1000-m paired relay and 500-m elevated gateway");
      }
  m_reviewedIngressEvents.resize(m_numDdn);
  m_reviewedIngressStates.resize(m_numDdn);
  m_reviewedIngressSequences.assign(m_numDdn, 0);
  m_reviewedIngressBusy.assign(m_numDdn, false);
  m_reviewedIngressCreditPending.assign(m_numDdn, false);
  m_reviewedIngressClock.assign(m_numDdn, 0.0);
  m_reviewedIngressAckTxEnd.assign(m_numDdn, 0.0);
  m_reviewedIngressAckRxEnd.assign(m_numDdn, 0.0);
  if (m_reviewedPhyEnergy)
    {
      NS_ABORT_MSG_IF(m_reviewedPhys.size() < m_numBsn + m_numDrn + m_numDdn,
                      "Finite ingress requires bound shared gateway PHY ledgers");
      for (uint32_t i = 0; i < m_numDdn; ++i)
        {
          closure_energy::Powers p; p.base_w = m_reviewedIngressSourceBaseW;
          m_reviewedIngressSourceLedgers.push_back(std::make_shared<closure_energy::PowerLedger>(
              std::make_shared<closure_energy::Battery>(m_reviewedIngressSourceBudgetJ), p, m_simStop));
        }
    }
  if (!m_reviewedIngressCsv.empty())
    {
      std::ofstream out(m_reviewedIngressCsv, std::ios::trunc);
      NS_ABORT_MSG_IF(!out, "Cannot open reviewedIngressCsv");
      out << "time,kind,ddnIndex,recordId,producerSequence,windowId,payloadBytes,acquisitionStartS,releaseS,epochAttempt,archiveBytes,rawGatewayBytes,energyOwner,energyStartS,energyEndS,powerW,joules,priority,sourceNode,sourceReadyS,windowNumericId,windowPayloadBytes,windowChunkIndex,windowChunkCount\n";
    }
  for (uint32_t i = 0; i < m_numDdn; ++i)
    {
      closure_ingress::Callbacks cb;
      cb.on_admitted = [this, i](const closure_ingress::Record& r, double time) {
        NS_ABORT_MSG_IF(std::abs(Simulator::Now().GetSeconds() - time) > 1e-6,
                        "Finite ingress admission was dispatched late");
        m_reviewedIngressClock[i] = time;
        if (r.priority != closure_ingress::Priority::Low)
          return ReviewedVerticalAdmission(i, r, time);
        DataPacket p{r.id, std::numeric_limits<uint32_t>::max(),
                     r.acquisition_start_s, Simulator::Now().GetSeconds(),
                     Priority::LOW, r.payload_bytes};
        p.windowNumericId = r.windowNumericId;
        p.windowPayloadBytes = r.windowPayloadBytes;
        p.windowChunkIndex = r.windowChunkIndex;
        p.windowChunkCount = r.windowChunkCount;
        // The authoritative all-priority queue may reject the incoming raw
        // record. Its source retains custody; it is not a buffer-drop outcome.
        m_reviewedIngressAdmissionId = r.id;
        const bool admitted = EnqueueContribPacket(i, p);
        m_reviewedIngressAdmissionId = 0;
        if (admitted)
          {
            ++m_stats.reachedDdn;
            TracePacket("gateway_origin", p, i, -1, 0, 0.0,
                        "replay_instrument_finite_ingress");
          }
        return admitted;
      };
      cb.on_event = [this, i](const closure_ingress::Event& e) {
        m_reviewedIngressClock[i] = e.time_s;
        ReviewedIngressTrace(i, e);
        if (m_reviewedVerticalIngress && e.record.priority != closure_ingress::Priority::Low &&
            e.kind == "source_admission_ack_archive_retained")
          Simulator::ScheduleNow(&AuvEdcExperiment::ReviewedVerticalAcknowledge, this, e.record.id);
      };
      cb.on_energy = [this, i](const closure_ingress::EnergyEvent& e) {
        ReviewedIngressEnergy(i, e);
      };
      if (m_reviewedPhyEnergy)
        {
          cb.on_activity = [this, i](const closure_ingress::EnergyEvent& e) { ReviewedIngressActivity(i, e); };
          cb.powered = [this, i](double) { return ReviewedIngressPowered(i); };
          cb.acquisition_available = [this, i](const closure_ingress::Record& r) {
            const auto source = m_reviewedIngressSourceLedgers.at(i);
            source->Advance(Simulator::Now().GetSeconds());
            // Immutable input rows remain measurement opportunities in the
            // denominator, even when power loss prevented window completion.
            return r.priority == closure_ingress::Priority::Low
                ? source->DepletedAt() + 1e-9 >= r.release_s
                : source->Budget()->Remaining() > 0;
          };
          cb.data_delivered = [this, i](const closure_ingress::Record&, double txEnd,
                                        double, double admittedAt) {
            ReviewedIngressPowered(i);
            const auto source = m_reviewedIngressSourceLedgers.at(i);
            const auto gateway = m_reviewedLedgers.at(m_numBsn + m_numDrn + i);
            return source->DepletedAt() + 1e-9 >= txEnd &&
                   gateway->DepletedAt() + 1e-9 >= admittedAt;
          };
          cb.powered_until = [this, i](const std::string& owner, double end) {
            return ReviewedIngressPoweredUntil(i, owner, end);
          };
          cb.ack_delivered = [this, i](const closure_ingress::Record&, bool,
                                       uint32_t, double) {
            ReviewedIngressPowered(i);
            const auto source = m_reviewedIngressSourceLedgers.at(i);
            const auto gateway = m_reviewedLedgers.at(m_numBsn + m_numDrn + i);
            return gateway->DepletedAt() + 1e-9 >= m_reviewedIngressAckTxEnd[i] &&
                   source->DepletedAt() + 1e-9 >= m_reviewedIngressAckRxEnd[i];
          };
        }
      m_reviewedIngressLinks.emplace_back(new closure_ingress::Link(config, cb));
      m_reviewedIngressStates[i] = m_reviewedIngressLinks[i]->Snapshot(0.0);
    }
}

void
AuvEdcExperiment::ReviewedIngressSchedule(uint32_t i)
{
  m_reviewedIngressEvents.at(i).Cancel();
  const auto next = m_reviewedIngressLinks.at(i)->NextEventTime();
  if (!next || *next > m_simStop) return;
  // ns-3 uses quantized time. Round a mathematical event upward by at most one
  // nanosecond; never make DATA usable before its finite ingress completion.
  Time target = Seconds(*next);
  if (target.GetSeconds() < *next) target += NanoSeconds(1);
  const Time delay = target > Simulator::Now() ? target - Simulator::Now() : Seconds(0);
  m_reviewedIngressEvents[i] = Simulator::Schedule(delay,
      &AuvEdcExperiment::ReviewedIngressTick, this, i, *next);
}

void
AuvEdcExperiment::ReviewedIngressTick(uint32_t i, double time)
{
  auto& link = *m_reviewedIngressLinks.at(i);
  m_reviewedIngressBusy[i] = true;
  m_reviewedIngressClock[i] = time;
  link.Advance(time);
  // A space notification generated during an atomic admission cannot reenter
  // the kernel. Send its finite coalesced reverse credit after that transaction.
  if (m_reviewedIngressCreditPending[i])
    {
      m_reviewedIngressCreditPending[i] = false;
      link.NotifyGatewaySpace(time);
    }
  m_reviewedIngressStates[i] = link.Snapshot(time);
  m_reviewedIngressBusy[i] = false;
  NS_ABORT_MSG_IF(!m_reviewedIngressStates[i].conservation_ok,
                  "Reviewed ingress per-owner conservation failed");
  ReviewedIngressSchedule(i);
}

void
AuvEdcExperiment::ReviewedIngressRefresh(double now)
{
  if (!m_reviewedIngress) return;
  for (uint32_t i = 0; i < m_reviewedIngressLinks.size(); ++i)
    {
      ReviewedIngressTick(i, now);
      // Audit only at a settled sampling boundary, not inside a pop/requeue
      // transaction. Mirrored raw descriptors are not a second payload store.
      std::map<uint64_t, uint64_t> raw;
      auto add = [&](const DataPacket& p) {
        if (m_reviewedVerticalIngress ||
            (p.sourceBsn == std::numeric_limits<uint32_t>::max() && p.priority == Priority::LOW))
          raw.emplace(p.id, RecordStorageBytes(p));
      };
      for (const auto* q : {&m_ddnBuffers[i], &m_ddnHighBuffers[i],
                            &m_ddnMediumBuffers[i], &m_ddnLowBuffers[i]})
        for (const auto& p : *q) add(p);
      for (const auto& p : m_pendingDirect) if (p.second.ddnIndex == i) add(p.second.packet);
      for (const auto& p : m_pendingMobile) if (p.second.ddnIndex == i) add(p.second.packet);
      for (const auto& p : m_pendingOptical) if (p.second.ddnIndex == i) add(p.second.packet);
      uint64_t bytes = 0;
      for (const auto& p : raw) bytes += p.second;
      NS_ABORT_MSG_IF(bytes != m_reviewedIngressStates[i].raw_gateway_resident_bytes ||
                      raw.size() != m_reviewedIngressStates[i].raw_gateway_records,
                      "Reviewed ingress raw mirror differs from authoritative gateway resident ownership");
    }
  if (m_reviewedPhyEnergy)
    {
      double baseJ = 0;
      for (const auto& source : m_reviewedIngressSourceLedgers)
        { source->Advance(now); baseJ += source->Accounting().base_j; }
      const double delta = baseJ - m_reviewedIngressSourceBaseEnergyJ;
      NS_ABORT_MSG_IF(delta < -1e-9, "Source acquisition energy decreased");
      m_stats.energyConsumedJ += std::max(0.0, delta);
      m_stats.lowEnergyJ += std::max(0.0, delta);
      m_reviewedIngressSourceBaseEnergyJ = baseJ;
    }
}

bool
AuvEdcExperiment::ReviewedIngressPowered(uint32_t i)
{
  const double now = Simulator::Now().GetSeconds();
  const uint32_t gateway = m_numBsn + m_numDrn + i;
  m_reviewedIngressSourceLedgers.at(i)->Advance(now);
  m_reviewedPhys.at(gateway)->Sync();
  return m_reviewedIngressSourceLedgers[i]->Budget()->Remaining() > 0 &&
         m_reviewedLedgers.at(gateway)->Budget()->Remaining() > 0;
}

double
AuvEdcExperiment::ReviewedIngressPoweredUntil(uint32_t i, const std::string& owner, double end)
{
  ReviewedIngressPowered(i);
  const auto ledger = owner == "source" ? m_reviewedIngressSourceLedgers.at(i)
      : m_reviewedLedgers.at(m_numBsn + m_numDrn + i);
  return std::min(end, ledger->DepletedAt());
}

void
AuvEdcExperiment::ReviewedIngressActivity(uint32_t i, closure_ingress::EnergyEvent e)
{
  if (e.activity == "cable_ack_tx") m_reviewedIngressAckTxEnd[i] = e.end_s;
  if (e.activity == "cable_ack_rx") m_reviewedIngressAckRxEnd[i] = e.end_s;
  Time start = Seconds(e.start_s);
  if (start.GetSeconds() < e.start_s) start += NanoSeconds(1);
  if (start <= Simulator::Now()) ReviewedIngressBeginLoad(i, e);
  else Simulator::Schedule(start - Simulator::Now(),
      &AuvEdcExperiment::ReviewedIngressBeginLoad, this, i, e);
}

void
AuvEdcExperiment::ReviewedIngressBeginLoad(uint32_t i, closure_ingress::EnergyEvent e)
{
  const double now = Simulator::Now().GetSeconds();
  const double duration = e.end_s - now;
  if (duration <= 0) return;
  uint64_t loadId = 0;
  if (e.owner == "source")
    {
      const auto ledger = m_reviewedIngressSourceLedgers.at(i);
      ledger->Advance(now);
      if (ledger->Budget()->Remaining() <= 0) return;
      loadId = ledger->AddLoad(now, duration, e.power_w);
    }
  else
    {
      const uint32_t gateway = m_numBsn + m_numDrn + i;
      m_reviewedPhys.at(gateway)->Sync();
      if (m_reviewedLedgers.at(gateway)->Budget()->Remaining() <= 0) return;
      loadId = m_reviewedPhys[gateway]->AddExternalLoad(duration, e.power_w);
    }
  if (!loadId) return;
  Time end = Seconds(e.end_s);
  if (end.GetSeconds() < e.end_s) end += NanoSeconds(1);
  Simulator::Schedule(end - Simulator::Now(),
      &AuvEdcExperiment::ReviewedIngressEndLoad, this, i, e.owner, loadId);
}

void
AuvEdcExperiment::ReviewedIngressEndLoad(uint32_t i, std::string owner, uint64_t id)
{
  if (owner == "source")
    m_reviewedIngressSourceLedgers.at(i)->StopLoad(Simulator::Now().GetSeconds(), id);
  else m_reviewedPhys.at(m_numBsn + m_numDrn + i)->StopExternalLoad(id);
}

void
AuvEdcExperiment::ReviewedIngressRelease(uint32_t i, uint64_t id,
                                         closure_ingress::Removal reason, bool freedSpace)
{
  if (!m_reviewedIngress || i >= m_reviewedIngressLinks.size()) return;
  const bool reentrant = m_reviewedIngressBusy[i];
  const double now = reentrant ? m_reviewedIngressClock[i] : Simulator::Now().GetSeconds();
  m_reviewedIngressBusy[i] = true;
  if (!reentrant) m_reviewedIngressClock[i] = now;
  const bool released = m_reviewedIngressLinks[i]->ReleaseGateway(id, reason, now);
  if (released || freedSpace) m_reviewedIngressCreditPending[i] = true;
  if (reentrant) return;
  m_reviewedIngressBusy[i] = false;
  // Do not inspect sender-visible credits until the current main queue
  // operation is finished, especially a pop followed by a new pending transfer.
  m_reviewedIngressEvents[i].Cancel();
  m_reviewedIngressEvents[i] = Simulator::ScheduleNow(
      &AuvEdcExperiment::ReviewedIngressTick, this, i, now);
}

void
AuvEdcExperiment::ReviewedIngressTrace(uint32_t i, const closure_ingress::Event& e) const
{
  if (m_reviewedIngressCsv.empty()) return;
  std::ofstream out(m_reviewedIngressCsv, std::ios::app);
  NS_ABORT_MSG_IF(!out, "Cannot append reviewedIngressCsv");
  std::string window;
  for (char c : e.record.window_id) { if (c == '"') window += '"'; window += c; }
  out << std::setprecision(17) << e.time_s << ',' << e.kind << ',' << i << ','
      << e.record.id << ',' << e.record.producer_sequence << ",\"" << window << "\","
      << e.record.payload_bytes << ',' << e.record.acquisition_start_s << ','
      << e.record.release_s << ',' << e.epoch_attempt << ',' << e.archive_bytes << ','
      << e.raw_gateway_bytes << ",,,,,," << static_cast<unsigned>(e.record.priority) << ','
      << e.record.source_node << ','
      << (e.record.source_ready_s<0 ? e.record.release_s : e.record.source_ready_s) << ','
      << e.record.windowNumericId << ',' << e.record.windowPayloadBytes << ','
      << e.record.windowChunkIndex << ',' << e.record.windowChunkCount << '\n';
}

void
AuvEdcExperiment::ReviewedIngressEnergy(uint32_t i, const closure_ingress::EnergyEvent& e)
{
  // Auxiliary activity accounting, separate from authoritative PHY batteries.
  // Source is an industrial logger, never a tiny BSN assigned its archive.
  const auto relayed=m_reviewedVertical.find(e.record_id);
  const Priority priority=relayed==m_reviewedVertical.end() ? Priority::LOW : relayed->second.packet.priority;
  if (e.owner == "source")
    {
      m_reviewedIngressSourceEnergyJ += e.joules;
      m_stats.energyConsumedJ += e.joules;
      if (priority==Priority::HIGH) m_stats.highEnergyJ += e.joules;
      else if (priority==Priority::MEDIUM) m_stats.mediumEnergyJ += e.joules;
      else m_stats.lowEnergyJ += e.joules;
    }
  else
    {
      m_reviewedIngressGatewayEnergyJ += e.joules;
      ChargePacketEnergy(priority, e.joules, NodeRole::DDN);
    }
  if (m_reviewedIngressCsv.empty()) return;
  std::ofstream out(m_reviewedIngressCsv, std::ios::app);
  NS_ABORT_MSG_IF(!out, "Cannot append reviewedIngressCsv energy interval");
  out << std::setprecision(17) << e.end_s << ',' << e.activity << ',' << i << ','
      << e.record_id << ",0,,0,0,0,0,0,0," << e.owner << ',' << e.start_s << ','
      << e.end_s << ',' << e.power_w << ',' << e.joules << ",,,,,,,\n";
}

uint64_t
AuvEdcExperiment::ReviewedIngressSourceHeld() const
{
  uint64_t count = 0;
  // H/M cable staging is an additional finite physical copy of the still-owned
  // BSN transaction, already present in acousticPending. Do not count it twice.
  for (const auto& s : m_reviewedIngressStates) count += s.low_source_custody_records;
  return count;
}

uint64_t
AuvEdcExperiment::ReviewedIngressArchiveOverflow() const
{
  uint64_t count = 0;
  for (const auto& s : m_reviewedIngressStates) count += s.counters.low_archive_overflow_records;
  return count;
}

uint64_t
AuvEdcExperiment::ReviewedIngressSourceUnavailable() const
{
  uint64_t count = 0;
  for (const auto& s : m_reviewedIngressStates) count += s.counters.low_source_unavailable_records;
  return count;
}

void
AuvEdcExperiment::ReviewedIngressSummary() const
{
  if (!m_reviewedIngress) return;
  for (uint32_t i = 0; i < m_reviewedIngressStates.size(); ++i)
    {
      const auto& s = m_reviewedIngressStates[i];
      std::cout << std::setprecision(17) << "REVIEWED_INGRESS {\"version\":\"finite-ingress-v1\","
          << "\"full_validation\":false,\"cable_battery_binding\":"
          << (m_reviewedPhyEnergy ? "true" : "false") << ','
          << "\"acquisition_baseline_and_completion_power_gate\":"
          << (m_reviewedPhyEnergy ? "true" : "false") << ','
          << "\"sample_level_acquisition_pipeline_modelled\":false,\"no_intrinsic_cable_bit_error_assumption\":true,"
          << "\"high_medium_vertical_ingress_integrated\":" << (m_reviewedVerticalIngress ? "true" : "false") << ','
          << "\"shared_cable_nonpreemptive_priority\":" << (m_reviewedVerticalIngress ? "true" : "false") << ','
          << "\"gateway_mirror_scope\":\"" << (m_reviewedVerticalIngress ? "all_priorities" : "LOW") << "\","
          << "\"local_drn_logger_interface_model\":\"co_located_finite_shared_archive_admission\","
          << "\"cable_source_battery_owner\":\"logger\",\"drn_radio_uses_own_phy_battery\":true,"
          << "\"relay_copies_excluded_from_unique_generation\":true,\"time_quantum_s\":1e-9,"
          << "\"completed_window_descriptor_wire_bytes\":24,\"descriptor_in_64B_store_allowance\":true,"
          << "\"gateway_all_priority_queue_authoritative\":true,\"archive_retained_after_ack\":true,"
          << "\"ddn_index\":" << i << ",\"horizon_s\":" << s.time_s
          << ",\"source_archive_capacity_bytes\":" << m_reviewedIngressArchiveBytes
          << ",\"source_budget_j\":" << m_reviewedIngressSourceBudgetJ
          << ",\"source_acquisition_base_w\":" << m_reviewedIngressSourceBaseW
          << ",\"gateway_outbox_capacity_bytes\":" << m_ddnBufferCapacityBytes
          << ",\"source_staging_ram_capacity_bytes\":" << m_reviewedIngressSourceRamBytes
          << ",\"gateway_staging_ram_capacity_bytes\":" << m_reviewedIngressGatewayRamBytes
          << ",\"stored_metadata_bytes\":" << m_ddnRecordOverheadBytes
          << ",\"bit_rate_bps\":" << m_reviewedIngressBitRate
          << ",\"wire_bits_per_byte\":" << m_reviewedIngressWireBitsPerByte
          << ",\"data_header_bytes\":" << m_reviewedIngressDataHeaderBytes
          << ",\"ack_credit_bytes\":" << m_reviewedIngressAckBytes
          << ",\"propagation_s\":" << m_reviewedIngressPropagationS
          << ",\"durable_admission_delay_s\":" << m_reviewedIngressDurableWriteS
          << ",\"turnaround_s\":" << m_reviewedIngressTurnaroundS
          << ",\"retry_backoff_s\":" << m_reviewedIngressRetryS
          << ",\"lost_ack_timeout_s\":" << m_reviewedIngressAckTimeoutS
          << ",\"max_attempts_per_credit_epoch\":" << m_reviewedIngressMaxAttempts
          << ",\"archive_retention_end_s\":" << m_reviewedIngressRetentionS
          << ",\"source_tx_power_w\":" << m_reviewedIngressSourceTxW
          << ",\"source_rx_power_w\":" << m_reviewedIngressSourceRxW
          << ",\"gateway_tx_power_w\":" << m_reviewedIngressGatewayTxW
          << ",\"gateway_rx_power_w\":" << m_reviewedIngressGatewayRxW
          << ",\"generated_records\":" << s.counters.generated_records
          << ",\"generated_payload_bytes\":" << s.counters.generated_payload_bytes
          << ",\"source_held_records\":" << s.source_custody_records
          << ",\"low_source_held_unique_records\":" << s.low_source_custody_records
          << ",\"relay_source_held_copies\":" << s.source_custody_records-s.low_source_custody_records
          << ",\"source_held_payload_bytes\":" << s.source_custody_payload_bytes
          << ",\"source_archive_bytes\":" << s.archive_resident_bytes
          << ",\"historical_archive_records\":" << s.retained_historical_records
          << ",\"sender_unconfirmed_records\":" << s.source_unconfirmed_records
          << ",\"raw_gateway_records\":" << s.raw_gateway_records
          << ",\"raw_gateway_resident_bytes\":" << s.raw_gateway_resident_bytes
          << ",\"archive_overflow_records\":" << s.counters.archive_overflow_records
          << ",\"archive_overflow_payload_bytes\":" << s.counters.archive_overflow_payload_bytes
          << ",\"source_unavailable_records\":" << s.counters.source_unavailable_records
          << ",\"power_interrupted_attempts\":" << s.counters.power_interrupted_attempts
          << ",\"power_blocked_records\":" << s.counters.power_blocked_records
          << ",\"sender_copies_forwarded\":" << s.counters.forwarded_records
          << ",\"sender_copies_discarded\":" << s.counters.gateway_dropped_records
          << ",\"source_staging_ram_bytes\":" << s.source_staging_ram_bytes
          << ",\"gateway_staging_ram_bytes\":" << s.gateway_staging_ram_bytes
          << ",\"source_ram_high_water_bytes\":" << s.source_ram_high_water_bytes
          << ",\"gateway_ram_high_water_bytes\":" << s.gateway_ram_high_water_bytes
          << ",\"archive_high_water_bytes\":" << s.archive_high_water_bytes
          << ",\"raw_gateway_high_water_bytes\":" << s.raw_gateway_high_water_bytes
          << ",\"data_attempts\":" << s.counters.data_attempts
          << ",\"admissions\":" << s.counters.gateway_admissions
          << ",\"backpressure_rejections\":" << s.counters.gateway_rejections
          << ",\"upstream_local_backpressure_rejections\":" << s.counters.upstream_backpressure_rejections
          << ",\"credit_frames\":" << s.counters.credit_frames
          << ",\"ram_blocked_records\":" << s.counters.ram_blocked_records
          << ",\"source_cable_active_j\":" << s.source_cable_active_energy_j
          << ",\"gateway_cable_active_j\":" << s.gateway_cable_active_energy_j
          << ",\"conservation_ok\":" << (s.conservation_ok ? "true" : "false");
      if (m_reviewedPhyEnergy)
        {
          const auto ledger = m_reviewedIngressSourceLedgers.at(i);
          std::cout << ",\"source_battery_consumed_j\":" << ledger->Budget()->Consumed()
              << ",\"source_battery_remaining_j\":" << ledger->Budget()->Remaining()
              << ",\"source_acquisition_base_j\":" << ledger->Accounting().base_j
              << ",\"source_cable_ledger_j\":" << ledger->Accounting().extra_j
              << ",\"source_depleted_at_s\":";
          if (std::isfinite(ledger->DepletedAt())) std::cout << ledger->DepletedAt();
          else std::cout << "null";
        }
      std::cout << "}\n";
    }
}

void
AuvEdcExperiment::GenerateReplayPacket(edc_replay::Event event)
{
  // Legacy and narrow unit-test entry point, without a completed-window batch.
  GenerateReviewedReplayPacket(event, 0, 0, 0, 0);
}

void
AuvEdcExperiment::GenerateReviewedReplayBatch(std::vector<edc_replay::Event> events)
{
  struct Window {
    uint64_t first = 0, bytes = 0;
    uint32_t count = 0, emitted = 0;
    int32_t source = -1, ddn = -1;
    double acquisition = 0, release = 0;
  };
  std::map<std::string, Window> windows;
  const double now = Simulator::Now().GetSeconds();
  // A logger knows the chunks of its own completed acquisition at release.
  // No cross-source descriptor or unreleased input survives this callback.
  for (const auto& event : events)
    {
      NS_ABORT_MSG_IF(std::abs(now-event.release_s) > 1e-6,
                      "Producer batch contains an unavailable/future record");
      if (event.priority != edc_replay::Priority::Low) continue;
      auto& w = windows[event.window_id];
      if (!w.count)
        { w.first=event.record_id; w.source=event.source_bsn; w.ddn=event.ddn_index;
          w.acquisition=event.acquisition_start_s; w.release=event.release_s; }
      NS_ABORT_MSG_IF(w.source!=event.source_bsn || w.ddn!=event.ddn_index ||
          w.acquisition!=event.acquisition_start_s || w.release!=event.release_s ||
          w.count==std::numeric_limits<uint32_t>::max() ||
          event.payload_bytes>std::numeric_limits<uint64_t>::max()-w.bytes,
          "Invalid completed-window producer descriptor");
      w.first=std::min(w.first,event.record_id); w.bytes+=event.payload_bytes; ++w.count;
    }
  for (const auto& event : events)
    if (event.priority==edc_replay::Priority::Low)
      {
        auto& w=windows.at(event.window_id);
        GenerateReviewedReplayPacket(event,w.first,w.bytes,w.emitted++,w.count);
      }
    else GenerateReviewedReplayPacket(event,0,0,0,0);
}

void
AuvEdcExperiment::GenerateReviewedReplayPacket(edc_replay::Event event, uint64_t windowNumericId,
    uint64_t windowPayloadBytes, uint32_t windowChunkIndex, uint32_t windowChunkCount)
{
  const double now = Simulator::Now().GetSeconds();
  NS_ABORT_MSG_IF(now >= m_trafficStop || std::abs(now - event.release_s) > 1e-6,
                  "Replay event dispatched outside its declared release time");
  const bool inspection = event.priority == edc_replay::Priority::Low;
  const Priority priority = inspection ? Priority::LOW :
      (event.priority == edc_replay::Priority::High ? Priority::HIGH : Priority::MEDIUM);
  const uint32_t source = inspection ? std::numeric_limits<uint32_t>::max() :
      static_cast<uint32_t>(event.source_bsn);
  const uint32_t ddnIndex = inspection ? static_cast<uint32_t>(event.ddn_index) :
      DdnForDrn(AssociatedDrn(source));
  DataPacket packet{event.record_id, source,
      inspection && m_reviewedIngress ? event.acquisition_start_s : now,
      0.0, priority, event.payload_bytes};
  packet.windowNumericId=windowNumericId; packet.windowPayloadBytes=windowPayloadBytes;
  packet.windowChunkIndex=windowChunkIndex; packet.windowChunkCount=windowChunkCount;
  m_stats.generated++;
  AccountGeneratedPriority(priority);
  TracePacket("generated", packet, static_cast<int32_t>(ddnIndex), -1,
              0, -1.0, "replay");
  if (inspection)
    {
      if (m_reviewedIngress)
        {
          ++m_stats.gatewayOriginated; // Source identity, not premature gateway arrival.
          closure_ingress::Record r{event.record_id, ++m_reviewedIngressSequences.at(ddnIndex),
              event.window_id, event.payload_bytes, closure_ingress::Priority::Low,
              event.acquisition_start_s, event.release_s};
          r.windowNumericId=windowNumericId; r.windowPayloadBytes=windowPayloadBytes;
          r.windowChunkIndex=windowChunkIndex; r.windowChunkCount=windowChunkCount;
          m_reviewedIngressBusy[ddnIndex] = true;
          m_reviewedIngressClock[ddnIndex] = event.release_s;
          const auto admitted = m_reviewedIngressLinks.at(ddnIndex)->Submit(r, event.release_s);
          m_reviewedIngressBusy[ddnIndex] = false;
          NS_ABORT_MSG_IF(admitted == closure_ingress::SubmitResult::Duplicate,
                          "Immutable replay generated an ingress record twice");
          TracePacket(admitted == closure_ingress::SubmitResult::ArchiveFull
                          ? "source_archive_overflow" :
                          admitted == closure_ingress::SubmitResult::SourceUnavailable
                          ? "source_unavailable_at_window_release" : "source_archive_queued",
                      packet, ddnIndex, -1, 0, -1.0, "finite_ingress");
          ReviewedIngressTick(ddnIndex, event.release_s);
          return;
        }
      // The source is a separate pipeline instrument, never a tiny BSN.
      // This first replay boundary starts at completed-window gateway release;
      // acquisition age/window identity stay in the immutable replay input.
      m_stats.gatewayOriginated++;
      m_stats.reachedDdn++;
      packet.ddnArrivalAt = now;
      TracePacket("gateway_origin", packet, static_cast<int32_t>(ddnIndex), -1,
                  0, 0.0, "replay_instrument_ideal_ingress");
      EnqueueContribPacket(ddnIndex, packet);
      return;
    }
  m_pendingAcoustic.emplace(packet.id,
      PendingAcousticPacket{packet, ddnIndex, 0, EventId()});
  StartAcousticAttempt(packet.id);
}

void
AuvEdcExperiment::GeneratePacket()
{
  double now = Simulator::Now().GetSeconds();
  if (now >= m_trafficStop)
    {
      return;
    }

  uint32_t bsnIndex = m_nextBsn++ % m_bsns.size();
  uint32_t drnIndex = AssociatedDrn(bsnIndex);
  const bool hasEomGateway = (m_protocol == ProtocolMode::CONTRIBUTION ||
                              m_protocol == ProtocolMode::PURE_ACOUSTIC);
  uint32_t ddnIndex = hasEomGateway
                          ? DdnForDrn(drnIndex)
                          : NearestDdn(m_bsns[bsnIndex].position.x);
  Priority priority = AssignPriority(m_nextPacketId);
  DataPacket packet{m_nextPacketId, bsnIndex, now, 0.0, priority,
                    PacketSizeBytes(priority)};
  m_nextPacketId++;
  m_stats.generated++;
  AccountGeneratedPriority(packet.priority);
  TracePacket("generated", packet, static_cast<int32_t>(ddnIndex), -1,
              0, -1.0, "created");

  // Large waveform/image records may be produced by a sensor integrated with
  // the collection gateway. This switch keeps that explicitly separate from
  // the default model, in which every application record traverses the bottom
  // acoustic access network before AUV/direct offload.
  if (m_lowPayloadAtGateway && packet.priority == Priority::LOW)
    {
      m_stats.gatewayOriginated++;
      packet.ddnArrivalAt = now;
      m_stats.reachedDdn++;
      TracePacket("gateway_origin", packet, static_cast<int32_t>(ddnIndex), -1,
                  0, 0.0, "enqueued");
      if (UsesPriorityQueues())
        {
          EnqueueContribPacket(ddnIndex, packet);
        }
      else
        {
          if (m_ddnBufferCapacity > 0 &&
              m_ddnBuffers[ddnIndex].size() >= m_ddnBufferCapacity)
            {
              m_stats.bufferDropped++;
              m_stats.bufferDroppedLow++;
            }
          else
            {
              m_ddnBuffers[ddnIndex].push_back(packet);
            }
        }
      Simulator::Schedule(Seconds(m_arrivalRng->GetValue()),
                          &AuvEdcExperiment::GeneratePacket, this);
      return;
    }

  // The BSN-to-DDN path is now driven by actual AquaSim packet transmissions.
  // Source timeout/retry state remains pending until the endpoint DATA-arrival
  // callback supplies the delivery confirmation.
  m_pendingAcoustic.emplace(packet.id,
                            PendingAcousticPacket{packet, ddnIndex, 0, EventId()});
  StartAcousticAttempt(packet.id);
  Simulator::Schedule(Seconds(m_arrivalRng->GetValue()),
                      &AuvEdcExperiment::GeneratePacket, this);
  return;
}

void
AuvEdcExperiment::EnqueueAtDdn(uint32_t ddnIndex, DataPacket packet)
{
  packet.ddnArrivalAt = Simulator::Now().GetSeconds();

  // Acoustic-chain delay: BSN→DDN hop (comparable to Ahmed 2022's 1–9 s metric).
  // Recorded here for ALL protocols, independently of how/when surface delivery occurs.
  m_stats.acousticDelivered++;
  m_stats.acousticDelaySum += packet.ddnArrivalAt - packet.createdAt;

  if (UsesPriorityQueues())
    {
      m_stats.reachedDdn++;
      EnqueueContribPacket(ddnIndex, packet);
    }
  else
    {
      // BASELINE: packet reaches DDN — buffer for AUV collection.
      // Surface delivery is counted in AuvTick() when AUV delivers to sink,
      // same measurement point as the proposed and direct EOM architectures.
      m_stats.reachedDdn++;
      if (m_ddnBufferCapacity > 0 &&
          m_ddnBuffers[ddnIndex].size() >= m_ddnBufferCapacity)
        {
          m_stats.bufferDropped++;
          return; // baseline: tail-drop on overflow
        }
      m_ddnBuffers[ddnIndex].push_back(packet);
    }
}

// Priority-aware finite-buffer enqueue for contribution and matched-control DDN queues.
// Returns false if the packet was dropped on overflow.
bool
AuvEdcExperiment::EnqueueContribPacket(uint32_t ddnIndex, const DataPacket& packet,
                                       bool atFront)
{
  // Settle same-time cable arrivals before computing an eviction plan. Later
  // RecordBufferDrop hooks must not recursively admit a raw record into the
  // middle of this all-or-nothing main-queue mutation.
  if (m_reviewedIngress && ddnIndex < m_reviewedIngressLinks.size() &&
      !m_reviewedIngressBusy[ddnIndex])
    ReviewedIngressTick(ddnIndex, Simulator::Now().GetSeconds());
  uint64_t records = DdnQueueRecords(ddnIndex);
  // With byte mode disabled retain the legacy one-victim behavior even if a
  // prior legacy requeue temporarily exceeded its record count. New byte mode
  // strictly enforces both independently configured limits.
  const uint64_t countCeiling = m_ddnBufferCapacityBytes == 0 &&
      m_ddnBufferCapacity > 0 && records >= m_ddnBufferCapacity
          ? records : m_ddnBufferCapacity;
  uint64_t bytes = m_ddnByteBudgetScope == "resident"
      ? DdnResidentStorage(ddnIndex, packet.id).second : DdnQueueBytes(ddnIndex);
  const uint64_t incoming = RecordStorageBytes(packet);
  auto fits = [&]() {
    return (m_ddnBufferCapacity == 0 || records < countCeiling) &&
           (m_ddnBufferCapacityBytes == 0 ||
            (incoming <= m_ddnBufferCapacityBytes &&
             bytes <= m_ddnBufferCapacityBytes - incoming));
  };
  // Plan all evictions before changing queues: a large incoming record must
  // not destroy lower-priority records and then still fail admission.
  std::vector<std::deque<DataPacket>*> victims;
  if (!fits() && m_ddnBufferPolicy == "priority" &&
      (m_ddnBufferCapacityBytes == 0 || incoming <= m_ddnBufferCapacityBytes))
    {
      auto plan = [&](std::deque<DataPacket>& queue) {
        for (auto it = queue.rbegin(); it != queue.rend() && !fits(); ++it)
          {
            victims.push_back(&queue);
            --records;
            bytes -= RecordStorageBytes(*it);
          }
      };
      if (packet.priority == Priority::HIGH || packet.priority == Priority::MEDIUM)
        plan(m_ddnLowBuffers[ddnIndex]);
      if (packet.priority == Priority::HIGH) plan(m_ddnMediumBuffers[ddnIndex]);
    }
  if (!fits())
    {
      if ((m_reviewedIngress && m_reviewedIngressAdmissionId == packet.id) ||
          (m_reviewedTransport && m_reviewedUpstreamAdmissionId == packet.id))
        {
          WriteStorage(m_reviewedUpstreamAdmissionId == packet.id
                           ? "upstream_admission_rejected" : "ingress_rejected",
                       ddnIndex, &packet, m_reviewedUpstreamAdmissionId == packet.id
                           ? "source_retains_retry_custody"
                           : "source_retains_finite_archive_and_forwarding_custody");
          return false;
        }
      RecordBufferDrop(ddnIndex, packet,
          m_ddnBufferCapacityBytes > 0 && incoming > m_ddnBufferCapacityBytes
              ? "record_exceeds_byte_capacity" : "admission_capacity");
      return false;
    }
  for (auto* queue : victims)
    {
      const DataPacket victim = queue->back();
      queue->pop_back();
      RecordBufferDrop(ddnIndex, victim, "priority_eviction");
    }
  std::deque<DataPacket>* queue = &m_ddnLowBuffers[ddnIndex];
  if (packet.priority == Priority::HIGH) queue = &m_ddnHighBuffers[ddnIndex];
  else if (packet.priority == Priority::MEDIUM) queue = &m_ddnMediumBuffers[ddnIndex];
  if (atFront) queue->push_front(packet);
  else queue->push_back(packet);
  ObserveStorage(ddnIndex);
  WriteStorage(atFront ? "requeue" : "admission", ddnIndex, &packet);
  return true;
}

uint64_t
AuvEdcExperiment::RecordStorageBytes(const DataPacket& packet) const
{
  return static_cast<uint64_t>(packet.sizeBytes) + m_ddnRecordOverheadBytes;
}

uint64_t
AuvEdcExperiment::DdnQueueRecords(uint32_t i) const
{
  return m_ddnBuffers[i].size() + m_ddnHighBuffers[i].size() +
         m_ddnMediumBuffers[i].size() + m_ddnLowBuffers[i].size();
}

uint64_t
AuvEdcExperiment::DdnQueueBytes(uint32_t i) const
{
  uint64_t bytes = 0;
  for (const auto* queue : {&m_ddnBuffers[i], &m_ddnHighBuffers[i],
                            &m_ddnMediumBuffers[i], &m_ddnLowBuffers[i]})
    for (const auto& packet : *queue) bytes += RecordStorageBytes(packet);
  return bytes;
}

std::pair<uint64_t, uint64_t>
AuvEdcExperiment::DdnResidentStorage(uint32_t i, uint64_t excludeId) const
{
  // One logical retained record per packet ID, even while an event handler
  // briefly contains references in both queue and transmission state.
  // Direct/mobile waiting lists store IDs already represented by pending maps.
  // This is application payload+metadata allocation, not allocator overhead,
  // modem buffers, ns-3 object copies, or complete physical-device RAM usage.
  std::unordered_set<uint64_t> ids;
  uint64_t bytes = 0;
  auto add = [&](const DataPacket& packet) {
    if (packet.id != excludeId && ids.insert(packet.id).second)
      bytes += RecordStorageBytes(packet);
  };
  for (const auto* queue : {&m_ddnBuffers[i], &m_ddnHighBuffers[i],
                            &m_ddnMediumBuffers[i], &m_ddnLowBuffers[i]})
    for (const auto& packet : *queue) add(packet);
  for (const auto& entry : m_pendingDirect)
    if (entry.second.ddnIndex == i) add(entry.second.packet);
  for (const auto& entry : m_pendingOptical)
    if (entry.second.ddnIndex == i) add(entry.second.packet);
  for (const auto& entry : m_pendingMobile)
    if (entry.second.ddnIndex == i) add(entry.second.packet);
  return {ids.size(), bytes};
}

void
AuvEdcExperiment::ObserveStorage(uint32_t i)
{
  if (m_storageCsv.empty() && m_ddnBufferCapacityBytes == 0) return;
  m_storageHighWaterRecords[i] = std::max(m_storageHighWaterRecords[i], DdnQueueRecords(i));
  m_storageHighWaterBytes[i] = std::max(m_storageHighWaterBytes[i], DdnQueueBytes(i));
  const auto resident = DdnResidentStorage(i);
  m_residentHighWaterRecords[i] = std::max(m_residentHighWaterRecords[i], resident.first);
  m_residentHighWaterBytes[i] = std::max(m_residentHighWaterBytes[i], resident.second);
  const uint64_t boundedBytes = m_ddnByteBudgetScope == "resident"
      ? resident.second : DdnQueueBytes(i);
  NS_ABORT_MSG_IF(m_ddnBufferCapacityBytes > 0 && boundedBytes > m_ddnBufferCapacityBytes,
                  "Gateway byte-budget invariant violated at DDN " << i);
}

void
AuvEdcExperiment::RecordBufferDrop(uint32_t i, const DataPacket& packet,
                                   const std::string& reason)
{
  ++m_stats.bufferDropped;
  if (packet.priority == Priority::HIGH) ++m_stats.bufferDroppedHigh;
  else if (packet.priority == Priority::MEDIUM) ++m_stats.bufferDroppedMedium;
  else ++m_stats.bufferDroppedLow;
  ReviewedIngressRelease(i, packet.id, closure_ingress::Removal::Dropped,
                          reason == "priority_eviction");
  WriteStorage("drop", i, &packet, reason);
}

void
AuvEdcExperiment::WriteStorageHeader() const
{
  std::ofstream out(m_storageCsv, std::ios::trunc);
  if (!out) NS_FATAL_ERROR("Cannot open storageCsv: " << m_storageCsv);
  out << "time,event,ddnIndex,packetId,priority,payloadBytes,recordBytes,createdAt,ageS,reason,"
      << "queuedRecords,queuedBytes,highWaterRecords,highWaterBytes,capacityRecords,capacityBytes,"
      << "recordOverheadBytes,byteBudgetScope,residentRecords,residentBytes,"
      << "residentHighWaterRecords,residentHighWaterBytes\n";
}

void
AuvEdcExperiment::WriteStorage(const std::string& event, uint32_t i,
                                const DataPacket* packet,
                                const std::string& reason) const
{
  if (m_storageCsv.empty()) return;
  std::ofstream out(m_storageCsv, std::ios::app);
  if (!out) NS_FATAL_ERROR("Cannot append storageCsv: " << m_storageCsv);
  const double now = Simulator::Now().GetSeconds();
  const auto resident = DdnResidentStorage(i);
  out << std::fixed << std::setprecision(6) << now << ',' << event << ',' << i << ',';
  if (packet)
    out << packet->id << ',' << static_cast<uint32_t>(packet->priority) << ','
        << packet->sizeBytes << ',' << RecordStorageBytes(*packet) << ','
        << packet->createdAt << ',' << now - packet->createdAt << ',';
  else out << "-1,-1,0,0,-1,-1,";
  out << reason << ',' << DdnQueueRecords(i) << ',' << DdnQueueBytes(i) << ','
      << m_storageHighWaterRecords[i] << ',' << m_storageHighWaterBytes[i] << ','
      << m_ddnBufferCapacity << ',' << m_ddnBufferCapacityBytes << ','
      << m_ddnRecordOverheadBytes << ',' << m_ddnByteBudgetScope << ','
      << resident.first << ',' << resident.second << ','
      << m_residentHighWaterRecords[i] << ',' << m_residentHighWaterBytes[i] << '\n';
}

closure_adaptive::ChunkDescriptor
AuvEdcExperiment::ReviewedMissionChunk(const DataPacket& packet, uint32_t station) const
{
  return {packet.id, packet.windowNumericId, packet.windowPayloadBytes, station,
          packet.windowChunkIndex, packet.windowChunkCount, packet.sizeBytes,
          packet.createdAt, packet.createdAt + DeadlineSeconds(packet.priority)};
}

void
AuvEdcExperiment::ReviewedTelemetryTrace(const std::string& event, uint32_t station,
    uint64_t sequence, double observedAt, uint64_t bytes, uint64_t windows, const std::string& reason) const
{
  if (m_reviewedTelemetryCsv.empty()) return;
  std::ofstream out(m_reviewedTelemetryCsv, std::ios::app);
  NS_ABORT_MSG_IF(!out, "Cannot append causal telemetry trace");
  out << std::setprecision(17) << Simulator::Now().GetSeconds() << ',' << event << ','
      << station << ',' << sequence << ',' << observedAt << ',' << bytes << ',' << windows << ',' << reason << '\n';
}

void
AuvEdcExperiment::ReviewedTelemetryInitialize()
{
  if (!m_reviewedMission || m_reviewedMissionPolicy != "adaptive") return;
  NS_ABORT_MSG_IF(m_reviewedApps.size() != m_allNodes.GetN(),
                  "Causal telemetry must initialize after actual acoustic forwarding apps");
  SetReviewedControlHandler(1, [this](uint32_t sender, uint32_t receiver, uint64_t sequence,
                                     const std::vector<uint8_t>& bytes) {
    ReviewedTelemetryReceive(sender, receiver, sequence, bytes);
  });
  m_reviewedTelemetrySequences.assign(m_numDdn, 0);
  for (uint32_t station = 0; station < m_numDdn; ++station)
    {
      const double phase = m_reviewedTelemetryPeriodS * station / m_numDdn;
      Simulator::Schedule(Seconds(phase), &AuvEdcExperiment::ReviewedTelemetrySend, this, station);
    }
}

void
AuvEdcExperiment::ReviewedTelemetrySend(uint32_t station)
{
  const double now = Simulator::Now().GetSeconds();
  if (now >= m_simStop) return;
  NS_ABORT_MSG_IF(station >= m_numDdn || station >= m_reviewedTelemetrySequences.size(),
                  "Invalid local telemetry producer");
  // This function executes at the GATEWAY producer. Only its own currently
  // transferable durable queue is read. The controller receives encoded bytes
  // later; neither the snapshot nor any queue is re-read in its RX callback.
  std::vector<closure_adaptive::ChunkDescriptor> chunks;
  for (const auto& packet : m_ddnLowBuffers.at(station))
    if (packet.windowNumericId && packet.windowChunkCount)
      chunks.push_back(ReviewedMissionChunk(packet, station));
  const auto windows = closure_adaptive::completeWindows(chunks, m_ddnRecordOverheadBytes, now);
  closure_adaptive::Report report{station, ++m_reviewedTelemetrySequences[station], now, 0, windows};
  const auto bytes = closure_adaptive::encodeReport(report, m_reviewedControlMaxPayloadBytes);
  const auto bounded = closure_adaptive::decodeReport(bytes, m_reviewedControlMaxPayloadBytes);
  const uint32_t source = m_numBsn + m_numDrn + station;
  const uint32_t auv = m_numBsn + m_numDrn + m_numDdn + m_sinks;
  const auto token = SendReviewedControl(source, auv, 1, report.sequence, bytes);
  if (token) ++m_reviewedTelemetrySent;
  ReviewedTelemetryTrace(token ? "report_enqueued" : "report_local_rejected", station,
      report.sequence, report.observedAt, bytes.size(), bounded.windows.size(),
      "omitted_complete_windows=" + std::to_string(bounded.omittedCompleteWindows));
  if (now + m_reviewedTelemetryPeriodS < m_simStop)
    Simulator::Schedule(Seconds(m_reviewedTelemetryPeriodS),
                        &AuvEdcExperiment::ReviewedTelemetrySend, this, station);
}

void
AuvEdcExperiment::ReviewedTelemetryReceive(uint32_t sender, uint32_t receiver, uint64_t sequence,
                                           const std::vector<uint8_t>& bytes)
{
  const uint32_t first = m_numBsn + m_numDrn;
  const uint32_t auv = first + m_numDdn + m_sinks;
  const double now = Simulator::Now().GetSeconds();
  try
    {
      closure_mission::require(sender >= first && sender < first + m_numDdn && receiver == auv,
                               "Telemetry arrived at the wrong physical endpoint");
      const auto report = closure_adaptive::decodeReport(bytes, m_reviewedControlMaxPayloadBytes);
      closure_mission::require(report.sequence == sequence, "Telemetry body/header sequence mismatch");
      const bool accepted = m_reviewedTelemetryReports.accept(report, sender - first, now);
      if (accepted) ++m_reviewedTelemetryReceived;
      else ++m_reviewedTelemetryObsolete;
      ReviewedTelemetryTrace(accepted ? "report_received" : "report_obsolete", report.station,
          sequence, report.observedAt, bytes.size(), report.windows.size(),
          "received_bytes_only_no_remote_queue_lookup");
    }
  catch (const std::exception& error)
    {
      ++m_reviewedTelemetryInvalid;
      ReviewedTelemetryTrace("report_invalid", sender, sequence, now, bytes.size(), 0, error.what());
    }
}

closure_adaptive::ServicePrediction
AuvEdcExperiment::ReviewedMissionServicePrediction() const
{
  closure_adaptive::ServicePrediction service;
  service.pickupRateBps = m_opticalDataRateBps;
  service.pickupAcquisitionS = ReviewedOpticalServicePredictionSeconds({});
  service.pickupPerRecordS = ReviewedOpticalServicePredictionSeconds({1}) -
      service.pickupAcquisitionS - 8.0 / m_opticalDataRateBps;
  service.offloadRateBps = m_acousticBitRate;
  service.offloadFragmentBytes = m_acousticFramePayloadBytes;
  EdcHeader data; data.m_reviewed = true;
  service.offloadOtherHeaderBytes = data.GetSerializedSize();
  service.offloadAckBytes = data.GetSerializedSize() + m_reviewedAckPayloadBytes;
  data.m_windowMetadata = true;
  service.offloadHeaderBytes = data.GetSerializedSize();
  service.offloadAttempts = m_reviewedMaxAttempts;
  service.offloadPropagationRoundTripS = 2.0 * m_reviewedMissionConstraints.acousticServiceGateM / m_acousticSpeed;
  service.offloadGuardS = m_acousticAckTimeout + m_acousticRetryBackoff;
  // LOW metadata is present in EACH actual DATA fragment header (75 B);
  // scientific payload, HIGH/MED headers and ACK headers remain separate.
  service.validate();
  return service;
}

bool
AuvEdcExperiment::ReviewedMissionAdaptivePlan(double now, const closure_mission::Pose& current, double guardedW)
{
  auto& mission = *m_reviewedMissionModel;
  const auto& constraints = m_reviewedMissionConstraints;
  closure_adaptive::Input input;
  input.now = now; input.horizon = m_simStop; input.current = current; input.home = m_reviewedMissionHome;
  input.remainingJ = mission.remainingEnergyJ(); input.reserveJ = mission.reserveEnergyJ();
  input.guardedPowerW = guardedW; input.recoveryS = m_reviewedMissionRecoveryS;
  input.recoveryJ = m_reviewedMissionRecoveryJ + m_reviewedMissionRecoveryS * m_reviewedSleepPowerW;
  input.carryCapacity = mission.carry.capacityBytes(); input.carryUsed = mission.carry.usedBytes();
  input.coverage = m_reviewedMissionCoverage; input.service = ReviewedMissionServicePrediction();
  input.freshReports = m_reviewedTelemetryReports.fresh(now, m_reviewedTelemetryMaxAgeS);
  std::vector<closure_adaptive::ChunkDescriptor> onboard;
  for (const auto& entry : m_reviewedMissionOnboardChunks) onboard.push_back(entry.second);
  input.onboard = closure_adaptive::completeWindows(onboard, m_ddnRecordOverheadBytes, now);
  std::set<uint64_t> completeIds;
  for (const auto& w : input.onboard) completeIds.insert(w.id);
  for (const auto& entry : m_reviewedMissionOnboardChunks)
    if (!completeIds.count(entry.second.windowId))
      {
        const auto& p = entry.second;
        closure_adaptive::Window partial{p.windowId, p.payloadBytes,
            static_cast<uint64_t>(p.payloadBytes) + m_ddnRecordOverheadBytes,
            p.station, p.generatedAt, p.deadlineAt, {p.payloadBytes}};
        partial.lowInspection = p.lowInspection;
        input.otherOffloadSeconds += input.service.offloadSeconds(partial);
      }

  auto cachedLeg = [&](const std::string& id, const closure_mission::Pose& from,
                       const closure_mission::Pose& to, bool service, double serviceS) {
    std::ostringstream key;
    key << std::setprecision(17) << from.position.x << ':' << from.position.y << ':' << from.position.z << ':' << from.yaw
        << '>' << to.position.x << ':' << to.position.y << ':' << to.position.z << ':' << to.yaw << ':' << service << ':' << serviceS;
    auto found = m_reviewedMissionGeometryCache.find(key.str());
    if (found == m_reviewedMissionGeometryCache.end())
      {
        auto leg = service ? closure_mission::makeOffloadLeg(id, from, {to.position.x, to.position.y, 0},
            to.yaw, serviceS, m_acousticBitRate, constraints) : closure_mission::makeLeg(id, from, to, constraints);
        found = m_reviewedMissionGeometryCache.emplace(key.str(), std::move(leg)).first;
      }
    auto result = found->second; result.id = id; return result;
  };
  auto terminal = [&](const closure_mission::Pose& from) {
    return closure_adaptive::Step{cachedLeg("adaptive-home-return-and-final-service", from,
        m_reviewedMissionHome, true, m_reviewedMissionFinalServiceS), "terminal-offload-return", -1, 0};
  };
  std::vector<closure_adaptive::Option> options;
  auto suffixes = [&](const std::string& id, const std::vector<closure_adaptive::Step>& prefix,
                      const closure_mission::Pose& from) {
    for (uint32_t sink = 0; sink < m_sinkNodes.size(); ++sink)
      {
        try
          {
            auto steps = prefix;
            const auto& fixed = m_sinkNodes[sink].position;
            const closure_mission::Pose destination{{fixed.x, fixed.y, constraints.offloadDepthM}, from.yaw};
            auto offload = cachedLeg(id + "/surface-" + std::to_string(sink), from, destination,
                                     true, m_reviewedMissionServiceS);
            steps.push_back({offload, "offload", -1, 0});
            steps.push_back(terminal(offload.end));
            options.push_back({id + "/surface-" + std::to_string(sink), std::move(steps)});
          }
        catch (const std::invalid_argument&) { /* Geometry-inadmissible options are never scored. */ }
      }
  };
  try { options.push_back({"immediate-home-return", {terminal(current)}}); }
  catch (const std::invalid_argument&) { /* The previously reserved fallback remains available. */ }
  suffixes("immediate-offload", {}, current);
  for (uint32_t first = 0; first < m_numDdn; ++first)
    {
      if (!input.coverage.allows(first)) continue;
      const auto& fixed = m_ddns[first].position;
      // A receding-horizon decision at a just-visited waypoint must move on;
      // a zero-duration pickup cannot become a zero-time event loop.
      if (closure_mission::distance(current.position, {fixed.x, fixed.y, fixed.z}) < 1e-6) continue;
      for (double heading : {0.0, closure_mission::pi})
        {
          try
            {
              const auto id = "pickup-" + std::to_string(first) + (heading == 0 ? "-east" : "-west");
              const closure_mission::Pose target{{fixed.x, fixed.y, fixed.z}, heading};
              auto leg = cachedLeg(id, current, target, false, 0);
              // At most R/v immediately BEFORE the exact gateway passage is
              // guaranteed inside the contact ball by arclength alone. No
              // stationary hold, full descent, or unknown later contact counts.
              const double contact = std::min(leg.durationS(), m_opticalRange / constraints.speedMps);
              std::vector<closure_adaptive::Step> one{{leg, "pickup", static_cast<int32_t>(first), contact,
                                                       m_reviewedOpticalCreditBytes}};
              suffixes(id, one, target);
              auto after = input.coverage;
              if (!after.visit(first, now + leg.durationS())) continue;
              for (uint32_t second = 0; second < m_numDdn; ++second)
                {
                  if (second == first || !after.allows(second)) continue;
                  const auto& next = m_ddns[second].position;
                  for (double nextHeading : {0.0, closure_mission::pi})
                    {
                      try
                        {
                          const auto nextId = id + "/pickup-" + std::to_string(second) + (nextHeading == 0 ? "-east" : "-west");
                          const closure_mission::Pose nextTarget{{next.x, next.y, next.z}, nextHeading};
                          auto secondLeg = cachedLeg(nextId, target, nextTarget, false, 0);
                          auto two = one;
                          two.push_back({secondLeg, "pickup", static_cast<int32_t>(second),
                              std::min(secondLeg.durationS(), m_opticalRange / constraints.speedMps), m_reviewedOpticalCreditBytes});
                          suffixes(nextId, two, nextTarget);
                        }
                      catch (const std::invalid_argument&) { /* No fictitious feasible geometry. */ }
                    }
                }
            }
          catch (const std::invalid_argument&) { /* Inadmissible station/heading remains explicit via catalogue counts. */ }
        }
    }
  const auto selected = closure_adaptive::select(input, options);
  ++m_reviewedMissionAdaptiveDecisions;
  if (!selected.chosen)
    {
      ++m_reviewedMissionAdaptiveNoFeasible;
      ReviewedTelemetryTrace("decision_no_feasible", 0, m_reviewedMissionAdaptiveDecisions, now, 0,
          input.onboard.size(), "catalogue=" + std::to_string(options.size()) + ";reserved_fallback_only");
      return false;
    }
  const auto& choice = *selected.chosen;
  const auto& option = options.at(choice.optionIndex);
  m_reviewedMissionSteps.clear();
  for (const auto& step : option.steps)
    m_reviewedMissionSteps.push_back({step.leg, step.kind, step.station});
  m_reviewedMissionReservedReturn.reset(); // Whole causal-plan suffix is now retained in the deque.
  ReviewedTelemetryTrace("decision_selected", 0, m_reviewedMissionAdaptiveDecisions, now,
      choice.onTimeBytes, choice.onTime.size(), option.id + ";fresh_reports=" + std::to_string(input.freshReports.size()) +
      ";catalogue=" + std::to_string(options.size()) + ";feasible=" + std::to_string(selected.feasible.size()) +
      ";snapshot_prediction_not_delivery");
  return true;
}

void
AuvEdcExperiment::ReviewedMissionInitialize()
{
  if (!m_reviewedMission) return;
  NS_ABORT_MSG_IF(m_sinkNodes.empty() || m_ddns.empty(), "Reviewed mission requires unchanged existing gateways and surface sinks");
  const uint32_t auv = m_numBsn + m_numDrn + m_numDdn + m_sinks;
  NS_ABORT_MSG_IF(auv >= m_reviewedLedgers.size() || !m_reviewedLedgers[auv],
                  "Initialize reviewed mission after authoritative per-node PHY energy binding");
  auto ledger = m_reviewedLedgers[auv];
  const auto& first = m_sinkNodes.front().position;
  m_reviewedMissionHome = {{first.x, first.y, 50.0}, 0.0};
  std::ofstream out(m_reviewedMissionCsv, std::ios::trunc);
  NS_ABORT_MSG_IF(!out, "Cannot open reviewed mission trajectory CSV");
  out << "time,event,leg,x,y,depth,yaw,lengthM,durationS,remainingJ,carryBytes,assessment\n";
  out.flush(); // The first decision appends through a separate stream below.
  if (m_protocol == ProtocolMode::PURE_ACOUSTIC)
    {
      ledger->SetBasePower(0, 0);
      ledger->SetAwake(0, false);
      // The counterfactual has no deployed AUV. Its placeholder ns-3 node is
      // disabled, stationary and excluded from the deployed-vehicle mission.
      out << "0,vehicle_not_deployed,," << first.x << ',' << first.y
          << ",50,0,0,0," << ledger->Budget()->Remaining() << ",0,direct_counterfactual\n";
      return;
    }
  try
    {
      auto& c = m_reviewedMissionConstraints;
      c.maximumX = m_pipelineLength + 500.0;
      c.acousticServiceGateM = std::min({m_sinkRange, m_closureAuvRangeM, m_closureSinkRangeM});
      c.validate();
      closure_mission::RecoveryAllowance recovery{m_reviewedMissionRecoveryS,
          m_reviewedMissionRecoveryJ,
          "Explicit study handling allowance; submerged rendezvous is not a modeled deck recovery"};
      // Declared semantic allocations, not a measurement of host C++ heap RAM.
      // Shared acoustic reassembly is charged to both H-S and H-A; only H-A
      // stores gateway reports and its bounded temporary decoding workspace.
      const uint64_t controlMemory = 2 * static_cast<uint64_t>(m_reviewedControlMaxInflight) *
          (m_reviewedControlMaxPayloadBytes ? static_cast<uint64_t>(m_reviewedControlMaxPayloadBytes) + 64 : 0);
      const uint64_t reportMemory = m_reviewedMissionPolicy == "adaptive"
          ? (static_cast<uint64_t>(m_numDdn) + 2) * m_reviewedControlMaxPayloadBytes : 0;
      const uint64_t reservedMemory = controlMemory + reportMemory +
          (m_reviewedOpticalControl ? ReviewedOpticalReservedMemoryBytes() : 0);
      m_reviewedMissionReservedMemoryBytes = reservedMemory;
      NS_ABORT_MSG_IF(reservedMemory >= m_reviewedMissionCarryBytes,
                      "Explicit control/cache memory leaves no finite vehicle data storage");
      m_reviewedMissionModel = std::make_unique<closure_mission::Mission>(
          m_reviewedMissionHome, c, recovery, m_reviewedMissionCarryBytes - reservedMemory,
          m_reviewedMissionOperationW, 67.0, 0.2, ledger->Budget(), ledger);
      m_reviewedMissionCoverage.maxRevisitS = m_reviewedMissionMaxRevisitS;
      for (uint32_t i = 0; i < m_numDdn; ++i) m_reviewedMissionCoverage.stations.push_back(i);
      // Notify the PHY boundary scheduler of the same baseline already set
      // by Mission; this is a state update, not another energy consumption.
      m_reviewedPhys.at(auv)->SetBasePower(m_reviewedMissionOperationW);
      // Canonical static patrol: visit each physical station eastward, then
      // westward. Every heading reversal is part of a finite checked curve.
      for (uint32_t i = 0; i < m_ddns.size(); ++i)
        m_reviewedMissionVisitOrder.emplace_back(i, 0.0);
      for (int i = static_cast<int>(m_ddns.size()) - 2; i > 0; --i)
        m_reviewedMissionVisitOrder.emplace_back(static_cast<uint32_t>(i), closure_mission::pi);
      if (m_reviewedMissionPolicy == "adaptive")
        {
          std::ofstream telemetry(m_reviewedTelemetryCsv, std::ios::trunc);
          NS_ABORT_MSG_IF(!telemetry, "Cannot open causal telemetry trace");
          telemetry << "time,event,station,sequence,observedAt,payloadBytes,completeWindows,reason\n";
          telemetry.close();
          // Construction precedes app installation. The first catalogue uses
          // zero reports; actual telemetry registration occurs after apps start.
          Simulator::Schedule(NanoSeconds(1), &AuvEdcExperiment::ReviewedTelemetryInitialize, this);
        }
      ReviewedMissionDecision();
    }
  catch (const std::exception& e)
    { NS_FATAL_ERROR("REVIEWED_MISSION_GEOMETRY_OR_ENERGY_GATE_FAILED: " << e.what()); }
}

void
AuvEdcExperiment::ReviewedMissionAdvance(double now)
{
  if (m_reviewedMissionModel)
    {
      NS_ABORT_MSG_IF(now > Simulator::Now().GetSeconds() + 1e-8,
                      "Mission energy may advance only at actual ns-3 event time, not speculative future queries");
      if (now < m_reviewedMissionModel->nowS()) return;
      const auto before = m_reviewedMissionModel->state();
      m_reviewedMissionModel->advance(now);
      const auto after = m_reviewedMissionModel->state();
      if (after != before)
        {
          const uint32_t auv = m_numBsn + m_numDrn + m_numDdn + m_sinks;
          if (after == closure_mission::Mission::State::RecoveryAccountingComplete)
            m_reviewedPhys.at(auv)->SetBasePower(0);
          else m_reviewedPhys.at(auv)->Sync();
        }
      // Diagnostic legacy column only. The authoritative total remains the
      // simultaneous per-owner PHY ledger, with no second packet/base debit.
      m_stats.auvOperationalEnergyJ = m_reviewedMissionModel->operationConsumedJ();
    }
}

Vector
AuvEdcExperiment::ReviewedMissionVelocity(double time) const
{
  if (!m_reviewedMissionModel) return Vector();
  const_cast<AuvEdcExperiment*>(this)->ReviewedMissionAdvance(time);
  const auto v = m_reviewedMissionModel->evaluatedVelocity(time);
  return Vector(v.x, v.y, v.z);
}

void
AuvEdcExperiment::ReviewedMissionRecoveryComplete()
{
  ReviewedMissionAdvance(Simulator::Now().GetSeconds());
}

bool
AuvEdcExperiment::ReviewedMissionCommunicationsAvailable() const
{
  if (!m_reviewedMission) return true;
  const_cast<AuvEdcExperiment*>(this)->ReviewedMissionAdvance(Simulator::Now().GetSeconds());
  return m_reviewedMissionModel && m_reviewedMissionModel->communicationsAvailable();
}

bool
AuvEdcExperiment::ReviewedMissionReserve(const DataPacket& packet)
{
  if (!m_reviewedMission) return true;
  if (!ReviewedMissionCommunicationsAvailable()) return false;
  const bool accepted = m_reviewedMissionModel->carry.reserve(std::to_string(packet.id), RecordStorageBytes(packet));
  if (!accepted) ++m_reviewedMissionCarryRefusals;
  m_reviewedMissionCarryPeak = std::max(m_reviewedMissionCarryPeak, m_reviewedMissionModel->carry.usedBytes());
  return accepted;
}

void
AuvEdcExperiment::ReviewedMissionCommit(uint64_t id, uint32_t station)
{
  if (m_reviewedMission)
    {
      NS_ABORT_MSG_IF(!m_reviewedMissionModel || !m_reviewedMissionModel->carry.commit(std::to_string(id)),
                      "AUV custody requires an existing finite whole-record reservation");
      if (m_reviewedMissionPolicy == "adaptive")
        {
          const auto packet = std::find_if(m_auvBuffer.begin(), m_auvBuffer.end(),
                                           [id](const DataPacket& p) { return p.id == id; });
          NS_ABORT_MSG_IF(packet == m_auvBuffer.end(), "Adaptive onboard descriptor requires a genuinely received local DATA record");
          if (packet->priority == Priority::LOW)
            {
              NS_ABORT_MSG_IF(station >= m_numDdn || !packet->windowNumericId || !packet->windowChunkCount,
                              "Adaptive LOW custody requires decoded producer metadata and actual gateway identity");
              m_reviewedMissionOnboardChunks[id] = ReviewedMissionChunk(*packet, station);
            }
          else
            {
              // An actually received alert/summary is one complete observation;
              // no producer LOW-window metadata is fabricated for it.
              m_reviewedMissionOnboardChunks[id] = {packet->id, packet->id, packet->sizeBytes,
                  station, 0, 1, packet->sizeBytes, packet->createdAt,
                  packet->createdAt + DeadlineSeconds(packet->priority), false};
            }
        }
    }
}

void
AuvEdcExperiment::ReviewedMissionRelease(uint64_t id)
{
  if (m_reviewedMissionModel) m_reviewedMissionModel->carry.release(std::to_string(id));
  m_reviewedMissionOnboardChunks.erase(id);
}

void
AuvEdcExperiment::ReviewedMissionDecision()
{
  if (!m_reviewedMissionModel) return;
  const double now = Simulator::Now().GetSeconds();
  ReviewedMissionAdvance(now);
  auto& mission = *m_reviewedMissionModel;
  if (!mission.communicationsAvailable()) return;
  if (m_reviewedMissionLastKind == "pickup")
    {
      ++m_reviewedMissionGatewayVisits;
      if (m_reviewedMissionMaxRevisitS > 0)
        {
          NS_ABORT_MSG_IF(m_reviewedMissionLastStation < 0,
                          "Completed pickup has no physical coverage station");
          if (!m_reviewedMissionCoverage.allows(m_reviewedMissionLastStation) ||
              !m_reviewedMissionCoverage.visit(m_reviewedMissionLastStation, now))
            ++m_reviewedMissionCoverageViolations;
        }
    }
  if (m_reviewedMissionLastKind == "offload" || m_reviewedMissionLastKind == "terminal-offload-return" ||
      m_reviewedMissionLastKind == "terminal-retry") ++m_reviewedMissionSurfaceServices;
  auto current = mission.evaluatedPose(now);
  if (m_reviewedMissionCanonicalEnd)
    {
      NS_ABORT_MSG_IF(closure_mission::distance(current.position, m_reviewedMissionCanonicalEnd->position) >= 1e-8 ||
          std::abs(closure_mission::angleDifference(current.yaw, m_reviewedMissionCanonicalEnd->yaw)) >= 1e-9,
          "Reviewed mission decision must join the actual completed leg within one scheduler quantum");
      // Stable canonical joins prevent nanometre scheduler rounding from
      // changing a same-pose bounded-curvature return into an extra full turn.
      current = *m_reviewedMissionCanonicalEnd;
    }
  const auto& c = m_reviewedMissionConstraints;
  const auto& sink0 = m_sinkNodes.front().position;
  auto homeService = [&](const closure_mission::Pose& start) {
    return closure_mission::makeOffloadLeg("home-return-and-final-service", start,
        {sink0.x, sink0.y, sink0.z}, m_reviewedMissionHome.yaw,
        m_reviewedMissionFinalServiceS, m_acousticBitRate, c);
  };
  // This conservative guard reserves simultaneous radio + optical load over
  // the whole suffix, not just a guessed number of successful packets.
  const double opticalPeakW = m_reviewedOpticalControl
      ? std::max(m_reviewedOpticalVehicleStartupW, m_opticalRxPowerW) / m_reviewedOpticalEfficiency
      : m_opticalRxPowerW;
  const double additionalW = std::max({m_closureAcousticTxPowerW,
      m_closureAcousticRxPowerW, m_reviewedIdlePowerW, m_reviewedSleepPowerW}) + opticalPeakW;
  const double guardedW = m_reviewedMissionOperationW + additionalW;
  auto fits = [&](double seconds) {
    return now + seconds + m_reviewedMissionRecoveryS <= m_simStop + 1e-8 &&
      mission.remainingEnergyJ() + 1e-6 >= seconds * guardedW +
        m_reviewedMissionRecoveryJ + m_reviewedMissionRecoveryS * m_reviewedSleepPowerW + mission.reserveEnergyJ();
  };
  try
    {
      if (m_reviewedMissionReturning && m_reviewedMissionSteps.empty())
        {
          const bool receiverBusy = m_reviewedOpticalControl ? ReviewedOpticalReceiverBusy() : !m_pendingOptical.empty();
          const bool custodySettled = mission.carry.usedBytes() == 0 && m_auvBuffer.empty() &&
                                     m_pendingAuvSurface.empty() && !receiverBusy;
          const Time recoveryEnd = Simulator::Now() + Seconds(m_reviewedMissionRecoveryS);
          if (custodySettled && mission.beginExternalRecoveryAccounting(now, true, recoveryEnd.GetSeconds()))
            {
              const uint32_t auv = m_numBsn + m_numDrn + m_numDdn + m_sinks;
              m_reviewedPhys.at(auv)->PowerOff();
              m_reviewedMissionRecoveryScheduled = true;
              m_reviewedMissionLastKind = "external-recovery-accounting";
              Simulator::Schedule(recoveryEnd - Simulator::Now(),
                  &AuvEdcExperiment::ReviewedMissionRecoveryComplete, this);
              return;
            }
          auto retry = closure_mission::makeLoiter("home-paid-final-retry", current, c);
          if (fits(retry.durationS())) m_reviewedMissionSteps.push_back({retry, "terminal-retry"});
          else
            {
              // No success is fabricated. The powered vehicle remains in its
              // continuous home loiter; final report explicitly fails closure.
              m_reviewedMissionLastKind = "final-transfer-unconfirmed";
              return;
            }
        }
      if (m_reviewedMissionPolicy == "adaptive" && !m_reviewedMissionReturning)
        {
          const bool replanned = ReviewedMissionAdaptivePlan(now, current, guardedW);
          if (!replanned && m_reviewedMissionSteps.empty())
            {
              // Initial no-feasible catalogue or finite coverage infeasibility:
              // execute only the explicit geometrically safe terminal fallback.
              // No result is relabelled as a feasible coverage-serving plan.
              auto terminal = homeService(current);
              NS_ABORT_MSG_IF(!fits(terminal.durationS()),
                              "Adaptive controller has no energy/time-feasible initial return fallback");
              if (m_reviewedMissionMaxRevisitS > 0 &&
                  !m_reviewedMissionCoverage.withinBound(now + terminal.durationS()))
                ++m_reviewedMissionCoverageViolations;
              m_reviewedMissionSteps.push_back({terminal, "terminal-offload-return", -1});
            }
        }
      if (m_reviewedMissionSteps.empty() && m_reviewedMissionPolicy == "static")
        {
          if (m_reviewedMissionMaxRevisitS > 0)
            {
              std::size_t skipped = 0;
              while (!m_reviewedMissionCoverage.allows(m_reviewedMissionVisitOrder.at(m_reviewedMissionVisitCursor).first) &&
                     skipped++ < m_reviewedMissionVisitOrder.size())
                m_reviewedMissionVisitCursor = (m_reviewedMissionVisitCursor + 1) % m_reviewedMissionVisitOrder.size();
              NS_ABORT_MSG_IF(skipped > m_reviewedMissionVisitOrder.size(), "Static coverage traversal has no eligible station");
            }
          const auto visit = m_reviewedMissionVisitOrder.at(m_reviewedMissionVisitCursor);
          const auto& gateway = m_ddns.at(visit.first).position;
          closure_mission::Pose target{{gateway.x, gateway.y, gateway.z}, visit.second};
          auto pickup = closure_mission::makeLeg("static-gateway-" + std::to_string(visit.first), current, target, c);
          uint32_t nearest = 0;
          for (uint32_t s = 1; s < m_sinkNodes.size(); ++s)
            if (CalculateDistance(gateway, m_sinkNodes[s].position) < CalculateDistance(gateway, m_sinkNodes[nearest].position)) nearest = s;
          const auto& sink = m_sinkNodes[nearest].position;
          auto offload = closure_mission::makeOffloadLeg("static-existing-sink-" + std::to_string(nearest), target,
              {sink.x, sink.y, sink.z}, visit.second, m_reviewedMissionServiceS, m_acousticBitRate, c);
          auto suffix = homeService(offload.end);
          bool coverageFits = true;
          if (m_reviewedMissionMaxRevisitS > 0)
            {
              auto predictedCoverage = m_reviewedMissionCoverage;
              coverageFits = predictedCoverage.visit(visit.first, now + pickup.durationS()) &&
                  predictedCoverage.withinBound(now + pickup.durationS() + offload.durationS() + suffix.durationS());
            }
          if (coverageFits && fits(pickup.durationS() + offload.durationS() + suffix.durationS()))
            {
              // Commit the same certified fallback whose time/energy passed
              // the pickup admission gate. Re-solving a forward-only return
              // from a nanometre of scheduler overshoot can add a whole turn.
              m_reviewedMissionReservedReturn = suffix;
              m_reviewedMissionSteps.push_back({pickup, "pickup", static_cast<int32_t>(visit.first)});
              m_reviewedMissionSteps.push_back({offload, "offload"});
              m_reviewedMissionVisitCursor = (m_reviewedMissionVisitCursor + 1) % m_reviewedMissionVisitOrder.size();
            }
          else
            {
              const auto terminal = m_reviewedMissionReservedReturn
                  ? *m_reviewedMissionReservedReturn : homeService(current);
              NS_ABORT_MSG_IF(closure_mission::distance(current.position, terminal.start.position) >= 1e-8 ||
                  std::abs(closure_mission::angleDifference(current.yaw, terminal.start.yaw)) >= 1e-9,
                  "REVIEWED_MISSION_RESERVED_RETURN_POSE_MISMATCH: committed fallback is not at the actual completed offload pose");
              NS_ABORT_MSG_IF(!fits(terminal.durationS()),
                  "REVIEWED_MISSION_RETURN_GATE_FAILED: the configured mission cannot retain its explicit return/service/recovery/reserve suffix");
              m_reviewedMissionReturning = true;
              m_reviewedMissionSteps.push_back({terminal, "terminal-offload-return"});
            }
        }
      const auto step = m_reviewedMissionSteps.front();
      if (step.kind == "terminal-offload-return") m_reviewedMissionReturning = true;
      double suffixS = 0;
      for (std::size_t i = 1; i < m_reviewedMissionSteps.size(); ++i) suffixS += m_reviewedMissionSteps[i].leg.durationS();
      if (!m_reviewedMissionReturning && m_reviewedMissionPolicy == "static")
        {
          NS_ABORT_MSG_IF(!m_reviewedMissionReservedReturn,
                          "Reviewed pickup lacks its committed return suffix");
          suffixS += m_reviewedMissionReservedReturn->durationS();
        }
      const double suffixJ = suffixS * guardedW + step.leg.durationS() * additionalW +
                            m_reviewedMissionRecoveryS * m_reviewedSleepPowerW;
      NS_ABORT_MSG_IF(!mission.startLeg(step.leg, now, suffixJ),
                      "REVIEWED_MISSION_START_GATE_FAILED: no energy-feasible continuous leg with complete terminal suffix");
      m_reviewedMissionSteps.pop_front();
      m_reviewedMissionLastKind = step.kind;
      m_reviewedMissionLastStation = step.station;
      m_reviewedMissionCanonicalEnd = step.leg.end;
      ++m_reviewedMissionLegsStarted;
      std::ofstream out(m_reviewedMissionCsv, std::ios::app);
      out << std::setprecision(15) << now << ',' << step.kind << ',' << step.leg.id << ','
          << current.position.x << ',' << current.position.y << ',' << current.position.z << ',' << current.yaw << ','
          << step.leg.lengthM << ',' << step.leg.durationS() << ',' << mission.remainingEnergyJ() << ','
          << mission.carry.usedBytes() << ",engineering_C1_3kn_min20m_R3D50m_pitch45deg_calm_water_not_vendor_certification\n";
      const double exactEnd = now + step.leg.durationS();
      Time eventEnd = Seconds(exactEnd);
      if (eventEnd.GetSeconds() < exactEnd) eventEnd += NanoSeconds(1);
      // Dispatch no earlier than the analytic leg endpoint, with at most one
      // scheduler time quantum of representation difference. The callback
      // always reads actual Simulator::Now(), never a captured future double.
      Simulator::Schedule(eventEnd - Simulator::Now(), &AuvEdcExperiment::ReviewedMissionDecision, this);
    }
  catch (const std::exception& e) { NS_FATAL_ERROR("REVIEWED_MISSION_GATE_FAILED: " << e.what()); }
}

void
AuvEdcExperiment::ReviewedMissionSummary() const
{
  if (!m_reviewedMission) return;
  std::cout << "REVIEWED_MISSION policy=" << m_reviewedMissionPolicy << " fixedSpeedMps=" << closure_mission::fixedThreeKnotsMps
            << " gatewayVisits=" << m_reviewedMissionGatewayVisits << " serviceStops=" << m_reviewedMissionSurfaceServices
            << " carryPeakBytes=" << m_reviewedMissionCarryPeak << " carryRefusals=" << m_reviewedMissionCarryRefusals
            << " carryResidualBytes=" << (m_reviewedMissionModel ? m_reviewedMissionModel->carry.usedBytes() : 0)
            << " totalMemoryBytes=" << m_reviewedMissionCarryBytes
            << " reservedControlMemoryBytes=" << m_reviewedMissionReservedMemoryBytes
            << " dataCarryCapacityBytes=" << (m_reviewedMissionModel ? m_reviewedMissionModel->carry.capacityBytes() : 0)
            << " maximumRevisitS=" << m_reviewedMissionMaxRevisitS
            << " coverageEnabled=" << (m_reviewedMissionMaxRevisitS > 0 ? 1 : 0)
            << " coveredStations=" << m_reviewedMissionCoverage.lastVisits.size()
            << " coverageAllStationsVisited=" << (m_reviewedMissionMaxRevisitS > 0 &&
                 m_reviewedMissionCoverage.lastVisits.size() == m_numDdn ? 1 : 0)
            << " coverageViolations=" << m_reviewedMissionCoverageViolations
            << " adaptiveDecisions=" << m_reviewedMissionAdaptiveDecisions
            << " adaptiveNoFeasible=" << m_reviewedMissionAdaptiveNoFeasible
            << " telemetrySent=" << m_reviewedTelemetrySent << " telemetryReceived=" << m_reviewedTelemetryReceived
            << " telemetryObsolete=" << m_reviewedTelemetryObsolete << " telemetryInvalid=" << m_reviewedTelemetryInvalid
            << " terminalAction=" << m_reviewedMissionLastKind
            << " endState=" << (m_reviewedMissionModel ? m_reviewedMissionModel->endStateReason() : "vehicle_not_deployed")
            << " engineeringEnvelopeOnly=1 onboardCustodySettledAtRecovery="
            << (m_reviewedMissionModel && m_reviewedMissionRecoveryScheduled ? 1 : 0) << '\n';
}

void
AuvEdcExperiment::AuvTick()
{
  double now = Simulator::Now().GetSeconds();
  if (m_reviewedMission) ReviewedMissionAdvance(now);
  Vector auvPos = GetAuvPosition(now);
  if (!m_reviewedMission) m_auvMobility->SetPosition(auvPos);
  if (!m_phyEnergyCsv.empty()) ObservePhyEnergy();
  if (!m_diagnosticsCsv.empty()) ObserveGatewayDiagnostics(now, auvPos);
  const double queueDt = std::max(0.0, std::min(m_tick, m_simStop - now));
  const uint64_t ddnBufferedNow = BufferedAtDdn();
  m_stats.maxDdnBuffered = std::max(m_stats.maxDdnBuffered, ddnBufferedNow);
  m_stats.maxAuvBuffered = std::max(
      m_stats.maxAuvBuffered, static_cast<uint64_t>(m_auvBuffer.size()));
  m_stats.ddnQueuePacketSeconds += ddnBufferedNow * queueDt;
  m_stats.auvQueuePacketSeconds += m_auvBuffer.size() * queueDt;
  m_stats.queueObservationSeconds += queueDt;
  if (m_includeMobilityIdleEnergy && !m_reviewedMission)
    {
      double speed = m_auvSpeedKmh / 3.6;
      if (m_completePatrolInSimTime)
        {
          speed = std::max(speed, 2.0 * m_pipelineLength / std::max(m_simStop - m_helloTimeout, 1.0));
        }
      const double auvMobilityEnergyJ =
          (m_protocol == ProtocolMode::PURE_ACOUSTIC)
              ? 0.0
              : (m_auvInstantaneousPower
                    ? AuvOperationalEnergy(now, queueDt)
                    : (m_auvFixedPowerW + m_auvPropulsionCoeff * std::pow(speed, 3.0)) * m_tick);
      m_stats.auvOperationalEnergyJ += auvMobilityEnergyJ;
      const double nodeIdleEnergyJ =
          m_nodeIdlePowerW * static_cast<double>(m_totalNodes) * m_tick;
      ChargeGlobalEnergy(NodeRole::AUV, auvMobilityEnergyJ);
      double underwaterNodes = static_cast<double>(std::max(1u, m_numBsn + m_numDrn + m_numDdn));
      ChargeGlobalEnergy(NodeRole::BSN, nodeIdleEnergyJ * m_numBsn / underwaterNodes);
      ChargeGlobalEnergy(NodeRole::DRN, nodeIdleEnergyJ * m_numDrn / underwaterNodes);
      ChargeGlobalEnergy(NodeRole::DDN, nodeIdleEnergyJ * m_numDdn / underwaterNodes);
    }

  if (m_tracePositions)
    {
      std::cout << std::fixed << std::setprecision(2)
                << "t=" << now << " AUV=(" << auvPos.x << "," << auvPos.y << ","
                << auvPos.z << ") buffer=" << m_auvBuffer.size() << "\n";
    }

  if (m_protocol == ProtocolMode::PURE_ACOUSTIC)
    {
      for (uint32_t i = 0; i < m_numDdn; ++i)
        {
          if (m_matchHybridQueues)
            {
              // Keep queued records in the finite per-class buffer until the
              // non-preemptive direct transmitter becomes free. This avoids a
              // secondary FIFO queue masking later HIGH arrivals.
              if (!m_directActive[i] && m_directWaiting[i].empty())
                {
                  DataPacket packet;
                  if (PopNextDdnPacket(i, packet))
                    {
                      QueueDirectAcoustic(i, packet);
                    }
                }
              continue;
            }
          uint32_t directMoved = 0;
          uint64_t directBits = 0;
          const uint64_t directCapacityBits =
              static_cast<uint64_t>(std::floor(m_acousticBitRate * m_tick));
          while (directBits < directCapacityBits &&
                 (m_sinkContactPacketsPerTick == 0 ||
                  directMoved < m_sinkContactPacketsPerTick) &&
                 !m_ddnBuffers[i].empty())
            {
              const DataPacket pkt = m_ddnBuffers[i].front();
              QueueDirectAcoustic(i, pkt);
              m_ddnBuffers[i].pop_front();
              directMoved++;
              directBits += static_cast<uint64_t>(pkt.sizeBytes) * 8;
            }
        }

      if (now + m_tick <= m_simStop)
        {
          Simulator::Schedule(Seconds(m_tick), &AuvEdcExperiment::AuvTick, this);
        }
      return;
    }

  const bool missionServiceAvailable = ReviewedMissionCommunicationsAvailable();
  for (uint32_t i = 0; i < m_ddns.size(); ++i)
    {
      // The collection medium is an independent experimental factor. Optical
      // service uses the finite UWOC contact; acoustic service uses the same
      // event-driven DDN->AUV path as the scheduled baseline.
      const bool useOptical = (m_auvCollectionMedium == "optical");
      const double effectiveRange = useOptical ? m_opticalRange
                                               : m_ddnCollectionRange;
      if (!missionServiceAvailable || !InRange(auvPos, m_ddns[i].position, effectiveRange))
        {
          continue;
        }
      uint32_t moved = 0;
      uint64_t movedBits = 0;

      if (!useOptical)
        {
          const uint64_t contactCapacityBits =
              static_cast<uint64_t>(std::floor(m_acousticBitRate * m_tick));
          DataPacket packet;
          while (movedBits < contactCapacityBits &&
                 (m_ddnContactPacketsPerTick == 0 || moved < m_ddnContactPacketsPerTick) &&
                 PopNextDdnPacket(i, packet))
            {
              movedBits += static_cast<uint64_t>(packet.sizeBytes) * 8;
              QueueMobileAcoustic(i, packet);
              moved++;
            }
          continue;
        }

      TryStartOpticalTransfer(i);
    }

  // CONTRIBUTION: priority-based direct DDN→sink delivery.
  //
  //  HIGH   (10%) -> reserved emergency acoustic fallback after tau_H.
  //  MEDIUM (30%) -> aging-aware opportunistic fallback after tau_M, admitted
  //                  only when no HIGH backlog exists and rho_max leaves room.
  //  LOW    (60%) -> best-effort AUV/UWOC collection, no direct acoustic fallback.
  //
  // This gives the optimizer x=(h, s_DDN, tau_H, tau_M, rho_max, v_AUV) while
  // keeping emergency traffic protected from routine acoustic fallback.
  if (m_protocol == ProtocolMode::CONTRIBUTION && m_enableCriticalDirect)
    {
      constexpr double HIGH_TRAFFIC_FRACTION = 0.10;
      constexpr double LOW_TRAFFIC_FRACTION = 0.60;
      const double highReservedLoad = HIGH_TRAFFIC_FRACTION * m_trafficLoad;
      const double mediumAdmittedLoad = MediumFallbackRefillRate();
      if (mediumAdmittedLoad > 0.0)
        {
          const double tokenCap =
              std::max(1.0, mediumAdmittedLoad * m_mediumFallbackBurstSeconds);
          m_mediumFallbackTokens =
              std::min(tokenCap, m_mediumFallbackTokens + mediumAdmittedLoad * m_tick);
        }
      else
        {
          m_mediumFallbackTokens = 0.0;
        }
      const double lowOfferedLoad = LOW_TRAFFIC_FRACTION * m_trafficLoad;
      const double lowFallbackCapLoad = m_lowFallbackRhoMax * m_trafficLoad;
      const double lowAdmittedLoad =
          std::min(lowOfferedLoad,
                   std::max(0.0, lowFallbackCapLoad - highReservedLoad - mediumAdmittedLoad));
      if (lowAdmittedLoad > 0.0)
        {
          const double tokenCap =
              std::max(1.0, lowAdmittedLoad * m_lowFallbackBurstSeconds);
          m_lowFallbackTokens =
              std::min(tokenCap, m_lowFallbackTokens + lowAdmittedLoad * m_tick);
        }
      else
        {
          m_lowFallbackTokens = 0.0;
        }

      for (uint32_t i = 0; i < m_numDdn; ++i)
        {
          // Suppress fallback only while the selected AUV collection service is
          // actually possible at this gateway.
          const double collectionRange =
              (m_auvCollectionMedium == "optical") ? m_opticalRange
                                                    : m_ddnCollectionRange;
          if (missionServiceAvailable && InRange(auvPos, m_ddns[i].position, collectionRange))
            {
              if (!m_diagnosticsCsv.empty() && m_enableMediumFallback &&
                  !m_ddnMediumBuffers[i].empty() &&
                  now - m_ddnMediumBuffers[i].front().createdAt >= m_mediumFallbackTimeout)
                {
                  m_gatewayDiagnostics[i].mediumEligibleSeconds += queueDt;
                  m_gatewayDiagnostics[i].mediumContactDeferredSeconds += queueDt;
                }
              continue;
            }

          auto directDeliver = [&](const DataPacket& pkt) -> bool {
            QueueDirectAcoustic(i, pkt);
            return true;
          };

          uint32_t directMoved = 0;
          uint64_t directBits = 0;
          const uint64_t directCapacityBits =
              static_cast<uint64_t>(std::floor(m_acousticBitRate * m_tick));

          // HIGH: direct delivery after the high-priority threshold.
          while (m_enableHighFallback &&
                 directBits < directCapacityBits &&
                 (m_sinkContactPacketsPerTick == 0 ||
                  directMoved < m_sinkContactPacketsPerTick) &&
                 !m_ddnHighBuffers[i].empty())
            {
              DataPacket& front = m_ddnHighBuffers[i].front();
              if (now - front.createdAt < m_highFallbackTimeout) break;
              DataPacket pkt = front;
              directMoved++;
              directBits += static_cast<uint64_t>(pkt.sizeBytes) * 8;
              if (!directDeliver(pkt)) continue;
              m_ddnHighBuffers[i].pop_front();
            }

          // MEDIUM: opportunistic aging fallback. It is blocked whenever any
          // HIGH packet is waiting anywhere in the DDN layer.
          if (!m_diagnosticsCsv.empty() && m_enableMediumFallback &&
              !m_ddnMediumBuffers[i].empty() &&
              now - m_ddnMediumBuffers[i].front().createdAt >= m_mediumFallbackTimeout)
            {
              auto& diagnostic = m_gatewayDiagnostics[i];
              diagnostic.mediumEligibleSeconds += queueDt;
              // Exclusive reasons, evaluated at this gateway's actual turn
              // in the fixed-order scheduler. HIGH takes precedence.
              if (HasHighBacklog()) diagnostic.mediumHighBlockedSeconds += queueDt;
              else if (m_mediumFallbackTokens < 1.0)
                diagnostic.mediumTokenBlockedSeconds += queueDt;
            }
          while (m_enableMediumFallback &&
                 directBits < directCapacityBits &&
                 (m_sinkContactPacketsPerTick == 0 ||
                  directMoved < m_sinkContactPacketsPerTick) &&
                 m_mediumFallbackTokens >= 1.0 &&
                 !HasHighBacklog() &&
                 !m_ddnMediumBuffers[i].empty())
            {
              DataPacket& front = m_ddnMediumBuffers[i].front();
              if (now - front.createdAt < m_mediumFallbackTimeout) break;
              DataPacket pkt = front;
              // Optional narrow admission ablation: retain the submitted
              // queue and gate semantics, but avoid spending a token on a
              // candidate that is already expired when otherwise eligible
              // for service. Other queued records are not proactively purged.
              if (m_purgeExpiredMediumBeforeAdmission && DeadlineExpired(pkt))
                {
                  m_ddnMediumBuffers[i].pop_front();
                  RecordDeadlineExpiry(pkt);
                  if (!m_diagnosticsCsv.empty())
                    m_gatewayDiagnostics[i].mediumPurgedBeforeAdmission++;
                  TracePacket("deadline_expired", pkt, static_cast<int32_t>(i),
                              -1, 0, -1.0, "before_admission");
                  continue;
                }
              directMoved++;
              directBits += static_cast<uint64_t>(pkt.sizeBytes) * 8;
              m_mediumFallbackTokens -= 1.0;
              if (!m_diagnosticsCsv.empty())
                {
                  auto& diagnostic = m_gatewayDiagnostics[i];
                  diagnostic.mediumAdmissions++;
                  diagnostic.mediumTokensCharged++;
                  if (DeadlineExpired(pkt)) diagnostic.mediumTokensChargedExpired++;
                }
              if (!directDeliver(pkt)) continue;
              m_ddnMediumBuffers[i].pop_front();
            }

          // LOW: experimental opportunistic fallback only after all higher-priority
          // queues are empty and an explicit LOW rho budget is configured.
          while (directBits < directCapacityBits &&
                 (m_sinkContactPacketsPerTick == 0 ||
                  directMoved < m_sinkContactPacketsPerTick) &&
                 m_lowFallbackTokens >= 1.0 &&
                 !HasHighBacklog() &&
                 !HasMediumBacklog() &&
                 !m_ddnLowBuffers[i].empty())
            {
              DataPacket& front = m_ddnLowBuffers[i].front();
              if (now - front.createdAt < m_lowFallbackTimeout) break;
              DataPacket pkt = front;
              directMoved++;
              directBits += static_cast<uint64_t>(pkt.sizeBytes) * 8;
              m_lowFallbackTokens -= 1.0;
              if (!directDeliver(pkt)) continue;
              m_ddnLowBuffers[i].pop_front();
            }
        }
    }

  // Deliver buffered data to a sink when the AUV's sinusoidal path brings it to
  // the surface (midpoint between DDNs).  The check is a plain range test; with
  // the corrected sinusoidal z-path the AUV reaches z≈0 exactly at sink positions.
  int32_t nearSinkIndex = -1;
  double nearSinkDistance = std::numeric_limits<double>::max();
  for (uint32_t s = 0; s < m_sinkNodes.size(); ++s)
    {
      const double distance = CalculateDistance(auvPos, m_sinkNodes[s].position);
      if (distance <= m_sinkRange && distance < nearSinkDistance)
        {
          nearSinkIndex = static_cast<int32_t>(s);
          nearSinkDistance = distance;
        }
    }
  if (missionServiceAvailable && nearSinkIndex >= 0 && m_pendingAuvSurface.empty() && !m_auvBuffer.empty())
    {
      DataPacket packet = m_auvBuffer.front();
      m_auvBuffer.pop_front();
      QueueAuvSurface(packet, static_cast<uint32_t>(nearSinkIndex));
    }

  if (now + m_tick <= m_simStop)
    {
      Simulator::Schedule(Seconds(m_tick), &AuvEdcExperiment::AuvTick, this);
    }
}

Vector
AuvEdcExperiment::GetAuvPosition(double time) const
{
  if (m_reviewedMission)
    {
      if (!m_reviewedMissionModel)
        {
          NS_ABORT_MSG_IF(m_sinkNodes.empty(), "Reviewed mission initial pose requires existing surface topology");
          return Vector(m_sinkNodes.front().position.x, m_sinkNodes.front().position.y, 50.0);
        }
      const_cast<AuvEdcExperiment*>(this)->ReviewedMissionAdvance(time);
      const auto p = m_reviewedMissionModel->evaluatedPosition(time);
      return Vector(p.x, p.y, p.z);
    }
  double speed = m_auvSpeedKmh / 3.6;
  // Use the actual span between the first and last DDN so the sinusoidal depth
  // profile hits DDN contact depth at every DDN x-position regardless of whether
  // DDNs cover the full pipeline length (important when sparse N < 126 on the
  // fixed 12600 m pipeline places the last DDN before the pipeline endpoint).
  double ddnSpan = (m_numDdn > 1)
                       ? m_ddns.back().position.x - m_ddns.front().position.x
                       : m_pipelineLength;
  double ddnSpacing = (m_numDdn > 1) ? ddnSpan / (m_numDdn - 1) : ddnSpan;

  // Paper (Sec. III-B): AUV follows a sinusoidal path in the vertical plane.
  //   - At each DDN x-position the AUV descends to DDN depth (contact).
  //   - At midpoints between DDNs the AUV ascends to the surface (sink delivery).
  // The commanded AUV speed is path speed, not free horizontal speed.  Deep DDNs
  // therefore reduce horizontal progress; EOM-lifted DDNs shorten the cycle.
  double ddnContactDepth = (m_protocol == ProtocolMode::CONTRIBUTION ||
                            m_protocol == ProtocolMode::PURE_ACOUSTIC)
                               ? std::max(0.0, m_depth - m_ddnEomOffset)
                               : m_depth;
  double stretch = SinusoidalPathStretch(ddnContactDepth, ddnSpacing);
  if (m_completePatrolInSimTime)
    {
      speed = std::max(speed,
                       2.0 * m_pipelineLength * stretch /
                           std::max(m_simStop - m_helloTimeout, 1.0));
    }
  double horizontalSpeed = speed / stretch;
  double pathPeriod = 2.0 * m_pipelineLength / horizontalSpeed;

  // Start the AUV at the first sink position (x = ddnSpacing/2, z = 0) instead
  // of at DDN-0 (x=0).  DDN-0 has an empty buffer at t=0, so starting there
  // wastes the first AUV contact window.  Starting at the midpoint means the
  // AUV first visits a DDN *after* it has had time to accumulate packets, and
  // the simulation ends with the AUV back near a sink rather than near an
  // undelivered DDN buffer.
  double startOffset = (ddnSpacing / 2.0) / horizontalSpeed; // time to reach first sink
  double t = std::fmod(time + startOffset, pathPeriod);
  double x = (t <= m_pipelineLength / horizontalSpeed)
                 ? horizontalSpeed * t
                 : 2.0 * m_pipelineLength - horizontalSpeed * t;
  double z = 0.5 * ddnContactDepth * (1.0 + std::cos(2.0 * M_PI * x / ddnSpacing));

  // Optional lateral oscillation (m_auvAmplitude defaults to 0).
  double y = m_auvAmplitude * std::sin(2.0 * M_PI * x / ddnSpacing);

  return Vector(x, y, z);
}

double
AuvEdcExperiment::AuvInstantaneousSpeed(double time) const
{
  if (m_reviewedMission) return CalculateDistance(ReviewedMissionVelocity(time), Vector());
  // Derivative norm of the unchanged legacy x-z path. Commanded speed is
  // its period-averaged arc speed, not its instantaneous speed.
  const double horizontal = m_motionHorizontalSpeed;
  const double halfPeriod = m_pipelineLength / horizontal;
  const double t = std::fmod(time + m_motionSpacing / (2.0 * horizontal),
                             2.0 * halfPeriod);
  const double x = t <= halfPeriod ? horizontal * t
                                   : 2.0 * m_pipelineLength - horizontal * t;
  const double slope = m_motionContactDepth * M_PI / m_motionSpacing *
                       std::sin(2.0 * M_PI * x / m_motionSpacing);
  return horizontal * std::sqrt(1.0 + slope * slope);
}

double
AuvEdcExperiment::AuvPeakSpeed() const
{
  if (m_reviewedMission) return m_protocol == ProtocolMode::PURE_ACOUSTIC ? 0.0 : closure_mission::fixedThreeKnotsMps;
  const double slopeBound = m_motionContactDepth * M_PI / m_motionSpacing;
  return m_motionHorizontalSpeed * std::sqrt(1.0 + slopeBound * slopeBound);
}

double
AuvEdcExperiment::AuvOperationalEnergy(double start, double duration) const
{
  if (duration <= 0.0) return 0.0;
  // Five-point Gauss-Legendre quadrature, subdivided to <=1 second and <=
  // 1/128 of the vertical spatial period. Split at path turnarounds, where
  // the derivative of the speed magnitude need not be continuous. No RNG
  // calls or simulation events are introduced. Fixed power is integrated
  // over the actual interval (including a final fractional scheduler tick).
  constexpr double nodes[] = {-0.9061798459386640, -0.5384693101056831,
                              0.0, 0.5384693101056831, 0.9061798459386640};
  constexpr double weights[] = {0.2369268850561891, 0.4786286704993665,
                                0.5688888888888889, 0.4786286704993665,
                                0.2369268850561891};
  const double panelMax = std::min(1.0, m_motionSpacing /
                                      (128.0 * m_motionHorizontalSpeed));
  const double halfPeriod = m_pipelineLength / m_motionHorizontalSpeed;
  const double offset = m_motionSpacing / (2.0 * m_motionHorizontalSpeed);
  const double end = start + duration;
  double energy = 0.0;
  for (double left = start; left < end; )
    {
      double nextTurn = (std::floor((left + offset) / halfPeriod) + 1.0) *
                       halfPeriod - offset;
      if (nextTurn <= left + 1e-9) nextTurn += halfPeriod;
      const double right = std::min({end, left + panelMax, nextTurn});
      const double half = 0.5 * (right - left);
      const double mid = 0.5 * (right + left);
      for (uint32_t k = 0; k < 5; ++k)
        {
          const double speed = AuvInstantaneousSpeed(mid + half * nodes[k]);
          energy += half * weights[k] *
                    (m_auvFixedPowerW + m_auvPropulsionCoeff * speed * speed * speed);
        }
      left = right;
    }
  return energy;
}

double
AuvEdcExperiment::SinusoidalPathStretch(double contactDepth, double spacing)
{
  if (contactDepth <= 0.0 || spacing <= 0.0)
    {
      return 1.0;
    }

  constexpr uint32_t samples = 512;
  double sum = 0.0;
  for (uint32_t i = 0; i < samples; ++i)
    {
      double u = (static_cast<double>(i) + 0.5) / static_cast<double>(samples);
      double dzdx = -contactDepth * M_PI / spacing * std::sin(2.0 * M_PI * u);
      sum += std::sqrt(1.0 + dzdx * dzdx);
    }
  return sum / static_cast<double>(samples);
}

uint32_t
AuvEdcExperiment::NearestDdn(double x) const
{
  uint32_t best = 0;
  double bestDistance = std::numeric_limits<double>::max();
  for (uint32_t i = 0; i < m_ddns.size(); ++i)
    {
      double distance = std::abs(m_ddns[i].position.x - x);
      if (distance < bestDistance)
        {
          best = i;
          bestDistance = distance;
        }
    }
  return best;
}

uint32_t
AuvEdcExperiment::AssociatedDrn(uint32_t bsnIndex) const
{
  if (m_protocol == ProtocolMode::CONTRIBUTION || m_protocol == ProtocolMode::PURE_ACOUSTIC)
    {
      // Return the paired DRN (co-located with the nearest DDN in x).
      // BSN greedy forwarding routes through relay DRNs to reach it.
      double   bsnX   = m_bsns[bsnIndex].position.x;
      uint32_t ddnIdx = NearestDdn(bsnX);
      double   ddnX   = m_ddns[ddnIdx].position.x;
      uint32_t best   = 0;
      double   bestD  = std::numeric_limits<double>::max();
      for (uint32_t i = 0; i < m_drns.size(); ++i)
        {
          double d = std::abs(m_drns[i].position.x - ddnX);
          if (d < bestD) { bestD = d; best = i; }
        }
      return best;
    }
  // BASELINE: nearest DDN index (used as proxy for relay endpoint).
  return std::min(NearestDdn(m_bsns[bsnIndex].position.x),
                  static_cast<uint32_t>(m_drns.size() - 1));
}

uint32_t
AuvEdcExperiment::DdnForDrn(uint32_t drnIndex) const
{
  if (m_ddns.empty() || m_drns.empty()) return 0;
  drnIndex = std::min(drnIndex, static_cast<uint32_t>(m_drns.size() - 1));
  return NearestDdn(m_drns[drnIndex].position.x);
}


uint32_t
AuvEdcExperiment::PacketSizeBytes(Priority priority) const
{
  switch (priority)
    {
    case Priority::HIGH: return m_highPacketSize;
    case Priority::MEDIUM: return m_mediumPacketSize;
    case Priority::LOW: return m_lowPacketSize;
    }
  return m_mediumPacketSize;
}

double
AuvEdcExperiment::DeadlineSeconds(Priority priority) const
{
  switch (priority)
    {
    case Priority::HIGH: return m_highDeadline;
    case Priority::MEDIUM: return m_mediumDeadline;
    case Priority::LOW: return m_lowDeadline;
    }
  return m_lowDeadline;
}

bool
AuvEdcExperiment::DeadlineExpired(const DataPacket& packet) const
{
  return Simulator::Now().GetSeconds() - packet.createdAt >
      DeadlineSeconds(packet.priority);
}

void
AuvEdcExperiment::RecordDeadlineExpiry(const DataPacket& packet)
{
  m_stats.deadlineExpired++;
  switch (packet.priority)
    {
    case Priority::HIGH: m_stats.deadlineExpiredHigh++; break;
    case Priority::MEDIUM: m_stats.deadlineExpiredMedium++; break;
    case Priority::LOW: m_stats.deadlineExpiredLow++; break;
    }
}

void
AuvEdcExperiment::AccountGeneratedPriority(Priority priority)
{
  switch (priority)
    {
    case Priority::HIGH:
      m_stats.highGenerated++;
      break;
    case Priority::MEDIUM:
      m_stats.mediumGenerated++;
      break;
    case Priority::LOW:
      m_stats.lowGenerated++;
      break;
    }
}

void
AuvEdcExperiment::AccountDeliveredPriority(Priority priority, double delay, bool burst,
                                         uint32_t payloadBytes)
{
  if (burst)
    {
      m_stats.burstDelivered++;
      if (delay >= 0.0)
        {
          m_stats.burstDelaySum += delay;
          if (delay > m_highDeadline) m_stats.burstDeadlineMiss++;
        }
    }
  switch (priority)
    {
    case Priority::HIGH:
      m_stats.highDelivered++;
      m_stats.highDeliveredBytes += payloadBytes;
      if (delay >= 0.0)
        {
          m_stats.highDelaySum += delay;
          if (delay > m_highDeadline) m_stats.highDeadlineMiss++;
        }
      break;
    case Priority::MEDIUM:
      m_stats.mediumDelivered++;
      m_stats.mediumDeliveredBytes += payloadBytes;
      if (delay >= 0.0)
        {
          m_stats.mediumDelaySum += delay;
          if (delay > m_mediumDeadline) m_stats.mediumDeadlineMiss++;
        }
      break;
    case Priority::LOW:
      m_stats.lowDelivered++;
      m_stats.lowDeliveredBytes += payloadBytes;
      if (delay >= 0.0)
        {
          m_stats.lowDelaySum += delay;
          if (delay > m_lowDeadline) m_stats.lowDeadlineMiss++;
        }
      break;
    }
}

void
AuvEdcExperiment::InjectAlarmBurst()
{
  if (m_alarmBurstSize == 0 || m_numDdn == 0) return;
  double now = Simulator::Now().GetSeconds();
  // A correlated event (leak / anchor strike) makes a spatial cluster of sensors
  // raise alarms that reach one DDN essentially simultaneously, so the whole
  // burst contends for the shared acoustic bypass at once.
  uint32_t ddnIndex = m_numDdn / 2;
  for (uint32_t k = 0; k < m_alarmBurstSize; ++k)
    {
      DataPacket pkt{m_nextPacketId++, 0, now, now, Priority::HIGH,
                     m_highPacketSize};
      pkt.burst = true;
      m_stats.generated++;
      m_stats.reachedDdn++;
      m_stats.burstGenerated++;
      AccountGeneratedPriority(Priority::HIGH);
      TracePacket("generated_burst", pkt, static_cast<int32_t>(ddnIndex), -1,
                  0, -1.0, "created_at_ddn");
      if (UsesPriorityQueues())
        EnqueueContribPacket(ddnIndex, pkt);
      else
        m_ddnBuffers[ddnIndex].push_back(pkt);
    }
}

void
AuvEdcExperiment::ChargeNodeRoleEnergy(NodeRole role, double joules)
{
  if (joules <= 0.0)
    {
      return;
    }
  switch (role)
    {
    case NodeRole::BSN:
      m_stats.bsnEnergyJ += joules;
      break;
    case NodeRole::DRN:
      m_stats.drnEnergyJ += joules;
      break;
    case NodeRole::DDN:
      m_stats.ddnEnergyJ += joules;
      break;
    case NodeRole::AUV:
      m_stats.auvEnergyJ += joules;
      break;
    case NodeRole::SINK:
      m_stats.sinkEnergyJ += joules;
      break;
    }
}

void
AuvEdcExperiment::ChargeGlobalEnergy(NodeRole role, double joules)
{
  if (joules <= 0.0)
    {
      return;
    }
  m_stats.energyConsumedJ += joules;
  ChargeNodeRoleEnergy(role, joules);
}

void
AuvEdcExperiment::ChargePacketEnergy(Priority priority, double joules, NodeRole role)
{
  if (joules <= 0.0)
    {
      return;
    }
  m_stats.energyConsumedJ += joules;
  switch (priority)
    {
    case Priority::HIGH:
      m_stats.highEnergyJ += joules;
      break;
    case Priority::MEDIUM:
      m_stats.mediumEnergyJ += joules;
      break;
    case Priority::LOW:
      m_stats.lowEnergyJ += joules;
      break;
    }
  ChargeNodeRoleEnergy(role, joules);
}

void
AuvEdcExperiment::ChargePacketEnergySplit(Priority priority,
                                          double joules,
                                          NodeRole firstRole,
                                          double firstShare,
                                          NodeRole secondRole)
{
  if (joules <= 0.0)
    {
      return;
    }
  const double clampedFirstShare = std::min(1.0, std::max(0.0, firstShare));
  ChargePacketEnergy(priority, joules * clampedFirstShare, firstRole);
  ChargePacketEnergy(priority, joules * (1.0 - clampedFirstShare), secondRole);
}

Priority
AuvEdcExperiment::AssignPriority(uint64_t packetId) const
{
  // Priority distribution for underwater pipeline monitoring:
  //   HIGH   10% — critical alerts: leak detection, pressure anomaly, structural alarm.
  //   MEDIUM 30% — periodic status: flow rate, temperature, corrosion index.
  //   LOW    60% — routine telemetry: background acoustic noise level, ambient data.
  (void)packetId;  // priority is now drawn stochastically, not derived from the id
  const double u = m_priorityRng->GetValue();  // U[0,1)
  if (u < 0.10) return Priority::HIGH;    // 10%
  if (u < 0.40) return Priority::MEDIUM;  // 30%
  return Priority::LOW;                    // 60%
}

std::string
AuvEdcExperiment::NormalizeName(std::string value)
{
  value.erase(std::remove_if(value.begin(),
                             value.end(),
                             [](unsigned char c) {
                               return c == '-' || c == '_' || std::isspace(c);
                             }),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

double
AuvEdcExperiment::OpticalExtinctionCoefficient() const
{
  if (m_opticalExtCoeffOverride > 0.0)
    {
      return m_opticalExtCoeffOverride;
    }

  const std::string water = NormalizeName(m_opticalWaterType);
  if (water == "veryclear" || water == "clear" || water == "jerlovi" || water == "i")
    {
      return 0.056;
    }
  if (water == "clearocean" || water == "ocean" || water == "jerlovib" || water == "ib")
    {
      return 0.151;
    }
  if (water == "jerlovii" || water == "ii")
    {
      return 0.200;
    }
  if (water == "coastal" || water == "coastal1c" || water == "jerloviii" ||
      water == "iii" || water == "1c")
    {
      return 0.398;
    }
  if (water == "turbid" || water == "turbidharbor" || water == "turbidharbour" ||
      water == "harbor" || water == "harbour" || water == "jerlov3c" || water == "3c")
    {
      return 2.195;
    }

  NS_FATAL_ERROR("unknown opticalWaterType: " << m_opticalWaterType);
  return 0.151;
}

double
AuvEdcExperiment::QFunction(double x)
{
  return 0.5 * std::erfc(x / std::sqrt(2.0));
}

double
AuvEdcExperiment::OpticalPacketSuccessProbability(uint32_t ddnIndex,
                                                   double distance,
                                                   uint32_t packetBytes,
                                                   double& snrDb,
                                                   double& ber,
                                                   double& pointingGain)
{
  if (m_disableOpticalLoss)
    {
      snrDb = 300.0;
      ber = 0.0;
      pointingGain = 1.0;
      return 1.0;
    }

  const double d = std::max(distance, 0.1);
  const double c = OpticalExtinctionCoefficient();

  // Per-transfer radial misalignment. A Rayleigh radial offset is the norm of
  // independent zero-mean Gaussian pointing errors in the transverse plane.
  pointingGain = 1.0;
  if (m_opticalPointingSigma > 0.0)
    {
      double radialOffset = 0.0;
      const double now = Simulator::Now().GetSeconds();
      const bool redrawPerPacket = m_opticalPointingCoherenceSeconds <= 0.0;
      const bool stale = ddnIndex >= m_opticalPointingLastDraw.size() ||
          m_opticalPointingLastDraw[ddnIndex] < 0.0 ||
          now - m_opticalPointingLastDraw[ddnIndex] >=
              m_opticalPointingCoherenceSeconds;
      if (redrawPerPacket || stale)
        {
          const double u = std::max(1e-12, m_opticalRng->GetValue());
          radialOffset = m_opticalPointingSigma *
              std::sqrt(-2.0 * std::log(u));
          if (!redrawPerPacket && ddnIndex < m_opticalPointingLastDraw.size())
            {
              m_opticalPointingLastDraw[ddnIndex] = now;
              m_opticalPointingRadialOffset[ddnIndex] = radialOffset;
            }
        }
      else
        {
          radialOffset = m_opticalPointingRadialOffset[ddnIndex];
        }
      const double halfAngleRad = (m_opticalDivergenceDeg * M_PI / 180.0) / 2.0;
      const double spotRadius = std::max(0.01, d * std::tan(halfAngleRad));
      pointingGain = std::exp(-2.0 * radialOffset * radialOffset / (spotRadius * spotRadius));
    }

  // Calibrated LOS link budget. It preserves the Beer-Lambert exponential term
  // and 1/d^2 geometric spreading, while avoiding arbitrary receiver-noise
  // constants in the simulator. At (referenceRange, referenceExtCoeff), SNR is
  // fixed to m_opticalReferenceSnr, which gives BER ~= 1e-5 for OOK.
  const double rangeTerm = (m_opticalReferenceRange / d) * (m_opticalReferenceRange / d);
  const double attenuationTerm =
      std::exp(-c * d + m_opticalReferenceExtCoeff * m_opticalReferenceRange);
  const double snrLinear =
      std::max(0.0, m_opticalReferenceSnr * rangeTerm * attenuationTerm * pointingGain);
  snrDb = 10.0 * std::log10(std::max(snrLinear, 1e-30));

  // IM/DD OOK approximation: BER = Q(sqrt(2*SNR)).
  ber = std::min(0.5, std::max(0.0, QFunction(std::sqrt(2.0 * snrLinear))));
  const double bitSuccess = std::max(0.0, 1.0 - ber);
  return std::pow(bitSuccess, static_cast<double>(packetBytes * 8));
}

double
AuvEdcExperiment::AcousticPacketSuccessProbability(double distance,
                                                    uint32_t packetBytes) const
{
  const double d = std::max(distance, 1.0);
  const double f = m_acousticFreqKhz;
  const double alphaDbKm =
      0.11*f*f/(1.0+f*f) + 44.0*f*f/(4100.0+f*f) +
      2.75e-4*f*f + 0.003;
  const double transmissionLossDb =
      15.0 * std::log10(d) + alphaDbKm * d / 1000.0;
  // MAC contention/collision is already generated by AquaSim events. Do not
  // subtract the superseded load-dependent SNR penalty here.
  const double snrLinear =
      std::pow(10.0, (m_txPowerMarginDb - transmissionLossDb) / 10.0);
  const double ber = 0.5 * std::exp(-std::max(0.0, snrLinear));
  return std::pow(std::max(0.0, 1.0 - ber),
                  static_cast<double>(packetBytes * 8));
}

double
AuvEdcExperiment::AcousticTxEnergyJ(uint32_t packetBytes, double distance) const
{
  if (m_closureMode)
    return m_closureAcousticTxPowerW * (packetBytes * 8.0 / m_acousticBitRate);
  if (m_energyModel == "hardware")
    {
      // EvoLogics S2C power steps: 5.5 W at 250 m, 8 W at 500 m,
      // 18 W at 1000 m, and 40 W at maximum output.
      const double txPowerW = distance <= 250.0 ? 5.5
                            : distance <= 500.0 ? 8.0
                            : distance <= 1000.0 ? 18.0
                            : 40.0;
      return txPowerW * (packetBytes * 8.0 / m_acousticBitRate);
    }
  return packetBytes * distance * m_acousticEnergyPerByteMeter;
}

double
AuvEdcExperiment::AcousticRxEnergyJ(uint32_t packetBytes) const
{
  if (m_closureMode)
    return m_closureAcousticRxPowerW * (packetBytes * 8.0 / m_acousticBitRate);
  if (m_energyModel == "hardware")
    {
      return m_acousticRxPowerW * (packetBytes * 8.0 / m_acousticBitRate);
    }
  return packetBytes * m_acousticReceiveEnergyPerByte;
}

double
AuvEdcExperiment::AcousticRecordDurationS(uint32_t payloadBytes) const
{
  const uint64_t fragmentCount =
      (static_cast<uint64_t>(payloadBytes) + m_acousticFramePayloadBytes - 1) /
      m_acousticFramePayloadBytes;
  const uint64_t wireBytes = static_cast<uint64_t>(payloadBytes) +
      fragmentCount * EdcHeader().GetSerializedSize();
  return wireBytes * 8.0 / m_acousticBitRate;
}

bool
AuvEdcExperiment::PopNextDdnPacket(uint32_t ddnIndex, DataPacket& packet)
{
  if (UsesPriorityQueues())
    {
      if (!m_ddnHighBuffers[ddnIndex].empty())
        {
          packet = m_ddnHighBuffers[ddnIndex].front();
          m_ddnHighBuffers[ddnIndex].pop_front();
          return true;
        }
      if (!m_ddnMediumBuffers[ddnIndex].empty())
        {
          packet = m_ddnMediumBuffers[ddnIndex].front();
          m_ddnMediumBuffers[ddnIndex].pop_front();
          return true;
        }
      if (!m_ddnLowBuffers[ddnIndex].empty())
        {
          packet = m_ddnLowBuffers[ddnIndex].front();
          m_ddnLowBuffers[ddnIndex].pop_front();
          return true;
        }
      return false;
    }

  if (m_ddnBuffers[ddnIndex].empty())
    {
      return false;
    }
  packet = m_ddnBuffers[ddnIndex].front();
  m_ddnBuffers[ddnIndex].pop_front();
  return true;
}

void
AuvEdcExperiment::RequeueFrontDdnPacket(uint32_t ddnIndex,
                                        const DataPacket& packet)
{
  if (m_ddnBufferCapacityBytes > 0)
    {
      // Failed transmissions can return after later arrivals. Apply the same
      // capacity policy again instead of silently overflowing the byte budget.
      EnqueueContribPacket(ddnIndex, packet, true);
      return;
    }
  if (!UsesPriorityQueues())
    {
      m_ddnBuffers[ddnIndex].push_front(packet);
      ObserveStorage(ddnIndex);
      WriteStorage("requeue", ddnIndex, &packet);
      return;
    }
  switch (packet.priority)
    {
    case Priority::HIGH: m_ddnHighBuffers[ddnIndex].push_front(packet); break;
    case Priority::MEDIUM: m_ddnMediumBuffers[ddnIndex].push_front(packet); break;
    case Priority::LOW: m_ddnLowBuffers[ddnIndex].push_front(packet); break;
    }
  ObserveStorage(ddnIndex);
  WriteStorage("requeue", ddnIndex, &packet);
}

bool
AuvEdcExperiment::InRange(const Vector& a, const Vector& b, double range) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= range;
}

void
AuvEdcExperiment::PrintSummary() const
{
  ReviewedOpticalSummary();
  uint64_t bufferedAtDdn = BufferedAtDdn();
  ReviewedIngressSummary();

  if (m_reviewedTransport)
    {
      uint64_t cancelled = 0;
      for (const auto& state : m_closureNodeTransport) cancelled += state->obsoleteTxCancelled;
      std::cout << "REVIEWED_FEATURES {\"version\":\"reviewed-ack-v1\","
                << "\"full_validation\":false,\"actual_reverse_acoustic_ack\":true,"
                << "\"data_arrival_separate_from_sender_confirmation\":true,"
                << "\"exactly_once_receiver_commit\":true,\"lifetime_leg_attempts\":true,"
                << "\"upstream_queue_admission_checked\":true,"
                << "\"upstream_persistent_write_validated\":false,"
                << "\"upstream_durable_gateway_admission_modelled\":" << (m_reviewedVerticalIngress ? "true" : "false") << ','
                << "\"upstream_ack_waits_for_cable_confirmation\":" << (m_reviewedVerticalIngress ? "true" : "false") << ','
                << "\"upstream_complete_data_receipts\":" << m_reviewedUpstreamCompleteReceipts << ','
                << "\"upstream_unique_data_received\":" << m_reviewedUpstreamReceivedIds.size() << ','
                << "\"upstream_admission_rejections\":" << m_reviewedUpstreamAdmissionRejected << ','
                << "\"local_obsolete_data_cancellation\":true,"
                << "\"authoritative_phy_state_energy_integrated\":" << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"half_duplex_integrated\":" << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"ack_timer_from_local_physical_completion\":" << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"local_fragment_refusal_notified\":" << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"lower_mac_silent_drop_notifications\":false,"
                << "\"physical_attempts_settled\":" << m_reviewedPhysicalAttemptsSettled << ','
                << "\"local_rejected_attempts\":" << m_reviewedLocalRejectedAttempts << ','
                << "\"optical_reverse_feedback_transported\":" << (m_reviewedOpticalControl ? "true" : "false") << ','
                << "\"ingress_scope\":\"" << (m_reviewedVerticalIngress ? "all_priorities_shared_logger_cable" : "LOW_gateway_connected_logger")
                << "\",\"durable_ingress_integrated\":"
                << (m_reviewedIngress ? "true" : "false") << ','
                << "\"ack_payload_bytes\":" << m_reviewedAckPayloadBytes << ','
                << "\"optional_low_window_descriptor_bytes\":24,\"low_window_descriptor_wire_validated\":true,"
                << "\"wire_header_bytes\":51,\"maximum_leg_attempts\":" << m_reviewedMaxAttempts << ','
                << "\"receiver_commits\":" << m_reviewedDataArrivals << ','
                << "\"duplicate_data_receipts\":" << m_reviewedDuplicateData << ','
                << "\"ack_messages_queued\":" << m_reviewedAckSent << ','
                << "\"sender_confirmations\":" << m_reviewedConfirmed << ','
                << "\"received_but_sender_unconfirmed\":" << m_reviewedUnconfirmedReceived << ','
                << "\"terminal_unreceived_offload_legs\":" << m_reviewedRetryDropped << ','
                << "\"upstream_deadline_expiries\":" << m_reviewedUpstreamExpired << ','
                << "\"late_inflight_recoveries\":" << m_reviewedLateRecovered << ','
                << "\"stale_endpoint_frames\":" << m_reviewedStaleFrames << ','
                << "\"committed_sender_copies_still_pending\":" << ReviewedCommittedPending() << ','
                << "\"obsolete_source_fragments_cancelled\":" << cancelled << "}\n";
      std::cout << "REVIEWED_CONTROL {\"version\":1,\"full_validation\":false,\"enabled\":"
                << (m_reviewedControlMaxPayloadBytes ? "true" : "false")
                << ",\"reserved_leg\":255,\"wire_header_bytes\":61,\"actual_encoded_body\":true,"
                << "\"receiver_reassembly_callback_only\":true,\"point_to_point\":true,"
                << "\"implicit_ack_or_retry\":false,\"control_cost_in_authoritative_phy_energy\":"
                << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"legacy_data_counters_include_control\":false,\"max_payload_bytes\":" << m_reviewedControlMaxPayloadBytes
                << ",\"lifetime_s\":" << m_reviewedControlLifetimeS << ",\"receiver_slots\":" << m_reviewedControlMaxInflight
                << ",\"source_slots\":" << m_reviewedControlMaxInflight << ",\"source_slots_release\":\"local_wire_expiry_no_remote_oracle\""
                << ",\"enqueued\":" << m_reviewedControlEnqueued << ",\"received\":" << m_reviewedControlReceived
                << ",\"rejected\":" << m_reviewedControlRejected << ",\"fragment_handoffs\":" << m_reviewedControlFragmentHandoffs
                << ",\"fragment_rejections\":" << m_reviewedControlFragmentRejections
                << ",\"payload_bytes_enqueued\":" << m_reviewedControlPayloadBytesEnqueued
                << ",\"application_wire_bytes_handed_off\":" << m_reviewedControlAppWireBytes << "}\n";
    }

  if (m_closureMode)
    {
      uint64_t txReservations = 0, txRangeRejected = 0;
      uint64_t uniqueApplicationRx = 0, duplicateRxCallbacks = 0;
      double maxAdmissionWaitS = 0.0;
      for (const auto& state : m_closureNodeTransport)
        {
          txReservations += state->txReservations;
          txRangeRejected += state->txRangeRejected;
          uniqueApplicationRx += state->uniqueApplicationRx;
          duplicateRxCallbacks += state->duplicateRxCallbacks;
          maxAdmissionWaitS = std::max(maxAdmissionWaitS, state->maxAdmissionWaitS);
        }
      // This is a parseable partial-implementation declaration, not a claim
      // that this simulator has passed the complete physical launch gates.
      std::cout << "CLOSURE_FEATURES {\"version\":\"transport-slice-v1\","
                << "\"full_validation\":false,\"fixed_role_ranges\":true,"
                << "\"explicit_acoustic_powers\":true,\"app_rx_attempt_dedup\":true,"
                << "\"app_wire_tx_serialization\":true,\"reverse_ack\":"
                << (m_reviewedTransport ? "true" : "false") << ','
                << "\"complete_phy_state_energy\":" << (m_reviewedPhyEnergy ? "true" : "false") << ','
                << "\"durable_ingress\":"
                << (m_reviewedIngress ? "true" : "false") << ','
                << "\"finite_mission_integrated\":" << (m_reviewedMission ? "true" : "false") << ','
                << "\"adaptive_policy_integrated\":false,"
                << "\"retry_semantics\":\"" << m_closureRetrySemantics << "\","
                << "\"tx_power_w\":" << m_closureAcousticTxPowerW << ','
                << "\"rx_power_w\":" << m_closureAcousticRxPowerW << ','
                << "\"bit_rate_bps\":" << m_acousticBitRate << ','
                << "\"frequency_khz\":" << m_acousticFreqKhz << ','
                << "\"bsn_range_m\":" << m_closureBsnRangeM << ','
                << "\"drn_range_m\":" << m_closureDrnRangeM << ','
                << "\"gateway_range_m\":" << m_closureGatewayRangeM << ','
                << "\"auv_range_m\":" << m_closureAuvRangeM << ','
                << "\"sink_range_m\":" << m_closureSinkRangeM << ','
                << "\"tx_reservations\":" << txReservations << ','
                << "\"tx_range_rejected\":" << txRangeRejected << ','
                << "\"unique_application_rx\":" << uniqueApplicationRx << ','
                << "\"duplicate_rx_callbacks\":" << duplicateRxCallbacks << ','
                << "\"max_admission_wait_s\":" << maxAdmissionWaitS << "}\n";
    }

  double opticalLossPct = m_stats.opticalAttemptedPackets == 0 ? 0.0
      : 100.0 * m_stats.opticalLostPackets / m_stats.opticalAttemptedPackets;
  double opticalMeanSuccess = m_stats.opticalContactSamples == 0 ? 0.0
      : m_stats.opticalSuccessProbSum / m_stats.opticalContactSamples;
  double opticalMeanSnrDb = m_stats.opticalContactSamples == 0 ? 0.0
      : m_stats.opticalSnrDbSum / m_stats.opticalContactSamples;

  std::cout << "\n----------- AUV-Based EDC UW-LSN summary -----------\n";
	  std::cout << "Paper setup: MAC=Underwater MAC, UDP-like packets, omni acoustic, "
	            << "BW=" << m_channelBandwidth << " Hz"
	            << ", rate=" << m_acousticBitRate << " bps"
	            << ", simTime=" << m_simStop
	            << " s, trafficStop=" << m_trafficStop
	            << " s, protocol=" << m_protocolName
                    << ", auvCollectionMedium=" << m_auvCollectionMedium
                    << ", energyModel=" << m_energyModel
                    << ", matchHybridQueues=" << m_matchHybridQueues
                    << ", ddnBufferPolicy=" << m_ddnBufferPolicy
                    << ", nodeIdlePowerW=" << m_nodeIdlePowerW << "\n";
  std::cout << "Acoustic MAC: " << m_acousticMac
            << (m_acousticMac == "aloha" ? " (hop ACKs off; end-to-end retry)" : "")
            << "\n";
  std::cout << "Topology: BSN=" << m_numBsn
            << " DRN=" << m_numDrn
            << " DDN=" << m_numDdn
            << " SINK=" << m_sinks
            << " sinkPlacement=" << m_sinkPlacement
            << " AUV=" << (m_protocol == ProtocolMode::PURE_ACOUSTIC ? 0 : 1)
            << " length=" << m_pipelineLength
            << " m pipelineDepth=" << m_depth
            << " m ddnEomOffset="
            << ((m_protocol == ProtocolMode::CONTRIBUTION ||
                 m_protocol == ProtocolMode::PURE_ACOUSTIC) ? m_ddnEomOffset : 0.0)
            << " m\n";
  std::cout << "Ranges: BSN=" << m_bsnRange
            << " DRN=" << m_drnRange
            << " DDN=" << m_ddnRange
            << " DDN-AUV=" << m_ddnCollectionRange
            << " SINK=" << m_sinkRange << " m\n";
  const double auvContactDepth =
      (m_protocol == ProtocolMode::CONTRIBUTION ||
       m_protocol == ProtocolMode::PURE_ACOUSTIC)
          ? std::max(0.0, m_depth - m_ddnEomOffset)
          : m_depth;
  std::cout << "AUV: speed=" << m_auvSpeedKmh
            << " km/h contactDepth=" << auvContactDepth
            << " m sinusAmplitude=" << m_auvAmplitude
            << " m completePatrolInSimTime=" << m_completePatrolInSimTime << "\n";
  std::cout << "UWOC: waterType=" << m_opticalWaterType
            << " c=" << OpticalExtinctionCoefficient()
            << " 1/m opticalRange=" << m_opticalRange
            << " m dataRate=" << m_opticalDataRateBps
            << " bps pointingSigma=" << m_opticalPointingSigma
            << " m pointingCoherence=" << m_opticalPointingCoherenceSeconds
            << " s"
            << " m divergence=" << m_opticalDivergenceDeg
            << " deg disableOpticalLoss=" << m_disableOpticalLoss << "\n";
  std::cout << "Traffic: load=" << m_trafficLoad
            << " pkt/s payloadBytes(H/M/L)=" << m_highPacketSize << '/'
            << m_mediumPacketSize << '/' << m_lowPacketSize
            << " ddnContactPacketsPerTick=" << m_ddnContactPacketsPerTick
            << " sinkContactPacketsPerTick=" << m_sinkContactPacketsPerTick << "\n";
  std::cout << "Generated=" << m_stats.generated
            << " gatewayOriginated=" << m_stats.gatewayOriginated
            << " reachedDDN=" << m_stats.reachedDdn
            << " collectedByAUV=" << m_stats.collectedByAuv
            << " deliveredToSink=" << m_stats.deliveredToSink
            << " directCriticalToSink=" << m_stats.directCriticalToSink
            << " directMediumToSink=" << m_stats.directMediumToSink
            << " directLowToSink=" << m_stats.directLowToSink
            << " mediumFallbackRhoMax=" << m_mediumFallbackRhoMax
            << " lowFallbackRhoMax=" << m_lowFallbackRhoMax
            << " opticalTransfers=" << m_stats.opticalTransfers
            << " opticalAttempted=" << m_stats.opticalAttemptedPackets
            << " opticalLost=" << m_stats.opticalLostPackets
            << " opticalPending=" << m_pendingOptical.size()
            << " opticalLossPct=" << opticalLossPct
            << " opticalMeanPktSuccess=" << opticalMeanSuccess
            << " opticalMeanSnrDb=" << opticalMeanSnrDb
            << " acousticTransfers=" << m_stats.acousticTransfers
            << " acousticTxBytes=" << m_stats.acousticTxBytes
            << " acousticTxAirtimeS=" << m_stats.acousticTxAirtimeS
            << " acousticSourceAttempts=" << m_stats.acousticSourceAttempts
            << " acousticRetransmissions=" << m_stats.acousticRetransmissions
            << " acousticRetryExhausted=" << m_stats.acousticRetryExhausted
            << " acousticAcks=" << m_stats.acousticAcks
            << " acousticPending=" << m_pendingAcoustic.size()
            << " directSourceAttempts=" << m_stats.directSourceAttempts
            << " directRetransmissions=" << m_stats.directRetransmissions
            << " directCyclesExhausted=" << m_stats.directCyclesExhausted
            << " directAcks=" << m_stats.directAcks
            << " directPending=" << m_pendingDirect.size()
            << " mobileSourceAttempts=" << m_stats.mobileSourceAttempts
            << " mobileRetransmissions=" << m_stats.mobileRetransmissions
            << " mobileContactFailures=" << m_stats.mobileContactFailures
            << " mobileAcks=" << m_stats.mobileAcks
            << " mobilePending=" << m_pendingMobile.size()
            << " auvSurfaceSourceAttempts=" << m_stats.auvSurfaceSourceAttempts
            << " auvSurfaceRetransmissions=" << m_stats.auvSurfaceRetransmissions
            << " auvSurfaceContactFailures=" << m_stats.auvSurfaceContactFailures
            << " auvSurfaceAcks=" << m_stats.auvSurfaceAcks
            << " auvSurfacePending=" << m_pendingAuvSurface.size()
            << " energyConsumedJ=" << m_stats.energyConsumedJ
            << " ddnBuffered=" << bufferedAtDdn
            << " auvBuffered=" << m_auvBuffer.size() << "\n";
  std::cout << "Priority: highGenerated=" << m_stats.highGenerated
            << " mediumGenerated=" << m_stats.mediumGenerated
            << " lowGenerated=" << m_stats.lowGenerated
            << " highDelivered=" << m_stats.highDelivered
            << " mediumDelivered=" << m_stats.mediumDelivered
            << " lowDelivered=" << m_stats.lowDelivered << "\n";
  // ── Compute all metrics ────────────────────────────────────────────────────

  // 1. PDR and delay at surface sink (same point for all protocols)
  double pdrSurface           = m_stats.generated == 0 ? 0.0
      : 100.0 * m_stats.surfaceDelivered / m_stats.generated;
  double avgDelayE2E          = m_stats.surfaceDelivered == 0 ? 0.0
      : m_stats.surfaceDelaySum / m_stats.surfaceDelivered;
  const uint64_t deliveredPayloadBytes =
      m_stats.highDeliveredBytes + m_stats.mediumDeliveredBytes + m_stats.lowDeliveredBytes;
  double throughputSurfaceBps = (m_simStop <= 0.0) ? 0.0
      : (deliveredPayloadBytes * 8.0) / m_simStop;

  // 2. Acoustic-chain-only delay: BSN→DDN (comparable to Ahmed 2022 baseline, ~1-9 s)
  double pdrDDN               = m_stats.generated == 0 ? 0.0
      : 100.0 * m_stats.acousticDelivered / m_stats.generated;
  double avgDelayAcoustic     = m_stats.acousticDelivered == 0 ? 0.0
      : m_stats.acousticDelaySum / m_stats.acousticDelivered;

  // 3. Energy efficiency per packet delivered to surface
  double energyPerPktMj       = m_stats.surfaceDelivered == 0 ? 0.0
      : (m_stats.energyConsumedJ * 1000.0) / static_cast<double>(m_stats.surfaceDelivered);

  // 4. Per-priority PDR (contribution only; 0% for others — no differentiation)
  double pdrHigh   = m_stats.highGenerated   == 0 ? 0.0
      : 100.0 * m_stats.highDelivered   / m_stats.highGenerated;
  double pdrMedium = m_stats.mediumGenerated == 0 ? 0.0
      : 100.0 * m_stats.mediumDelivered / m_stats.mediumGenerated;
  double pdrLow    = m_stats.lowGenerated    == 0 ? 0.0
      : 100.0 * m_stats.lowDelivered    / m_stats.lowGenerated;
  double highAvgDelay = m_stats.highDelivered == 0 ? 0.0
      : m_stats.highDelaySum / m_stats.highDelivered;
  double mediumAvgDelay = m_stats.mediumDelivered == 0 ? 0.0
      : m_stats.mediumDelaySum / m_stats.mediumDelivered;
  double lowAvgDelay = m_stats.lowDelivered == 0 ? 0.0
      : m_stats.lowDelaySum / m_stats.lowDelivered;
  double highEnergyMjByte = m_stats.highDelivered == 0 ? 0.0
      : 1000.0 * m_stats.highEnergyJ / m_stats.highDeliveredBytes;
  double mediumEnergyMjByte = m_stats.mediumDelivered == 0 ? 0.0
      : 1000.0 * m_stats.mediumEnergyJ / m_stats.mediumDeliveredBytes;
  double lowEnergyMjByte = m_stats.lowDelivered == 0 ? 0.0
      : 1000.0 * m_stats.lowEnergyJ / m_stats.lowDeliveredBytes;
  double nodeRoleEnergyTotal = m_stats.bsnEnergyJ + m_stats.drnEnergyJ + m_stats.ddnEnergyJ +
                               m_stats.auvEnergyJ + m_stats.sinkEnergyJ;
  auto rolePct = [nodeRoleEnergyTotal](double energyJ) {
    return nodeRoleEnergyTotal <= 0.0 ? 0.0 : 100.0 * energyJ / nodeRoleEnergyTotal;
  };
  std::string dominantRole = "BSN";
  double dominantRoleEnergyJ = m_stats.bsnEnergyJ;
  if (m_stats.drnEnergyJ > dominantRoleEnergyJ)
    {
      dominantRole = "DRN";
      dominantRoleEnergyJ = m_stats.drnEnergyJ;
    }
  if (m_stats.ddnEnergyJ > dominantRoleEnergyJ)
    {
      dominantRole = "DDN";
      dominantRoleEnergyJ = m_stats.ddnEnergyJ;
    }
  if (m_stats.auvEnergyJ > dominantRoleEnergyJ)
    {
      dominantRole = "AUV";
      dominantRoleEnergyJ = m_stats.auvEnergyJ;
    }
  if (m_stats.sinkEnergyJ > dominantRoleEnergyJ)
    {
      dominantRole = "SINK";
      dominantRoleEnergyJ = m_stats.sinkEnergyJ;
    }
  uint64_t highUndelivered = m_stats.highGenerated > m_stats.highDelivered
      ? m_stats.highGenerated - m_stats.highDelivered
      : 0;
  uint64_t mediumUndelivered = m_stats.mediumGenerated > m_stats.mediumDelivered
      ? m_stats.mediumGenerated - m_stats.mediumDelivered
      : 0;
  uint64_t lowUndelivered = m_stats.lowGenerated > m_stats.lowDelivered
      ? m_stats.lowGenerated - m_stats.lowDelivered
      : 0;
  double highMissRatio = m_stats.highGenerated == 0 ? 0.0
      : 100.0 * (m_stats.highDeadlineMiss + highUndelivered) / m_stats.highGenerated;
  double mediumMissRatio = m_stats.mediumGenerated == 0 ? 0.0
      : 100.0 * (m_stats.mediumDeadlineMiss + mediumUndelivered) / m_stats.mediumGenerated;
  double lowMissRatio = m_stats.lowGenerated == 0 ? 0.0
      : 100.0 * (m_stats.lowDeadlineMiss + lowUndelivered) / m_stats.lowGenerated;

  std::cout << std::fixed << std::setprecision(4)
            // ── Metric 1: surface delivery (fair cross-protocol comparison) ──
            << "PDR_surface="      << pdrSurface        << " %\n"
            << "AvgDelay_E2E="     << avgDelayE2E       << " s"
            << "  (BSN->surface-sink, includes AUV transit)\n"
            << "Throughput="       << throughputSurfaceBps << " bps\n"
            << "EnergyPerPkt="     << energyPerPktMj    << " mJ/pkt\n"
            // ── Metric 2: acoustic-chain only (comparable to Ahmed 2022) ────
            << "PDR_DDN="          << pdrDDN            << " %"
            << "  (BSN->DDN acoustic, comparable to Ahmed 2022)\n"
            << "AvgDelay_acoustic=" << avgDelayAcoustic  << " s"
            << "  (BSN->DDN hop only, ~1-9 s reference range)\n"
            // ── Priority QoS (contribution mode) ────────────────────────────
            << "Priority_PDR: HIGH=" << pdrHigh << "% MEDIUM=" << pdrMedium
            << "% LOW=" << pdrLow << "%\n"
            << "Priority_delay: HIGH=" << highAvgDelay << "s MEDIUM=" << mediumAvgDelay
            << "s LOW=" << lowAvgDelay << "s\n"
            << "Priority_miss: HIGH=" << highMissRatio << "% MEDIUM=" << mediumMissRatio
            << "% LOW=" << lowMissRatio << "%\n"
            << "Priority_energy: HIGH=" << highEnergyMjByte << " mJ/byte MEDIUM="
            << mediumEnergyMjByte << " mJ/byte LOW=" << lowEnergyMjByte
            << " mJ/byte\n"
            << "Node_role_energy: BSN=" << m_stats.bsnEnergyJ << " J DRN="
            << m_stats.drnEnergyJ << " J DDN=" << m_stats.ddnEnergyJ
            << " J AUV=" << m_stats.auvEnergyJ << " J SINK=" << m_stats.sinkEnergyJ
            << " J dominant=" << dominantRole << "\n"
            << "Node_role_energy_pct: BSN=" << rolePct(m_stats.bsnEnergyJ) << "% DRN="
            << rolePct(m_stats.drnEnergyJ) << "% DDN=" << rolePct(m_stats.ddnEnergyJ)
            << "% AUV=" << rolePct(m_stats.auvEnergyJ) << "% SINK="
            << rolePct(m_stats.sinkEnergyJ) << "%\n";

	  std::cout << "RESULT"
	            << " nodes="             << m_totalNodes
	            << " protocol="          << m_protocolName
	            << " matchHybridQueues=" << m_matchHybridQueues
	            << " ddnBufferPolicy="   << m_ddnBufferPolicy
	            << " nodeIdlePowerW="    << m_nodeIdlePowerW
	            << " trafficLoad="       << m_trafficLoad
	            << " trafficStop="       << m_trafficStop
	            << " sinkPlacement="     << m_sinkPlacement
	            << " generated="         << m_stats.generated
            << " delivered="         << m_stats.surfaceDelivered
            << " pdr="               << pdrSurface
            << " avgDelayE2E="       << avgDelayE2E
            << " avgDelayAcoustic="  << avgDelayAcoustic
            << " pdrDDN="            << pdrDDN
            << " throughputBps="     << throughputSurfaceBps
            << " energyJ="           << m_stats.energyConsumedJ
            << " energyPerPktMj="    << energyPerPktMj
            << " pdrHigh="           << pdrHigh
            << " pdrMedium="         << pdrMedium
            << " pdrLow="            << pdrLow
            << " highAvgDelay="      << highAvgDelay
            << " mediumAvgDelay="    << mediumAvgDelay
            << " lowAvgDelay="       << lowAvgDelay
            << " highMissRatio="     << highMissRatio
            << " mediumMissRatio="   << mediumMissRatio
            << " lowMissRatio="      << lowMissRatio
            << " burstGenerated="    << m_stats.burstGenerated
            << " burstDelivered="    << m_stats.burstDelivered
            << " burstMeanDelay="    << (m_stats.burstDelivered ? m_stats.burstDelaySum / m_stats.burstDelivered : 0.0)
            << " burstDeadlineMiss=" << m_stats.burstDeadlineMiss
            << " highDeadlineMiss="  << m_stats.highDeadlineMiss
            << " mediumDeadlineMiss=" << m_stats.mediumDeadlineMiss
            << " lowDeadlineMiss="   << m_stats.lowDeadlineMiss
            << " directCritical="    << m_stats.directCriticalToSink
            << " bufferDropped="     << m_stats.bufferDropped
            << " bufferDroppedHigh=" << m_stats.bufferDroppedHigh
            << " bufferDroppedMedium=" << m_stats.bufferDroppedMedium
            << " bufferDroppedLow="  << m_stats.bufferDroppedLow
            << " directLow="         << m_stats.directLowToSink
            << " acousticSourceAttempts=" << m_stats.acousticSourceAttempts
            << " acousticRetransmissions=" << m_stats.acousticRetransmissions
            << " acousticRetryExhausted=" << m_stats.acousticRetryExhausted
            << " acousticAcks="       << m_stats.acousticAcks
            << " acousticPending="    << m_pendingAcoustic.size()
            << " directSourceAttempts=" << m_stats.directSourceAttempts
            << " directRetransmissions=" << m_stats.directRetransmissions
            << " directCyclesExhausted=" << m_stats.directCyclesExhausted
            << " directAcks="         << m_stats.directAcks
            << " directPending="      << m_pendingDirect.size()
            << " mobileSourceAttempts=" << m_stats.mobileSourceAttempts
            << " mobileRetransmissions=" << m_stats.mobileRetransmissions
            << " mobileContactFailures=" << m_stats.mobileContactFailures
            << " mobileAcks="         << m_stats.mobileAcks
            << " mobilePending="      << m_pendingMobile.size()
            << " auvSurfaceSourceAttempts=" << m_stats.auvSurfaceSourceAttempts
            << " auvSurfaceRetransmissions=" << m_stats.auvSurfaceRetransmissions
            << " auvSurfaceContactFailures=" << m_stats.auvSurfaceContactFailures
            << " auvSurfaceAcks="     << m_stats.auvSurfaceAcks
            << " auvSurfacePending="  << m_pendingAuvSurface.size()
            << " optical="           << m_stats.opticalTransfers
            << " opticalAttempted="  << m_stats.opticalAttemptedPackets
	            << " opticalLost="       << m_stats.opticalLostPackets
            << " opticalPending="    << m_pendingOptical.size()
            << " opticalLossPct="    << opticalLossPct
            << " opticalMeanPktSuccess=" << opticalMeanSuccess
            << " opticalMeanSnrDb="  << opticalMeanSnrDb
            << " directMedium="      << m_stats.directMediumToSink
            << " highEnergyJ="       << m_stats.highEnergyJ
            << " mediumEnergyJ="     << m_stats.mediumEnergyJ
            << " lowEnergyJ="        << m_stats.lowEnergyJ
            << " bsnEnergyJ="        << m_stats.bsnEnergyJ
            << " drnEnergyJ="        << m_stats.drnEnergyJ
            << " ddnEnergyJ="        << m_stats.ddnEnergyJ
            << " auvEnergyJ="        << m_stats.auvEnergyJ
            << " sinkEnergyJ="       << m_stats.sinkEnergyJ
            << " bsnEnergyPct="      << rolePct(m_stats.bsnEnergyJ)
            << " drnEnergyPct="      << rolePct(m_stats.drnEnergyJ)
            << " ddnEnergyPct="      << rolePct(m_stats.ddnEnergyJ)
            << " auvEnergyPct="      << rolePct(m_stats.auvEnergyJ)
            << " sinkEnergyPct="     << rolePct(m_stats.sinkEnergyJ)
            << " dominantNodeRole="  << dominantRole
            << " highEnergyMjByte="  << highEnergyMjByte
            << " mediumEnergyMjByte=" << mediumEnergyMjByte
            << " lowEnergyMjByte="   << lowEnergyMjByte
            << " rhoMax="            << m_mediumFallbackRhoMax
            << " lowRhoMax="         << m_lowFallbackRhoMax
            << " opticalExtCoeff="   << OpticalExtinctionCoefficient() << "\n";

  const uint64_t generatedThroughAcoustic =
      m_stats.generated - m_stats.burstGenerated - m_stats.gatewayOriginated;
  const uint64_t acousticAccounted = m_stats.acousticDelivered +
      m_stats.acousticRetryExhausted + m_pendingAcoustic.size() +
      m_reviewedUpstreamExpired - ReviewedCommittedPending(1);
  if (generatedThroughAcoustic != acousticAccounted)
    {
      std::cerr << "INVARIANT_ERROR acoustic generated=" << generatedThroughAcoustic
                << " deliveredToDdn=" << m_stats.acousticDelivered
                << " retryExhausted=" << m_stats.acousticRetryExhausted
                << " pending=" << m_pendingAcoustic.size() << "\n";
    }
  const uint64_t allAccounted = m_stats.surfaceDelivered +
      m_stats.acousticRetryExhausted + m_stats.deadlineExpired +
      m_stats.bufferDropped + BufferedAtDdn() + m_auvBuffer.size() +
      m_pendingAcoustic.size() + m_pendingDirect.size() + m_pendingMobile.size() +
      m_pendingAuvSurface.size() + ReviewedOpticalUnreceivedPending() +
      m_reviewedRetryDropped - ReviewedCommittedPending() +
      ReviewedIngressSourceHeld() + ReviewedIngressArchiveOverflow() + ReviewedIngressSourceUnavailable();
  if (m_stats.generated != allAccounted)
    {
      std::cerr << "INVARIANT_ERROR generated=" << m_stats.generated
                << " surfaceDelivered=" << m_stats.surfaceDelivered
                << " upstreamRetryExhausted=" << m_stats.acousticRetryExhausted
                << " deadlineExpired=" << m_stats.deadlineExpired
                << " bufferDropped=" << m_stats.bufferDropped
                << " ingressSourceHeld=" << ReviewedIngressSourceHeld()
                << " ingressArchiveOverflow=" << ReviewedIngressArchiveOverflow()
                << " ingressSourceUnavailable=" << ReviewedIngressSourceUnavailable()
                << " ddnBuffered=" << BufferedAtDdn()
                << " auvBuffered=" << m_auvBuffer.size()
                << " upstreamPending=" << m_pendingAcoustic.size()
                << " directPending=" << m_pendingDirect.size()
                << " mobilePending=" << m_pendingMobile.size()
                << " auvSurfacePending=" << m_pendingAuvSurface.size()
                << " opticalPending=" << m_pendingOptical.size() << "\n";
    }
}

void
AuvEdcExperiment::SampleMetrics()
{
  double now = Simulator::Now().GetSeconds();
  ReviewedIngressRefresh(now);
  if (!m_storageCsv.empty())
    for (uint32_t i = 0; i < m_numDdn; ++i)
      {
        ObserveStorage(i);
        WriteStorage(now + 1e-9 >= m_simStop ? "terminal" : "snapshot", i);
      }
  double pdr = m_stats.generated == 0 ? 0.0 : 100.0 * m_stats.deliveredToSink / m_stats.generated;
  double avgDelay = m_stats.deliveredToSink == 0 ? 0.0 : m_stats.delaySum / m_stats.deliveredToSink;
  double upstreamPdr = m_stats.generated == 0 ? 0.0
      : 100.0 * m_stats.reachedDdn / m_stats.generated;
  double avgDelayAcoustic = m_stats.acousticDelivered == 0 ? 0.0
      : m_stats.acousticDelaySum / m_stats.acousticDelivered;
  double highAvgDelay = m_stats.highDelivered == 0 ? 0.0
      : m_stats.highDelaySum / m_stats.highDelivered;
  double mediumAvgDelay = m_stats.mediumDelivered == 0 ? 0.0
      : m_stats.mediumDelaySum / m_stats.mediumDelivered;
  double lowAvgDelay = m_stats.lowDelivered == 0 ? 0.0
      : m_stats.lowDelaySum / m_stats.lowDelivered;
  uint64_t highUndelivered = m_stats.highGenerated > m_stats.highDelivered
      ? m_stats.highGenerated - m_stats.highDelivered
      : 0;
  uint64_t mediumUndelivered = m_stats.mediumGenerated > m_stats.mediumDelivered
      ? m_stats.mediumGenerated - m_stats.mediumDelivered
      : 0;
  uint64_t lowUndelivered = m_stats.lowGenerated > m_stats.lowDelivered
      ? m_stats.lowGenerated - m_stats.lowDelivered
      : 0;
  double highMissRatio = m_stats.highGenerated == 0 ? 0.0
      : 100.0 * (m_stats.highDeadlineMiss + highUndelivered) / m_stats.highGenerated;
  double mediumMissRatio = m_stats.mediumGenerated == 0 ? 0.0
      : 100.0 * (m_stats.mediumDeadlineMiss + mediumUndelivered) / m_stats.mediumGenerated;
  double lowMissRatio = m_stats.lowGenerated == 0 ? 0.0
      : 100.0 * (m_stats.lowDeadlineMiss + lowUndelivered) / m_stats.lowGenerated;
  const uint64_t deliveredPayloadBytes =
      m_stats.highDeliveredBytes + m_stats.mediumDeliveredBytes + m_stats.lowDeliveredBytes;
  double throughputBps = now <= 0.0 ? 0.0 : (deliveredPayloadBytes * 8.0) / now;
  double opticalLossPct = m_stats.opticalAttemptedPackets == 0 ? 0.0
      : 100.0 * m_stats.opticalLostPackets / m_stats.opticalAttemptedPackets;
  double opticalMeanSuccess = m_stats.opticalContactSamples == 0 ? 0.0
      : m_stats.opticalSuccessProbSum / m_stats.opticalContactSamples;
  double opticalMeanSnrDb = m_stats.opticalContactSamples == 0 ? 0.0
      : m_stats.opticalSnrDbSum / m_stats.opticalContactSamples;
  const double acousticTxAirtimePct = now <= 0.0 ? 0.0
      : 100.0 * m_stats.acousticTxAirtimeS / now;
  const double meanDdnBuffered = m_stats.queueObservationSeconds <= 0.0 ? 0.0
      : m_stats.ddnQueuePacketSeconds / m_stats.queueObservationSeconds;
  const double meanAuvBuffered = m_stats.queueObservationSeconds <= 0.0 ? 0.0
      : m_stats.auvQueuePacketSeconds / m_stats.queueObservationSeconds;
  double highEnergyMjByte = m_stats.highDelivered == 0 ? 0.0
      : 1000.0 * m_stats.highEnergyJ / m_stats.highDeliveredBytes;
  double mediumEnergyMjByte = m_stats.mediumDelivered == 0 ? 0.0
      : 1000.0 * m_stats.mediumEnergyJ / m_stats.mediumDeliveredBytes;
  double lowEnergyMjByte = m_stats.lowDelivered == 0 ? 0.0
      : 1000.0 * m_stats.lowEnergyJ / m_stats.lowDeliveredBytes;
  double nodeRoleEnergyTotal = m_stats.bsnEnergyJ + m_stats.drnEnergyJ + m_stats.ddnEnergyJ +
                               m_stats.auvEnergyJ + m_stats.sinkEnergyJ;
  const double submergedEnergyJ = m_stats.bsnEnergyJ + m_stats.drnEnergyJ +
                                  m_stats.ddnEnergyJ + m_reviewedIngressSourceEnergyJ +
                                  m_reviewedIngressSourceBaseEnergyJ;
  const double incrementalMissionEnergyJ =
      std::max(0.0, m_stats.energyConsumedJ - m_stats.auvOperationalEnergyJ);
  auto rolePct = [nodeRoleEnergyTotal](double energyJ) {
    return nodeRoleEnergyTotal <= 0.0 ? 0.0 : 100.0 * energyJ / nodeRoleEnergyTotal;
  };
  Vector auvPos = GetAuvPosition(now);
  const bool eomArchitecture =
      (m_protocol == ProtocolMode::CONTRIBUTION ||
       m_protocol == ProtocolMode::PURE_ACOUSTIC);
  const double reportedEomOffset = eomArchitecture ? m_ddnEomOffset : 0.0;
  const double reportedDdnDepth = eomArchitecture
      ? std::max(0.0, m_depth - m_ddnEomOffset)
      : m_depth;

  std::ofstream out(m_metricsCsv, std::ios::app);
  out << std::fixed << std::setprecision(6)
	      << now << ','
	      << m_protocolName << ','
	      << m_auvCollectionMedium << ','
	      << m_energyModel << ','
	      << m_lowPayloadAtGateway << ','
	      << m_enableHighFallback << ','
	      << m_enableMediumFallback << ','
	      << m_matchHybridQueues << ','
	      << m_ddnBufferPolicy << ','
	      << m_nodeIdlePowerW << ','
	      << m_totalNodes << ','
	      << m_sinks << ','
	      << m_sinkPlacement << ','
	      << m_trafficLoad << ','
	      << m_trafficStop << ','
	      << m_auvSpeedKmh << ','
      << m_txPowerMarginDb << ','
      << reportedEomOffset << ','
      << reportedDdnDepth << ','
      << m_highFallbackTimeout << ','
      << m_mediumFallbackTimeout << ','
      << m_mediumFallbackRhoMax << ','
	      << m_stats.generated << ','
	      << m_stats.gatewayOriginated << ','
      << m_stats.reachedDdn << ','
      << m_stats.collectedByAuv << ','
      << m_stats.deliveredToSink << ','
      << m_stats.directCriticalToSink << ','
      << m_stats.directMediumToSink << ','
      << m_stats.directLowToSink << ','
      << m_stats.bufferDropped << ','
      << m_stats.bufferDroppedHigh << ','
      << m_stats.bufferDroppedMedium << ','
      << m_stats.bufferDroppedLow << ','
      << m_stats.opticalTransfers << ','
      << m_stats.opticalAttemptedPackets << ','
      << m_stats.opticalLostPackets << ','
      << ReviewedOpticalUnreceivedPending() << ','
      << opticalLossPct << ','
      << opticalMeanSuccess << ','
      << opticalMeanSnrDb << ','
      << m_opticalWaterType << ','
      << m_opticalRange << ','
      << m_opticalPointingCoherenceSeconds << ','
      << OpticalExtinctionCoefficient() << ','
      << m_stats.acousticTransfers << ','
      << m_stats.acousticTxBytes << ','
      << m_stats.acousticTxAirtimeS << ','
      << acousticTxAirtimePct << ','
      << m_stats.acousticSourceAttempts << ','
      << m_stats.acousticRetransmissions << ','
      << m_stats.acousticRetryExhausted << ','
      << m_stats.acousticAcks << ','
      << m_pendingAcoustic.size() << ','
      << m_stats.directSourceAttempts << ','
      << m_stats.directRetransmissions << ','
      << m_stats.directCyclesExhausted << ','
      << m_stats.directAcks << ','
      << m_pendingDirect.size() << ','
      << m_stats.mobileSourceAttempts << ','
      << m_stats.mobileRetransmissions << ','
      << m_stats.mobileContactFailures << ','
      << m_stats.mobileAcks << ','
      << m_pendingMobile.size() << ','
      << m_stats.auvSurfaceSourceAttempts << ','
      << m_stats.auvSurfaceRetransmissions << ','
      << m_stats.auvSurfaceContactFailures << ','
      << m_stats.auvSurfaceAcks << ','
      << m_pendingAuvSurface.size() << ','
      << m_stats.deadlineExpired << ','
      << m_stats.deadlineExpiredHigh << ','
      << m_stats.deadlineExpiredMedium << ','
      << m_stats.deadlineExpiredLow << ','
      << m_stats.highGenerated << ','
      << m_stats.mediumGenerated << ','
      << m_stats.lowGenerated << ','
      << m_stats.highDelivered << ','
      << m_stats.mediumDelivered << ','
      << m_stats.lowDelivered << ','
      << highAvgDelay << ','
      << mediumAvgDelay << ','
      << lowAvgDelay << ','
      << highMissRatio << ','
      << mediumMissRatio << ','
      << lowMissRatio << ','
      << m_stats.highDeadlineMiss << ','
      << m_stats.mediumDeadlineMiss << ','
      << m_stats.lowDeadlineMiss << ','
      << m_stats.energyConsumedJ << ','
      << m_stats.highEnergyJ << ','
      << m_stats.mediumEnergyJ << ','
      << m_stats.lowEnergyJ << ','
      << m_stats.bsnEnergyJ << ','
      << m_stats.drnEnergyJ << ','
      << m_stats.ddnEnergyJ << ','
      << m_stats.auvEnergyJ << ','
      << m_stats.auvOperationalEnergyJ << ','
      << submergedEnergyJ << ','
      << incrementalMissionEnergyJ << ','
      << m_stats.sinkEnergyJ << ','
      << rolePct(m_stats.bsnEnergyJ) << ','
      << rolePct(m_stats.drnEnergyJ) << ','
      << rolePct(m_stats.ddnEnergyJ) << ','
      << rolePct(m_stats.auvEnergyJ) << ','
      << rolePct(m_stats.sinkEnergyJ) << ','
      << highEnergyMjByte << ','
      << mediumEnergyMjByte << ','
      << lowEnergyMjByte << ','
      << meanDdnBuffered << ','
      << m_stats.maxDdnBuffered << ','
      << meanAuvBuffered << ','
      << m_stats.maxAuvBuffered << ','
      << BufferedAtDdn() << ','
      << m_auvBuffer.size() << ','
      << upstreamPdr << ','
      << avgDelayAcoustic << ','
      << pdr << ','
      << avgDelay << ','
      << throughputBps << ','
      << auvPos.x << ','
      << auvPos.y << ','
      << auvPos.z;
  if (m_reviewedTransport)
    out << ',' << ReviewedCommittedPending() << ',' << ReviewedCommittedPending(1)
        << ',' << m_reviewedRetryDropped << ',' << m_reviewedUpstreamExpired
        << ',' << m_reviewedConfirmed << ',' << m_reviewedDataArrivals
        << ',' << m_reviewedLateRecovered;
  if(m_reviewedControlMaxPayloadBytes)
    out << ',' << m_reviewedControlEnqueued << ',' << m_reviewedControlReceived
        << ',' << m_reviewedControlRejected << ',' << m_reviewedControlFragmentHandoffs
        << ',' << m_reviewedControlFragmentRejections << ',' << m_reviewedControlAppWireBytes;
  if (m_reviewedIngress)
    {
      uint64_t archiveBytes = 0, rawGatewayBytes = 0, historical = 0, sourceUnconfirmed = 0;
      for (const auto& s : m_reviewedIngressStates)
        {
          archiveBytes += s.archive_resident_bytes;
          rawGatewayBytes += s.raw_gateway_resident_bytes;
          historical += s.retained_historical_records;
          sourceUnconfirmed += s.source_unconfirmed_records;
        }
      out << ',' << ReviewedIngressSourceHeld() << ',' << ReviewedIngressArchiveOverflow()
          << ',' << archiveBytes << ',' << rawGatewayBytes << ',' << historical
          << ',' << sourceUnconfirmed << ',' << m_reviewedIngressSourceEnergyJ
          << ',' << m_reviewedIngressGatewayEnergyJ << ',' << ReviewedIngressSourceUnavailable()
          << ',' << m_reviewedIngressSourceBaseEnergyJ;
    }
  out << '\n';

  if (now + m_sampleInterval <= m_simStop)
    {
      Simulator::Schedule(Seconds(m_sampleInterval), &AuvEdcExperiment::SampleMetrics, this);
    }
}

void
AuvEdcExperiment::WriteMetricsHeader() const
{
  std::ofstream out(m_metricsCsv, std::ios::trunc);
  out << "time,protocol,auvCollectionMedium,energyModel,lowPayloadAtGateway,"
      << "enableHighFallback,enableMediumFallback,matchHybridQueues,ddnBufferPolicy,nodeIdlePowerW,"
      << "nodes,sinks,sinkPlacement,trafficLoad,trafficStop,auvSpeedKmh,"
      << "txPowerMarginDb,"
      << "ddnEomOffset,ddnDepth,"
      << "highFallbackTimeout,mediumFallbackTimeout,mediumFallbackRhoMax,"
      << "generated,gatewayOriginated,reachedDdn,collectedByAuv,"
      << "delivered,directCriticalToSink,directMediumToSink,directLowToSink,"
      << "bufferDropped,bufferDroppedHigh,bufferDroppedMedium,bufferDroppedLow,opticalTransfers,"
      << "opticalAttempted,opticalLost,opticalPending,opticalLossPct,"
      << "opticalMeanPktSuccess,opticalMeanSnrDb,"
      << "opticalWaterType,opticalRange,opticalPointingCoherenceSeconds,"
      << "opticalExtCoeff,acousticTransfers,"
      << "acousticTxBytes,acousticTxAirtimeS,acousticTxAirtimePct,"
      << "acousticSourceAttempts,acousticRetransmissions,acousticRetryExhausted,"
      << "acousticAcks,acousticPending,"
      << "directSourceAttempts,directRetransmissions,directCyclesExhausted,"
      << "directAcks,directPending,"
      << "mobileSourceAttempts,mobileRetransmissions,mobileContactFailures,"
      << "mobileAcks,mobilePending,"
      << "auvSurfaceSourceAttempts,auvSurfaceRetransmissions,auvSurfaceContactFailures,"
      << "auvSurfaceAcks,auvSurfacePending,"
      << "deadlineExpired,deadlineExpiredHigh,deadlineExpiredMedium,deadlineExpiredLow,"
      << "highGenerated,mediumGenerated,lowGenerated,highDelivered,mediumDelivered,lowDelivered,"
      << "highAvgDelay,mediumAvgDelay,lowAvgDelay,"
      << "highMissRatio,mediumMissRatio,lowMissRatio,"
      << "highDeadlineMiss,mediumDeadlineMiss,lowDeadlineMiss,"
      << "energyConsumedJ,highEnergyJ,mediumEnergyJ,lowEnergyJ,"
      << "bsnEnergyJ,drnEnergyJ,ddnEnergyJ,auvEnergyJ,auvOperationalEnergyJ,"
      << "submergedEnergyJ,incrementalMissionEnergyJ,sinkEnergyJ,"
      << "bsnEnergyPct,drnEnergyPct,ddnEnergyPct,auvEnergyPct,sinkEnergyPct,"
      << "highEnergyMjByte,mediumEnergyMjByte,lowEnergyMjByte,"
      << "meanDdnBuffered,maxDdnBuffered,meanAuvBuffered,maxAuvBuffered,ddnBuffered,"
      << "auvBuffered,upstreamPdr,avgDelayAcoustic,pdr,avgDelay,throughputBps,auvX,auvY,auvZ";
  if (m_reviewedTransport)
    out << ",reviewedCommittedSenderCopies,reviewedUpstreamCommittedSenderCopies,"
        << "reviewedUnreceivedOffloadLegs,reviewedUpstreamExpired,"
        << "reviewedAckConfirmations,reviewedDataArrivals,reviewedLateRecovered";
  if(m_reviewedControlMaxPayloadBytes)
    out << ",reviewedControlEnqueued,reviewedControlReceived,reviewedControlRejected,"
        << "reviewedControlFragmentHandoffs,reviewedControlFragmentRejections,reviewedControlAppWireBytes";
  if (m_reviewedIngress)
    out << ",reviewedIngressSourceHeld,reviewedIngressArchiveOverflow,reviewedIngressArchiveBytes,"
        << "reviewedIngressRawGatewayBytes,reviewedIngressHistoricalCopies,reviewedIngressUnconfirmedSources,"
        << "reviewedIngressSourceActiveJ,reviewedIngressGatewayActiveJ,"
        << "reviewedIngressSourceUnavailable,reviewedIngressSourceAcquisitionBaseJ";
  out << '\n';
}

void
AuvEdcExperiment::WritePhyEnergyHeader() const
{
  std::ofstream out(m_phyEnergyCsv, std::ios::trunc);
  if (!out) NS_FATAL_ERROR("Cannot open phyEnergyCsv: " << m_phyEnergyCsv);
  out << "time,event,nodeId,role,initialEnergyJ,remainingEnergyJ,internalConsumedJ,"
      << "firstDepletedAt,auvSurfaceAcks,directAcks,auvBuffered\n";
}

void
AuvEdcExperiment::BindReviewedPhyEnergy()
{
  m_reviewedPhys.reserve(m_allNodes.GetN());
  m_reviewedLedgers.reserve(m_allNodes.GetN());
  for (uint32_t i=0; i<m_allNodes.GetN(); ++i)
    {
      const auto role=static_cast<unsigned>(NodeRoleFromNs3Id(i));
      auto dev=DynamicCast<AquaSimNetDevice>(m_allNodes.Get(i)->GetDevice(0));
      auto phy=DynamicCast<ReviewedAquaSimPhy>(dev->GetPhy());
      NS_ABORT_MSG_IF(!phy || role>=5, "Missing reviewed per-node PHY owner");
      auto battery=std::make_shared<closure_energy::Battery>(m_reviewedBatteryJ[role]);
      const bool notDeployed=role==static_cast<unsigned>(NodeRole::AUV) &&
                             m_protocol==ProtocolMode::PURE_ACOUSTIC;
      auto ledger=std::make_shared<closure_energy::PowerLedger>(battery,
        notDeployed ? closure_energy::Powers{} :
        closure_energy::Powers{m_closureAcousticTxPowerW,m_closureAcousticRxPowerW,
          m_reviewedIdlePowerW,m_reviewedSleepPowerW,m_reviewedBaseW[role]},m_simStop);
      phy->EM()->SetInitialEnergy(battery->Capacity());
      phy->SetLedger(ledger);
      phy->SetLocalTxOutcome([this,i](const ReviewedLocalTxTag& tag, bool accepted, double endAt) {
        ReviewedLocalTxOutcome(tag,i,accepted,endAt);
      });
      if(notDeployed) phy->PowerOff();
      m_reviewedPhys.push_back(phy);
      m_reviewedLedgers.push_back(ledger);
    }
}

void
AuvEdcExperiment::FinalizeReviewedPhyEnergy()
{
  std::ofstream out(m_reviewedEnergyJson,std::ios::trunc);
  NS_ABORT_MSG_IF(!out,"Cannot write authoritative reviewed energy receipt");
  out << std::setprecision(17)
      << "{\"schema\":\"reviewed-waveform-energy-v1\",\"full_validation\":false,"
      << "\"legacy_metric_energy_is_diagnostic\":true,\"horizon_s\":"
      << Simulator::Now().GetSeconds() << ",\"nodes\":[";
  const char* roles[]={"BSN","DRN","DDN","SINK","AUV"};
  for (uint32_t i=0;i<m_reviewedPhys.size();++i)
    {
      m_reviewedPhys[i]->Sync();
      const auto ledger=m_reviewedLedgers[i];
      const auto battery=ledger->Budget();
      const auto& a=ledger->Accounting();
      const double residual=battery->Consumed()-a.TotalJ();
      NS_ABORT_MSG_IF(std::abs(residual)>1e-6*std::max(1.0,battery->Consumed()),
        "Unaccounted external debit on authoritative per-device energy owner");
      if(i) out << ',';
      out << "{\"node_id\":" << i << ",\"role\":\"" << roles[static_cast<unsigned>(NodeRoleFromNs3Id(i))]
          << "\",\"deployed\":" << ((NodeRoleFromNs3Id(i)==NodeRole::AUV && m_protocol==ProtocolMode::PURE_ACOUSTIC) ? "false" : "true")
          << ",\"capacity_j\":" << battery->Capacity() << ",\"remaining_j\":" << battery->Remaining()
          << ",\"consumed_j\":" << battery->Consumed() << ",\"tx_j\":" << a.tx_j
          << ",\"rx_j\":" << a.rx_j << ",\"idle_j\":" << a.idle_j
          << ",\"sleep_j\":" << a.sleep_j << ",\"base_j\":" << a.base_j
          << ",\"extra_j\":" << a.extra_j << ",\"tx_s\":" << a.tx_s << ",\"rx_s\":" << a.rx_s
          << ",\"idle_s\":" << a.idle_s << ",\"sleep_s\":" << a.sleep_s
          << ",\"depleted_s\":" << a.depleted_s << ",\"depleted_at_s\":";
      if(std::isfinite(ledger->DepletedAt())) out << ledger->DepletedAt(); else out << "null";
      out << ",\"half_duplex_tx_rejections\":" << m_reviewedPhys[i]->RejectedHalfDuplexTx()
          << ",\"half_duplex_rx_rejections\":" << m_reviewedPhys[i]->RejectedHalfDuplexRx()
          << ",\"waveforms\":" << m_reviewedPhys[i]->ReceivedWaveforms()
          << ",\"error_flag_waveforms\":" << m_reviewedPhys[i]->FailedReceiveFlags()
          << ",\"emitter_supply_rejections\":" << m_reviewedPhys[i]->RejectedEmissionSupply()
          << ",\"energy_residual_j\":" << residual << '}';
    }
  out << "]}\n";
}

void
AuvEdcExperiment::ObservePhyEnergy(bool terminal)
{
  // These library getters only return stored fields. In particular, do not
  // call UpdateIdleEnergy here: that would change the model being observed.
  // Reuse AuvTick rather than scheduling any extra events or RNG draws.
  const double now = Simulator::Now().GetSeconds();
  for (uint32_t i = 0; i < m_phyEnergyModels.size(); ++i)
    {
      if (m_phyFirstDepletedAt[i] < 0.0 && m_phyEnergyModels[i]->GetEnergy() <= 0.0)
        {
          m_phyFirstDepletedAt[i] = now;
          WritePhyEnergy("first_depleted", static_cast<int32_t>(i));
          NS_LOG_UNCOND("PHY_ENERGY_DEPLETED time=" << now
                        << " nodeId=" << m_allNodes.Get(i)->GetId()
                        << " role=" << static_cast<uint32_t>(NodeRoleFromNs3Id(i))
                        << " initialEnergyJ=" << m_phyEnergyModels[i]->GetInitialEnergy()
                        << " auvSurfaceAcks=" << m_stats.auvSurfaceAcks
                        << " auvBuffered=" << m_auvBuffer.size());
        }
    }
  if (terminal)
    WritePhyEnergy("terminal");
  else if (now + 1e-9 >= m_nextPhyEnergyTime)
    {
      WritePhyEnergy("snapshot");
      m_nextPhyEnergyTime = now + m_phyEnergyInterval;
    }
}

void
AuvEdcExperiment::WritePhyEnergy(const std::string& event, int32_t nodeIndex) const
{
  if (m_phyEnergyCsv.empty()) return;
  std::ofstream out(m_phyEnergyCsv, std::ios::app);
  if (!out) NS_FATAL_ERROR("Cannot append phyEnergyCsv: " << m_phyEnergyCsv);
  const char* roleNames[] = {"BSN", "DRN", "DDN", "SINK", "AUV"};
  const uint32_t begin = nodeIndex < 0 ? 0 : static_cast<uint32_t>(nodeIndex);
  const uint32_t end = nodeIndex < 0 ? m_phyEnergyModels.size() : begin + 1;
  for (uint32_t i = begin; i < end; ++i)
    {
      const auto& energy = m_phyEnergyModels[i];
      out << std::fixed << std::setprecision(6)
          << Simulator::Now().GetSeconds() << ',' << event << ','
          << m_allNodes.Get(i)->GetId() << ','
          << roleNames[static_cast<uint32_t>(NodeRoleFromNs3Id(i))] << ','
          << energy->GetInitialEnergy() << ',' << energy->GetEnergy() << ','
          << energy->GetTotalEnergyConsumption() << ',' << m_phyFirstDepletedAt[i] << ','
          << m_stats.auvSurfaceAcks << ',' << m_stats.directAcks << ','
          << m_auvBuffer.size() << '\n';
    }
}

double
AuvEdcExperiment::MediumFallbackRefillRate() const
{
  if (m_mediumFallbackRefillRate >= 0.0) return m_mediumFallbackRefillRate;
  // Preserve both operations and constants from the submitted implementation.
  const double mediumOfferedLoad = 0.30 * m_trafficLoad;
  const double mediumFallbackCapLoad = m_mediumFallbackRhoMax * m_trafficLoad;
  const double highReservedLoad = 0.10 * m_trafficLoad;
  return std::min(mediumOfferedLoad,
                  std::max(0.0, mediumFallbackCapLoad - highReservedLoad));
}

void
AuvEdcExperiment::ObserveGatewayDiagnostics(double now, const Vector& auvPos)
{
  const double dt = std::max(0.0, std::min(m_tick, m_simStop - now));
  for (uint32_t i = 0; i < m_numDdn; ++i)
    {
      auto& diagnostic = m_gatewayDiagnostics[i];
      const uint64_t queued = m_ddnBuffers[i].size() + m_ddnHighBuffers[i].size() +
                              m_ddnMediumBuffers[i].size() + m_ddnLowBuffers[i].size();
      diagnostic.queueRecordSeconds += queued * dt;
      diagnostic.maxGatewayQueued = std::max(diagnostic.maxGatewayQueued, queued);
      const bool contact = m_protocol != ProtocolMode::PURE_ACOUSTIC &&
          m_auvCollectionMedium == "optical" &&
          InRange(auvPos, m_ddns[i].position, m_opticalRange);
      if (contact && !diagnostic.contactActive)
        {
          diagnostic.contactActive = true;
          diagnostic.contactCount++;
          diagnostic.contactStartTime = now;
          diagnostic.contactStartQueued = queued;
          diagnostic.contactAttempts = 0;
          diagnostic.contactCollected = 0;
          WriteGatewayDiagnostics("contact_start", static_cast<int32_t>(i));
        }
      else if (!contact && diagnostic.contactActive)
        {
          diagnostic.contactActive = false;
          WriteGatewayDiagnostics("contact_end", static_cast<int32_t>(i));
        }
      if (contact) diagnostic.contactSeconds += dt;
    }
  if (now + 1e-9 >= m_nextDiagnosticsTime)
    {
      WriteGatewayDiagnostics("snapshot");
      m_nextDiagnosticsTime = now + m_diagnosticsInterval;
    }
}

void
AuvEdcExperiment::WriteGatewayDiagnosticsHeader() const
{
  std::ofstream out(m_diagnosticsCsv, std::ios::trunc);
  if (!out) NS_FATAL_ERROR("Cannot open diagnosticsCsv: " << m_diagnosticsCsv);
  out << "time,event,ddnIndex,ddnX,queueHigh,queueMedium,queueLow,queueFifo,"
      << "oldestHighAgeS,oldestMediumAgeS,oldestLowAgeS,directPending,"
      << "mediumTokens,mediumRefillRate,contactActive,contactCount,contactSeconds,"
      << "contactStartTime,contactStartQueued,contactAttempts,contactCollected,"
      << "opticalAttempts,opticalCollected,opticalLostPer,opticalLostContact,opticalBytes,"
      << "mediumAdmissions,mediumTokensCharged,mediumTokensChargedExpired,"
      << "mediumPurgedBeforeAdmission,mediumExpiredBeforeAttempt,mediumEligibleSeconds,"
      << "mediumTokenBlockedSeconds,mediumHighBlockedSeconds,mediumContactDeferredSeconds,"
      << "queueRecordSeconds,maxGatewayQueued\n";
}

void
AuvEdcExperiment::WriteGatewayDiagnostics(const std::string& event, int32_t ddnIndex) const
{
  if (m_diagnosticsCsv.empty()) return;
  std::ofstream out(m_diagnosticsCsv, std::ios::app);
  if (!out) NS_FATAL_ERROR("Cannot append diagnosticsCsv: " << m_diagnosticsCsv);
  const double now = Simulator::Now().GetSeconds();
  auto oldestAge = [now](const std::deque<DataPacket>& queue) {
    double age = 0.0;
    for (const auto& packet : queue) age = std::max(age, now - packet.createdAt);
    return age;
  };
  const uint32_t begin = ddnIndex < 0 ? 0 : static_cast<uint32_t>(ddnIndex);
  const uint32_t end = ddnIndex < 0 ? m_numDdn : begin + 1;
  for (uint32_t i = begin; i < end; ++i)
    {
      const auto& d = m_gatewayDiagnostics[i];
      out << std::fixed << std::setprecision(6)
          << now << ',' << event << ',' << i << ',' << m_ddns[i].position.x << ','
          << m_ddnHighBuffers[i].size() << ',' << m_ddnMediumBuffers[i].size() << ','
          << m_ddnLowBuffers[i].size() << ',' << m_ddnBuffers[i].size() << ','
          << oldestAge(m_ddnHighBuffers[i]) << ',' << oldestAge(m_ddnMediumBuffers[i]) << ','
          << oldestAge(m_ddnLowBuffers[i]) << ',' << m_directWaiting[i].size() << ','
          << m_mediumFallbackTokens << ',' << MediumFallbackRefillRate() << ','
          << d.contactActive << ',' << d.contactCount << ',' << d.contactSeconds << ','
          << d.contactStartTime << ',' << d.contactStartQueued << ','
          << d.contactAttempts << ',' << d.contactCollected << ','
          << d.opticalAttempts << ',' << d.opticalCollected << ','
          << d.opticalLostPer << ',' << d.opticalLostContact << ',' << d.opticalBytes << ','
          << d.mediumAdmissions << ',' << d.mediumTokensCharged << ','
          << d.mediumTokensChargedExpired << ',' << d.mediumPurgedBeforeAdmission << ','
          << d.mediumExpiredBeforeAttempt << ',' << d.mediumEligibleSeconds << ','
          << d.mediumTokenBlockedSeconds << ',' << d.mediumHighBlockedSeconds << ','
          << d.mediumContactDeferredSeconds << ',' << d.queueRecordSeconds << ','
          << d.maxGatewayQueued << '\n';
    }
}

void
AuvEdcExperiment::WritePacketTraceHeader() const
{
  std::ofstream out(m_packetTraceCsv, std::ios::trunc);
  out << "time,event,packetId,sourceBsn,priority,payloadBytes,ddnIndex,sinkIndex,"
      << "attempt,distanceM,outcome\n";
}

void
AuvEdcExperiment::TracePacket(const std::string& event,
                              const DataPacket& packet,
                              int32_t ddnIndex,
                              int32_t sinkIndex,
                              uint32_t attempt,
                              double distance,
                              const std::string& outcome) const
{
  if (m_packetTraceCsv.empty()) return;
  std::ofstream out(m_packetTraceCsv, std::ios::app);
  out << std::fixed << std::setprecision(6)
      << Simulator::Now().GetSeconds() << ','
      << event << ','
      << packet.id << ','
      << packet.sourceBsn << ','
      << static_cast<uint32_t>(packet.priority) << ','
      << packet.sizeBytes << ','
      << ddnIndex << ','
      << sinkIndex << ','
      << attempt << ','
      << distance << ','
      << outcome << '\n';
}

void
AuvEdcExperiment::TraceAcousticHop(uint64_t packetId,
                                   uint32_t sourceId,
                                   uint8_t priority,
                                   uint32_t payloadBytes,
                                   uint8_t attempt,
                                   uint32_t fromNode,
                                   uint32_t toNode,
                                   uint32_t destinationNode,
                                   double distance) const
{
  if (m_packetTraceCsv.empty() || m_reviewedCompactTrace) return;
  std::ofstream out(m_packetTraceCsv, std::ios::app);
  out << std::fixed << std::setprecision(6)
      << Simulator::Now().GetSeconds() << ",acoustic_hop_tx,"
      << packetId << ',' << sourceId << ','
      << static_cast<uint32_t>(priority) << ',' << payloadBytes
      << ",-1,-1," << static_cast<uint32_t>(attempt) << ',' << distance
      << ",from=" << fromNode << ";to=" << toNode
      << ";dst=" << destinationNode << '\n';
}

uint64_t
AuvEdcExperiment::BufferedAtDdn() const
{
  uint64_t buffered = 0;
  for (const auto& queue : m_ddnBuffers)
    {
      buffered += queue.size();
    }
  for (const auto& queue : m_ddnHighBuffers)
    {
      buffered += queue.size();
    }
  for (const auto& queue : m_ddnMediumBuffers)
    {
      buffered += queue.size();
    }
  for (const auto& queue : m_ddnLowBuffers)
    {
      buffered += queue.size();
    }
  return buffered;
}

bool
AuvEdcExperiment::HasHighBacklog() const
{
  return std::any_of(m_ddnHighBuffers.begin(),
                     m_ddnHighBuffers.end(),
                     [](const std::deque<DataPacket>& queue) {
                       return !queue.empty();
                     });
}

bool
AuvEdcExperiment::HasMediumBacklog() const
{
  return std::any_of(m_ddnMediumBuffers.begin(),
                     m_ddnMediumBuffers.end(),
                     [](const std::deque<DataPacket>& queue) {
                       return !queue.empty();
                     });
}

bool
AuvEdcExperiment::RunReviewedIngressSelfTests()
{
  if (!m_reviewedIngressSelfTest) return false;
  uint32_t checks = 0;
  auto check = [&](bool ok, const char* label) {
    NS_ABORT_MSG_IF(!ok, "Reviewed ingress integration self-test failed: " << label);
    ++checks;
  };
  check(ReviewedCablePriority(Priority::HIGH)==closure_ingress::Priority::High,
        "HIGH main wire value maps to HIGH cable service despite opposite enum order");
  check(ReviewedCablePriority(Priority::MEDIUM)==closure_ingress::Priority::Medium,
        "MEDIUM main wire value maps to MEDIUM cable service");
  check(ReviewedCablePriority(Priority::LOW)==closure_ingress::Priority::Low,
        "LOW main wire value maps to LOW cable service despite opposite enum order");
  auto setup = [](AuvEdcExperiment& e) {
    e.m_protocol = ProtocolMode::CONTRIBUTION;
    e.m_reviewedTransport = e.m_reviewedIngress = true;
    e.m_numDdn = 1; e.m_simStop = e.m_trafficStop = 30.0;
    e.m_ddnBufferCapacityBytes = 164; e.m_ddnRecordOverheadBytes = 64;
    e.m_ddnByteBudgetScope = "resident";
    e.m_ddnBuffers.resize(1); e.m_ddnHighBuffers.resize(1);
    e.m_ddnMediumBuffers.resize(1); e.m_ddnLowBuffers.resize(1);
    e.m_storageHighWaterBytes.assign(1, 0); e.m_storageHighWaterRecords.assign(1, 0);
    e.m_residentHighWaterBytes.assign(1, 0); e.m_residentHighWaterRecords.assign(1, 0);
    e.m_directWaiting.resize(1); e.m_directActive.assign(1, false);
    e.m_mobileWaiting.resize(1); e.m_mobileActive.assign(1, false);
    e.m_reviewedIngressArchiveBytes = 10000;
    e.m_reviewedIngressSourceRamBytes = e.m_reviewedIngressGatewayRamBytes = 1000;
    e.m_reviewedIngressBitRate = 1000.0; e.m_reviewedIngressWireBitsPerByte = 10;
    e.m_reviewedIngressDataHeaderBytes = e.m_reviewedIngressAckBytes = 10;
    e.m_reviewedIngressPropagationS = 0.1; e.m_reviewedIngressDurableWriteS = 0.2;
    e.m_reviewedIngressTurnaroundS = 0.1; e.m_reviewedIngressRetryS = 1;
    e.m_reviewedIngressAckTimeoutS = 1; e.m_reviewedIngressMaxAttempts = 2;
    e.m_reviewedIngressRetentionS = 100;
    e.m_reviewedIngressSourceTxW = 2; e.m_reviewedIngressSourceRxW = 3;
    e.m_reviewedIngressGatewayTxW = 4; e.m_reviewedIngressGatewayRxW = 5;
  };
  auto record = [](uint64_t id, double released) {
    return edc_replay::Event{released, id, -1, edc_replay::Priority::Low,
                             100, 0, "synthetic-window-" + std::to_string(id), 0.0};
  };
  {
    AuvEdcExperiment e; setup(e); e.ReviewedIngressInitialize();
    Simulator::Schedule(Seconds(5), &AuvEdcExperiment::GenerateReplayPacket, &e, record(1, 5));
    Simulator::Schedule(Seconds(5.5), [&]() {
      e.ReviewedIngressRefresh(5.5);
      check(e.m_stats.generated == 1 && e.m_stats.reachedDdn == 0 &&
            e.ReviewedIngressSourceHeld() == 1, "generated source record is not free gateway ingress");
      check(e.m_reviewedIngressStates[0].source_staging_ram_bytes == 174 &&
            e.m_reviewedIngressStates[0].conservation_ok, "pending horizon retains bounded source custody and staging");
    });
    Simulator::Schedule(Seconds(6.8), [&]() {
      e.ReviewedIngressRefresh(6.8);
      const auto& p = e.m_ddnLowBuffers[0].front();
      check(p.id == 1 && p.createdAt == 0 && std::abs(p.ddnArrivalAt - 6.4) < 1e-6,
            "oldest sample and exact finite ingress arrival preserved");
      check(e.m_reviewedIngressStates[0].archive_resident_bytes == 164 &&
            e.m_reviewedIngressStates[0].source_unconfirmed_records == 0,
            "source logger archive retained after finite reverse ACK");
      e.m_pendingDirect.emplace(1, PendingDirectPacket{p, 0, 0, 0, EventId()});
      e.m_ddnLowBuffers[0].pop_front();
      ReviewedLegState leg; leg.packet = p; leg.token = 1; leg.leg = 2; leg.arrived = true;
      e.m_reviewedLegs.emplace(1, leg); e.m_reviewedLegTokens.emplace(std::make_pair(1, 2), 1);
      check(e.DdnResidentStorage(0).second == 164 &&
            e.m_reviewedIngressStates[0].raw_gateway_resident_bytes == 164,
            "receiver commitment retains physical sender storage until ACK");
    });
    Simulator::Schedule(Seconds(7), [&]() {
      e.m_pendingDirect.erase(1); e.m_reviewedLegs.at(1).terminal = true;
      e.FinishDirectPacket(0, 1); e.ReviewedIngressRefresh(7);
      check(e.m_reviewedIngressStates[0].raw_gateway_records == 0 &&
            e.m_reviewedIngressStates[0].counters.forwarded_records == 1 &&
            e.m_reviewedIngressStates[0].archive_resident_bytes == 164,
            "sender ACK release removes only gateway copy, retaining logger history");
      check(std::abs(e.m_reviewedIngressSourceEnergyJ - 2.5) < 1e-9 &&
            std::abs(e.m_reviewedIngressGatewayEnergyJ - 5.9) < 1e-9,
            "separate source and gateway active cable energy exact");
    });
    Simulator::Stop(Seconds(8)); Simulator::Run(); Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.ReviewedIngressInitialize();
    e.EnqueueContribPacket(0, DataPacket{900, 0, 0, 0, Priority::HIGH, 100});
    e.GenerateReplayPacket(record(2, 0));
    Simulator::Schedule(Seconds(10), [&]() {
      e.ReviewedIngressRefresh(10);
      check(e.m_stats.bufferDropped == 0 && e.m_stats.reachedDdn == 0 &&
            e.ReviewedIngressSourceHeld() == 1, "incoming ingress NACK is not a permanent main buffer drop");
      check(e.m_reviewedIngressStates[0].counters.data_attempts == 2 &&
            !e.m_reviewedIngressLinks[0]->NextEventTime(), "backpressure retries bounded before explicit space credit");
      e.m_ddnHighBuffers[0].pop_front();
      e.ReviewedIngressRelease(0, 900, closure_ingress::Removal::Forwarded);
    });
    Simulator::Schedule(Seconds(15), [&]() {
      e.ReviewedIngressRefresh(15);
      check(e.m_stats.reachedDdn == 1 && e.m_ddnLowBuffers[0].size() == 1 &&
            e.m_reviewedIngressStates[0].counters.credit_frames == 1,
            "higher-priority space release sends finite credit and admits retained source");
      e.EnqueueContribPacket(0, DataPacket{901, 0, 15, 15, Priority::HIGH, 100});
      e.ReviewedIngressRefresh(15);
      check(e.m_stats.bufferDroppedLow == 1 && e.m_reviewedIngressStates[0].raw_gateway_records == 0 &&
            e.m_reviewedIngressStates[0].archive_resident_bytes == 164,
            "authoritative priority eviction releases raw mirror without deleting history");
    });
    Simulator::Schedule(Seconds(20), [&]() {
      e.ReviewedIngressRefresh(20);
      check(e.m_reviewedIngressStates[0].counters.data_attempts == 3 &&
            e.m_reviewedIngressStates[0].conservation_ok,
            "evicted raw history never silently replayed");
    });
    Simulator::Stop(Seconds(21)); Simulator::Run(); Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedIngressArchiveBytes = 164;
    e.ReviewedIngressInitialize(); e.GenerateReplayPacket(record(3, 0));
    e.GenerateReplayPacket(record(4, 0));
    check(e.m_stats.generated == 2 && e.ReviewedIngressArchiveOverflow() == 1 &&
          e.ReviewedIngressSourceHeld() == 1 && e.m_stats.bufferDropped == 0,
          "finite source archive overflow is a separate generated-record outcome");
    Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedIngress = false;
    e.GenerateReplayPacket(record(5, 0));
    check(e.m_stats.generated == 1 && e.m_stats.reachedDdn == 1 &&
          e.m_ddnLowBuffers[0].size() == 1 && e.m_reviewedIngressLinks.empty(),
          "disabled ingress preserves legacy immediate replay behavior");
    Simulator::Destroy();
  }
  for (double rate : {6000.0, 9600.0})
    {
      AuvEdcExperiment e; setup(e); e.m_reviewedIngressBitRate = rate;
      e.ReviewedIngressInitialize();
      const double released = 0.333333333333;
      const double expected = released + 1100.0 / rate + 0.1 + 0.2;
      Time release = Seconds(released);
      if (release.GetSeconds() < released) release += NanoSeconds(1);
      Simulator::Schedule(release, &AuvEdcExperiment::GenerateReplayPacket, &e, record(6, released));
      Simulator::Schedule(Seconds(2), [&]() {
        e.ReviewedIngressRefresh(2);
        const double arrived = e.m_ddnLowBuffers[0].front().ddnArrivalAt;
        check(arrived + 1e-12 >= expected && arrived - expected <= 1.0001e-9,
              "fractional serial rate rounds admission upward by at most one ns");
        check(e.m_reviewedIngressStates[0].conservation_ok &&
              e.m_reviewedIngressStates[0].source_unconfirmed_records == 0,
              "fractional serial rate preserves finite reverse ACK and custody");
      });
      Simulator::Stop(Seconds(3)); Simulator::Run(); Simulator::Destroy();
    }
  {
    AuvEdcExperiment e; setup(e); e.m_ddnBufferCapacityBytes=328;
    e.ReviewedIngressInitialize();
    auto first=record(10,0), second=record(11,0); second.window_id=first.window_id;
    e.GenerateReviewedReplayBatch({first,second});
    Simulator::Schedule(Seconds(5),[&]() {
      e.ReviewedIngressRefresh(5);
      check(e.m_ddnLowBuffers[0].size()==2,"completed raw window produces the original two chunks");
      const auto& a=e.m_ddnLowBuffers[0][0]; const auto& b=e.m_ddnLowBuffers[0][1];
      check(a.windowNumericId==10 && b.windowNumericId==10 && a.windowPayloadBytes==200 &&
            b.windowPayloadBytes==200 && a.windowChunkCount==2 && b.windowChunkCount==2 &&
            a.windowChunkIndex==0 && b.windowChunkIndex==1,
            "producer-local complete-window descriptors survive actual finite gateway admission");
      check(std::abs(a.ddnArrivalAt-1.64)<1e-8 && a.createdAt==0 && b.createdAt==0,
            "24-byte raw descriptor is serialized without changing acquisition age or scientific payload");
      check(e.m_reviewedIngressStates[0].archive_resident_bytes==328 && e.m_stats.generated==2,
            "descriptor fits declared metadata allocation with no duplicated raw generation");
    });
    Simulator::Stop(Seconds(6)); Simulator::Run(); Simulator::Destroy();
  }
  auto verticalSetup=[&](AuvEdcExperiment& e) {
    setup(e); e.m_reviewedVerticalIngress=true;
    e.m_numBsn=1; e.m_numDrn=1; e.m_sinks=1;
    e.m_drns.push_back({0,NodeRole::DRN,Vector(0,0,1000),1000});
    e.m_ddns.push_back({0,NodeRole::DDN,Vector(0,0,500),1000});
    e.m_reviewedAckPayloadBytes=16; e.m_reviewedMaxAttempts=3;
    for(uint32_t i=0;i<2;++i) {
      auto app=CreateObject<EdcForwardingApp>();
      app->SetClosureTransport(std::make_shared<ClosureTransportState>());
      app->SetFragmentation(64,6000); e.m_reviewedApps.push_back(app);
    }
    // No nodes/devices are created. ACK dispatch is observed below and a
    // transport confirmation is explicitly injected by the component fixture.
    e.ReviewedIngressInitialize();
  };
  auto upstream=[&](AuvEdcExperiment& e,uint64_t id,Priority priority,uint32_t bytes) {
    DataPacket packet{id,0,0,0,priority,bytes};
    e.m_stats.generated++; e.AccountGeneratedPriority(priority);
    e.m_pendingAcoustic.emplace(id,PendingAcousticPacket{packet,0,1,EventId()});
    ReviewedLegState state; state.packet=packet; state.token=id; state.leg=1;
    state.sender=0; state.receiver=1; state.attempts=1;
    e.m_reviewedLegs.emplace(id,state); e.m_reviewedLegTokens.emplace(std::make_pair(id,uint8_t(1)),id);
    EdcHeader h; h.m_reviewed=true; h.m_token=id; h.m_leg=1; h.m_id=id;
    h.m_srcId=0; h.m_dstId=1; h.m_attempt=1; h.m_payloadBytes=bytes;
    h.m_prio=static_cast<uint8_t>(priority); return h;
  };
  auto conserved=[&](AuvEdcExperiment& e) {
    return e.m_stats.generated==e.BufferedAtDdn()+e.m_pendingAcoustic.size()-
      e.ReviewedCommittedPending()+e.ReviewedIngressSourceHeld()+e.ReviewedIngressArchiveOverflow()+
      e.ReviewedIngressSourceUnavailable();
  };
  {
    AuvEdcExperiment e; verticalSetup(e); const auto h=upstream(e,42,Priority::HIGH,64);
    Simulator::Schedule(Seconds(2),[&]() { e.ReviewedArrival(h); });
    Simulator::Schedule(Seconds(2.3),[&]() {
      e.ReviewedIngressRefresh(2.3);
      check(!e.m_reviewedLegs.at(42).arrived && e.m_reviewedAckSent==0 &&
            e.m_reviewedIngressStates[0].source_custody_records==1 && conserved(e),
            "complete acoustic receipt waits in finite shared cable storage without early upstream ACK");
      e.ReviewedArrival(h);
      check(e.m_reviewedVertical.size()==1 && e.m_reviewedIngressStates[0].counters.data_attempts==1,
            "duplicate upstream receipt reuses one bounded local transaction");
    });
    Simulator::Schedule(Seconds(3.05),[&]() {
      e.ReviewedIngressRefresh(3.05);
      check(e.m_reviewedLegs.at(42).arrived && e.m_stats.reachedDdn==1 && e.m_reviewedAckSent==0 &&
            e.m_pendingAcoustic.size()==1 && conserved(e),
            "durable gateway commitment precedes cable confirmation and conserves retained BSN copy");
      const auto& p=e.m_ddnHighBuffers[0].front();
      check(p.sourceBsn==0 && p.createdAt==0 && std::abs(p.ddnArrivalAt-3.04)<1e-8,
            "HIGH source identity and original age include finite vertical latency");
    });
    Simulator::Schedule(Seconds(3.4),[&]() {
      e.ReviewedIngressRefresh(3.4);
      check(e.m_reviewedVertical.at(42).cableConfirmed && e.m_reviewedAckSent==1 &&
            e.m_reviewedConfirmed==0 && e.m_pendingAcoustic.size()==1 && conserved(e),
            "only actual cable ACK enables reverse acoustic ACK; dispatch is not BSN confirmation");
      EdcHeader ack=h; ack.m_ack=true; ack.m_srcId=1; ack.m_dstId=0; ack.m_payloadBytes=16;
      e.ReviewedArrival(ack);
      check(e.m_stats.acousticAcks==1 && e.m_pendingAcoustic.empty() && conserved(e),
            "only reverse acoustic receipt releases BSN sender custody");
      e.ReviewedArrival(h);
      check(e.m_stats.reachedDdn==1 && e.m_reviewedIngressStates[0].archive_resident_bytes==128,
            "late duplicate cannot duplicate gateway delivery or retained archive");
    });
    Simulator::Stop(Seconds(4)); Simulator::Run(); Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedIngressArchiveBytes=164;
    // Apply the endpoint fixture after the shared archive capacity is fixed.
    e.m_reviewedVerticalIngress=true; e.m_numBsn=1; e.m_numDrn=1;
    e.m_drns.push_back({0,NodeRole::DRN,Vector(0,0,1000),1000});
    e.m_ddns.push_back({0,NodeRole::DDN,Vector(0,0,500),1000});
    e.ReviewedIngressInitialize(); e.GenerateReplayPacket(record(1,0));
    const auto h=upstream(e,43,Priority::HIGH,64);
    Simulator::Schedule(Seconds(2),[&]() {
      e.ReviewedArrival(h); e.ReviewedIngressRefresh(2);
      check(e.m_reviewedVertical.empty() && e.m_reviewedAckSent==0 &&
            e.m_reviewedIngressStates[0].counters.upstream_backpressure_rejections==1 &&
            e.m_reviewedIngressStates[0].counters.archive_overflow_records==0 && conserved(e),
            "shared source archive saturation is upstream backpressure, not a fabricated new loss");
    });
    Simulator::Stop(Seconds(3)); Simulator::Run(); Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; verticalSetup(e);
    const auto high=upstream(e,44,Priority::HIGH,100), medium=upstream(e,45,Priority::MEDIUM,100);
    e.ReviewedArrival(high);
    Simulator::Schedule(Seconds(2),[&]() { e.ReviewedArrival(medium); });
    Simulator::Schedule(Seconds(10),[&]() {
      e.ReviewedIngressRefresh(10);
      check(e.m_reviewedLegs.at(44).arrived && !e.m_reviewedLegs.at(45).arrived &&
            e.m_reviewedAckSent==1 && e.m_stats.bufferDropped==0 && conserved(e),
            "full higher-priority gateway NACKs MEDIUM without upstream confirmation or permanent source drop");
      check(e.m_reviewedIngressStates[0].counters.gateway_rejections==2 &&
            !e.m_reviewedIngressLinks[0]->NextEventTime() &&
            e.m_reviewedIngressStates[0].source_custody_records==1,
            "rejected MEDIUM retries stop at the finite cable epoch bound pending explicit credit");
    });
    Simulator::Stop(Seconds(11)); Simulator::Run(); Simulator::Destroy();
  }
  std::cout << "REVIEWED_INGRESS_SELF_TEST_PASS checks=" << checks
            << " full_validation=false network_runs=0 synthetic_parameters=true\n";
  return true;
}

bool
AuvEdcExperiment::RunReviewedSelfTests()
{
  if (!m_reviewedSelfTest) return false;
  uint32_t checks = EdcForwardingApp::RunReviewedFrameSelfTests();
  checks += EdcForwardingApp::RunControlSelfTests();
  auto check = [&](bool ok, const char* label) {
    NS_ABORT_MSG_IF(!ok, "Reviewed custody self-test failed: " << label);
    ++checks;
  };
  auto setup = [](AuvEdcExperiment& e) {
    e.m_reviewedTransport = true; e.m_closureMode = true;
    e.m_reviewedAckPayloadBytes = 16; e.m_reviewedMaxAttempts = 3;
    e.m_numBsn = 0; e.m_numDrn = 0; e.m_numDdn = 1; e.m_sinks = 1;
    e.m_directWaiting.resize(1); e.m_directActive.assign(1, true);
    const DataPacket packet{42, 0, 0.0, 0.0, Priority::HIGH, 64};
    e.m_pendingDirect.emplace(42, PendingDirectPacket{packet, 0, 1, 0, EventId()});
    e.m_directWaiting[0].push_back(42);
    ReviewedLegState s; s.packet = packet; s.token = 1; s.leg = 2;
    s.sender = 0; s.receiver = 1; s.attempts = 2;
    e.m_reviewedLegs.emplace(1, s);
    e.m_reviewedLegTokens.emplace(std::make_pair(42ULL, uint8_t(2)), 1);
    for (uint32_t id = 0; id < 2; ++id)
      {
        auto app = CreateObject<EdcForwardingApp>();
        app->SetClosureTransport(std::make_shared<ClosureTransportState>());
        app->SetFragmentation(64, 6000);
        e.m_reviewedApps.push_back(app);
      }
  };
  auto data = []() {
    EdcHeader h; h.m_reviewed = true; h.m_token = 1; h.m_leg = 2;
    h.m_id = 42; h.m_srcId = 0; h.m_dstId = 1; h.m_attempt = 1;
    h.m_payloadBytes = 64; return h;
  };
  auto ack = [&]() {
    auto h = data(); h.m_ack = true; h.m_srcId = 1; h.m_dstId = 0;
    h.m_payloadBytes = 16; return h;
  };
  {
    AuvEdcExperiment e; setup(e);
    e.ReviewedArrival(data());
    check(e.m_stats.surfaceDelivered == 1 && e.m_stats.directAcks == 0 &&
          e.m_pendingDirect.size() == 1 && e.ReviewedCommittedPending() == 1,
          "DATA commits at receiver while sender retains one unconfirmed copy");
    e.ReviewedArrival(data());
    check(e.m_stats.surfaceDelivered == 1 && e.m_reviewedAckSent == 2 &&
          e.m_reviewedDuplicateData == 1, "lost ACK retry re-ACKs without duplicate delivery");
    e.ReviewedArrival(ack());
    check(e.m_stats.directAcks == 1 && e.m_pendingDirect.empty() &&
          e.m_reviewedLegs.at(1).confirmed && e.ReviewedCommittedPending() == 0,
          "earlier-attempt ACK confirms the same lifetime token and releases sender");
    e.ReviewedArrival(ack());
    check(e.m_stats.directAcks == 1 && e.m_stats.surfaceDelivered == 1 &&
          e.m_reviewedStaleFrames == 1, "late duplicate ACK cannot deliver or confirm twice");
    check(!e.ReviewedFrameActive(data(), 0) && e.ReviewedFrameActive(data(), 2) &&
          e.ReviewedFrameActive(ack(), 1), "only local obsolete DATA is cancelled, not relay/ACK traffic");
  }
  {
    AuvEdcExperiment e; setup(e);
    e.ReviewedArrival(ack());
    check(e.m_stats.directAcks == 0 && e.m_reviewedStaleFrames == 1,
          "ACK before receiver commitment is rejected");
    e.EndReviewedLeg(1, false, false);
    check(e.m_reviewedRetryDropped == 1 && e.m_pendingDirect.empty(),
          "unreceived exhausted source transaction is accounted exactly once");
    e.ReviewedArrival(data());
    check(e.m_stats.surfaceDelivered == 1 && e.m_reviewedRetryDropped == 0 &&
          e.m_reviewedLateRecovered == 1 && e.m_reviewedLegs.at(1).terminal &&
          !e.m_reviewedLegs.at(1).confirmed, "late in-flight DATA is real delivery without source resurrection");
    e.ReviewedArrival(ack());
    check(e.m_stats.directAcks == 0 && e.m_reviewedUnconfirmedReceived == 1,
          "late ACK cannot restart or confirm a terminated sender transaction");
  }
  {
    AuvEdcExperiment e; setup(e);
    e.ReviewedArrival(data());
    e.EndReviewedLeg(1, false, true);
    check(e.m_stats.surfaceDelivered == 1 && e.m_stats.deadlineExpired == 0 &&
          e.m_reviewedUnconfirmedReceived == 1 && e.m_pendingDirect.empty(),
          "ACK deadline cannot turn already received data into a second drop");
    e.EndReviewedLeg(1, false, true);
    check(e.m_reviewedUnconfirmedReceived == 1, "terminal accounting is idempotent");
  }
  {
    AuvEdcExperiment e; setup(e);
    e.m_protocol = ProtocolMode::CONTRIBUTION;
    e.m_pendingDirect.clear(); e.m_directWaiting[0].clear();
    e.m_ddnBuffers.resize(1); e.m_ddnHighBuffers.resize(1);
    e.m_ddnMediumBuffers.resize(1); e.m_ddnLowBuffers.resize(1);
    e.m_storageHighWaterRecords.assign(1, 0); e.m_storageHighWaterBytes.assign(1, 0);
    e.m_residentHighWaterRecords.assign(1, 0); e.m_residentHighWaterBytes.assign(1, 0);
    e.m_ddnBufferCapacity = 1; e.m_ddnBufferCapacityBytes = 64;
    e.m_ddnRecordOverheadBytes = 0; e.m_ddnByteBudgetScope = "resident";
    auto& s = e.m_reviewedLegs.at(1); s.leg = 1;
    e.m_pendingAcoustic.emplace(42, PendingAcousticPacket{s.packet, 0, 0, EventId()});
    e.m_ddnHighBuffers[0].push_back(DataPacket{99, 0, 0, 0, Priority::HIGH, 64});
    auto frame = data(); frame.m_leg = 1;
    e.ReviewedArrival(frame);
    check(!s.arrived && e.m_reviewedAckSent == 0 && e.m_pendingAcoustic.size() == 1 &&
          e.m_stats.bufferDropped == 0 && e.m_stats.acousticDelivered == 0 &&
          e.m_reviewedUpstreamAdmissionRejected == 1 && e.m_reviewedUpstreamCompleteReceipts == 1 &&
          e.m_reviewedUpstreamReceivedIds.size() == 1,
          "full equal-priority queue rejects receipt without ACK, custody transfer or permanent drop");
    e.m_ddnHighBuffers[0].clear(); frame.m_attempt = 2;
    e.ReviewedArrival(frame);
    check(s.arrived && e.m_ddnHighBuffers[0].size() == 1 && e.m_reviewedAckSent == 1 &&
          e.m_stats.acousticDelivered == 1 && e.m_stats.reachedDdn == 1 &&
          e.m_pendingAcoustic.size() == 1 && e.ReviewedCommittedPending(1) == 1 &&
          e.m_reviewedUpstreamAdmissionId == 0,
          "space release allows next DATA retry to admit exactly once before ACK");
    e.ReviewedArrival(frame);
    check(e.m_ddnHighBuffers[0].size() == 1 && e.m_stats.acousticDelivered == 1 &&
          e.m_reviewedAckSent == 2 && e.m_reviewedUpstreamCompleteReceipts == 3 &&
          e.m_reviewedUpstreamReceivedIds.size() == 1 && e.m_stats.bufferDropped == 0,
          "duplicate after admission re-ACKs without duplicate queueing or delivery counters");
    auto confirmation = ack(); confirmation.m_leg = 1; confirmation.m_attempt = 2;
    e.ReviewedArrival(confirmation);
    check(s.confirmed && e.m_pendingAcoustic.empty() && e.m_stats.acousticAcks == 1 &&
          e.m_ddnHighBuffers[0].size() == 1 && e.ReviewedCommittedPending(1) == 0,
          "only received ACK releases upstream sender after admitted receiver custody");
  }
  // The tests enqueue reverse frames but never run a network or PHY event.
  Simulator::Destroy();
  // Local driver-clock component probes only; no device/channel is installed.
  {
    ReviewedLocalTxTag tag; tag.token=9; tag.attempt=3; tag.fragment=562; tag.fragments=563;
    auto packet=Create<Packet>(); packet->AddPacketTag(tag);
    ReviewedLocalTxTag decoded;
    check(packet->Copy()->PeekPacketTag(decoded) && decoded.token==9 && decoded.attempt==3 &&
          decoded.fragment==562 && decoded.fragments==563,
          "local TX completion tag survives packet copies without adding wire bytes");
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedPhyEnergy=true;
    auto& s=e.m_reviewedLegs.at(1); s.localExpectedFragments=2;
    ReviewedLocalTxTag tag; tag.token=1; tag.attempt=1; tag.fragments=2;
    e.ReviewedLocalTxOutcome(tag,0,true,100);
    tag.attempt=2; e.ReviewedLocalTxOutcome(tag,1,true,100);
    check(s.localFragmentOutcomes.empty() && !s.localCompletionScheduled,
          "old-attempt and relay completion cannot change current local timer");
    e.ReviewedLocalTxOutcome(tag,0,true,1);
    check(s.localFragmentOutcomes.size()==1 && !s.localCompletionScheduled,
          "first frame does not start ACK grace before remaining physical frames");
    tag.fragment=1; e.ReviewedLocalTxOutcome(tag,0,true,2);
    e.ReviewedLocalTxOutcome(tag,0,false,99);
    check(s.localCompletionScheduled && s.localFragmentOutcomes.size()==2 &&
          !s.localTxRejected && s.timeout.GetTs()==static_cast<uint64_t>(Seconds(2).GetTimeStep()),
          "final physical frame schedules exact completion and duplicate outcomes are ignored");
    Simulator::Stop(Seconds(2.1)); Simulator::Run();
    check(e.m_reviewedPhysicalAttemptsSettled==1 && !s.terminal &&
          s.timeout.GetTs()==static_cast<uint64_t>(Seconds(14).GetTimeStep()),
          "unchanged 12-second ACK grace starts at actual 2-second physical completion");
    Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedPhyEnergy=true; e.m_reviewedMaxAttempts=2;
    auto& s=e.m_reviewedLegs.at(1); s.localExpectedFragments=2;
    ReviewedLocalTxTag tag; tag.token=1; tag.attempt=2; tag.fragments=2; tag.fragment=1;
    e.ReviewedLocalTxOutcome(tag,0,false,0);
    check(!s.localCompletionScheduled && s.localTxRejected,
          "final-index local refusal waits for earlier frames still queued at MAC");
    tag.fragment=0; e.ReviewedLocalTxOutcome(tag,0,true,2);
    check(s.localCompletionScheduled && s.timeout.GetTs()==static_cast<uint64_t>(Seconds(2).GetTimeStep()),
          "rejected attempt settles only after latest admitted waveform finishes");
    Simulator::Stop(Seconds(2.1)); Simulator::Run();
    check(s.terminal && !s.confirmed && e.m_reviewedLocalRejectedAttempts==1 &&
          e.m_reviewedRetryDropped==1 && e.m_pendingDirect.empty(),
          "known local refusal follows bounded retry exhaustion rather than whole-record deadline");
    Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e); e.m_reviewedPhyEnergy=true;
    auto& s=e.m_reviewedLegs.at(1); s.localExpectedFragments=1;
    ReviewedLocalTxTag tag; tag.token=1; tag.attempt=2; tag.fragments=1;
    e.ReviewedLocalTxOutcome(tag,0,true,1);
    // A delayed valid ACK from an earlier attempt can arrive while a newer
    // attempt's local-completion event is pending. It must win exactly once.
    s.arrived=true; e.m_stats.surfaceDelivered=1;
    e.ReviewedArrival(ack());
    Simulator::Stop(Seconds(1.1)); Simulator::Run();
    check(s.terminal && s.confirmed && e.m_stats.directAcks==1 &&
          e.m_reviewedPhysicalAttemptsSettled==0,
          "valid earlier ACK cancels pending local completion and cannot restart its timer");
    e.ReviewedLocalTxOutcome(tag,0,false,2);
    check(e.m_stats.directAcks==1 && !s.timeout.IsRunning(),
          "late physical refusal after confirmation cannot resurrect sender custody");
    Simulator::Destroy();
  }
  {
    AuvEdcExperiment e; setup(e);
    std::vector<uint8_t> observed;
    e.SetReviewedControlHandler(1,[&](uint32_t source,uint32_t receiver,uint64_t nonce,const std::vector<uint8_t>& bytes) {
      if(source==0 && receiver==1 && nonce==42) observed=bytes;
    });
    EdcHeader h; h.m_reviewed=true; h.m_leg=255; h.m_controlKind=1;
    h.m_srcId=0; h.m_dstId=1; h.m_id=42; h.m_payloadBytes=3; h.m_controlExpiryNs=1000000000;
    const std::vector<uint8_t> bytes{255,0,37}; e.ReceiveReviewedControl(h,bytes);
    check(observed==bytes && e.m_reviewedControlReceived==1 && e.m_stats.generated==0 &&
          e.m_stats.surfaceDelivered==0 && e.m_stats.acousticTransfers==0 && !e.m_reviewedLegs.at(1).arrived,
          "received control body invokes separate handler without touching DATA identity or accounting");
    h.m_controlKind=2; e.ReceiveReviewedControl(h,bytes);
    check(e.m_reviewedControlRejected==1 && e.m_reviewedControlReceived==1 && e.m_stats.directAcks==0,
          "unregistered control kind cannot manufacture DATA or ACK delivery");
    check(e.SendReviewedControl(0,1,1,42,bytes)==0 && e.m_reviewedControlEnqueued==0,
          "controls remain disabled by default and cannot silently acquire resources");
    Simulator::Destroy();
  }
  std::cout << "REVIEWED_SELF_TEST_PASS checks=" << checks
            << " version=reviewed-ack-v1 full_validation=false\n";
  return true;
}

bool
AuvEdcExperiment::RunClosureTransportSelfTests()
{
  if (!m_closureTransportSelfTest) return false;
  uint32_t checks = EdcForwardingApp::RunClosureSelfTests();
  auto check = [&](bool ok, const char* label) {
    NS_ABORT_MSG_IF(!ok, "Closure transport self-test failed: " << label);
    ++checks;
  };
  AuvEdcExperiment e;
  e.m_closureMode = true;
  e.m_energyModel = "hardware";
  e.m_acousticBitRate = 6000.0;
  e.m_closureAcousticTxPowerW = 12.0;
  e.m_closureAcousticRxPowerW = 2.0;
  check(std::abs(e.AcousticTxEnergyJ(750, 10.0) - 12.0) < 1e-12,
        "explicit TX power times application-wire duration");
  check(std::abs(e.AcousticTxEnergyJ(750, 1000.0) - 12.0) < 1e-12,
        "closure TX does not inherit a different vendor's distance-power steps");
  check(std::abs(e.AcousticRxEnergyJ(750) - 2.0) < 1e-12,
        "explicit RX power times application-wire duration");
  e.m_closureMode = false;
  check(std::abs(e.AcousticTxEnergyJ(750, 100.0) - 5.5) < 1e-12,
        "legacy hardware TX formula remains selected when closure is disabled");
  std::cout << "CLOSURE_TRANSPORT_SELF_TEST_PASS checks=" << checks
            << " version=transport-slice-v1 full_validation=false\n";
  return true;
}

bool
AuvEdcExperiment::RunHardwareSelfTests()
{
  if (!m_hardwareSelfTest) return false;
  uint32_t checks = 0;
  auto check = [&](bool ok, const char* label) {
    NS_ABORT_MSG_IF(!ok, "Hardware self-test failed: " << label);
    ++checks;
  };
  auto setup = [](AuvEdcExperiment& e, uint64_t capacity) {
    e.m_protocol = ProtocolMode::CONTRIBUTION;
    e.m_numDdn = 1;
    e.m_ddnBufferCapacityBytes = capacity;
    e.m_ddnByteBudgetScope = "resident";
    e.m_ddnBuffers.resize(1);
    e.m_ddnHighBuffers.resize(1);
    e.m_ddnMediumBuffers.resize(1);
    e.m_ddnLowBuffers.resize(1);
    e.m_storageHighWaterBytes.assign(1, 0);
    e.m_storageHighWaterRecords.assign(1, 0);
    e.m_residentHighWaterBytes.assign(1, 0);
    e.m_residentHighWaterRecords.assign(1, 0);
  };
  auto packet = [](uint64_t id, Priority priority, uint32_t bytes) {
    return DataPacket{id, 0, 0.0, 0.0, priority, bytes};
  };
  {
    AuvEdcExperiment e; setup(e, 300);
    for (uint32_t i = 1; i <= 3; ++i)
      check(e.EnqueueContribPacket(0, packet(i, Priority::LOW, 80)), "initial LOW admission");
    check(e.EnqueueContribPacket(0, packet(4, Priority::HIGH, 200)), "multi-victim admission");
    check(e.m_stats.bufferDroppedLow == 2 && e.DdnQueueBytes(0) == 280,
          "multiple lower-priority evictions exactly sized");
    check(e.m_ddnLowBuffers[0].front().id == 1, "evict tail, preserve oldest LOW");
  }
  {
    AuvEdcExperiment e; setup(e, 500);
    e.EnqueueContribPacket(0, packet(1, Priority::LOW, 100));
    check(!e.EnqueueContribPacket(0, packet(2, Priority::HIGH, 501)), "oversize record rejected");
    check(e.m_ddnLowBuffers[0].size() == 1 && e.m_stats.bufferDroppedLow == 0,
          "oversize arrival causes no collateral eviction");
  }
  {
    AuvEdcExperiment e; setup(e, 500);
    const DataPacket held = packet(1, Priority::HIGH, 400);
    e.m_pendingDirect.emplace(1, PendingDirectPacket{held, 0, 0, 0, EventId()});
    e.EnqueueContribPacket(0, packet(2, Priority::LOW, 30));
    check(!e.EnqueueContribPacket(0, packet(3, Priority::HIGH, 200)), "pending transmission reserves bytes");
    check(e.m_ddnLowBuffers[0].size() == 1 && e.m_stats.bufferDroppedLow == 0,
          "insufficient evictable space leaves queued victims intact");
    check(e.m_pendingDirect.size() == 1, "in-flight record never evicted");
    e.m_ddnHighBuffers[0].push_back(held);
    check(e.DdnResidentStorage(0).second == 430, "resident packet IDs deduplicated");
    e.m_ddnHighBuffers[0].pop_back();
    e.m_pendingDirect.erase(1);
    e.RequeueFrontDdnPacket(0, held);
    check(e.DdnResidentStorage(0).second == 430 && e.m_ddnHighBuffers[0].size() == 1,
          "ownership-preserving retransmission requeue");
  }
  {
    AuvEdcExperiment e; setup(e, 100);
    const DataPacket held = packet(1, Priority::LOW, 80);
    e.m_pendingOptical.emplace(1, PendingOpticalPacket{held, 0, 0.0, 1.0});
    check(!e.EnqueueContribPacket(0, packet(2, Priority::LOW, 30)), "optical in-flight bytes reserved");
    e.m_pendingOptical.erase(1);
    e.RequeueFrontDdnPacket(0, held);
    check(e.DdnResidentStorage(0).second == 80, "failed optical requeue retains allocation");
  }
  {
    AuvEdcExperiment e; setup(e, 100);
    e.m_ddnByteBudgetScope = "queue";
    const DataPacket held = packet(1, Priority::LOW, 80);
    e.m_pendingOptical.emplace(1, PendingOpticalPacket{held, 0, 0.0, 1.0});
    check(e.EnqueueContribPacket(0, packet(2, Priority::LOW, 30)), "queue-only scope explicit difference");
    e.m_pendingOptical.erase(1);
    e.RequeueFrontDdnPacket(0, held);
    check(e.DdnQueueBytes(0) == 30 && e.m_stats.bufferDroppedLow == 1,
          "queue-only requeue cannot overflow cap");
  }
  {
    AuvEdcExperiment e; setup(e, 100);
    e.m_ddnRecordOverheadBytes = 20;
    check(e.EnqueueContribPacket(0, packet(1, Priority::HIGH, 80)), "metadata counted at exact boundary");
    check(!e.EnqueueContribPacket(0, packet(2, Priority::HIGH, 1)), "equal priority never preempted");
    check(e.DdnResidentStorage(0).second == 100, "metadata occupancy exact");
  }
  {
    AuvEdcExperiment e; setup(e, 1000);
    e.m_ddnBufferCapacity = 1;
    e.EnqueueContribPacket(0, packet(1, Priority::LOW, 10));
    check(e.EnqueueContribPacket(0, packet(2, Priority::MEDIUM, 10)) &&
          e.m_stats.bufferDroppedLow == 1 && e.DdnQueueRecords(0) == 1,
          "independent record cap enforced");
    e.m_ddnBufferPolicy = "fifo";
    check(!e.EnqueueContribPacket(0, packet(3, Priority::HIGH, 10)), "FIFO does not preempt");
  }
  {
    AuvEdcExperiment e; setup(e, 0);
    e.m_ddnBufferCapacity = 1;
    e.m_ddnLowBuffers[0].push_back(packet(1, Priority::LOW, 10));
    e.m_ddnLowBuffers[0].push_back(packet(2, Priority::LOW, 10));
    check(e.EnqueueContribPacket(0, packet(3, Priority::HIGH, 10)) &&
          e.m_stats.bufferDroppedLow == 1 && e.DdnQueueRecords(0) == 2,
          "byte-disabled legacy one-eviction semantics preserved");
  }
  {
    AuvEdcExperiment e; setup(e, 100);
    const DataPacket held = packet(1, Priority::LOW, 80);
    e.m_pendingMobile.emplace(1, PendingMobilePacket{held, 0, 0, EventId()});
    check(e.DdnResidentStorage(0).second == 80, "mobile retained bytes included in audit");
    e.m_pendingAcoustic.emplace(2, PendingAcousticPacket{packet(2, Priority::HIGH, 50), 0, 0, EventId()});
    e.m_auvBuffer.push_back(packet(3, Priority::LOW, 50));
    check(e.DdnResidentStorage(0).second == 80, "upstream and AUV-owned records excluded");
  }
  {
    AuvEdcExperiment e;
    e.m_motionSpacing = 1750.0;
    e.m_motionContactDepth = 500.0;
    e.m_pipelineLength = 12600.0;
    const double meanSpeed = 6.087 / 3.6;
    e.m_motionHorizontalSpeed = meanSpeed / SinusoidalPathStretch(500.0, 1750.0);
    check(e.AuvPeakSpeed() > meanSpeed && e.AuvPeakSpeed() * 3.6 < 9.26,
          "geometric peak differs from admissible mean speed");
    const double duration = e.m_motionSpacing / e.m_motionHorizontalSpeed;
    const double energy = e.AuvOperationalEnergy(0.0, duration);
    const double meanPowerEnergy = duration * (e.m_auvFixedPowerW +
        e.m_auvPropulsionCoeff * meanSpeed * meanSpeed * meanSpeed);
    check(energy > meanPowerEnergy && energy < 1.1 * meanPowerEnergy,
          "convex cubic propulsion integrated above power of mean speed");
    e.m_motionContactDepth = 0.0;
    e.m_motionHorizontalSpeed = 2.0;
    const double flatEnergy = e.AuvOperationalEnergy(0.0, 0.37);
    check(std::abs(flatEnergy - 0.37 * (5.23 + 45.78 * 8.0)) < 1e-9,
          "partial-tick integration exact for constant speed");
  }
  // Optical accounting tests use deterministic frame windows, independently
  // of the unchanged channel RNG and service event schedule.
  auto opticalSetup = [&](AuvEdcExperiment& e) {
    setup(e, 1000000);
    e.m_energyModel = "hardware";
    e.m_opticalTxPowerW = 15.0;
    e.m_opticalRxPowerW = 10.0;
    e.m_opticalDataRateBps = 2500000.0;
    e.m_pipelineLength = 12600.0;
    e.m_opticalActive.assign(1, true);
    e.m_ddns.push_back(LogicalNode{0, NodeRole::DDN,
                                  e.GetAuvPosition(0.0), 30.0});
    e.m_opticalRng = CreateObject<UniformRandomVariable>();
    e.m_opticalRng->SetAttribute("Min", DoubleValue(0.5));
    e.m_opticalRng->SetAttribute("Max", DoubleValue(0.75));
  };
  for (int outcome = 0; outcome < 3; ++outcome)
    {
      AuvEdcExperiment e; opticalSetup(e);
      const DataPacket held = packet(7001, Priority::LOW, 25000);
      if (outcome == 2) e.m_ddns[0].position = Vector(1000000.0, 0.0, 0.0);
      e.m_pendingOptical.emplace(held.id,
          PendingOpticalPacket{held, 0, 0.0, outcome == 1 ? 0.0 : 1.0,
                               -0.08, 0.08, 0.0});
      e.CompleteOpticalTransfer(held.id);
      check(std::abs(e.m_stats.ddnEnergyJ - 1.2) < 1e-12 &&
            std::abs(e.m_stats.auvEnergyJ - 0.8) < 1e-12 &&
            std::abs(e.m_stats.energyConsumedJ - 2.0) < 1e-12,
            outcome == 0 ? "successful optical frame charged to both roles" :
            outcome == 1 ? "PER-failed optical frame charged to both roles" :
                           "contact-failed optical frame charged to both roles");
      check(e.m_pendingOptical.empty() &&
            (outcome == 0 ? e.m_stats.opticalTransfers == 1 && e.m_auvBuffer.size() == 1
                          : e.m_stats.opticalLostPackets == 1 && e.m_ddnLowBuffers[0].size() == 1),
            "optical accounting preserves success or requeue outcome");
      e.CompleteOpticalTransfer(held.id);
      e.SettlePendingOpticalEnergy(1.0);
      check(std::abs(e.m_stats.energyConsumedJ - 2.0) < 1e-12,
            "duplicate completion and final settlement do not charge twice");
      if (outcome == 1)
        {
          e.m_ddnLowBuffers[0].clear();
          e.m_pendingOptical.emplace(held.id,
              PendingOpticalPacket{held, 0, 0.0, 1.0, -0.08, 0.08, 0.0});
          e.CompleteOpticalTransfer(held.id);
          check(std::abs(e.m_stats.energyConsumedJ - 4.0) < 1e-12 &&
                e.m_stats.opticalLostPackets == 1 && e.m_stats.opticalTransfers == 1,
                "retry charges a fresh active frame after failed attempt");
        }
    }
  {
    AuvEdcExperiment e; opticalSetup(e);
    const DataPacket held = packet(7002, Priority::LOW, 25000);
    e.m_pendingOptical.emplace(held.id,
        PendingOpticalPacket{held, 0, 0.0, 1.0, 10.0, 0.08, 0.0});
    e.SettlePendingOpticalEnergy(9.0);
    e.SettlePendingOpticalEnergy(10.0);
    check(e.m_stats.energyConsumedJ == 0.0, "unelapsed optical window costs zero");
    e.SettlePendingOpticalEnergy(10.04);
    check(std::abs(e.m_stats.energyConsumedJ - 1.0) < 1e-11 &&
          std::abs(e.m_stats.ddnEnergyJ - 0.6) < 1e-11 &&
          std::abs(e.m_stats.auvEnergyJ - 0.4) < 1e-11 &&
          e.m_pendingOptical.size() == 1 && e.m_auvBuffer.empty(),
          "half-frame horizon charges elapsed energy and retains pending record");
    e.SettlePendingOpticalEnergy(10.04);
    e.SettlePendingOpticalEnergy(10.02);
    check(std::abs(e.m_stats.energyConsumedJ - 1.0) < 1e-11,
          "repeated or earlier optical settlement is idempotent");
    e.ChargeOpticalAttemptEnergy(e.m_pendingOptical.at(held.id), 10.08, true);
    e.SettlePendingOpticalEnergy(20.0);
    check(std::abs(e.m_stats.energyConsumedJ - 2.0) < 1e-11,
          "completion after partial settlement charges only remaining frame");
  }
  {
    AuvEdcExperiment e; opticalSetup(e);
    const DataPacket held = packet(7003, Priority::LOW, 25000);
    e.m_pendingOptical.emplace(held.id,
        PendingOpticalPacket{held, 0, 0.0, 1.0, -0.08, 0.08, 0.0});
    e.SettlePendingOpticalEnergy(0.0);
    e.CompleteOpticalTransfer(held.id);
    check(std::abs(e.m_stats.energyConsumedJ - 2.0) < 1e-12 &&
          e.m_stats.opticalTransfers == 1,
          "horizon at frame endpoint followed by completion is charged once");
  }
  {
    AuvEdcExperiment e;
    e.m_energyModel = "normalized";
    const DataPacket held = packet(7004, Priority::LOW, 25000);
    PendingOpticalPacket pending{held, 0, 20.0, 0.0, 0.0, 0.08, 0.0};
    const double fullEnergy = 0.001 + 25000.0 * 20.0 * e.m_opticalEnergyPerByteMeter;
    e.ChargeOpticalAttemptEnergy(pending, 0.04);
    check(std::abs(e.m_stats.energyConsumedJ - fullEnergy / 2.0) < 1e-12,
          "legacy abstract optical coefficient scales by elapsed frame fraction");
    e.ChargeOpticalAttemptEnergy(pending, 0.08, true);
    e.ChargeOpticalAttemptEnergy(pending, 1.0, true);
    check(std::abs(e.m_stats.energyConsumedJ - fullEnergy) < 1e-12 &&
          std::abs(e.m_stats.ddnEnergyJ - fullEnergy / 2.0) < 1e-12 &&
          std::abs(e.m_stats.auvEnergyJ - fullEnergy / 2.0) < 1e-12,
          "legacy abstract optical roles retain equal split without double charge");
  }
  {
    AuvEdcExperiment e;
    e.AccountDeliveredPriority(Priority::HIGH, 1.0, false, 64);
    e.AccountDeliveredPriority(Priority::MEDIUM, 2.0, false, 256);
    e.AccountDeliveredPriority(Priority::LOW, 3.0, false, 25000);
    e.AccountDeliveredPriority(Priority::LOW, 4.0, false, 11000);
    check(e.m_stats.lowDelivered == 2 && e.m_stats.lowDeliveredBytes == 36000,
          "variable final chunk contributes actual delivered bytes");
    check(e.m_stats.highDeliveredBytes == 64 && e.m_stats.mediumDeliveredBytes == 256,
          "fixed-size classes retain exact payload accounting");
  }
  std::cout << "HARDWARE_SELF_TEST_PASS checks=" << checks << '\n';
  return true;
}

void
AuvEdcExperiment::Run()
{
  m_priorityRng = CreateObject<UniformRandomVariable>();
  m_acousticReceptionRng = CreateObject<UniformRandomVariable>();
  m_opticalRng = CreateObject<UniformRandomVariable>();
  m_retryRng = CreateObject<UniformRandomVariable>();
  m_priorityRng->SetStream(100001);
  m_acousticReceptionRng->SetStream(100002);
  m_opticalRng->SetStream(100003);
  m_retryRng->SetStream(100004);
  // Poisson arrival process: inter-arrival times are Exp(mean = 1/lambda),
  // lambda = m_trafficLoad (pkt/s). Replaces the former deterministic 1/lambda period.
  m_arrivalRng = CreateObject<ExponentialRandomVariable>();
  m_arrivalRng->SetStream(100000);
  m_arrivalRng->SetAttribute("Mean", DoubleValue(1.0 / m_trafficLoad));
  BuildLogicalTopology();
  BuildNs3Topology();

  std::cout << "----------- Initializing AUV-Based EDC simulation -----------\n";
  if (m_auvInstantaneousPower || m_ddnBufferCapacityBytes > 0 || !m_storageCsv.empty())
    std::cout << std::setprecision(12)
              << "HARDWARE_EXTENSION ddnBufferCapacityBytes=" << m_ddnBufferCapacityBytes
              << " ddnRecordOverheadBytes=" << m_ddnRecordOverheadBytes
              << " ddnByteBudgetScope=" << m_ddnByteBudgetScope
              << " auvInstantaneousPower=" << m_auvInstantaneousPower
              << " auvPeakSpeedKmh=" << (m_auvInstantaneousPower ? AuvPeakSpeed() * 3.6 : -1.0)
              << "\n";
  std::cout << "Use sweeps like: ./ns3 run \"scratch/edc --trafficLoad=1 --nodes=126\"\n";
  
  std::unique_ptr<AnimationInterface> anim;
  if (m_enableAnimation && !m_animXml.empty())
    {
      anim = std::make_unique<AnimationInterface>(m_animXml);
      anim->EnablePacketMetadata(false);
      anim->SetMobilityPollInterval(Seconds(std::max(1.0, m_sampleInterval)));

      uint32_t nodeId = 0;
      for (uint32_t i = 0; i < m_numBsn; ++i, ++nodeId)
        {
          anim->UpdateNodeDescription(nodeId, "BSN-" + std::to_string(i));
          anim->UpdateNodeColor(nodeId, 15, 118, 110);
        }
      for (uint32_t i = 0; i < m_numDrn; ++i, ++nodeId)
        {
          anim->UpdateNodeDescription(nodeId, "DRN-" + std::to_string(i));
          anim->UpdateNodeColor(nodeId, 124, 58, 237);
        }
      for (uint32_t i = 0; i < m_numDdn; ++i, ++nodeId)
        {
          anim->UpdateNodeDescription(nodeId, "DDN-" + std::to_string(i));
          anim->UpdateNodeColor(nodeId, 217, 119, 6);
        }
      for (uint32_t i = 0; i < m_sinks; ++i, ++nodeId)
        {
          anim->UpdateNodeDescription(nodeId, "SINK-" + std::to_string(i));
          anim->UpdateNodeColor(nodeId, 37, 99, 235);
        }
      anim->UpdateNodeDescription(nodeId, "AUV");
      anim->UpdateNodeColor(nodeId, 220, 38, 38);
    }

  ScheduleTraffic();
  Simulator::Stop(Seconds(m_simStop));
  Simulator::Run();
  // A frame cut by the observation horizon still expends elapsed active-window
  // energy. Retain its pending state and settle before final metrics only.
  SettlePendingOpticalEnergy(Simulator::Now().GetSeconds());
  if (m_reviewedMission) ReviewedMissionAdvance(Simulator::Now().GetSeconds());
  if (m_reviewedPhyEnergy) FinalizeReviewedPhyEnergy();
  // Simulator::Stop() is scheduled before the periodic sample at the same
  // timestamp, so that sample is not dispatched.  Emit one explicit terminal
  // row while the simulator state and final clock are still available.  This
  // also charges the last partial interval of normalized idle energy.
  SampleMetrics();
  if (!m_phyEnergyCsv.empty()) ObservePhyEnergy(true);
  if (!m_diagnosticsCsv.empty()) WriteGatewayDiagnostics("terminal");
  Simulator::Destroy();
  PrintSummary();
  ReviewedMissionSummary();
}

} // namespace

int
main(int argc, char* argv[])
{
  LogComponentEnable("AuvBasedEdcUwlsn", LOG_LEVEL_INFO);
  AuvEdcExperiment experiment;
  experiment.Configure(argc, argv);
  if (experiment.RunReviewedOpticalSelfTests()) return 0;
  if (experiment.RunReviewedIngressSelfTests()) return 0;
  if (experiment.RunReviewedSelfTests()) return 0;
  if (experiment.RunClosureTransportSelfTests()) return 0;
  if (experiment.RunHardwareSelfTests()) return 0;
  experiment.Run();
  return 0;
}
