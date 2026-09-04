/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/aqua-sim-ng-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
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
    uint32_t GetSerializedSize() const override { return 41; }
    void Serialize(Buffer::Iterator i) const override
    {
        i.WriteHtonU64(m_id);
        i.WriteHtonU32(m_srcId);
        i.WriteHtonU32(m_dstId);
        i.WriteHtonU32(m_nextHopId);
        i.WriteHtonU32(m_createAtMs);
        i.WriteU8(m_prio);
        i.WriteU8(m_hops);
        i.WriteU8(m_attempt);
        i.WriteHtonU32(m_payloadBytes);
        i.WriteHtonU16(m_fragmentBytes);
        i.WriteHtonU16(m_fragmentIndex);
        i.WriteHtonU16(m_fragmentCount);
        i.WriteHtonU32(m_previousHopId);
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
};
NS_OBJECT_ENSURE_REGISTERED(EdcHeader);

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
    void SetFragmentation(uint32_t payloadBytes, double bitRate)
    {
        m_maxFragmentPayloadBytes = std::max(1u, std::min(payloadBytes, 65535u));
        m_bitRate = std::max(1.0, bitRate);
    }

    // Called by GeneratePacket for BSN nodes.
    void SendPkt(uint64_t pktId, uint8_t prio, uint32_t payloadBytes, uint8_t attempt)
    {
        if (!m_socket) return;
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
            Simulator::Schedule(Seconds(offsetSeconds),
                                &EdcForwardingApp::TransmitFrame, this, pkt, h);
            offsetSeconds +=
                (fragmentBytes + h.GetSerializedSize()) * 8.0 / m_bitRate;
          }
    }

  private:
    void TransmitFrame(Ptr<Packet> pkt, EdcHeader h)
    {
        const double distance =
            CalculateDistance(PositionOf(h.m_nextHopId), PositionOf(m_myId));
        if (m_energyCb)
          m_energyCb(h.m_prio, h.m_fragmentBytes, distance, m_myId, true);
        if (m_hopCb)
          m_hopCb(h.m_id, h.m_srcId, h.m_prio, h.m_fragmentBytes, h.m_attempt,
                  m_myId, h.m_nextHopId, h.m_dstId, distance);
        pkt->AddHeader(h);
        SendToNode(pkt, h.m_nextHopId);
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
        EdcHeader h;
        if (pkt->RemoveHeader(h) == 0) return;
        if (h.m_nextHopId != m_myId) return;
        if (h.m_previousHopId >= m_allPositions.size()) return;
        const double hopDistance =
            CalculateDistance(PositionOf(m_myId), PositionOf(h.m_previousHopId));
        if (m_receptionCb && !m_receptionCb(h.m_fragmentBytes, hopDistance)) return;
        uint64_t seenKey = h.m_id * 1315423911ULL ^
            (static_cast<uint64_t>(h.m_attempt) << 24) ^ h.m_fragmentIndex;
        if (m_seen.count(seenKey)) return;
        m_seen.insert(seenKey);
        if (m_energyCb)
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

    void SendToNode(Ptr<Packet> pkt, uint32_t nodeId)
    {
        if (nodeId == m_myId || nodeId >= NodeList::GetNNodes()) return;
        Ptr<Node> target = NodeList::GetNode(nodeId);
        if (!target || target->GetNDevices() == 0) return;
        Ptr<AquaSimNetDevice> sourceDevice =
            DynamicCast<AquaSimNetDevice>(GetNode()->GetDevice(0));
        sourceDevice->Send(pkt, target->GetDevice(0)->GetAddress(), 0);
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
};
NS_OBJECT_ENSURE_REGISTERED(EdcForwardingApp);

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

class AuvEdcExperiment
{
public:
  void Configure(int argc, char* argv[]);
  void Run();

private:
  void ScaleRoles();
  void BuildNs3Topology();
  void BuildLogicalTopology();
  void ScheduleTraffic();
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
  void CompleteOpticalTransfer(uint64_t packetId);
  NodeRole NodeRoleFromNs3Id(uint32_t nodeId) const;
  void BuildForwardingApps();
  void AuvTick();
  Vector GetAuvPosition(double time) const;
  uint32_t NearestDdn(double x) const;
  uint32_t AssociatedDrn(uint32_t bsnIndex) const;
  uint32_t DdnForDrn(uint32_t drnIndex) const;
  uint32_t PacketSizeBytes(Priority priority) const;
  double DeadlineSeconds(Priority priority) const;
  bool DeadlineExpired(const DataPacket& packet) const;
  void RecordDeadlineExpiry(const DataPacket& packet);
  void AccountGeneratedPriority(Priority priority);
  void AccountDeliveredPriority(Priority priority, double delay = -1.0, bool burst = false);
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
  bool EnqueueContribPacket(uint32_t ddnIndex, const DataPacket& packet);
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
  Ptr<ConstantPositionMobilityModel> m_auvMobility;
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
  cmd.Parse(argc, argv);
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

  m_ddnBufferPolicy = NormalizeName(m_ddnBufferPolicy);
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
      dev->GetPhy()->SetTransRange(std::max(node.range, m_sinkRange));
      positions->Add(node.position);
    }
  for (const auto& node : m_sinkNodes)
    {
      Ptr<AquaSimNetDevice> dev = CreateObject<AquaSimNetDevice>();
      asHelper.Create(m_allNodes.Get(nodeId++), dev);
      dev->GetPhy()->SetTransRange(node.range);
      positions->Add(node.position);
    }

  Ptr<AquaSimNetDevice> auvDev = CreateObject<AquaSimNetDevice>();
  asHelper.Create(m_auvNode, auvDev);
  auvDev->GetPhy()->SetTransRange(std::max(m_ddnCollectionRange, m_sinkRange) + 1.0);
  positions->Add(GetAuvPosition(0.0));

  mobility.SetPositionAllocator(positions);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(m_allNodes);

  m_auvMobility = m_auvNode->GetObject<ConstantPositionMobilityModel>();

  // The application selects one next hop within its geometric routing gate.
  // Configure the AquaSim static router so that this destination is forwarded
  // directly to itself, avoiding AquaSimRoutingDummy's flooding behavior.
  const std::string routeFile = "/tmp/edc-static-routes-" +
      std::to_string(RngSeedManager::GetRun()) + ".txt";
  {
    std::ofstream routes(routeFile, std::ios::trunc);
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
            routes << sourceAddress << ':' << destinationAddress << ':'
                   << destinationAddress << '\n';
          }
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

  // Install real forwarding applications on every sensor node.
  BuildForwardingApps();
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

  // The declared BSN/DRN ranges are lower bounds.  Both architectures apply a
  // deterministic role-count reachability rule so scaling the topology does
  // not disconnect the logical chain.  Because the EOM topology has additional
  // paired DRNs, its effective DRN gate differs from the scheduled baseline.
  // Packet success and energy still use the actual geometric hop distance.
  double bsnStep = (m_numBsn > 1) ? m_pipelineLength / (m_numBsn - 1) : m_pipelineLength;
  double drnStep = (m_numDrn > 1) ? m_pipelineLength / (m_numDrn - 1) : m_pipelineLength;
  // Effective gate: at least 10% greater than the mean spacing implied by the
  // count of each role.
  double effBsn = std::max(m_bsnRange, bsnStep * 1.1);
  double effDrn = std::max(m_drnRange, drnStep * 1.1);
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
          ddnDevice->GetPhy()->SetTransRange(
              std::max(effDdn, m_ddnCollectionRange) + 1.0);
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(ddnNodeId, m_ddns[k].position,
                     m_ddnCollectionRange + 1.0,
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
                    m_ddnCollectionRange + 1.0,
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
          ddnDevice->GetPhy()->SetTransRange(bestDistance + 1.0);
          auto app = CreateObject<EdcForwardingApp>();
          app->Setup(ddnNodeId, m_ddns[k].position, bestDistance + 1.0,
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
      sinkApp->Setup(sinkNodeId, m_sinkNodes[s].position, m_sinkRange + 1.0,
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
          app->Setup(auvNodeId, m_auvMobility->GetPosition(), m_sinkRange + 1.0,
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
  pending.attempts++;
  m_stats.acousticSourceAttempts++;
  if (pending.attempts > 1) m_stats.acousticRetransmissions++;

  const DataPacket& packet = pending.packet;
  TracePacket("upstream_tx", packet, static_cast<int32_t>(pending.ddnIndex), -1,
              pending.attempts, -1.0, "attempt");
  m_bsnFwdApps.at(packet.sourceBsn)->SendPkt(
      packet.id, static_cast<uint8_t>(packet.priority),
      packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + AcousticRecordDurationS(packet.sizeBytes)),
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
  m_ddnSinkApps.at(pending.ddnIndex)->SendPkt(
      packet.id, static_cast<uint8_t>(packet.priority),
      packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout + AcousticRecordDurationS(packet.sizeBytes)),
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
  AccountDeliveredPriority(packet.priority, e2eDelay, packet.burst);
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
  m_ddnAuvApps.at(pending.ddnIndex)->SendPkt(
      pending.packet.id, static_cast<uint8_t>(pending.packet.priority),
      pending.packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout +
              AcousticRecordDurationS(pending.packet.sizeBytes)),
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
  m_auvSinkApps.at(pending.sinkIndex)->SendPkt(
      pending.packet.id, static_cast<uint8_t>(pending.packet.priority),
      pending.packet.sizeBytes, pending.attempts);
  pending.timeout = Simulator::Schedule(
      Seconds(m_acousticAckTimeout +
              AcousticRecordDurationS(pending.packet.sizeBytes)),
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
  AccountDeliveredPriority(packet.priority, e2eDelay, packet.burst);
  TracePacket("auv_sink_ack", packet, -1, static_cast<int32_t>(sinkIndex),
              attempt, distance, "delivered");
}

void
AuvEdcExperiment::StartOpticalTransfer(uint32_t ddnIndex,
                                       const DataPacket& packet,
                                       double distance,
                                       double successProbability)
{
  if (m_pendingOptical.count(packet.id)) return;
  m_opticalActive.at(ddnIndex) = true;
  m_pendingOptical.emplace(packet.id,
      PendingOpticalPacket{packet, ddnIndex, distance, successProbability});
  const double txDuration =
      static_cast<double>(packet.sizeBytes * 8) / m_opticalDataRateBps;
  Simulator::Schedule(Seconds(std::max(txDuration, 1e-9)),
                      &AuvEdcExperiment::CompleteOpticalTransfer,
                      this, packet.id);
}

void
AuvEdcExperiment::TryStartOpticalTransfer(uint32_t ddnIndex)
{
  if (ddnIndex >= m_numDdn || m_opticalActive.at(ddnIndex)) return;
  const double now = Simulator::Now().GetSeconds();
  const Vector auvPos = GetAuvPosition(now);
  const double distance = CalculateDistance(auvPos, m_ddns[ddnIndex].position);
  if (distance > m_opticalRange) return;

  DataPacket packet;
  if (!PopNextDdnPacket(ddnIndex, packet)) return;
  double snrDb = 0.0;
  double ber = 0.0;
  double pointingGain = 1.0;
  const double successProbability = OpticalPacketSuccessProbability(
      ddnIndex, distance, packet.sizeBytes, snrDb, ber, pointingGain);
  m_stats.opticalContactSamples++;
  m_stats.opticalSuccessProbSum += successProbability;
  m_stats.opticalSnrDbSum += snrDb;
  m_stats.opticalAttemptedPackets++;
  TracePacket("optical_tx", packet, static_cast<int32_t>(ddnIndex), -1, 1,
              distance, "attempt");
  StartOpticalTransfer(ddnIndex, packet, distance, successProbability);
}

void
AuvEdcExperiment::CompleteOpticalTransfer(uint64_t packetId)
{
  auto it = m_pendingOptical.find(packetId);
  if (it == m_pendingOptical.end()) return;
  PendingOpticalPacket pending = it->second;
  m_pendingOptical.erase(it);
  m_opticalActive.at(pending.ddnIndex) = false;
  const Vector rxPosition = GetAuvPosition(Simulator::Now().GetSeconds());
  const double rxDistance = CalculateDistance(
      rxPosition, m_ddns[pending.ddnIndex].position);
  const bool contactHeld = rxDistance <= m_opticalRange;
  const bool received = contactHeld &&
      m_opticalRng->GetValue() <= pending.successProbability;
  if (!received)
    {
      m_stats.opticalLostPackets++;
      TracePacket("optical_loss", pending.packet,
                  static_cast<int32_t>(pending.ddnIndex), -1, 1,
                  rxDistance, contactHeld ? "requeued_per" : "requeued_contact_end");
      RequeueFrontDdnPacket(pending.ddnIndex, pending.packet);
      return;
    }

  m_auvBuffer.push_back(pending.packet);
  m_stats.collectedByAuv++;
  m_stats.opticalTransfers++;
  if (m_energyModel == "hardware")
    {
      const double txDuration = pending.packet.sizeBytes * 8.0 /
                                m_opticalDataRateBps;
      ChargePacketEnergy(pending.packet.priority,
                         m_opticalTxPowerW * txDuration,
                         NodeRole::DDN);
      ChargePacketEnergy(pending.packet.priority,
                         m_opticalRxPowerW * txDuration,
                         NodeRole::AUV);
    }
  else
    {
      const double collectionEnergyJ = 0.001 + pending.packet.sizeBytes *
          pending.txDistance * m_opticalEnergyPerByteMeter;
      ChargePacketEnergySplit(pending.packet.priority, collectionEnergyJ,
                              NodeRole::DDN, 0.5, NodeRole::AUV);
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
          m_sinkNodes.push_back({i, NodeRole::SINK, Vector(x, 0.0, 0.0), m_sinkRange});
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
}

void
AuvEdcExperiment::ScheduleTraffic()
{
  Simulator::Schedule(Seconds(0.5), &AuvEdcExperiment::GeneratePacket, this);
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
AuvEdcExperiment::EnqueueContribPacket(uint32_t ddnIndex, const DataPacket& packet)
{
  if (m_ddnBufferCapacity > 0)
    {
      size_t total = m_ddnHighBuffers[ddnIndex].size() +
                     m_ddnMediumBuffers[ddnIndex].size() +
                     m_ddnLowBuffers[ddnIndex].size();
      if (total >= m_ddnBufferCapacity)
        {
          // FIFO ablation: plain drop-tail with no priority preemption, so an
          // arriving packet of any class is dropped once the buffer is full.
          // This isolates the congestion-protection value of the priority
          // buffer (reviewer ablation).
          if (m_ddnBufferPolicy == "fifo")
            {
              m_stats.bufferDropped++;
              if (packet.priority == Priority::HIGH)        m_stats.bufferDroppedHigh++;
              else if (packet.priority == Priority::MEDIUM) m_stats.bufferDroppedMedium++;
              else                                          m_stats.bufferDroppedLow++;
              return false;
            }
          // Evict the tail of a strictly-lower-priority queue if one exists,
          // so a higher-priority alarm is never lost to buffer pressure.
          if (packet.priority == Priority::HIGH)
            {
              if (!m_ddnLowBuffers[ddnIndex].empty())
                { m_ddnLowBuffers[ddnIndex].pop_back(); m_stats.bufferDropped++; m_stats.bufferDroppedLow++; }
              else if (!m_ddnMediumBuffers[ddnIndex].empty())
                { m_ddnMediumBuffers[ddnIndex].pop_back(); m_stats.bufferDropped++; m_stats.bufferDroppedMedium++; }
              else
                { m_stats.bufferDropped++; m_stats.bufferDroppedHigh++; return false; }
            }
          else if (packet.priority == Priority::MEDIUM)
            {
              if (!m_ddnLowBuffers[ddnIndex].empty())
                { m_ddnLowBuffers[ddnIndex].pop_back(); m_stats.bufferDropped++; m_stats.bufferDroppedLow++; }
              else
                { m_stats.bufferDropped++; m_stats.bufferDroppedMedium++; return false; }
            }
          else // LOW
            {
              m_stats.bufferDropped++; m_stats.bufferDroppedLow++; return false;
            }
        }
    }
  switch (packet.priority)
    {
    case Priority::HIGH:   m_ddnHighBuffers[ddnIndex].push_back(packet);   break;
    case Priority::MEDIUM: m_ddnMediumBuffers[ddnIndex].push_back(packet); break;
    case Priority::LOW:    m_ddnLowBuffers[ddnIndex].push_back(packet);    break;
    }
  return true;
}

void
AuvEdcExperiment::AuvTick()
{
  double now = Simulator::Now().GetSeconds();
  Vector auvPos = GetAuvPosition(now);
  m_auvMobility->SetPosition(auvPos);
  const double queueDt = std::max(0.0, std::min(m_tick, m_simStop - now));
  const uint64_t ddnBufferedNow = BufferedAtDdn();
  m_stats.maxDdnBuffered = std::max(m_stats.maxDdnBuffered, ddnBufferedNow);
  m_stats.maxAuvBuffered = std::max(
      m_stats.maxAuvBuffered, static_cast<uint64_t>(m_auvBuffer.size()));
  m_stats.ddnQueuePacketSeconds += ddnBufferedNow * queueDt;
  m_stats.auvQueuePacketSeconds += m_auvBuffer.size() * queueDt;
  m_stats.queueObservationSeconds += queueDt;
  if (m_includeMobilityIdleEnergy)
    {
      double speed = m_auvSpeedKmh / 3.6;
      if (m_completePatrolInSimTime)
        {
          speed = std::max(speed, 2.0 * m_pipelineLength / std::max(m_simStop - m_helloTimeout, 1.0));
        }
      const double auvMobilityEnergyJ =
          (m_protocol == ProtocolMode::PURE_ACOUSTIC)
              ? 0.0
              : (m_auvFixedPowerW + m_auvPropulsionCoeff * std::pow(speed, 3.0)) * m_tick;
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

  for (uint32_t i = 0; i < m_ddns.size(); ++i)
    {
      // The collection medium is an independent experimental factor. Optical
      // service uses the finite UWOC contact; acoustic service uses the same
      // event-driven DDN->AUV path as the scheduled baseline.
      const bool useOptical = (m_auvCollectionMedium == "optical");
      const double effectiveRange = useOptical ? m_opticalRange
                                               : m_ddnCollectionRange;
      if (!InRange(auvPos, m_ddns[i].position, effectiveRange))
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
      constexpr double MEDIUM_TRAFFIC_FRACTION = 0.30;
      constexpr double LOW_TRAFFIC_FRACTION = 0.60;
      const double highReservedLoad = HIGH_TRAFFIC_FRACTION * m_trafficLoad;
      const double mediumOfferedLoad = MEDIUM_TRAFFIC_FRACTION * m_trafficLoad;
      const double mediumFallbackCapLoad =
          m_mediumFallbackRhoMax * m_trafficLoad;
      const double mediumAdmittedLoad =
          std::min(mediumOfferedLoad,
                   std::max(0.0, mediumFallbackCapLoad - highReservedLoad));
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
          if (InRange(auvPos, m_ddns[i].position, collectionRange))
            continue;

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
              directMoved++;
              directBits += static_cast<uint64_t>(pkt.sizeBytes) * 8;
              m_mediumFallbackTokens -= 1.0;
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
  if (nearSinkIndex >= 0 && m_pendingAuvSurface.empty() && !m_auvBuffer.empty())
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
AuvEdcExperiment::AccountDeliveredPriority(Priority priority, double delay, bool burst)
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
      if (delay >= 0.0)
        {
          m_stats.highDelaySum += delay;
          if (delay > m_highDeadline) m_stats.highDeadlineMiss++;
        }
      break;
    case Priority::MEDIUM:
      m_stats.mediumDelivered++;
      if (delay >= 0.0)
        {
          m_stats.mediumDelaySum += delay;
          if (delay > m_mediumDeadline) m_stats.mediumDeadlineMiss++;
        }
      break;
    case Priority::LOW:
      m_stats.lowDelivered++;
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
  if (!UsesPriorityQueues())
    {
      m_ddnBuffers[ddnIndex].push_front(packet);
      return;
    }
  switch (packet.priority)
    {
    case Priority::HIGH: m_ddnHighBuffers[ddnIndex].push_front(packet); break;
    case Priority::MEDIUM: m_ddnMediumBuffers[ddnIndex].push_front(packet); break;
    case Priority::LOW: m_ddnLowBuffers[ddnIndex].push_front(packet); break;
    }
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
  uint64_t bufferedAtDdn = BufferedAtDdn();

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
      m_stats.highDelivered * static_cast<uint64_t>(m_highPacketSize) +
      m_stats.mediumDelivered * static_cast<uint64_t>(m_mediumPacketSize) +
      m_stats.lowDelivered * static_cast<uint64_t>(m_lowPacketSize);
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
      : 1000.0 * m_stats.highEnergyJ / (m_stats.highDelivered * m_highPacketSize);
  double mediumEnergyMjByte = m_stats.mediumDelivered == 0 ? 0.0
      : 1000.0 * m_stats.mediumEnergyJ / (m_stats.mediumDelivered * m_mediumPacketSize);
  double lowEnergyMjByte = m_stats.lowDelivered == 0 ? 0.0
      : 1000.0 * m_stats.lowEnergyJ / (m_stats.lowDelivered * m_lowPacketSize);
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
      m_stats.acousticRetryExhausted + m_pendingAcoustic.size();
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
      m_pendingAuvSurface.size() + m_pendingOptical.size();
  if (m_stats.generated != allAccounted)
    {
      std::cerr << "INVARIANT_ERROR generated=" << m_stats.generated
                << " surfaceDelivered=" << m_stats.surfaceDelivered
                << " upstreamRetryExhausted=" << m_stats.acousticRetryExhausted
                << " deadlineExpired=" << m_stats.deadlineExpired
                << " bufferDropped=" << m_stats.bufferDropped
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
      m_stats.highDelivered * static_cast<uint64_t>(m_highPacketSize) +
      m_stats.mediumDelivered * static_cast<uint64_t>(m_mediumPacketSize) +
      m_stats.lowDelivered * static_cast<uint64_t>(m_lowPacketSize);
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
      : 1000.0 * m_stats.highEnergyJ / (m_stats.highDelivered * m_highPacketSize);
  double mediumEnergyMjByte = m_stats.mediumDelivered == 0 ? 0.0
      : 1000.0 * m_stats.mediumEnergyJ / (m_stats.mediumDelivered * m_mediumPacketSize);
  double lowEnergyMjByte = m_stats.lowDelivered == 0 ? 0.0
      : 1000.0 * m_stats.lowEnergyJ / (m_stats.lowDelivered * m_lowPacketSize);
  double nodeRoleEnergyTotal = m_stats.bsnEnergyJ + m_stats.drnEnergyJ + m_stats.ddnEnergyJ +
                               m_stats.auvEnergyJ + m_stats.sinkEnergyJ;
  const double submergedEnergyJ = m_stats.bsnEnergyJ + m_stats.drnEnergyJ +
                                  m_stats.ddnEnergyJ;
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
      << m_pendingOptical.size() << ','
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
      << auvPos.z << '\n';

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
      << "auvBuffered,upstreamPdr,avgDelayAcoustic,pdr,avgDelay,throughputBps,auvX,auvY,auvZ\n";
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
  if (m_packetTraceCsv.empty()) return;
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
  // Simulator::Stop() is scheduled before the periodic sample at the same
  // timestamp, so that sample is not dispatched.  Emit one explicit terminal
  // row while the simulator state and final clock are still available.  This
  // also charges the last partial interval of normalized idle energy.
  SampleMetrics();
  Simulator::Destroy();
  PrintSummary();
}

} // namespace

int
main(int argc, char* argv[])
{
  LogComponentEnable("AuvBasedEdcUwlsn", LOG_LEVEL_INFO);
  AuvEdcExperiment experiment;
  experiment.Configure(argc, argv);
  experiment.Run();
  return 0;
}
