#include "ns3/aqua-sim-ng-module.h"
#include "closure_phy.h"
#include "ns3/mobility-module.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <execinfo.h>
#include <csignal>
#include <unistd.h>

namespace ns3 {
class ReviewedProbePhy : public ReviewedAquaSimPhy {
 public:
  static TypeId GetTypeId() {
    static TypeId id = TypeId("ns3::ReviewedProbePhy")
      .SetParent<ReviewedAquaSimPhy>().AddConstructor<ReviewedProbePhy>();
    return id;
  }
  void ReceiveFor(double duration, bool error=false) { UpdateRxEnergy(Seconds(duration), error); }
  void TransmitFor(double duration) { UpdateTxEnergy(Seconds(duration)); }
  Ptr<Packet> InspectArrival(Ptr<Packet> p) { return PrevalidateIncomingPkt(p); }
};
NS_OBJECT_ENSURE_REGISTERED(ReviewedProbePhy);
}

int main() {
  std::signal(SIGSEGV, [](int) { void* frames[32]; int n=backtrace(frames,32); backtrace_symbols_fd(frames,n,2); _exit(139); });
  using namespace ns3;
  auto node = CreateObject<Node>();
  node->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
  auto device = CreateObject<AquaSimNetDevice>();
  auto channel = AquaSimChannelHelper::Default();
  auto helper = AquaSimHelper::Default();
  helper.SetChannel(channel.Create());
  helper.SetPhy("ns3::ReviewedProbePhy", "Frequency", DoubleValue(27.0));
  helper.SetMac("ns3::AquaSimBroadcastMac");
  helper.SetRouting("ns3::AquaSimStaticRouting");
  helper.Create(node, device);
  std::cerr << "PHY_TEST device_created\n";
  auto phy = DynamicCast<ReviewedProbePhy>(device->GetPhy());
  auto battery = std::make_shared<closure_energy::Battery>(20.0);
  auto ledger = std::make_shared<closure_energy::PowerLedger>(battery,
    closure_energy::Powers{4,2,1,0,1}, 10);
  phy->SetLedger(ledger);
  std::cerr << "PHY_TEST ledger_bound\n";
  unsigned checks=0;
  auto check=[&](bool okay) { ++checks; if(!okay) throw std::runtime_error("PHY component check "+std::to_string(checks)); };
  auto interruptedNode=CreateObject<Node>();
  interruptedNode->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
  auto interruptedDevice=CreateObject<AquaSimNetDevice>();
  helper.Create(interruptedNode,interruptedDevice);
  auto interrupted=DynamicCast<ReviewedProbePhy>(interruptedDevice->GetPhy());
  auto secondBudget=std::make_shared<closure_energy::Battery>(100);
  auto secondLedger=std::make_shared<closure_energy::PowerLedger>(secondBudget,closure_energy::Powers{4,2,1,0,0},10);
  interrupted->SetLedger(secondLedger);
  interrupted->TransmitFor(2);
  ReviewedSupplyTag interruptedTag;
  interruptedTag.node=interruptedNode->GetId(); interruptedTag.endNs=2000000000;
  interrupted->PowerOff();
  check(secondBudget->Remaining()==100);
  check(!ReviewedAquaSimPhy::EmissionSupplyValid(interruptedTag));
  interrupted->PowerOn();
  interrupted->TransmitFor(2);
  ReviewedSupplyTag newTag=interruptedTag; newTag.outageOrdinal=1;
  check(ReviewedAquaSimPhy::EmissionSupplyValid(newTag));
  auto tagged=Create<Packet>(); tagged->AddPacketTag(newTag);
  ReviewedSupplyTag roundtrip; check(tagged->PeekPacketTag(roundtrip) && roundtrip.outageOrdinal==1 && roundtrip.endNs==newTag.endNs);
  auto receivingNode=CreateObject<Node>();
  receivingNode->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
  auto receivingDevice=CreateObject<AquaSimNetDevice>(); helper.Create(receivingNode,receivingDevice);
  auto receiving=DynamicCast<ReviewedProbePhy>(receivingDevice->GetPhy());
  auto receivingLedger=std::make_shared<closure_energy::PowerLedger>(std::make_shared<closure_energy::Battery>(100),closure_energy::Powers{4,2,1,0,0},10);
  receiving->SetLedger(receivingLedger);
  auto incoming=Create<Packet>(64);
  MacHeader mac; mac.SetDemuxPType(MacHeader::UWPTYPE_OTHER); incoming->AddHeader(mac);
  AquaSimHeader arrival; arrival.SetSize(64); arrival.SetTxTime(Seconds(3)); arrival.SetErrorFlag(true);
  incoming->AddHeader(arrival);
  AquaSimPacketStamp stamp; stamp.SetFreq(27); stamp.SetPr(1); incoming->AddHeader(stamp);
  newTag.endNs=200000000; incoming->AddPacketTag(newTag);
  auto prevalidated=receiving->InspectArrival(incoming);
  check(prevalidated!=nullptr);
  check(receivingLedger->RxActive(0.199) && !receivingLedger->RxActive(0.201));
  AquaSimHeader restored; prevalidated->PeekHeader(restored);
  check(std::abs(restored.GetTxTime().GetSeconds()-0.2)<1e-12);
  phy->ReceiveFor(2.0);
  std::cerr << "PHY_TEST rx_started\n";
  Simulator::Schedule(Seconds(0.5), [&] { phy->ReceiveFor(2.0, true); });
  Simulator::Schedule(Seconds(3.0), [&] {
    phy->Sync();
    check(std::abs(ledger->Accounting().rx_s-2.5)<1e-9);
    check(std::abs(ledger->Accounting().rx_j-5.0)<1e-9);
    check(std::abs(ledger->Accounting().idle_j-0.5)<1e-9);
    check(phy->ReceivedWaveforms()==2 && phy->FailedReceiveFlags()==1);
    check(std::abs(phy->EM()->GetEnergy()-11.5)<1e-9);
    phy->TransmitFor(3.0);
  });
  Simulator::Schedule(Seconds(6.0), [&] {
    phy->Sync();
    check(battery->Remaining()==0);
    check(std::abs(ledger->DepletedAt()-5.3)<1e-8);
    check(std::abs(ledger->Accounting().TotalJ()-20.0)<1e-8);
    check(!phy->IsPoweredOn());
    check(phy->EM()->GetEnergy()==0);
  });
  Simulator::Stop(Seconds(6.1));
  Simulator::Run();
  std::cerr << "PHY_TEST events_complete checks=" << checks << '\n';
  Simulator::Destroy();
  std::cerr << "PHY_TEST disposed\n";
  std::cout << "REVIEWED_PHY_COMPONENT_PASS checks=" << checks
            << " network_runs=0 actual_frame_qa_pending=true\n";
}
