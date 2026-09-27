// Included inside the experiment namespace. This is a declared link-service
// abstraction: real acoustic control, timed optical DATA, finite local credit.
uint64_t AuvEdcExperiment::ReviewedOpticalReservedMemoryBytes() const
{
  if(!m_reviewedOpticalControl || m_protocol==ProtocolMode::PURE_ACOUSTIC) return 0;
  return uint64_t(m_reviewedOpticalReceiptEntries)*64 + uint64_t(m_numDdn)*256;
}
bool AuvEdcExperiment::ReviewedOpticalReceiverBusy() const
{ return m_reviewedOpticalControl && m_reviewedOpticalReceiver.has_value(); }
uint64_t AuvEdcExperiment::ReviewedOpticalUnreceivedPending() const
{
  uint64_t n=0;
  for(const auto& pair:m_pendingOptical) if(!pair.second.reviewedReceived) ++n;
  return n;
}
double AuvEdcExperiment::ReviewedOpticalServicePredictionSeconds(const std::vector<uint32_t>& payloads) const
{
  NS_ABORT_MSG_IF(!m_reviewedOpticalControl || m_reviewedOpticalControlPredictionS<=0 || m_reviewedOpticalStartupS<=0,
                  "Explicit optical control service prediction required");
  double seconds=m_reviewedOpticalControlPredictionS+m_reviewedOpticalStartupS;
  for(uint32_t bytes:payloads)
    seconds+=(double(bytes)+closure_optical::Descriptor::wireBytes)*8.0/m_opticalDataRateBps+
             m_reviewedOpticalTurnaroundS+m_reviewedOpticalControlPredictionS;
  return seconds;
}
void AuvEdcExperiment::ReviewedOpticalInitialize()
{
  if(!m_reviewedOpticalControl) return;
  const auto pos=[](double v){return std::isfinite(v)&&v>0;};
  NS_ABORT_MSG_IF(!m_reviewedMission||!m_reviewedPhyEnergy||!m_reviewedTransport||
    m_reviewedControlMaxPayloadBytes<28 || !pos(m_reviewedOpticalStartupS)||
    !pos(m_reviewedOpticalListenS)||m_reviewedOpticalListenS<=m_reviewedOpticalStartupS||
    !pos(m_reviewedOpticalPollS)||!pos(m_reviewedOpticalGatewayStartupW)||!pos(m_reviewedOpticalVehicleStartupW)||
    !pos(m_reviewedOpticalEfficiency)||m_reviewedOpticalEfficiency>1||
    !std::isfinite(m_reviewedOpticalFrameSuccess)||m_reviewedOpticalFrameSuccess<0||m_reviewedOpticalFrameSuccess>1||
    !pos(m_reviewedOpticalAckWaitS)||!pos(m_reviewedOpticalTurnaroundS)||!pos(m_reviewedOpticalControlPredictionS)||
    m_reviewedOpticalMaxAttempts==0||m_reviewedOpticalMaxAttempts>255||m_reviewedOpticalReceiptEntries==0||
    m_reviewedOpticalCreditBytes<=m_ddnRecordOverheadBytes,
    "REVIEWED_OPTICAL_CONTRACT_INCOMPLETE: explicit finite startup/credit/power/control parameters required");
  SetReviewedControlHandler(2,[this](uint32_t s,uint32_t r,uint64_t n,const std::vector<uint8_t>& b){ReviewedOpticalReceiveWake(s,r,n,b);});
  SetReviewedControlHandler(3,[this](uint32_t s,uint32_t r,uint64_t n,const std::vector<uint8_t>& b){ReviewedOpticalReceiveAck(s,r,n,b);});
}
void AuvEdcExperiment::ReviewedOpticalWake(uint32_t gateway)
{
  if(gateway>=m_numDdn || !m_reviewedMissionModel || !ReviewedMissionCommunicationsAvailable() || m_reviewedOpticalReceiver) return;
  const double now=Simulator::Now().GetSeconds();
  if(now>=m_simStop || now<m_reviewedOpticalNextPoll[gateway] ||
      CalculateDistance(GetAuvPosition(now),m_ddns[gateway].position)>m_opticalRange) return;
  auto& carry=m_reviewedMissionModel->carry;
  const uint64_t credit=std::min(m_reviewedOpticalCreditBytes,carry.capacityBytes()-carry.usedBytes());
  if(credit<=m_ddnRecordOverheadBytes) return;
  for(auto it=m_reviewedOpticalReceipts.begin();it!=m_reviewedOpticalReceipts.end();)
    if(it->second.second<=now) it=m_reviewedOpticalReceipts.erase(it); else ++it;
  const uint64_t nonce=m_reviewedOpticalNextNonce++;
  NS_ABORT_MSG_IF(nonce==0,"Optical session identity overflow");
  const double ready=now+m_reviewedOpticalStartupS, until=std::min(now+m_reviewedOpticalListenS,m_simStop);
  if(ready>=until) return;
  const std::string key="optical-credit:"+std::to_string(nonce);
  NS_ABORT_MSG_IF(!carry.reserve(key,credit),"Local optical receive-credit allocation failed");
  const uint32_t auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
  ReviewedOpticalReceiverSession session;
  session.nonce=nonce;session.creditBytes=credit;session.gateway=gateway;session.readyAt=ready;session.until=until;
  session.startupLoad=m_reviewedPhys.at(auv)->AddExternalLoad(m_reviewedOpticalStartupS,
      m_reviewedOpticalVehicleStartupW/m_reviewedOpticalEfficiency);
  m_reviewedOpticalReceiver=session;
  m_reviewedOpticalNextPoll[gateway]=now+m_reviewedOpticalPollS;
  Simulator::Schedule(Seconds(m_reviewedOpticalStartupS),&AuvEdcExperiment::ReviewedOpticalReady,this,nonce);
  Simulator::Schedule(Seconds(until-now),&AuvEdcExperiment::ReviewedOpticalClose,this,nonce);
  closure_optical::Wake wake{uint64_t(Seconds(ready).GetNanoSeconds()),uint64_t(Seconds(until).GetNanoSeconds()),credit};
  const auto token=SendReviewedControl(auv,m_numBsn+m_numDrn+gateway,2,nonce,wake.encode());
  if(!token) {ReviewedOpticalClose(nonce);return;}
  ++m_reviewedOpticalWakeSent;
}
void AuvEdcExperiment::ReviewedOpticalReady(uint64_t nonce)
{
  if(!m_reviewedOpticalReceiver || m_reviewedOpticalReceiver->nonce!=nonce) return;
  auto& r=*m_reviewedOpticalReceiver;
  const double now=Simulator::Now().GetSeconds();
  if(now>=r.until || !ReviewedMissionCommunicationsAvailable()) {ReviewedOpticalClose(nonce);return;}
  const uint32_t auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
  r.receiveLoad=m_reviewedPhys.at(auv)->AddExternalLoad(r.until-now,m_opticalRxPowerW/m_reviewedOpticalEfficiency);
}
void AuvEdcExperiment::ReviewedOpticalClose(uint64_t nonce)
{
  if(!m_reviewedOpticalReceiver || m_reviewedOpticalReceiver->nonce!=nonce) return;
  const auto r=*m_reviewedOpticalReceiver;
  const uint32_t auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
  m_reviewedPhys.at(auv)->StopExternalLoad(r.startupLoad);
  if(r.receiveLoad) m_reviewedPhys.at(auv)->StopExternalLoad(r.receiveLoad);
  m_reviewedMissionModel->carry.release("optical-credit:"+std::to_string(nonce));
  m_reviewedOpticalReceiver.reset();
}
void AuvEdcExperiment::ReviewedOpticalReceiveWake(uint32_t source,uint32_t destination,uint64_t nonce,const std::vector<uint8_t>& body)
{
  const uint32_t auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
  const uint32_t gatewayBase=m_numBsn+m_numDrn;
  if(source!=auv || destination<gatewayBase || destination>=gatewayBase+m_numDdn || !nonce) return;
  closure_optical::Wake wake;
  try{wake=closure_optical::Wake::decode(body);}catch(const std::exception&){return;}
  const double now=Simulator::Now().GetSeconds(), until=double(wake.listenUntilNs)*1e-9;
  const uint32_t gateway=destination-gatewayBase;
  if(until<=now+m_reviewedOpticalStartupS || until>now+m_reviewedOpticalListenS+1e-8 ||
      wake.creditBytes>m_reviewedOpticalCreditBytes || wake.creditBytes<=m_ddnRecordOverheadBytes) return;
  auto old=m_reviewedOpticalSenders.find(gateway);
  if(old!=m_reviewedOpticalSenders.end() && (nonce<=old->second.nonce || now<old->second.until)) return;
  ReviewedOpticalSenderSession s;
  s.nonce=nonce;s.creditBytes=wake.creditBytes;s.readyAt=std::max(now+m_reviewedOpticalStartupS,double(wake.readyAtNs)*1e-9);s.until=until;
  // A previously unconfirmed sender copy stays in its finite gateway outbox.
  for(const auto& p:m_pendingOptical) if(p.second.ddnIndex==gateway && p.second.reviewedWire) {s.activePacket=p.first;break;}
  m_reviewedOpticalSenders[gateway]=s;
  ++m_reviewedOpticalWakeReceived;
  m_reviewedPhys.at(destination)->AddExternalLoad(s.readyAt-now,m_reviewedOpticalGatewayStartupW/m_reviewedOpticalEfficiency);
  Simulator::Schedule(Seconds(s.readyAt-now),&AuvEdcExperiment::ReviewedOpticalSend,this,gateway,nonce);
}
void AuvEdcExperiment::ReviewedOpticalSend(uint32_t gateway,uint64_t nonce)
{
  auto sit=m_reviewedOpticalSenders.find(gateway);
  if(sit==m_reviewedOpticalSenders.end() || sit->second.nonce!=nonce) return;
  auto& s=sit->second;const double now=Simulator::Now().GetSeconds();
  if(now<s.readyAt-1e-8 || now>=s.until) return;
  if(!s.activePacket)
    {
      DataPacket packet;
      if(!PopNextDdnPacket(gateway,packet))
        {s.until=now;SendReviewedControl(m_numBsn+m_numDrn+gateway,m_numBsn+m_numDrn+m_numDdn+m_sinks,3,nonce,{0});return;}
      if(RecordStorageBytes(packet)>s.creditBytes)
        {RequeueFrontDdnPacket(gateway,packet);s.until=now;
         SendReviewedControl(m_numBsn+m_numDrn+gateway,m_numBsn+m_numDrn+m_numDdn+m_sinks,3,nonce,{0});return;}
      PendingOpticalPacket pending{packet,gateway,0,m_reviewedOpticalFrameSuccess};
      pending.reviewedWire=true;
      NS_ABORT_MSG_IF(m_pendingOptical.count(packet.id),"Duplicate gateway optical sender custody");
      m_pendingOptical.emplace(packet.id,pending);s.activePacket=packet.id;
    }
  auto pit=m_pendingOptical.find(s.activePacket);
  if(pit==m_pendingOptical.end()) {s.activePacket=0;return;}
  auto& pending=pit->second;
  if(DeadlineExpired(pending.packet) || pending.reviewedAttempts>=m_reviewedOpticalMaxAttempts)
    {ReviewedOpticalAbandon(pending.packet.id,"bounded_optical_lifetime_end");s.activePacket=0;
     Simulator::Schedule(Seconds(m_reviewedOpticalTurnaroundS),&AuvEdcExperiment::ReviewedOpticalSend,this,gateway,nonce);return;}
  if(RecordStorageBytes(pending.packet)>s.creditBytes) return;
  closure_optical::Descriptor d;
  d.nonce=nonce;d.packetId=pending.packet.id;d.gateway=gateway;d.source=pending.packet.sourceBsn;
  d.createdAtNs=Seconds(pending.packet.createdAt).GetNanoSeconds();d.priority=static_cast<uint8_t>(pending.packet.priority);
  d.payloadBytes=pending.packet.sizeBytes;d.windowId=pending.packet.windowNumericId;d.windowBytes=pending.packet.windowPayloadBytes;
  d.chunkIndex=pending.packet.windowChunkIndex;d.chunkCount=pending.packet.windowChunkCount;
  auto descriptor=d.encode();
  const double duration=(double(d.payloadBytes)+descriptor.size())*8.0/m_opticalDataRateBps;
  const double distance=CalculateDistance(GetAuvPosition(now),m_ddns[gateway].position);
  const double propagation=distance/(299792458.0/1.34);
  if(distance>m_opticalRange || now+duration+propagation>s.until) return;
  const uint32_t gw=m_numBsn+m_numDrn+gateway;
  m_reviewedPhys.at(gw)->Sync();
  if(m_reviewedLedgers.at(gw)->Budget()->Remaining()<=0) return;
  pending.startedAt=now;pending.durationSeconds=duration;pending.txDistance=distance;
  pending.reviewedNonce=nonce;++pending.reviewedAttempts;
  m_opticalActive.at(gateway)=true;
  ++m_stats.opticalAttemptedPackets;++m_stats.opticalContactSamples;
  m_stats.opticalSuccessProbSum+=m_reviewedOpticalFrameSuccess;
  m_reviewedOpticalDataWireBytes+=uint64_t(d.payloadBytes)+descriptor.size();
  m_reviewedPhys.at(gw)->AddExternalLoad(duration,m_opticalTxPowerW/m_reviewedOpticalEfficiency);
  TracePacket("optical_tx",pending.packet,gateway,-1,pending.reviewedAttempts,distance,"reviewed_descriptor_data_acoustic_confirmation");
  Simulator::Schedule(Seconds(duration+propagation),&AuvEdcExperiment::ReviewedOpticalArrival,this,gateway,nonce,
      d.packetId,pending.reviewedAttempts,descriptor);
  s.timeout.Cancel();
  s.timeout=Simulator::Schedule(Seconds(duration+propagation+m_reviewedOpticalAckWaitS),
      &AuvEdcExperiment::ReviewedOpticalTimeout,this,gateway,nonce,d.packetId,pending.reviewedAttempts);
}
void AuvEdcExperiment::ReviewedOpticalArrival(uint32_t gateway,uint64_t nonce,uint64_t packetId,uint32_t attempt,std::vector<uint8_t> bytes)
{
  auto pit=m_pendingOptical.find(packetId);
  if(pit==m_pendingOptical.end() || pit->second.reviewedNonce!=nonce || pit->second.reviewedAttempts!=attempt) return;
  auto& p=pit->second; const double now=Simulator::Now().GetSeconds();
  const uint32_t gw=m_numBsn+m_numDrn+gateway, auv=m_numBsn+m_numDrn+m_numDdn+m_sinks;
  m_reviewedPhys.at(gw)->Sync();m_reviewedPhys.at(auv)->Sync();
  closure_optical::Descriptor d;
  try{d=closure_optical::Descriptor::decode(bytes);}catch(const std::exception&){return;}
  const double distance=CalculateDistance(GetAuvPosition(now),m_ddns[gateway].position);
  const bool listened=m_reviewedOpticalReceiver && m_reviewedOpticalReceiver->nonce==nonce &&
      m_reviewedOpticalReceiver->gateway==gateway && m_reviewedOpticalReceiver->readyAt<=p.startedAt+1e-8 &&
      m_reviewedOpticalReceiver->until+1e-8>=now && ReviewedMissionCommunicationsAvailable();
  const bool powered=m_reviewedLedgers.at(gw)->DepletedAt()+1e-9>=p.startedAt+p.durationSeconds &&
      m_reviewedLedgers.at(auv)->DepletedAt()+1e-9>=now;
  const bool contact=closure_optical::continuousContact(p.txDistance,distance,now-p.startedAt,m_opticalRange,closure_mission::fixedThreeKnotsMps);
  if(!listened||!powered||!contact||d.nonce!=nonce||d.packetId!=packetId||d.gateway!=gateway||
      m_opticalRng->GetValue()>m_reviewedOpticalFrameSuccess)
    {++m_stats.opticalLostPackets;TracePacket("optical_loss",p.packet,gateway,-1,attempt,distance,"reviewed_data_not_received");return;}
  auto& r=*m_reviewedOpticalReceiver;
  auto seen=m_reviewedOpticalReceipts.find(packetId);
  if(seen==m_reviewedOpticalReceipts.end())
    {
      const uint64_t resident=uint64_t(d.payloadBytes)+m_ddnRecordOverheadBytes;
      if(resident>r.creditBytes || m_reviewedOpticalReceipts.size()>=m_reviewedOpticalReceiptEntries)
        {++m_reviewedOpticalCreditRejected;return;}
      DataPacket received{d.packetId,d.source,double(d.createdAtNs)*1e-9,0,static_cast<Priority>(d.priority),d.payloadBytes};
      received.windowNumericId=d.windowId;received.windowPayloadBytes=d.windowBytes;
      received.windowChunkIndex=d.chunkIndex;received.windowChunkCount=d.chunkCount;
      // Convert an already-paid, receiver-local credit atomically to data;
      // no remote free-space query is used by the gateway sender.
      auto& carry=m_reviewedMissionModel->carry;
      const auto key="optical-credit:"+std::to_string(nonce);
      carry.release(key);r.creditBytes-=resident;
      if(r.creditBytes) NS_ABORT_MSG_IF(!carry.reserve(key,r.creditBytes),"Remaining optical credit lost");
      NS_ABORT_MSG_IF(!ReviewedMissionReserve(received),"Advertised optical credit failed actual local admission");
      m_auvBuffer.push_back(received);ReviewedMissionCommit(received.id,gateway);
      m_reviewedOpticalReceipts[packetId]={d.payloadBytes,received.createdAt+DeadlineSeconds(received.priority)+m_reviewedControlLifetimeS};
      ++m_stats.collectedByAuv;++m_stats.opticalTransfers;
      TracePacket("optical_rx",received,gateway,-1,attempt,distance,"receiver_committed_sender_unconfirmed");
    }
  else
    {
      if(seen->second.first!=d.payloadBytes) return;
      ++m_reviewedOpticalDuplicates;
      TracePacket("optical_duplicate_rx",p.packet,gateway,-1,attempt,distance,"retained_receipt_reack");
    }
  p.reviewedReceived=true; // Global accounting annotation; never a sender decision input.
  closure_optical::Ack ack{nonce,packetId,gateway};
  SendReviewedControl(auv,gw,3,nonce,ack.encode());
}
void AuvEdcExperiment::ReviewedOpticalReceiveAck(uint32_t source,uint32_t destination,uint64_t nonce,const std::vector<uint8_t>& body)
{
  const uint32_t auv=m_numBsn+m_numDrn+m_numDdn+m_sinks, base=m_numBsn+m_numDrn;
  if(body==std::vector<uint8_t>{0})
    {
      if(destination==auv && m_reviewedOpticalReceiver && m_reviewedOpticalReceiver->nonce==nonce &&
          source==base+m_reviewedOpticalReceiver->gateway) ReviewedOpticalClose(nonce);
      return;
    }
  if(source!=auv || destination<base || destination>=base+m_numDdn) return;
  closure_optical::Ack ack;
  try{ack=closure_optical::Ack::decode(body);}catch(const std::exception&){return;}
  const uint32_t gateway=destination-base;
  auto sit=m_reviewedOpticalSenders.find(gateway);auto pit=m_pendingOptical.find(ack.packetId);
  if(ack.nonce!=nonce||ack.gateway!=gateway||sit==m_reviewedOpticalSenders.end()||
      sit->second.nonce!=nonce||sit->second.activePacket!=ack.packetId||pit==m_pendingOptical.end()) return;
  auto& s=sit->second;
  const auto p=pit->second;
  if(p.reviewedNonce!=nonce || RecordStorageBytes(p.packet)>s.creditBytes) return;
  s.timeout.Cancel();s.creditBytes-=RecordStorageBytes(p.packet);s.activePacket=0;
  m_pendingOptical.erase(pit);m_opticalActive[gateway]=false;
  ++m_reviewedOpticalAckReceived;
  ReviewedIngressRelease(gateway,p.packet.id,closure_ingress::Removal::Forwarded);
  TracePacket("optical_sender_confirmed",p.packet,gateway,-1,p.reviewedAttempts,-1,"actual_acoustic_control_ack");
  Simulator::Schedule(Seconds(m_reviewedOpticalTurnaroundS),&AuvEdcExperiment::ReviewedOpticalSend,this,gateway,nonce);
}
void AuvEdcExperiment::ReviewedOpticalTimeout(uint32_t gateway,uint64_t nonce,uint64_t packetId,uint32_t attempt)
{
  auto sit=m_reviewedOpticalSenders.find(gateway);auto pit=m_pendingOptical.find(packetId);
  if(sit==m_reviewedOpticalSenders.end()||sit->second.nonce!=nonce||pit==m_pendingOptical.end()||
      pit->second.reviewedAttempts!=attempt||pit->second.reviewedNonce!=nonce) return;
  m_opticalActive[gateway]=false;
  TracePacket("optical_ack_timeout",pit->second.packet,gateway,-1,attempt,-1,"sender_retains_copy");
  if(DeadlineExpired(pit->second.packet)||attempt>=m_reviewedOpticalMaxAttempts)
    {ReviewedOpticalAbandon(packetId,"optical_ack_attempt_limit");sit->second.activePacket=0;}
  if(Simulator::Now().GetSeconds()+m_reviewedOpticalTurnaroundS<sit->second.until)
    Simulator::Schedule(Seconds(m_reviewedOpticalTurnaroundS),&AuvEdcExperiment::ReviewedOpticalSend,this,gateway,nonce);
}
void AuvEdcExperiment::ReviewedOpticalAbandon(uint64_t packetId,const std::string& reason)
{
  auto it=m_pendingOptical.find(packetId);if(it==m_pendingOptical.end()) return;
  const auto p=it->second;m_pendingOptical.erase(it);m_opticalActive[p.ddnIndex]=false;
  if(p.reviewedReceived)
    {++m_reviewedOpticalCopyAbandoned;ReviewedIngressRelease(p.ddnIndex,packetId,closure_ingress::Removal::Dropped);}
  else RecordBufferDrop(p.ddnIndex,p.packet,reason);
  TracePacket("optical_sender_abandoned",p.packet,p.ddnIndex,-1,p.reviewedAttempts,-1,
      p.reviewedReceived ? "received_copy_preserved_sender_confirmation_missing" : "unreceived_record_end");
}
void AuvEdcExperiment::ReviewedOpticalSummary() const
{
  if(!m_reviewedOpticalControl) return;
  std::cout<<"REVIEWED_OPTICAL {\"enabled\":true,\"service_abstraction_not_modem_calibration\":true,"
    <<"\"actual_acoustic_wake_and_confirmation\":true,\"new_optical_hardware_added\":false,"
    <<"\"descriptor_bytes\":69,\"wake_payload_bytes\":24,\"ack_payload_bytes\":28,"
    <<"\"wakes_sent\":"<<m_reviewedOpticalWakeSent<<",\"wakes_received\":"<<m_reviewedOpticalWakeReceived
    <<",\"sender_confirmations\":"<<m_reviewedOpticalAckReceived<<",\"duplicate_receipts\":"<<m_reviewedOpticalDuplicates
    <<",\"credit_or_cache_rejections\":"<<m_reviewedOpticalCreditRejected<<",\"unconfirmed_received_copies_abandoned\":"<<m_reviewedOpticalCopyAbandoned
    <<",\"data_wire_bytes\":"<<m_reviewedOpticalDataWireBytes<<",\"reserved_metadata_bytes\":"<<ReviewedOpticalReservedMemoryBytes()
    <<",\"receipt_entries\":"<<m_reviewedOpticalReceipts.size()<<",\"receiver_busy\":"<<(ReviewedOpticalReceiverBusy()?"true":"false")
    <<",\"unreceived_pending\":"<<ReviewedOpticalUnreceivedPending()<<",\"full_validation\":false}\n";
}
bool AuvEdcExperiment::RunReviewedOpticalSelfTests()
{
  if(!m_reviewedOpticalSelfTest) return false;
  uint32_t checks=0;
  auto check=[&](bool ok,const char* label){NS_ABORT_MSG_IF(!ok,"Reviewed optical self-test: "<<label);++checks;};
  AuvEdcExperiment e;e.m_reviewedOpticalControl=true;e.m_numBsn=1;e.m_numDrn=0;e.m_numDdn=1;e.m_sinks=1;
  e.m_opticalActive.assign(1,false);e.m_reviewedOpticalMaxAttempts=1;e.m_reviewedOpticalTurnaroundS=.01;
  e.m_reviewedOpticalReceiptEntries=8;e.m_reviewedOpticalStartupS=1;e.m_reviewedOpticalControlPredictionS=.5;
  e.m_opticalDataRateBps=2500000;e.m_protocol=ProtocolMode::CONTRIBUTION;
  check(e.ReviewedOpticalReservedMemoryBytes()==768,"finite metadata allocation is explicit");
  check(!e.ReviewedOpticalReceiverBusy(),"receiver initially off");
  e.m_reviewedOpticalReceiver=ReviewedOpticalReceiverSession{};
  check(e.ReviewedOpticalReceiverBusy(),"active receiver credit blocks premature recovery");e.m_reviewedOpticalReceiver.reset();
  DataPacket packet{1,0,0,0,Priority::LOW,64};
  PendingOpticalPacket pending{packet,0,0,1};pending.reviewedWire=true;pending.reviewedAttempts=1;pending.reviewedNonce=9;
  e.m_pendingOptical.emplace(1,pending);
  check(e.ReviewedOpticalUnreceivedPending()==1,"unreceived DATA is unique pending custody");
  e.m_pendingOptical.at(1).reviewedReceived=true;
  check(e.ReviewedOpticalUnreceivedPending()==0,"retained sender copy is not a second record");
  e.ReviewedOpticalAbandon(1,"test");
  check(e.m_stats.bufferDropped==0&&e.m_reviewedOpticalCopyAbandoned==1,"unconfirmed sender-copy abandonment preserves receiver DATA");
  e.m_pendingOptical.emplace(1,pending);e.ReviewedOpticalAbandon(1,"test");
  check(e.m_stats.bufferDropped==1&&e.m_stats.bufferDroppedLow==1,"unreceived record abandonment is explicit loss");
  e.m_pendingOptical.emplace(1,pending);
  ReviewedOpticalSenderSession sender;sender.nonce=9;sender.creditBytes=1000;sender.activePacket=1;sender.until=0;
  e.m_reviewedOpticalSenders[0]=sender;
  e.ReviewedOpticalTimeout(0,8,1,1);
  check(e.m_pendingOptical.size()==1,"stale timeout has no custody effect");
  e.ReviewedOpticalTimeout(0,9,1,1);
  check(e.m_pendingOptical.empty()&&e.m_reviewedOpticalSenders.at(0).activePacket==0,"attempt-bound timeout settles local sender only");
  pending.reviewedReceived=true;e.m_pendingOptical.emplace(1,pending);e.m_reviewedOpticalSenders[0]=sender;
  closure_optical::Ack wrong{8,1,0};e.ReviewedOpticalReceiveAck(3,1,9,wrong.encode());
  check(e.m_reviewedOpticalAckReceived==0&&e.m_pendingOptical.size()==1,"wrong nonce cannot confirm");
  closure_optical::Ack ack{9,1,0};e.ReviewedOpticalReceiveAck(3,1,9,ack.encode());
  check(e.m_reviewedOpticalAckReceived==1&&e.m_pendingOptical.empty(),"actual decoded ACK handler releases sender");
  check(e.m_reviewedOpticalSenders.at(0).creditBytes==1000-e.RecordStorageBytes(packet),"sender decrements only advertised byte credit");
  e.ReviewedOpticalReceiveAck(3,1,9,ack.encode());
  check(e.m_reviewedOpticalAckReceived==1,"duplicate ACK does not release twice");
  check(std::abs(e.ReviewedOpticalServicePredictionSeconds({})-1.5)<1e-12,"prediction includes fixed wake and startup");
  check(std::abs(e.ReviewedOpticalServicePredictionSeconds({64})-(2.01+133.0*8/2500000))<1e-12,"prediction includes DATA descriptor and acoustic ACK service");
  Simulator::Destroy();
  std::cout<<"REVIEWED_OPTICAL_SELF_TEST_PASS checks="<<checks<<"\n";return true;
}
