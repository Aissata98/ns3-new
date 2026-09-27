#ifndef SENSORS_CLOSURE_ADAPTIVE_H
#define SENSORS_CLOSURE_ADAPTIVE_H

// Causal application state and bounded route-catalogue ranking. No ns-3 queue,
// replay, future-event, random-stream or network access is available here.
// A report enters ReceivedReports only after the network endpoint has decoded
// its actual transported bytes. Service predictions are declared planning
// allowances, never substituted for DATA/ACK receipts or physical validation.
#include "closure_mission.h"
#include <cstring>

namespace closure_adaptive {
using closure_mission::require;

struct ChunkDescriptor {
  std::uint64_t recordId=0, windowId=0, windowPayloadBytes=0;
  std::uint32_t station=0, chunkIndex=0, chunkCount=0, payloadBytes=0;
  double generatedAt=0, deadlineAt=0;
  bool lowInspection=true;
};

struct Window {
  std::uint64_t id=0, payloadBytes=0, storageBytes=0;
  std::uint32_t station=0;
  double generatedAt=0, deadlineAt=0;
  std::vector<std::uint32_t> chunkBytes;
  bool lowInspection=true;
};

inline std::uint64_t addBytes(std::uint64_t a,std::uint64_t b) {
  require(b<=std::numeric_limits<std::uint64_t>::max()-a,"Byte count overflow");
  return a+b;
}
inline void validateWindow(const Window& w,double now) {
  require(w.id>0 && w.payloadBytes>0 && !w.chunkBytes.empty() && w.chunkBytes.size()<=65535,
          "Invalid complete-window identity/size");
  require(std::isfinite(w.generatedAt) && w.generatedAt>=0 && w.generatedAt<=now+1e-9 &&
          std::isfinite(w.deadlineAt) && w.deadlineAt>=w.generatedAt,"Future/invalid window timing");
  std::uint64_t sum=0;
  for(auto b:w.chunkBytes) { require(b>0,"Zero-size window chunk");sum=addBytes(sum,b); }
  require(sum==w.payloadBytes && w.storageBytes>=sum,"Incomplete window or invalid storage size");
}
inline bool sameWindow(const Window& a,const Window& b) {
  return a.id==b.id&&a.station==b.station&&a.payloadBytes==b.payloadBytes&&a.storageBytes==b.storageBytes&&
      a.chunkBytes==b.chunkBytes&&a.lowInspection==b.lowInspection&&
      std::abs(a.generatedAt-b.generatedAt)<=1e-9&&std::abs(a.deadlineAt-b.deadlineAt)<=1e-9;
}

// Call with ONLY locally durable/committed chunks. Missing, queued-for-write,
// pending-optical or remotely advertised chunks cannot complete a local window.
inline std::vector<Window> completeWindows(const std::vector<ChunkDescriptor>& chunks,
                                         std::uint64_t metadataBytes,double observedAt) {
  struct Assembly { ChunkDescriptor identity;std::map<std::uint32_t,ChunkDescriptor> chunks; };
  std::map<std::uint64_t,Assembly> groups;
  std::set<std::uint64_t> recordIds;
  for(const auto& c:chunks) {
    require(c.recordId>0 && c.windowId>0 && c.windowPayloadBytes>0 && c.payloadBytes>0 &&
            c.chunkCount>0 && c.chunkCount<=65535 && c.chunkIndex<c.chunkCount,
            "Invalid producer window/chunk metadata");
    require(std::isfinite(c.generatedAt) && c.generatedAt>=0 && c.generatedAt<=observedAt+1e-9 &&
            std::isfinite(c.deadlineAt) && c.deadlineAt>=c.generatedAt,"Future/invalid chunk timing");
    require(recordIds.insert(c.recordId).second,"Duplicate local record must be deduplicated before assembly");
    auto found=groups.find(c.windowId);
    if(found==groups.end()) found=groups.emplace(c.windowId,Assembly{c,{}}).first;
    const auto& i=found->second.identity;
    require(i.station==c.station && i.windowPayloadBytes==c.windowPayloadBytes &&
            i.chunkCount==c.chunkCount && i.generatedAt==c.generatedAt && i.deadlineAt==c.deadlineAt&&i.lowInspection==c.lowInspection,
            "Conflicting immutable metadata inside a window");
    require(found->second.chunks.emplace(c.chunkIndex,c).second,"Duplicate window chunk index");
  }
  std::vector<Window> result;
  for(const auto& entry:groups) {
    const auto& a=entry.second;
    if(a.chunks.size()!=a.identity.chunkCount) continue;
    Window w{entry.first,a.identity.windowPayloadBytes,0,a.identity.station,
             a.identity.generatedAt,a.identity.deadlineAt,{}};
    w.lowInspection=a.identity.lowInspection;
    std::uint64_t sum=0;
    for(const auto& p:a.chunks) {
      sum=addBytes(sum,p.second.payloadBytes);
      w.storageBytes=addBytes(w.storageBytes,addBytes(p.second.payloadBytes,metadataBytes));
      w.chunkBytes.push_back(p.second.payloadBytes);
    }
    require(sum==w.payloadBytes,"All chunk indices present but total window payload is inconsistent");
    validateWindow(w,observedAt);result.push_back(std::move(w));
  }
  return result;
}

struct Report {
  std::uint32_t station=0;
  std::uint64_t sequence=0;
  double observedAt=0;
  std::uint32_t omittedCompleteWindows=0;
  std::vector<Window> windows;
};
namespace wire {
constexpr std::size_t reportHeaderBytes=29; // magic32/version8/station32/seq64/time64/omitted16/count16
constexpr std::size_t windowHeaderBytes=42; // id/payload/storage/time/time: 5*8 + chunks16
inline std::uint64_t ticks(double seconds) {
  require(std::isfinite(seconds)&&seconds>=0 && seconds<9e9,"Unencodable timestamp");
  return static_cast<std::uint64_t>(std::llround(seconds*1e9));
}
inline void put(std::vector<std::uint8_t>& b,std::uint64_t v,unsigned n) {
  for(unsigned i=n;i>0;--i)b.push_back(static_cast<std::uint8_t>(v>>(8*(i-1))));
}
inline std::uint64_t get(const std::vector<std::uint8_t>& b,std::size_t& at,unsigned n) {
  require(at<=b.size() && n<=b.size()-at,"Truncated telemetry payload");
  std::uint64_t v=0;for(unsigned i=0;i<n;++i)v=(v<<8)|b[at++];return v;
}
}

// Deterministic EDF truncation is explicit in the wire header; an incomplete
// report is not interpreted as proof that an unlisted station window is absent.
inline std::vector<std::uint8_t> encodeReport(Report report,std::size_t maximumBytes) {
  require(report.sequence>0 && std::isfinite(report.observedAt)&&report.observedAt>=0,
          "Invalid telemetry sequence/time");
  require(maximumBytes>=wire::reportHeaderBytes && maximumBytes<=1048576,
          "Explicit telemetry payload bound must be between 29 B and 1 MiB");
  std::sort(report.windows.begin(),report.windows.end(),[](const auto& a,const auto& b){
    return std::tie(a.deadlineAt,a.id)<std::tie(b.deadlineAt,b.id);
  });
  std::set<std::uint64_t> ids;std::vector<Window> included;
  std::size_t size=wire::reportHeaderBytes;
  std::uint64_t omitted=report.omittedCompleteWindows;
  for(const auto& w:report.windows) {
    validateWindow(w,report.observedAt);
    require(w.station==report.station && w.lowInspection && ids.insert(w.id).second,"Invalid/duplicate inspection report window");
    const auto required=wire::windowHeaderBytes+w.chunkBytes.size()*4;
    if(required>maximumBytes-size || included.size()==65535) {++omitted;continue;}
    size+=required;included.push_back(w);
  }
  require(omitted<=65535,"Report truncation count exceeds bounded wire format");
  std::vector<std::uint8_t> b;b.reserve(size);
  wire::put(b,0x43415231,4);wire::put(b,1,1);wire::put(b,report.station,4);
  wire::put(b,report.sequence,8);wire::put(b,wire::ticks(report.observedAt),8);
  wire::put(b,omitted,2);wire::put(b,included.size(),2);
  for(const auto& w:included) {
    wire::put(b,w.id,8);wire::put(b,w.payloadBytes,8);wire::put(b,w.storageBytes,8);
    wire::put(b,wire::ticks(w.generatedAt),8);wire::put(b,wire::ticks(w.deadlineAt),8);
    wire::put(b,w.chunkBytes.size(),2);
    for(auto bytes:w.chunkBytes)wire::put(b,bytes,4);
  }
  require(b.size()==size,"Telemetry encoded size mismatch");return b;
}
inline Report decodeReport(const std::vector<std::uint8_t>& b,std::size_t maximumBytes) {
  require(b.size()>=wire::reportHeaderBytes && b.size()<=maximumBytes,"Telemetry exceeds endpoint bound");
  std::size_t at=0;
  require(wire::get(b,at,4)==0x43415231 && wire::get(b,at,1)==1,"Unknown telemetry wire version");
  Report r;r.station=wire::get(b,at,4);r.sequence=wire::get(b,at,8);
  r.observedAt=wire::get(b,at,8)/1e9;r.omittedCompleteWindows=wire::get(b,at,2);
  const auto count=wire::get(b,at,2);
  require(r.sequence>0,"Zero telemetry sequence");std::set<std::uint64_t> ids;
  for(std::uint64_t i=0;i<count;++i) {
    Window w;w.station=r.station;w.id=wire::get(b,at,8);w.payloadBytes=wire::get(b,at,8);
    w.storageBytes=wire::get(b,at,8);w.generatedAt=wire::get(b,at,8)/1e9;w.deadlineAt=wire::get(b,at,8)/1e9;
    const auto chunks=wire::get(b,at,2);
    require(chunks>0 && chunks<=(b.size()-at)/4,"Truncated telemetry chunk list");
    for(std::uint64_t j=0;j<chunks;++j)w.chunkBytes.push_back(wire::get(b,at,4));
    validateWindow(w,r.observedAt);require(ids.insert(w.id).second,"Duplicate telemetry window");r.windows.push_back(std::move(w));
  }
  require(at==b.size(),"Trailing telemetry bytes");return r;
}

class ReceivedReports {
 public:
  bool accept(const Report& report,std::uint32_t physicalSourceStation,double receivedAt) {
    require(std::isfinite(receivedAt)&&receivedAt>=0&&report.observedAt<=receivedAt+1e-9,
            "Telemetry cannot arrive before its observation");
    require(report.station==physicalSourceStation,"Telemetry source endpoint mismatch");
    require(report.sequence>0,"Invalid telemetry sequence");
    std::set<std::uint64_t> ids;
    for(const auto& w:report.windows) {
      validateWindow(w,report.observedAt);
      require(w.station==report.station&&ids.insert(w.id).second,"Invalid report identity");
    }
    auto old=reports_.find(report.station);
    if(old!=reports_.end() && (report.sequence<=old->second.sequence || report.observedAt<old->second.observedAt))return false;
    reports_[report.station]=report;return true;
  }
  std::vector<Report> fresh(double now,double maxAge) const {
    require(std::isfinite(now)&&now>=0&&std::isfinite(maxAge)&&maxAge>=0,"Invalid report age bound");
    std::vector<Report> result;
    for(const auto& entry:reports_)if(entry.second.observedAt<=now+1e-9 && now-entry.second.observedAt<=maxAge)
      result.push_back(entry.second);
    return result;
  }
  std::size_t size()const{return reports_.size();}
 private:
  std::map<std::uint32_t,Report> reports_;
};

// Coverage is a hard traversal rule, not a score bonus. Unknown stations remain
// visitable. The same state/guard must be used by static and adaptive missions.
struct Coverage {
  std::vector<std::uint32_t> stations;
  std::map<std::uint32_t,double> lastVisits;
  std::set<std::uint32_t> visitedThisRound;
  double maxRevisitS=0;
  bool allows(std::uint32_t station)const {
    return std::find(stations.begin(),stations.end(),station)!=stations.end() && !visitedThisRound.count(station);
  }
  bool visit(std::uint32_t station,double at) {
    require(std::isfinite(maxRevisitS)&&maxRevisitS>0&&std::isfinite(at)&&at>=0,"Invalid coverage timing");
    require(allows(station),"Coverage forbids revisiting an easy station before completing the round");
    auto prior=lastVisits.find(station);const double previous=prior==lastVisits.end()?0:prior->second;
    const bool onTime=at<=previous+maxRevisitS+1e-9;
    lastVisits[station]=at;visitedThisRound.insert(station);
    if(visitedThisRound.size()==stations.size())visitedThisRound.clear();
    return onTime;
  }
  bool withinBound(double now)const {
    for(auto station:stations) {
      auto p=lastVisits.find(station);const double last=p==lastVisits.end()?0:p->second;
      if(now>last+maxRevisitS+1e-9)return false;
    }
    return true;
  }
};

struct Step {
  closure_mission::ContinuousLeg leg;
  std::string kind;
  std::int32_t station=-1;
  // Allowed elapsed channel service, not successful transfer invented here.
  double pickupServiceS=0;
  std::uint64_t pickupCreditBytes=std::numeric_limits<std::uint64_t>::max();
};
struct Option { std::string id;std::vector<Step> steps; };
struct ServicePrediction {
  // Include explicit control/acquisition, framing, ACK, turnaround and attempts.
  // Values are a common prediction contract, not measured success guarantees.
  double pickupRateBps=0,pickupPerRecordS=0,pickupAcquisitionS=0;
  double offloadRateBps=0,offloadPerRecordS=0;
  std::uint32_t offloadFragmentBytes=0,offloadHeaderBytes=0,offloadAckBytes=0,offloadAttempts=0;
  double offloadPropagationRoundTripS=0,offloadGuardS=0;
  std::uint32_t offloadOtherHeaderBytes=0;
  double pickupSeconds(const Window& w)const {
    return w.payloadBytes*8.0/pickupRateBps+w.chunkBytes.size()*pickupPerRecordS;
  }
  double offloadSeconds(const Window& w)const {
    double seconds=0;
    for(auto payload:w.chunkBytes) {
      const std::uint64_t body=payload;
      const std::uint64_t fragments=(body+offloadFragmentBytes-1)/offloadFragmentBytes;
      const auto header=(!w.lowInspection&&offloadOtherHeaderBytes)?offloadOtherHeaderBytes:offloadHeaderBytes;
      const double wire=static_cast<double>(body)+fragments*header;
      seconds+=offloadAttempts*((wire+offloadAckBytes)*8.0/offloadRateBps+
                              offloadPropagationRoundTripS+offloadGuardS+offloadPerRecordS);
    }
    return seconds;
  }
  void validate()const {
    for(double v:{pickupRateBps,offloadRateBps})require(std::isfinite(v)&&v>0,"Positive planning service rates required");
    for(double v:{pickupPerRecordS,pickupAcquisitionS,offloadPerRecordS,offloadPropagationRoundTripS,offloadGuardS})
      require(std::isfinite(v)&&v>=0,"Explicit nonnegative planning overhead required");
    require(offloadFragmentBytes>0&&offloadHeaderBytes>0&&offloadAckBytes>0&&offloadAttempts>0,
            "Explicit finite offload framing/ACK/attempt contract required");
  }
};
struct Input {
  double now=0,horizon=0,remainingJ=0,reserveJ=0,guardedPowerW=0,recoveryS=0,recoveryJ=0;
  std::uint64_t carryCapacity=0,carryUsed=0;
  closure_mission::Pose current,home;
  double otherOffloadSeconds=0;
  std::vector<Window> onboard;
  std::vector<Report> freshReports;
  Coverage coverage;
  ServicePrediction service;
};
struct Assessment {
  std::size_t optionIndex=0;
  std::vector<std::uint64_t> picked,onTime;
  std::size_t projectedLate=0,pickups=0;
  bool firstActionContinues=false;
  std::uint64_t onTimeBytes=0,peakCarry=0;
  double durationS=0,energyJ=0;
  std::string id;
};
struct Selection {
  std::optional<Assessment> chosen;
  std::vector<Assessment> feasible;
  std::vector<std::pair<std::string,std::string>> rejected;
};
inline bool better(const Assessment& a,const Assessment& b) {
  if(a.projectedLate!=b.projectedLate)return a.projectedLate<b.projectedLate;
  const long double left=static_cast<long double>(a.onTimeBytes)*b.durationS;
  const long double right=static_cast<long double>(b.onTimeBytes)*a.durationS;
  if(left!=right)return left>right;
  if(a.durationS!=b.durationS)return a.durationS<b.durationS;
  return a.id<b.id;
}

inline Selection select(const Input& in,const std::vector<Option>& options) {
  in.service.validate();
  require(std::isfinite(in.now)&&in.now>=0&&std::isfinite(in.horizon)&&in.horizon>in.now&&
          std::isfinite(in.coverage.maxRevisitS)&&in.coverage.maxRevisitS>0,"Explicit finite planning/coverage horizon required");
  for(double v:{in.remainingJ,in.reserveJ,in.recoveryJ,in.recoveryS,in.guardedPowerW})
    require(std::isfinite(v)&&v>=0,"Invalid planning energy contract");
  require(in.recoveryJ>0&&in.recoveryS>0&&in.guardedPowerW>0&&in.carryUsed<=in.carryCapacity,
          "Missing recovery or finite carry contract");
  require(std::isfinite(in.otherOffloadSeconds)&&in.otherOffloadSeconds>=0,"Invalid partial-record service allowance");
  std::set<std::uint32_t> stations;
  for(auto station:in.coverage.stations)require(stations.insert(station).second,"Duplicate coverage station");
  require(!stations.empty(),"Coverage requires physical stations");
  for(const auto& p:in.coverage.lastVisits)
    require(stations.count(p.first)&&std::isfinite(p.second)&&p.second>=0&&p.second<=in.now+1e-9,"Invalid last-visit observation");
  for(auto station:in.coverage.visitedThisRound)require(stations.count(station),"Invalid coverage-round station");
  std::map<std::uint64_t,Window> known,initial;
  std::map<std::uint32_t,std::vector<Window>> reports;
  for(const auto& r:in.freshReports) {
    require(r.observedAt<=in.now+1e-9&&reports.emplace(r.station,r.windows).second,"Invalid snapshot report input");
    for(const auto& w:r.windows){validateWindow(w,r.observedAt);
      require(w.station==r.station&&known.emplace(w.id,w).second,"Duplicate or conflicting gateway window identity");}
  }
  std::uint64_t represented=0;
  for(const auto& w:in.onboard){validateWindow(w,in.now);require(initial.emplace(w.id,w).second,"Duplicate onboard window");
    represented=addBytes(represented,w.storageBytes);
    const auto prior=known.find(w.id);
    require(prior==known.end()||sameWindow(prior->second,w),"Conflicting onboard/report immutable window identity");
    known[w.id]=w;}
  require(represented<=in.carryUsed,"Complete onboard descriptors exceed actual committed carry");
  Selection result;
  for(std::size_t oi=0;oi<options.size();++oi) {
    const auto& option=options[oi];Assessment a;a.optionIndex=oi;a.id=option.id;a.peakCarry=in.carryUsed;
    a.firstActionContinues=!option.steps.empty()&&option.steps.front().kind!="terminal-offload-return";
    std::string failure;auto carried=initial;std::set<std::uint64_t> seen;
    for(const auto& p:initial)seen.insert(p.first);
    auto coverage=in.coverage;auto pose=in.current;double clock=in.now;
    std::uint64_t used=in.carryUsed;std::map<std::uint64_t,Window> timely;
    double projectedOffload=in.now;
    double projectedOffloadAllowance=0;
    for(const auto& step:option.steps){projectedOffload+=step.leg.durationS();if(step.kind=="offload"||step.kind=="terminal-offload-return"){
        projectedOffloadAllowance=step.leg.serviceDurationS;break;}}
    double projectedOffloadUsed=in.otherOffloadSeconds;
    for(const auto& p:carried)projectedOffloadUsed+=in.service.offloadSeconds(p.second);
    bool returned=false;
    for(const auto& step:option.steps) {
      const auto& leg=step.leg;
      if(!leg.checked||!std::isfinite(leg.durationS())||leg.durationS()<=0||closure_mission::distance(pose.position,leg.start.position)>1e-8||
         std::abs(closure_mission::angleDifference(pose.yaw,leg.start.yaw))>1e-9){failure="uncertified_or_discontinuous_geometry";break;}
      if(std::abs(leg.constraints.speedMps-closure_mission::fixedThreeKnotsMps)>1e-12||!leg.constraints.calmWater){failure="inconsistent_fixed_speed_environment";break;}
      clock+=leg.durationS();pose=leg.end;
      if(clock+in.recoveryS>in.horizon+1e-9){failure="finite_horizon_return_recovery";break;}
      if((clock-in.now)*in.guardedPowerW+in.recoveryJ+in.reserveJ>in.remainingJ+1e-6){failure="finite_energy_return_reserve";break;}
      if(step.kind=="pickup") {
        if(step.station<0||!coverage.allows(step.station)){failure="coverage_round_progression";break;}
        if(!coverage.visit(step.station,clock)){failure="maximum_revisit_interval";break;}
        ++a.pickups;
        if(a.pickups>2){failure="catalogue_exceeds_two_pickups";break;}
        double service=std::max(0.0,step.pickupServiceS-in.service.pickupAcquisitionS);
        std::uint64_t credit=step.pickupCreditBytes;
        auto windows=reports[step.station];
        std::sort(windows.begin(),windows.end(),[](const auto& x,const auto& y){return std::tie(x.deadlineAt,x.id)<std::tie(y.deadlineAt,y.id);});
        for(const auto& w:windows) {
          const double needed=in.service.pickupSeconds(w);
          const double offloadNeeded=in.service.offloadSeconds(w);
          if(seen.count(w.id)||w.deadlineAt<projectedOffload||w.storageBytes>in.carryCapacity-used||w.storageBytes>credit||
             needed>service+1e-9||projectedOffloadUsed+offloadNeeded>projectedOffloadAllowance+1e-9)continue;
          carried[w.id]=w;seen.insert(w.id);used+=w.storageBytes;service-=needed;credit-=w.storageBytes;
          projectedOffloadUsed+=offloadNeeded;a.picked.push_back(w.id);a.peakCarry=std::max(a.peakCarry,used);
        }
      }
      if(step.kind=="offload"||step.kind=="terminal-offload-return") {
        double service=in.otherOffloadSeconds;for(const auto& p:carried)service+=in.service.offloadSeconds(p.second);
        if(service>leg.serviceDurationS+1e-9){failure="whole_window_offload_service_exceeded";break;}
        for(const auto& p:carried)if(clock<=p.second.deadlineAt+1e-9)timely[p.first]=p.second;
        carried.clear(); // Prediction only. Actual custody waits for ACK/terminal policy.
        used=in.carryUsed-represented;
        returned=step.kind=="terminal-offload-return";
      }
    }
    if(failure.empty()&&(!returned||closure_mission::distance(pose.position,in.home.position)>1e-8||
        std::abs(closure_mission::angleDifference(pose.yaw,in.home.yaw))>1e-9))failure="missing_terminal_offload_return";
    // Coverage obligations stop with the finite deployed mission, not the
    // observation horizon after a recovered vehicle has ceased operations.
    if(failure.empty()&&!coverage.withinBound(clock))failure="unserved_revisit_before_return";
    if(!failure.empty()){result.rejected.emplace_back(option.id,failure);continue;}
    for(const auto& p:timely){a.onTime.push_back(p.first);a.onTimeBytes=addBytes(a.onTimeBytes,p.second.payloadBytes);}
    for(const auto& p:known)if(p.second.deadlineAt<=in.horizon&&!timely.count(p.first))++a.projectedLate;
    a.durationS=clock-in.now;a.energyJ=a.durationS*in.guardedPowerW+in.recoveryJ;
    result.feasible.push_back(std::move(a));
  }
  // With no carried data, a feasible coverage visit precedes voluntary empty
  // recovery. This gives unknown stations an honest exploration opportunity;
  // no backlog is invented and no score is awarded for unknown data.
  const bool explore=in.carryUsed==0 && std::any_of(result.feasible.begin(),result.feasible.end(),[](const auto& a){return a.pickups>0;});
  // Predeclared finite-mission service guard: an offload is not definitive
  // recovery while a physically feasible continuation exists. The stored
  // terminal suffix remains a fallback; there is still no prediction of future
  // arrivals or hidden queues. Static and adaptive use the same service horizon.
  const bool continueMission=std::any_of(result.feasible.begin(),result.feasible.end(),[](const auto& a){return a.firstActionContinues;});
  std::sort(result.feasible.begin(),result.feasible.end(),better);
  for(const auto& a:result.feasible)if((!explore||a.pickups>0)&&(!continueMission||a.firstActionContinues)){result.chosen=a;break;}
  return result;
}
} // namespace closure_adaptive
#endif
