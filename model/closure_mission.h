#ifndef SENSORS_CLOSURE_MISSION_H
#define SENSORS_CLOSURE_MISSION_H

// C++17, standard library only. This is an executable engineering scenario,
// NOT manufacturer-certified HUGIN dynamics or a field-validation result.
// Coordinates use positive depth. Existing gateway and surface coordinates are
// supplied by EDC and are never relocated here. The reference is calm water:
// fixed water-relative speed then equals fixed ground arclength speed.
//
// EDC integration points (the adapter must actually wire ALL of these):
// 1. Construct east/west poses at existing 500-m gateways and a submerged
//    rendezvous under an EXISTING surface sink. Build/check legs before runs.
// 2. Replace GetAuvPosition with Mission::evaluatedPosition; BOTH mobility and
//    optical/acoustic start/end eligibility must query the same trajectory.
// 3. Schedule nextDecisionTime() exactly, not only rounded scheduler ticks;
//    startLeg executes the first selected leg and replans using RECEIVED reports.
// 4. In the integrated mode advance(t) delegates concurrent operation, acoustic
//    and added optical loads to the shared PowerLedger. Remove the old cubic
//    debit; never use debitCommunication alongside an elapsed-power owner.
// 5. Reserve carry bytes before optical admission, including pending transfers;
//    release only on explicit ownership/ACK policy. Geometry does not deliver data.
// 6. Offload legs include powered continuous circles, not free stationary waits.
//    Capacity is an optimistic allowance; EDC still executes packets, ACKs and
//    failures and decides whether final surface transfer actually completed.
// 7. Every proposed pickup includes a feasible offload/return suffix plus the
//    explicit recovery allowance and 20% reserve. A returned submerged vehicle
//    is NOT on deck. Optional external-recovery accounting is labeled as such.
//    Continue source arrivals over the original horizon during unavailability.
// 8. On depletion stop new AUV service and preserve queued data. Never recharge
//    implicitly, clamp time backwards, or continue motion after exhaustion.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "closure_energy.h"

namespace closure_mission {

constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double fixedThreeKnotsMps = 1852.0 / 1200.0;
inline double wrap(double a) { a = std::fmod(a, 2*pi); return a < 0 ? a + 2*pi : a; }
inline double angleDifference(double a, double b) { return std::remainder(a-b, 2*pi); }
inline bool finite(double x) { return std::isfinite(x); }
inline void require(bool condition, const std::string& reason) {
  if (!condition) throw std::invalid_argument(reason);
}

struct Vec3 { double x=0, y=0, z=0; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, double b) { return {a.x*b,a.y*b,a.z*b}; }
inline double norm(Vec3 p) { return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z); }
inline double distance(Vec3 a, Vec3 b) { return norm(a-b); }
struct Pose { Vec3 position; double yaw=0; };

struct RouteConstraints {
  double speedMps=fixedThreeKnotsMps;
  double minimumDepthM=20, maximumDepthM=6000;
  double minimumThreeDimensionalRadiusM=50;
  double constructionHorizontalRadiusM=100;
  double maximumPitchRadians=pi/4;
  double minimumX=-500, maximumX=13100, maximumAbsY=500;
  double offloadDepthM=50, acousticServiceGateM=500;
  bool calmWater=true;
  std::string provenance="Declared engineering route envelope; not a HUGIN maneuver certificate";
  void validate() const {
    require(calmWater, "Nonzero-current motion requires a separate water/ground vector model");
    require(finite(speedMps) && std::abs(speedMps-fixedThreeKnotsMps)<1e-12, "Only the fixed 3-kn regime is supported");
    require(finite(minimumDepthM) && minimumDepthM>0 && finite(maximumDepthM) && maximumDepthM>=minimumDepthM, "Invalid depth envelope");
    require(finite(minimumThreeDimensionalRadiusM) && minimumThreeDimensionalRadiusM>0 && finite(constructionHorizontalRadiusM)
            && constructionHorizontalRadiusM>minimumThreeDimensionalRadiusM, "Construction radius must exceed the conservative 3-D minimum");
    require(finite(maximumPitchRadians) && maximumPitchRadians>0 && maximumPitchRadians<pi/2, "Invalid pitch envelope");
    require(finite(minimumX) && finite(maximumX) && minimumX<maximumX && finite(maximumAbsY) && maximumAbsY>0, "Invalid working corridor");
    require(offloadDepthM>=minimumDepthM && offloadDepthM<=maximumDepthM && acousticServiceGateM>offloadDepthM,
            "Invalid submerged acoustic offload envelope");
    require(!provenance.empty(), "Engineering assumptions need a provenance label");
  }
};

struct PlanarSegment {
  Pose start;
  char turn='S';
  double lengthM=0, radiusM=100;
  Pose at(double q) const {
    q=std::clamp(q,0.0,lengthM);
    if (turn=='S') return {{start.position.x+q*std::cos(start.yaw),start.position.y+q*std::sin(start.yaw),start.position.z},start.yaw};
    double sign=turn=='L' ? 1.0 : -1.0, yaw=start.yaw+sign*q/radiusM;
    return {{start.position.x+sign*radiusM*(std::sin(yaw)-std::sin(start.yaw)),
             start.position.y-sign*radiusM*(std::cos(yaw)-std::cos(start.yaw)),start.position.z},yaw};
  }
};

struct PlanarPath {
  Pose start;
  std::vector<PlanarSegment> segments;
  double lengthM=0;
  void append(char kind,double length,double radius) {
    if (length<=1e-12) return;
    Pose current=segments.empty() ? start : segments.back().at(segments.back().lengthM);
    segments.push_back({current,kind,length,radius}); lengthM+=length;
  }
  Pose at(double q, double* curvature=nullptr) const {
    if (curvature) *curvature=0;
    for (const auto& segment:segments) {
      if (q<=segment.lengthM+1e-10) {
        if (curvature) *curvature=segment.turn=='S' ? 0 : 1/segment.radiusM;
        return segment.at(q);
      }
      q-=segment.lengthM;
    }
    return segments.empty() ? start : segments.back().at(segments.back().lengthM);
  }
  double maxCurvature() const {
    double result=0; for (const auto& s:segments) if(s.turn!='S') result=std::max(result,1/s.radiusM); return result;
  }
  bool inside(const RouteConstraints& c) const {
    auto valid=[&](Pose p) { return p.position.x>=c.minimumX-1e-8 && p.position.x<=c.maximumX+1e-8 && std::abs(p.position.y)<=c.maximumAbsY+1e-8; };
    if (!valid(start)) return false;
    for (const auto& s:segments) {
      if (!valid(s.at(s.lengthM))) return false;
      if (s.turn=='S') continue;
      double sign=s.turn=='L' ? 1.0 : -1.0;
      // x/y extrema occur at integer multiples of pi/2 of the tangent yaw.
      for (int k=0;k<4;++k) {
        double angle=k*pi/2, delta=wrap(sign*(angle-s.start.yaw));
        double q=delta*s.radiusM;
        if (q<=s.lengthM+1e-8 && !valid(s.at(q))) return false;
      }
    }
    return true;
  }
};

// The six circular/straight bounded-curvature words. Endpoints and tangents are
// independently checked, so an invalid algebraic candidate is never accepted.
inline std::vector<PlanarPath> planarCandidates(Pose start,Pose end,double r) {
  double dx=end.position.x-start.position.x,dy=end.position.y-start.position.y;
  double d=std::hypot(dx,dy)/r,theta=std::atan2(dy,dx),a=wrap(start.yaw-theta),b=wrap(end.yaw-theta);
  double sa=std::sin(a),sb=std::sin(b),ca=std::cos(a),cb=std::cos(b),cab=std::cos(a-b);
  std::vector<PlanarPath> result;
  auto add=[&](const char* word,double t,double p,double q) {
    if (!finite(t)||!finite(p)||!finite(q)||t<0||p<0||q<0) return;
    PlanarPath path;path.start=start;
    path.append(word[0],t*r,r);path.append(word[1],p*r,r);path.append(word[2],q*r,r);
    auto last=path.at(path.lengthM);
    if (std::hypot(last.position.x-end.position.x,last.position.y-end.position.y)<1e-6
        && std::abs(angleDifference(last.yaw,end.yaw))<1e-8) result.push_back(path);
  };
  double p2=2+d*d-2*cab+2*d*(sa-sb),tmp;
  if(p2>=-1e-12){tmp=std::atan2(cb-ca,d+sa-sb);add("LSL",wrap(-a+tmp),std::sqrt(std::max(0.0,p2)),wrap(b-tmp));}
  p2=2+d*d-2*cab+2*d*(-sa+sb);
  if(p2>=-1e-12){tmp=std::atan2(ca-cb,d-sa+sb);add("RSR",wrap(a-tmp),std::sqrt(std::max(0.0,p2)),wrap(-b+tmp));}
  p2=-2+d*d+2*cab+2*d*(sa+sb);
  if(p2>=-1e-12){double p=std::sqrt(std::max(0.0,p2));tmp=std::atan2(-ca-cb,d+sa+sb)-std::atan2(-2.0,p);add("LSR",wrap(-a+tmp),p,wrap(-b+tmp));}
  p2=d*d-2+2*cab-2*d*(sa+sb);
  if(p2>=-1e-12){double p=std::sqrt(std::max(0.0,p2));tmp=std::atan2(ca+cb,d-sa-sb)-std::atan2(2.0,p);add("RSL",wrap(a-tmp),p,wrap(b-tmp));}
  tmp=(6-d*d+2*cab+2*d*(sa-sb))/8;
  if(std::abs(tmp)<=1+1e-12){double p=wrap(2*pi-std::acos(std::clamp(tmp,-1.0,1.0)));double t=wrap(a-std::atan2(ca-cb,d-sa+sb)+p/2);add("RLR",t,p,wrap(a-b-t+p));}
  tmp=(6-d*d+2*cab+2*d*(-sa+sb))/8;
  if(std::abs(tmp)<=1+1e-12){double p=wrap(2*pi-std::acos(std::clamp(tmp,-1.0,1.0)));double t=wrap(-a-std::atan2(ca-cb,d+sa-sb)+p/2);add("LRL",t,p,wrap(b-a-t+p));}
  return result;
}

struct SpatialPiece {
  PlanarPath planar;
  double startDepth=0,endDepth=0,lengthM=0,maxCurvatureBound=0,maxPitch=0;
  std::vector<double> qGrid,sGrid;
  double dzdq(double q) const {if(planar.lengthM<=0)return 0;double u=q/planar.lengthM;return (endDepth-startDepth)*6*u*(1-u)/planar.lengthM;}
  double metric(double q) const {return std::hypot(1.0,dzdq(q));}
  double integral(double lo,double hi) const {
    if(startDepth==endDepth)return hi-lo;
    constexpr double x[]={-0.9061798459386640,-0.5384693101056831,0,0.5384693101056831,0.9061798459386640};
    constexpr double w[]={0.2369268850561891,0.4786286704993665,0.5688888888888889,0.4786286704993665,0.2369268850561891};
    double mid=(lo+hi)/2,half=(hi-lo)/2,sum=0;
    for(int i=0;i<5;++i)sum+=w[i]*metric(mid+half*x[i]);
    return half*sum;
  }
  bool initialize(const RouteConstraints& c) {
    double dz=std::abs(endDepth-startDepth),h=planar.lengthM;
    if(std::min(startDepth,endDepth)<c.minimumDepthM || std::max(startDepth,endDepth)>c.maximumDepthM || !planar.inside(c))return false;
    if(h<=1e-12){if(dz>1e-12)return false;lengthM=0;maxPitch=0;maxCurvatureBound=0;qGrid={0};sGrid={0};return true;}
    maxPitch=std::atan(1.5*dz/h);
    maxCurvatureBound=std::hypot(planar.maxCurvature(),6*dz/(h*h));
    if(maxPitch>c.maximumPitchRadians+1e-12 || maxCurvatureBound>1/c.minimumThreeDimensionalRadiusM+1e-12)return false;
    // 256 intervals plus local Newton inversion: true 3-D arclength, not an
    // average stretch. Tested against independent numerical derivatives.
    qGrid={0};sGrid={0};lengthM=0;
    for(int i=1;i<=256;++i){double q=h*i/256;lengthM+=integral(qGrid.back(),q);qGrid.push_back(q);sGrid.push_back(lengthM);}
    return true;
  }
  double horizontalAtArc(double s) const {
    if(lengthM<=0)return 0;
    s=std::clamp(s,0.0,lengthM);
    if(startDepth==endDepth)return s;
    auto it=std::upper_bound(sGrid.begin(),sGrid.end(),s);std::size_t hi=std::min<std::size_t>(256,it-sGrid.begin());
    if(hi==0)return 0;
    std::size_t lo=hi-1;
    double left=qGrid[lo],right=qGrid[hi],q=left+(right-left)*(s-sGrid[lo])/(sGrid[hi]-sGrid[lo]);
    for(int k=0;k<5;++k){double error=sGrid[lo]+integral(qGrid[lo],q)-s;
      if(std::abs(error)<1e-11)break;
      if(error>0)right=q;else left=q;
      double next=q-error/metric(q);q=(next>=left && next<=right)?next:(left+right)/2;}
    return std::clamp(q,qGrid[lo],qGrid[hi]);
  }
  Pose atArc(double s) const {
    double q=horizontalAtArc(s);auto pose=planar.at(q);
    double u=planar.lengthM>0?q/planar.lengthM:0;
    pose.position.z=startDepth+(endDepth-startDepth)*(3*u*u-2*u*u*u);return pose;
  }
  Vec3 tangentAtArc(double s) const {
    double q=horizontalAtArc(s);auto pose=planar.at(q);double z=dzdq(q),m=std::hypot(1.0,z);
    return {std::cos(pose.yaw)/m,std::sin(pose.yaw)/m,z/m};
  }
  double curvatureAtArc(double s) const {
    if(planar.lengthM<=0)return 0;
    double q=horizontalAtArc(s),k=0;planar.at(q,&k);
    double z=dzdq(q),zz=6*(endDepth-startDepth)*(1-2*q/planar.lengthM)/(planar.lengthM*planar.lengthM);
    return std::sqrt(k*k*(1+z*z)+zz*zz)/std::pow(1+z*z,1.5);
  }
};

struct ContinuousLeg {
  std::string id;
  RouteConstraints constraints;
  std::vector<SpatialPiece> pieces;
  Pose start,end;
  double lengthM=0,serviceStartS=0,serviceDurationS=0;
  std::uint64_t optimisticServiceBytes=0;
  bool checked=false;
  double durationS() const {return lengthM/constraints.speedMps;}
  Pose evaluatedPose(double elapsedS) const {
    double s=std::clamp(elapsedS,0.0,durationS())*constraints.speedMps;
    for(const auto& p:pieces){if(s<=p.lengthM+1e-9)return p.atArc(s);s-=p.lengthM;}return end;
  }
  Vec3 tangent(double elapsedS) const {
    double s=std::clamp(elapsedS,0.0,durationS())*constraints.speedMps;
    for(const auto& p:pieces){if(s<=p.lengthM+1e-9)return p.tangentAtArc(s);s-=p.lengthM;}
    return {std::cos(end.yaw),std::sin(end.yaw),0};
  }
  double curvature(double elapsedS) const {
    double s=std::clamp(elapsedS,0.0,durationS())*constraints.speedMps;
    for(const auto& p:pieces){if(s<=p.lengthM+1e-9)return p.curvatureAtArc(s);s-=p.lengthM;}return 0;
  }
};

inline ContinuousLeg makeLeg(const std::string& id,Pose start,Pose end,const RouteConstraints& c) {
  c.validate();require(!id.empty(),"Leg ID required");
  for(double v:{start.position.x,start.position.y,start.position.z,start.yaw,end.position.x,end.position.y,end.position.z,end.yaw})require(finite(v),"Nonfinite pose");
  std::optional<ContinuousLeg> best;
  for(const auto& base:planarCandidates(start,end,c.constructionHorizontalRadiusM)) {
    double dz=std::abs(end.position.z-start.position.z),kh=1/c.constructionHorizontalRadiusM;
    double required=std::max(1.5*dz/std::tan(c.maximumPitchRadians),
      std::sqrt(6*dz/std::sqrt(1/(c.minimumThreeDimensionalRadiusM*c.minimumThreeDimensionalRadiusM)-kh*kh)));
    double extra=std::ceil((required-base.lengthM)/(2*pi*c.constructionHorizontalRadiusM));
    if(!finite(extra)||extra>64)continue;
    int loops=extra>0?static_cast<int>(extra):0;
    if(loops>64)continue;
    for(char side:{'L','R'}) {
      PlanarPath path;path.start=start;
      for(int i=0;i<loops;++i)path.append(side,2*pi*c.constructionHorizontalRadiusM,c.constructionHorizontalRadiusM);
      for(const auto& segment:base.segments)path.append(segment.turn,segment.lengthM,segment.radiusM);
      SpatialPiece piece;piece.planar=path;piece.startDepth=start.position.z;piece.endDepth=end.position.z;
      if(!piece.initialize(c))continue;
      ContinuousLeg leg;leg.id=id;leg.constraints=c;leg.pieces={piece};leg.start=start;leg.end=end;leg.lengthM=piece.lengthM;leg.checked=true;
      if(!best || leg.lengthM<best->lengthM)best=leg;
    }
  }
  if(!best)throw std::invalid_argument("No continuous leg fits the declared engineering corridor/depth/curvature");
  return *best;
}

inline ContinuousLeg makeLoiter(const std::string& id,Pose pose,const RouteConstraints& c,int circles=1,char side='L') {
  c.validate();require(circles>0 && circles<=100000,"Positive bounded loiter-circle count required");
  require(side=='L'||side=='R',"Loiter direction must be L or R");
  PlanarPath path;path.start=pose;path.append(side,circles*2*pi*c.constructionHorizontalRadiusM,c.constructionHorizontalRadiusM);
  SpatialPiece piece;piece.planar=path;piece.startDepth=piece.endDepth=pose.position.z;
  require(piece.initialize(c),"Loiter violates engineering geometry envelope");
  ContinuousLeg leg;leg.id=id;leg.constraints=c;leg.pieces={piece};leg.start=leg.end=pose;leg.lengthM=piece.lengthM;leg.checked=true;return leg;
}

inline ContinuousLeg concatenate(const std::string& id,const ContinuousLeg& a,const ContinuousLeg& b) {
  require(a.checked&&b.checked && distance(a.end.position,b.start.position)<1e-6
          && std::abs(angleDifference(a.end.yaw,b.start.yaw))<1e-8,"Discontinuous leg join");
  require(std::abs(a.constraints.speedMps-b.constraints.speedMps)<1e-12,"Speed mismatch at join");
  ContinuousLeg result=a;result.id=id;result.pieces.insert(result.pieces.end(),b.pieces.begin(),b.pieces.end());
  result.end=b.end;result.lengthM=a.lengthM+b.lengthM;result.serviceStartS=0;result.serviceDurationS=0;result.optimisticServiceBytes=0;return result;
}

inline Pose submergedOffloadPose(Vec3 existingSurfaceSink,double yaw,const RouteConstraints& c) {
  require(std::abs(existingSurfaceSink.z)<1e-8,"Offload receiver must be an existing surface endpoint");
  return {{existingSurfaceSink.x,existingSurfaceSink.y,c.offloadDepthM},yaw};
}

inline ContinuousLeg makeOffloadLeg(const std::string& id,Pose start,Vec3 existingSurfaceSink,double yaw,
                                    double requestedServiceS,double applicationRateBps,const RouteConstraints& c) {
  require(finite(requestedServiceS)&&requestedServiceS>0 && finite(applicationRateBps)&&applicationRateBps>0,"Explicit positive service duration/rate required");
  Pose target=submergedOffloadPose(existingSurfaceSink,yaw,c);
  auto approach=makeLeg(id+"/approach",start,target,c);
  double needed=std::ceil(requestedServiceS/(2*pi*c.constructionHorizontalRadiusM/c.speedMps));
  require(needed<=100000,"Service request exceeds bounded circle catalogue");
  int circles=static_cast<int>(needed);
  std::optional<ContinuousLeg> hold;
  for(char side:{'L','R'}) {
    double sign=side=='L'?1.0:-1.0;
    Vec3 center{target.position.x-sign*c.constructionHorizontalRadiusM*std::sin(yaw),
                target.position.y+sign*c.constructionHorizontalRadiusM*std::cos(yaw),target.position.z};
    double maxHorizontal=std::hypot(center.x-existingSurfaceSink.x,center.y-existingSurfaceSink.y)+c.constructionHorizontalRadiusM;
    if(std::hypot(maxHorizontal,target.position.z-existingSurfaceSink.z)>c.acousticServiceGateM+1e-8)continue;
    try {hold=makeLoiter(id+"/service",target,c,circles,side);break;}catch(const std::invalid_argument&){}
  }
  require(hold.has_value(),"No continuous acoustic service circle remains in the receiver gate");
  auto result=concatenate(id,approach,*hold);result.serviceStartS=approach.durationS();result.serviceDurationS=hold->durationS();
  long double bytes=static_cast<long double>(result.serviceDurationS)*applicationRateBps/8;
  require(bytes<=std::numeric_limits<std::uint64_t>::max(),"Service capacity overflow");
  result.optimisticServiceBytes=static_cast<std::uint64_t>(std::floor(bytes));return result;
}

class FiniteCarry {
 public:
  struct Item {std::uint64_t bytes;bool committed;};
  explicit FiniteCarry(std::uint64_t capacity):capacity_(capacity){}
  bool reserve(const std::string& id,std::uint64_t bytes) {
    require(!id.empty()&&bytes>0,"Positive identified carry reservation required");
    auto found=items_.find(id);if(found!=items_.end()){require(found->second.bytes==bytes,"Conflicting carry reservation");return true;}
    if(bytes>capacity_-used_)return false;
    items_[id]={bytes,false};used_+=bytes;return true;
  }
  bool commit(const std::string& id){auto it=items_.find(id);if(it==items_.end())return false;it->second.committed=true;return true;}
  bool release(const std::string& id){auto it=items_.find(id);if(it==items_.end())return false;used_-=it->second.bytes;items_.erase(it);return true;}
  std::uint64_t usedBytes()const{return used_;}std::uint64_t capacityBytes()const{return capacity_;}
  const std::map<std::string,Item>& items()const{return items_;}
 private:std::uint64_t capacity_=0,used_=0;std::map<std::string,Item> items_;
};

struct RecoveryAllowance {
  double durationS,vehicleEnergyJ;
  std::string provenance;
  void validate()const{require(finite(durationS)&&durationS>0&&finite(vehicleEnergyJ)&&vehicleEnergyJ>0&&!provenance.empty(),
                               "A positive explicit recovery handling allowance is required; zero is not a physical recovery model");}
};

class Mission {
 public:
  enum class State {Operating,Depleted,ExternalRecoveryAccounting,RecoveryAccountingComplete};
  struct Block {double startS;ContinuousLeg leg;};
  Mission(Pose home,const RouteConstraints& constraints,RecoveryAllowance recovery,std::uint64_t carryBytes,
          double totalPowerW=67000.0/65.0,double batteryKwh=67.0,double reserveFraction=0.2,
          std::shared_ptr<closure_energy::Battery> sharedBattery=nullptr,
          std::shared_ptr<closure_energy::PowerLedger> powerOwner=nullptr)
      :carry(carryBytes),home_(home),constraints_(constraints),recovery_(std::move(recovery)),powerW_(totalPowerW),
       initialJ_(batteryKwh*3600000),reserveJ_(initialJ_*reserveFraction),
       battery_(sharedBattery ? std::move(sharedBattery) : std::make_shared<closure_energy::Battery>(initialJ_)),
       powerOwner_(std::move(powerOwner)) {
    constraints_.validate();recovery_.validate();
    require(finite(powerW_)&&powerW_>0&&finite(batteryKwh)&&batteryKwh>0&&finite(reserveFraction)&&reserveFraction>=0&&reserveFraction<1,"Invalid finite vehicle energy contract");
    require(std::abs(battery_->Capacity()-initialJ_)<1e-5,"Shared battery capacity differs from vehicle contract");
    if(powerOwner_){require(powerOwner_->Budget()==battery_&&powerOwner_->LastTime()==0,
                           "Mission and elapsed-power owner must share the same initially unadvanced battery");
      powerOwner_->SetBasePower(0,powerW_);accountedBaseJ_=powerOwner_->Accounting().base_j;}
    auto first=automaticLoiter(home_);blocks_.push_back({0,first});
  }
  FiniteCarry carry;
  double remainingEnergyJ()const{return battery_->Remaining();}double usedEnergyJ()const{return battery_->Consumed();}
  double operationConsumedJ()const{return operationConsumedJ_;}double recoveryConsumedJ()const{return recoveryConsumedJ_;}
  std::shared_ptr<closure_energy::Battery> sharedBattery()const{return battery_;}
  double reserveEnergyJ()const{return reserveJ_;}double recoveryEnergyJ()const{return recovery_.vehicleEnergyJ;}
  double operationPowerW()const{return powerW_;}double nowS()const{return now_;}
  State state()const{return state_;}bool communicationsAvailable()const{return state_==State::Operating&&battery_->Remaining()>0;}
  std::string endStateReason()const {
    if(state_==State::Depleted)return "vehicle_battery_depleted";
    if(state_==State::ExternalRecoveryAccounting)return "external_recovery_handling_accounting_not_hydrodynamic_ascent";
    if(state_==State::RecoveryAccountingComplete)return "external_recovery_accounting_complete_not_field_certified";
    return "operating_or_submerged_rendezvous_recovery_not_simulated";
  }
  double nextDecisionTime()const{return blocks_.back().startS+blocks_.back().leg.durationS();}
  Pose evaluatedPose(double t)const {
    require(finite(t)&&t>=0,"Invalid trajectory query time");
    require(!powerOwner_||t<=now_,"Externally powered trajectory requires advance at the actual event time before querying; no speculative PHY state");
    if(t>now_){Mission predicted=predictionCopy();predicted.advance(t);return predicted.evaluatedPose(t);}
    if(t>=stoppedAtS_)return stoppedPose_;
    for(auto it=blocks_.rbegin();it!=blocks_.rend();++it)if(t>=it->startS)return it->leg.evaluatedPose(t-it->startS);
    return home_;
  }
  Vec3 evaluatedPosition(double t)const{return evaluatedPose(t).position;}
  Vec3 evaluatedVelocity(double t)const {
    require(!powerOwner_||t<=now_,"Advance external elapsed-power owner before querying velocity");
    if(t>now_){Mission predicted=predictionCopy();predicted.advance(t);return predicted.evaluatedVelocity(t);}
    if(t>=stoppedAtS_)return {0,0,0};
    for(auto it=blocks_.rbegin();it!=blocks_.rend();++it)if(t>=it->startS)return it->leg.tangent(t-it->startS)*constraints_.speedMps;
    return {0,0,0};
  }
  bool canStartLeg(const ContinuousLeg& leg,double completeSuffixEnergyJ)const {
    if(state_!=State::Operating||!leg.checked||!finite(completeSuffixEnergyJ)||completeSuffixEnergyJ<0)return false;
    return battery_->CanConsume(leg.durationS()*powerW_+completeSuffixEnergyJ+recovery_.vehicleEnergyJ,reserveJ_);
  }
  bool startLeg(const ContinuousLeg& leg,double now,double completeSuffixEnergyJ) {
    advance(now);if(!canStartLeg(leg,completeSuffixEnergyJ))return false;
    auto present=evaluatedPose(now_);
    require(distance(present.position,leg.start.position)<1e-5 && std::abs(angleDifference(present.yaw,leg.start.yaw))<1e-7,"New route does not join actual vehicle pose/tangent");
    // At a decision boundary a new block supersedes the automatic loiter; old
    // history remains for past transfer checks and carries no duplicate energy.
    if(!blocks_.empty()&&std::abs(blocks_.back().startS-now_)<1e-9)blocks_.pop_back();
    blocks_.push_back({now_,leg});return true;
  }
  void advance(double target) {
    require(finite(target)&&target>=now_,"Mission time cannot move backwards");
    if(powerOwner_){advanceExternallyPowered(target);return;}
    // External PHY debits use the same battery. Caller advances operation before
    // that event; this observes exhaustion without inventing another energy pool.
    if(state_==State::Operating&&battery_->Remaining()<=0){stoppedPose_=evaluatedPose(now_);stoppedAtS_=now_;state_=State::Depleted;}
    while(now_<target && state_==State::Operating) {
      double end=nextDecisionTime();
      if(end<=now_+1e-10){auto pose=blocks_.back().leg.end;blocks_.push_back({end,automaticLoiter(pose)});continue;}
      double dt=std::min(target-now_,end-now_),actual=battery_->Consume(dt*powerW_);
      double used=actual/powerW_;operationConsumedJ_+=actual;now_+=used;
      if(used+1e-10<dt || battery_->Remaining()<=1e-8){stoppedPose_=blocks_.back().leg.evaluatedPose(now_-blocks_.back().startS);stoppedAtS_=now_;state_=State::Depleted;break;}
    }
    if(state_==State::ExternalRecoveryAccounting && now_<target) {
      double p=recoveryPowerW_,dt=std::min(target-now_,recoveryEndsS_-now_);
      double actual=battery_->Consume(dt*p),used=actual/p;recoveryConsumedJ_+=actual;now_+=used;
      if(used+1e-10<dt||battery_->Remaining()<=1e-8){state_=State::Depleted;}
      else if(now_>=recoveryEndsS_-1e-9)state_=State::RecoveryAccountingComplete;
    }
    now_=target;
  }
  bool debitCommunication(double now,double joules) {
    require(!powerOwner_,"Additional PHY loads must use the authoritative elapsed-power owner, not an instantaneous second debit");
    require(finite(joules)&&joules>=0,"Invalid additional communication energy");advance(now);
    if(state_!=State::Operating)return false;
    bool enough=battery_->CanConsume(joules);battery_->Consume(joules);
    if(battery_->Remaining()<=1e-8){stoppedPose_=evaluatedPose(now_);stoppedAtS_=now_;state_=State::Depleted;}
    return enough;
  }
  // Settled custody can include an explicitly counted deadline/retry terminal
  // loss. It never means all collected data were successfully surface-delivered.
  bool beginExternalRecoveryAccounting(double now,bool allOnboardCustodySettled,
                                      double exactScheduledEndS=std::numeric_limits<double>::quiet_NaN()) {
    advance(now);if(state_!=State::Operating||!allOnboardCustodySettled)return false;
    Pose present=evaluatedPose(now_);
    if(distance(present.position,home_.position)>1e-5 || !battery_->CanConsume(recovery_.vehicleEnergyJ,reserveJ_))return false;
    const double declaredEnd=now_+recovery_.durationS;
    recoveryEndsS_=std::isnan(exactScheduledEndS)?declaredEnd:exactScheduledEndS;
    require(finite(recoveryEndsS_)&&recoveryEndsS_>now_&&std::abs(recoveryEndsS_-declaredEnd)<=1e-9,
            "Scheduled recovery endpoint may differ only by at most one nanosecond of time representation");
    // The simulator supplies its exact quantized event endpoint. Integrating
    // this power over that interval yields exactly the declared handling J.
    recoveryPowerW_=recovery_.vehicleEnergyJ/(recoveryEndsS_-now_);
    stoppedPose_=present;stoppedAtS_=now_;state_=State::ExternalRecoveryAccounting;
    if(powerOwner_)powerOwner_->SetBasePower(now_,recoveryPowerW_);
    return true;
  }
 private:
  void advanceExternallyPowered(double target) {
    // One ledger integrates operation + radio + optical loads concurrently.
    // In particular, a long base-only debit cannot steal energy from an
    // overlapping PHY interval and shift the actual depletion position.
    double endpoint=target;
    if(state_==State::ExternalRecoveryAccounting)endpoint=std::min(target,recoveryEndsS_);
    if(powerOwner_->LastTime()>endpoint){
      require(powerOwner_->LastTime()-endpoint<=1e-9&&powerOwner_->LastTime()<=target,
              "Power owner advanced beyond a mission base-state boundary; schedule exact recovery completion");
      endpoint=powerOwner_->LastTime(); // numerical coincidence only, never rewind an elapsed ledger
    }
    powerOwner_->Advance(endpoint);
    const double baseDelta=std::max(0.0,powerOwner_->Accounting().base_j-accountedBaseJ_);
    accountedBaseJ_=powerOwner_->Accounting().base_j;
    if(state_==State::Operating)operationConsumedJ_+=baseDelta;
    else if(state_==State::ExternalRecoveryAccounting)recoveryConsumedJ_+=baseDelta;
    double aliveUntil=std::min(endpoint,powerOwner_->DepletedAt());
    if(state_==State::Operating){
      while(now_<aliveUntil){double end=nextDecisionTime();
        if(end<=now_+1e-10){auto pose=blocks_.back().leg.end;blocks_.push_back({end,automaticLoiter(pose)});continue;}
        now_=std::min(aliveUntil,end);}
      if(powerOwner_->DepletedAt()<=endpoint){
        const double stop=powerOwner_->DepletedAt();
        for(auto it=blocks_.rbegin();it!=blocks_.rend();++it)if(stop>=it->startS){stoppedPose_=it->leg.evaluatedPose(stop-it->startS);break;}
        stoppedAtS_=stop;state_=State::Depleted;}
    }else if(state_==State::ExternalRecoveryAccounting){
      if(powerOwner_->DepletedAt()<=endpoint)state_=State::Depleted;
      else if(endpoint>=recoveryEndsS_-1e-9){state_=State::RecoveryAccountingComplete;powerOwner_->SetBasePower(endpoint,0);}
    }
    now_=target;
    if(target>endpoint)powerOwner_->Advance(target);
  }
  Mission predictionCopy()const {
    Mission copy=*this;copy.battery_=std::make_shared<closure_energy::Battery>(battery_->Capacity());
    copy.battery_->Consume(battery_->Consumed());return copy;
  }
  ContinuousLeg automaticLoiter(Pose pose)const {
    try{return makeLoiter("automatic-powered-loiter",pose,constraints_,1,'L');}
    catch(const std::invalid_argument&){return makeLoiter("automatic-powered-loiter",pose,constraints_,1,'R');}
  }
  Pose home_,stoppedPose_;RouteConstraints constraints_;RecoveryAllowance recovery_;
  double powerW_,initialJ_,reserveJ_,now_=0,stoppedAtS_=std::numeric_limits<double>::infinity(),recoveryEndsS_=0,recoveryPowerW_=0;
  double operationConsumedJ_=0,recoveryConsumedJ_=0;
  std::shared_ptr<closure_energy::Battery> battery_;
  std::shared_ptr<closure_energy::PowerLedger> powerOwner_;
  double accountedBaseJ_=0;
  State state_=State::Operating;std::vector<Block> blocks_;
};

// Causal finite-catalogue selector. Only received manifests are inputs; the
// network adapter must charge their transport, delay and staleness. In particular
// do NOT populate reports by looking into remote live gateway queues. Nodes can
// encode east/west approach poses; pickupStation is the canonical station ID.
struct ObservedWindow {
  std::string id,station;
  std::uint64_t totalBytes=0,availableBytes=0;
  double generatedAt=0,deadlineAt=0;
  bool complete()const{return totalBytes>0&&availableBytes==totalBytes;}
};
struct StationReport {std::string station;double observedAt=0;std::vector<ObservedWindow> windows;};
struct RevisitRequirement {std::string station;double lastVisitAt=0,maxIntervalS=0;};
struct PlannerInput {
  std::string currentNode,returnNode;
  Pose currentPose;
  double nowS=0,assessmentHorizonS=0,reportMaxAgeS=0;
  double remainingEnergyJ=0,reserveEnergyJ=0,recoveryEnergyJ=0,recoveryDurationS=0,operationPowerW=67000.0/65.0;
  std::uint64_t carryCapacityBytes=0,otherResidentBytes=0;
  unsigned maxPickups=2;
  std::vector<ObservedWindow> onboardCompleteWindows;
  std::vector<StationReport> reports;
  std::vector<RevisitRequirement> revisitRequirements;
};
struct RouteAction {
  std::string id,fromNode,toNode,pickupStation;
  ContinuousLeg leg;
  bool isOffload=false;
  std::uint64_t payloadCapacityBytes=0;
  double extraEnergyJ=0;
};
struct CandidatePlan {
  std::vector<std::string> actionIds,visitedStations,pickedWindowIds,onTimeWindowIds,projectedMissedWindowIds;
  double durationS=0,energyJ=0,offloadAt=-1,returnAt=0,recoveryAccountingFinishAt=0;
  std::uint64_t peakCarryBytes=0,onTimeBytes=0;
};
struct PlannerResult {
  std::optional<CandidatePlan> chosen;
  std::vector<CandidatePlan> feasible;
  std::vector<std::pair<std::vector<std::string>,std::string>> rejected;
  std::vector<std::string> ignoredStaleStations;
  std::size_t enumerated=0;
};
inline bool planBetter(const CandidatePlan& a,const CandidatePlan& b) {
  if(a.projectedMissedWindowIds.size()!=b.projectedMissedWindowIds.size())return a.projectedMissedWindowIds.size()<b.projectedMissedWindowIds.size();
  long double left=a.durationS>0?static_cast<long double>(a.onTimeBytes)/a.durationS:0;
  long double right=b.durationS>0?static_cast<long double>(b.onTimeBytes)/b.durationS:0;
  if(left!=right)return left>right;
  if(a.durationS!=b.durationS)return a.durationS<b.durationS;
  return a.actionIds<b.actionIds;
}

inline PlannerResult planRoutes(const PlannerInput& input,const std::vector<RouteAction>& suppliedActions) {
  require(!input.currentNode.empty()&&!input.returnNode.empty(),"Planner node IDs required");
  require(finite(input.nowS)&&input.nowS>=0&&finite(input.assessmentHorizonS)&&input.assessmentHorizonS>input.nowS
          &&finite(input.reportMaxAgeS)&&input.reportMaxAgeS>=0,"Invalid common planning horizon/report age");
  require(finite(input.remainingEnergyJ)&&input.remainingEnergyJ>=0&&finite(input.reserveEnergyJ)&&input.reserveEnergyJ>=0
          &&finite(input.recoveryEnergyJ)&&input.recoveryEnergyJ>0&&finite(input.recoveryDurationS)&&input.recoveryDurationS>0
          &&finite(input.operationPowerW)&&input.operationPowerW>0,
          "Invalid finite planner energy/recovery contract");
  require(input.maxPickups<=2,"Catalogue allows at most two pickups");
  auto validateWindow=[&](const ObservedWindow& w,double observedAt){
    require(!w.id.empty()&&!w.station.empty()&&w.totalBytes>0&&w.availableBytes<=w.totalBytes,"Invalid observed complete-window descriptor");
    require(finite(w.generatedAt)&&w.generatedAt>=0&&w.generatedAt<=observedAt&&finite(w.deadlineAt)&&w.deadlineAt>=w.generatedAt,
            "A manifest cannot contain future-generated or invalid windows");
  };
  std::map<std::string,ObservedWindow> known,onboard,identities;
  std::uint64_t initialUsed=input.otherResidentBytes;
  require(initialUsed<=input.carryCapacityBytes,"Other carry occupancy exceeds capacity");
  for(const auto& w:input.onboardCompleteWindows){validateWindow(w,input.nowS);require(w.complete(),"Onboard descriptors must be complete windows");
    require(onboard.emplace(w.id,w).second,"Duplicate onboard window ID");
    require(w.totalBytes<=input.carryCapacityBytes-initialUsed,"Finite initial carry exceeded");initialUsed+=w.totalBytes;identities[w.id]=w;
  }
  std::map<std::string,StationReport> fresh;std::set<std::string> reportStations,manifestIds;
  PlannerResult result;
  for(const auto& report:input.reports){
    require(!report.station.empty()&&reportStations.insert(report.station).second,"Duplicate/empty station report");
    require(finite(report.observedAt)&&report.observedAt>=0&&report.observedAt<=input.nowS,"Future report forbidden");
    for(const auto& w:report.windows){validateWindow(w,report.observedAt);require(w.station==report.station&&manifestIds.insert(w.id).second,"Invalid station/window identity");
      auto prior=identities.find(w.id);
      if(prior!=identities.end())require(prior->second.station==w.station&&prior->second.totalBytes==w.totalBytes
          &&prior->second.generatedAt==w.generatedAt&&prior->second.deadlineAt==w.deadlineAt,"Conflicting immutable window identity");
      identities[w.id]=w;
    }
    if(input.nowS-report.observedAt<=input.reportMaxAgeS){fresh[report.station]=report;for(const auto& w:report.windows)if(w.complete())known[w.id]=w;}
    else result.ignoredStaleStations.push_back(report.station);
  }
  for(const auto& p:onboard)known[p.first]=p.second;
  std::set<std::string> due,requiredStations;
  for(const auto& p:known)if(p.second.deadlineAt<=input.assessmentHorizonS)due.insert(p.first);
  for(const auto& r:input.revisitRequirements){
    require(!r.station.empty()&&requiredStations.insert(r.station).second&&finite(r.lastVisitAt)&&r.lastVisitAt>=0&&r.lastVisitAt<=input.nowS
            &&finite(r.maxIntervalS)&&r.maxIntervalS>0,"Invalid revisit requirement");
  }
  std::vector<RouteAction> actions=suppliedActions;
  std::sort(actions.begin(),actions.end(),[](const auto& a,const auto& b){return a.id<b.id;});
  std::set<std::string> ids;
  for(const auto& a:actions){require(!a.id.empty()&&ids.insert(a.id).second&&!a.fromNode.empty()&&!a.toNode.empty(),"Duplicate/invalid route action");
    require(finite(a.extraEnergyJ)&&a.extraEnergyJ>=0,"Invalid action energy");
    require(a.pickupStation.empty()||!a.isOffload,"Separate pickup and offload actions required");
  }
  using Route=std::vector<std::size_t>;
  std::map<std::vector<std::string>,Route> catalogue;
  auto insert=[&](const Route& route){std::vector<std::string> key;for(auto i:route)key.push_back(actions[i].id);catalogue[key]=route;};
  auto offloadSuffix=[&](const Route& prefix,const std::string& node){
    for(std::size_t i=0;i<actions.size();++i)if(actions[i].fromNode==node&&actions[i].isOffload){
      auto route=prefix;route.push_back(i);
      if(actions[i].toNode==input.returnNode)insert(route);
      else for(std::size_t j=0;j<actions.size();++j)if(actions[j].fromNode==actions[i].toNode&&actions[j].toNode==input.returnNode
                                                    &&actions[j].pickupStation.empty()){
        auto home=route;home.push_back(j);insert(home);
      }
    }
  };
  if(input.currentNode==input.returnNode)insert({});
  for(std::size_t i=0;i<actions.size();++i)if(actions[i].fromNode==input.currentNode&&actions[i].toNode==input.returnNode&&actions[i].pickupStation.empty())insert({i});
  offloadSuffix({},input.currentNode);
  for(std::size_t i=0;i<actions.size()&&input.maxPickups>0;++i)if(actions[i].fromNode==input.currentNode&&!actions[i].pickupStation.empty()){
    offloadSuffix({i},actions[i].toNode);
    if(input.maxPickups==2)for(std::size_t j=0;j<actions.size();++j)if(actions[j].fromNode==actions[i].toNode&&!actions[j].pickupStation.empty()
                                                                   &&actions[j].pickupStation!=actions[i].pickupStation)offloadSuffix({i,j},actions[j].toNode);
  }
  result.enumerated=catalogue.size();
  auto evaluate=[&](const Route& route,CandidatePlan& plan)->std::string {
    double now=input.nowS,energy=0,projectedOffload=std::numeric_limits<double>::infinity(),scan=now;
    for(auto i:route){scan+=actions[i].leg.durationS();if(actions[i].isOffload){projectedOffload=scan;break;}}
    auto carried=onboard;std::set<std::string> seen;for(const auto& p:carried)seen.insert(p.first);
    std::uint64_t used=initialUsed;plan.peakCarryBytes=used;
    std::map<std::string,double> visited;std::map<std::string,ObservedWindow> useful;
    Pose pose=input.currentPose;std::string node=input.currentNode;
    for(auto i:route){const auto& a=actions[i];plan.actionIds.push_back(a.id);
      if(!a.leg.checked)return "unchecked_geometry";
      if(a.fromNode!=node||distance(pose.position,a.leg.start.position)>1e-5||std::abs(angleDifference(pose.yaw,a.leg.start.yaw))>1e-7)return "pose_or_tangent_discontinuity";
      if(std::abs(a.leg.constraints.speedMps-fixedThreeKnotsMps)>1e-12||!a.leg.constraints.calmWater)return "unsupported_speed_or_current";
      now+=a.leg.durationS();energy+=a.leg.durationS()*input.operationPowerW+a.extraEnergyJ;
      if(now>input.assessmentHorizonS+1e-9)return "common_horizon_exceeded";
      if(energy+input.recoveryEnergyJ+input.reserveEnergyJ>input.remainingEnergyJ+1e-7)return "vehicle_return_recovery_reserve_exceeded";
      node=a.toNode;pose=a.leg.end;
      if(!a.pickupStation.empty()){
        visited[a.pickupStation]=now;plan.visitedStations.push_back(a.pickupStation);
        auto report=fresh.find(a.pickupStation);std::uint64_t allowance=a.payloadCapacityBytes;
        if(report!=fresh.end()){
          auto windows=report->second.windows;std::sort(windows.begin(),windows.end(),[](const auto& x,const auto& y){return std::tie(x.deadlineAt,x.id)<std::tie(y.deadlineAt,y.id);});
          for(const auto& w:windows){
            if(!w.complete()||seen.count(w.id)||w.deadlineAt<projectedOffload||w.totalBytes>allowance||w.totalBytes>input.carryCapacityBytes-used)continue;
            carried[w.id]=w;seen.insert(w.id);allowance-=w.totalBytes;used+=w.totalBytes;plan.peakCarryBytes=std::max(plan.peakCarryBytes,used);plan.pickedWindowIds.push_back(w.id);
          }
        }
      }
      if(a.isOffload){std::uint64_t bytes=0;for(const auto& p:carried){if(p.second.totalBytes>a.payloadCapacityBytes-bytes)return "offload_capacity_exceeded";bytes+=p.second.totalBytes;}
        for(const auto& p:carried)if(now<=p.second.deadlineAt)useful[p.first]=p.second;
        carried.clear();used=input.otherResidentBytes;plan.offloadAt=now;
      }
    }
    if(node!=input.returnNode)return "missing_return";
    if(now+input.recoveryDurationS>input.assessmentHorizonS+1e-9)return "recovery_handling_horizon_exceeded";
    if(energy+input.recoveryEnergyJ+input.reserveEnergyJ>input.remainingEnergyJ+1e-7)return "vehicle_return_recovery_reserve_exceeded";
    for(const auto& r:input.revisitRequirements){double deadline=r.lastVisitAt+r.maxIntervalS;auto seenVisit=visited.find(r.station);
      if(seenVisit!=visited.end()&&seenVisit->second>deadline)return "revisit_late";
      if(deadline<=input.assessmentHorizonS&&seenVisit==visited.end())return "due_revisit_unserved";
      if(seenVisit!=visited.end()&&now-seenVisit->second>r.maxIntervalS)return "revisit_expired_before_return";
    }
    for(const auto& p:useful){plan.onTimeWindowIds.push_back(p.first);plan.onTimeBytes+=p.second.totalBytes;}
    for(const auto& id:due)if(!useful.count(id))plan.projectedMissedWindowIds.push_back(id);
    plan.energyJ=energy;plan.returnAt=now;plan.recoveryAccountingFinishAt=now+input.recoveryDurationS;
    plan.durationS=now-input.nowS;return {};
  };
  for(const auto& entry:catalogue){CandidatePlan plan;auto failure=evaluate(entry.second,plan);
    if(failure.empty())result.feasible.push_back(std::move(plan));else result.rejected.emplace_back(entry.first,failure);}
  std::sort(result.feasible.begin(),result.feasible.end(),planBetter);
  std::sort(result.ignoredStaleStations.begin(),result.ignoredStaleStations.end());
  if(!result.feasible.empty())result.chosen=result.feasible.front();
  return result;
}

// The selector is exact only over this supplied finite route catalogue with its
// fixed EDF whole-window packing and snapshot prediction. It is not a knapsack
// solver, stochastic optimum, or reproduction of another paper's RL algorithm.

} // namespace closure_mission
#endif
