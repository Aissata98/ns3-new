#ifndef EDC_CLOSURE_OPTICAL_H
#define EDC_CLOSURE_OPTICAL_H

// Explicit, abstract optical service contract, not a BlueComm firmware model.
// Acoustic wake messages are transported by ns-3. Optical descriptor/DATA and
// reverse confirmation are separate, timed, powered service frames.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace closure_optical {
inline void require(bool ok, const char* why) { if (!ok) throw std::invalid_argument(why); }
inline bool positive(double v) { return std::isfinite(v) && v > 0; }
struct Config {
  double bitRateBps=0, startupS=0, startupPowerW=0, listenWindowS=0;
  double turnaroundS=0, ackWaitS=0, retryBackoffS=0, pollIntervalS=0;
  double txPowerW=0, rxPowerW=0, supplyEfficiency=0, frameSuccessProbability=-1;
  double contactRangeM=0, maximumSpeedMps=0;
  std::uint32_t maxAttempts=0, maxPayloadBytes=0;
  void validate() const {
    require(positive(bitRateBps)&&positive(startupS)&&positive(startupPowerW), "Explicit optical bitrate/startup/consumption required");
    require(positive(listenWindowS)&&listenWindowS>startupS, "Optical receive window must include startup");
    require(positive(turnaroundS)&&positive(ackWaitS)&&positive(retryBackoffS)&&positive(pollIntervalS), "Explicit positive optical transaction timers required");
    require(positive(txPowerW)&&positive(rxPowerW)&&positive(supplyEfficiency)&&supplyEfficiency<=1, "Optical powers and conversion efficiency required");
    require(std::isfinite(frameSuccessProbability)&&frameSuccessProbability>=0&&frameSuccessProbability<=1, "Explicit optical frame-success scenario required");
    require(positive(contactRangeM)&&positive(maximumSpeedMps)&&maxAttempts>0&&maxAttempts<=255&&maxPayloadBytes>0, "Finite optical contact and transaction bounds required");
  }
  double wireDuration(std::uint64_t bytes) const {
    require(positive(bitRateBps)&&bytes>0, "Positive optical service frame required");
    return static_cast<double>(bytes)*8.0/bitRateBps;
  }
  double propagationBoundS() const { return contactRangeM/(299792458.0/1.34); }
};

inline void put(std::vector<std::uint8_t>& out, std::uint64_t value, unsigned width) {
  require(width>0&&width<=8, "Invalid field width");
  if(width<8) require(value<(std::uint64_t{1}<<(8*width)), "Optical field overflow");
  for(unsigned n=width;n>0;--n) out.push_back(static_cast<std::uint8_t>(value>>(8*(n-1))));
}
inline std::uint64_t get(const std::vector<std::uint8_t>& bytes, std::size_t& offset, unsigned width) {
  require(width>0&&width<=8&&offset<=bytes.size()&&width<=bytes.size()-offset, "Truncated optical field");
  std::uint64_t value=0; for(unsigned n=0;n<width;++n) value=(value<<8)|bytes[offset++]; return value;
}
inline std::uint32_t crc(const std::vector<std::uint8_t>& bytes, std::size_t count) {
  std::uint32_t value=0xffffffffu;
  for(std::size_t i=0;i<count;++i) {
    value^=bytes[i];
    for(unsigned b=0;b<8;++b) value=(value>>1)^(0xedb88320u&static_cast<std::uint32_t>(-static_cast<std::int32_t>(value&1u)));
  }
  return ~value;
}
inline void seal(std::vector<std::uint8_t>& bytes) { const auto value=crc(bytes,bytes.size()); put(bytes,value,4); }
inline void checkSeal(const std::vector<std::uint8_t>& bytes, std::size_t expected) {
  require(bytes.size()==expected&&expected>=4, "Wrong optical descriptor size");
  std::size_t pos=expected-4; require(get(bytes,pos,4)==crc(bytes,expected-4), "Corrupt optical descriptor");
}
struct Wake {
  std::uint64_t readyAtNs=0, listenUntilNs=0, creditBytes=0;
  void validate() const { require(readyAtNs<listenUntilNs&&creditBytes>0, "Finite receiver window and positive advertised credit required"); }
  std::vector<std::uint8_t> encode() const {
    validate(); std::vector<std::uint8_t> out; put(out,readyAtNs,8);put(out,listenUntilNs,8);put(out,creditBytes,8);return out;
  }
  static Wake decode(const std::vector<std::uint8_t>& bytes) {
    require(bytes.size()==24, "Wrong acoustic optical-wake payload length"); std::size_t p=0;
    Wake w{get(bytes,p,8),get(bytes,p,8),get(bytes,p,8)};w.validate();return w;
  }
};
struct Descriptor {
  std::uint64_t nonce=0, packetId=0, createdAtNs=0, windowId=0, windowBytes=0;
  std::uint32_t gateway=0, source=0, payloadBytes=0, chunkIndex=0, chunkCount=0;
  std::uint8_t priority=0;
  static constexpr std::uint32_t wireBytes=69;
  void validate() const {
    require(nonce>0&&packetId>0&&payloadBytes>0&&priority<=2, "Invalid optical record identity");
    require((windowId==0&&windowBytes==0&&chunkIndex==0&&chunkCount==0)||
            (windowId>0&&windowBytes>=payloadBytes&&chunkCount>0&&chunkIndex<chunkCount), "Inconsistent optical window descriptor");
  }
  std::vector<std::uint8_t> encode() const {
    validate();std::vector<std::uint8_t> out;
    put(out,0x454f,2);put(out,1,1);put(out,1,1);put(out,nonce,8);put(out,gateway,4);
    put(out,packetId,8);put(out,createdAtNs,8);put(out,source,4);put(out,priority,1);put(out,payloadBytes,4);
    put(out,windowId,8);put(out,windowBytes,8);put(out,chunkIndex,4);put(out,chunkCount,4);seal(out);
    require(out.size()==wireBytes, "Optical wire descriptor accounting error");return out;
  }
  static Descriptor decode(const std::vector<std::uint8_t>& bytes) {
    checkSeal(bytes,wireBytes);std::size_t p=0;
    require(get(bytes,p,2)==0x454f&&get(bytes,p,1)==1&&get(bytes,p,1)==1,"Unsupported optical DATA descriptor");
    Descriptor d;d.nonce=get(bytes,p,8);d.gateway=get(bytes,p,4);d.packetId=get(bytes,p,8);d.createdAtNs=get(bytes,p,8);
    d.source=get(bytes,p,4);d.priority=get(bytes,p,1);d.payloadBytes=get(bytes,p,4);d.windowId=get(bytes,p,8);
    d.windowBytes=get(bytes,p,8);d.chunkIndex=get(bytes,p,4);d.chunkCount=get(bytes,p,4);d.validate();return d;
  }
};
struct Ack {
  std::uint64_t nonce=0, packetId=0;std::uint32_t gateway=0;
  static constexpr std::uint32_t wireBytes=28;
  std::vector<std::uint8_t> encode() const {
    require(nonce>0&&packetId>0,"Invalid optical ACK identity");std::vector<std::uint8_t> out;
    put(out,0x454f,2);put(out,1,1);put(out,2,1);put(out,nonce,8);put(out,packetId,8);put(out,gateway,4);seal(out);return out;
  }
  static Ack decode(const std::vector<std::uint8_t>& bytes) {
    checkSeal(bytes,wireBytes);std::size_t p=0;
    require(get(bytes,p,2)==0x454f&&get(bytes,p,1)==1&&get(bytes,p,1)==2,"Unsupported optical ACK");
    Ack a{get(bytes,p,8),get(bytes,p,8),static_cast<std::uint32_t>(get(bytes,p,4))};require(a.nonce>0&&a.packetId>0,"Invalid optical ACK identity");return a;
  }
};
inline bool continuousContact(double startDistance, double endDistance, double duration, double range, double speed) {
  require(std::isfinite(startDistance)&&startDistance>=0&&std::isfinite(endDistance)&&endDistance>=0&&
          std::isfinite(duration)&&duration>=0&&positive(range)&&positive(speed), "Invalid optical contact interval");
  return startDistance<=range&&endDistance<=range&&
      (startDistance+endDistance+speed*duration)/2.0<=range+1e-9;
}
} // namespace closure_optical
#endif
