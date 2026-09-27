#ifndef EDC_CLOSURE_ENERGY_H
#define EDC_CLOSURE_ENERGY_H

// Event-time energy bookkeeping, not a fitted battery chemistry model.
// One shared Battery owns each device's energy. Concurrent receive waveforms
// occupy one receiver state, not several independent receivers. Explicit extra
// loads represent physically separate powered components. All powers are inputs.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

namespace closure_energy {
inline void Nonnegative(double value) {
  if (!std::isfinite(value) || value < 0) throw std::invalid_argument("nonfinite or negative energy input");
}
class Battery {
 public:
  explicit Battery(double capacity_j) : capacity_(capacity_j), remaining_(capacity_j) {
    Nonnegative(capacity_j);
    if (capacity_j == 0) throw std::invalid_argument("battery capacity must be positive");
  }
  double Capacity() const { return capacity_; }
  double Remaining() const { return remaining_; }
  double Consumed() const { return capacity_ - remaining_; }
  double Consume(double requested_j) {
    Nonnegative(requested_j);
    const double actual = std::min(requested_j, remaining_);
    remaining_ -= actual;
    return actual;
  }
  bool CanConsume(double joules, double reserve_j = 0) const {
    Nonnegative(joules); Nonnegative(reserve_j);
    return reserve_j <= remaining_ && joules <= remaining_ - reserve_j;
  }
 private:
  double capacity_, remaining_;
};

struct Powers {
  double tx_w = 0, rx_w = 0, idle_w = 0, sleep_w = 0, base_w = 0;
};
struct Totals {
  double tx_j = 0, rx_j = 0, idle_j = 0, sleep_j = 0, base_j = 0, extra_j = 0;
  double tx_s = 0, rx_s = 0, idle_s = 0, sleep_s = 0, depleted_s = 0;
  double TotalJ() const { return tx_j + rx_j + idle_j + sleep_j + base_j + extra_j; }
  double TotalS() const { return tx_s + rx_s + idle_s + sleep_s + depleted_s; }
};
class PowerLedger {
 public:
  PowerLedger(std::shared_ptr<Battery> battery, Powers powers, double horizon_s)
      : battery_(std::move(battery)), powers_(powers), horizon_(horizon_s) {
    if (!battery_) throw std::invalid_argument("shared battery is required");
    Nonnegative(horizon_s);
    for (double value : {powers.tx_w, powers.rx_w, powers.idle_w, powers.sleep_w, powers.base_w}) Nonnegative(value);
  }
  std::shared_ptr<Battery> Budget() const { return battery_; }
  const Totals& Accounting() const { return totals_; }
  double LastTime() const { return last_; }
  double DepletedAt() const { return depleted_at_; }
  bool Awake() const { return awake_; }
  bool TxActive(double now) const { return now < tx_end_; }
  bool RxActive(double now) const { return now < rx_end_; }

  void Advance(double requested_time) {
    Nonnegative(requested_time);
    const double time = std::min(requested_time, horizon_);
    if (time < last_) throw std::invalid_argument("energy time moved backwards");
    while (last_ < time) {
      Prune();
      double end = time;
      if (tx_end_ > last_) end = std::min(end, tx_end_);
      if (rx_end_ > last_) end = std::min(end, rx_end_);
      double extra_power = 0;
      for (const auto& item : loads_) {
        end = std::min(end, item.second.end);
        extra_power += item.second.power;
      }
      const double dt = end - last_;
      if (dt <= 0) throw std::logic_error("nonprogressing energy interval");
      const int state = !awake_ ? 3 : TxActive(last_) ? 0 : RxActive(last_) ? 1 : 2;
      const double radio_power = state == 0 ? powers_.tx_w : state == 1 ? powers_.rx_w : state == 2 ? powers_.idle_w : powers_.sleep_w;
      const double sum = radio_power + powers_.base_w + extra_power;
      if (!std::isfinite(sum * dt)) throw std::overflow_error("energy integration overflow");
      const double powered_dt = battery_->Remaining() <= 0 ? 0 : sum > 0 ? std::min(dt, battery_->Remaining() / sum) : dt;
      const double spent = battery_->Consume(sum * powered_dt);
      // Dividing a rounded product may exceed the interval by one ulp. Keep
      // every state duration nonnegative without adding time or free energy.
      const double measured_dt = std::min(powered_dt, sum > 0 ? spent / sum : powered_dt);
      const double radio_j = radio_power * measured_dt;
      if (state == 0) { totals_.tx_j += radio_j; totals_.tx_s += measured_dt; }
      else if (state == 1) { totals_.rx_j += radio_j; totals_.rx_s += measured_dt; }
      else if (state == 2) { totals_.idle_j += radio_j; totals_.idle_s += measured_dt; }
      else { totals_.sleep_j += radio_j; totals_.sleep_s += measured_dt; }
      totals_.base_j += powers_.base_w * measured_dt;
      totals_.extra_j += extra_power * measured_dt;
      totals_.depleted_s += dt - measured_dt;
      if (battery_->Remaining() <= 0 && !std::isfinite(depleted_at_)) depleted_at_ = last_ + measured_dt;
      last_ = end;
    }
    Prune();
  }

  bool BeginTx(double now, double duration_s) {
    Duration(duration_s); Advance(now);
    if (now >= horizon_ || !awake_ || battery_->Remaining() <= 0 || TxActive(now) || RxActive(now)) return false;
    tx_end_ = std::min(horizon_, now + duration_s);
    return true;
  }
  bool BeginRx(double now, double duration_s) {
    Duration(duration_s); Advance(now);
    if (now >= horizon_ || !awake_ || battery_->Remaining() <= 0 || TxActive(now)) return false;
    rx_end_ = std::min(horizon_, std::max(rx_end_, now + duration_s));
    return true;
  }
  void SetAwake(double now, bool awake) {
    Advance(now); awake_ = awake;
    if (!awake) tx_end_ = rx_end_ = std::min(now, horizon_);
  }
  void SetBasePower(double now, double watts) { Nonnegative(watts); Advance(now); powers_.base_w = watts; }
  std::uint64_t AddLoad(double now, double duration_s, double watts) {
    Duration(duration_s); Nonnegative(watts); Advance(now);
    if (now >= horizon_) return 0;
    const auto id = next_load_++;
    if (id == 0) throw std::overflow_error("load identity overflow");
    loads_.emplace(id, Load{std::min(horizon_, now + duration_s), watts});
    return id;
  }
  void StopLoad(double now, std::uint64_t id) { Advance(now); loads_.erase(id); }
  double NextBoundary(double now) const {
    double end = horizon_;
    if (tx_end_ > now) end = std::min(end, tx_end_);
    if (rx_end_ > now) end = std::min(end, rx_end_);
    double extra_power = 0;
    for (const auto& item : loads_) {
      if (item.second.end > now) end = std::min(end, item.second.end);
      extra_power += item.second.power;
    }
    const double radio = !awake_ ? powers_.sleep_w : TxActive(now) ? powers_.tx_w : RxActive(now) ? powers_.rx_w : powers_.idle_w;
    const double watts = powers_.base_w + radio + extra_power;
    if (watts > 0 && battery_->Remaining() > 0) end = std::min(end, now + battery_->Remaining() / watts);
    return end;
  }
 private:
  static void Duration(double value) { Nonnegative(value); if (value == 0) throw std::invalid_argument("duration must be positive"); }
  void Prune() { for (auto it = loads_.begin(); it != loads_.end();) { if (it->second.end <= last_) it = loads_.erase(it); else ++it; } }
  struct Load { double end, power; };
  std::shared_ptr<Battery> battery_;
  Powers powers_;
  double horizon_, last_ = 0, tx_end_ = 0, rx_end_ = 0;
  double depleted_at_ = std::numeric_limits<double>::infinity();
  bool awake_ = true;
  std::uint64_t next_load_ = 1;
  std::map<std::uint64_t, Load> loads_;
  Totals totals_;
};
} // namespace closure_energy
#endif
