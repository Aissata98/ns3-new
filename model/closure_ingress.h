#ifndef CLOSURE_INGRESS_H
#define CLOSURE_INGRESS_H

// Finite, event-driven STORAGE MODEL, not a device firmware implementation.
// One Link represents one dedicated source/logger -> gateway cable. The main
// simulator owns the all-priority gateway queue. Gateway bytes below mirror its
// raw subset and are NOT an additional payload store. Source records, including
// their declared metadata, stay in a finite archive after admission ACK.
// RAM counters model whole-chunk staging, not the archive or all firmware RAM.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace closure_ingress {

enum class Priority : std::uint8_t { High = 0, Medium = 1, Low = 2 };
enum class Removal { Forwarded, Dropped };
enum class SubmitResult { Accepted, ArchiveFull, SourceUnavailable, Duplicate };
enum class Custody { Source, Gateway, Forwarded, Dropped };

struct Record {
  std::uint64_t id = 0;
  // Strictly increasing per source, assigned on original generation, not retry.
  // This watermark bounds historical duplicate suppression without an unlimited
  // tombstone set. Original packet/window IDs are preserved independently.
  std::uint64_t producer_sequence = 0;
  std::string window_id;
  std::uint32_t payload_bytes = 0;
  Priority priority = Priority::Low;
  double acquisition_start_s = 0;
  double release_s = 0;
  // Producer-known description of one completed raw window. These four fields
  // occupy 24 bytes within the declared persistent metadata allocation. Zero
  // for HIGH/MEDIUM and legacy callers; chunk index is zero-based.
  std::uint64_t windowNumericId = 0, windowPayloadBytes = 0;
  std::uint32_t windowChunkIndex = 0, windowChunkCount = 0;
  // -1 preserves the historical release==ingress-enqueue contract. Relayed
  // H/M keep their original production clock and become locally available only
  // after complete acoustic reception at the paired bottom endpoint.
  double source_ready_s = -1;
  std::uint32_t source_node = std::numeric_limits<std::uint32_t>::max();
};

struct Config {
  std::uint64_t source_archive_capacity_bytes = 0;
  std::uint64_t gateway_outbox_capacity_bytes = 16ULL * 1024 * 1024;
  std::uint64_t source_staging_ram_capacity_bytes = 0;
  std::uint64_t gateway_staging_ram_capacity_bytes = 0;
  std::uint32_t record_overhead_bytes = 64; // Study allocation, not measured sizeof.
  double bit_rate_bps = 0;
  double wire_bits_per_byte = 10; // Explicit study framing; e.g. 8N1.
  std::uint32_t data_header_bytes = 0;
  std::uint32_t ack_wire_bytes = 16; // Positive serialized ACK/NACK/credit size.
  double one_way_propagation_s = 0;
  double durable_admission_delay_s = 0;
  double turnaround_s = 0;
  double retry_backoff_s = 1;
  double failed_ack_timeout_s = 1;
  std::uint32_t max_attempts_per_credit_epoch = 3;
  double archive_retention_end_s = 0; // No automatic purge; PurgeRetained explicit.
  // Optional physical-owner binding contract. The event host supplies the
  // actual finite ledger and prospective callbacks; no second battery exists.
  double source_budget_j = 0, source_acquisition_base_w = -1;
  // Electrical active cable powers only, not acquisition/idle/flash/optics.
  // Specify all four or none. Absence means UNKNOWN, never measured zero power.
  std::optional<double> source_tx_w, source_rx_w, gateway_tx_w, gateway_rx_w;
};

struct Event {
  std::string kind;
  Record record;
  double time_s = 0;
  std::uint32_t epoch_attempt = 0;
  std::uint64_t archive_bytes = 0;
  std::uint64_t raw_gateway_bytes = 0;
};

struct EnergyEvent {
  std::string owner, activity;
  std::uint64_t record_id = 0;
  double start_s = 0, end_s = 0, power_w = 0, joules = 0;
};

struct Callbacks {
  // Called after finite DATA arrival + durable-admission delay. Must perform
  // admission against the MAIN all-priority resident allocation. false is an
  // ingress rejection/backpressure, NOT permanent generated-record loss.
  // Reentrant ReleaseGateway for an evicted raw record is explicitly supported.
  std::function<bool(const Record&, double)> on_admitted;
  std::function<void(const Event&)> on_event;
  std::function<void(const EnergyEvent&)> on_energy;
  // Prospective intervals allow the event host to register loads BEFORE energy
  // is consumed, on the same owner ledger as concurrent radios/other devices.
  std::function<void(const EnergyEvent&)> on_activity;
  std::function<bool(double)> powered;
  std::function<bool(const Record&)> acquisition_available;
  std::function<bool(const Record&, double, double, double)> data_delivered;
  // Latest electrically powered endpoint <= requested end, clipping intervals.
  std::function<double(const std::string&, double)> powered_until;
  // Optional deterministic/fault-model hook at ACK arrival. Default reliable
  // cable ACK is an explicit model assumption, not an observed channel claim.
  std::function<bool(const Record&, bool, std::uint32_t, double)> ack_delivered;
};

struct Counters {
  std::uint64_t generated_records = 0, generated_payload_bytes = 0;
  std::uint64_t archive_overflow_records = 0, archive_overflow_payload_bytes = 0;
  std::uint64_t source_unavailable_records = 0, source_unavailable_payload_bytes = 0;
  std::uint64_t low_archive_overflow_records = 0, low_source_unavailable_records = 0;
  std::uint64_t upstream_backpressure_rejections = 0;
  std::uint64_t duplicate_submissions = 0, data_attempts = 0;
  std::uint64_t gateway_admissions = 0, gateway_rejections = 0;
  std::uint64_t duplicate_data_attempts = 0, acks_lost = 0, credit_frames = 0;
  std::uint64_t backpressure_exhaustions = 0, ram_blocked_records = 0;
  std::uint64_t power_blocked_records = 0, power_interrupted_attempts = 0;
  std::uint64_t forwarded_records = 0, forwarded_payload_bytes = 0;
  std::uint64_t gateway_dropped_records = 0, gateway_dropped_payload_bytes = 0;
  std::uint64_t archive_purged_records = 0, archive_purged_storage_bytes = 0;
};

struct State {
  double time_s = 0;
  Counters counters;
  std::uint64_t archive_resident_bytes = 0, archive_resident_records = 0;
  std::uint64_t retained_historical_records = 0, source_unconfirmed_records = 0;
  std::uint64_t source_custody_records = 0, source_custody_payload_bytes = 0;
  std::uint64_t low_source_custody_records = 0;
  std::uint64_t raw_gateway_records = 0, raw_gateway_payload_bytes = 0;
  std::uint64_t raw_gateway_resident_bytes = 0;
  std::uint64_t source_staging_ram_bytes = 0, gateway_staging_ram_bytes = 0;
  std::uint64_t archive_high_water_bytes = 0, raw_gateway_high_water_bytes = 0;
  std::uint64_t source_ram_high_water_bytes = 0, gateway_ram_high_water_bytes = 0;
  bool wire_active = false, active_power_known = false;
  double source_cable_active_energy_j = 0, gateway_cable_active_energy_j = 0;
  bool conservation_ok = false;
};

class Link {
 public:
  explicit Link(Config config, Callbacks callbacks)
      : cfg_(std::move(config)), cb_(std::move(callbacks)) {
    if (!cfg_.source_archive_capacity_bytes || !cfg_.gateway_outbox_capacity_bytes ||
        !cfg_.source_staging_ram_capacity_bytes || !cfg_.gateway_staging_ram_capacity_bytes ||
        !cfg_.ack_wire_bytes || !cfg_.max_attempts_per_credit_epoch || !cb_.on_admitted)
      throw std::invalid_argument("finite ingress requires capacities, serialized ACK and an authoritative admission callback");
    Positive(cfg_.bit_rate_bps); Positive(cfg_.wire_bits_per_byte);
    Positive(cfg_.retry_backoff_s); Positive(cfg_.failed_ack_timeout_s);
    Positive(cfg_.archive_retention_end_s);
    Nonnegative(cfg_.one_way_propagation_s); Nonnegative(cfg_.durable_admission_delay_s);
    Nonnegative(cfg_.turnaround_s);
    const unsigned power_count = unsigned(bool(cfg_.source_tx_w)) + unsigned(bool(cfg_.source_rx_w)) +
        unsigned(bool(cfg_.gateway_tx_w)) + unsigned(bool(cfg_.gateway_rx_w));
    if (power_count != 0 && power_count != 4)
      throw std::invalid_argument("specify all four active cable powers, or explicitly leave them unknown");
    power_known_ = power_count == 4;
    if (power_known_) for (double p : {*cfg_.source_tx_w, *cfg_.source_rx_w,
                                     *cfg_.gateway_tx_w, *cfg_.gateway_rx_w}) Nonnegative(p);
    if (cb_.powered || cb_.acquisition_available || cb_.data_delivered || cb_.powered_until || cb_.on_activity) {
      if (!cb_.powered || !cb_.acquisition_available || !cb_.data_delivered ||
          !cb_.powered_until || !cb_.on_activity || !power_known_)
        throw std::invalid_argument("physical ingress owner binding requires every power/activity callback");
      Positive(cfg_.source_budget_j); Nonnegative(cfg_.source_acquisition_base_w);
    }
    const auto ack_ram = Add(cfg_.record_overhead_bytes, cfg_.ack_wire_bytes);
    if (ack_ram > cfg_.source_staging_ram_capacity_bytes || ack_ram > cfg_.gateway_staging_ram_capacity_bytes)
      throw std::invalid_argument("staging RAM cannot hold the declared control frame");
  }

  SubmitResult Submit(const Record& record, double now, bool upstream_retains_custody = false) {
    ExternalTime(now);
    if (!record.id || !record.producer_sequence || !record.payload_bytes || record.window_id.empty() ||
        record.window_id.size() > 128 || static_cast<unsigned>(record.priority) > 2)
      throw std::invalid_argument("invalid ingress record identity, size or priority");
    Nonnegative(record.acquisition_start_s); Nonnegative(record.release_s);
    if (record.windowNumericId || record.windowPayloadBytes || record.windowChunkIndex || record.windowChunkCount) {
      if (!record.windowNumericId || !record.windowPayloadBytes || !record.windowChunkCount ||
          record.windowChunkIndex >= record.windowChunkCount ||
          record.payload_bytes > record.windowPayloadBytes || cfg_.record_overhead_bytes < 24)
        throw std::invalid_argument("invalid finite completed-window descriptor");
    }
    const double ready = record.source_ready_s < 0 ? record.release_s : record.source_ready_s;
    Nonnegative(ready);
    if (record.acquisition_start_s > record.release_s || ready < record.release_s || ready != now)
      throw std::invalid_argument("preserve acquisition/release and submit at actual local availability");
    if (record.producer_sequence <= sequence_watermark_) {
      auto found = archive_.find(record.id);
      if (found != archive_.end() && !SameRecord(found->second.record, record))
        throw std::invalid_argument("duplicate identity has different immutable metadata");
      ++stats_.duplicate_submissions;
      Emit("duplicate_submission", record);
      return SubmitResult::Duplicate;
    }
    if (archive_.count(record.id) || gateway_.count(record.id))
      throw std::invalid_argument("new producer sequence reuses an existing record ID");
    const auto size = StorageBytes(record);
    const bool available = !cb_.acquisition_available || cb_.acquisition_available(record);
    if (upstream_retains_custody && (!available || size > cfg_.source_archive_capacity_bytes - archive_bytes_)) {
      // A failed DRN/logger admission is backpressure, not a new generated
      // record or permanent loss: the remote acoustic sender still owns it.
      ++stats_.upstream_backpressure_rejections;
      Emit("upstream_source_retained_local_admission_rejected", record);
      return available ? SubmitResult::ArchiveFull : SubmitResult::SourceUnavailable;
    }
    sequence_watermark_ = record.producer_sequence;
    ++stats_.generated_records; stats_.generated_payload_bytes = Add(stats_.generated_payload_bytes, record.payload_bytes);
    if (!available) {
      ++stats_.source_unavailable_records;
      if (record.priority == Priority::Low) ++stats_.low_source_unavailable_records;
      stats_.source_unavailable_payload_bytes = Add(stats_.source_unavailable_payload_bytes, record.payload_bytes);
      Emit("source_unavailable_at_window_release", record);
      return SubmitResult::SourceUnavailable;
    }
    if (size > cfg_.source_archive_capacity_bytes - archive_bytes_) {
      ++stats_.archive_overflow_records;
      if (record.priority == Priority::Low) ++stats_.low_archive_overflow_records;
      stats_.archive_overflow_payload_bytes = Add(stats_.archive_overflow_payload_bytes, record.payload_bytes);
      Emit("source_archive_overflow", record);
      return SubmitResult::ArchiveFull;
    }
    Entry entry; entry.record = record; entry.ready_s = now;
    archive_.emplace(record.id, entry); archive_bytes_ += size;
    archive_high_ = std::max(archive_high_, archive_bytes_);
    Emit("source_archive_admitted", record);
    return SubmitResult::Accepted;
  }

  // Invoke only on actual onward custody or actual gateway eviction/drop,
  // including records in pending transmissions. A dequeue is not a release.
  bool ReleaseGateway(std::uint64_t id, Removal reason, double now) {
    if (inside_admission_) {
      if (now != now_) throw std::logic_error("reentrant gateway release changed event time");
    } else ExternalTime(now);
    auto found = gateway_.find(id);
    if (found == gateway_.end()) return false; // Duplicate callbacks are idempotent.
    if (found->second.tentative) {
      deferred_removal_ = reason;
      gateway_.erase(found);
      return true;
    }
    const Record record = found->second.record;
    raw_gateway_bytes_ -= StorageBytes(record);
    gateway_.erase(found);
    auto archived = archive_.find(id);
    if (archived != archive_.end()) archived->second.custody = reason == Removal::Forwarded ? Custody::Forwarded : Custody::Dropped;
    CountRemoval(record, reason);
    Emit(reason == Removal::Forwarded ? "gateway_custody_forwarded" : "gateway_record_dropped_archive_retained", record);
    return true;
  }

  // Explicit gateway space/credit signaling, not an instantaneous source-side
  // view of remote occupancy. Coalesced to one pending control frame; serialized
  // on the SAME half-duplex link. It resumes source-custody blocked records only.
  // It never redeems a previously accepted-and-evicted gateway record.
  void NotifyGatewaySpace(double now) {
    ExternalTime(now);
    if (std::any_of(archive_.begin(), archive_.end(), [](const auto& pair) {
          return pair.second.blocked && !pair.second.ram_blocked && !pair.second.power_blocked && !pair.second.ever_admitted;
        })) credit_pending_ = true;
  }

  std::optional<double> NextEventTime() const {
    if (active_) return active_->event_s;
    if (credit_pending_) return now_;
    std::optional<double> next;
    for (const auto& pair : archive_) {
      const auto& entry = pair.second;
      if (!entry.confirmed && !entry.blocked) {
        const double t = std::max(now_, entry.ready_s);
        if (!next || t < *next) next = t;
      }
    }
    return next;
  }

  // ns-3 wrapper must schedule the returned NextEventTime, not defer callbacks
  // to a later ns-3 time. Offline tests may advance several events at once.
  void Advance(double until) {
    Nonnegative(until);
    if (until < now_ || advancing_) throw std::logic_error("ingress time reversal or recursive Advance");
    advancing_ = true;
    try {
      while (true) {
        const auto next = NextEventTime();
        if (!next || *next > until) break;
        ChargeTo(*next); now_ = *next;
        if (active_) FinishStage(); else StartNext();
      }
      ChargeTo(until); now_ = until; advancing_ = false;
    } catch (...) { advancing_ = false; throw; }
  }

  State Snapshot(double now) {
    ExternalTime(now);
    if (inside_admission_) throw std::logic_error("snapshot requested inside atomic gateway admission");
    State s; s.time_s = now_; s.counters = stats_;
    s.archive_resident_bytes = archive_bytes_; s.archive_resident_records = archive_.size();
    s.raw_gateway_resident_bytes = raw_gateway_bytes_;
    for (const auto& pair : archive_) {
      const auto& e = pair.second;
      if (!e.confirmed) ++s.source_unconfirmed_records;
      if (e.ever_admitted) ++s.retained_historical_records;
      if (e.custody == Custody::Source) {
        ++s.source_custody_records; s.source_custody_payload_bytes += e.record.payload_bytes;
        if (e.record.priority == Priority::Low) ++s.low_source_custody_records;
      }
    }
    for (const auto& pair : gateway_) if (!pair.second.tentative) {
      ++s.raw_gateway_records; s.raw_gateway_payload_bytes += pair.second.record.payload_bytes;
    }
    s.source_staging_ram_bytes = source_ram_; s.gateway_staging_ram_bytes = gateway_ram_;
    s.archive_high_water_bytes = archive_high_; s.raw_gateway_high_water_bytes = raw_gateway_high_;
    s.source_ram_high_water_bytes = source_ram_high_; s.gateway_ram_high_water_bytes = gateway_ram_high_;
    s.wire_active = bool(active_); s.active_power_known = power_known_;
    s.source_cable_active_energy_j = source_energy_; s.gateway_cable_active_energy_j = gateway_energy_;
    s.conservation_ok =
        stats_.generated_records == stats_.archive_overflow_records + stats_.source_unavailable_records + s.source_custody_records + s.raw_gateway_records + stats_.forwarded_records + stats_.gateway_dropped_records &&
        stats_.generated_payload_bytes == stats_.archive_overflow_payload_bytes + stats_.source_unavailable_payload_bytes + s.source_custody_payload_bytes + s.raw_gateway_payload_bytes + stats_.forwarded_payload_bytes + stats_.gateway_dropped_payload_bytes &&
        archive_bytes_ <= cfg_.source_archive_capacity_bytes && raw_gateway_bytes_ <= cfg_.gateway_outbox_capacity_bytes &&
        source_ram_ <= cfg_.source_staging_ram_capacity_bytes && gateway_ram_ <= cfg_.gateway_staging_ram_capacity_bytes;
    return s;
  }

  // Retention expiry is explicit and never purges pending/unconfirmed sources.
  // Call Snapshot at the horizon BEFORE optional archival housekeeping.
  std::uint64_t PurgeRetained(double now) {
    ExternalTime(now);
    if (now < cfg_.archive_retention_end_s) throw std::logic_error("archive retention boundary not reached");
    std::uint64_t count = 0;
    for (auto it = archive_.begin(); it != archive_.end();) {
      if (it->second.confirmed && it->second.custody != Custody::Source) {
        const auto record = it->second.record; const auto size = StorageBytes(record);
        archive_bytes_ -= size; ++stats_.archive_purged_records; stats_.archive_purged_storage_bytes += size;
        it = archive_.erase(it); ++count; Emit("retained_archive_purged", record);
      } else ++it;
    }
    return count;
  }

  const Config& configuration() const { return cfg_; }

 private:
  struct Entry {
    Record record; Custody custody = Custody::Source;
    bool confirmed = false, ever_admitted = false, blocked = false, ram_blocked = false, power_blocked = false;
    std::uint32_t epoch_attempt = 0;
    double ready_s = 0;
  };
  struct GatewayEntry { Record record; bool tentative = false; };
  enum class Stage { Data, Admission, Ack, Credit };
  struct Active {
    Stage stage = Stage::Data; std::uint64_t id = 0;
    double start_s = 0, tx_end_s = 0, event_s = 0, charged_until_s = 0;
    bool admitted = false;
  };
  Config cfg_; Callbacks cb_; std::map<std::uint64_t, Entry> archive_;
  std::map<std::uint64_t, GatewayEntry> gateway_;
  std::optional<Active> active_; std::optional<Removal> deferred_removal_;
  bool advancing_ = false, inside_admission_ = false, credit_pending_ = false, power_known_ = false;
  double now_ = 0, source_energy_ = 0, gateway_energy_ = 0;
  std::uint64_t sequence_watermark_ = 0, archive_bytes_ = 0, raw_gateway_bytes_ = 0;
  std::uint64_t source_ram_ = 0, gateway_ram_ = 0, archive_high_ = 0, raw_gateway_high_ = 0;
  std::uint64_t source_ram_high_ = 0, gateway_ram_high_ = 0;
  Counters stats_;

  static void Nonnegative(double x) { if (!std::isfinite(x) || x < 0) throw std::invalid_argument("finite nonnegative timing/power required"); }
  static void Positive(double x) { if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("finite positive parameter required"); }
  static std::uint64_t Add(std::uint64_t a, std::uint64_t b) {
    if (b > std::numeric_limits<std::uint64_t>::max() - a) throw std::overflow_error("storage/counter overflow");
    return a + b;
  }
  static bool SameRecord(const Record& a, const Record& b) {
    return std::tie(a.id, a.producer_sequence, a.window_id, a.payload_bytes, a.priority, a.acquisition_start_s, a.release_s,
                    a.windowNumericId, a.windowPayloadBytes, a.windowChunkIndex, a.windowChunkCount,
                    a.source_ready_s, a.source_node) ==
           std::tie(b.id, b.producer_sequence, b.window_id, b.payload_bytes, b.priority, b.acquisition_start_s, b.release_s,
                    b.windowNumericId, b.windowPayloadBytes, b.windowChunkIndex, b.windowChunkCount,
                    b.source_ready_s, b.source_node);
  }
  std::uint64_t StorageBytes(const Record& r) const { return Add(r.payload_bytes, cfg_.record_overhead_bytes); }
  double WireSeconds(std::uint64_t bytes) const {
    const double duration = double(bytes) * cfg_.wire_bits_per_byte / cfg_.bit_rate_bps;
    Positive(duration); return duration;
  }
  double Future(double delay) const {
    const double value = now_ + delay;
    if (!std::isfinite(value) || value < now_) throw std::overflow_error("ingress event time overflow");
    return value;
  }
  void ExternalTime(double now) {
    if (inside_admission_) throw std::logic_error("only ReleaseGateway may reenter admission callback");
    if (advancing_) throw std::logic_error("external mutation inside ingress callback");
    Advance(now);
  }
  void Emit(const std::string& kind, const Record& r) {
    if (cb_.on_event) {
      auto it = archive_.find(r.id);
      cb_.on_event(Event{kind, r, now_, it == archive_.end() ? 0 : it->second.epoch_attempt, archive_bytes_, raw_gateway_bytes_});
    }
  }
  void SetRam(std::uint64_t bytes) {
    if (bytes > cfg_.source_staging_ram_capacity_bytes || bytes > cfg_.gateway_staging_ram_capacity_bytes)
      throw std::logic_error("finite staging RAM exceeded");
    source_ram_ = gateway_ram_ = bytes;
    source_ram_high_ = std::max(source_ram_high_, bytes); gateway_ram_high_ = std::max(gateway_ram_high_, bytes);
  }
  void CountRemoval(const Record& r, Removal reason) {
    if (reason == Removal::Forwarded) { ++stats_.forwarded_records; stats_.forwarded_payload_bytes += r.payload_bytes; }
    else { ++stats_.gateway_dropped_records; stats_.gateway_dropped_payload_bytes += r.payload_bytes; }
  }
  void Announce(const std::string& owner, const std::string& activity,
                double start, double end, double watts) {
    if (cb_.on_activity) cb_.on_activity(EnergyEvent{owner, activity, active_->id,
                                                   start, end, watts, (end - start) * watts});
  }
  void AnnounceData() {
    if (!cb_.on_activity) return;
    Announce("source", "cable_data_tx", active_->start_s, active_->tx_end_s, *cfg_.source_tx_w);
    Announce("gateway", "cable_data_rx", active_->start_s + cfg_.one_way_propagation_s,
             active_->tx_end_s + cfg_.one_way_propagation_s, *cfg_.gateway_rx_w);
  }
  void AnnounceControl(bool credit) {
    if (!cb_.on_activity) return;
    Announce("gateway", credit ? "cable_credit_tx" : "cable_ack_tx",
             active_->start_s, active_->tx_end_s, *cfg_.gateway_tx_w);
    Announce("source", credit ? "cable_credit_rx" : "cable_ack_rx",
             active_->start_s + cfg_.one_way_propagation_s,
             active_->tx_end_s + cfg_.one_way_propagation_s, *cfg_.source_rx_w);
  }
  void StartNext() {
    if (credit_pending_) {
      credit_pending_ = false;
      if (cb_.powered && !cb_.powered(now_)) return;
      ++stats_.credit_frames;
      const double end = Future(WireSeconds(cfg_.ack_wire_bytes));
      active_ = Active{Stage::Credit, 0, now_, end, end + cfg_.one_way_propagation_s, now_, false};
      SetRam(Add(cfg_.ack_wire_bytes, cfg_.record_overhead_bytes)); AnnounceControl(true); return;
    }
    Entry* selected = nullptr;
    for (auto& pair : archive_) {
      auto& e = pair.second;
      if (e.confirmed || e.blocked || e.ready_s > now_) continue;
      if (!selected || std::tie(e.record.priority, e.record.release_s, e.record.id) <
                       std::tie(selected->record.priority, selected->record.release_s, selected->record.id)) selected = &e;
    }
    if (!selected) throw std::logic_error("ready ingress event without eligible source record");
    if (cb_.powered && !cb_.powered(now_)) {
      selected->blocked = selected->power_blocked = true; ++stats_.power_blocked_records;
      Emit("source_forwarding_blocked_by_depleted_owner", selected->record); return;
    }
    const auto ram = Add(StorageBytes(selected->record), cfg_.data_header_bytes);
    if (ram > cfg_.source_staging_ram_capacity_bytes || ram > cfg_.gateway_staging_ram_capacity_bytes) {
      selected->blocked = selected->ram_blocked = true; ++stats_.ram_blocked_records;
      Emit("source_staging_ram_blocked", selected->record); return;
    }
    ++selected->epoch_attempt; ++stats_.data_attempts;
    const auto descriptor = selected->record.windowChunkCount ? 24u : 0u;
    const double end = Future(WireSeconds(Add(Add(selected->record.payload_bytes, descriptor), cfg_.data_header_bytes)));
    active_ = Active{Stage::Data, selected->record.id, now_, end, end + cfg_.one_way_propagation_s, now_, false};
    SetRam(ram); AnnounceData(); Emit("ingress_data_started", selected->record);
  }
  void FinishStage() {
    if (active_->stage == Stage::Credit) {
      active_.reset(); SetRam(0);
      if (cb_.powered && !cb_.powered(now_)) return;
      for (auto& pair : archive_) {
        auto& e = pair.second;
        if (e.blocked && !e.ram_blocked && !e.power_blocked && !e.ever_admitted) {
          e.blocked = false; e.epoch_attempt = 0; e.ready_s = now_;
          Emit("source_credit_received", e.record);
        }
      }
      return;
    }
    auto& entry = archive_.at(active_->id);
    if (active_->stage == Stage::Data) {
      active_->stage = Stage::Admission;
      active_->event_s = Future(cfg_.durable_admission_delay_s);
      Emit("ingress_data_arrived", entry.record); return;
    }
    if (active_->stage == Stage::Admission) {
      if (cb_.data_delivered && !cb_.data_delivered(entry.record, active_->tx_end_s,
            active_->tx_end_s + cfg_.one_way_propagation_s, now_)) {
        entry.blocked = entry.power_blocked = true;
        ++stats_.power_blocked_records; ++stats_.power_interrupted_attempts;
        Emit("ingress_power_interrupted_source_retained", entry.record);
        active_.reset(); SetRam(0); return;
      }
      bool accepted = entry.ever_admitted;
      if (accepted) { ++stats_.duplicate_data_attempts; Emit("duplicate_data_custody_already_transferred", entry.record); }
      else {
        // The tentative descriptor is in bounded staging RAM, not an extra
        // persistent allocation. Main admission may evict other raw entries.
        gateway_.emplace(entry.record.id, GatewayEntry{entry.record, true});
        deferred_removal_.reset(); inside_admission_ = true;
        try { accepted = cb_.on_admitted(entry.record, now_); }
        catch (...) { inside_admission_ = false; gateway_.erase(entry.record.id); deferred_removal_.reset(); throw; }
        inside_admission_ = false;
        if (accepted) {
          ++stats_.gateway_admissions; entry.ever_admitted = true; entry.custody = Custody::Gateway;
          if (deferred_removal_) {
            entry.custody = *deferred_removal_ == Removal::Forwarded ? Custody::Forwarded : Custody::Dropped;
            CountRemoval(entry.record, *deferred_removal_);
          } else {
            if (StorageBytes(entry.record) > cfg_.gateway_outbox_capacity_bytes - raw_gateway_bytes_)
              throw std::logic_error("main gateway accepted beyond the configured raw-subset capacity");
            gateway_.at(entry.record.id).tentative = false;
            raw_gateway_bytes_ += StorageBytes(entry.record); raw_gateway_high_ = std::max(raw_gateway_high_, raw_gateway_bytes_);
          }
          Emit("gateway_durable_admission", entry.record);
        } else {
          gateway_.erase(entry.record.id); ++stats_.gateway_rejections;
          Emit("gateway_admission_rejected_source_retained", entry.record);
        }
        deferred_removal_.reset();
      }
      active_->admitted = accepted; active_->stage = Stage::Ack;
      active_->start_s = Future(cfg_.turnaround_s);
      active_->tx_end_s = active_->start_s + WireSeconds(cfg_.ack_wire_bytes);
      active_->event_s = active_->tx_end_s + cfg_.one_way_propagation_s;
      active_->charged_until_s = now_;
      SetRam(Add(cfg_.ack_wire_bytes, cfg_.record_overhead_bytes)); AnnounceControl(false); return;
    }
    const bool delivered = !cb_.ack_delivered || cb_.ack_delivered(entry.record, active_->admitted, entry.epoch_attempt, now_);
    const bool acknowledged = delivered && active_->admitted;
    if (acknowledged) { entry.confirmed = true; Emit("source_admission_ack_archive_retained", entry.record); }
    else {
      if (!delivered) { ++stats_.acks_lost; Emit("ingress_ack_lost", entry.record); }
      else Emit("source_admission_nack", entry.record);
      if (cb_.powered && !cb_.powered(now_)) {
        entry.blocked = entry.power_blocked = true; ++stats_.power_blocked_records;
        Emit("source_confirmation_blocked_by_depleted_owner", entry.record);
      } else if (entry.epoch_attempt >= cfg_.max_attempts_per_credit_epoch) {
        entry.blocked = true; ++stats_.backpressure_exhaustions; Emit("source_backpressure_wait_credit", entry.record);
      } else entry.ready_s = Future(cfg_.retry_backoff_s + (delivered ? 0 : cfg_.failed_ack_timeout_s));
    }
    active_.reset(); SetRam(0);
  }
  void ChargeSpan(double until, const std::string& owner, const std::string& activity,
                  double start, double end, double power) {
    const double begin = std::max(active_->charged_until_s, start);
    double finish = std::min(until, end);
    if (cb_.powered_until) {
      const double powered = cb_.powered_until(owner, finish);
      Nonnegative(powered); finish = std::min(finish, powered);
    }
    if (finish <= begin) return;
    const double energy = (finish - begin) * power;
    if (owner == "source") source_energy_ += energy; else gateway_energy_ += energy;
    if (cb_.on_energy) cb_.on_energy(EnergyEvent{owner, activity, active_->id, begin, finish, power, energy});
  }
  void ChargeTo(double until) {
    if (!active_ || !power_known_) return;
    if (active_->stage == Stage::Data) {
      ChargeSpan(until, "source", "cable_data_tx", active_->start_s, active_->tx_end_s, *cfg_.source_tx_w);
      ChargeSpan(until, "gateway", "cable_data_rx", active_->start_s + cfg_.one_way_propagation_s,
                 active_->tx_end_s + cfg_.one_way_propagation_s, *cfg_.gateway_rx_w);
    } else if (active_->stage == Stage::Ack || active_->stage == Stage::Credit) {
      ChargeSpan(until, "gateway", active_->stage == Stage::Ack ? "cable_ack_tx" : "cable_credit_tx",
                 active_->start_s, active_->tx_end_s, *cfg_.gateway_tx_w);
      ChargeSpan(until, "source", active_->stage == Stage::Ack ? "cable_ack_rx" : "cable_credit_rx",
                 active_->start_s + cfg_.one_way_propagation_s,
                 active_->tx_end_s + cfg_.one_way_propagation_s, *cfg_.source_rx_w);
    }
    active_->charged_until_s = until;
  }
};

} // namespace closure_ingress
#endif
