#ifndef EDC_EXTERNAL_WORKLOAD_REPLAY_H
#define EDC_EXTERNAL_WORKLOAD_REPLAY_H

// Standalone, header-only validation of complete exogenous application workloads.
// No ns-3 dependency and no scheduling, routing, random draws or file mutation.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <istream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace edc_replay {

constexpr const char* kCsvHeader =
    "release_s,record_id,source_bsn,priority,payload_bytes,ddn_index,window_id,acquisition_start_s";

enum class Priority : std::uint8_t { High = 0, Medium = 1, Low = 2 };

inline const char* PriorityName(Priority priority) {
    switch (priority) {
    case Priority::High: return "HIGH";
    case Priority::Medium: return "MEDIUM";
    case Priority::Low: return "LOW";
    }
    throw std::logic_error("invalid replay priority enum");
}

struct ValidationOptions {
    // Required: parser rejects incomplete inputs instead of truncating at stop_s.
    double stop_s = 0.0;
    std::uint32_t source_bsn_count = 144;
    std::uint32_t ddn_count = 8;
    // Current physical profile: LOW belongs to a gateway-attached instrument,
    // encoded as -1; it must never index the small-BSN array. Explicit false
    // selects the alternative BSN-source role and its usual nonnegative range.
    bool low_source_is_gateway_instrument = true;
};

struct Event {
    double release_s;
    std::uint64_t record_id;
    std::int32_t source_bsn;
    Priority priority;
    std::uint32_t payload_bytes;
    std::int32_t ddn_index;
    std::string window_id;
    double acquisition_start_s;
};

struct WindowSummary {
    double release_s;
    double acquisition_start_s;
    std::int32_t source_bsn;
    std::int32_t ddn_index;
    std::uint64_t record_count;
    std::uint64_t payload_bytes;
};

struct Workload {
    // Deterministic replay order: release_s ascending, then record_id ascending.
    std::vector<Event> events;
    // Identifiers are global to the file. All chunks of a window share metadata.
    std::map<std::string, WindowSummary> windows;
    std::array<std::uint64_t, 3> record_counts{{0, 0, 0}};
    std::array<std::uint64_t, 3> payload_bytes{{0, 0, 0}};
};

class ParseError : public std::runtime_error {
public:
    explicit ParseError(const std::string& message) : std::runtime_error(message) {}
};

namespace detail {

inline void Fail(const std::string& origin, std::size_t line,
                 const std::string& field, const std::string& reason) {
    std::ostringstream message;
    message << origin << ":" << line;
    if (!field.empty()) message << " [" << field << "]";
    message << ": " << reason;
    throw ParseError(message.str());
}

inline void StripLineEnding(std::string& line) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
}

inline std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (;;) {
        const std::size_t comma = line.find(',', start);
        fields.push_back(line.substr(start, comma == std::string::npos
                                               ? comma : comma - start));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return fields;
}

inline std::uint64_t Unsigned(const std::string& token, std::uint64_t maximum,
                              const std::string& origin, std::size_t line,
                              const std::string& field) {
    if (token.empty()) Fail(origin, line, field, "expected unsigned decimal integer");
    std::uint64_t value = 0;
    for (char c : token) {
        if (c < '0' || c > '9')
            Fail(origin, line, field, "expected unsigned decimal integer without whitespace");
        const std::uint64_t digit = static_cast<std::uint64_t>(c - '0');
        if (value > maximum / 10 ||
            (value == maximum / 10 && digit > maximum % 10))
            Fail(origin, line, field, "integer exceeds allowed range");
        value = value * 10 + digit;
    }
    return value;
}

inline double Time(const std::string& token, const std::string& origin,
                   std::size_t line, const std::string& field) {
    if (token.empty()) Fail(origin, line, field, "expected finite nonnegative seconds");
    // Classic locale fixes the decimal point independently of the host locale.
    std::istringstream input(token);
    input.imbue(std::locale::classic());
    double value = 0.0;
    input >> std::noskipws >> value;
    if (input.fail() || !input.eof() || !std::isfinite(value) || value < 0.0)
        Fail(origin, line, field, "expected finite nonnegative seconds without whitespace");
    return value;
}

inline void Add(std::uint64_t& total, std::uint64_t amount,
                const std::string& origin, std::size_t line,
                const std::string& field) {
    if (amount > std::numeric_limits<std::uint64_t>::max() - total)
        Fail(origin, line, field, "aggregate uint64 overflow");
    total += amount;
}

inline bool ValidWindowId(const std::string& id) {
    if (id.empty()) return false;
    for (char c : id) {
        const bool valid = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-' ||
                           c == '.' || c == ':';
        if (!valid) return false;
    }
    return true;
}

inline void ValidateOptions(const ValidationOptions& options, const std::string& origin) {
    if (!std::isfinite(options.stop_s) || options.stop_s <= 0.0)
        Fail(origin, 0, "stop_s", "must be finite and positive");
    const std::uint32_t largest_count =
        static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) + 1u;
    if (options.source_bsn_count == 0 || options.source_bsn_count > largest_count)
        Fail(origin, 0, "source_bsn_count", "must admit nonnegative int32 indices");
    if (options.ddn_count == 0 || options.ddn_count > largest_count)
        Fail(origin, 0, "ddn_count", "must admit nonnegative int32 indices");
}

} // namespace detail

inline Workload ParseCsv(std::istream& input, const ValidationOptions& options,
                         const std::string& origin = "<workload>") {
    detail::ValidateOptions(options, origin);
    std::string line;
    if (!std::getline(input, line)) detail::Fail(origin, 1, "header", "missing header");
    detail::StripLineEnding(line);
    if (line != kCsvHeader)
        detail::Fail(origin, 1, "header", "header must exactly match replay schema");

    Workload workload;
    std::set<std::uint64_t> identifiers;
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        detail::StripLineEnding(line);
        const std::vector<std::string> fields = detail::Split(line);
        if (fields.size() != 8)
            detail::Fail(origin, line_number, "row", "expected exactly eight unquoted CSV fields");

        Event event{};
        event.release_s = detail::Time(fields[0], origin, line_number, "release_s");
        if (event.release_s >= options.stop_s)
            detail::Fail(origin, line_number, "release_s", "must be strictly before stop_s; input is not truncated");
        event.record_id = detail::Unsigned(fields[1], std::numeric_limits<std::uint64_t>::max(),
                                           origin, line_number, "record_id");
        if (event.record_id == 0)
            detail::Fail(origin, line_number, "record_id", "must be positive");
        if (!identifiers.insert(event.record_id).second)
            detail::Fail(origin, line_number, "record_id", "duplicate record identifier");

        if (fields[3] == "HIGH") event.priority = Priority::High;
        else if (fields[3] == "MEDIUM") event.priority = Priority::Medium;
        else if (fields[3] == "LOW") event.priority = Priority::Low;
        else detail::Fail(origin, line_number, "priority", "expected HIGH, MEDIUM or LOW");

        if (event.priority == Priority::Low && options.low_source_is_gateway_instrument) {
            if (fields[2] != "-1")
                detail::Fail(origin, line_number, "source_bsn", "LOW gateway instrument requires -1, not a BSN index");
            event.source_bsn = -1;
        } else {
            event.source_bsn = static_cast<std::int32_t>(detail::Unsigned(
                fields[2], options.source_bsn_count - 1u, origin, line_number, "source_bsn"));
        }
        event.payload_bytes = static_cast<std::uint32_t>(detail::Unsigned(
            fields[4], std::numeric_limits<std::uint32_t>::max(), origin, line_number, "payload_bytes"));
        if (event.payload_bytes == 0)
            detail::Fail(origin, line_number, "payload_bytes", "must be positive");

        event.window_id = fields[6];
        event.acquisition_start_s = detail::Time(fields[7], origin, line_number, "acquisition_start_s");
        if (event.acquisition_start_s > event.release_s)
            detail::Fail(origin, line_number, "acquisition_start_s", "must not exceed release_s");

        if (event.priority == Priority::Low) {
            event.ddn_index = static_cast<std::int32_t>(detail::Unsigned(
                fields[5], options.ddn_count - 1u, origin, line_number, "ddn_index"));
            if (!detail::ValidWindowId(event.window_id))
                detail::Fail(origin, line_number, "window_id", "LOW needs a nonempty ASCII identifier [A-Za-z0-9_.:-]+");
            const auto found = workload.windows.find(event.window_id);
            if (found == workload.windows.end()) {
                workload.windows.emplace(event.window_id, WindowSummary{
                    event.release_s, event.acquisition_start_s, event.source_bsn,
                    event.ddn_index, 1, event.payload_bytes});
            } else {
                WindowSummary& window = found->second;
                if (window.release_s != event.release_s ||
                    window.acquisition_start_s != event.acquisition_start_s ||
                    window.source_bsn != event.source_bsn || window.ddn_index != event.ddn_index)
                    detail::Fail(origin, line_number, "window_id", "chunks of one window must share source, gateway, release and acquisition start");
                detail::Add(window.record_count, 1, origin, line_number, "window record count");
                detail::Add(window.payload_bytes, event.payload_bytes, origin, line_number, "window payload bytes");
            }
        } else {
            if (fields[5] != "-1")
                detail::Fail(origin, line_number, "ddn_index", "HIGH/MEDIUM require -1 for automatic routing");
            event.ddn_index = -1;
            if (!event.window_id.empty())
                detail::Fail(origin, line_number, "window_id", "HIGH/MEDIUM require an empty window identifier");
            if (event.acquisition_start_s != event.release_s)
                detail::Fail(origin, line_number, "acquisition_start_s", "HIGH/MEDIUM use release_s (report availability)");
        }

        const std::size_t priority_index = static_cast<std::size_t>(event.priority);
        detail::Add(workload.record_counts[priority_index], 1, origin, line_number, "record count");
        detail::Add(workload.payload_bytes[priority_index], event.payload_bytes,
                    origin, line_number, "class payload bytes");
        workload.events.push_back(event);
    }
    if (input.bad() || (input.fail() && !input.eof()))
        detail::Fail(origin, line_number, "stream", "I/O failure while reading workload");
    if (workload.events.empty()) detail::Fail(origin, line_number, "records", "workload must be nonempty");
    std::stable_sort(workload.events.begin(), workload.events.end(),
                     [](const Event& a, const Event& b) {
                         return a.release_s < b.release_s ||
                                (a.release_s == b.release_s && a.record_id < b.record_id);
                     });
    return workload;
}

inline Workload LoadCsv(const std::string& path, const ValidationOptions& options) {
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    if (!input.is_open()) detail::Fail(path, 0, "file", "cannot open workload for reading");
    return ParseCsv(input, options, path);
}

} // namespace edc_replay
#endif // EDC_EXTERNAL_WORKLOAD_REPLAY_H
