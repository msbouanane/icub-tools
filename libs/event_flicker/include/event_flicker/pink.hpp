// SPDX-License-Identifier: GPL-3.0-or-later
// Timestamp-free, PINK-style heuristic fallback.
// This is intentionally a different algorithm from EFR and does not claim
// equivalence to the comb-filter implementation in filter.hpp.
#pragma once

#include <event_flicker/filter.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace event_flicker {

struct PinkConfig {
    std::uint32_t width = 640;
    std::uint32_t height = 480;
    // How many events are processed before deciding whether they look like flicker.
    std::size_t packet_size = 128;
    // Maximum packet history retained for the temporary balanced-polarity test.
    std::size_t window_events = 512;
    // Ratio of overlap between positive and negative activity required to flag a pixel.
    double flicker_ratio = 0.5;
    // Minimum per-pixel overlap count needed before suppression kicks in.
    std::size_t min_overlap = 2;
};

// Heuristic timestamp-free anti-flicker fallback.
// It accumulates short packets of events and suppresses pixels whose recent
// positive/negative activity is strongly balanced, which resembles the PINK-style
// polarity-overlap idea but is not equivalent to the time-delay comb filter.
class PinkFilter {
public:
    using Emit = std::function<void(const Event&)>;

    explicit PinkFilter(PinkConfig config = {});
    ~PinkFilter();
    PinkFilter(const PinkFilter&) = delete;
    PinkFilter& operator=(const PinkFilter&) = delete;

    // Events are processed in arrival order; timestamp fields are ignored by design.
    void push(const Event& event, const Emit& onEvent);
    void flush(const Emit& onEvent);
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace event_flicker
