// SPDX-License-Identifier: GPL-3.0-or-later
// Equation-based EFR implementation; see README.md for provenance and limits.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace event_flicker {

// One stream/clock per Filter. Coordinates are zero-based; polarity is +/-1.
// Use sensor time, unwrapped into a non-negative monotonic nanosecond clock.
struct Event {
    std::int64_t timestamp_ns;
    std::uint32_t x, y;
    int polarity;
};

struct Config {
    std::uint32_t width = 640, height = 480;
    double frequency_hz = 50.0;
    double rho = 0.6;
    double threshold = 0.7;
    // Multiple of 10. Default: 200 us bins at 50 Hz, 166.667 us at 60 Hz.
    unsigned bins_per_period = 100;
    // Stop scheduling a feedback tail once its bin amplitude falls below this.
    double tail_epsilon = 1e-8;
    // Explicit resource ceilings: exceeding either faults the filter, never
    // silently drops events. reset() is required before subsequent processing.
    std::size_t max_active_pixels = 640 * 480;
    std::size_t max_pending_bins = 8 * 1024 * 1024;
};

struct Stats {
    std::uint64_t input_events = 0, output_events = 0, processed_bins = 0;
    std::size_t active_pixels = 0, pending_bins = 0;
    std::size_t peak_active_pixels = 0, peak_pending_bins = 0;
};

// Causal, time-binned implementation of Wang et al., ICRA 2022, Eq. (7).
// This is not a bit-for-bit port of the authors' offline queue implementation.
// Single-thread owned. The synchronous emit callback must not throw or re-enter.
class Filter {
public:
    using Emit = std::function<void(const Event&)>;
    explicit Filter(Config config = {});
    ~Filter();
    Filter(const Filter&) = delete;
    Filter& operator=(const Filter&) = delete;

    // Emits completed bins BEFORE ingesting this event. Equal input timestamps
    // are allowed. Output is regenerated (+/-1), not a subset of input events.
    void push(const Event& event, const Emit& emit);
    // Declare that no future input precedes watermark_ns. Emits bins ending at
    // or before it, including delayed responses at pixels with no new input.
    // Never advance using host arrival time unless mapped to the sensor clock.
    void advance_to(std::int64_t watermark_ns, const Emit& emit);
    void reset();
    // Optional independently calibrated mean polarity in [-1,1]. Applied to
    // future input only; defaults to zero. Not estimated from moving scenes.
    void set_bias(std::uint32_t x, std::uint32_t y, double bias);
    const Stats& stats() const;
    std::int64_t bin_width_ns() const;
    double effective_frequency_hz() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Adapter for modulo sensor counters (e.g. iCub AE ticks). Supply the modulus
// and measured/documented ns per tick; returns relative nanoseconds. Rejects
// ambiguous jumps >= half a wrap. Reset after reconnect or a long data gap.
class TickClock {
public:
    TickClock(std::uint64_t modulus, double tick_ns);
    std::int64_t unwrap(std::uint64_t tick);
    void reset();
private:
    std::uint64_t modulus_, previous_ = 0, elapsed_ = 0;
    double tick_ns_;
    bool started_ = false;
};
} // namespace event_flicker
