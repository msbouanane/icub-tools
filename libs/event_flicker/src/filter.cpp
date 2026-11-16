// SPDX-License-Identifier: GPL-3.0-or-later
// Wang et al., ICRA 2022, Eq. (7), implemented with causal time bins.
#include <event_flicker/filter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace event_flicker {
struct Filter::Impl {
    struct Cell {
        explicit Cell(unsigned size) : values(size), queued(size) {}
        std::vector<double> values;
        std::vector<unsigned char> queued;
        std::size_t pending = 0;
    };
    Config cfg;
    Stats stats;
    std::int64_t step, cursor = 0, watermark = 0;
    bool started = false, faulted = false;
    unsigned ring_size;
    std::vector<std::unique_ptr<Cell>> cells;
    std::vector<double> residual, bias;
    std::vector<std::vector<std::uint32_t>> due;

    explicit Impl(Config c) : cfg(c) {
        const auto pixels = std::uint64_t(c.width) * c.height;
        if (!pixels || pixels > 100000000 ||
            !std::isfinite(c.frequency_hz) || c.frequency_hz < 1 || c.frequency_hz > 10000 ||
            !std::isfinite(c.rho) || c.rho <= 0 || c.rho >= 1 ||
            !std::isfinite(c.threshold) || c.threshold < 0.01 ||
            c.bins_per_period < 10 || c.bins_per_period > 1000 || c.bins_per_period % 10 ||
            !std::isfinite(c.tail_epsilon) || c.tail_epsilon <= 0 || c.tail_epsilon >= c.threshold ||
            !c.max_active_pixels || !c.max_pending_bins)
            throw std::invalid_argument("Invalid event flicker filter configuration");
        step = std::llround(1e9 / (c.frequency_hz * c.bins_per_period));
        ring_size = c.bins_per_period + 1;
        cells.resize(pixels);
        residual.resize(pixels);
        bias.resize(pixels);
        due.resize(ring_size);
    }

    std::uint32_t index(std::uint32_t x, std::uint32_t y) const {
        if (x >= cfg.width || y >= cfg.height)
            throw std::invalid_argument("Event coordinate outside configured image");
        return y * cfg.width + x;
    }
    void check_time(std::int64_t t) const {
        if (faulted) throw std::runtime_error("Flicker filter faulted; reset required");
        if (t < 0 || t < watermark ||
            t > std::numeric_limits<std::int64_t>::max() - step * (ring_size + 1))
            throw std::invalid_argument("Non-monotonic or out-of-range sensor timestamp");
    }
    void add(std::uint32_t pixel, std::int64_t tick, double value) {
        if (value == 0) return;
        if (!cells[pixel]) {
            if (stats.active_pixels == cfg.max_active_pixels)
                throw std::runtime_error("Flicker filter active pixel limit exceeded");
            cells[pixel] = std::make_unique<Cell>(ring_size);
            ++stats.active_pixels;
            stats.peak_active_pixels = std::max(stats.peak_active_pixels, stats.active_pixels);
        }
        auto& cell = *cells[pixel];
        const auto slot = static_cast<unsigned>(tick % ring_size);
        if (!cell.queued[slot]) {
            if (stats.pending_bins == cfg.max_pending_bins)
                throw std::runtime_error("Flicker filter pending bin limit exceeded");
            due[slot].push_back(pixel);
            cell.queued[slot] = 1;
            ++cell.pending;
            ++stats.pending_bins;
            stats.peak_pending_bins = std::max(stats.peak_pending_bins, stats.pending_bins);
        }
        cell.values[slot] += value;
    }

    void advance(std::int64_t time, const Emit& onEvent) {
        const auto target = time / step;
        if (!started) { cursor = target; started = true; }
        while (cursor < target && stats.pending_bins) {
            const auto slot = static_cast<unsigned>(cursor % ring_size);
            for (auto pixel : due[slot]) {
                auto& cell = *cells[pixel];
                const double value = cell.values[slot];
                cell.values[slot] = 0;
                cell.queued[slot] = 0;
                --cell.pending;
                --stats.pending_bins;
                ++stats.processed_bins;
                if (std::abs(value) >= cfg.tail_epsilon) {
                    // H(z) = [(1 + z^-M + ... + z^-9M)(1-rho2*z^-M)]
                    //        / (1-rho*z^-10M), M = bins_per_period/10.
                    // The FIR part is scheduled by push(); this is feedback.
                    add(pixel, cursor + cfg.bins_per_period, cfg.rho * value);
                    residual[pixel] += value;
                    while (std::abs(residual[pixel]) >= cfg.threshold) {
                        const int sign = residual[pixel] > 0 ? 1 : -1;
                        residual[pixel] -= sign * cfg.threshold;
                        onEvent({(cursor + 1) * step, pixel % cfg.width, pixel / cfg.width, sign});
                        ++stats.output_events;
                    }
                }
                if (cell.pending == 0) {
                    cells[pixel].reset();
                    --stats.active_pixels;
                }
            }
            due[slot].clear();
            ++cursor;
        }
        if (!stats.pending_bins) cursor = target;
        watermark = time;
    }
};

Filter::Filter(Config config) : impl_(std::make_unique<Impl>(config)) {}
Filter::~Filter() = default;

void Filter::push(const Event& e, const Emit& onEvent) {
    auto& p = *impl_;
    p.check_time(e.timestamp_ns);
    const auto pixel = p.index(e.x, e.y);
    if (e.polarity != 1 && e.polarity != -1)
        throw std::invalid_argument("Event polarity must be +1 or -1");
    if (!onEvent) throw std::invalid_argument("Output callback is required");
    try {
        p.advance(e.timestamp_ns, onEvent);
        const auto tick = e.timestamp_ns / p.step;
        const double amplitude = e.polarity - p.bias[pixel];
        const double gain = (1.0 - p.cfg.rho) / 10.0;
        p.add(pixel, tick, amplitude);
        for (unsigned k = 1; k < 10; ++k)
            p.add(pixel, tick + k * (p.cfg.bins_per_period / 10), gain * amplitude);
        p.add(pixel, tick + p.cfg.bins_per_period, -(1.0 - gain) * amplitude);
        ++p.stats.input_events;
    } catch (...) { p.faulted = true; throw; }
}

void Filter::advance_to(std::int64_t time, const Emit& onEvent) {
    impl_->check_time(time);
    if (!onEvent) throw std::invalid_argument("Output callback is required");
    try { impl_->advance(time, onEvent); }
    catch (...) { impl_->faulted = true; throw; }
}

void Filter::reset() {
    auto& p = *impl_;
    for (auto& cell : p.cells) cell.reset();
    for (auto& bucket : p.due) bucket.clear();
    std::fill(p.residual.begin(), p.residual.end(), 0);
    p.stats = {};
    p.cursor = p.watermark = 0;
    p.started = p.faulted = false;
}
void Filter::set_bias(std::uint32_t x, std::uint32_t y, double bias) {
    if (!std::isfinite(bias) || std::abs(bias) > 1)
        throw std::invalid_argument("Bias must be finite and in [-1, 1]");
    impl_->bias[impl_->index(x, y)] = bias;
}
const Stats& Filter::stats() const { return impl_->stats; }
std::int64_t Filter::bin_width_ns() const { return impl_->step; }
double Filter::effective_frequency_hz() const {
    return 1e9 / (impl_->step * impl_->cfg.bins_per_period);
}

TickClock::TickClock(std::uint64_t modulus, double tick_ns)
    : modulus_(modulus), tick_ns_(tick_ns) {
    if (modulus < 2 || modulus > (std::uint64_t(1) << 32) ||
        !std::isfinite(tick_ns) || tick_ns <= 0)
        throw std::invalid_argument("Invalid sensor clock parameters");
}
std::int64_t TickClock::unwrap(std::uint64_t tick) {
    if (tick >= modulus_) throw std::invalid_argument("Sensor tick out of range");
    const auto delta = started_ ? (tick + modulus_ - previous_) % modulus_ : 0;
    if (delta >= modulus_ / 2)
        throw std::invalid_argument("Sensor clock discontinuity; reset required");
    if (delta > std::numeric_limits<std::uint64_t>::max() - elapsed_)
        throw std::overflow_error("Sensor clock overflow");
    const long double time = (static_cast<long double>(elapsed_) + delta) * tick_ns_;
    if (time >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()) - 1)
        throw std::overflow_error("Sensor clock overflow");
    elapsed_ += delta;
    previous_ = tick;
    started_ = true;
    return static_cast<std::int64_t>(std::llround(time));
}
void TickClock::reset() { previous_ = elapsed_ = 0; started_ = false; }
} // namespace event_flicker
