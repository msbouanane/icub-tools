// SPDX-License-Identifier: GPL-3.0-or-later
// Heuristic, timestamp-free anti-flicker fallback based on polarity overlap.
#include <event_flicker/pink.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace event_flicker {

struct PinkFilter::Impl {
    PinkConfig cfg;
    std::vector<unsigned> pos_counts;
    std::vector<unsigned> neg_counts;
    std::deque<Event> pending;
    std::size_t packet_events = 0;

    explicit Impl(PinkConfig c) : cfg(c) {
        if (!c.width || !c.height || !c.packet_size || !c.window_events)
            throw std::invalid_argument("Invalid PINK filter configuration");
        const auto pixels = std::size_t(c.width) * std::size_t(c.height);
        pos_counts.assign(pixels, 0);
        neg_counts.assign(pixels, 0);
    }

    std::size_t index(std::uint32_t x, std::uint32_t y) const {
        if (x >= cfg.width || y >= cfg.height)
            throw std::invalid_argument("Event coordinate outside configured image");
        return std::size_t(y) * std::size_t(cfg.width) + std::size_t(x);
    }

    void reset() {
        std::fill(pos_counts.begin(), pos_counts.end(), 0);
        std::fill(neg_counts.begin(), neg_counts.end(), 0);
        pending.clear();
        packet_events = 0;
    }

    void decay_recent_window() {
        // Keep the recent polarity overlap estimates stable without timestamps.
        // A light decay prevents the fallback from overreacting to a single burst.
        const auto decay = std::max<std::size_t>(1, cfg.window_events / 8);
        for (std::size_t i = 0; i < pos_counts.size(); ++i) {
            if (pos_counts[i] > static_cast<unsigned>(decay)) {
                pos_counts[i] -= std::min(pos_counts[i], static_cast<unsigned>(decay));
            }
            if (neg_counts[i] > static_cast<unsigned>(decay)) {
                neg_counts[i] -= std::min(neg_counts[i], static_cast<unsigned>(decay));
            }
        }
    }

    void flush(const Emit& onEvent) {
        if (pending.empty()) return;

        std::vector<unsigned> local_pos(pos_counts.size(), 0);
        std::vector<unsigned> local_neg(pos_counts.size(), 0);
        for (const auto& event : pending) {
            const auto idx = index(event.x, event.y);
            if (event.polarity > 0) ++local_pos[idx];
            else ++local_neg[idx];
        }

        std::vector<unsigned char> keep(pending.size(), 1);
        for (std::size_t i = 0; i < pending.size(); ++i) {
            const auto& event = pending[i];
            const auto idx = index(event.x, event.y);
            const unsigned pos = local_pos[idx];
            const unsigned neg = local_neg[idx];
            const unsigned total = pos + neg;
            const auto overlap = (total > 0 && pos > 0 && neg > 0)
                ? std::min(pos, neg) / static_cast<double>(total)
                : 0.0;
            const bool flicker_like = (pos >= cfg.min_overlap) && (neg >= cfg.min_overlap) &&
                (overlap >= cfg.flicker_ratio);
            if (flicker_like) keep[i] = 0;
        }

        for (std::size_t i = 0; i < pending.size(); ++i) {
            if (keep[i]) onEvent(pending[i]);
        }

        for (const auto& event : pending) {
            const auto idx = index(event.x, event.y);
            if (event.polarity > 0) ++pos_counts[idx];
            if (event.polarity < 0) ++neg_counts[idx];
        }

        decay_recent_window();
        pending.clear();
        packet_events = 0;
    }
};

PinkFilter::PinkFilter(PinkConfig config) : impl_(std::make_unique<Impl>(config)) {}
PinkFilter::~PinkFilter() = default;

void PinkFilter::push(const Event& event, const Emit& onEvent) {
    if (!onEvent) throw std::invalid_argument("Output callback is required");
    if (event.polarity != 1 && event.polarity != -1)
        throw std::invalid_argument("Event polarity must be +1 or -1");

    impl_->pending.push_back(event);
    ++impl_->packet_events;

    if (impl_->packet_events >= impl_->cfg.packet_size) {
        impl_->flush(onEvent);
    }
}

void PinkFilter::flush(const Emit& onEvent) {
    if (!onEvent) throw std::invalid_argument("Output callback is required");
    impl_->flush(onEvent);
}

void PinkFilter::reset() {
    impl_->reset();
}

} // namespace event_flicker
