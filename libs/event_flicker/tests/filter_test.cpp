// SPDX-License-Identifier: GPL-3.0-or-later
#include <event_flicker/filter.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

using namespace event_flicker;
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
template<class F> void rejects(F fn) {
    bool caught = false;
    try { fn(); } catch (const std::exception&) { caught = true; }
    require(caught, "Expected rejection");
}
Config small() { Config c; c.width = 4; c.height = 2; return c; }

// Independent dense reference: direct Eq. (8), not the factored scheduler.
void reference_equation(unsigned bins) {
    auto c = small(); c.threshold = 0.731; c.tail_epsilon = 1e-12;
    c.bins_per_period = bins;
    Filter filter(c);
    constexpr int count = 2000;
    const int d1 = bins, d2 = bins / 10;
    std::vector<double> x(count), y(count);
    std::mt19937 rng(1234);
    for (int n = 0; n < 500; ++n) x[n] = static_cast<int>(rng() % 7) - 3;
    std::vector<Event> actual, expected;
    auto emit = [&](const Event& e) { actual.push_back(e); };
    for (int n = 0; n < count; ++n) {
        for (int j = 0; j < std::abs(x[n]); ++j)
            filter.push({n * filter.bin_width_ns(), 1, 1, x[n] > 0 ? 1 : -1}, emit);
    }
    filter.advance_to(count * filter.bin_width_ns(), emit);
    auto at = [](const std::vector<double>& v, int n) { return n < 0 ? 0.0 : v[n]; };
    double residual = 0;
    const double rho2 = 1 - (1 - c.rho) / 10;
    for (int n = 0; n < count; ++n) {
        y[n] = x[n] - at(x, n-d1) - rho2*at(x, n-d2) + rho2*at(x, n-d1-d2)
            + c.rho*at(y, n-d1) + at(y, n-d2) - c.rho*at(y, n-d1-d2);
        residual += y[n];
        while (std::abs(residual) >= c.threshold) {
            int p = residual > 0 ? 1 : -1;
            expected.push_back({(n+1)*filter.bin_width_ns(), 1, 1, p});
            residual -= p*c.threshold;
        }
    }
    require(actual.size() == expected.size(), "Eq.8 output count differs");
    for (std::size_t i = 0; i < actual.size(); ++i)
        require(actual[i].timestamp_ns == expected[i].timestamp_ns &&
                actual[i].polarity == expected[i].polarity, "Eq.8 output differs");
}

void flicker_and_motion() {
    Filter filter(small());
    std::vector<Event> out;
    auto emit = [&](const Event& e) { out.push_back(e); };
    // 100Hz illumination on pixel 0; isolated moving edge crossings on pixel 1.
    for (int ms = 0; ms < 1000; ++ms) {
        if (ms % 5 == 0)
            filter.push({ms * 1000000LL, 0, 0, ms % 10 == 0 ? 1 : -1}, emit);
        if (ms == 257 || ms == 613 || ms == 887)
            filter.push({ms * 1000000LL, 1, 0, 1}, emit);
    }
    filter.advance_to(1100000000LL, emit);
    auto remaining = std::count_if(out.begin(), out.end(), [](const Event& e) {
        return e.x == 0 && e.timestamp_ns >= 200000000 && e.timestamp_ns < 1000000000;
    });
    auto motion = std::count_if(out.begin(), out.end(), [](const Event& e) {
        return e.x == 1 && e.polarity == 1;
    });
    require(remaining < 8, "Periodic flicker not attenuated by >95%");
    require(motion >= 3, "Isolated moving edges lost");
    require(std::is_sorted(out.begin(), out.end(), [](const Event& a, const Event& b) {
        return a.timestamp_ns < b.timestamp_ns;
    }), "Output order is not monotonic across pixels");
    std::cout << "100Hz flicker after warmup: " << remaining
              << "/160 events; moving-edge positive outputs: " << motion << '\n';
}

void latency_and_lifecycle() {
    Filter f(small());
    std::vector<Event> out;
    auto emit = [&](const Event& e) { out.push_back(e); };
    f.push({12345, 0, 0, 1}, emit);
    f.advance_to(f.bin_width_ns()-1, emit);
    require(out.empty(), "Incomplete bin emitted");
    f.advance_to(f.bin_width_ns(), emit);
    require(out.size() == 1 && out[0].timestamp_ns == f.bin_width_ns(), "First event latency");
    // Advances quiet pixels too; fully drain finite truncated tails, then skip a huge gap.
    f.advance_to(2000000000LL, emit);
    require(f.stats().pending_bins == 0 && f.stats().active_pixels == 0, "Tail did not drain");
    f.advance_to(1000000000000000LL, emit);
    rejects([&] { f.push({1,0,0,1},emit); });
    f.reset(); out.clear();
    f.advance_to(1000000, emit);
    require(out.empty() && f.stats().input_events == 0, "Reset leaked output");
    rejects([&] { f.push({1000000,4,0,1},emit); });
    rejects([&] { f.push({1000000,0,0,0},emit); });
    rejects([&] { f.set_bias(0,0,std::numeric_limits<double>::quiet_NaN()); });
    f.set_bias(0,0,1); f.push({1000000,0,0,1},emit); f.advance_to(2000000,emit);
    require(out.empty(), "Calibrated bias not applied");
    auto c = small(); c.max_pending_bins = 1;
    Filter bounded(c);
    rejects([&] { bounded.push({0,0,0,1},emit); });
    rejects([&] { bounded.advance_to(100000,emit); });
    bounded.reset(); bounded.set_bias(0,0,1); bounded.push({0,0,0,1},emit);
    c = small(); c.frequency_hz = std::numeric_limits<double>::infinity();
    rejects([&] { Filter invalid(c); });
    c = small(); c.bins_per_period = 11;
    rejects([&] { Filter invalid(c); });
}

void clocks_and_independence() {
    TickClock clock(1024, 80.0);
    require(clock.unwrap(1020) == 0, "Clock origin");
    require(clock.unwrap(2) == 480, "Clock wrap");
    require(clock.unwrap(2) == 480, "Equal ticks");
    rejects([&] { clock.unwrap(1); });
    clock.reset(); require(clock.unwrap(800) == 0, "Clock reset");
    rejects([&] { clock.unwrap(1024); });
    auto c = small(); c.frequency_hz = 60;
    Filter left(c), right(c);
    unsigned leftCount = 0, rightCount = 0;
    auto le = [&](const Event&) { ++leftCount; };
    auto re = [&](const Event&) { ++rightCount; };
    left.push({0,0,0,1},le);
    left.advance_to(left.bin_width_ns(),le);
    right.advance_to(right.bin_width_ns(),re);
    require(leftCount == 1 && rightCount == 0, "Stereo state crossed");
    require(left.bin_width_ns() == 166667, "60Hz time resolution");
}

void packet_boundaries() {
    Filter single(small()), batched(small());
    std::vector<Event> a, b;
    auto emitA = [&](const Event& e) { a.push_back(e); };
    auto emitB = [&](const Event& e) { b.push_back(e); };
    std::mt19937 rng(27);
    std::vector<Event> events;
    for (int i=0; i<2000; ++i)
        events.push_back({i*123456LL, static_cast<std::uint32_t>(rng()%4), 0, (rng()&1)?1:-1});
    for (std::size_t i=0; i<events.size(); ++i) {
        single.push(events[i],emitA);
        batched.push(events[i],emitB);
        if (i%17 == 0) batched.advance_to(events[i].timestamp_ns,emitB);
    }
    single.advance_to(1000000000LL,emitA);
    batched.advance_to(1000000000LL,emitB);
    require(a.size() == b.size(), "Packet boundaries changed output count");
    for (std::size_t i=0; i<a.size(); ++i)
        require(a[i].timestamp_ns == b[i].timestamp_ns && a[i].x == b[i].x &&
                a[i].polarity == b[i].polarity, "Packet boundaries changed output");
}

int main() {
    try {
        for (unsigned bins : {10u,20u,100u}) reference_equation(bins);
        flicker_and_motion(); latency_and_lifecycle(); clocks_and_independence(); packet_boundaries();
        std::cout << "All event_flicker tests passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
