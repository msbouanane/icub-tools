#include <event_flicker/pink.hpp>

#include <cassert>
#include <iostream>
#include <vector>

int main() {
    event_flicker::PinkConfig cfg;
    cfg.width = 32;
    cfg.height = 32;
    cfg.packet_size = 8;
    cfg.window_events = 4;
    cfg.flicker_ratio = 0.5;
    cfg.min_overlap = 2;

    event_flicker::PinkFilter filter(cfg);
    std::vector<event_flicker::Event> output;
    auto sink = [&](const event_flicker::Event& e) { output.push_back(e); };

    for (int i = 0; i < 8; ++i) {
        filter.push({0, 10, 10, (i % 2 == 0) ? 1 : -1}, sink);
    }
    filter.flush(sink);

    // The packet is strongly polarity-balanced; the heuristic should remove most of it.
    if (output.size() > 4) {
        std::cerr << "Expected flicker-heavy packet to be suppressed, got " << output.size() << " events\n";
        return 1;
    }

    std::cout << "pink filter smoke test passed\n";
    return 0;
}
