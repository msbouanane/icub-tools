// SPDX-License-Identifier: GPL-3.0-or-later
#include <event_flicker/filter.hpp>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

// Adapter example: ordered "timestamp_ns x y polarity" on stdin; same format
// on stdout. "watermark timestamp_ns" drains completed bins during quiet input.
// This text demo is not intended as a high-rate transport.
int main() {
    event_flicker::Filter filter;
    auto emit = [](const event_flicker::Event& e) {
        std::cout << e.timestamp_ns << ' ' << e.x << ' ' << e.y << ' ' << e.polarity << '\n';
    };
    try {
        std::string line;
        while (std::getline(std::cin,line)) {
            std::istringstream row(line);
            if (line.rfind("watermark ",0) == 0) {
                std::string tag; std::int64_t t;
                if (!(row >> tag >> t)) throw std::runtime_error("Invalid watermark");
                filter.advance_to(t,emit);
            } else {
                event_flicker::Event e;
                if (!(row >> e.timestamp_ns >> e.x >> e.y >> e.polarity))
                    throw std::runtime_error("Expected timestamp_ns x y polarity");
                filter.push(e,emit);
            }
            std::cout.flush();
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
