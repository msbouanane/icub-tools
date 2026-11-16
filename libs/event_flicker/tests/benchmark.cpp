// SPDX-License-Identifier: GPL-3.0-or-later
#include <event_flicker/filter.hpp>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// No sleep or I/O in the measured path. One second of synthetic sensor time.
// Run per camera and concurrently for stereo; not a hardware latency guarantee.
int main(int argc, char** argv) try {
    using Clock = std::chrono::steady_clock;
    if (argc > 1 && std::string(argv[1]) == "--help") {
        std::cout << "Synthetic: benchmark [events_per_second=200000] [pixels=4096] [bins=100]\n"
                  << "Recording: benchmark --replay FILE [bins=100] [frequency_hz=50] [batch_events=1000]\n"
                  << "FILE: ordered timestamp_ns x y polarity(+1/-1), one event per line, 640x480.\n"
                  << "File loading is excluded. Replay is unpaced; queue lag is modeled, not measured hardware latency.\n";
        return 0;
    }
    const bool replay = argc > 1 && std::string(argv[1]) == "--replay";
    event_flicker::Config config;
    std::vector<event_flicker::Event> input;
    std::size_t batchSize = 1000;
    std::int64_t endTime = 1000000000;
    unsigned pixels = 0;
    if (replay) {
        if (argc < 3) throw std::invalid_argument("--replay requires a file");
        if (argc > 3) config.bins_per_period = std::stoul(argv[3]);
        if (argc > 4) config.frequency_hz = std::stod(argv[4]);
        if (argc > 5) batchSize = std::stoul(argv[5]);
        if (!batchSize) throw std::invalid_argument("Batch size must be positive");
        std::ifstream file(argv[2]);
        if (!file) throw std::runtime_error("Cannot open recording");
        std::string line;
        std::size_t lineNumber = 0;
        std::vector<bool> touched(config.width*config.height);
        while (std::getline(file,line)) {
            ++lineNumber;
            if (line.find_first_not_of(" \t\r") == std::string::npos || line[0] == '#') continue;
            std::istringstream row(line);
            event_flicker::Event e;
            std::string extra;
            if (!(row >> e.timestamp_ns >> e.x >> e.y >> e.polarity) || (row >> extra) ||
                e.timestamp_ns < 0 || e.x >= config.width || e.y >= config.height ||
                (e.polarity != 1 && e.polarity != -1) ||
                (!input.empty() && e.timestamp_ns < input.back().timestamp_ns))
                throw std::runtime_error("Invalid/unordered event at line " + std::to_string(lineNumber));
            touched[e.y*config.width+e.x] = true;
            input.push_back(e);
        }
        if (file.bad()) throw std::runtime_error("Recording read failed");
        if (input.size() < 2 || input.back().timestamp_ns == input.front().timestamp_ns)
            throw std::invalid_argument("Recording must span positive sensor time");
        endTime = input.back().timestamp_ns;
        pixels = std::count(touched.begin(),touched.end(),true);
    } else {
        const unsigned rate = argc > 1 ? std::stoul(argv[1]) : 200000;
        pixels = argc > 2 ? std::stoul(argv[2]) : 4096;
        if (!rate || !pixels || pixels > 640*480) throw std::invalid_argument("Invalid synthetic workload");
        if (argc > 3) config.bins_per_period = std::stoul(argv[3]);
        std::mt19937 random(42);
        for (unsigned i = 0; i < rate; ++i) {
            const auto pixel = static_cast<std::uint32_t>(random() % pixels);
            input.push_back({static_cast<std::int64_t>(i * 1000000000ULL / rate),
                             pixel % 640, pixel / 640, (random() & 1) ? 1 : -1});
        }
    }
    event_flicker::Filter filter(config);
    const double sensorSeconds = (endTime-input.front().timestamp_ns)/1e9;
    std::uint64_t output = 0;
    auto sink = [&](const event_flicker::Event&) { ++output; };
    std::vector<double> batchUs;
    double virtualFinish = 0, maxCompletionLag = 0;
    const auto start = Clock::now();
    for (std::size_t i = 0; i < input.size();) {
        const auto first = i;
        const auto batchStart = Clock::now();
        const auto end = i + std::min(batchSize,input.size()-i);
        for (; i < end; ++i) filter.push(input[i],sink);
        const double service = std::chrono::duration<double>(Clock::now()-batchStart).count();
        batchUs.push_back(service*1e6);
        // Idealized source delivers a complete batch when its last event occurs.
        // Accumulate measured service costs without pretending this is live I/O.
        const double arrival = (input[end-1].timestamp_ns-input.front().timestamp_ns)/1e9;
        virtualFinish = std::max(virtualFinish,arrival) + service;
        const double oldest = (input[first].timestamp_ns-input.front().timestamp_ns)/1e9;
        maxCompletionLag = std::max(maxCompletionLag,virtualFinish-oldest);
    }
    filter.advance_to(endTime,sink);
    const double seconds = std::chrono::duration<double>(Clock::now()-start).count();
    std::sort(batchUs.begin(),batchUs.end());
    std::cout << "mode=" << (replay ? "recording" : "synthetic")
              << ", input=" << input.size() << ", pixels=" << pixels << ", output=" << output
              << ", sensor_seconds=" << sensorSeconds
              << ", compute_seconds=" << seconds << ", compute_per_sensor_second=" << seconds/sensorSeconds
              << ", input_MEv/s=" << input.size()/seconds/1e6
              << ", batch_events=" << batchSize
              << ", batch_p50_us=" << batchUs[batchUs.size()/2]
              << ", batch_p99_us=" << batchUs[std::min(batchUs.size()-1,batchUs.size()*99/100)]
              << ", modeled_final_queue_ms=" << std::max(0.0,virtualFinish-sensorSeconds)*1000
              << ", modeled_max_batch_completion_ms=" << maxCompletionLag*1000
              << ", bin_us=" << filter.bin_width_ns()/1000.0
              << ", peak_active_pixels=" << filter.stats().peak_active_pixels
              << ", peak_pending_bins=" << filter.stats().peak_pending_bins << '\n';
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
