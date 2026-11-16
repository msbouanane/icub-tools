# Reusable event flicker filter

`event_flicker` is a C++17 library with **no Qt, YARP, OpenCV, Python, camera,
file-I/O, or thread dependencies**. Copy this directory into another project and
link it directly to the application that consumes the events. The GUI is one
consumer of this library, not the owner of the algorithm.

It implements the comb filter in Wang et al., *A Linear Comb Filter for Event
Flicker Removal*, ICRA 2022, equation (7):

```
H = (1 - D1) / (1 - rho1*D1) * (1 - rho2*D2) / (1 - D2)
D1 = delay(1 / frequency), D2 = delay(1 / (10*frequency))
rho2 = 1 - (1-rho1)/10
```

This is a **new, causal, time-binned implementation of the paper's equations**,
not a verbatim port or a bit-for-bit reproduction of the EFR executable. The
authors' offline queue implementation and its recording I/O are not embedded.
E2VID and the upstream dataset/sorting tools are not required.

The output contains regenerated signed events. It is **not a subset** of the
input: output timestamps/polarities can differ and the event count can increase.
Do not interpret output/input as a percentage of original events retained.

## Use in another C++ project

```cmake
add_subdirectory(path/to/event_flicker)
target_link_libraries(my_application PRIVATE event_flicker::event_flicker)
```

```cpp
#include <event_flicker/filter.hpp>

event_flicker::Config config;
config.width = 640;
config.height = 480;
config.frequency_hz = 50;
event_flicker::Filter filter(config); // One instance per camera.

event_flicker::Filter::Emit output = [&](const event_flicker::Event& event) {
    // Consume/copy event into your application's processing or bounded queue.
    // This callback runs synchronously: keep it short, nonthrowing, non-reentrant.
};

// Feed ALL events, before display decimation; use actual sensor timestamps.
filter.push({timestamp_ns, x, y, positive ? 1 : -1}, output);

// When the source guarantees no future event will precede this sensor time:
filter.advance_to(sensor_watermark_ns, output);
```

Own each instance from one thread. No internal queue, sleeps, GUI refresh, or
fixed-size packet accumulation is imposed on the caller. `push()` drains earlier
completed bins; `advance_to()` also advances quiet pixels' delayed responses.
All output is ordered across pixels within a stream; stereo cameras have
independent clocks and state. An event at the watermark is allowed, an earlier
event is rejected. After processing a packet, its final event timestamp is a
safe watermark only if subsequent packets are ordered.

**Clock contract:** use non-negative, monotonically increasing, unwrapped
nanoseconds. Equal timestamps are accepted. `TickClock` adapts a modulo sensor
counter given its modulus and nanoseconds per tick. It rejects backwards or
ambiguous jumps of half a wrap or more. Reset it and the filter on reconnect,
clock restart, replay seek, or gaps long enough to make wrap count ambiguous.
Do not use packet arrival time as a substitute for sensor timestamps.

If no events arrive, advance only using a trustworthy **sensor-clock watermark**
from the source. Otherwise delayed outputs and the last partial bin wait until
the next event. The library cannot infer elapsed sensor time from a wall clock.
For finite recordings, explicitly advance to the desired end time/tail horizon.

`reset()` discards queued responses, residuals and statistics, retaining optional
per-pixel bias calibration. For frequency/resolution changes, construct a new
instance. `set_bias(x,y,mean_polarity)` supports independent calibration; default
bias is zero. The GUI does not estimate bias from a moving scene or use upstream
dataset calibration files.

## Timing and resource trade-offs

Default `rho=0.6`, `threshold=0.7`, `frequency_hz=50` follow EFR's configuration.
The frequency is configurable; a 50 Hz base also targets its 100 Hz harmonic.
There is no automatic frequency estimation. Frequencies outside the configured
comb notches, changing LED PWM, temporal jitter and lost events can reduce
suppression. Test the actual lighting and motion of interest.

`bins_per_period` must be a multiple of 10, from 10 to 1000:

| Bins/period | Bin duration at 50 Hz | Trade-off |
|---|---|---|
| 100 (default) | 0.2 ms | Finer timestamps; more CPU and memory |
| 20 | 1 ms | Less processing and memory |
| 10 | 2 ms | Coarsest supported timing |

Bin duration is rounded to integer nanoseconds; query `bin_width_ns()` and
`effective_frequency_hz()`. Inputs inside a bin are combined; output is stamped
at its end, so it is never emitted with a future timestamp relative to the
watermark. With timely watermarks, the binning wait is at most one bin.

**That bound is not an end-to-end latency guarantee.** Add source packetization,
transport, computation, downstream queues and callback cost. The filter also
changes the temporal signal: delayed responses may occur over several cycles.
The paper's approximately 0.1 s convergence is startup settling, not a fixed
100 ms input buffer, and is not a guaranteed convergence time for every scene.

This implementation is not hard real-time: active pixel delay buffers allocate
on demand. Scheduling cost depends strongly on the number of active pixels and
phase bins, not just events/second. The default state can consume hundreds of MB
per camera for broad activity. `max_active_pixels` and `max_pending_bins` bound
active state/work; statistics expose current and peak occupancy. Vector capacity
may remain allocated after bins drain. They are not exact process-RSS limits.

Feedback tails below `tail_epsilon` (default `1e-8`) are truncated, permitting
inactive delay state to be released. The subthreshold sampler residual remains.
Resource exhaustion faults the instance and throws; reset before using it again.
Callbacks already delivered before a fault cannot be rolled back. Invalid input
coordinates/polarity/timestamps are rejected. Do not hide faults by silently
labelling unfiltered input as filtered output.

## Build and verify without robot dependencies

```bash
cmake -S libs/event_flicker -B /tmp/event-flicker-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DEVENT_FLICKER_BUILD_TESTS=ON -DEVENT_FLICKER_BUILD_EXAMPLE=ON
cmake --build /tmp/event-flicker-build -j
ctest --test-dir /tmp/event-flicker-build --output-on-failure
/tmp/event-flicker-build/event_flicker_benchmark 1000000 4096 100
```

Benchmark arguments: events per simulated second, active pixel pool, bins per
period. It measures one second of deterministic, random-polarity input without
sleep or I/O in the measured loop. Output counts include regenerated events.
Packet timing percentiles cover calls for batches of 1000 inputs. The benchmark
is a synthetic stress workload, not a recorded robot scene or live latency test.
Run two instances concurrently for a stereo CPU-load check.

### Measure a recording on the target machine

Export each camera separately as ordered text lines `timestamp_ns x y polarity`,
with polarity `+1`/`-1` and the original unwrapped sensor timestamps in
nanoseconds. The replay reader expects 640x480 coordinates, accepts blank lines
and lines starting with `#`, and rejects malformed or unordered records. This is
an explicit interchange format, not a direct reader for YARP binary dumps or
the upstream EFR text files (which use microseconds).

```bash
/tmp/event-flicker-build/event_flicker_benchmark --replay left.txt 100 50 1000
```

Arguments after the file are bins per period, base frequency in Hz, and input
events per benchmark batch. File loading is excluded from measurement. The full
recording is loaded into memory; use representative excerpts if necessary.

For stereo load, run both concurrently:

```bash
/tmp/event-flicker-build/event_flicker_benchmark --replay left.txt 100 50 1000 > /tmp/flicker-left.log &
/tmp/event-flicker-build/event_flicker_benchmark --replay right.txt 100 50 1000 > /tmp/flicker-right.log &
wait
cat /tmp/flicker-left.log /tmp/flicker-right.log
```

- `compute_per_sensor_second`: values above 1 mean computation cannot keep up
  with that recording on this machine. Values below 1 alone do not establish a
  worst-case latency guarantee.
- `batch_p50_us`, `batch_p99_us`: measured processing times per input batch,
  including synchronous filter output generation, excluding file loading.
- `modeled_final_queue_ms`, `modeled_max_batch_completion_ms`: an **idealized
  queue model**, using the measured processing costs and assuming each batch
  arrives at its last event's sensor timestamp. Completion delay is measured
  relative to the oldest event in the batch and includes batching wait. These
  are not measured live sensor-to-application latencies and do not include
  actual network delays, real packet boundaries or downstream processing.

For example, gathering 1000 events at 200 kEv/s already takes about 5 ms before
filtering. Use the actual source packet size/cadence when interpreting the model;
do not equate the filter's 0.2 ms bin width with the full application delay.
The model also excludes any trailing partial-bin wait and filter phase response.

Compare raw/filtered activity in the same lighting scene as well as timing:
motion without flicker, flicker without motion, and both together. Performance
alone does not validate flicker suppression or preservation of useful motion.

If CMake is unavailable, from the repository root:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Ilibs/event_flicker/include libs/event_flicker/src/filter.cpp \
  libs/event_flicker/tests/filter_test.cpp -o /tmp/event_flicker_tests
/tmp/event_flicker_tests
```

Tests compare output to an independent dense implementation of paper equation
(8) at all three GUI resolutions; test periodic flicker plus isolated moving
edges, causal bin completion, cross-pixel ordering, packet boundaries, reset,
resource limits, invalid inputs, calibration, clock wrap and stereo independence.

`event_flicker_stream` demonstrates a standalone stdin/stdout adapter using lines
`timestamp_ns x y polarity` and `watermark timestamp_ns`. It does no camera I/O.
Text parsing/flushing is illustrative, not recommended as a high-rate transport.

## Initial measurements (2026-11-16)

GCC 13.3, `-O3 -DNDEBUG`, this development environment, one filter. These are
individual runs, not certified throughput limits for the robot laptop:

| Input (events/s) | Pixel pool | Bins | Compute time for 1 s of data |
|---|---|---|---|
| 200,000 | 4,096 | 100 | 0.279 s |
| 1,000,000 | 4,096 | 100 | 0.474 s |
| 1,000,000 | 100,000 | 100 | 13.144 s |
| 1,000,000 | 100,000 | 20 | 3.514 s |
| 1,000,000 | 100,000 | 10 | 1.458 s |

A subsequent concurrent stereo run (one process per camera, same deterministic
input workload in both, one-second excerpts) measured:

| Input per camera (events/s) | Pixel pool per camera | Bins | Left compute time | Right compute time |
|---|---|---|---|---|
| 200,000 | 4,096 | 100 | 0.167 s | 0.169 s |
| 1,000,000 | 4,096 | 100 | 0.242 s | 0.234 s |
| 1,000,000 | 100,000 | 100 | 15.184 s | 15.321 s |
| 1,000,000 | 100,000 | 10 | 2.008 s | 2.003 s |

These runs occurred at different times from the single-camera baseline; runtime
varies with host scheduling/load. Do not interpret the lower localized timings
as a stereo speedup. They are processing-time measurements, not live event delay.

**Broad-field tests did not keep up with real time**, even at coarse resolution.
Do not deploy this as a real-time front end without representative stereo replay
on the target machine, sustained queue/backlog measurement, and a latency budget.
If these limits are exceeded, further optimization or another implementation is
needed; adding a queue only increases delay. The GUI reports packet processing
time and flags processing slower than the packet's sensor-time span.

The synthetic 100 Hz test emitted zero flicker events in the 0.2–1.0 s interval
(160 raw events), while three isolated positive moving-edge inputs generated four
positive outputs. This verifies a simple controlled case, not real-camera
performance. No upstream dataset or robot recording has been tested here.

## References and license

- [Paper, Eq. (7)/(8)](https://arxiv.org/html/2205.08090)
- [EFR source/configuration, revision a0e0189](https://github.com/ziweiWWANG/EFR/tree/a0e018912ee1995fd734309437b44ca0ae0680c4)
- Citation: Wang, Yuan, Ng and Mahony, “A Linear Comb Filter for Event Flicker
  Removal,” ICRA 2022, pp. 398–404.

This component is provided under GPL-3.0-or-later; see `COPYING`, consistent with
the upstream source license. EFR's README also labels its release “For academic
use only.” This directory contains a new equation-based implementation; it does
not claim identical behavior, endorsement, or the paper's reported accuracy.
