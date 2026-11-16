# Event camera control GUI

## Purpose

A standalone Qt viewer for both locally republished event streams and their EROS representations. It subscribes only to `/primi-ball-balance/eventCam/{left,right}/raw:o` and does not create extra robot-to-laptop event connections or restart acquisition.

## Build and run

The root `CMakeLists.txt` includes the GUI and reusable flicker library. Build:

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)" --target event_control_gui
./scripts/run_event_control_gui.sh
```

Requires the existing YARP, event-driven, OpenCV, and Qt5/Qt6 Widgets development packages. Start `./run_eventCam.sh` in a separate terminal. The viewer reconnects to the local streams after acquisition restarts.

## Controls

* Four panes: left/right events on top and left/right EROS/SAE/TOS below.
* **Remove light flicker (both event views)** enables the reusable
  [`event_flicker` C++ library](../libs/event_flicker/README.md). Off by default.
  The top panes then show regenerated filtered events. The bottom panes retain
  their existing representation/salt-and-pepper processing. Raw YARP ports and
  other applications are unaffected.
* Flicker base frequency defaults to 50 Hz. Fine/medium/coarse time resolution
  selects 0.2/1/2 ms bins at 50 Hz (scaling inversely with frequency). Changes
  reset the filter and clear display history. Each camera has independent state.
* The filter runs on every event in the camera worker before display subsampling,
  outside the image mutex. Status shows warmup, output count and processing time
  per packet; a warning compares computation to the packet's sensor-time span.
  This is not an end-to-end latency or queue-depth measurement.
* Flicker removal requires real individual sensor timestamps. Its checkbox is
  disabled if `ev::ts_status` is false. It unwraps sensor ticks using
  `ev::max_stamp + 1` and `ev::tsscaler`; packet arrival time is never used for
  flicker processing. Source-clock discontinuities/long gaps reset state; invalid
  ordering, stalled timestamps or resource exhaustion stop filtered display with
  an explicit status. Toggle off/on to retry. Verify the installed library's
  timestamp constants against the actual acquisition configuration.
* The final partial bin and quiet-pixel responses advance only when new sensor
  time is available. No host-time extrapolation is used while the source is idle.
  The portable library exposes `advance_to()` for applications whose source
  supplies trustworthy sensor-clock watermarks.
* Raw accumulation is a host **arrival-time display window** in milliseconds. Event polarity is color coded.
* EROS is calculated from raw (unfiltered) events using the event-driven algorithm: multiply the event's padded spatial neighbourhood by `decay^(1/kernel)` and set the centre to 1. Changing kernel or decay resets the respective surface.
* Left/right link checkboxes synchronize sliders.
* Camera settings are sent **only upon clicking Apply camera settings**. These are hardware bias-control values, **not calibrated physical units**.

## Robot-side live control

`patches/zynqGrabber-live-bias-control.patch` is a proposed change to `robotology/event-driven`, introducing `/zynqGrabber/control/rpc` in `zynqModule.cpp`. It uses the existing camera-specific I2C operations and accepts Bottle RPC commands such as `set sensitivity left 70` and `set refractory right 1`.

Apply it in the **actual Ultrascale checkout** from which `zynqGrabber` is built:

```bash
cd /path/to/robotology-event-driven
git apply --check /path/to/primi-ball-balance/patches/zynqGrabber-live-bias-control.patch
git apply /path/to/primi-ball-balance/patches/zynqGrabber-live-bias-control.patch
# Rebuild and install zynqGrabber using the established Ultrascale process.
```

**Do not deploy without first reviewing and compiling on the installed version.** The upstream bias setters do not return register-write success, so a positive RPC reply does not prove the hardware accepted the change. Test conservatively with robot motion disabled.

## Limitations and validation

* EROS here is **independent** of `moveEnetOFK_online`: changing GUI parameters does not alter MoveEnet's own filtered EROS processing.
* The `--filter` HPU acquisition setting remains startup-only. There is no verified safe live setter yet.
* The upstream `ev::EROS` caches the decay with a function-local static, so the GUI implements its spatial update directly without that stale-parameter behavior.
* Image size is currently 640x480. Use one GUI instance at a time due to fixed YARP port names.
* The new code **has not been compiled or exercised against the robot's YARP/event-driven/Qt versions or camera hardware**.
* The standalone flicker library passes equation-reference and synthetic tests,
  but broad-field synthetic benchmarks exceeded real-time processing budgets.
  See its README for measured limits and reuse instructions. The GUI still
  refreshes every 33 ms; this refresh interval is not imposed by the library.

Manual acceptance checks: build on `iiticublap267`, display live left/right frames and event rates, change each EROS slider individually, test kernel changes without crashes, restart acquisition and check reconnection, then validate the robot-side RPC and verify sensor response before relying on live hardware control.
