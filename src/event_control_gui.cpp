// Local stereo event viewer. Acquisition and camera-hardware control are separate.
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <event-driven/vis/filters.h>
#include <event_flicker/filter.hpp>
#include <event_flicker/pink.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QSlider>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <mutex>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
constexpr int width = 640, height = 480;
const char* sources[2] = {
    "/primi-ball-balance/eventCam/left/raw:o",
    "/primi-ball-balance/eventCam/right/raw:o"
};
const char* inputs[2] = {
    "/primi-ball-balance/eventControl/left/events:i",
    "/primi-ball-balance/eventControl/right/events:i"
};
using Clock = std::chrono::steady_clock;

struct DisplayEvent {
    uint16_t x, y;
    bool polarity;
};
struct DisplayPacket {
    Clock::time_point received;
    std::vector<DisplayEvent> events;
};

struct Camera {
    ev::BufferedPort<ev::AE> input;
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<int> kernel{7}, decayMilli{300}, mode{0}, windowMs{30};
    std::atomic<bool> denoise{false};
    std::atomic<bool> removeFlicker{false};
    std::atomic<int> flickerMode{1};
    std::atomic<int> flickerHz{50};
    std::atomic<int> flickerBins{100};
    std::mutex mutex;
    std::deque<DisplayPacket> history;
    std::size_t historyCount{0};
    ev::EROS eros;
    ev::SAE sae;
    ev::TOS tos;
    ev::vNoiseFilter filter;
    int currentKernel{7}, currentDecay{300}, currentMode{0};
    bool currentDenoise{false};
    double lastEventTime{0}, eventRate{0}, acceptedPercent{100};
    Clock::time_point lastPacket{};
    QString flickerStatus{"off"}; // protected by mutex, like image history

    // Filter and sensor clock are worker-owned, never touched by the UI thread.
    std::unique_ptr<event_flicker::Filter> flicker;
    std::unique_ptr<event_flicker::TickClock> flickerClock;
    std::unique_ptr<event_flicker::PinkFilter> pinkFlicker;
    bool currentFlicker{false};
    int currentFlickerMode{1};
    int currentFlickerHz{50};
    int currentFlickerBins{100};
    bool flickerFault{false};
    Clock::time_point lastFlickerPacket{};
    Clock::time_point lastClockAdvance{};
    double lastSourceEnvelope{0};
    std::int64_t lastFlickerTime{0};

    void configureFlicker() {
        const bool enabled = removeFlicker.load();
        const int mode = ev::ts_status ? flickerMode.load() : 1;
        const int hz = flickerHz.load();
        const int bins = flickerBins.load();
        if (enabled == currentFlicker && mode == currentFlickerMode &&
            hz == currentFlickerHz && bins == currentFlickerBins) return;
        currentFlicker = enabled;
        currentFlickerMode = mode;
        currentFlickerHz = hz;
        currentFlickerBins = bins;
        flicker.reset();
        flickerClock.reset();
        pinkFlicker.reset();
        flickerFault = false;
        lastFlickerPacket = {};
        lastClockAdvance = {};
        lastFlickerTime = 0;
        lastSourceEnvelope = 0;
        QString status = "off";
        if (enabled) {
            try {
                if (mode == 0) {
                    if (!ev::ts_status)
                        throw std::runtime_error("sensor timestamps required for EFR");
                    event_flicker::Config config;
                    config.width = width;
                    config.height = height;
                    config.frequency_hz = hz;
                    config.bins_per_period = bins;
                    flicker = std::make_unique<event_flicker::Filter>(config);
                    flickerClock = std::make_unique<event_flicker::TickClock>(
                        std::uint64_t(ev::max_stamp) + 1, ev::tsscaler * 1e9);
                    status = "EFR warming up";
                } else {
                    event_flicker::PinkConfig config;
                    config.width = width;
                    config.height = height;
                    config.packet_size = 64;
                    config.window_events = 512;
                    config.flicker_ratio = 0.5;
                    config.min_overlap = 2;
                    pinkFlicker = std::make_unique<event_flicker::PinkFilter>(config);
                    status = "PINK warming up";
                }
            } catch (const std::exception& e) {
                flickerFault = true;
                status = QString("unavailable: ") + e.what();
            }
        }
        std::lock_guard<std::mutex> lock(mutex);
        history.clear();
        historyCount = 0;
        flickerStatus = status;
    }

    // Operates BEFORE any display decimation and OUTSIDE the image mutex.
    // Callbacks feed only the event display; the portable library emits complete
    // timestamped events suitable for application consumers as well.
    void filterDisplay(ev::packet<ev::AE>& packet, DisplayPacket& display,
                       Clock::time_point now, std::size_t stride) {
        std::size_t emitted = 0;
        event_flicker::Filter::Emit onEvent = [&](const event_flicker::Event& e) {
            if (emitted++ % stride == 0 && display.events.size() < 250000)
                display.events.push_back({static_cast<uint16_t>(e.x),
                    static_cast<uint16_t>(e.y), e.polarity > 0});
        };
        if (!currentFlicker) {
            for (const auto& e : packet) {
                if (e.x >= width || e.y >= height) continue;
                onEvent({0, e.x, e.y, e.p ? 1 : -1});
            }
            return;
        }
        if (flickerFault) return; // Never label raw fallback as filtered.
        QString status;
        const auto started = Clock::now();
        try {
            if (currentFlickerMode == 1) {
                if (!pinkFlicker) throw std::runtime_error("PINK filter not configured");
                for (const auto& e : packet) {
                    if (e.x >= width || e.y >= height) continue;
                    pinkFlicker->push({0, e.x, e.y, e.p ? 1 : -1}, onEvent);
                }
                pinkFlicker->flush(onEvent);
                status = QString("PINK fallback, %1 out").arg(static_cast<qulonglong>(emitted));
            } else {
                if (!flicker || !flickerClock) throw std::runtime_error("EFR filter not configured");
                const double envelope = packet.envelope().getTime();
                const double halfWrapSeconds = (std::uint64_t(ev::max_stamp) + 1) * ev::tsscaler / 2;
                const double gapLimit = std::min(0.5, halfWrapSeconds);
                const bool gap = lastFlickerPacket != Clock::time_point{} &&
                    std::chrono::duration<double>(now - lastFlickerPacket).count() >= gapLimit;
                const bool sourceJump = lastSourceEnvelope > 0 && envelope > 0 &&
                    (envelope < lastSourceEnvelope || envelope - lastSourceEnvelope >= gapLimit);
                if (gap || sourceJump) {
                    flicker->reset();
                    flickerClock->reset();
                    lastClockAdvance = {};
                    lastFlickerTime = 0;
                    std::lock_guard<std::mutex> lock(mutex);
                    history.clear(); historyCount = 0;
                }
                bool advancedClock = false;
                std::int64_t firstTime = -1;
                std::size_t checked = 0;
                for (const auto& e : packet) {
                    if ((checked++ % 256) == 0 && (!running.load() || !removeFlicker.load() ||
                        flickerHz.load() != currentFlickerHz || flickerBins.load() != currentFlickerBins)) {
                        display.events.clear();
                        return; // The next worker iteration applies the new setting.
                    }
                    const auto t = flickerClock->unwrap(e.ts);
                    if (firstTime < 0) firstTime = t;
                    advancedClock = advancedClock || t > lastFlickerTime;
                    lastFlickerTime = t;
                    if (e.x >= width || e.y >= height) continue;
                    flicker->push({t, e.x, e.y, e.p ? 1 : -1}, onEvent);
                }
                // The newest SENSOR timestamp is a watermark; host arrival time is
                // not. A final partial bin waits for the next packet (or watermark).
                if (firstTime >= 0) flicker->advance_to(lastFlickerTime, onEvent);
                if (advancedClock || lastClockAdvance == Clock::time_point{}) lastClockAdvance = now;
                if (now - lastClockAdvance >= std::chrono::milliseconds(100))
                    throw std::runtime_error("sensor timestamps are not advancing");
                lastFlickerPacket = now;
                lastSourceEnvelope = envelope;
                const double ms = std::chrono::duration<double,std::milli>(Clock::now()-started).count();
                status = QString("%1, %2 Hz, %3 ms/packet, %4 out")
                    .arg(lastFlickerTime < 100000000 ? "warming up" : "on")
                    .arg(currentFlickerHz).arg(ms, 0, 'f', 2)
                    .arg(static_cast<qulonglong>(emitted));
                const double sourceSpanMs = (lastFlickerTime - firstTime) / 1e6;
                if (sourceSpanMs > 0 && ms > sourceSpanMs)
                    status += " (slower than this packet's sensor span)";
            }
        } catch (const std::exception& e) {
            flickerFault = true;
            display.events.clear();
            status = QString("stopped: %1; toggle off/on to reset").arg(e.what());
            std::lock_guard<std::mutex> lock(mutex);
            history.clear(); historyCount = 0;
        }
        std::lock_guard<std::mutex> lock(mutex);
        flickerStatus = status;
    }

    Camera() { reset(); }

    void reset() {
        eros.init(width, height, currentKernel, currentDecay / 1000.0);
        sae.init(width, height);
        tos.init(width, height, currentKernel, 2.0);
        filter = ev::vNoiseFilter();
        filter.initialise(width, height);
        filter.use_spatial_filter(0.005, 1);
        lastEventTime = 0;
    }

    bool start(int side) {
        if (!input.open(inputs[side])) return false;
        running = true;
        worker = std::thread([this, side] {
            bool sourceAvailable = false;
            auto nextConnect = Clock::time_point{};
            auto lastRate = Clock::now();
            uint64_t seen = 0, kept = 0;
            while (running.load()) {
                configureFlicker();
                auto now = Clock::now();
                // Connect only when the source becomes available. Do not repeatedly
                // connect on transient shmem status checks.
                if (now >= nextConnect) {
                    const bool available = yarp::os::Network::exists(sources[side]);
                    if (available && !sourceAvailable)
                        yarp::os::Network::connect(sources[side], inputs[side], "shmem");
                    sourceAvailable = available;
                    nextConnect = now + std::chrono::seconds(2);
                }
                auto* packet = input.read(false);
                if (packet && packet->size()) {
                    // YARP packet envelope is the reference time. With enabled
                    // event timestamps, recover each event's relative offset.
                    double packetTime = packet->envelope().getTime();
                    if (packetTime <= 0)
                        packetTime = std::chrono::duration<double>(now.time_since_epoch()).count();
                    int lastTick = 0;
                    for (const auto& event : *packet) lastTick = event.ts;
                    DisplayPacket display{now, {}};
                    const std::size_t stride = std::max<std::size_t>(1, packet->size() / 2000);
                    display.events.reserve(packet->size() / stride + 1);

                    filterDisplay(*packet, display, now, stride);

                    std::lock_guard<std::mutex> lock(mutex);
                    int k = kernel.load(), d = decayMilli.load(), m = mode.load();
                    bool noise = denoise.load();
                    if (k != currentKernel || d != currentDecay ||
                        m != currentMode || noise != currentDenoise) {
                        currentKernel = k;
                        currentDecay = d;
                        currentMode = m;
                        currentDenoise = noise;
                        reset();
                    }
                    for (const auto& e : *packet) {
                        const int x = e.x, y = e.y;
                        if (x < 0 || x >= width || y < 0 || y >= height) continue;
                        ++seen;
                        double t = packetTime;
                        if (ev::ts_status) {
                            const double age = ev::deltaS(lastTick, static_cast<int>(e.ts));
                            if (age >= 0 && age < 2.0) t -= age;
                        }
                        if (noise && !filter.check(x, y, e.p, t)) continue;
                        ++kept;
                        lastEventTime = std::max(lastEventTime, t);
                        if (m == 0) eros.update(x, y);
                        else if (m == 1) sae.update(x, y, t);
                        else tos.update(x, y);
                    }
                    historyCount += display.events.size();
                    history.push_back(std::move(display));
                    // Bound display memory even at very high event rates.
                    while (historyCount > 250000 && !history.empty()) {
                        historyCount -= history.front().events.size();
                        history.pop_front();
                    }
                    lastPacket = now;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                now = Clock::now();
                if (now - lastRate >= std::chrono::milliseconds(500)) {
                    std::lock_guard<std::mutex> lock(mutex);
                    const double dt = std::chrono::duration<double>(now - lastRate).count();
                    eventRate = seen / dt;
                    acceptedPercent = seen ? 100.0 * kept / seen : 100.0;
                    seen = kept = 0;
                    lastRate = now;
                }
            }
        });
        return true;
    }

    void stop() {
        running = false;
        input.interrupt();
        if (worker.joinable()) worker.join();
        input.close();
    }

    void snapshot(int rawMode, QImage& rawImage, QImage& representationImage,
                  double& rate, double& accepted, bool& receiving, QString& flickerInfo) {
        auto now = Clock::now();
        cv::Mat raw(height, width, CV_8UC3,
                    rawMode == 2 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255));
        cv::Mat processed;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto cutoff = now - std::chrono::milliseconds(windowMs.load());
            while (!history.empty() && history.front().received < cutoff) {
                historyCount -= history.front().events.size();
                history.pop_front();
            }
            for (const auto& pkt : history)
                for (const auto& e : pkt.events) {
                    auto& pixel = raw.at<cv::Vec3b>(e.y, e.x);
                    if (rawMode == 0)
                        pixel = e.polarity ? cv::Vec3b(40, 40, 220)
                                           : cv::Vec3b(220, 110, 30);
                    else if (rawMode == 1)
                        pixel = cv::Vec3b(0, 0, 0);
                    else
                        pixel = cv::Vec3b(255, 255, 255);
                }
            if (currentMode == 0) {
                eros.getSurface().convertTo(processed, CV_8U, 255.0);
            } else if (currentMode == 1) {
                // SAE is a timestamp surface; only its DISPLAY mapping uses
                // exponential temporal decay. This does not modify ev::SAE.
                cv::Mat stamps = sae.getSurface();
                cv::Mat age = lastEventTime - stamps;
                cv::max(age, cv::Scalar(0), age);
                cv::exp(-age / 0.1, age);
                age.setTo(0, stamps == 0);
                age.convertTo(processed, CV_8U, 255.0);
            } else {
                tos.getSurface().convertTo(processed, CV_8U, 255.0);
            }
            rate = eventRate;
            flickerInfo = flickerStatus;
            accepted = acceptedPercent;
            receiving = lastPacket != Clock::time_point{} &&
                        now - lastPacket < std::chrono::seconds(2);
        }
        cv::cvtColor(raw, raw, cv::COLOR_BGR2RGB);
        rawImage = QImage(raw.data, width, height, int(raw.step), QImage::Format_RGB888).copy();
        representationImage = QImage(processed.data, width, height, int(processed.step),
                                     QImage::Format_Grayscale8).copy();
    }
};

QWidget* panel(const QString& title, QLabel*& view) {
    auto* widget = new QWidget;
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(4, 4, 4, 4);
    auto* name = new QLabel(title);
    name->setAlignment(Qt::AlignCenter);
    view = new QLabel;
    view->setAlignment(Qt::AlignCenter);
    view->setMinimumSize(240, 180);
    view->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    layout->addWidget(name);
    layout->addWidget(view, 1);
    return widget;
}

QWidget* slider(int lo, int hi, int initial, QSlider*& control) {
    auto* widget = new QWidget;
    auto* row = new QHBoxLayout(widget);
    row->setContentsMargins(0, 0, 0, 0);
    control = new QSlider(Qt::Horizontal);
    control->setRange(lo, hi);
    control->setValue(initial);
    auto* number = new QSpinBox;
    number->setRange(lo, hi);
    number->setValue(initial);
    QObject::connect(control, &QSlider::valueChanged, number, &QSpinBox::setValue);
    QObject::connect(number, QOverload<int>::of(&QSpinBox::valueChanged),
                     control, &QSlider::setValue);
    row->addWidget(control, 1);
    row->addWidget(number);
    return widget;
}

class Window : public QMainWindow {
    Camera cameras[2];
    QLabel *rawViews[2]{}, *processedViews[2]{};
    QComboBox *rawModes[2]{}, *representations[2]{};
    QCheckBox *noise[2]{}, *linkHardware{}, *linkEros{};
    QCheckBox* removeFlicker{};
    QComboBox* flickerModeSelector{};
    QSpinBox* flickerFrequency{};
    QComboBox* flickerResolution{};
    QLabel* flickerInfo{};
    QSlider *sensitivity[2]{}, *refractory[2]{}, *kernel[2]{}, *decay[2]{}, *accumulation{};
    QPushButton* apply{};
    yarp::os::RpcClient rpc;
    std::future<QString> pending;
    QString rpcMessage;
    Clock::time_point rpcMessageUntil{};

public:
    Window() {
        setWindowTitle("PRIMI - Event Cameras and EROS");
        resize(1300, 960);
        auto* central = new QWidget;
        auto* root = new QVBoxLayout(central);
        auto* grid = new QGridLayout;
        for (int s = 0; s < 2; ++s) {
            grid->addWidget(panel(s ? "RIGHT EVENTS" : "LEFT EVENTS", rawViews[s]), 0, s);
            grid->addWidget(panel(s ? "RIGHT REPRESENTATION" : "LEFT REPRESENTATION",
                                  processedViews[s]), 1, s);
        }
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 1);
        root->addLayout(grid, 1);

        auto* settings = new QHBoxLayout;
        auto* hardwareBox = new QGroupBox("Camera hardware (Ultrascale I2C RPC)");
        auto* hardware = new QFormLayout(hardwareBox);
        linkHardware = new QCheckBox("Link cameras");
        linkHardware->setChecked(true);
        hardware->addRow(linkHardware);
        for (int s = 0; s < 2; ++s) {
            hardware->addRow(s ? "Right sensitivity" : "Left sensitivity",
                             slider(10, 90, 70, sensitivity[s]));
            hardware->addRow(s ? "Right refractory bias" : "Left refractory bias",
                             slider(1, 99, 1, refractory[s]));
        }
        apply = new QPushButton("Apply camera settings");
        hardware->addRow(apply);
        settings->addWidget(hardwareBox);

        auto* processingBox = new QGroupBox("Local representation and filtering");
        auto* processing = new QFormLayout(processingBox);
        linkEros = new QCheckBox("Link EROS parameters");
        linkEros->setChecked(true);
        processing->addRow(linkEros);
        removeFlicker = new QCheckBox("Remove light flicker (both event views)");
        removeFlicker->setToolTip("Choose the flicker filter mode below. EFR needs sensor timestamps; PINK works without them.");
        removeFlicker->setEnabled(true);
        processing->addRow(removeFlicker);
        flickerModeSelector = new QComboBox;
        flickerModeSelector->addItems({"EFR comb filter", "PINK fallback"});
        flickerModeSelector->setCurrentIndex(1);
        if (!ev::ts_status) {
            flickerModeSelector->setCurrentIndex(1);
            flickerModeSelector->setEnabled(false);
            flickerModeSelector->setToolTip("PINK fallback is active because this build has no sensor timestamps.");
        } else {
            flickerModeSelector->setToolTip("Use the original EFR comb filter or the timestamp-free PINK heuristic fallback.");
        }
        processing->addRow("Flicker mode", flickerModeSelector);
        flickerFrequency = new QSpinBox;
        flickerFrequency->setRange(10, 1000);
        flickerFrequency->setValue(50);
        flickerFrequency->setSuffix(" Hz");
        processing->addRow("Flicker base frequency", flickerFrequency);
        flickerResolution = new QComboBox;
        flickerResolution->addItems({"Fine: 0.2 ms at 50 Hz", "Medium: 1 ms at 50 Hz", "Coarse: 2 ms at 50 Hz"});
        flickerResolution->setToolTip("Coarser bins reduce CPU/memory cost but lose temporal detail. Bin duration scales inversely with frequency.");
        processing->addRow("Flicker time resolution", flickerResolution);
        flickerInfo = new QLabel("Flicker removal off");
        flickerInfo->setWordWrap(true);
        processing->addRow(flickerInfo);
        connect(removeFlicker, &QCheckBox::toggled, this, [this](bool enabled) {
            for (auto& c : cameras) c.removeFlicker = enabled;
        });
        connect(flickerModeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
            if (!ev::ts_status && index == 0) {
                flickerModeSelector->setCurrentIndex(1);
                return;
            }
            for (auto& c : cameras) c.flickerMode = index;
        });
        connect(flickerFrequency, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int hz) {
            for (auto& c : cameras) c.flickerHz = hz;
        });
        connect(flickerResolution, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
            const int bins[] = {100, 20, 10};
            for (auto& c : cameras) c.flickerBins = bins[index];
        });
        for (int s = 0; s < 2; ++s) {
            auto* modes = new QWidget;
            auto* row = new QHBoxLayout(modes);
            row->setContentsMargins(0, 0, 0, 0);
            rawModes[s] = new QComboBox;
            rawModes[s]->addItems({"Polarity", "Black on white", "White on black"});
            representations[s] = new QComboBox;
            representations[s]->addItems({"EROS", "SAE", "TOS"});
            row->addWidget(rawModes[s]);
            row->addWidget(representations[s]);
            processing->addRow(s ? "Right display" : "Left display", modes);
            noise[s] = new QCheckBox("Salt-and-pepper filter before representation");
            processing->addRow(noise[s]);
            processing->addRow(s ? "Right kernel (odd)" : "Left kernel (odd)",
                               slider(1, 15, 7, kernel[s]));
            processing->addRow(s ? "Right EROS decay x1000" : "Left EROS decay x1000",
                               slider(1, 999, 300, decay[s]));
        }
        processing->addRow("Raw display window (ms)", slider(5, 200, 30, accumulation));
        settings->addWidget(processingBox);
        root->addLayout(settings);
        setCentralWidget(central);

        for (int s = 0; s < 2; ++s) {
            const int other = 1 - s;
            connect(sensitivity[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkHardware->isChecked()) sensitivity[other]->setValue(n);
            });
            connect(refractory[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkHardware->isChecked()) refractory[other]->setValue(n);
            });
            connect(kernel[s], &QSlider::valueChanged, this, [=](int n) {
                if (!(n & 1)) {
                    kernel[s]->setValue(n > cameras[s].kernel.load() ? n + 1 : n - 1);
                    return;
                }
                if (linkEros->isChecked()) kernel[other]->setValue(n);
                cameras[s].kernel = n;
            });
            connect(decay[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkEros->isChecked()) decay[other]->setValue(n);
                cameras[s].decayMilli = n;
            });
            connect(representations[s], QOverload<int>::of(&QComboBox::currentIndexChanged),
                    this, [=](int n) { cameras[s].mode = n; });
            connect(noise[s], &QCheckBox::toggled, this,
                    [=](bool checked) { cameras[s].denoise = checked; });
            if (!cameras[s].start(s))
                statusBar()->showMessage(QString("Cannot open event input: ") + inputs[s]);
        }
        connect(accumulation, &QSlider::valueChanged, this, [this](int n) {
            for (auto& c : cameras) c.windowMs = n;
        });
        rpc.open("/primi-ball-balance/eventControl/rpc:o");
        connect(apply, &QPushButton::clicked, this, [this] {
            if (pending.valid()) return;
            const int values[2][2] = {{sensitivity[0]->value(), refractory[0]->value()},
                                      {sensitivity[1]->value(), refractory[1]->value()}};
            apply->setEnabled(false);
            pending = std::async(std::launch::async, [this, values] {
                if (!yarp::os::Network::connect(rpc.getName(), "/primi-ball-balance/cameraControl/rpc"))
                    return QString("Camera RPC unavailable; start camera_control on Ultrascale.");
                int accepted = 0;
                for (int s = 0; s < 2; ++s)
                    for (int p = 0; p < 2; ++p) {
                        yarp::os::Bottle request, response;
                        request.addString("set");
                        request.addString(p ? "refractory" : "sensitivity");
                        request.addString(s ? "right" : "left");
                        request.addInt32(values[s][p]);
                        if (rpc.write(request, response) && response.size() &&
                            response.get(0).asString() == "ok") ++accepted;
                    }
                return QString("Camera RPC acknowledged: %1/4 (inspect Ultrascale readback).").arg(accepted);
            });
        });
        auto* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] {
            QString summary;
            QString flickerSummary;
            for (int s = 0; s < 2; ++s) {
                QImage raw, processed;
                double rate, accepted;
                bool receiving;
                QString filterInfo;
                cameras[s].snapshot(rawModes[s]->currentIndex(), raw, processed,
                                    rate, accepted, receiving, filterInfo);
                flickerSummary += QString("%1: %2%3").arg(s ? "Right" : "Left")
                    .arg(filterInfo).arg(s ? "" : "\n");
                rawViews[s]->setPixmap(QPixmap::fromImage(raw).scaled(rawViews[s]->size(),
                                     Qt::KeepAspectRatio, Qt::FastTransformation));
                processedViews[s]->setPixmap(QPixmap::fromImage(processed).scaled(
                    processedViews[s]->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
                summary += QString("%1: %2, %3 kEv/s, representation kept %4%  ")
                    .arg(s ? "Right" : "Left").arg(receiving ? "live" : "no data")
                    .arg(rate / 1000.0, 0, 'f', 1).arg(accepted, 0, 'f', 1);
            }
            if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
                rpcMessage = pending.get();
                rpcMessageUntil = Clock::now() + std::chrono::seconds(8);
                apply->setEnabled(true);
            }
            if (Clock::now() < rpcMessageUntil) summary += " | " + rpcMessage;
            flickerInfo->setText(flickerSummary);
            statusBar()->showMessage(summary);
        });
        timer->start(33);
    }
    ~Window() override {
        for (auto& c : cameras) c.stop();
        if (pending.valid()) pending.wait();
        rpc.close();
    }
};
} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(2.0)) return 1;
    Window window;
    window.show();
    return app.exec();
}
