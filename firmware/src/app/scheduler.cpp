#include "pa/app/scheduler.hpp"

#include <algorithm>

#include "pa/config/board.hpp"

namespace pa::app {
namespace {

uint32_t period_from_hz(float hz) {
    return hz > 0.0f ? static_cast<uint32_t>(1'000'000.0f / hz + 0.5f) : 0u;
}

// Signed difference on a wrapping 32-bit microsecond counter. The counter
// wraps every 71 minutes and the arm is meant to run for longer than that, so
// every comparison in this file goes through here rather than using < on the
// raw values.
int32_t elapsed(uint32_t now, uint32_t then) {
    return static_cast<int32_t>(now - then);
}

}  // namespace

void TimingStats::record(uint32_t microseconds) {
    min_us_ = std::min(min_us_, microseconds);
    max_us_ = std::max(max_us_, microseconds);
    total_us_ += microseconds;
    ++count_;
}

void TimingStats::reset() {
    min_us_ = 0xFFFFFFFFu;
    max_us_ = 0;
    total_us_ = 0;
    count_ = 0;
}

Scheduler::Scheduler() {
    for (std::size_t i = 0; i < kTaskCount; ++i) config_[i] = TaskConfig{};
}

void Scheduler::configure(Task task, const TaskConfig& config) {
    config_[static_cast<std::size_t>(task)] = config;
}

void Scheduler::reset(uint32_t now_us) {
    for (std::size_t i = 0; i < kTaskCount; ++i) {
        next_due_us_[i] = now_us + config_[i].period_us;
        stats_[i].reset();
        overruns_[i] = 0;
        missed_[i] = 0;
    }
    started_ = true;
}

Due Scheduler::poll(uint32_t now_us) {
    Due due;
    if (!started_) reset(now_us);

    for (std::size_t i = 0; i < kTaskCount; ++i) {
        const uint32_t period = config_[i].period_us;
        if (period == 0) continue;

        const int32_t late = elapsed(now_us, next_due_us_[i]);
        if (late < 0) continue;

        due.run[i] = true;
        if (config_[i].deadline_us > 0 &&
            static_cast<uint32_t>(late) > config_[i].deadline_us) {
            due.overrun[i] = true;
            due.any_overrun = true;
            ++overruns_[i];
        }

        // Advance by whole periods from the slot that was due, so the schedule
        // stays anchored to absolute time and does not drift by however long
        // the caller took. Slots that went by entirely are counted and skipped
        // rather than run back to back.
        uint32_t skipped = 0;
        do {
            next_due_us_[i] += period;
            ++skipped;
        } while (elapsed(now_us, next_due_us_[i]) >= 0);
        if (skipped > 1) missed_[i] += skipped - 1;
    }
    return due;
}

uint32_t Scheduler::total_overruns() const {
    uint32_t total = 0;
    for (std::size_t i = 0; i < kTaskCount; ++i) total += overruns_[i];
    return total;
}

Scheduler default_scheduler(uint32_t now_us) {
    Scheduler scheduler;

    // Acquisition has the tightest deadline of the five: a sample taken late
    // is a sample at the wrong time and the classifier's window is built on
    // the assumption that the interval is uniform. A tenth of a period.
    scheduler.configure(Task::kAcquire,
                        {period_from_hz(config::kSemgSampleRateHz),
                         period_from_hz(config::kSemgSampleRateHz) / 10});

    // A servo frame arriving a little late shortens the gap, which the servo
    // does not mind. A quarter of a period.
    scheduler.configure(Task::kServo,
                        {period_from_hz(config::kServoFrameRateHz),
                         period_from_hz(config::kServoFrameRateHz) / 4});

    scheduler.configure(Task::kControl,
                        {period_from_hz(config::kJointControlRateHz),
                         period_from_hz(config::kJointControlRateHz) / 4});

    scheduler.configure(Task::kImu, {period_from_hz(config::kImuRateHz),
                                     period_from_hz(config::kImuRateHz) / 2});

    // Telemetry is best effort and has no deadline: a late status message is
    // a late status message.
    scheduler.configure(Task::kStatus, {period_from_hz(10.0f), 0});

    scheduler.reset(now_us);
    return scheduler;
}

}  // namespace pa::app
