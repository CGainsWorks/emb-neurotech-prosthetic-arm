// The loop: which task runs when and how long each one took.
//
// A rate-monotonic dispatcher rather than an RTOS. There are five periodic
// tasks, none of them blocks and the longest is bounded; the whole scheduling
// problem fits in this header and a kernel would add a context switch, a
// stack per task and a scheduler nobody in this repository can measure the
// jitter of.
//
// Timing statistics are collected here rather than bolted on, because the
// control loop timing budget in docs/CONTROL_LOOP_TIMING.md is only worth
// anything if the numbers come from the running system rather than from a
// spreadsheet.
#pragma once

#include <cstddef>
#include <cstdint>

namespace pa::app {

// Min, max, mean and count in microseconds. No histogram: the interesting
// question on a control loop is the worst case and a mean next to it says
// whether the worst case was an outlier or the normal state of affairs.
class TimingStats {
  public:
    void record(uint32_t microseconds);
    void reset();

    uint32_t min_us() const { return count_ > 0 ? min_us_ : 0; }
    uint32_t max_us() const { return max_us_; }
    uint32_t mean_us() const {
        return count_ > 0 ? static_cast<uint32_t>(total_us_ / count_) : 0;
    }
    uint64_t count() const { return count_; }

  private:
    uint32_t min_us_{0xFFFFFFFFu};
    uint32_t max_us_{0};
    uint64_t total_us_{0};
    uint64_t count_{0};
};

enum class Task : uint8_t {
    kAcquire = 0,   // one sample per channel, 2 kHz
    kServo,         // refresh the pulse train, 333 Hz
    kControl,       // joint control and safety, 100 Hz
    kImu,           // orientation, 100 Hz
    kStatus,        // telemetry, 10 Hz
    kCount,
};

inline constexpr std::size_t kTaskCount = static_cast<std::size_t>(Task::kCount);

struct TaskConfig {
    uint32_t period_us{0};
    // A task that misses its slot by more than this is reported as an overrun.
    // Set per task because a late acquisition sample is a lost sample, while a
    // late status message is nothing.
    uint32_t deadline_us{0};
};

struct Due {
    bool run[kTaskCount]{};
    bool overrun[kTaskCount]{};
    bool any_overrun{false};
};

// Fixed-priority, no preemption: the tasks are listed in priority order and
// the caller runs whichever are due in that order. Missing a slot is reported
// rather than silently caught up, because running two acquisition ticks back
// to back would put two samples at the same timestamp.
class Scheduler {
  public:
    Scheduler();

    void configure(Task task, const TaskConfig& config);
    void reset(uint32_t now_us);

    // Which tasks are due at `now_us`. Call once per iteration of the main
    // loop; the caller runs the tasks and reports how long each took through
    // `record`.
    Due poll(uint32_t now_us);

    void record(Task task, uint32_t microseconds) {
        stats_[static_cast<std::size_t>(task)].record(microseconds);
    }

    const TimingStats& stats(Task task) const { return stats_[static_cast<std::size_t>(task)]; }
    uint32_t overruns(Task task) const { return overruns_[static_cast<std::size_t>(task)]; }
    uint32_t total_overruns() const;

    // Slots that came and went without the task being run at all, because the
    // loop was busy elsewhere for longer than a whole period. Distinct from an
    // overrun, which is a task that ran late but ran.
    uint32_t missed(Task task) const { return missed_[static_cast<std::size_t>(task)]; }

  private:
    TaskConfig config_[kTaskCount]{};
    uint32_t next_due_us_[kTaskCount]{};
    TimingStats stats_[kTaskCount]{};
    uint32_t overruns_[kTaskCount]{};
    uint32_t missed_[kTaskCount]{};
    bool started_{false};
};

// The rates the board actually runs at, from pa/config/board.hpp. Kept here so
// main.cpp and the tests configure the scheduler identically.
Scheduler default_scheduler(uint32_t now_us);

}  // namespace pa::app
