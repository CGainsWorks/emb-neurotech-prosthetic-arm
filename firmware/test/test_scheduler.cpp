#include "pa/app/scheduler.hpp"
#include "test_framework.hpp"

using namespace pa;
using namespace pa::app;

TEST(timing_statistics_are_what_they_claim_to_be) {
    TimingStats stats;
    CHECK_EQ(static_cast<int>(stats.count()), 0);
    CHECK_EQ(static_cast<int>(stats.min_us()), 0);

    for (uint32_t value : {10u, 20u, 30u, 40u}) stats.record(value);
    CHECK_EQ(static_cast<int>(stats.min_us()), 10);
    CHECK_EQ(static_cast<int>(stats.max_us()), 40);
    CHECK_EQ(static_cast<int>(stats.mean_us()), 25);
    CHECK_EQ(static_cast<int>(stats.count()), 4);

    stats.reset();
    CHECK_EQ(static_cast<int>(stats.count()), 0);
    CHECK_EQ(static_cast<int>(stats.max_us()), 0);
}

TEST(the_mean_does_not_overflow_after_a_long_run) {
    // A 32-bit accumulator of microseconds overflows in about seventy minutes
    // at these rates and the arm is meant to run for longer than that.
    TimingStats stats;
    for (int i = 0; i < 2'000'000; ++i) stats.record(3000);
    CHECK_EQ(static_cast<int>(stats.mean_us()), 3000);
}

TEST(each_task_runs_at_the_rate_it_was_configured_for) {
    Scheduler scheduler = default_scheduler(0);

    int counts[kTaskCount]{};
    // One second, polled at 100 kHz, which is faster than anything is due.
    for (uint32_t now = 0; now <= 1'000'000; now += 10) {
        const Due due = scheduler.poll(now);
        for (std::size_t i = 0; i < kTaskCount; ++i)
            if (due.run[i]) ++counts[i];
    }

    CHECK_NEAR(counts[static_cast<std::size_t>(Task::kAcquire)], 2000, 2);
    CHECK_NEAR(counts[static_cast<std::size_t>(Task::kServo)], 333, 2);
    CHECK_NEAR(counts[static_cast<std::size_t>(Task::kControl)], 100, 2);
    CHECK_NEAR(counts[static_cast<std::size_t>(Task::kImu)], 100, 2);
    CHECK_NEAR(counts[static_cast<std::size_t>(Task::kStatus)], 10, 2);
}

TEST(the_schedule_does_not_drift_when_the_caller_is_slow) {
    // Slots are anchored to absolute time, not to when the last one finished.
    // Advancing by "now plus period" instead accumulates every late poll and
    // the rate walks away over minutes.
    Scheduler scheduler;
    scheduler.configure(Task::kControl, {10'000, 5'000});
    scheduler.reset(0);

    int runs = 0;
    uint32_t last_run = 0;
    // Poll on an awkward interval that never lines up with the period.
    for (uint32_t now = 0; now <= 10'000'000; now += 3'333) {
        if (scheduler.poll(now).run[static_cast<std::size_t>(Task::kControl)]) {
            ++runs;
            last_run = now;
        }
    }
    CHECK_NEAR(runs, 1000, 2);
    // After ten seconds the last run is still within one poll interval of
    // where the schedule says it should be.
    CHECK(last_run > 9'990'000u);
}

TEST(a_late_task_is_reported_as_an_overrun) {
    Scheduler scheduler;
    scheduler.configure(Task::kAcquire, {500, 50});
    scheduler.reset(0);

    // On time.
    Due due = scheduler.poll(500);
    CHECK(due.run[static_cast<std::size_t>(Task::kAcquire)]);
    CHECK(!due.overrun[static_cast<std::size_t>(Task::kAcquire)]);

    // A hundred microseconds late, against a fifty microsecond deadline.
    due = scheduler.poll(1100);
    CHECK(due.run[static_cast<std::size_t>(Task::kAcquire)]);
    CHECK(due.overrun[static_cast<std::size_t>(Task::kAcquire)]);
    CHECK(due.any_overrun);
    CHECK_EQ(static_cast<int>(scheduler.overruns(Task::kAcquire)), 1);
}

TEST(slots_that_went_by_entirely_are_counted_and_not_run_back_to_back) {
    // Running two acquisition ticks in a row to catch up would put two samples
    // at the same instant, which is worse than admitting one was lost.
    Scheduler scheduler;
    scheduler.configure(Task::kAcquire, {500, 50});
    scheduler.reset(0);

    // Nothing polls for five milliseconds: ten slots have gone.
    const Due due = scheduler.poll(5'000);
    CHECK(due.run[static_cast<std::size_t>(Task::kAcquire)]);
    CHECK_EQ(static_cast<int>(scheduler.missed(Task::kAcquire)), 9);

    // And the next slot is one period from now, not ten periods ago.
    CHECK(!scheduler.poll(5'100).run[static_cast<std::size_t>(Task::kAcquire)]);
    CHECK(scheduler.poll(5'500).run[static_cast<std::size_t>(Task::kAcquire)]);
}

TEST(a_task_with_no_deadline_never_overruns) {
    // Telemetry is best effort. A late status message is a late status message.
    Scheduler scheduler = default_scheduler(0);
    // Polled every 250 ms, well past the 100 ms status period, so slots are
    // being skipped throughout and none of them counts as an overrun.
    for (uint32_t now = 0; now <= 2'000'000; now += 250'000) scheduler.poll(now);
    CHECK_EQ(static_cast<int>(scheduler.overruns(Task::kStatus)), 0);
    CHECK(scheduler.missed(Task::kStatus) > 0);
}

TEST(the_microsecond_counter_wrapping_does_not_stall_the_schedule) {
    // The counter wraps every 71 minutes. Every comparison in the scheduler
    // goes through a signed difference for this reason and if one of them
    // ever stops doing so, the arm freezes an hour in.
    Scheduler scheduler;
    scheduler.configure(Task::kControl, {10'000, 5'000});

    const uint32_t start = 0xFFFF0000u;
    scheduler.reset(start);

    int runs = 0;
    for (uint32_t step = 0; step <= 2'000'000; step += 1'000) {
        // Deliberate wrapping arithmetic: this is the case being tested.
        const uint32_t now = start + step;
        if (scheduler.poll(now).run[static_cast<std::size_t>(Task::kControl)]) ++runs;
    }
    CHECK_NEAR(runs, 200, 2);
}

TEST(timings_are_recorded_per_task) {
    Scheduler scheduler = default_scheduler(0);
    scheduler.record(Task::kControl, 120);
    scheduler.record(Task::kControl, 340);
    scheduler.record(Task::kAcquire, 8);

    CHECK_EQ(static_cast<int>(scheduler.stats(Task::kControl).max_us()), 340);
    CHECK_EQ(static_cast<int>(scheduler.stats(Task::kControl).mean_us()), 230);
    CHECK_EQ(static_cast<int>(scheduler.stats(Task::kAcquire).max_us()), 8);
    CHECK_EQ(static_cast<int>(scheduler.stats(Task::kImu).count()), 0);
}

TEST(the_default_schedule_is_feasible_on_paper) {
    // Rate-monotonic utilisation: the sum of execution time over period for
    // every task, against the Liu and Layland bound for five tasks. The
    // execution times here are the budget from docs/CONTROL_LOOP_TIMING.md,
    // not measurements and the point of this test is that the budget itself
    // adds up. What the tasks actually cost on the target is unknown, because
    // no board has been run.
    struct Budget {
        Task task;
        double period_us;
        double budget_us;
    };
    const Budget budget[] = {
        {Task::kAcquire, 500.0, 40.0},
        {Task::kServo, 3003.0, 30.0},
        {Task::kControl, 10000.0, 900.0},
        {Task::kImu, 10000.0, 300.0},
        {Task::kStatus, 100000.0, 200.0},
    };

    double utilisation = 0.0;
    for (const auto& entry : budget) utilisation += entry.budget_us / entry.period_us;

    // n (2^(1/n) - 1) for n = 5 is 0.7435.
    CHECK(utilisation < 0.7435);
    CHECK(utilisation > 0.0);
}

TEST_MAIN()
