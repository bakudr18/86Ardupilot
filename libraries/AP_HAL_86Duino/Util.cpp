#include "Util.h"
#include "io.h"
#include <cmath>
#include <dos.h>
#include <stdlib.h>

extern const AP_HAL::HAL& hal ;

namespace x86Duino {

#ifndef PRIu64
#define PRIu64 "llu"
#endif

static inline uint64_t now_nsec()
{
    uint64_t nowclocks;
    __asm__ __volatile__ ("rdtsc" : "=A"(nowclocks) );
    return (nowclocks*1000)/vx86_CpuCLK();
}

Util::Util()
{
    /* TODO: this number should come from vehicle code - just estimate the
     * number of perf counters for now; if we grow more, it will just
     * reallocate the memory pool */
    _perf_counters.reserve(50);

}

enum Util::safety_state Util::safety_switch_state()
{
    return SAFETY_NONE;
}

//void Util::set_hw_rtc(uint64_t time_utc_usec)
//{
//    struct timeval tp;
//    tp.tv_sec = time_utc_usec/1000000 ;
//    tp.tv_usec = time_utc_usec%1000000 ;
//    settimeofday(&tp);
//}
//
//uint64_t Util::get_hw_rtc() const
//{
//    struct timeval ts;
//    gettimeofday(&ts, nullptr);
//    return ((uint64_t)(ts.tv_sec) * 1000) + (ts.tv_usec / 1000);
//}

void Util::set_system_clock(uint64_t time_utc_usec)
{
	struct timeval tp;
	tp.tv_sec = time_utc_usec / 1000000;
	tp.tv_usec = time_utc_usec % 1000000;
	settimeofday(&tp);
}

bool Util::get_system_id(char buf[]) { return false; }

uint32_t Util::available_memory()
{
    // As djgpp cross compiler didn't release mallinfo() structure in stdlib.h,
    // we assumed that memory is always big enough for malloc.
    return 40960;
}

Util::perf_counter_t Util::perf_alloc(perf_counter_type t, const char *name)
{
    if (t != Util::PC_COUNT && t != Util::PC_ELAPSED) {
        /*
         * Other perf counters not implemented for now since they are not
         * used anywhere.
         */
        return (Util::perf_counter_t)(uintptr_t) -1;
    }

    Util::perf_counter_t pc = (Util::perf_counter_t) _perf_counters.size();
    _perf_counters.emplace_back(t, name);

    return pc;
}

void Util::perf_begin(perf_counter_t h)
{
    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start != 0) {
        hal.console->printf("perf_begin() called twice on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    perf.start = now_nsec();
}

void Util::perf_end(perf_counter_t h)
{
    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start == 0) {
        hal.console->printf("perf_begin() called before begin() on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    const uint64_t elapsed = now_nsec() - perf.start;
    perf.count++;
    perf.total += elapsed;

    if (perf.min > elapsed) {
        perf.min = elapsed;
    }

    if (perf.max < elapsed) {
        perf.max = elapsed;
    }

    /*
     * Maintain avg and variance of interval in nanoseconds
     * Knuth/Welford recursive avg and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */
    const double delta_intvl = elapsed - perf.avg;
    perf.avg += (delta_intvl / perf.count);
    perf.m2 += (delta_intvl * (elapsed - perf.avg));
    perf.start = 0;
}

void Util::perf_count(perf_counter_t h)
{
    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_COUNT) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_COUNT type.\n",
                            perf.name);
        return;
    }

    _update_count++;
    perf.count++;

}

void Util::_debug_counters()
{
    uint64_t now = AP_HAL::millis64();

    if (now - _last_debug_msec < 5000) {
        return;
    }

    unsigned int uc = _update_count;
    auto v = _perf_counters;

    if (uc != _update_count) {
        fprintf(stderr, "WARNING!! potentially wrong counters!!!");
    }

    for (auto &c : v) {
        if (!c.count) {
            hal.console->printf(
                    "%-30s\t" "(no events)\n", c.name);
        } else if (c.type == Util::PC_ELAPSED) {
            hal.console->printf(
                    "%-30s\t"
                    "count: %" PRIu64 "\t"
                    "min: %" PRIu64 "\t"
                    "max: %" PRIu64 "\t"
                    "avg: %.4f\t"
                    "stddev: %.4f\n",
                    c.name, c.count, c.min, c.max, c.avg, sqrt(c.m2));
        } else {
            hal.console->printf(
                    "%-30s\t" "count: %" PRIu64 "\n",
                    c.name, c.count);
        }
    }

    _last_debug_msec = now;
}
AP_HAL::Semaphore *Util::new_semaphore() { return new Semaphore; }

}
