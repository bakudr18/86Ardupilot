#pragma once

#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <vector>
#include <climits>
#include <atomic>
#include <time.h>


namespace x86Duino {

class Perf_Counter {
    using perf_counter_type = AP_HAL::Util::perf_counter_type;
    using perf_counter_t = AP_HAL::Util::perf_counter_t;

public:
    Perf_Counter(perf_counter_type type_, const char *name_)
        : name{name_}
        , type{type_}
        , min{ULONG_MAX}
        , max{0}
    {
    }

    const char *name;

    perf_counter_type type;

    uint64_t count;

    /* Everything below is in nanoseconds */
    uint64_t start;
    uint64_t total;
    uint64_t min;
    uint64_t max;

    double avg;
    double m2;
};

class Util : public AP_HAL::Util {
public:
    Util();
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
    /*
      return state of safety switch, if applicable
     */
    enum safety_state safety_switch_state(void);

    /*
      set system clock in UTC microseconds
     */
    //void set_hw_rtc(uint64_t time_utc_usec);

    //uint64_t get_hw_rtc() const;

	void set_system_clock(uint64_t time_utc_usec);
    /*
      get system identifier (eg. serial number)
      return false if a system identifier is not available

      Buf should be filled with a printable string and must be null
      terminated
     */
    bool get_system_id(char buf[40]);

    uint32_t available_memory(void) override;
    
    perf_counter_t perf_alloc(perf_counter_type t, const char *name) override;
    void perf_begin(perf_counter_t h) override;
    void perf_end(perf_counter_t h) override;
    void perf_count(perf_counter_t h) override;

    // create a new semaphore
    AP_HAL::Semaphore *new_semaphore(void) override;
    void _debug_counters() override;

private:
    std::vector<Perf_Counter> _perf_counters;
    std::atomic<unsigned int> _update_count;
    uint64_t _last_debug_msec;

};

}
