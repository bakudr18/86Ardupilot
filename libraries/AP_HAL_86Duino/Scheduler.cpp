
#include <DataFlash/DataFlash.h>

#include "Scheduler.h"
#include "AnalogIn.h"
#include "io.h"
#include "mcm.h"
#include "irq.h"

#include <assert.h>
#include <stdarg.h>
#include <time.h>
#include <fcntl.h>

#define MC_1k 3     // 1k hz timer
#define MD_1k 2

extern const AP_HAL::HAL & hal;

using namespace x86Duino;

Scheduler::Scheduler() :
	_initialized(false),
	_failsafe(nullptr),
	_timer_proc({ nullptr }),
	_num_timer_procs(0),
	_in_timer_proc(false),
	_io_proc({ nullptr }),
	_num_io_procs(0),
	_in_io_proc(false),
	_timer_suspended(false),
	_timer_event_missed(false),
	_timer_1k_enable(false),
	_perf_timers(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_timers")),
	_perf_io_timers(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_IO_timers")),
	_perf_storage_timer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_storage_timers")),
	_perf_delay(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_delay"))
{
	// protected variables
	_min_delay_cb_ms = 65535;
	_delay_cb = nullptr;
}

static int mcint_offset[3] = { 0, 8, 16 };
int timer_1k_count = 0;

// timer 1khz ISR'
static char* isrname_one = (char*)"timer_1k";
static int timer1k_isr_handler(int irq, void* data)
{
	if ((mc_inp(MC_1k, 0x04) & (PULSE_END_INT << mcint_offset[MD_1k])) == 0) return ISR_NONE;
	mc_outp(MC_1k, 0x04, (PULSE_END_INT << mcint_offset[MD_1k]));   // clear flag
	timer_1k_count++;

	((Scheduler*)hal.scheduler)->_run_timer_procs(true);

	return ISR_HANDLED;
}

void Scheduler::init()
{
	// setup file name case
	setenv("FNCASE", "y", 1);
	// set default file open mode
	_fmode = O_BINARY;
	// setup Time Zone
	setenv("TZ", "GMT+8", 1);   // set TZ system variable (set to GMT+0)
	tzset();    // setup time zone

	// initialize()
	mcpwm_Disable(MC_1k, MD_1k);

	// disable_MCINT
	mc_outp(MC_1k, 0x00, mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k]));  // disable mc interrupt
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << MC_1k));
	// clear_INTSTATUS
	mc_outp(MC_1k, 0x04, 0xffL << mcint_offset[MD_1k]); //for EX

	if (!irq_InstallISR(GetMCIRQ(), timer1k_isr_handler, isrname_one))
		printf("timer 1k hz IRQ_install fail\n");

	// enable_MCINT(MC_1k, MD_1k, PULSE_END_INT);
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_1k));
	mc_outp(MC_1k, 0x00, (mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k])) | (PULSE_END_INT << mcint_offset[MD_1k]));

	mcpwm_SetWidth(MC_1k, MD_1k, 1000 * SYSCLK, 0L);    // 1k hz timer loop
	mcpwm_Enable(MC_1k, MD_1k);
	_timer_1k_enable = true;
}

void Scheduler::delay(uint16_t ms)
{
	if (in_timerprocess()) {
		printf("ERROR: delay() from timer process\n");
		return;
	}
	uint64_t start = AP_HAL::micros64();

	while ((AP_HAL::micros64() - start) / 1000 < ms) {
		delay_microseconds(1000);
		if (_min_delay_cb_ms <= ms) {
			call_delay_cb();
		}
	}
}

void Scheduler::delay_microseconds(uint16_t us)
{
	timer_DelayMicroseconds(us);
}

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
	uint16_t min_time_ms)
{
	_delay_cb = proc;
	_min_delay_cb_ms = min_time_ms;
}

void Scheduler::call_delay_cb()
{
	if (_delay_cb == nullptr) {
		return;
	}
	if (_in_delay_callback) {
		// don't recurse!
		return;
	}
	_in_delay_callback = true;
	_delay_cb();
	_in_delay_callback = false;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
	for (uint8_t i = 0; i < _num_timer_procs; i++) {
		if (_timer_proc[i] == proc) {
			return;
		}
	}

	if (_num_timer_procs < X86DUINO_SCHEDULER_MAX_TIMER_PROCS) {
		/* this write to _timer_proc can be outside the critical section
		 * because that memory won't be used until _num_timer_procs is
		 * incremented. */
		_timer_proc[_num_timer_procs] = proc;
		/* _num_timer_procs is used from interrupt, and multiple bytes long. */
		io_DisableINT();
		_num_timer_procs++;
		io_RestoreINT();
	}
	else {
		hal.console->printf("Out of timer processes\n");
	}
}

void Scheduler::suspend_timer_procs() {
	_timer_suspended = true;
}

void Scheduler::resume_timer_procs() {
	_timer_suspended = false;
	if (_timer_event_missed == true) {
		_run_timer_procs(false);
		_timer_event_missed = false;
	}
}

bool Scheduler::in_timerprocess() {
	return _in_timer_proc;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
	// IO processes not implemented yet.
	for (uint8_t i = 0; i < _num_io_procs; i++) {
		if (_io_proc[i] == proc) {
			return;
		}
	}

	if (_num_io_procs < X86DUINO_SCHEDULER_MAX_IO_PROCS) {
		/* this write to _timer_proc can be outside the critical section
		 * because that memory won't be used until _num_timer_procs is
		 * incremented. */
		_io_proc[_num_io_procs] = proc;
		/* _num_timer_procs is used from interrupt, and multiple bytes long. */
		io_DisableINT();
		_num_io_procs++;
		io_RestoreINT();
	}
	else {
		hal.console->printf("Out of io processes\n");
	}
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
	_failsafe = failsafe;
}

void Scheduler::system_initialized()
{
	if (_initialized) {
		AP_HAL::panic("PANIC: scheduler::system_initialized called"
			"more than once");
	}
	_initialized = true;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
	// disarm motors to ensure they are off during a bootloader upload
	//hal.rcout->force_safety_on();
	//hal.rcout->force_safety_no_wait();

	//stop logging
	//DataFlash_Class::instance()->StopLogging();

	hal.console->printf("GOING DOWN FOR A REBOOT\r\n");
	delay(100);

	io_DisableINT();
	if (hold_in_bootloader) io_outpb(0xf21A, 0x5a); // write soft reset key
	io_outpb(0x64, 0xfe); // reboot
}

void Scheduler::_run_timer_procs(bool called_from_isr)
{
	if (_in_timer_proc) {
		//AP_HAL::panic("_in_timer_proc should not happened\n");
		return;
	}
	_in_timer_proc = true;

	if (!_timer_suspended) {
		// now call the timer based drivers
		for (int i = 0; i < _num_timer_procs; i++) {
			if (_timer_proc[i]) {
				_timer_proc[i]();
			}
		}
	}
	else if (called_from_isr) {
		_timer_event_missed = true;
	}

	uint32_t t;
	// and the failsafe, if one is setup
	if (_failsafe != nullptr) {
		static uint32_t last_failsafe = 0;
		t = AP_HAL::millis();
		if (t - last_failsafe > 10) {
			last_failsafe = t + 50; // 50ms = 20Hz
			_failsafe();
		}
	}


	static uint32_t last_dataflash = 0;
	t = AP_HAL::millis();
	if (t - last_dataflash > 10) {
		last_dataflash = t + 300; // 300ms ~= 3Hz
		for (int i = 0; i < _num_io_procs; i++) {
			if (_io_proc[i]) {
				_io_proc[i]();
			}
		}
	}

	hal.storage->_timer_tick();

	// process analog input
	//((AnalogIn*)hal.analogin)->_timer_tick();

	_in_timer_proc = false;
}

void Scheduler::_run_io(void)
{
}