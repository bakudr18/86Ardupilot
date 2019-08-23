#include <AP_HAL/AP_HAL.h>
#include "Scheduler.h"
#include "AnalogIn.h"
#include "Storage.h"

#include <stdarg.h>
#include <time.h>
#include <fcntl.h>

#include "io.h"
#include "mcm.h"
#include "irq.h"

extern const AP_HAL::HAL& hal;

namespace x86Duino {
extern volatile bool in_loop ;

#define MC_1k 3     // 1k hz timer
#define MD_1k 2

static int mcint_offset[3] = {0, 8, 16};
int wdt_count = 0, timer_1k_count = 0 ;
int spi_count = 0, spi_mpu9250_count = 0;

// timer 1khz ISR'
static char* isrname_one = (char*)"timer_1k";
static int timer1k_isr_handler(int irq, void* data)
{    
    if((mc_inp(MC_1k, 0x04) & (PULSE_END_INT << mcint_offset[MD_1k])) == 0) return ISR_NONE;
    mc_outp(MC_1k, 0x04, (PULSE_END_INT << mcint_offset[MD_1k]));   // clear flag
    timer_1k_count++;
    
    ((Scheduler*)hal.scheduler)->_run_timer_procs();

    return ISR_HANDLED;
}

Scheduler::Scheduler()
{
    _timer_1k_enable = false;
    _wdt_1k_enable = false;
    _in_timer_1k = false;
    _timer_suspended = false;
    initialized = false;
}

void Scheduler::init()
{
    if(_timer_1k_enable) return;

    // setup file name case
    setenv("FNCASE", "y", 1);
    // set default file open mode
    _fmode = O_BINARY;    
    // setup Time Zone
    setenv("TZ", "GMT+8", 1);   // set TZ system variable (set to GMT+0)
    tzset();    // setup time zone

    mcpwm_Disable(MC_1k, MD_1k);

    // disable_MCINT
    mc_outp(MC_1k, 0x00, mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k]));  // disable mc interrupt
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << MC_1k));
    // clear_INTSTATUS
    mc_outp(MC_1k, 0x04, 0xffL << mcint_offset[MD_1k]); //for EX

    if( !irq_InstallISR(GetMCIRQ(), timer1k_isr_handler, isrname_one) )
        printf("timer 1k hz IRQ_install fail\n");

    // enable_MCINT(MC_1k, MD_1k, PULSE_END_INT);
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_1k));
    mc_outp(MC_1k, 0x00, (mc_inp(MC_1k, 0x00) & ~(0xffL<<mcint_offset[MD_1k])) | (PULSE_END_INT << mcint_offset[MD_1k]));

    mcpwm_SetWidth(MC_1k, MD_1k, 1000*SYSCLK, 0L);    // 1k hz timer loop
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
    while ((AP_HAL::micros64() - start)/1000 < ms)
    {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) // callback if delay is long enough..
        {
            if (_delay_cb)
            {
                _delay_cb();
            }
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

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

AP_HAL::Device::PeriodicHandle Scheduler::register_i2c_process(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    for (uint8_t i = 0; i < _num_i2c_procs; i++) {
        if (_i2c_proc[i].cb == cb) {
            _i2c_proc[i].period_usec = period_usec ;    // update period_usec
            return (&_i2c_proc[i]);
        }
    }

    if (_num_i2c_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _i2c_proc[_num_i2c_procs].cb = cb;
        _i2c_proc[_num_i2c_procs].period_usec = period_usec;
        _i2c_proc[_num_i2c_procs].next_usec = AP_HAL::micros64()+period_usec;
        _num_i2c_procs++;
        return (&_i2c_proc[_num_i2c_procs]);
    } else {
        hal.console->printf("Out of I2C processes\n");
        return nullptr;
    }
}

AP_HAL::Device::PeriodicHandle Scheduler::register_spi_process(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    for (uint8_t i = 0; i < _num_spi_procs; i++) {
        if (_spi_proc[i].cb == cb) {
            _spi_proc[i].period_usec = period_usec ;    // update period_usec
            return (&_spi_proc[i]);
        }
    }

    if (_num_spi_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _spi_proc[_num_spi_procs].cb = cb;
        _spi_proc[_num_spi_procs].period_usec = period_usec;
        _spi_proc[_num_spi_procs].next_usec = AP_HAL::micros64()+period_usec;
        _num_spi_procs++;
        return (&_spi_proc[_num_spi_procs]);
    } else {
        hal.console->printf("Out of SPI processes\n");
        return nullptr;
    }
}

void Scheduler::run_io(void)
{
    _in_io_proc = true;

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) _io_proc[i]();

    }
    ((Storage*)hal.storage)->_timer_tick();

    _in_io_proc = false;
}

void Scheduler::run_i2c_thread(void)
{
    _in_i2c_proc = true;    
    // now call the I2C device driver
    for (int i = 0; i < _num_i2c_procs; i++) {
        uint64_t now = AP_HAL::micros64() ;
        if ( now >= _i2c_proc[i].next_usec)
        {
            while( now >= _i2c_proc[i].next_usec ){
                _i2c_proc[i].next_usec += _i2c_proc[i].period_usec ;
            }
            // process!
            _i2c_proc[i].cb();
        }
    }
    _in_i2c_proc = false;
}

void Scheduler::run_spi_thread(void)
{
    _in_spi_proc = true;

    spi_count++;
    // now call the SPI device driver
    for (int i = 0; i < _num_spi_procs; i++) {
        uint64_t now = AP_HAL::micros64() ;
        if ( now >= _spi_proc[i].next_usec)
        {
            while( now >= _spi_proc[i].next_usec ){
                _spi_proc[i].next_usec += _spi_proc[i].period_usec ;                
            }
            // process!
            _spi_proc[i].cb();
            spi_mpu9250_count++;
        }
    }
    _in_spi_proc = false;
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
}

bool Scheduler::in_timerprocess() {
    return _in_timer_1k;
}

bool Scheduler::in_main_thread() const {
    return true;
}

void Scheduler::system_initialized()
{
    if (initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    initialized = true;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    io_DisableINT();
    // for one, usb_detect_pin = 0 , usb_on_off_pin = 1, (usb_on_off_data = GPIO_BASE_ADDR + 7)
    unsigned short usb_on_off_data = 0xf100 + 7 ;
    char usb_on_off_pin = 1;
    io_outpb(usb_on_off_data, io_inpb(usb_on_off_data) | (1 << usb_on_off_pin));
    if(hold_in_bootloader) io_outpb(0xf21A, 0x5a); // write soft reset key
    io_outpb(0x64, 0xfe); // reboot
}

void Scheduler::_run_timer_procs()
{
    _in_timer_1k = true;
    // 1k timer schedule
    if( !_timer_suspended )
    {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++)
        {
            if (_timer_proc[i]) _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr)
    {
        _failsafe();
    }

    _in_timer_1k = false;

    // AD timer thread
    ((AnalogIn*)hal.analogin)->update();
}

}
