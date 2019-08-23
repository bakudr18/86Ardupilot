#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

#include <stdio.h>
#include <assert.h>
#include <signal.h>

#include "HAL_86Duino_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Util.h"
#include "USBSerial.h"

#include "io.h"
#include "irq.h"
#include "pins_arduino.h"

unsigned _stklen = 4096 * 1024; // set stack to 4096k

using namespace x86Duino;
static Scheduler x86Scheduler;
static RCInput  x86RCInput;
static RCOutput x86RCOutput;
static AnalogIn x86AnalogIn;
static GPIO x86GPIO;
static I2CDeviceManager x86I2C;
static SPIDeviceManager x86SPI;
static Storage x86Storage;
static Util x86Util;


static UARTDriver Serial1(COM1, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial2(COM2, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial3(COM3, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial485(COM4, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static USBSerial usbUart;

extern int wdt_count, timer_1k_count, spi_count, spi_mpu9250_count ,rc_in_count, uart_count;
volatile bool in_loop = false ;

void _86Duino_error_process(int num) {
	// disable all irq except usb irq (5)
	i8259_DisableIRQ(0);
	i8259_DisableIRQ(1);
	i8259_DisableIRQ(3);
	i8259_DisableIRQ(4);
	i8259_DisableIRQ(6);
	i8259_DisableIRQ(7);
	i8259_DisableIRQ(8);
	i8259_DisableIRQ(9);
	i8259_DisableIRQ(10);
	i8259_DisableIRQ(11);
	i8259_DisableIRQ(12);
	i8259_DisableIRQ(13);
	i8259_DisableIRQ(14);

	// print error message
	printf("\nOop, this program is crash :(\n");
	printf("You may write a bug in your sketch, check and upload it again.\n");

	while(1)
	{
	}
}

static __attribute__((constructor(101))) void _f_init()
{
	signal(SIGSEGV, _86Duino_error_process);
	signal(SIGFPE, _86Duino_error_process);
}

HAL_86Duino::HAL_86Duino() :
    AP_HAL::HAL(
        &usbUart,   /* uartA */
        &Serial1,   /* uartB */
        &Serial2,   /* uartC */
        &Serial3,   /* uartD */
        &Serial485,   /* uartE */
        nullptr,   /* uartF */
        &x86I2C,   /* i2c */
        &x86SPI,          /* spi */
        &x86AnalogIn,      /* analogin */
        &x86Storage, /* storage */
        &usbUart,   /* console */
        &x86GPIO,          /* gpio */
        &x86RCInput,       /* rcinput */
        &x86RCOutput,      /* rcoutput */
        &x86Scheduler,     /* scheduler */
        &x86Util,      /* util */
        nullptr, /* onboard optical flow */
        nullptr) /* CAN bus */
{}

void HAL_86Duino::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    if(io_Init() == false) return;
    timer_NowTime(); // initialize timer

    // Set IRQ4 as level-trigger
    io_outpb(0x4D0, io_inpb(0x4D0) | 0x10);

    //set corssbar Base Address
    int crossbarBase = sb_Read16(SB_CROSSBASE) & 0xfffe;
    if(crossbarBase == 0 || crossbarBase == 0xfffe)
    {
        sb_Write16(SB_CROSSBASE, CROSSBARBASE | 0x01);
    }

    // Force set HIGH speed ISA on SB
    sb_Write(SB_FCREG, sb_Read(SB_FCREG) | 0x8000C000L);

    // GPIO->init
    x86GPIO.init();
    // AD->init
    x86AnalogIn.init();
    // set MCM Base Address, init before using PWM in/out
    set_MMIO();
    mcmInit();
    for(int i=0; i<4; i++)
        mc_SetMode(i, MCMODE_PWM_SIFB);

    // set MCM IRQ
    if(irq_Init() == false) printf("MCM IRQ init fail\n");
    if(irq_Setting(GetMCIRQ(), IRQ_LEVEL_TRIGGER | IRQ_DISABLE_INTR | IRQ_USE_FPU) == false)
        printf("MCM IRQ Setting fail\n");

    Set_MCIRQ(GetMCIRQ());
    // RC in/out
    x86RCOutput.init();
    x86RCInput.init();

    // USB-CDC init()
    usbUart.begin(115200);

    // Scheduler init
    x86Scheduler.init();

    // Storage init
    Serial1.begin(115200);    
    x86Storage.init();

    // test zone
    x86GPIO.pinMode(13, HAL_GPIO_OUTPUT);
//    x86Util.set_system_clock(1506577910909*1000ULL);


    callbacks->setup();
    scheduler->system_initialized();

    for (;;) 
    { 
        x86Scheduler.run_io();
        callbacks->loop();  // use polling - self control frequency
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_86Duino hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
