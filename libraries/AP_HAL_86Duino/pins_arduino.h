#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>
#include "mcm.h"

#define LOW          (0x00)
#define HIGH         (0x01)
#define CHANGE       (0x02)
#define FALLING      (0x03)
#define RISING       (0x04)

#define INPUT          (0x00)
#define OUTPUT         (0x01)
#define INPUT_PULLUP   (0x02)
#define INPUT_PULLDOWN (0x03)

// for 86Duino
#define SB_CROSSBASE  (0x64)
#define CROSSBARBASE  (0x0A00)
#define SB_FCREG      (0xC0)
#define SB_GPIOBASE   (0x62)
#define GPIOCTRLBASE  (0xF100)
#define GPIODATABASE  (0xF200)
#define GPIODIRBASE   (0xF202)

#define NOUSED   (-1)
#define NOAD     (-1)
#define NOPWM    (-1)
#define NOENC    (-1)
#define NOSTATUS (-1)

//#define CRB_DEBUGMODE

struct pinsconfig {
    // GPIO
    int gpN;
    int gpMode;
    int gpOutput;

    // AD
    int AD;

    // PWM
    int PWMMC;
    int PWMMD;

    // Interrupt
    int ENCMC;
    int ENCMD;
    void (*userfunc)(void);
};

#endif
