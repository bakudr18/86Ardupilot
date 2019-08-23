#pragma once

#include "AP_HAL_86Duino.h"

namespace x86Duino {

class GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t mode);
    int8_t  analogPinToDigitalPin(uint8_t pin); // empty implement
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t val);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */ // empty implement
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
    void    setPinStatus();
};

class DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    uint8_t _v;
};

}
