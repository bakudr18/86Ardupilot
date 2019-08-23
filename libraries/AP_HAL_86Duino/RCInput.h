#pragma once

#include "AP_HAL_86Duino.h"

#define RCINPUT_MAX_CH 8    // CH_7 & CH_8 are software simulate

class x86Duino::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init();
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
private:
    void _open_encoder_pin(int mc, int pin);
    void _set_MCM_CaptureMode(uint16_t mc);
    bool _interrupt_init(void);
    void _clear_interrupt_state(int mc, int bit);
    bool _check_interrupt_state(int mc, int bit);
    bool _check_interrupt_enable(int mc, int bit);

    /* override state */
    uint16_t _override[RCINPUT_MAX_CH];
    bool _override_valid;
};
