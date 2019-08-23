#pragma once

#include "AP_HAL_86Duino.h"

namespace x86Duino {
#define PWM_MAX_CH  4
typedef struct
{
    uint8_t     pin;    // 86duino pin
    bool        enable;
    uint32_t    width;  // PWM width in us
    uint32_t    freq;   // PWM frequency in Hz
    uint32_t    period;
} CH_Info;

class RCOutput : public AP_HAL::RCOutput {
public:
    RCOutput();
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     cork(void) override ;
    void     push(void) override ;
    bool     force_safety_on(void) ;
    void     force_safety_off(void) ;
    void     set_output_mode(enum output_mode mode) ;

private:
    void     init_channel(uint8_t ch, uint8_t pin, uint16_t freq_hz, uint16_t width);
    CH_Info  CH_List[PWM_MAX_CH];
    bool     _cork_on, _safty_on;
    void     PWM_output(uint8_t ch);
};

}
