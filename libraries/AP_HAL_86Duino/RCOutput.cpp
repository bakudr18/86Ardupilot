#include "RCOutput.h"
#include "io.h"
#include "mcm.h"
#include "v86clock.h"
#include "pins_arduino.h"
extern const AP_HAL::HAL& hal ;

namespace x86Duino {
extern pinsconfig PIN86[];

RCOutput::RCOutput()
{
    _cork_on = false;
    _safty_on = false;
}

void RCOutput::init()
{
    // pin 29~32 map to channel 1~4
    init_channel( CH_1, 29, 50, 0);
    init_channel( CH_2, 30, 50, 0);
    init_channel( CH_3, 31, 50, 0);
    init_channel( CH_4, 32, 50, 0);
}

void RCOutput::init_channel( uint8_t ch, uint8_t pin, uint16_t freq_hz, uint16_t width)
{
    if( ch >= PWM_MAX_CH )   return;
    int mc, md;
    mc = PIN86[pin].PWMMC;
    md = PIN86[pin].PWMMD;
    if(mc == NOPWM || md == NOPWM)  return;
    
    if( freq_hz > 1600 ) freq_hz = 1600 ;
    if( freq_hz < 50 )  freq_hz = 50 ;
    CH_List[ch].freq = freq_hz ;
    CH_List[ch].period = 1000000L/freq_hz;
    
    io_DisableINT();
    
    mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_CANCEL);
    mcpwm_SetOutMask(mc, md, MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
    mcpwm_SetOutPolarity(mc, md, MCPWM_HPOL_NORMAL + MCPWM_LPOL_NORMAL);
    mcpwm_SetDeadband(mc, md, 0L);
    mcpwm_ReloadOUT_Unsafe(mc, md, MCPWM_RELOAD_NOW);
    mcpwm_SetWaveform(mc, md, MCPWM_EDGE_A0I1);
    mcpwm_SetSamplCycle(mc, md, 1999L);   // no function for us
    
    mcpwm_SetWidth(mc, md, CH_List[ch].period*SYSCLK, width*SYSCLK);   // cycle = 20ms -> 50hz default, 0 for no output
    mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_PEREND);
    
    mcpwm_Enable(mc, md);
    io_outpb(CROSSBARBASE + 0x90 + PIN86[pin].gpN, 0x08);
    
    io_RestoreINT();
    
    CH_List[ch].pin = pin ;
    CH_List[ch].width = width ;
    CH_List[ch].enable = true ;
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    // limit frequency between 50~1600 hz
    if( freq_hz > 1600 ) freq_hz = 1600 ;
    if( freq_hz < 50 )  freq_hz = 50 ;
    
    uint32_t period = 1000000L/freq_hz ; // in us
    int mc, md;
    
    io_DisableINT();
    for( int i = 0 ; i < PWM_MAX_CH ; i++ )
    {
        if( chmask & (0x01 << i) )  // check mask
        {
            mc = PIN86[CH_List[i].pin].PWMMC;
            md = PIN86[CH_List[i].pin].PWMMD;
            if(mcpwm_ReadReloadPWM(mc, md) != 0) mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_CANCEL);
            mcpwm_SetWidth(mc, md, period*SYSCLK, CH_List[i].width*SYSCLK);   // cycle = 20ms -> 50hz default, 0 for no output
            mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_PEREND);
            CH_List[i].freq = freq_hz;
            CH_List[i].period = period;
        }
    }
    io_RestoreINT();
    
}

uint16_t RCOutput::get_freq(uint8_t ch) {
    if( ch >= PWM_MAX_CH )   return 50 ;
    return CH_List[ch].freq ;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if( ch >= PWM_MAX_CH )   return ;
    if( !CH_List[ch].enable )
        init_channel(ch, CH_List[ch].pin, CH_List[ch].freq, CH_List[ch].width);  // re-initialize
}

void RCOutput::disable_ch(uint8_t ch)
{
    if( ch >= PWM_MAX_CH )   return ;
    int mc, md;
    mc = PIN86[CH_List[ch].pin].PWMMC;
    md = PIN86[CH_List[ch].pin].PWMMD;
    
    io_DisableINT();
    mcpwm_Disable(mc, md);  // stop MCM
    io_RestoreINT();
    
    CH_List[ch].enable = false;
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if( ch >= PWM_MAX_CH )   return ;
    CH_List[ch].width = period_us;
    if( !_cork_on ) PWM_output(ch);
}

void RCOutput::PWM_output(uint8_t ch)
{
    if( ch >= PWM_MAX_CH )   return ;
    int mc, md;
    mc = PIN86[CH_List[ch].pin].PWMMC;
    md = PIN86[CH_List[ch].pin].PWMMD;
    
    io_DisableINT();
    if(mcpwm_ReadReloadPWM(mc, md) != 0) mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_CANCEL);
    mcpwm_SetWidth(mc, md, CH_List[ch].period*SYSCLK, CH_List[ch].width*SYSCLK);   // cycle = 20ms -> 50hz default, 0 for no output
    mcpwm_ReloadPWM(mc, md, MCPWM_RELOAD_PEREND);
    io_RestoreINT();
}

uint16_t RCOutput::read(uint8_t ch)
{
    if( ch >= PWM_MAX_CH )   return 0;
    return CH_List[ch].width;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for( int i = 0; i < len; i++ )
    {
        if( i < PWM_MAX_CH )
            period_us[i] = CH_List[i].width;
        else
            period_us[i] = 0;
    }
}

void RCOutput::cork(void)
{
    _cork_on = true;
}

void RCOutput::push(void)
{
    for( int i = 0; i < PWM_MAX_CH; i++ )
        PWM_output(i);
    _cork_on = false;
}

bool RCOutput::force_safety_on(void) { return (_safty_on = true); }
void RCOutput::force_safety_off(void) { _safty_on = false; }

void RCOutput::set_output_mode(enum output_mode mode)
{
    
}

}
