#include <string.h>

#include "RCInput.h"
#include "io.h"
#include "mcm.h"
#include "irq.h"
#include "v86clock.h"
#include "pins_arduino.h"

extern const AP_HAL::HAL& hal ;

namespace x86Duino {
#define MCMINT_ENABLE_REG  (0x00)
#define MCMINT_STAT_REG    (0x04)
// define SIFB interrupt bits for pulse & capture mode
#define SIFB_CAP1INTBIT        (29)
#define SIFB_CAP2INTBIT        (30)
#define SIFB_CAP3INTBIT        (31)
#define SIFB_CAP_ALL_INTBIT     (0xE0000000)

static unsigned long (*(readCapStat[3]))(int, int) = {mcpfau_ReadCAPSTAT1, mcpfau_ReadCAPSTAT2, mcpfau_ReadCAPSTAT3};
static unsigned long (*(readCapFIFO[3]))(int, int, unsigned long*) = {mcpfau_ReadCAPFIFO1, mcpfau_ReadCAPFIFO2, mcpfau_ReadCAPFIFO3};
static volatile unsigned long CH[RCINPUT_MAX_CH] = {0};
static volatile int CH_flag = 0x00;
static volatile bool CH_updated = false;
static const char* name = "rcinput_Int";

extern pinsconfig PIN86[];
extern int INTPINSMAP[];	// miss include GPIO.h ?

static void enable_MCINT(int mc, int bit) {
    mc_outp(mc, 0x00, mc_inp(mc, 0x00) | (0x01<<bit));
}

static void _clear_interrupt_state(int mc, int bit) {
    mc_outp(mc, MCMINT_STAT_REG, (0x01)<<bit);
}

static bool _check_interrupt_state(int mc, int bit) {
    if((mc_inp(mc, MCMINT_STAT_REG) & (0x01<<bit)) != 0L) return true;
    return false;
}

static bool _check_interrupt_enable(int mc, int bit) {
    if((mc_inp(mc, MCMINT_ENABLE_REG) & (0x01<<bit)) != 0L) return true;
    return false;
}

int rc_in_count = 0 ;
static int rcinput_int(int irq, void* data)
{
    int i, mc, irq_handled = 0;
    unsigned long capdata, stat;
    
    // detect all sensor interface
    for(mc=2; mc<4; mc++)   // MC_MODULE2 and MC_MODULE3
    {
        for(i=0; i<3; i++)
        {
            if(_check_interrupt_enable(mc, SIFB_CAP1INTBIT+i) == true &&
                    _check_interrupt_state(mc, SIFB_CAP1INTBIT+i) == true) // USER EVT
            {
                _clear_interrupt_state(mc, SIFB_CAP1INTBIT+i);
                rc_in_count++;
                irq_handled |= 0x04;
                
                while(readCapStat[i](mc, MCSIF_MODULEB) != MCPFAU_CAPFIFO_EMPTY)
                {
                    stat = readCapFIFO[i](mc, MCSIF_MODULEB, &capdata);
                    if(stat == MCPFAU_CAP_1TO0EDGE)
                    {
                        int j = (mc - 2)*3 + i ;    // remap to RC input channel
                        CH[j] = capdata ;
                        CH_flag |= (0x01 << j);
                        if( (CH_flag & 0x0F) == 0x0F)    // inform user if 1~4 channel is updated
                        {
                            CH_flag = 0 ;
                            CH_updated = true ;
                        }
                    }
                }
            }
        }
    }
    
    if(irq_handled == 0x00) return ISR_NONE;
    
    return ISR_HANDLED;
}

RCInput::RCInput()
{
    // set at middle position and loweset throttle
    CH[0] = 150000;
    CH[1] = 150000;
    CH[2] =  90000;
    CH[3] = 150000;
    CH[4] = 100000;
    CH[5] = 100000;
    CH[6] = 190000; // CH_7 default enable
    CH[7] = 100000;
    
    memset(&_override[0],0,sizeof(_override));
    _override_valid = false;
}

void RCInput::init()
{
    mcsif_Disable(MC_MODULE2, MCSIF_MODULEB);
    mcsif_Disable(MC_MODULE3, MCSIF_MODULEB);
    
    _set_MCM_CaptureMode(MC_MODULE2);
    _set_MCM_CaptureMode(MC_MODULE3);
    
    // init ISR
    _interrupt_init();
    // clear INT
    mc_outp(MC_MODULE2, 0x04, 0xff000000L); //for EX SIFB
    mc_outp(MC_MODULE3, 0x04, 0xff000000L); //for EX SIFB
    // enable pins capture interupt
    enable_MCINT(MC_MODULE2, SIFB_CAP1INTBIT);
    enable_MCINT(MC_MODULE2, SIFB_CAP2INTBIT);
    enable_MCINT(MC_MODULE2, SIFB_CAP3INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP1INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP2INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP3INTBIT);
    
    // use pin 33~38 (one) aka ENC2 and ENC3 as RC input
    _open_encoder_pin(MC_MODULE2, 0);
    _open_encoder_pin(MC_MODULE2, 1);
    _open_encoder_pin(MC_MODULE2, 2);
    _open_encoder_pin(MC_MODULE3, 0);
    _open_encoder_pin(MC_MODULE3, 1);
    _open_encoder_pin(MC_MODULE3, 2);
    
    // enable SIF module
    mcsif_Enable(MC_MODULE2, MCSIF_MODULEB);
    mcsif_Enable(MC_MODULE3, MCSIF_MODULEB);
}

bool RCInput::new_input()
{
    if (_override_valid) {
        // if we have RC overrides active, then always consider it valid
        _override_valid = false;
        return true;
    }
    return CH_updated;
}

uint8_t RCInput::num_channels()
{
    return RCINPUT_MAX_CH;
}

uint16_t RCInput::read(uint8_t ch)
{
    if( ch >= RCINPUT_MAX_CH ) return 0 ;
    
    if (_override[ch]) return _override[ch];
    
    CH_updated = false;
    return (CH[ch]/100);
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > RCINPUT_MAX_CH)   len = RCINPUT_MAX_CH;
    
    for (uint8_t i = 0; i < len; i++)
    {
        periods[i] = read(i);
    }
    return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override)
{
    if (override < 0) {
        return false; /* -1: no change. */
    }
    if (channel >= RCINPUT_MAX_CH) {
        return false;
    }
    _override[channel] = override;
    if (override != 0) {
        _override_valid = true;
        return true;
    }
    return false;
}

void RCInput::clear_overrides()
{
    for (uint8_t i = 0; i < RCINPUT_MAX_CH; i++) {
        set_override(i, 0);
    }
}

void RCInput::_open_encoder_pin(int mc, int pin) {
    unsigned short crossbar_ioaddr;
    crossbar_ioaddr = sb_Read16(0x64)&0xfffe;
    io_outpb(crossbar_ioaddr + 0x90 + PIN86[INTPINSMAP[mc*3+pin]].gpN, 0x08);//RICH IO
}


void RCInput::_set_MCM_CaptureMode( uint16_t mc)
{
    // set filterAndSampleWindowInit
    mcsif_SetInputFilter(mc, MCPWM_MODULEB, 0L);
    mcsif_SetSWDeadband(mc, MCPWM_MODULEB, 0L);
    mcsif_SetSWPOL(mc, MCPWM_MODULEB, MCSIF_SWPOL_REMAIN);
    mcsif_SetSamplWin(mc, MCPWM_MODULEB, MCSIF_SWSTART_DISABLE + MCSIF_SWEND_NOW);
    mcsif_SetSamplWin(mc, MCPWM_MODULEB, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);
    
    mcsif_SetMode(mc, MCPWM_MODULEB, MCSIF_PFAU);
    
    mcpfau_SetCapMode1(mc, MCPWM_MODULEB, MCPFAU_CAP_BOTH_CLEAR);   // capture both edge and reset counter
    mcpfau_SetCapInterval1(mc, MCPWM_MODULEB, 1L);
    mcpfau_SetCap1INT(mc, MCPWM_MODULEB, 1L);   // interrupt every capture
    mcpfau_SetPolarity1(mc, MCPWM_MODULEB, MCPFAU_POL_NORMAL);
    mcpfau_SetMask1(mc, MCPWM_MODULEB, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG1(mc, MCPWM_MODULEB, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU1TRIG(mc, MCPWM_MODULEB, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU1RELS(mc, MCPWM_MODULEB, MCPFAU_FAURELS_INPUT0);
    
    mcpfau_SetCapMode2(mc, MCPWM_MODULEB, MCPFAU_CAP_BOTH_CLEAR);   // capture both edge and reset counter
    mcpfau_SetCapInterval2(mc, MCPWM_MODULEB, 1L);
    mcpfau_SetCap2INT(mc, MCPWM_MODULEB, 1L);   // interrupt every capture
    mcpfau_SetPolarity2(mc, MCPWM_MODULEB, MCPFAU_POL_NORMAL);
    mcpfau_SetMask2(mc, MCPWM_MODULEB, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG2(mc, MCPWM_MODULEB, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU2TRIG(mc, MCPWM_MODULEB, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU2RELS(mc, MCPWM_MODULEB, MCPFAU_FAURELS_INPUT0);
    
    mcpfau_SetCapMode3(mc, MCPWM_MODULEB, MCPFAU_CAP_BOTH_CLEAR);   // capture both edge and reset counter
    mcpfau_SetCapInterval3(mc, MCPWM_MODULEB, 1L);
    mcpfau_SetCap3INT(mc, MCPWM_MODULEB, 1L);   // interrupt every capture
    mcpfau_SetPolarity3(mc, MCPWM_MODULEB, MCPFAU_POL_NORMAL);
    mcpfau_SetMask3(mc, MCPWM_MODULEB, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG3(mc, MCPWM_MODULEB, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU3TRIG(mc, MCPWM_MODULEB, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU3RELS(mc, MCPWM_MODULEB, MCPFAU_FAURELS_INPUT0);
    
}

bool RCInput::_interrupt_init(void)
{
    if(irq_InstallISR(GetMCIRQ(), rcinput_int, (void*)name) == false)
    {
        printf("irq_install fail\n"); return false;
    }
    // enable mcm general interrupt function
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_MODULE2));
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_MODULE3));
    return true;
}

}
