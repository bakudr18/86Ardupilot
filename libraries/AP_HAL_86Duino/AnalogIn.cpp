#include "AnalogIn.h"
#include "io.h"

extern const AP_HAL::HAL& hal ;

namespace x86Duino {
#define BaseAddress (0xfe00)
#define TimeOut     (1000)
#define MCM_MC      (0)
#define MCM_MD      (1)
#define ADC_RESOLUTION    (2048) // for 86Duino, 11bits

#define AD_START    0
#define AD_READ     1

AnalogSource::AnalogSource(uint8_t p)
{
    _pin = p ;
    _value = 0;
    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0 ;
}

float AnalogSource::read_average()
{
    if (_sum_count == 0) {
        return _value;
    }
    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;
    return _value;
}

float AnalogSource::voltage_average()
{
    read_average();
    return 3.3 * _value / 2048.0f;
}

float AnalogSource::voltage_latest() {
    return 3.3 *_latest_value / 2048.0f;
}

void AnalogSource::_add_value(float v)
{
    _latest_value = v;
    _sum_value += v;
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}

float AnalogSource::read_latest() {
    return _latest_value;
}

void AnalogSource::set_pin(uint8_t p)
{
    if( _pin == p ) return ;
    _pin = p ;
    _value = 0;
    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0 ;
}

void AnalogSource::set_stop_pin(uint8_t p)
{
    _stop_pin = p ;
}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{
    _settle_time_ms = settle_time_ms ;
}

AnalogIn::AnalogIn()
{
    AD_State = AD_START;
    for(int i = 0 ; i < ANALOG_MAX_CHANNELS ; i++ )
    {
        _channel[i] = new x86Duino::AnalogSource(i);
    }
}

void AnalogIn::init()
{
    // set ADC Base Address
    sb_Write(0xbc, sb_Read(0xbc) & (~(1L<<28)));  // active adc
    sb1_Write16(0xde, sb1_Read16(0xde) | 0x02);   // not Available for 8051A Access ADC
    sb1_Write(0xe0, 0x0050fe00L); // baseaddr = 0xfe00, disable irq
    io_outpb(0xfe01, 0x00);
    for(int i=0; (io_inpb(0xfe02) & 0x01) != 0 && i<16; i++) io_inpb(0xfe04); // clear ADC FIFO
}

void AnalogIn::update()
{
    unsigned long d;
//    static int count = 0 ;

    switch( AD_State )
    {
    default:
    case AD_START:  // set AD start sample
        io_DisableINT();

        io_outpb(BaseAddress + 1, 0x08); // disable ADC
        io_outpb(BaseAddress + 0, 0x7F); // ch 0~6
        io_outpb(BaseAddress + 1, 0x01); // enable ADC_ST

        io_RestoreINT();
        AD_State = AD_READ;
        break;

    case AD_READ:   // read AD value
        if( (io_inpb(BaseAddress + 2) & 0x01) == 0) break;  // not ready!

//        count++;
        for( int i = 0; i<ANALOG_MAX_CHANNELS ; i++)
        {
            d = io_inpw(BaseAddress + 4);
            int16_t ad = d & 0x07FF;
            int16_t ch = (d & 0xE000)>>13;
            for( int j = 0; j<ANALOG_MAX_CHANNELS ; j++)
            {
                x86Duino::AnalogSource *c = _channel[j];
                if( c->_pin == ch ) c->_add_value(ad);
            }
        }
        AD_State = AD_START;
        break;
    }
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n)
{
    if( n >= ANALOG_MAX_CHANNELS ) return nullptr ;
    return _channel[n] ;
}

float AnalogIn::board_voltage(void)
{
    return 5.0f;
}

}
