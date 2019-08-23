#pragma once

#include "AP_HAL_86Duino.h"

#define ANALOG_MAX_CHANNELS 7
namespace x86Duino {

class AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class x86Duino::AnalogIn;
    AnalogSource(uint8_t p) ;
    float read_average() ;
    float read_latest() ;
    void set_pin(uint8_t p) ;

    // optionally allow setting of a pin that stops the device from
    // reading. This is needed for sonar devices where you have more
    // than one sonar, and you want to stop them interfering with each
    // other. It assumes that if held low the device is stopped, if
    // held high the device starts reading.
    void set_stop_pin(uint8_t p);

    // optionally allow a settle period in milliseconds. This is only
    // used if a stop pin is set. If the settle period is non-zero
    // then the analog input code will wait to get a reading for that
    // number of milliseconds. Note that this will slow down the
    // reading of analog inputs.
    void set_settle_time(uint16_t settle_time_ms);

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    float voltage_average();

    // return a voltage from 0.0 to 5.0V, scaled
    // against a reference voltage
    float voltage_latest();

    // return a voltage from 0.0 to 5.0V, assuming a ratiometric
    // sensor
    float voltage_average_ratiometric() { return voltage_average(); }    

private:
    // what pin it is attached to
    int16_t _pin;
    int16_t _stop_pin;
    uint16_t _settle_time_ms;

    // what value it has
    float _value;
    float _latest_value;
    float _sum_value;
    uint8_t _sum_count;

    void _add_value(float v);
};

class AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init() ;
    AP_HAL::AnalogSource* channel(int16_t n) ;

    // board 5V rail voltage in volts
    float board_voltage(void) ;

    // servo rail voltage in volts, or 0 if unknown
    float servorail_voltage(void) { return 0; }

    // power supply status flags, see MAV_POWER_STATUS
    uint16_t power_status_flags(void) { return 0; }
    void update();     // must called in timer loop to sample data
private:
    x86Duino::AnalogSource* _channel[ANALOG_MAX_CHANNELS] ;
    uint32_t    AD_State;
};

}
