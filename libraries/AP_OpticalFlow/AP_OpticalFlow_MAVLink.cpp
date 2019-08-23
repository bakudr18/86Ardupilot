#include "AP_OpticalFlow_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "OpticalFlow.h"
extern const AP_HAL::HAL& hal;

AP_OpticalFlow_MAVLink::AP_OpticalFlow_MAVLink(OpticalFlow &_frontend):
    OpticalFlow_backend(_frontend) 
{
    _integral_count = 0 ;
    _last_read_ms = AP_HAL::millis();
}

void AP_OpticalFlow_MAVLink::init()
{
    
}

void AP_OpticalFlow_MAVLink::update()
{
    // read at 10 hz
    if( _integral_count > 0 && AP_HAL::millis() - _last_read_ms > 100 )
    {
        struct OpticalFlow::OpticalFlow_state state;
        state.device_id = 0;
        state.surface_quality = _integral_quality/_integral_count; 
        
        state.flowRate = Vector2f(_integral_flow_x/_integral_count,
                                  _integral_flow_y/_integral_count );
        
        // delta_time is in microseconds so multiply to get back to seconds
        state.bodyRate = Vector2f(_integral_body_x/_integral_count ,_integral_body_y/_integral_count);
        
        _applyYaw(state.flowRate);
        // copy results to front end
        _update_frontend(state);
        
        _last_read_ms = AP_HAL::millis();
        _integral_count = 0 ;
        _integral_flow_x = 0 ;
        _integral_flow_y = 0 ;
        _integral_body_x = 0 ;
        _integral_body_y = 0 ;
    }
}

void AP_OpticalFlow_MAVLink::handle_msg(mavlink_message_t *msg)
{
    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(msg, &packet);
       
    const Vector3f &gyro = get_ahrs().get_gyro();
        
    const Vector2f flowScaler = _flowScaler();
    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
    
    _integral_count++;
    _integral_quality += packet.quality; 
    _integral_flow_x += packet.flow_rate_x * flowScaleFactorX ;
    _integral_flow_y += packet.flow_rate_y * flowScaleFactorY ;
    
    _integral_body_x += gyro.x ;
    _integral_body_y += gyro.y ;
    
}
