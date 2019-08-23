#ifndef AP_OPTICALFLOW_MAVLINK_H
#define AP_OPTICALFLOW_MAVLINK_H

#pragma once

#include "OpticalFlow.h"

class AP_OpticalFlow_MAVLink : public OpticalFlow_backend
{
public:
    AP_OpticalFlow_MAVLink(OpticalFlow &_frontend);
    void init(void);
    void update(void);
    void handle_msg(mavlink_message_t *msg) override;
private:
    uint32_t _last_read_ms;
    uint32_t _integral_count ;
    uint32_t _integral_quality ;
    float _integral_flow_x, _integral_flow_y;
    float _integral_body_x, _integral_body_y;
};

#endif // AP_OPTICALFLOW_MAVLINK_H
