#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

#include "AP_HAL_86Duino.h"
#include "AP_HAL_86Duino_Namespace.h"

class HAL_86Duino : public AP_HAL::HAL {
public:
    HAL_86Duino();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;

private:
//    HALSITL::SITL_State *_sitl_state;
};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
