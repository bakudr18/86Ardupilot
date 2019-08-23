#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
#include "HAL_86Duino_Class.h"

#endif // CONFIG_HAL_BOARD
