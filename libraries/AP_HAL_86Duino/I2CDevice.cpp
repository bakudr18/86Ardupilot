/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Scheduler.h"
#include <stdio.h>

#include "I2CDevice.h"
#include "io.h"
#include "i2c.h"

extern const AP_HAL::HAL& hal ;

namespace x86Duino {

Semaphore I2CDevice::i2c_semaphore;

I2CDevice::I2CDevice(uint8_t address) :
    _address(address)
{
    set_device_bus(0);  //I2C0
    set_device_address(address);
}

I2CDevice::~I2CDevice()
{
    hal.console->printf("I2C device bus 0 address 0x%02x closed\n",
           (unsigned)_address);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if( send_len )
    {
        i2cmaster_StartN(0, _address, I2C_WRITE, send_len);
        for( uint32_t i = 0 ; i < send_len ; i++ )
            i2cmaster_WriteN( 0, send[i]);
    }

    if( recv_len )
    {
        i2cmaster_StartN(0, _address, I2C_READ, recv_len);
        for( uint32_t i = 0 ; i < recv_len ; i++ )
            recv[i] = i2cmaster_ReadN(0) ;
    }
    return true;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &i2c_semaphore;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return ((Scheduler*)hal.scheduler)->register_i2c_process( period_usec, cb) ;
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    if( !_is_initailized )  init();
    if( bus != 0 ) return nullptr ;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(address));
    return dev;
}

void I2CDeviceManager::init(void)
{
    i2c_Init2(0xFB00, I2C_USEMODULE0, I2CIRQ_DISABLE, I2CIRQ_DISABLE);
    i2c_SetSpeed(0, I2CMODE_AUTO, 400000L);
    _is_initailized = true;
}

}
