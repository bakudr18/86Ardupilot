#pragma once

#include "AP_HAL_86Duino.h"

class x86Duino::USBSerial : public AP_HAL::UARTDriver {
public:
    USBSerial() ;
    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Empty implementations of Stream virtual methods */
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    // 86duino additional API
    int peek(void);
    bool isConnected(void);
    operator bool();

private:
    bool peek_stored;
    int peek_val;
    void* USBDEV;
};
