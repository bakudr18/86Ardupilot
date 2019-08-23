#include <cstdio>

#include "USBSerial.h"
#include "USBCore.h"

namespace x86Duino {

USBSerial::USBSerial()
{
    peek_stored = false;
    peek_val = -1;
    USBDEV = NULL;
}


void USBSerial::begin(uint32_t b)
{
    peek_stored = false;
    peek_val = -1;

    if( USBDEV != NULL ) return;
    //CDC
    USBDEV = CreateUSBDevice();
    if(USBDEV == NULL)
    {
        std::printf("USB(CDC) create error\n");
    }

    usb_SetUSBPins(USBDEV, 7, 0, 7, 1);
    usb_SetTimeOut(USBDEV, 0L, 500L); // USB RX timerout is 0ms and TX timeout is 500ms
    if(usb_Init(USBDEV) == false)
    {
        std::printf("USB(CDC) init error\n");
        USBDEV = NULL;
    }
}

void USBSerial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    peek_stored = false;
    peek_val = -1;
}

void USBSerial::end()
{
    if(USBDEV == NULL) return;
    usb_Close(USBDEV);
    USBDEV = NULL;
}

void USBSerial::flush()
{
    if(USBDEV == NULL) return;
    usb_FlushTxQueue(USBDEV);
}

bool USBSerial::is_initialized()
{
    bool result = false;
    if (usb_Ready(USBDEV) != false)
            result = true;
//    timer_Delay(10);  // temp commit nasamit
    return result;
}
void USBSerial::set_blocking_writes(bool blocking) {}
bool USBSerial::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t USBSerial::available()
{
    if(USBDEV == NULL) return 0;
    return usb_QueryRxQueue(USBDEV);
}

uint32_t USBSerial::txspace()
{
    if(USBDEV == NULL) return 0;
    return ( 4096 - usb_QueryTxQueue(USBDEV) );
}

int16_t USBSerial::read()
{
    int c;
    if(USBDEV == NULL) return -1;
    if(peek_stored == true)
    {
        peek_stored = false;
        return peek_val;
    }
    else
    {
        c = usb_Read(USBDEV);
        return (c == 0xFFFF) ? -1 : c;
    }
}

/* Empty implementations of Print virtual methods */
size_t USBSerial::write(uint8_t c)
{
    if (USBDEV != NULL && usb_Ready(USBDEV) != false)
    {
        return (usb_Write(USBDEV, c) == true) ? 1 : 0;
    }

    return 0;
}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{
    if (USBDEV != NULL && usb_Ready(USBDEV) != false)
    {
        int r = usb_Send(USBDEV, (uint8_t *)buffer, size);
        return (r > 0) ? r : 0;
    }
    return 0;
}

// 86duino additional API
int USBSerial::peek(void)
{
    if(USBDEV == NULL) return -1;
    if(peek_stored == true)
        return peek_val;
    else
    {
        if((peek_val = usb_Read(USBDEV)) == 0xFFFF)
            return -1;//peek_val = -1;
        peek_stored = true;
        return peek_val;
    }
}

bool USBSerial::isConnected()
{
    // return ture if CDC is connected!
    if(USBDEV == NULL) return false;
    if( usb_State(USBDEV) == USB_DEV_CDC_CONNECT ) return true;
    return false;
}


USBSerial::operator bool() {
	bool result = false;
	if (usb_Ready(USBDEV) != false)
		result = true;
//	timer_Delay(10);    // temp commit @nasamit
	return result;
}

}
