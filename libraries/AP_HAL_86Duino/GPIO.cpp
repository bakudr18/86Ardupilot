
#include "GPIO.h"
#include "v86clock.h"
#include "io.h"
#include "pins_arduino.h"
#include "USBSerial.h"  // for checking usb connection

extern const AP_HAL::HAL& hal ;
namespace x86Duino {

#define TRI_STATE     (0x00)
#define PULL_UP       (0x01)
#define PULL_DOWN     (0x02)

//----------------------------- Platform Pin define ----------------------------
#if defined (__86DUINO_ONE) || defined (__86DUINO_ZERO) || defined (__86DUINO_EDUCAKE)
#define PINS                       (52) // GPIO PINS (45) + A/D PINS (7) = 52 pINS
#define EXTERNAL_NUM_INTERRUPTS    (12)
#define CRBTABLE_SIZE              (0xC0)

#define ENCSSI_MODE_AVAILABLE

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t LED_BUILTIN = 13;

static const uint8_t A0 = 45;
static const uint8_t A1 = 46;
static const uint8_t A2 = 47;
static const uint8_t A3 = 48;
static const uint8_t A4 = 49;
static const uint8_t A5 = 50;
static const uint8_t A6 = 51;

static const uint8_t SDA = 52;
static const uint8_t SCL = 53;

struct pinsconfig PIN86[PINS] =
{
{11, NOSTATUS, NOSTATUS, 0, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 0
{10, NOSTATUS, NOSTATUS, 1, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 1
{39, NOSTATUS, NOSTATUS, 2, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 2
{23, NOSTATUS, NOSTATUS, 3, MC_MODULE3, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 3
{37, NOSTATUS, NOSTATUS, 4, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 4
{20, NOSTATUS, NOSTATUS, 5, MC_MODULE0, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 5
{19, NOSTATUS, NOSTATUS, 6, MC_MODULE0, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 6
{35, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 7
{33, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 8
{17, NOSTATUS, NOSTATUS, NOAD, MC_MODULE0, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 9
{28, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 10
{27, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 11
{32, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 12
{25, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 13
{12, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 14
{13, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 15
{14, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 16
{15, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 17
{24, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 18
{26, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 19
{29, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 20
{47, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 21
{46, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 22
{45, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 23
{44, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 24
{43, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 25
{42, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 26
{41, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 27
{40, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 28
{ 1, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 29
{ 3, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 30
{ 4, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 31
{31, NOSTATUS, NOSTATUS, NOAD, MC_MODULE3, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 32
{ 0, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 33
{ 2, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 34
{ 5, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 35
{22, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 36
{30, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 37
{ 6, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 38
{38, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 39
{36, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 40
{34, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 41
{16, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 42
{18, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 43
{21, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 44
{NOUSED, NOSTATUS, NOSTATUS, 0, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 45
{NOUSED, NOSTATUS, NOSTATUS, 1, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 46
{NOUSED, NOSTATUS, NOSTATUS, 2, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 47
{NOUSED, NOSTATUS, NOSTATUS, 3, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 48
{NOUSED, NOSTATUS, NOSTATUS, 4, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 49
{NOUSED, NOSTATUS, NOSTATUS, 5, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 50
{NOUSED, NOSTATUS, NOSTATUS, 6, NOPWM, NOPWM, NOENC, NOENC, NULL} // pin 51
};

int INTPINSMAP[EXTERNAL_NUM_INTERRUPTS] = {42, 43, 44, 18, 19, 20, 33, 34, 35, 36, 37, 38};

#if defined CRB_DEBUGMODE
static int CROSSBARTABLE[CRBTABLE_SIZE] = {0x03, 0x08, 0x01, 0x02, 0x00, 0x00, 0x06, 0x00, 0x0A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x0E, 0x0D, 0x01, 0x02, 0x05, 0x06, 0x03, 0x04, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x1F, 0x11, 0x12,
                                           0x09, 0x0A, 0x0C, 0x0B, 0x07, 0x08, 0x0F, 0x10, 0xF0, 0x0F, 0x01, 0xF8, 0x7F, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x01, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x08, 0x01, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x08, 0x08, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
#endif

#elif defined (__86DUINO_PLC)

// #define PLCBOARDID

#define PINS                       (21)
#define EXTERNAL_NUM_INTERRUPTS    (4)
#define CRBTABLE_SIZE              (0xC0)

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t LED_BUILTIN = 13;

static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;
static const uint8_t A6 = 20;

static const uint8_t SDA = 21;
static const uint8_t SCL = 22;

static struct pinsconfig PIN86[PINS] =
{
{1, OUTPUT, LOW, NOAD, MC_MODULE0, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 0
{3, OUTPUT, LOW, NOAD, MC_MODULE0, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 1
{4, OUTPUT, LOW, NOAD, MC_MODULE0, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 2
{7, OUTPUT, LOW, NOAD, MC_MODULE3, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 3
{0, INPUT, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 4
{2, INPUT, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 5
{5, INPUT, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 6
{6, INPUT, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 7
{NOUSED, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 8
{NOUSED, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 9
{NOUSED, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 10
{NOUSED, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 11
{NOUSED, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 12
{8, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 13
{NOUSED, NOSTATUS, NOSTATUS, 0, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 14
{NOUSED, NOSTATUS, NOSTATUS, 1, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 15
{NOUSED, NOSTATUS, NOSTATUS, 2, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 16
{NOUSED, NOSTATUS, NOSTATUS, 3, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 17
{NOUSED, NOSTATUS, NOSTATUS, 4, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 18
{NOUSED, NOSTATUS, NOSTATUS, 5, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 19
{NOUSED, NOSTATUS, NOSTATUS, 6, NOPWM, NOPWM, NOENC, NOENC, NULL} // pin 20
};

static int INTPINSMAP[EXTERNAL_NUM_INTERRUPTS] = {4, 5, 6, 7};

#if defined CRB_DEBUGMODE
static int CROSSBARTABLE[CRBTABLE_SIZE] = {0x01, 0x00, 0x00, 0x08, 0x00, 0x00, 0x06, 0x00, 0x09, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x0E, 0x0D, 0x07, 0x08, 0x00, 0x1E, 0x1F, 0x21, 0x09, 0x0A, 0x0C, 0x0B, 0x17, 0x18, 0x0F, 0x10,
                                           0x27, 0x28, 0x29, 0x2A, 0x2B, 0x1A, 0x11, 0x12, 0x00, 0x00, 0x01, 0xE0, 0x7F, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01,
                                           0x01, 0x01, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x08, 0x01, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08};
#endif

#elif defined (__86DUINO_AI)
#define PINS                       (40) // GPIO PINS (38) + A/D PINS (2) = 40 PINS
#define EXTERNAL_NUM_INTERRUPTS    (12)
#define CRBTABLE_SIZE              (0xC0)

#define ENCSSI_MODE_AVAILABLE

static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t LED_BUILTIN = 13;

static const uint8_t A0 = 38;
static const uint8_t A1 = 39;

static const uint8_t SDA = 40;
static const uint8_t SCL = 41;

struct pinsconfig PIN86[PINS] =
{
{25, NOSTATUS, NOSTATUS, 0, MC_MODULE0, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 0
{27, NOSTATUS, NOSTATUS, 1, MC_MODULE0, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 1
{28, NOSTATUS, NOSTATUS, NOAD, MC_MODULE0, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 2
{31, NOSTATUS, NOSTATUS, NOAD, MC_MODULE3, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 3
{24, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 4
{26, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 5
{29, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE0, MCPWM_MODULEB, NULL}, // pin 6
{17, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 7
{19, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 8
{20, NOSTATUS, NOSTATUS, NOAD, MC_MODULE1, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 9
{16, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 10
{18, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 11
{21, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE1, MCPWM_MODULEB, NULL}, // pin 12
{ 1, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEA, NOENC, NOENC, NULL}, // pin 13
{ 3, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 14
{ 4, NOSTATUS, NOSTATUS, NOAD, MC_MODULE2, MCPWM_MODULEC, NOENC, NOENC, NULL}, // pin 15
{ 0, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 16
{ 2, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 17
{ 5, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE2, MCPWM_MODULEB, NULL}, // pin 18
{30, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 19
{22, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 20
{ 6, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, MC_MODULE3, MCPWM_MODULEB, NULL}, // pin 21
{ 7, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 22
{23, NOSTATUS, NOSTATUS, NOAD, MC_MODULE3, MCPWM_MODULEB, NOENC, NOENC, NULL}, // pin 23
{64, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 24
{65, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 25
{12, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 26
{13, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 27
{14, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 28
{15, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 29
{66, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 30
{67, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 31
{68, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 32
{69, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 33
{ 8, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 34
{ 9, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 35
{10, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 36
{11, NOSTATUS, NOSTATUS, NOAD, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 37
{NOUSED, NOSTATUS, NOSTATUS, 0, NOPWM, NOPWM, NOENC, NOENC, NULL}, // pin 38
{NOUSED, NOSTATUS, NOSTATUS, 1, NOPWM, NOPWM, NOENC, NOENC, NULL} // pin 39
};

int INTPINSMAP[EXTERNAL_NUM_INTERRUPTS] = {4, 5, 6, 10, 11, 12, 16, 17, 18, 19, 20, 21};

#if defined CRB_DEBUGMODE
static int CROSSBARTABLE[CRBTABLE_SIZE] = {0x03, 0x08, 0x02, 0x01, 0x00, 0x09, 0x06, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1E, 0x11, 0x12,
                                           0x09, 0x0A, 0x0C, 0x0B, 0x00, 0x00, 0x0E, 0x0D, 0xC0, 0x0B, 0x03, 0xE0, 0xFF, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x01, 0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x01, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x08, 0x08, 0x01, 0x01, 0x01, 0x01,
                                           0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
#endif

#endif
//----------------------------- Platform Pin define ----------------------------

GPIO::GPIO()
{}

void GPIO::init()
{
    int gpioBase, i;
    //set SB GPIO Base Address
    gpioBase = sb_Read16(SB_GPIOBASE) & 0xfffe;
    if(gpioBase == 0 || gpioBase == 0xfffe)
    {
        sb_Write16(SB_GPIOBASE, GPIOCTRLBASE | 0x01);
        gpioBase = GPIOCTRLBASE;
    }
    
    // Enable GPIO 0 ~ 9
    io_outpdw(gpioBase, 0x03ff);
    
    // set GPIO Port 0~7 dircetion & data Address
    for(i=0;i<10;i++)
        io_outpdw(gpioBase + (i+1)*4,((GPIODIRBASE + i*4)<<16) + GPIODATABASE + i*4);
    
    setPinStatus();
}

void GPIO::pinMode(uint8_t pin, uint8_t mode)
{
    int crossbar_bit;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return;
    
    crossbar_bit = PIN86[pin].gpN;
    
    io_DisableINT();
    if (mode == INPUT)  // HAL_GPIO_INPUT
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, TRI_STATE);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else if(mode == INPUT_PULLDOWN)
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, PULL_DOWN);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else if (mode == INPUT_PULLUP)  // HAL_GPIO_ALT
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, PULL_UP);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else    // HAL_GPIO_OUTPUT
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))|(1<<(crossbar_bit%8)));
    
    io_RestoreINT();
}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t GPIO::read(uint8_t pin) {
    int crossbar_bit, val;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return LOW;
    
    crossbar_bit = PIN86[pin].gpN;
    
    //#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
    //    if(pin == 32) timer1_pin32_isUsed = true;
    //#endif
    
    io_DisableINT();
    
    if(crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit/8), 0x01);
    else if(crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
        //        Close_Pwm(pin);   // important!!
    }
    
    val = io_inpb(GPIODATABASE + 4*(crossbar_bit/8))&(1<<(crossbar_bit%8));
    
    io_RestoreINT();
    
    if(val != 0) return HIGH;
    return LOW;
}

void GPIO::write(uint8_t pin, uint8_t val)
{
    unsigned int port;
    unsigned int value;
    int crossbar_bit;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return;
    
    //#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
    //    if(pin == 32) timer1_pin32_isUsed = true;
    //#endif
    
    crossbar_bit = PIN86[pin].gpN;
    port = GPIODATABASE + 4*(crossbar_bit/8);
    value = 1<<(crossbar_bit%8);
    
    io_DisableINT();
    
    if(crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit/8), 0x01);
    else if(crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
        //        Close_Pwm(pin);    // important!!
    }
    
    if (val == LOW)
        io_outpb(port, io_inpb(port)&(~value));
    else
        io_outpb(port, io_inpb(port)|value);
    
    io_RestoreINT();
}

void GPIO::toggle(uint8_t pin)
{
    write( pin, !read(pin));
}

void GPIO::setPinStatus(void)
{
    int i;
    for(i=0; i<PINS; i++)
    {
        if(PIN86[i].gpMode != NOSTATUS) pinMode(i, PIN86[i].gpMode);
        if(PIN86[i].gpOutput != NOSTATUS) write(i, PIN86[i].gpOutput);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                            uint8_t mode) {
    return true;
}

bool GPIO::usb_connected(void)
{    
    return ((USBSerial*)hal.uartA)->isConnected();
}

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t DigitalSource::read() {
    return hal.gpio->read(_v);
}

void DigitalSource::write(uint8_t value) {
    return hal.gpio->write(_v,value);
}

void DigitalSource::toggle() {
    write(!read());
}

}
