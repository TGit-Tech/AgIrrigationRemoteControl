/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O Control of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _REMOTEMENU_H
#define _REMOTEMENU_H

#include "UserControl.h"
#include <LiquidCrystal.h>
#include <SSoftwareSerial.h>

//typedef enum PinStatus { ISOFF = 0, ISON = 1, WAIT = 2, ERR = 3, OKAY = 4 };
//typedef enum eControl { SET_PIN, PID_SET, CONNECT_TO, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };

#define BUTTON_DEBOUNCE_MS    400       // How close in milliseconds two button presses will register
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle
#define DEBUG                 1

typedef enum Button        { RIGHT=0, UP=1, DOWN=2, LEFT=3, SELECT=4, NONE=5 };

static Button           bpress = NONE;                       // Store button presses for processing by the loop()
static Button           prev_bpress = NONE;                       // Used with 'ButtonHeld' to see if same button is registered twice
static int              ButtonHeld = 0;                              // Used to count how long a Button is held by counting
static unsigned long    last_bpress_clk = 0;
static unsigned long    last_iteration_clk = 0;
static bool             bIterating = false;

class RemoteMenu {
  public:
    RemoteMenu(uint8_t RxPin, uint8_t TxPin, uint8_t ThisDevice, LiquidCrystal *_LCD);
    void          DeviceDisplayName( uint8_t _Device, char *_Name );        // Add System Devices
    static void   ButtonCheck(int adc_value);                 // Button check must be public for ISR()
    void          AddPin(PinPoint *_PinPoint);
    void          StartDisplay(PinPoint *_FirstPin);
    void          loop();
  
    PinPoint        *FirstPin = NULL;
    PinPoint        *CurrPin = NULL;
    LiquidCrystal   *LCD = NULL;
  private:
    char            *mDeviceDisplayName[16];
    void            LCD_display();

};
#endif
