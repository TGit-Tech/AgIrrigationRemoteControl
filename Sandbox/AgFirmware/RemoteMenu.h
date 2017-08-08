/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O Control of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _REMOTEMENU_H
#define _REMOTEMENU_H

#include "UserControl.h"
#include "PeerIOSerialControl.h"
#include <LiquidCrystal.h>
#include <SSoftwareSerial.h>

#define BUTTON_DEBOUNCE_MS    400               // How close in milliseconds two button presses will register
#define START_STATUS_ITERATE  30000             // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000              // Iterate Menu-Items every (ms); when idle
#define DEBUG                 1                 // Set to '1' to send debug messages to Serial Monitor

typedef enum Button        { RIGHT=0, UP=1, DOWN=2, LEFT=3, SELECT=4, NONE=5 };

static Button           bpress = NONE;          // Store button presses for processing by the loop()
static Button           prev_bpress = NONE;     // Used with 'ButtonHeld' to see if same button is registered twice
static int              ButtonHeld = 0;         // Counts how long a Button has been held down
static unsigned long    last_bpress_clk = 0;    // Last time a button was pressed
static unsigned long    last_iteration_clk = 0; // Last time the Menu iterated
static bool             bIterating = false;     // Whether the menu is iterating

class RemoteMenu {
  public:
    RemoteMenu(LiquidCrystal *_LCD);
    static void   ButtonCheck(int adc_value);
    
    void          Setup(uint8_t ThisDevice, bool XBeeConfig = false, int RxPin = -1, int TxPin = -1 );
    void          DeviceName( uint8_t _Device, char *_Name );
    
    void          AddPin(PinPoint *_PinPoint);
    void          Begin(PinPoint *_FirstPin);
    void          loop();
  
    PinPoint        *FirstPin = NULL;
    PinPoint        *CurrPin = NULL;
    LiquidCrystal   *LCD = NULL;
  private:
    char            *mDeviceName[16];
    void            LCD_display();
};
#endif
