/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition (I/O PinPoints, Controls, LCD) on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _DEVICE_H
#define _DEVICE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "PinPoint.h"
#include "UserControl.h"
#include <LiquidCrystal.h>
#include <SSoftwareSerial.h>

#define DEBUG                 1                 // Set to '1' to send debug messages to Serial Monitor
#define FIRMWAREVER           20170814          // Firmware release date ( Displayed during XBeeConfig Mode )
#define BUTTON_DEBOUNCE_MS    400               // How close in milliseconds two button presses will register  
#define AUTO_UPDATE_AFTER_MS  30000             // Start iterating Menu-Items after idle for (ms)

#ifndef _ECONTROL
typedef enum eState : byte { WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE };
#define _ECONTROL
#endif
typedef enum eButton : byte { RIGHT, UP, DOWN, LEFT, SELECT, NONE };

//----------------------------------------------------------------------------------------------------------------------
namespace ThisDevice {
//----------------------------------------------------------------------------------------------------------------------
  // Functions
  void              Update();
  void              LCD(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, bool _NoInterrupts );
  void              Communications(uint8_t RxPin, uint8_t TxPin, unsigned long _UpdateInterval, bool _XBeeConfig);
  PinPoint*         Pin(uint8_t _DeviceID, uint8_t _Pin, char *DeviceName);                    // Add or Gets a PinPoint to the Device
  UserControl*      Control(PinPoint *InputPin, char _ID = '?');                // Returns a new Control Object attached to the 'InputPin'
  
  void              DisplayControl(UserControl *Ctrl);
  void              DisplayPin(PinPoint *InPin);
  
  void              ButtonCheck(int adc_value);
  int               availableMemory();

  // Objects
  extern SSoftwareSerial  *IOSerial;
  extern LiquidCrystal    *oLCD;                // Contains pointer to LCD Control Object
  extern UserControl      NullControl;          // NullControl for controls that are NOT for 'ThisDevice'
  extern PinPoint         *FirstPin;            // Pin Link-List for 'ThisDevice'
  extern PinPoint         *CurrPin;             // Current Pin for 'ThisDevice'
  

  // Variables
  extern uint8_t          DeviceID;             // Static Byte to mark which Device is 'ThisDevice'
  extern bool             XBeeConfig;           // True - when in XBee Config Mode

  extern bool             Forward;              // True if Iterating the Pins forward ( DOWN/NEXT )
  extern bool             AutoUpdate;           // True - If its time to iterate controls  
  extern unsigned long    lastupdate;
  extern unsigned long    UpdateInterval;       // Time lapse between executing each Control
  
  // LCD Variables
  extern eButton          bpress;               // Store button presses for processing by the update()
  extern eButton          prev_bpress;          // Used with 'ButtonHeld' to see if same button is registered twice
  extern int              ButtonHeld;           // Counts how long a Button has been held down
  extern unsigned long    last_bpress_clk;      // Last time a button was pressed
}

//----------------------------------------------------------------------------------------------------------------------
class Device {
//----------------------------------------------------------------------------------------------------------------------
  public:
    Device(char *_Name, uint8_t _DeviceID );      // Constructor
    
    byte            DeviceID = 0;
        
    void            LCD(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, bool _NoInterrupts );
    void            Communications(uint8_t RxPin, uint8_t TxPin, unsigned long _UpdateInterval, bool _XBeeConfig);
    PinPoint*       Pin(uint8_t _Pin);                            // Add or Gets a PinPoint to the Device
    UserControl*    Control(PinPoint *InputPin, char _ID = '?');  // Returns a new Control Object attached to the 'InputPin'

  private:
    char            *Name;
};
#endif
