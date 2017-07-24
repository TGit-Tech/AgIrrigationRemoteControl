/******************************************************************************************************************//**
 * @brief  Arduino Sketch firmware to be uploaded onto the AgIrrigationRemoteControl Hand-Remote Device.
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @remarks Version 2017.06.17
 * @todo
 *  - Implement Firmata for Base/Desktop operation
 * @authors 
 *    tgit23        01/2017       Original
 *    tgit23        07/2017       Implemented keypad button interrupts and Non-Blocking functionality
**********************************************************************************************************************/
#ifndef _PEERREMOTEMENU_h
#define _PEERREMOTEMENU_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "MenuItem.h"
#include <LiquidCrystal.h>
#include <PeerIOSerialControl.h>        //See https://github.com/tgit23/PeerIOSerialControl
#include <SSoftwareSerial.h>            //See https://github.com/tgit23/SSoftwareSerial


enum Button       { UP, DOWN, RIGHT, LEFT, SELECT, NONE };

#define NODEVICE              0
#define NOPIN                 0xFF      // A non-usable value to signify 'NOPIN' has been assigned
#define BUZZER                0xFF      // A non-usable value to signify the ALARM drive device is a local BUZZER
#define VALUE_ERR             -1        // A Value that marks an 'error'
#define VALUE_WAIT            -2        // A Value waiting for a Reply Packet
#define DEBUG                 1         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )
#define BLOCKING              0         // Blocking mode (1) stalls screen till item is gotten, (0) releases screen
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle
#define BUTTON_DEBOUNCE_MS    400       // How close in milliseconds two button presses will register
#define ISONBIT               13

volatile static unsigned long last_bpress_millis = 0;   // Track last button press time
static Button last_bpress = NONE;                       // Store last button press for processing by the loop()
static Button prev_bpress = NONE;                       // Used with 'ButtonHeld' to see if same button is registered twice
static int ButtonHeld = 0;                              // Used to count how long a Button is held by counting
static uint8_t ThisDeviceID = 0;

class PeerRemoteMenu {
  public:
    PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _ThisDeviceID, uint8_t _BuzzerPin = NOPIN );
    MenuItem* AddMenuItem( char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff = false );
    void AddDevice ( uint8_t _Device, char *_Name );        // Add System Devices
    void Start( MenuItem *StartItem );                      // Initialize Menu Setup and Set Starting Item
    static void ButtonCheck(int adc_value);                 // Button check must be public for ISR()
    void loop();

  private:
    // Class Variables
    char *Devices[16];
    bool AlarmHalt = false;                                     // Track when an Alarm should Halt the iteration
    bool bIterating = false;                                    // Alarm only active while iterating the menu
    unsigned long wait_reply = 0;                               // Track non-blocking reply time
    unsigned long wait_reply_ison = 0;
    unsigned long last_iter_millis = 0;                         // Track last status iteration time
    uint8_t BuzzerPin = NOPIN;                                  // Track which pin the BUZZER is signalled on

    // Object pointers
    LiquidCrystal *LCD;
    PeerIOSerialControl *XBee;
    MenuItem *FirstItem = NULL;
    MenuItem *CurrItem = NULL;
        
    // Function prototypes
    void EEPROMSet(unsigned int Offset, int Value, int OnOff = -1);
    int EEPROMGet(unsigned int Offset, bool *IsOn = NULL);
    void GetItem(MenuItem *Item);
    void SetPin(uint8_t _DriveDevice, uint8_t _DrivePin, int _Value );
    void CheckValue(MenuItem *Item);
    void LCD_display();
};

#endif
