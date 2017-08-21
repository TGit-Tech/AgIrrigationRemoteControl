/******************************************************************************************************************//**
 * @file Device.cpp
 * @brief Class definition and container for the various Devices.
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @remarks 
 *  - Devices           - Statically ( Globally ) contains a Link-List of all Devices defined
 *  - PinPoints         - Contains a PinPoints Link-List of all pins on the Device ( i.e. Wiring )
 *  - PinPoint::XBee    - Initializes the PinPoint::XBee communications<PeerIOSerialControl> object.
 *  - LCD Keypad Shield - Initializes and handles button presses via ISR( Interrupt Service Requests )
 *  - Control           - Creates 'UserControl' objects that are attached to their Input-Pin 'PinPoint'
 *
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "Device.h"

#if DEBUG>0                             // Activate Debug Messages
  #define DBL(x) Serial.println x
  #define DBFL(x) Serial.println F(x)
  #define DB(x) Serial.print x
  #define DBF(x) Serial.print F(x)
  #define DBC Serial.print(F(", "))
#else                                   // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DBFL(x)
  #define DB(x)
  #define DBF(x)
  #define DBC  
#endif

//=====================================================================================================================
//------------------------------ Device Class -------------------------------------------------------------------------
//=====================================================================================================================
/******************************************************************************************************************//**
 * @brief Create a New Device ( Constructor )
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
Device::Device(char *_Name, byte _DeviceID ) {
  Name = _Name; DeviceID = _DeviceID;
}

/******************************************************************************************************************//**
 * @brief Creates or Accesses a Device Pin ( PinPoint object on the Device )
 * @remarks PinPoint objects are identified by their Pin#; The Very-First Pin#-access on the device creates the PinPoint.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
PinPoint* Device::Pin(uint8_t _Pin) {
  DB((Name));DB((F("::Pin(")));DB((_Pin));
  return ThisDevice::Pin(DeviceID, _Pin, Name);
}

/******************************************************************************************************************//**
 * @brief Create a UserControl on the Selected InputPin
 * @remarks Selectively only adds the Controls for 'ThisDevice'
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
UserControl* Device::Control(PinPoint *InputPin, char _ID = '?') {
  DBL((""));DBF(("Device::Control("));DB((InputPin->DeviceName));DBF((":"));DB((InputPin->Name));DBC;DB((_ID));
  
  if ( DeviceID == ThisDevice::DeviceID ) { DBFL((")")); return ThisDevice::Control(InputPin, _ID); }
  else { DBFL((") - Control is NOT for this Device!")); return &ThisDevice::NullControl; }
  
}

/******************************************************************************************************************//**
 * @brief Assign the pins and Initialize the LCD Keypad Shield
 * @remarks Selectively only starts the LCD for 'ThisDevice'
 * - LCD() must be called BEFORE Communications() to prevent issues with Interrupt activation
 * - XBee Config Mode will not work when Interrupts are activated.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void Device::LCD(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, bool _NoInterrupts ) {
  if ( DeviceID == ThisDevice::DeviceID ) { ThisDevice::LCD(p1,p2,p3,p4,p5,p6,_NoInterrupts); }
}

/******************************************************************************************************************//**
 * @brief Start serial communications with Debug & XBee Module
 * @remarks Selectively only starts Communications for 'ThisDevice'
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void Device::Communications(uint8_t RxPin, uint8_t TxPin, unsigned long _UpdateInterval, bool _XBeeConfig) {
  if ( DeviceID == ThisDevice::DeviceID ) { ThisDevice::Communications(RxPin, TxPin, _UpdateInterval, _XBeeConfig); }
}

//=====================================================================================================================
//------------------------------ ThisDevice Namespace -----------------------------------------------------------------
//=====================================================================================================================
namespace ThisDevice {

  // Objects
  SSoftwareSerial  *IOSerial = NULL;
  LiquidCrystal    *oLCD = NULL;
  UserControl      NullControl(NULL, NULL, ' ');   // NullControl for controls that are NOT for 'ThisDevice'
  PinPoint         *FirstPin = NULL;               // Pin Link-List for 'ThisDevice'
  PinPoint         *CurrPin = NULL;                // Current Pin for 'ThisDevice'
  
  // Variables
  uint8_t          DeviceID = 0;                   // Static Byte to mark which Device is 'ThisDevice'
  unsigned long    UpdateInterval = 3000;
  bool             XBeeConfig = false;
  bool             Forward = true;
  unsigned long    lastupdate = 0;
  eButton          bpress = NONE;                 // Store button presses for processing by the update()
  eButton          prev_bpress = NONE;            // Used with 'ButtonHeld' to see if same button is registered twice
  int              ButtonHeld = 0;                // Counts how long a Button has been held down
  unsigned long    last_bpress_clk = 0;           // Last time a button was pressed
  bool             AutoUpdate = true;  


  // Functions
  /******************************************************************************************************************//**
  * @brief Start serial communications with Debug & XBee Module
  * @remarks
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  PinPoint* Pin(uint8_t _DeviceID, uint8_t _Pin, char *DeviceName) {
    
    uint8_t PinDeviceID = _DeviceID;
    if ( PinDeviceID == DeviceID ) PinDeviceID = 0;
  
    // Find or Create the Pin
    if ( FirstPin == NULL ) {
      FirstPin = new PinPoint(PinDeviceID, _Pin, DeviceName);
      DB((") - CREATED! *"));DB((availableMemory()));DBFL(("Bytes Free"));
      return FirstPin;
    } else {
      PinPoint *thisPin = FirstPin;
      while ( thisPin != NULL ) {
        if ( thisPin->Device == PinDeviceID && thisPin->Pin == _Pin ) {
          DB((") - FOUND! *"));DB((availableMemory()));DBFL(("Bytes Free"));
          return thisPin;
        } 
        if ( thisPin->Next == NULL ) {
          thisPin->Next = new PinPoint(PinDeviceID, _Pin, DeviceName);
          thisPin->Next->Prev = thisPin;
          DB((") - CREATED! *"));DB((availableMemory()));DBFL(("Bytes Free"));
          return thisPin->Next;
        }
        thisPin = thisPin->Next;
      }
    }
  }

  /******************************************************************************************************************//**
  * @brief Create a PinPoint-Control on the Device
  * @remarks Returns a UserControl Object attached to the Input Pin object
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  UserControl* Control(PinPoint *InputPin, char _ID = '?') {
  
    // Create the Control
    UserControl *NewControl = new UserControl(InputPin, oLCD, _ID );
    DB(("CONTROL CREATED! *"));DB((availableMemory()));DBFL(("Bytes Free"));
    
    // Attach Control to the InputPin
    if ( InputPin->FirstControl == NULL ) { 
      InputPin->FirstControl = NewControl; 
      return InputPin->FirstControl;
    }
    else {
      UserControl *thisControl = InputPin->FirstControl;
      while ( thisControl->Next != NULL ) { thisControl = thisControl->Next; }
      thisControl->Next = NewControl;
      thisControl->Next->Prev = thisControl;
      return thisControl->Next;
    }
  }
  
  /******************************************************************************************************************//**
  * @brief Initialize an LCD Keypad Shield
  * @remarks 
  * - LCD() must be called BEFORE Communications() to prevent issues with Interrupt activation
  * - XBee Config Mode will not work when Interrupts are activated.
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  void LCD(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, bool _NoInterrupts ) {
  
    //Initialize LCD
    if ( oLCD == NULL ) oLCD = new LiquidCrystal(p1, p2, p3, p4, p5, p6);
    oLCD->begin(16, 2);
    
    if ( analogRead(A0) < 790 ) { XBeeConfig = true; }      // If SELECT button is held - enter XBeeConfig
    else if ( !_NoInterrupts ) {
      noInterrupts();               // switch interrupts off while messing with their settings  
      PCICR =0x02;                  // Enable 'PCIE1' bit of PCICR Pin Change Interrupt the PCINT1 interrupt
      PCMSK1 = 0b00000001;          // Pin Change Interrupt Mask ( NA, RESET, A5, A4, A3, A2, A1, A0 ) - Activate A0              
      interrupts();                 // turn interrupts back on
    }
  }
  
  /******************************************************************************************************************//**
  * @brief Setup & Begin serial communications for both Debug & XBee Module; including XBee Config Mode
  * @remarks
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  void Communications(uint8_t RxPin, uint8_t TxPin, unsigned long _UpdateInterval, bool _XBeeConfig) {
    UpdateInterval = _UpdateInterval;
  
    if ( !XBeeConfig ) XBeeConfig = _XBeeConfig;      // Set XBeeConfig but don't shut it OFF if LCD() set it to ON
    
    pinMode(RxPin, INPUT);                            // Setup Communications
    pinMode(TxPin, OUTPUT);
    static SSoftwareSerial IOSerial(RxPin, TxPin);    // SoftSerial required for XBee Communications & Debug
      
    IOSerial.begin(9600);                             // Start Communications
    Serial.begin(9600);

    if ( !XBeeConfig ) { PinPoint::XBee = new PeerIOSerialControl(DeviceID, IOSerial, Serial); }  // XBee Object
    else if ( oLCD != NULL ) {                      
      oLCD->begin(16, 2);                             // Show XBee Config on Display when in XBee Config Mode
      oLCD->clear();oLCD->setCursor(0,0);oLCD->print("XBee Config Mode");
      oLCD->clear();oLCD->setCursor(0,1);oLCD->print("Ver: ");oLCD->print(FIRMWAREVER);
    }

    DB((F("Device::Communications(")));               // Debug
    DB((F("Rx=")));DB((RxPin));DBC;
    DB((F("Tx=")));DB((TxPin));DBC;
    DB((F("UpdateInterval=")));DB((UpdateInterval));DBL((")"));
    DB((F("FREE MEMORY = ")));DBL((availableMemory()));
  }
  
  /******************************************************************************************************************//**
  * @brief Update() - Iterates the Device Controls
  * @remarks
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  void Update() {
  
    if ( XBeeConfig ) {                                                   // Forward communications if in XBeeConfig mode.
      if ( IOSerial->available()>0 ) Serial.write(IOSerial->read());
      if ( Serial.available()>0 ) IOSerial->write(Serial.read());
      return;    
    }
  
    if ( PinPoint::XBee != NULL ) PinPoint::XBee->Available();            // Check Communications
    if ( oLCD != NULL ) ButtonCheck(analogRead(0));          // Check for button-press if LCD attached
  
    //--------------------------------------------------------------------------------------------------------------
    // Insure a Device.Pin with a Control while moving either FORWARD(Next) or in Reverse(Prev)
    //--------------------------------------------------------------------------------------------------------------
    if ( CurrPin == NULL ) {                                  // Insure a Pin
      PinPoint *thisPin = FirstPin;                           // Start at First Pin
      if ( thisPin == NULL ) return;                          // If FirstPin is NULL there are no Pins
      if ( Forward ) { CurrPin = thisPin; }                   // Leave at FirstPin if moving FORWARD
      else {                                                              // ELSE
        while ( thisPin->Next != NULL ) { thisPin = thisPin->Next; }      // Get the Last Pin on the Device
        CurrPin = thisPin;
      }
    }
    //DB(( CurrDevice->Name ));DBF((":"));DBL(( CurrPin->Name ));
  
    if ( CurrPin->FirstControl == NULL ) {                    // Insure Pin has at least one Control
      if ( Forward ) { CurrPin = CurrPin->Next; }             // If not move Forward(NEXT)
      else { CurrPin = CurrPin->Prev; }                       // or Back(PREV)
      return;
    }
    
    //--------------------------------------------------------------------------------------------------------------
    // Read Pin Value and Display
    //--------------------------------------------------------------------------------------------------------------
    if ( CurrPin->GetState() == COMPLETE ) {
      CurrPin->ReadValue();                     // Start by requesting the Input Pin value
      DisplayPin(CurrPin);                      // Update Display
      return;
    }
          
    if ( CurrPin->GetState() == READY ) {       
      DisplayPin(CurrPin);                      // Update Display with the READY value
      CurrPin->SetState(PAUSE);                 // PAUSE until we're ready to Move-on
      
      if ( !AutoUpdate ) return;                // Don't apply Pin controls in Manual Moves
      UserControl *thisControl = CurrPin->FirstControl;
      while ( thisControl != NULL ) {
        thisControl->Apply();                   // Apply Every Control on this Pin
        thisControl = thisControl->Next; 
      }
      
      return;
    }
          
    if ( CurrPin->GetState() == PAUSE ) {
      if ( AutoUpdate ) {
        Forward = true;                                             // AutoUpdate goes Forward
        unsigned long ms = millis();
        if ((unsigned long)(ms - lastupdate) >= UpdateInterval) {
          CurrPin->SetState(COMPLETE);                              // COMPLETE last Pin State
          CurrPin = CurrPin->Next;                                  // Move to the Next Pin
          lastupdate = ms;                                          // Record AutoUpdate time
        }
      }
    }
  
    if ( CurrPin->GetState() == SETTING ) {
      if ( CurrPin->CurrControl == NULL ) {                         // Exit Controls when out-of-bounds
        CurrPin->SetState(COMPLETE);                                // COMPLETE state will re-read the CurrPin
        return;
      }
      DisplayControl(CurrPin->CurrControl);
      CurrPin->SetState(SETPAUSE);
    }
  
    if ( CurrPin->GetState() == SETPAUSE ) {
      
    }
    
    //--------------------------------------------------------------------------------------------------------------
    // Handle Button Control
    //--------------------------------------------------------------------------------------------------------------
    if ( bpress == NONE ) return;                           // No-Button to Process
    
    if (bpress == DOWN ) {
      if ( CurrPin->GetState() == SETTING || CurrPin->GetState() == SETPAUSE ) {
        if ( CurrPin->CurrControl != NULL ) {
          CurrPin->CurrControl->SetPointAdd(-1);
          DisplayControl(CurrPin->CurrControl);
        }
      } else {
        CurrPin->SetState(COMPLETE);                          // COMPLETE last Pin State
        CurrPin = CurrPin->Next;                              // Move to the Next Pin
        Forward = true;
      }
    }
    
    if ( bpress == UP ) {
      if ( CurrPin->GetState() == SETTING || CurrPin->GetState() == SETPAUSE ) {
        if ( CurrPin->CurrControl != NULL ) {
          CurrPin->CurrControl->SetPointAdd(1);
          DisplayControl(CurrPin->CurrControl);
        }
      } else {
        CurrPin->SetState(COMPLETE);                          // COMPLETE last Pin State
        CurrPin = CurrPin->Prev;                              // Move to the Prev Pin
        Forward = false;
      }
    }
    
    if ( bpress == RIGHT ) {
      CurrPin->SetState(SETTING);
      if ( CurrPin->CurrControl == NULL ) { CurrPin->CurrControl = CurrPin->FirstControl; }
      else { CurrPin->CurrControl = CurrPin->CurrControl->Next; }
  
      // Now; what to do with the Control;
      
    }
    if ( bpress == LEFT ) {
      if ( CurrPin->CurrControl != NULL ) {
        CurrPin->SetState(SETTING);
        CurrPin->CurrControl->Save();
        CurrPin->CurrControl = CurrPin->CurrControl->Prev;
      }
    }

    
    if ( bpress == SELECT ) {
      DBFL(("bpress == SELECT"));
      if ( CurrPin->CurrControl != NULL ) {
        if ( CurrPin->CurrControl->ControlType != SET_PIN ) {
          if ( CurrPin->CurrControl->IsOn() ) { CurrPin->CurrControl->IsOn(false); }
          else { CurrPin->CurrControl->IsOn(true); }
          CurrPin->SetState(SETTING);
        } else {
          CurrPin->SetState(SETTING);    
          CurrPin->CurrControl->Apply(true);
        }
      } else {
        CurrPin->SetState(COMPLETE);    // Query for an update
      }
    }
    bpress = NONE;                    // Clear the Button after Processing
  }
  
  /******************************************************************************************************************//**
  * @brief this function will return the number of bytes currently free in RAM
  * @remarks written by David A. Mellis based on code by Rob Faludi http://www.faludi.com
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  int availableMemory() {
    int size = 2048; // Use 2048 with ATmega328
    byte *buf;
  
    while ((buf = (byte *) malloc(--size)) == NULL)
      ;
  
    free(buf);
  
    return size;
  }
  
  /******************************************************************************************************************//**
  * @brief  Save the Controls 'Setpoint' to Eprom and VirtualPins
  * @remarks
  *  @code
  *    exmaple code
  *  @endcode
  **********************************************************************************************************************/
  void DisplayPin(PinPoint *InPin) {
    if ( oLCD == NULL ) return;
    if ( InPin == NULL ) return;
    
    // Display Device ( Top Row Left )
    oLCD->clear();oLCD->setCursor(0,0);
    oLCD->print( InPin->DeviceName );
  
    //Display the Pin and Read Value
    oLCD->setCursor(0,1);oLCD->print( InPin->Name );oLCD->print("=");
    
    if ( InPin->GetState() == WAIT ) { oLCD->print("?"); }
    else if ( InPin->GetState() == READY ) {
      if ( InPin->GetStatus() == ERR ) { oLCD->print("ERR"); }
      else if ( InPin->IsOnOff ) {
        if ( InPin->GetRawValue() == 0 ) { oLCD->print("Off"); }
        else { oLCD->print("On"); }
      } else {
        oLCD->print(InPin->GetModifiedValue());
      }
    }  
  }
  /******************************************************************************************************************//**
  * @brief Start serial communications with Debug & XBee Module
  * @remarks
  * @code
  *   exmaple code
  * @endcode
  **********************************************************************************************************************/
  void DisplayControl(UserControl *Ctrl) {
    
    if ( oLCD == NULL ) return;
    if ( Ctrl == NULL ) return;

    // These controls don't need a Control display
    if ( Ctrl->ControlType == LCD_READONLY || Ctrl->ControlType == TIE_PINS ) return;
    
    // Display Device ( Top Row Left )
    oLCD->clear();oLCD->setCursor(0,0);
    oLCD->print( Ctrl->InPin->DeviceName );

    // Display Control ON/OFF Status
    if ( Ctrl->ControlType != SET_PIN ) {
      if ( Ctrl->IsOn() ) { oLCD->setCursor(14,0);oLCD->print("On"); }
      else { oLCD->setCursor(13,0);oLCD->print("Off"); }
    }
  
    //Display the Pin and Setpoint
    oLCD->setCursor(0,1);oLCD->print( Ctrl->InPin->Name );oLCD->print(" ");
    
    switch ( Ctrl->ControlType ) {
      case SET_PIN:         oLCD->print("SET");oLCD->print(char(126));break; // 126 ->
      case PID_SET:         oLCD->print("PID");oLCD->print(char(126));break; // 126 ->
      case SET_CONTROLLER:  oLCD->print("SET");oLCD->print(char(126));break; // 126 ->
      case LESS_THAN:       oLCD->print(char(225));oLCD->print("<");break; // 225 = a-dots
      case GREATER_THAN:    oLCD->print(char(225));oLCD->print(">");break; // 225 = a-dots
      case EQUAL_TO:        oLCD->print(char(225));oLCD->print("=");break; // 225 = a-dots
      case NOT_EQUAL_TO:    oLCD->print(char(225));oLCD->print(char(183));break; // 225 = a-dots, 183 = slashed =
    }
    //ISOFF = 0, ISON = 1, ERR = 2, OKAY = 3

    if ( Ctrl->Status == ERR ) { oLCD->print("ERR"); }
    else {
      if ( Ctrl->InPin->IsOnOff ) {
        if ( Ctrl->SetPoint() == 0 ) { oLCD->print("Off"); }
        else { oLCD->print("On"); }
      } else {
        oLCD->print(Ctrl->SetPoint());
      }
    }
  }
  
  //=====================================================================================================================
  //------------------------------ BUTTON FUNCTIONS ---------------------------------------------------------------------
  //=====================================================================================================================
  /******************************************************************************************************************//**
  * @brief  Receives button press ( adc_value ), decodes it, and saves it to be processed by loop()
  * @remarks
  * - This function must be static because it is called by an ISR() routine
  * - Button_Debounce_ms defined in 'PeerRemoteMenu.h' only allows a button press to register every ? milliseconds.
  * @code
  *   ButtonCheck(analogRead(0));
  * @endcode
  **********************************************************************************************************************/
  void ButtonCheck(int adc_value) {
    //DB((F("ButtonCheck(")));DB((adc_value));DBL((F(")")));
    
    unsigned long clk = millis();
    if ( clk - last_bpress_clk < BUTTON_DEBOUNCE_MS ) return;         // Debounce button presses
  
    if (adc_value > 1000) { bpress = NONE; }
    else if (adc_value < 50) { bpress = RIGHT; }
    else if (adc_value < 195) { bpress = UP; }
    else if (adc_value < 380) { bpress = DOWN; }
    else if (adc_value < 555) { bpress = LEFT; }
    else if (adc_value < 790) { bpress = SELECT; }
  
    if ( prev_bpress == bpress ) { ButtonHeld++; } else { ButtonHeld=0; }   // Determine is same button is held
    prev_bpress = bpress;                                                   // Record button for above
    if ( bpress != NONE ) { last_bpress_clk = clk; }                        // Record every button time
    AutoUpdate = ( clk - last_bpress_clk > AUTO_UPDATE_AFTER_MS );
  }
  
  /******************************************************************************************************************//**
  * @brief  ISR ( Interrupt Service Routine ) for Keypad Up, Down, and Right arrow buttons.
  * @remarks
  * - PCINT1_vect Pin Change Interrupt will not trigger on Left or Select buttons ( digital threshold? )
  * - The interrupt stores the button pressed by calling ButtonCheck() and processes it when the loop() is called.
  * - The original SoftwareSerial Library calls ALL Interrupts so a modified 'SSoftwareSerial' must be used to compile
  **********************************************************************************************************************/
  ISR(PCINT1_vect) {
    //DB((F("ISR")));
    ButtonCheck(analogRead(A0));
  }
} // End Namespace
