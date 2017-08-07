/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O Control of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "RemoteMenu.h"
#include "PeerIOSerialControl.h"


#if XBEECONFIG>0                        // XBEECONFIG defined in PeerRemoteMenu.h
  #undef BLOCKING                       // Undefine any previously defined BLOCKING
  #define BLOCKING 1                    // Digi-Xbee-XCTU configuration software has problems with active interupts
#endif

#if DEBUG>0                             // Activate Debug Messages ( DEBUG defined in PeerRemoteMenu.h )
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else                                   // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif

//=====================================================================================================================
//------------------------------ STATIC FUNCTIONS ---------------------------------------------------------------------
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
static void RemoteMenu::ButtonCheck(int adc_value) {
  //DB(("ButtonCheck("));DB((adc_value));DBL((")"));
  
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
  if ( bpress != NONE ) { 
    bIterating = false;
    last_bpress_clk = clk; }                        // Record every button time
  else {
    if ( clk - last_bpress_clk > START_STATUS_ITERATE ) {                 // Time to start iterating?
      if ( clk - last_iteration_clk > ITERATE_EVERY ) {                   // Time to iterate again?
        bIterating = true;
        bpress = DOWN;last_iteration_clk = clk;                             // DOWN menu to iterate
      }
    }
  }
}

#if BLOCKING==0
/******************************************************************************************************************//**
 * @brief  ISR ( Interrupt Service Routine ) for Keypad Up, Down, and Right arrow buttons.
 * @remarks
 * - PCINT1_vect Pin Change Interrupt will not trigger on Left or Select buttons ( digital threshold? )
 * - The interrupt stores the button pressed by calling ButtonCheck() and processes it when the loop() is called.
 * - The original SoftwareSerial Library calls ALL Interrupts so a modified 'SSoftwareSerial' must be used to compile
**********************************************************************************************************************/
ISR(PCINT1_vect) {
  RemoteMenu::ButtonCheck(analogRead(0));
}
#endif

/******************************************************************************************************************//**
 * @brief Constructor
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
RemoteMenu::RemoteMenu(uint8_t RxPin, uint8_t TxPin, uint8_t ThisDevice, LiquidCrystal *_LCD) {
  DB(("RemoteMenu::RemoteMenu("));DB((RxPin));DBC;DB((TxPin));DBC;DB((ThisDevice));DBL((")"));
  
  // Start Communications
  pinMode(RxPin, INPUT);
  pinMode(TxPin, OUTPUT);
  static SSoftwareSerial IOSerial(RxPin, TxPin);
  PinPoint::XBee = new PeerIOSerialControl(ThisDevice, IOSerial, Serial);
  PinPoint::ThisDeviceID = ThisDevice;
  LCD = _LCD;

#if BLOCKING==0
  noInterrupts();               // switch interrupts off while messing with their settings  
  PCICR =0x02;                  // Enable 'PCIE1' bit of PCICR Pin Change Interrupt the PCINT1 interrupt
  PCMSK1 = 0b00000001;          // Pin Change Interrupt Mask ( NA, RESET, A5, A4, A3, A2, A1, A0 ) - Activate A0              
  interrupts();                 // turn interrupts back on
#endif

}

/******************************************************************************************************************//**
 * @brief AddPin
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::AddPin(PinPoint *_PinPoint) {
  if ( FirstPin == NULL ) { FirstPin = _PinPoint; }
  else {
    PinPoint *thisPin = FirstPin;
    while ( thisPin->Next != NULL ) { thisPin = thisPin->Next; }
    thisPin->Next = _PinPoint;
    thisPin->Next->Prev = thisPin;
  }
}

void RemoteMenu::StartDisplay(PinPoint *StartingPin) {
  CurrPin = StartingPin;
  LCD_display();
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::DeviceDisplayName( uint8_t _Device, char *_Name ) {
  DB(("RemoteMenu::DeviceDisplayName("));DB((_Device));DBC;DB((_Name));DBL((")"));
  if ( _Device > 0 && _Device < 16 ) mDeviceDisplayName[_Device] = _Name;
}

/******************************************************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::LCD_display() {
  DB(("LCD_display("));DB((CurrPin->Name));DBL((")"));

  // Display Device ( Top Row Left )
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( mDeviceDisplayName[CurrPin->Device] );

  if ( CurrPin->CurrControl != NULL ) {
    if ( CurrPin->CurrControl->ControlType != SET_PIN ) {
      if ( CurrPin->CurrControl->IsOn() ) { LCD->setCursor(14,0);LCD->print("On"); }
      else { LCD->setCursor(13,0);LCD->print("Off"); }
    }
  }
/*
  //char *Status;
  char Status[20] = "";
  CurrPin->GetStatusLine(Status);
  if ( strlen(Status) + strlen(Devices[CurrPin->Device]) > 16 ) { LCD->setCursor(strlen(Devices[CurrPin->Device]),0); }
  else { LCD->setCursor(16-strlen(Status),0); }
  LCD->print(Status);
*/

  //Display the Bottom Row
  LCD->setCursor(0,1);LCD->print(CurrPin->Name);LCD->print(" ");
  if ( CurrPin->CurrControl == NULL ) {

    // Display the Read Value
    LCD->print("=");
    if ( CurrPin->GetStatus() == WAIT ) { LCD->print("?"); }
    else if ( CurrPin->GetStatus() == ERR ) { LCD->print("ERR"); }
    else if ( CurrPin->IsOnOff ) {
      if ( CurrPin->GetRawValue() != 0 ) { LCD->print("On"); }
      else { LCD->print("Off"); }
    } else {
      LCD->print(CurrPin->GetModifiedValue());
    }
    
  // ELSE - Display the Pin-Control
  } else {
    switch ( CurrPin->CurrControl->ControlType ) {
      case LESS_THAN:     LCD->print(char(225));LCD->print("<"); break; // 225 = a - dots
      case GREATER_THAN:  LCD->print(char(225));LCD->print(">"); break; // 225 = a - dots
      case EQUAL_TO:      LCD->print(char(225));LCD->print("="); break; // 225 = a - dots
      case NOT_EQUAL_TO:  LCD->print(char(225));LCD->print(char(183)); break; // 183 = slashed equal
      case SET_PIN:       LCD->print("SET");LCD->print(char(126)); break; // 126 = ->
      case PID_SET:       LCD->print("PID");LCD->print(char(126)); break; // 126 = ->
    }
    // Display the Setpoint
    if ( CurrPin->IsOnOff ) {
      if ( CurrPin->CurrControl->SetPoint() != 0 ) { LCD->print("On"); }
      else { LCD->print("Off"); }
    } else {
      LCD->print(CurrPin->CurrControl->SetPoint());
    }
  }
}

/******************************************************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::loop() {

  if ( CurrPin->UpdateAvailable() ) LCD_display();                      // Check for Pin Updates
  
  if ( bIterating ) {
    if ( CurrPin->CurrControl != NULL ) {
      CurrPin->CurrControl->Save();
      CurrPin->CurrControl = NULL;    // Return To MAIN READ
    }
  }
  
  if ( bpress == NONE ) {
    ButtonCheck(analogRead(A0));
    return;
  }
  else if (bpress == SELECT) {
    DBL(("Button-SELECT"));
    if ( CurrPin->CurrControl == NULL ) { CurrPin->ReadValue(); }
    else { 
      if ( CurrPin->CurrControl->IsOn() ) { CurrPin->CurrControl->IsOn(false); }
      else { CurrPin->CurrControl->IsOn(true); }
    }
  }
  else if (bpress == UP ) {
    DBL(("Button-UP"));
    if ( CurrPin->CurrControl == NULL ) {                             // Not in a Control
      if ( CurrPin->Prev != NULL ) { CurrPin = CurrPin->Prev; }         // Set to Previous Pin
      else {
        CurrPin = FirstPin; 
        while ( CurrPin->Next != NULL ) { CurrPin = CurrPin->Next; }    // Find last Pin
      }
      CurrPin->ReadValue();
    } else { 
      if ( ButtonHeld > 5 ) { CurrPin->CurrControl->SetPointAdd(10); }
      else { CurrPin->CurrControl->SetPointAdd(1); }
    }                  // Add 1 if in a control
  }
  else if (bpress == DOWN ) {
    DBL(("Button-DOWN"));
    if ( CurrPin->CurrControl == NULL ) {
      if ( CurrPin->Next == NULL ) { CurrPin = FirstPin; }            // Restart at First-Pin
      else { CurrPin = CurrPin->Next; }                               // Else goto Next-Pin
      CurrPin->ReadValue();
    } else { 
      if ( ButtonHeld > 5 ) { CurrPin->CurrControl->SetPointAdd(-10); }
      else { CurrPin->CurrControl->SetPointAdd(-1); }
    }
  }
  else if (bpress == RIGHT) { 
    DBL(("Button-RIGHT"));
    if ( CurrPin->CurrControl == NULL ) { CurrPin->CurrControl = CurrPin->FirstControl; }
    else if ( CurrPin->CurrControl->Next != NULL ) { CurrPin->CurrControl = CurrPin->CurrControl->Next; }
  }
  else if (bpress == LEFT) { 
    DBL(("Button-LEFT"));
    if ( CurrPin->CurrControl != NULL ) {
      CurrPin->CurrControl->Save();
      if ( CurrPin->CurrControl->Prev != NULL ) { CurrPin->CurrControl = CurrPin->CurrControl->Prev; }
      else { CurrPin->CurrControl = NULL; }
    }
  }
  bpress = NONE;
  LCD_display();                           // Update Display after every button press
}

