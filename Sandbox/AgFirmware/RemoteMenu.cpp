/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O Control of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "RemoteMenu.h"

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

/******************************************************************************************************************//**
 * @brief  ISR ( Interrupt Service Routine ) for Keypad Up, Down, and Right arrow buttons.
 * @remarks
 * - PCINT1_vect Pin Change Interrupt will not trigger on Left or Select buttons ( digital threshold? )
 * - The interrupt stores the button pressed by calling ButtonCheck() and processes it when the loop() is called.
 * - The original SoftwareSerial Library calls ALL Interrupts so a modified 'SSoftwareSerial' must be used to compile
**********************************************************************************************************************/
ISR(PCINT1_vect) {
  //DB(("ISR"));
  RemoteMenu::ButtonCheck(analogRead(0));
}

/******************************************************************************************************************//**
 * @brief Constructor
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
RemoteMenu::RemoteMenu(LiquidCrystal *_LCD) {
  LCD = _LCD;
}

/******************************************************************************************************************//**
 * @brief Setup must be called in sketch setup() function
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::Setup(uint8_t ThisDevice, bool XBeeConfig = false, int RxPin = -1, int TxPin = -1 ) {
  if ( RxPin = -1 ) RxPin = 3; if ( TxPin = -1 ) TxPin = 2;   // Default Rx, Tx Pins
  PinPoint::ThisDeviceID = ThisDevice;

  if ( !XBeeConfig ) {
    noInterrupts();               // switch interrupts off while messing with their settings  
    PCICR =0x02;                  // Enable 'PCIE1' bit of PCICR Pin Change Interrupt the PCINT1 interrupt
    PCMSK1 = 0b00000001;          // Pin Change Interrupt Mask ( NA, RESET, A5, A4, A3, A2, A1, A0 ) - Activate A0              
    interrupts();                 // turn interrupts back on
  }
  
  // Setup Communications
  pinMode(RxPin, INPUT);
  pinMode(TxPin, OUTPUT);
  static SSoftwareSerial IOSerial(RxPin, TxPin);
    
  // Start Communications & Display
  IOSerial.begin(9600);
  Serial.begin(9600);
  PinPoint::XBee = new PeerIOSerialControl(ThisDevice, IOSerial, Serial);
  LCD->begin(16, 2);  
  DB(("RemoteMenu::Setup(Device="));DB((ThisDevice));DBC;
  DB(("XBeeConfig="));DB((XBeeConfig));DBC;
  DB(("RxPin="));DB((RxPin));DBC;
  DB(("TxPin="));DB((TxPin));DBL((")"));
  
  if ( XBeeConfig ) {
    LCD->clear();LCD->setCursor(0,0);
    LCD->print( "XBEE Config Mode" );
    while (1) {
      if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());
      if ( Serial.available()>0 ) IOSerial.write(Serial.read());
    }
  }
}

/******************************************************************************************************************//**
 * @brief Begin tells the Menu which PinPoint to start with
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::Begin(PinPoint *StartingPin) {
  CurrPin = StartingPin;
  CurrPin->ReadValue();delay(3);
  CurrPin->UpdateAvailable();
  LCD_display();
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

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void RemoteMenu::DeviceName( uint8_t _Device, char *_Name ) {
  DB(("RemoteMenu::DeviceName("));DB((_Device));DBC;DB((_Name));DBL((")"));
  if ( _Device > 0 && _Device < 16 ) mDeviceName[_Device] = _Name;
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
  LCD->print( mDeviceName[CurrPin->Device] );

  // Display Status ( Top Row Right )
  if ( CurrPin->CurrControl != NULL ) {
    switch ( CurrPin->CurrControl->Status ) {
      case ISOFF: LCD->setCursor(13,0);LCD->print("Off");break;
      case ISON:  LCD->setCursor(14,0);LCD->print("On");break;
      case WAIT:  LCD->setCursor(15,0);LCD->print("?");break;
      case ERR:   LCD->setCursor(13,0);LCD->print("ERR");break;
    }
  } else {
    int pos = 15;
    for ( int i=0; i<16; i++ ) {
      //DB(("OnControls["));DB((i));DB(("]="));DBL((UserControl::OnControls[i]));
      if (UserControl::OnControls[i]!=' ' && UserControl::OnControls[i]!='\0') {
        LCD->setCursor(pos--,0);
        LCD->print(UserControl::OnControls[i]);
      }
    }
  }

  //Display the Bottom Row
  LCD->setCursor(0,1);LCD->print(CurrPin->Name);LCD->print(" ");
  
  // Display the Read Value
  if ( CurrPin->CurrControl == NULL ) {
    LCD->print("=");
    if ( CurrPin->GetStatus() == WAIT ) { LCD->print("?"); }
    else if ( CurrPin->GetStatus() == ERR ) { LCD->print("ERR"); }
    else if ( CurrPin->IsOnOff ) {
      if ( CurrPin->GetRawValue() != 0 ) { LCD->print("On"); }
      else { LCD->print("Off"); }
    } else {
      LCD->print(CurrPin->GetModifiedValue());
    }
    
  // ELSE - Display the Control Setpoint
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

  const bool Block = true;
  
  PinPoint::XBee->Available();
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
    if ( CurrPin->CurrControl == NULL ) { CurrPin->ReadValue(Block); }
    else {
      if ( CurrPin->CurrControl->ControlType == SET_PIN ) {
        CurrPin->CurrControl->Apply();delay(3);
        CurrPin->CurrControl->Save();CurrPin->CurrControl = NULL;
        CurrPin->ReadValue(Block);
      } else {
        if ( CurrPin->CurrControl->IsOn() ) { CurrPin->CurrControl->IsOn(false); }
        else { CurrPin->CurrControl->IsOn(true); }
      }
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
      CurrPin->ReadValue(Block);
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

