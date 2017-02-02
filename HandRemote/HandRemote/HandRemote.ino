/*********************************************************************************//**
 * @brief  Arduino Sketch firmware to be uploaded onto the Ag-Irrigation Hand-Remote 
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @todo
 *  - Need a good way to monitor multiple pumps
 * @authors 
 *    tgit23        01/2017       Original
*************************************************************************************/
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PeerIOSerialControl.h> //See https://github.com/tgit23/PeerIOSerialControl
#define DEBUG 1
#if DEBUG>0
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif

//---[ IO-PIN SETTINGS ]--------------------------------------------------------------
#define SBUZZ 2                 // Signal-Pin on Buzzer
#define SS_TX_PIN 11            // Pin 2 is used by Buzzer
#define SS_RX_PIN 12            // Pin 10 is input backlight LCD; pin 8,9 are LCD
#define BATT_PIN 1
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);          // select the pins used on the LCD panel

//---[ PROGRAM BEHAVIOR SETTINGS ]----------------------------------------------------
#define WAIT_REPLY_MS         1000      // How long to wait for a XBee reply
#define START_STATUS_ITERATE  15000     // Start iterating statuses after idle (ms)
#define ITERATE_EVERY         3000      // Iterate Status every (ms)

//---[ STATIC SETTINGS ]---------------------------------------------------------------
// Val[] Indexes
#define MAIN 0                // The current-read value of the Menu Item
#define SET 1                 // Value used to SET a MAIN-Value
#define LOALARM 2             // Value used for a Low Alarm on MAIN-Value
#define HIALARM 3             // Value used for a High Alarm on MAIN-Value
#define VALITEMS 4            // Total number of Value's per Menu Item

// Setup a Software Serial for XBEE (Allows Debug)
#include <SoftwareSerial.h>
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);   // ( rxPin, txPin )
PeerIOSerialControl XBee(1,IOSerial,Serial);    // ArduinoID, IOSerial, DebugSerial

enum Button { UP, DOWN, RIGHT, LEFT, SELECT, NONE };
enum eValueLocation { 
  PROG, LOCALPIN_DIGITAL, LOCALPIN_ANALOG, REMOTEPIN_DIGITAL, REMOTEPIN_ANALOG, EPROM };
unsigned long last_bpress = 0;                // Track last button press time
unsigned long last_change = 0;                // Track last status iteration time
int idx = 0;
int ValIdx = MAIN;

//---[ MENU STRUCTURE ]----------------------------------------------------------------
#define MENUITEMS 5                           // # of Menu Items ( 1 + index )
#define MAXOPTIONS 2                          // Maximum # of Options allowed
#define PUMPIDX 0                             // Menu-idx of Select Pump Location
#define PUMPADDR 1020                         // EEPROM address to store Selected Pump
#define STARTIDX 1                            // First Menu item displayed after reset

/*********************************************************************************//**
 * @brief  User-defined data structure for Menu Items that use a Pick-Option method
*************************************************************************************/
struct uItemOption {
  char *Text;
  int Value = LOW;
  uint8_t PumpID = 0;                         // Special place for Target AduinoID
};

/*********************************************************************************//**
 * @brief  User-defined data structure for MAIN, SET, LOALARM, HIALARM Values.
*************************************************************************************/
struct uValues {
  int Value = -1;
  int ToneHz = 1000;
  char ID = NULL;         // A set ID activates the alarm setting in the menu
  bool Active = false;    // [MAIN]Active=Valid, [SET]Active=Settable, ALARM=Is ON
};

/*********************************************************************************//**
 * @brief  User-defined data structure for every Menu Item ( Value ) on the Unit
*************************************************************************************/
struct MenuItems {
  char *Text;
  uValues Val[VALITEMS];
  eValueLocation ValueLocation = PROG;      // PROG = Internally in Program
  byte Pin = 0;                             // Arduino Pin#, Analog/Digital Set below
  uItemOption Option[MAXOPTIONS];
  byte LastOption = 0;                      // If entry doesn't have options its numeric
  bool Poll=true;
} Menu[MENUITEMS];

/*********************************************************************************//**
 * @brief  Record Menu Item Values in EEPROM non-volitale memory.
 * @remarks
 * - Arduino UNO offers 1024-bytes or (146)7-Byte Menu Item Storage
 * - Using 980-bytes or (140)7-Byte Values allowing Pump Select over byte 1000-Address
 * - Limits (4)Pumps with (35)7-Byte Values = (245)Bytes per Pump or (980)Bytes Total
 * - This function is also called on all Menu Items to set Alarm values into EEPROM.
 *  @code
 *    exmaple code
 *  @endcode
*************************************************************************************/
void EEPROMSet(int i = -1) {
  if ( i == -1 ) i = idx;DBL(("EEPROMSet()"));
  int iPumpOffset = Menu[PUMPIDX].Val[MAIN].Value * 245;
  
  if ( i == PUMPIDX ) {
    EEPROM.update(PUMPADDR,(char)Menu[i].Val[MAIN].Value);    // Write
    EEPROM.update(PUMPADDR+1,0x22);                           // Validate in EEPROM
    Menu[i].Val[MAIN].Active = true;
  
  } else {
    // Store Value in EEPROM
    if ( Menu[i].ValueLocation == EPROM ) {
      byte valLowByte = ((Menu[i].Val[MAIN].Value >> 0) & 0xFF);
      byte valHighByte = ((Menu[i].Val[MAIN].Value >> 8) & 0xFF);
      
      EEPROM.update( (i*7+0)+iPumpOffset, valLowByte);
      DB(("EEPROM.update( "));DB(((i*7+0)+iPumpOffset));
      DBC;DB((valLowByte, HEX));DBL((")"));
      
      EEPROM.update( (i*7+1)+iPumpOffset, valHighByte);
      DB(("EEPROM.update( "));DB(((i*7+1)+iPumpOffset));
      DBC;DB((valHighByte, HEX));DBL((")"));
    }
    
    // Store Alarm Values in EEPROM
    byte loAlarmLoByte = ((Menu[i].Val[LOALARM].Value >> 0) & 0xFF);
    byte loAlarmHiByte = ((Menu[i].Val[LOALARM].Value >> 8) & 0xFF);
    byte hiAlarmLoByte = ((Menu[i].Val[HIALARM].Value >> 0) & 0xFF);
    byte hiAlarmHiByte = ((Menu[i].Val[HIALARM].Value >> 8) & 0xFF);
    byte AlarmSet = 0x22;   // 2=OFF, A=ON
    if ( Menu[i].Val[LOALARM].Active ) bitSet(AlarmSet,3);
    if ( Menu[i].Val[HIALARM].Active ) bitSet(AlarmSet,7);
    
    EEPROM.update( (i*7+2)+iPumpOffset, loAlarmLoByte);
    DB(("EEPROM.update( "));DB(((i*7+2)+iPumpOffset));
    DBC;DB((loAlarmLoByte, HEX));DBL((")"));
    
    EEPROM.update( (i*7+3)+iPumpOffset, loAlarmHiByte);
    DB(("EEPROM.update( "));DB(((i*7+3)+iPumpOffset));
    DBC;DB((loAlarmHiByte, HEX));DBL((")"));
    
    EEPROM.update( (i*7+4)+iPumpOffset, hiAlarmLoByte);
    DB(("EEPROM.update( "));DB(((i*7+4)+iPumpOffset));
    DBC;DB((hiAlarmLoByte, HEX));DBL((")"));
    
    EEPROM.update( (i*7+5)+iPumpOffset, hiAlarmHiByte);
    DB(("EEPROM.update( "));DB(((i*7+5)+iPumpOffset));
    DBC;DB((hiAlarmHiByte, HEX));DBL((")"));
    
    EEPROM.update( (i*7+6)+iPumpOffset, AlarmSet);
    DB(("EEPROM.update( "));DB(((i*7+6)+iPumpOffset));
    DBC;DB((AlarmSet, HEX));DBL((")"));
  }
}

/*********************************************************************************//**
 * @brief  Read Menu Item Values from Arduino EEPROM non-volitale memory.
 * @see    EEPROMSet for Addressing notation
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void EEPROMGet(int i = -1) {
  byte StatusByte = 0;
  if ( i == -1 ) i = idx;
  
  if ( i == PUMPIDX ) {
    Menu[i].Val[MAIN].Value = (int)EEPROM.read(PUMPADDR);
    if (EEPROM.read(PUMPADDR+1) == 0x22) Menu[i].Val[MAIN].Active = true;
  } else {
    // Get Values from EEPROM
    int iPumpOffset = Menu[PUMPIDX].Val[MAIN].Value * 245;
    if ( Menu[i].ValueLocation == EPROM ) {
      byte valLowByte = EEPROM.read( ((i*7+0)+iPumpOffset) );
      byte valHighByte = EEPROM.read( ((i*7+1)+iPumpOffset) );
      Menu[i].Val[MAIN].Value = (int)((valLowByte << 0) & 0xFF) + ((valHighByte << 8) & 0xFF00);
    }

    // Get Alarm Values from EEPROM
    byte loAlarmLoByte = EEPROM.read( ((i*7+2)+iPumpOffset) );
    byte loAlarmHiByte = EEPROM.read( ((i*7+3)+iPumpOffset) );
    byte hiAlarmLoByte = EEPROM.read( ((i*7+4)+iPumpOffset) );
    byte hiAlarmHiByte = EEPROM.read( ((i*7+5)+iPumpOffset) );
    byte AlarmSet = EEPROM.read( ((i*7+6)+iPumpOffset) );
    
    Menu[i].Val[LOALARM].Value = (int)((loAlarmLoByte << 0) & 0xFF) + ((loAlarmHiByte << 8) & 0xFF00);
    Menu[i].Val[HIALARM].Value = (int)((hiAlarmLoByte << 0) & 0xFF) + ((hiAlarmHiByte << 8) & 0xFF00);
    Menu[i].Val[LOALARM].Active = bitRead(AlarmSet,3);
    Menu[i].Val[HIALARM].Active = bitRead(AlarmSet,7);

    // EPROM Values are 'Active' if the AlarmSet Bit Check passes ( Value was set/not garbage )
    if ( Menu[i].ValueLocation == EPROM ) {
      if ( (AlarmSet & 0x77) == 0x22 ) Menu[i].Val[MAIN].Active = true;
    }
  }

}

/*********************************************************************************//**
 * @brief  Preforms all needed changes to the system when changing a Selected Pump.
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void SetPump(int PumpID) {
  
  if ( PumpID != XBee.TargetArduinoID() ) {
    for (int i=0;i<MENUITEMS;i++) {
      if ( i != PUMPIDX ) {
        Menu[i].Val[MAIN].Active = false;     // De-Activate previouse Read's
        EEPROMGet(i);                         // Load Alarm Values
      }
    }
    XBee.TargetArduinoID(PumpID);
  }
  
}

/*********************************************************************************//**
 * @brief  Obtain menu values and compare them against their alarm values
 * @remarks
 *  - Battery read 458 when running off USB and 856 when running from 9VDC Battery.
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void GetItem(int i = -1) {
  if ( i == -1 ) i = idx;DBL(("GetItem()"));

  // Make sure we have a valid Pump Selected
  if ( i != PUMPIDX && !Menu[PUMPIDX].Val[MAIN].Active ) return;

  if ( Menu[i].ValueLocation == EPROM ) {
      EEPROMGet();          // Read Value from EEPROM
  } else {
    if ( Menu[i].ValueLocation == LOCALPIN_DIGITAL ) {
      Menu[i].Val[MAIN].Value = digitalRead(Menu[i].Pin);
    } else if ( Menu[i].ValueLocation == LOCALPIN_ANALOG ) {
      Menu[i].Val[MAIN].Value = analogRead(Menu[i].Pin);
    } else if ( Menu[i].ValueLocation == REMOTEPIN_DIGITAL ) {
      Menu[i].Val[MAIN].Value = XBee.digitalReadB(Menu[i].Pin);
    } else if ( Menu[i].ValueLocation == REMOTEPIN_ANALOG ) {
      Menu[i].Val[MAIN].Value = XBee.analogReadB(Menu[i].Pin);
    }
    if ( Menu[i].Val[MAIN].Value != -1 ) Menu[i].Val[MAIN].Active = true;
  }

  // Check this Value for an Active Alarm
  if ( Menu[i].Val[MAIN].Active ) {
    if ( Menu[i].Val[LOALARM].Active && Menu[i].Val[LOALARM].ID != NULL ) {
      if ( Menu[i].LastOption > 0 ) {
        if ( Menu[i].Val[MAIN].Value == Menu[i].Val[LOALARM].Value ) tone(SBUZZ,Menu[i].Val[LOALARM].ToneHz);
      } else {
        if ( Menu[i].Val[MAIN].Value < Menu[i].Val[LOALARM].Value ) tone(SBUZZ, Menu[i].Val[LOALARM].ToneHz);
      }
    }
    if ( Menu[i].Val[HIALARM].Active && Menu[i].Val[HIALARM].ID != NULL ) {
      if ( Menu[i].Val[MAIN].Value > Menu[i].Val[HIALARM].Value ) tone(SBUZZ, Menu[i].Val[HIALARM].ToneHz);
    }
  }
}

/*********************************************************************************//**
 * @brief  Preform value setting and recording when the 'Select' button is pressed.
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void SetItem(int i = -1) {
  if ( i == -1 ) i = idx;int iSetValue = 0;DBL(("SetItem()"));
  if ( i != PUMPIDX && !Menu[PUMPIDX].Val[MAIN].Active ) return;  // No control till a pump is selected

  // ALARM - If viewing an Alarm Value - Select will Toggle the Alarm ON/OFF
  if ( ValIdx == LOALARM || ValIdx == HIALARM ) {
    Menu[i].Val[ValIdx].Active = !Menu[i].Val[ValIdx].Active;
    if ( Menu[i].ValueLocation != EPROM ) EEPROMSet();

  // MAIN - If viewing a 'MAIN' Value; Select will Refresh the Value
  } else if ( ValIdx == MAIN ) {
    GetItem();

  // SET - If viewing a 'SET' Value; Select will Set the Value
  } else if ( ValIdx == SET ) {

    if ( ValIdx != SET ) return;
  
    // Obtain the Set Value
    iSetValue = Menu[i].Val[SET].Value;
    if ( Menu[i].LastOption > 0 ) {
      if (Menu[i].Val[SET].Value < 0 || Menu[i].Val[SET].Value > MAXOPTIONS) return;
      iSetValue = Menu[i].Option[Menu[i].Val[SET].Value].Value;
    }
    Menu[i].Val[MAIN].Active = false; // Remove the last Read Value; its changing

    switch ( Menu[i].ValueLocation ) {
      case EPROM:
        Menu[i].Val[MAIN].Value = Menu[i].Val[SET].Value;   // Set the Value
        EEPROMSet();                                        // Write the Value to EEPROM
        Menu[i].Val[MAIN].Active = true;                    // Validate
        if ( i == PUMPIDX ) SetPump( Menu[i].Option[ Menu[i].Val[MAIN].Value ].PumpID );
        break;

      case LOCALPIN_DIGITAL:
        Menu[i].Val[MAIN].Active = true;
        digitalWrite(Menu[i].Pin, iSetValue);
        break;

      case LOCALPIN_ANALOG:
        analogWrite(Menu[i].Pin, iSetValue);
        break;

      case REMOTEPIN_DIGITAL:
        XBee.digitalWriteNB(Menu[i].Pin, iSetValue);
        break;

      case REMOTEPIN_ANALOG:
        XBee.analogWriteNB(Menu[i].Pin, iSetValue);
        break;
    }
  }
}

/*********************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to users configuration
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void SetupMenu() {

  // Be sure to change 'MENUITEMS' when adding/deleting items
  Menu[PUMPIDX].Text = "Pump";
  Menu[PUMPIDX].ValueLocation = EPROM;
  Menu[PUMPIDX].Option[0].Text = "Canal";
  Menu[PUMPIDX].Option[0].PumpID = 10;           // Target ArduinoID
  Menu[PUMPIDX].Option[1].Text = "Ditch";
  Menu[PUMPIDX].Option[1].PumpID = 11;           // Target ArduinoID
  Menu[PUMPIDX].LastOption = 1;
  Menu[PUMPIDX].Poll=false;                     // Poll for Updates
  Menu[PUMPIDX].Val[SET].Active = true;
  
  Menu[1].Text = "Power";
  Menu[1].ValueLocation = REMOTEPIN_DIGITAL; 
  Menu[1].Option[0].Text = "Off";   
  Menu[1].Option[0].Value = LOW;  
  Menu[1].Option[1].Text = "On";
  Menu[1].Option[1].Value = HIGH;
  Menu[1].LastOption = 1;
  Menu[1].Pin = 7;
  Menu[1].Val[SET].Active = true;             // Allows Value to be 'SET'
  Menu[1].Val[LOALARM].ID = 'p';

  Menu[2].Text = "Water L.";
  Menu[2].ValueLocation = REMOTEPIN_ANALOG; 
  Menu[2].Pin = 64;
  Menu[2].Val[LOALARM].ID = 'l';
  Menu[2].Val[HIALARM].ID = 'L';
  
  Menu[3].Text = "Battery";
  Menu[3].ValueLocation = LOCALPIN_ANALOG;
  Menu[3].Pin = BATT_PIN; // Analog A2
  Menu[3].Val[LOALARM].ID = 'b';
  
  Menu[4].Text = "Pressure";
  Menu[4].ValueLocation = REMOTEPIN_ANALOG;
  Menu[4].Pin = 0;
  Menu[4].Val[LOALARM].ID='r';
  Menu[4].Val[HIALARM].ID='R';
    
  // Read all values at start-up
  GetItem(PUMPIDX);
  SetPump( Menu[PUMPIDX].Option[ Menu[PUMPIDX].Val[MAIN].Value ].PumpID );
  idx = STARTIDX;
}

/*********************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void LCD_display() {
  
  // Top Row (Status)
  lcd.clear();lcd.setCursor(0,0);
  if(Menu[PUMPIDX].Val[MAIN].Active) { 
    lcd.print(Menu[PUMPIDX].Option[Menu[PUMPIDX].Val[MAIN].Value].Text);
    lcd.setCursor(strlen(Menu[PUMPIDX].Option[Menu[PUMPIDX].Val[MAIN].Value].Text)+1,0);lcd.print("Pump");
  } else {
    lcd.print("ERR Pump");
  }
  
  // Alarm Statuses
  int pos = 15;
  for (int i=0;i<MENUITEMS;i++) {
    if ( Menu[i].Val[LOALARM].Active && Menu[i].Val[LOALARM].ID != NULL ) {
      lcd.setCursor(pos,0);lcd.print(Menu[i].Val[LOALARM].ID);pos--;
    }
    if ( Menu[i].Val[HIALARM].Active && Menu[i].Val[HIALARM].ID != NULL ) {
      lcd.setCursor(pos,0);lcd.print(Menu[i].Val[HIALARM].ID);pos--;
    }
  }
  
  // Menu Item Text on Bottom Row
  lcd.setCursor(0,1);lcd.print(Menu[idx].Text);
  lcd.setCursor(strlen(Menu[idx].Text),1);
  
  if ( ValIdx == MAIN ) {
    lcd.print(" =");
    lcd.setCursor(strlen(Menu[idx].Text)+2,1);
  } else if ( ValIdx == SET ) {
    lcd.print(" Set");
    lcd.print((char)126); //Character '->'
    lcd.setCursor(strlen(Menu[idx].Text)+5,1);
  } else {
    if ( Menu[idx].LastOption > 0 ) {
      lcd.print(" !=");
    } else {
      if ( ValIdx == LOALARM) lcd.print(" !<");
      if ( ValIdx == HIALARM) lcd.print(" !>");
    }
    lcd.setCursor(strlen(Menu[idx].Text)+3,1);
  }
  
  // Value
  if ( ValIdx == MAIN && !Menu[idx].Val[MAIN].Active ) {
    lcd.print("ERR");
  } else {
    if ( Menu[idx].LastOption > 0 ) {
      int OptionIdx = Menu[idx].Val[ValIdx].Value;
      if( OptionIdx < 0 || OptionIdx > Menu[idx].LastOption ) {
        lcd.print("ERR-");lcd.print(OptionIdx);
      } else {
        lcd.print(Menu[idx].Option[OptionIdx].Text); 
      }
    } else {
        lcd.print(Menu[idx].Val[ValIdx].Value);
    }
  }
  delay(200);
}


/*********************************************************************************//**
 * @brief  Arduino Sketch Setup routine - Initialize the environment.
 * @remarks
 * - pin#10 INPUT Backlit shorting see http://forum.arduino.cc/index.php?topic=96747.0
 * @code
 *   exmaple code
 * @endcode
*************************************************************************************/
void setup(){
    pinMode(10,INPUT);            // Fix for Q1-LCD Backlit shorting issue
    pinMode(SBUZZ,OUTPUT);
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    
    IOSerial.begin(9600);
    Serial.begin(9600);           // Start UART Communications with the XBee->Module
    lcd.begin(16, 2);             // Start the LCD library
    SetupMenu();
    XBee.Timeout(WAIT_REPLY_MS);  // Set the Timeout
    LCD_display();                // Display the main menu
}

/*********************************************************************************//**
 * @brief  Arduino Sketch Loop() routine
 * @remarks
 * - This function is called automatically over-and-over by Arduino
 * - Handles incoming XBee communications
 * - Handles button presses and LCD response updates
 * - Handles Menu iteratation during idle.
*************************************************************************************/
void loop(){

  // Check for a button press
  Button bpress = NONE;
  int adc_key_in = analogRead(0);
  if (adc_key_in > 1000) { bpress = NONE; }
  else if (adc_key_in < 50) { bpress = RIGHT; }
  else if (adc_key_in < 195) { bpress = UP; }
  else if (adc_key_in < 380) { bpress = DOWN; }
  else if (adc_key_in < 555) { bpress = LEFT; }
  else if (adc_key_in < 790) { bpress = SELECT; }

  // Check for incoming communications
  XBee.Available();

  // Preform duties
  if(bpress == NONE) {
    if ( millis() - last_bpress > START_STATUS_ITERATE && 
         millis() - last_change > ITERATE_EVERY ) {
          
          ValIdx = MAIN;
          int i = idx;                  // Mark current idx
          do {
            i++;if(i>MENUITEMS-1) i=0;  // reset idx to beginning
            if(i == idx) break;         // break if made a full rotation
          } while (!Menu[i].Poll);      // stop if item is Poll
          idx=i;                        // set new 'idx' to next with a Poll
          GetItem();
          last_change=millis();
          LCD_display();
    }
  } else { last_bpress = millis(); }
  
  if(bpress !=NONE) {
    noTone(SBUZZ);

      if (bpress == SELECT) {
        SetItem();
      
      } else if (bpress == UP) {
        ValIdx = MAIN;
        idx--;if(idx<0) idx=MENUITEMS-1;
        if ( !Menu[idx].Val[MAIN].Active ) GetItem();
        
      } else if (bpress == DOWN) {
        ValIdx++;
        if ( ValIdx == SET && !Menu[idx].Val[SET].Active ) ValIdx++;      // Skip SET if not Active
        if ( ValIdx == LOALARM && Menu[idx].Val[LOALARM].ID == NULL ) ValIdx++;
        if ( ValIdx == HIALARM && Menu[idx].Val[HIALARM].ID == NULL ) ValIdx++;    // Skip HIALARM if Option
        if ( ValIdx > VALITEMS-1 ) {
          ValIdx = 0;
          idx++;if(idx>MENUITEMS-1) idx=0;
        }
        if ( ValIdx == MAIN && !Menu[idx].Val[MAIN].Active ) GetItem();
      }    

      // Change the Value
      if ( ValIdx != MAIN ) {
        if (bpress == RIGHT) {
          Menu[idx].Val[ValIdx].Value++;
          if ( Menu[idx].LastOption>0 && Menu[idx].Val[ValIdx].Value > Menu[idx].LastOption) Menu[idx].Val[ValIdx].Value = 0;
        
        } else if(bpress == LEFT) {
          if ( Menu[idx].Val[ValIdx].Value > 0 ) Menu[idx].Val[ValIdx].Value--;
          if ( Menu[idx].LastOption>0 && Menu[idx].Val[ValIdx].Value > Menu[idx].LastOption) Menu[idx].Val[ValIdx].Value = Menu[idx].LastOption;
        }
      }

    // Update Display
    LCD_display();
  }
}
