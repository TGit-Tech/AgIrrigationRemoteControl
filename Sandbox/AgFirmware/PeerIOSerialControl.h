/************************************************************************//**
 * @file PeerIOSerialControl.h
 * @brief Arduino Peer IO-Control through Serial Port Communications.
 * @authors 
 *    tgit23        1/2017       Original
 ******************************************************************************/
#ifndef _PEERIOSERIALCONTROL_h
#define _PEERIOSERIALCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG 0
#if defined(DEBUG)
    #if DEBUG>0
        #define DBL(x) DBPort->println x
        #define DB(x) DBPort->print x
        #define DBC DBPort->print(",")
    #else
        #define DBL(x)
        #define DB(x)
        #define DBC  
    #endif
#else
    #define DBL(x)
    #define DB(x)
    #define DBC    
#endif

#define DIGITAL 1
#define ANALOG 0
#define READ 1
#define WRITE 0
#define VPIN_STATUS_ON
typedef enum PinStatus { ISOFF = 0, ISON = 1, WAIT = 2, ERR = 3, OKAY = 4 };

/**********************************************************************************************//**
 *  @class  PeerIOSerialControl
 *  @brief  Arduino Sketch for Peer I/O Control of Arduino boards through Serial.
 *   
 * - This sketch uses a Byte level protocol; thus it is NOT for Serial Monitor text control
 * 
 * Usage:
 * - Connect Serial communications between multiple Arduino's
 * - Set the 'ArduinoID' variable (Line #63) below to a unique ID for each Arduino
 * - Upload this same sketch to every Arduino connected
 * 
 * [[[ Serial Control Communication Protocol ]]]
 * <PRE>
 * Byte  |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | - Bit
 *   0   |       |  D/A  |  R/W  |  S/R  |  ID3  |  ID2  |  ID1  |  ID0  | - ArduinoID
 *   1   |       |  H/L  | PIN5  | PIN4  | PIN3  | PIN2  | PIN1  | PIN0  | - Pin Number (6D/7A-bits)
 *   2   |       |  AV6  |  AV5  |  AV4  |  AV3  |  AV2  |  AV1  |  AV0  | - Analog Value (0-6)
 *   3   |  1    | AV13  | AV12  | AV11  | AV10  |  AV9  |  AV8  |  AV7  | - Analog Value (7-13)
 * </PRE>
 * Using (4) Bytes to represent all possible scenario's as follows:
 *   - D/A         = Digital or Analog Flag
 *   - R/W         = Read or Write Flag
 *   - H/L         = High or Low Flag          ( Only used on Digital; set by D/A Flag )
 *   - ID(4-bits)  = Arduino Board Identifier  ( Range 0-15 )
 *   - S/R         = Send or Reply Flag
 *   - PIN(6-bits) = Arduino Pin Number        ( Range 0-63 Digital or 0-127 Analog  )
 *   - AV(14-bits) = Analog Value              ( Range 0-16,382 )
 *
 * - End of packet flag 'END_BIT' is bit-7 on ANYONE of the 4-bytes.
 *   Byte data is always 7-bit unless the END_BIT flags the end of the packet
 *
 * - When not using Analog Values the last two bytes are omitted ( 2-byte communications )
 *************************************************************************************************/
class PeerIOSerialControl {
public:
/***********************************************************************************//**
 * @brief Initialize a PeerIOSerialControl Object
 * @param[in] ThisArduinoID - A unique numerical identifier ( 0-15 )
 * @param[in] CommunicationPort - Arduino serial stream object for PeerIO Communications
 * @param[in] DebugPort - Arduino serial stream object for console debug messages
 * 
 * <B>Example:</B>@code{.cpp}
 *    SoftwareSerial IOSerial(rxPin,txPin);
 *    PeerIOSerialControl XBee(1,IOSerial,Serial);
 *    void setup(){
 *      IOSerial.begin(9600);
 *      Serial.begin(9600);
 *    }
 * @endcode
 **************************************************************************************/
  PeerIOSerialControl(int ThisArduinoID, Stream &CommunicationPort, Stream &DebugPort);
  
/***********************************************************************************//**
 * @brief Blocking; Preforms a digital write on the TargetArduinoID()
 * @param[in] Pin - Target Arduino's digital pin number to write to.
 * @param[in] Value - HIGH or LOW
 *
 * <B>Example:</B>@code{.cpp}
 *    XBee.digitalWriteB(3,HIGH);
 * @endcode
 **************************************************************************************/
  void digitalWriteB(uint8_t Pin, uint8_t Value);
  
  /***********************************************************************************//**
 * @brief Blocking; Preforms a digital read on the TargetArduinoID()
 * @param[in] Pin - Target Arduino's digital pin number to read from.
 * @return HIGH or LOW
 *
 * <B>Example:</B>@code{.cpp}
 *    if ( XBee.digitalReadB(3) == HIGH ) return 0;
 *  @endcode
 **************************************************************************************/
  int digitalReadB(uint8_t Pin);
  
/***********************************************************************************//**
 * @brief Blocking; Preforms an analog read on the TargetArduinoID()
 * @param[in] Pin - Target Arduino's analog pin number to read from.
 * @return ADC Value ( 0-1023 )
 * 
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.analogReadB(3);
 *    int Reply = XBee.GetReply(packetID);
 *  @endcode
 **************************************************************************************/
  int analogReadB(uint8_t Pin);
  
/***********************************************************************************//**
 * @brief Blocking; Preforms an analog write on the TargetArduinoID()
 * @param[in] Pin - Target Arduino's analog pin number to write to.
 * @param[in] Value - PWM write value ( 0-255 )
 * @return packetID
 * 
 * <B>Example:</B>@code{.cpp}
 *    XBee.analogWriteNB(3,200);
 * @endcode
 **************************************************************************************/
  void analogWriteB(uint8_t Pin, int Value);
  
/***********************************************************************************//**
 * @brief Non-Blocking; returns packetID.
 * @param[in] Pin - Target Arduino's digital pin number to write to.
 * @param[in] Value - HIGH or LOW
 * @return packetID
 * 
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.digitalWriteNB(3,LOW);
 *    int Reply = XBee.GetReply(packetID);
 * @endcode
 **************************************************************************************/
  int digitalWriteNB(uint8_t Pin, uint8_t Value);
  
/***********************************************************************************//**
 * @brief Non-Blocking; returns packetID
 * @param[in] Pin - Target Arduino's digital pin to read from.
 * @return packetID
 * 
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.digitalReadNB(3);
 *    int Reply = XBee.GetReply(packetID);
 *  @endcode
 **************************************************************************************/
  int digitalReadNB(uint8_t Pin);
  
/***********************************************************************************//**
 * @brief Non-Blocking; returns packetID
 * @param[in] Pin - Target Arduino's analog pin to read from.
 * @return packetID
 *
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.analogReadNB(3);
 *    int Reply = XBee.GetReply(packetID);
 *  @endcode
 **************************************************************************************/
  int analogReadNB(uint8_t Pin);
  
  /***********************************************************************************//**
 * @brief Non-Blocking; returns packetID
 * @param[in] Pin - Target Arduino's analog pin to write to.
 * @param[in] Value - PWM write value ( 0-255 )
 * @return packetID
 *
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.analogReadNB(3);
 *    int Reply = XBee.GetReply(packetID);
 *  @endcode
 **************************************************************************************/
  int analogWriteNB(uint8_t Pin, int Value);
  
/***********************************************************************************//**
 * @brief Sets the Target/Destination ArduinoID where sent packets will be processed.
 * @param[in] ID - The unique numeric identifier set by ThisArduinoID() on the Target Arduino. ( 0-15 )
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    XBee.TargetArduinoID(2);
 *  @endcode
 **************************************************************************************/
  void TargetArduinoID(int ID);
  
/***********************************************************************************//**
 * @brief Gets the Target/Destination ArduinoID where sent packets are processed.
 * @return Target Arduino's ID ( 0-15 )
 *
 * <B>Example:</B>@code{.cpp}
 *    if ( XBee.TargetArduinoID() == 2 ) return 0;
 *  @endcode
 **************************************************************************************/
  int TargetArduinoID();

/***********************************************************************************//**
 * @brief Gets the reply of a Non-Blocking command.
 * @param[in] packetID - Two byte packet integer returned from a NB command()
 * @return The value read, 0 for Okay on Write, or -1 for No Reply Received
 *
 * <B>Example:</B>@code{.cpp}
 *    int packetID = XBee.analogReadNB(3);
 *    int Reply = XBee.GetReply(packetID);
 *  @endcode
 **************************************************************************************/
  int GetReply(int packetID = -1);
  
/***********************************************************************************//**
 * @brief Debugging function; displays the flags, pin, arduinoID within a packet
 * @param[in] packetID - The 2-byte packetID integer returned from a NB command()
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    int packetID = Xbee.analogReadNB(3);
 *    Xbee.DecodePacket(packetID);
 *  @endcode
 **************************************************************************************/
  void DecodePacket(long lPacket = -1);
  
/***********************************************************************************//**
 * @brief Check for incoming packets.
 * @return true on the last byte of an incoming packet.
 *
 * <B>Example:</B>
 *  @code{.cpp}
 *    void loop() {
 *      XBee.Available();
 *    }
 *  @endcode
 **************************************************************************************/
  bool Available();

/***********************************************************************************//**
 * @brief Sets the timeout value (ms) for Blocking commands
 * @param[in] milliseconds - The time to spend waiting for a reply.
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    XBee.Timeout(1000); // Wait 1-Second for replies
 *  @endcode
 **************************************************************************************/
  void Timeout(int milliseconds);
  
/***********************************************************************************//**
 * @brief Gets the timeout value (ms) for Blocking commands
 * @return milliseconds
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    if ( XBee.Timeout() > 5000 ) return 0;
 *  @endcode
 **************************************************************************************/
  int Timeout();

/***********************************************************************************//**
 * @brief Sets a Value for one of the Analog Virtual Pins (i.e. 64-127)
 * @return 
 * @remarks Virtual Pins can store a Value range of ( 0 - 8191 )
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    XBee.VirtualPin(70, 15200);
 *  @endcode
 **************************************************************************************/
  void VirtualPin(int Pin, int Value, PinStatus _PinStatus = OKAY);

/***********************************************************************************//**
 * @brief Gets the Value for one of the Analog Virtual Pins (i.e. 64-127)
 * @return Value
 * @remarks Virtual Pins can store a Value range of ( 0 - 8191 )
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    if ( XBee.VirtualPin(3) > 5000 ) return 0;
 *  @endcode
 **************************************************************************************/
  int VirtualPin(int Pin);

/***********************************************************************************//**
 * @brief Gets the Value for one of the Analog Virtual Pins (i.e. 64-127)
 * @return Value
 * @remarks Virtual Pins can store a Value range of ( 0 - 8191 )
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    if ( XBee.VirtualPin(3) > 5000 ) return 0;
 *  @endcode
 **************************************************************************************/
  void VirtualPinStatus(int Pin, PinStatus _PinStatus);

/***********************************************************************************//**
 * @brief Gets the Value for one of the Analog Virtual Pins (i.e. 64-127)
 * @return Value
 * @remarks Virtual Pins can store a Value range of ( 0 - 8191 )
 * 
 * <B>Example:</B>
 *  @code{.cpp}
 *    if ( XBee.VirtualPin(3) > 5000 ) return 0;
 *  @endcode
 **************************************************************************************/
  PinStatus VirtualPinStatus(int Pin);

  
private:
  Stream *COMPort;
  Stream *DBPort;
  int ArduinoID;
  int iTargetArduinoID;
  int BlockingTimeoutMS = 1000;
 
  int Bytes[4] = { 0,0,0,0 };
  byte RBytes[10][4] = { 
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} };
  int RBI = 0;                          // Reply Bytes (RBytes) index
  int idx = 0;                          // Working 'Bytes' index 
  int iVirtualPin[63];

  int SendPacket(bool DA, bool RW, byte Pin, int Value = -1);
  void ProcessPacket();
  int ValueTo8bits(byte lByte, byte hByte);
  int ValueTo7bits(int From8BitValue);
};

#endif

