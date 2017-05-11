///
/// @file		SPIstuff.h
/// @brief		Library header
/// @details	<#details#>
/// @n	
/// @n @b		Project BM019 Inventory
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Uwe Petersen
/// @author		Uwe Petersen
///
/// @date		15.11.15 12:13
/// @version	<#version#>
/// 
/// @copyright	(c) Uwe Petersen, 2015
/// @copyright	<#license#>
///
/// @see		ReadMe.txt for references
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#   include "libpandora_types.h"
#   include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#   include "Arduino.h"
#elif defined(SPARK) // Spark specific
#   include "application.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

#ifndef SPIstuff_cpp
#define SPIstuff_cpp

//#include <SPI.h>  // the sensor communicates using SPI, so include the library:


/// @brief  Sends wake up pulse to BM019 and sets its mode to SPI or UART
/// @details  After power up BM019 requires a wake up pulse on pin 2 (IRQ/DIN)
/// @n      Depending on the value of pin 7 (SS_0) either SPI (HIGH) or UART (LOW) is set.
/// @n      The value of pin 7 (SS_0) must not be changed while running the sketch, thus
/// @n      consider to wire it either to pin 8 (VDD, 3.3V ouput and thus HIGH) or to
/// @n      wire it at all (and thus set it to LOW)
/// @n      Another 10 ms after the pulse was sent and SPI or UART set, the first valid comand will be accepted.
/// @n      These 10 ms are handled in here.
/// @param  wakeupPin pin to which the corresponding pin of BM019 is attached
void sendWakeupPulse(uint8_t wakeupPin);

/// @brief Reads the wake-up register of the CR95HF (after wake-up).
/// @param  ssPin slave select pin for SPI digital write
void readWakeUPEventRegister(int ssPIN, byte *RXBuffer);

/// @brief Prints results of read wake up register to serial console
/// @param  RXBuffer buffer in which the response is stored
void printWakeUpRegisterResponseToSerial(byte *RXBuffer);

/// @brief Sends the CH95RF into hibernate mode
/// @param  ssPin slave select pin for SPI digital write
void sendCR95HFToHibernate(int ssPIN);

/// @brief Receives the CR95HF response when waking up from hibernate mode
/// @param  ssPin slave select pin for SPI digital write
/// @param	RXBuffer buffer with response
void receiveCR95HFWakeupResponse(int ssPIN, byte *RXBuffer);
    
///// @brief	Checks if response from BM019 has an error
///// @detail Error means that either the result code (byte 0) is not 0x80 or bit 0 of the response flag (byte 2) is 1
///// @param	RXBuffer buffer with response
//bool responseHasError(byte *RXBuffer);

/// @brief	Checks if response from BM019 has no error
/// @detail No error means that the result code (byte 0) is 0x80 and bit 0 of the response flag (byte 2) is 0
/// @param	RXBuffer buffer with response
bool responseHasNoError(byte *RXBuffer);

/// @brief	Checks if IDN response from BM019 has no error
/// @detail No error means that the result code (byte 0) is 0x00. Warning: this is different to normal command response
/// @param	RXBuffer buffer with response
bool idnResponseHasNoError(byte *RXBuffer);

///// @brief	Sends command to BM019 and receives response, all via SPI
///// @param  ssPin slave select pin for SPI digital write
///// @param	RXBuffer buffer used for command and response
///// @detail Warning: Ensure that RXBuffer has appropriate size.
//void sendPollReceiveSPI(int ssPIN, byte *RXBuffer);

/// @brief	Sends command to BM019 and receives response, all via SPI
/// @param  ssPin slave select pin for SPI digital write
/// @param  command array with the command
/// @param  length of command array
/// @param	RXBuffer buffer used for command and response
/// @detail Warning: Ensure that RXBuffer has appropriate size.
void sendPollReceiveSPINew(int ssPIN, byte *command, int commandLength, byte *RXBuffer);


/// @brief	Sends command to BM019
/// @details	 contains response from BM019
/// @param  ssPin slave select pin for SPI digital write
/// @param	commandArray command, length, data
/// @param  length        length of the command data to transfer
void sendSPICommand(int ssPin, byte *commandArray, int length);

/// @brief	Polls BM019 until a response is ready
/// @details	Send 0x03 over SPI until the response byte has bit 3 set which indicates that a response is ready.
/// @n      Such a response would be for instance B'0000 1000' or B'0110 1001' or ...
/// @param  ssPin slave select pin for SPI digital write
/// @param	RXBuffer buffer used to receive data
void pollSPIUntilResponsIsReady(int ssPin, byte *RXBuffer);

/// @brief	Receives response from BM019 into buffer
/// @details	Buffer contains response from BM019.
/// @n      Byte 0 contains response code,
/// @n      byte 1 contains the length of the response (number of bytes to follow),
/// @n      bytes 2 ... length contain the data for this response
/// @param  ssPin slave select pin for SPI digital write
/// @param	RXBuffer buffer used for response
void receiveSPIResponse(int ssPin, byte *RXBuffer);

/// @brief clears the buffer by filling up with zeros
void clearBuffer(byte *RXBuffer);




#endif


