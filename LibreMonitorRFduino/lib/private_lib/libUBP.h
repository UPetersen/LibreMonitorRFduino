///
/// @file		libUBP.h
/// @brief		Header
/// @details	<#details#>
/// @n	
/// @n @b		Project BM019 Inventory
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Uwe Petersen
/// @author		Uwe Petersen
///
/// @date		02.01.16 14:23
/// @version	<#version#>
/// 
/// @copyright	(c) Uwe Petersen, 2016
/// @copyright	<#licence#>
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


#ifndef libUBP_h
#define libUBP_h
#endif


typedef enum {
    
    UBP_TxFlagNone = 0 << 0,
    UBP_TxFlagIsRPC = 1 << 0,
    UBP_TxFlagRequiresACK = 1 << 1
    
} UBP_TxFlags;


// Public
void UBP_pump();

bool UBP_queuePacketTransmission(unsigned short packetIdentifier, UBP_TxFlags txFlags, const char *packetBytes, unsigned short byteCount);

bool UBP_isBusy();

// Private
void _UBP_pumpTxQueue();

void _UBP_ingestRxBytes(char *receivedBytes, int byteCount);

int _UBP_makeEscapedCopy(const char *inputBuffer, unsigned short inputBufferLength, char *outputBuffer, unsigned short outputBufferLength);

int _UBP_makeUnEscapedCopy(const char *inputBuffer, unsigned short inputBufferLength, char *outputBuffer);

void _UBP_hostDisconnected();


// To be implemented by end-user externally
extern void UBP_incomingChecksumFailed() __attribute__((weak));

extern void UBP_receivedPacket(unsigned short packetIdentifier, UBP_TxFlags txFlags, void *packetBuffer) __attribute__((weak));

extern void UBP_didAdvertise(bool start) __attribute__((weak));

extern void UBP_didConnect() __attribute__((weak));

extern void UBP_didDisconnect() __attribute__((weak));

