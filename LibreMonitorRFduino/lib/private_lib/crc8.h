///
/// @file		crc8.h
/// @brief		Header
/// @details	<#details#>
/// @n	
/// @n @b		Project BM019 Inventory
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
/// 
/// @author		Uwe Petersen
/// @author		Uwe Petersen
///
/// @date		02.01.16 14:10
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


#ifndef crc8_h
#define crc8_h
#endif

// LICENSES: [077915]
// -----------------------------------
// The contents of this file contains the aggregate of contributions
//   covered under one or more licences. The full text of those licenses
//   can be found in the "LICENSES" file at the top level of this project
//   identified by the MD5 fingerprints listed above.

//byte CRC8(void *data_in, byte number_of_bytes_to_read);
byte CRC8(void *data_in, uint16_t number_of_bytes_to_read); // fixed bug that made app crashed with data longer than 256 bytes
