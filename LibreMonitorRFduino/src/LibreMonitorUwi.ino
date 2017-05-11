#include <Arduino.h>

///
///   LibreMonitor
///
///   Copyright (c) 2015 Uwe Petersen, all right reserved
///
///   Wiring connections:
///
///      BM019            RFduino/Simblee
///      DIN:  pin 2      IRQ:  GPIO pin 2
///      SS:   pin 3      SS:   GPIO pin 3
///      MISO: pin 4      MISO: GPIO pin 4
///      MOSI: pin 5      MOSI: GPIO pin 5
///      SCK:  pin 6      SCK:  GPIO pin 6
///      SS0:  pin 7      +3V-pin
///      GND:  pin 10     GND-pin
///
///      BM019            BM019
///      SS0:  pin 7       VDD:  pin 8 (Output 3,3V from BM019)
///
///      BM019            Lipo Power source (I use a 100mAh lip which lasts a full day
///      VIN:  pin 9      "+" of lipo / lipo charger
///      GND:  pin 10     GND/"-" of lipo / lipo charger
///
///      You can place a switch between GND of lipo/lipo-charger and GND of the BM019
///
///      SPI-WIRING
///      If wired as suggested above, then you have to change the Simblee SPI pins for SS, MOSI, MISO and SCK.
///      This is done in the variant.h file. In my case this file is located (dependent on the Simblee/RFduino
///      version) in
///
///         /Users/[my user name]/Library/Arduino15/packages/Simblee/hardware/Simblee/1.0.0/variants/Simblee or
///         /Users/[my user name]/Library/Arduino15/packages/RFduino/hardware/RFduino/2.3.3/variants
///
///      In this file set the defines for the SPI pins as follows:
///
///         #define PIN_SPI_SS           (3u)
///         #define PIN_SPI_MOSI         (5u)
///         #define PIN_SPI_MISO         (4u)
///         #define PIN_SPI_SCK          (6u)
///
///
///   Acknowledgements:
///
///      RFduinoUBP
///
///         This code uses portions of RFduinoUB, which can be retrieved from
///              https://github.com/cconway/RFduinoUBP under the following license:
///         The MIT License (MIT)
///         Copyright (c) 2015 cconway
///         Permission is hereby granted, free of charge, to any person obtaining a copy
///         of this software and associated documentation files (the "Software"), to deal
///         in the Software without restriction, including without limitation the rights
///         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
///         copies of the Software, and to permit persons to whom the Software is
///         furnished to do so, subject to the following conditions:
///         The above copyright notice and this permission notice shall be included in all
///         copies or substantial portions of the Software.
///         THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
///         IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
///         FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
///         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
///         LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
///         OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
///         SOFTWARE.
///
///      Solutions Cubed LLC
///
///         The author is grateful to Solutions Cubed LLC, see http://www.solutions-cubed.com,
///         for providing helpful code samples for their BM019 nfc module, portions of which are
///         used within this code.
///
///
///   Choose between Simblee or RFduino hardware
///
///   If you want to switch between RFduino and Simblee or vice versa, be aware of the following tasks:
///     1. Follow the instructions in the simblee quick start guide (https://www.simblee.com/Simblee_Quickstart_Guide_v1.1.0.pdf)
///        or the RFduino dcoumentation (https://github.com/RFduino/RFduino/blob/master/README.md) on how to install the
///        arduino libraries needed.
///     2. Set your additional board manager in the Arduino IDE preferences to either Simblee or RFduino.
///        With the Arduino IDE open, click on Arduino > Preferences
///        a) For Simblee: Copy and Paste the following link into Additional Boards Manager
///           URLs: https://www.simblee.com/package_simblee166_index.json then press “OK”
///        b) For RFduino: Add http://rfduino.com/package_rfduino166_index.json to Additional Board Manager URLs and save.
///     3. Go to Tools > Board: > Boards Manager... and install the corresponding board.
///     4. Go to Go to Tools > Board: and choose the Simblee or RFduino board.
///     5. In the arduino IDE goto sketch and incorporate the RFduinoBLE or the SimbleBLE library respectively
///     6. If you had to install the Simblee or RFduino package be sure to reconfigure the SPI wiring in the
///        variants.h file, as explained in the SPI-WIRING section in the above comments on wiring connections
///     7. Set the following #define to SIMBLEE for using a Simblee or comment it out to use a RFduino
///     8. Do the same in the libUPB.cpp in the corresponding library for this project.
///
// Choose between Simblee or RFduino
//#define SIMBLEE   // IF a Simblee is used, you have to define SIMBLEE, otherwise it is assumed you use a RFduino.

// Include application, user and local libraries
#ifdef SIMBLEE
#include <SimbleeBLE.h>
#include <SimbleeForMobile.h>
#else
#include <RFduinoBLE.h>
#endif

#include <SPI.h>               // the sensor communicates using SPI, so include the library
// This code uses the SLIP protocol to transer the data via bluetooth, see https://github.com/cconway/RFduinoUBP
#include <constants.h>
#include <crc8.h>
#include <data_types.h>
#include <libUBP.h>
#include <SPIstuff.h>


// Define variables and constants

/* CR95HF Commands */
#define IDN               0x01  // identification number of CR95HF
#define SELECT_PROTOCOL   0x02  // select protocol
#define POLL              0x03  // poll
#define SENDRECEIVE       0x04  // send and receive data (most commonly used)
#define READ              0x05  // read values from registers internal to CR95HF
#define WRITE             0x06  // write values to registers internal to CR95HF
#define ECHO              0x55

// send receive commands for ISO/IEC 15693 protocol
#define INVENTORY               0x01  // receives information about tags in range
#define STAY_QUIET              0x02  // selected unit will not send back a response
#define READ_BLOCK              0x20  // read single block of memory from RF tag
#define WRITE_BLOCK             0x21  // write single block to memory of RF tag
#define LOCK_BLOCK              0x22  // permanently locks a block of memory on RF tag
#define READ_BLOCKS             0x23  // reads multiple blocks of memory from RF tag
#define WRITE_BLOCKS            0x24  // writes multiple blocks of memory to RF tag
#define SELECT                  0x25  // used to select a specific tag for communication via the uid
#define RESET_TO_READY          0x26  // resets RF tag to ready state
#define WRITE_AFI               0x27  // writes application family identifier to RF tag
#define LOCK_AFI                0x28  // permanently locks application family identifier
#define WRITE_DSFID             0x29  // writes data storage format identifier to RF tag
#define LOCK_DSFID              0x2A  // permanentlylocks data storage format identifier
#define GET_SYSTEM_INFORMATION  0x2B  // gets information from RF tag that includes memory
// block size in bytes and number of memory blocks
#define GET_BLOCKS_SECURITY_STATUS  0x2C

// Request flag with bits set these rules:
//     Bit  Flag name           Description
//      1   Sub-carrier flag    0 ... single sub-carrier used
//                              1 ... two sub-carriers used
//      2   Data rate flag      0 ... low data rate is used
//                              1 ... high data rate is used
//      3   Inventory flag      State determines how bits 5-8 are defined
//                              0 ... flag not set
//                              1 ... flag set
//      4   Protocol            0: no protocol extension
//          extension flag      1: protocol format is extended
//   If Inventory flag = 0 (not set):
//      5   Select flag         0: request shall be executed based on setting of the address flag
//                              1: request shall be executed only by devices in the selected state
//      6   Address flag        0: request is not addressed
//                              1: request is addressed, optional UID field is present
//      7   Option flag         Meaning is defined by the command description, if not used set to 0
//      8   reserved
//   If Inventory flag = 1 (set):
//      5   AFI flag            0: AFI field is not present
//                              1: AFI field is present
//      6   Number of           0: 16 slots
//          slots flag          1: 1 slot
//      7   Option flag         Meaning is defined by the command description, if not used set to 0
//      8   reserved
//
//  Helper bit field by to quickly calculate integers from hex and vice versa:
//      Bit no.:     8   7   6   5      4   3   2   1
//      value:     128  64  32  16      8   4   2   1
//      choose:      0   0   0   0      4   0   2   1   result is 0x03
//
// request flags byte, 0x26 means:
//   single sub carrier, high data rate, inventory flag set, no protocoll extentsion, AFI not present, one slot)


// Settings

 //#define DEBUG

//const int SS_PIN = 3;   // Slave Select pin, LibreMonitor wiring changed to new value on 2016-03-21
//const int SS_PIN = 6;   // Slave Select pin, wiring for Marek Macner's PCB
const int SS_PIN = SS;   // Slave Select pin, uses standard wiring from variant.h (see comment section above)

const int IRQ_PIN = 2;  // IRQ/DIN pin used for wake-up pulse
byte RXBuffer[400];     // receive buffer
byte dataBuffer[400];   // buffer for Freestyle Libre byte data

Sensor sensor;          // struct for sensor data


byte NFCReady = 0;      // used to track NFC state
byte oldNFCReady = 0;
bool nfcStateTransmissionSuccess = false;

const int SPI_FREQUENCY = 2000; // max. for CR95HF is 2000 = 2 MHz
//const uint64_t SLEEP_DURATION = 20000;  // Duration of Simblee ultra low power mode in ms
const uint64_t SLEEP_DURATION = 120000;  // Duration of Simblee ultra low power mode in ms


//===================================================================================================
//   setup routines
//===================================================================================================

// setup pins / wiring for Simblee / RFduino
void setupPins() {
    Serial.println("Setting pins for Simblee/RFduino ...");
    pinMode(IRQ_PIN, OUTPUT);
    pinMode(SS_PIN, OUTPUT);
    // Commented out by Uwi on 15.11.2015: was not needed obviously
    //    digitalWrite(SS_PIN, HIGH);
    #ifdef DEBUG
    Serial.println("... done setting pins for Simblee/RFduino.");
    #endif
}

// setup SPI communication
void setupSPI() {
    #ifdef DEBUG
    Serial.println("Setting up SPI ...");
    #endif
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(SPI_FREQUENCY);
    #ifdef DEBUG
    Serial.println("... done setting up SPI.");
    #endif
}

// setup bluetooth stack and start communication for Simblee / RFduino
void setupBluetoothConnection() {
    #ifdef DEBUG
    Serial.println("Setup Bluetooth stack and start connection...");
    #endif
    #ifdef SIMBLEE
    SimbleeBLE.deviceName = "LibreCGM";
    SimbleeBLE.customUUID = "2220";
    SimbleeBLE.advertisementData = "data";
    SimbleeBLE.advertisementInterval = MILLISECONDS(500);  // Default was 300
    SimbleeBLE.txPowerLevel = 4;  // Possible values: -20, -16, -12, -8, -4, 0 or +4 (dbM)  // up to 20016-05-17 this was 0
    SimbleeBLE.begin();           // Start the BLE stack
    #else
    RFduinoBLE.deviceName = "LibreCGM";
    //  RFduinoBLE.customUUID = "2220";
    RFduinoBLE.advertisementData = "data";
    RFduinoBLE.advertisementInterval = MILLISECONDS(500);  // Default was 300
    RFduinoBLE.txPowerLevel = 4;  // Possible values: -20, -16, -12, -8, -4, 0 or +4 (dbM)  // up to 20016-05-17 this was 0
    RFduinoBLE.begin();           // Start the BLE stack
    #endif
    #ifdef DEBUG
    Serial.println("... done seting up Bluetooth stack and started connection.");
    #endif
}

// init routine for selective nfc reading
void initSensor(Sensor *sensor){
    (*sensor).nextTrend = 0;
    (*sensor).oldNextTrend = 0;
    (*sensor).nextHistory = 0;
    (*sensor).oldNextHistory = 0;
    (*sensor).minutesSinceStart = 0;
    (*sensor).oldMinutesSinceStart = 0;
    for (uint i = 0; i < sizeof((*sensor).uid); i++) {
        (*sensor).uid[i] = 0;
        (*sensor).oldUid[i] = 0;
    }
    for (uint i = 0; i < sizeof((*sensor).fram); i++) {
        (*sensor).fram[i] = 0;
    }
    for (uint i = 0; i < sizeof((*sensor).resultCodes); i++) {
        (*sensor).resultCodes[i] = 0;
    }
}

//===================================================================================================
//   general routines
//===================================================================================================

// SetProtocol_Command programs the CR95HF for ISO/IEC 15693 operation.
// If the correct response is received the serial monitor is used to display successful programming.
// Warning: if the parameters of the protocol are changed, e.g. sub carrier, then the request flags
// of the other commands (e.g. inventory, read single block) have to be changed accordingly.
void SetProtocol_Command() {

    // TODO: change this to use sendPollReceiveSPINew as below
    // step 1 send the command
    digitalWrite(SS_PIN, LOW);
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    SPI.transfer(0x02);  // Set protocol command
    SPI.transfer(0x02);  // length of data to follow
    SPI.transfer(0x01);  // code for ISO/IEC 15693
    SPI.transfer(0x0F);  // Up till 2016-06-13: crc16, single, 30%, wait for SOF (wrong: Wait for SOF, 100% modulation, append CRC)
    //    SPI.transfer(0x0D);  // Up till 2016-06-13: crc16, single, 30%, wait for SOF (wrong: Wait for SOF, 100% modulation, append CRC)
    digitalWrite(SS_PIN, HIGH);
    delay(1);

    // step 2, poll for data ready
    pollSPIUntilResponsIsReady(SS_PIN, RXBuffer);

    // step 3, read the data
    receiveSPIResponse(SS_PIN, RXBuffer);
    #ifdef DEBUG
    Serial.println("RXBuffer is");
    for (byte i = 0; i < 2; i++) {
        Serial.print(RXBuffer[i], HEX);
        Serial.print(" ");
    }
    #endif
    if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0)) {
        Serial.println("PROTOCOL SET-");  //
        NFCReady = 1; // NFC is ready
    } else {
        Serial.println("BAD RESPONSE TO SET PROTOCOL");
        NFCReady = 0; // NFC not ready
    }
    Serial.println(" ");
}


// Reads a (single) block of 8 bytes at the position blockNum from the FreeStyle Libre FRAM and copies
// the content to the corresponding position of the dataBuffer.
// If an error occurs (result code != 128), the reading process is repeated up to maxTrials times.
// The final result code is returned.
// blockNum    number of the block of the FreeStyle Libre FRAM to be read, possible values are 0 to 243.
// maxTrials   maximum number of reading attempts.
// RXBuffer    buffer used for sending and receiving data.
// fram        buffer of bytes the final block of data is copied to at the position corresponding to blockNum.
//             Example 1: If blockNumber is 0, the 8 bytes of data are copied into fram[0] to fram[7] (i.e. 0*8 to 0*8+7)
//             Example 2: If blockNumber is 17, the 8 bytes of data are copied into fram[136] to fram[143] (i.e. 17*8 = 136.. 17*8+7 = 143)
byte readSingleBlock(byte blockNum, int maxTrials, byte *RXBuffer, byte *fram, int ssPIN) {

    byte command[5];
    command[ 0] = SENDRECEIVE;   // command code for send receive CR95HF command
    command[ 1] = 0x03;          // length of data that follows (3 bytes)
    // command[ 2] = 0x02;          // request Flags byte, single carrier, high data rate
    command[ 2] = 0x03;          // request Flags byte dual carrier, high data rate
    command[ 3] = 0x20;          // read single block Command for ISO/IEC 15693
    command[ 4] = blockNum;      // Block number

    int trials = 0;
    do {
        sendPollReceiveSPINew(ssPIN, command, sizeof(command), RXBuffer);
        trials++;
        delay(1);

        #ifdef DEBUG
        // Print to Serial for debugging
        if (RXBuffer[0] == 0x80)  {
            Serial.printf("readSingleBlock: block #%d:", blockNum);
            for (byte i = 3; i < RXBuffer[1] + 3 - 4; i++) {
                Serial.print(RXBuffer[i], HEX);
                Serial.print(" ");
            }
        } else {
            Serial.printf("readSingleBlock: block #%d not available, response code is: ", blockNum);
            Serial.println(RXBuffer[0], HEX);
        }
        Serial.println(" ");
        #endif

    } while (RXBuffer[0] != 0x80 && trials <= maxTrials); // repeat until result code == 0x80 (no error), but only maxTrials times

    if (RXBuffer[0] == 0x80) {
        for (byte i = 0; i < 8; i++) {
            fram[blockNum*8+i] = RXBuffer[i+3];  // succes case: copy received data to fram
        }
    } else {
        for (byte i = 0; i < 8; i++) {
            fram[blockNum*8+i] = 0;  // error case: fill fram with zeros
        }
    }
    return RXBuffer[0];  // result code of command response
}




// Reads the single block of 8 bytes with number blockNum from the BM019.
// The BM019 has 244 Blocks of 8 bytes that can be read via ISO 15693 commands. The blocks are numbered 0 to 243.
// Returns the result code of the command response (that indicates wether there was success or an error)
byte ReadSingleBlockReturn(int blockNum) {

    RXBuffer[0] = SENDRECEIVE;   // command code for send receive CR95HF command
    RXBuffer[1] = 0x03;          // length of data that follows (3 bytes)
    //    RXBuffer[2] = 0x02;          // request Flags byte, single carrier, high data rate
    RXBuffer[2] = 0x03;        // request Flags byte dual carrier, high data rate
    RXBuffer[3] = 0x20;          // read single block Command for ISO/IEC 15693
    RXBuffer[4] = blockNum;      // Block number

    // step 1 send the command
    sendSPICommand(SS_PIN, RXBuffer, 5);

    // step 2, poll for data ready
    pollSPIUntilResponsIsReady(SS_PIN, RXBuffer);

    // step 3, read the data
    receiveSPIResponse(SS_PIN, RXBuffer);

    delay(1);
    #ifdef DEBUG
    // Print to Serial
    if (RXBuffer[0] == 128)  {
        Serial.printf("Read single block #%d:", blockNum);
        for (byte i = 3; i < RXBuffer[1] + 3 - 4; i++) {
            Serial.print(RXBuffer[i], HEX);
            Serial.print(" ");
        }
    } else {
        Serial.print("Error reading single block, response code is ");
        Serial.println(RXBuffer[0], HEX);
    }
    Serial.println(" ");
    #endif

    return RXBuffer[0];  // result code of command response
}



/// @brief  Sends command to BM019 and receives response serveral times until response with no error, max maxTrial times.
/// @detail After maxTrials trials the last response is returned, even  if there is still an error.
/// @detail Warning: Ensure that RXBuffer has appropriate size.
/// @param  ssPin slave select pin for SPI digital write
/// @param  RXBuffer buffer used for command and response
/// @param  maxTrials max number of trials
void runSPICommandUntilNoError(int ssPin, byte *command, int length, byte *RXBuffer, int maxTrials) {

    int count = 0;
    bool success;
    do {
        delay(1);
        #ifdef DEBUG
        Serial.printf("Before: Count: %d, success: %b, RXBuffer[0]: %x \r\n", count, success, RXBuffer[0]);
        #endif
        count++;

        // clear RXBuffer with zeros
        memset(RXBuffer, 0, sizeof(RXBuffer));

        // run SPI command
        sendPollReceiveSPINew(ssPin, command, sizeof(command), RXBuffer);
        success = responseHasNoError(RXBuffer);

        #ifdef DEBUG
        Serial.printf("After: Count: %d, success: %b, RXBuffer[0]: %x \r\n", count, success, RXBuffer[0]);
        #endif

    } while ( !success && (count < maxTrials));
    delay(1);
    #ifdef DEBUG
    Serial.printf("Exiting at count: %d, RXBuffer[0]: %x \r\n", count, RXBuffer[0]);
    #endif
}

/// @brief  Sends IDN command to BM019 and receives response serveral times until response with no error, max maxTrial times.
/// @detail After maxTrials trials the last response is returned, even  if there is still an error.
/// @detail Warning: Ensure that RXBuffer has appropriate size.
/// @param  ssPin slave select pin for SPI digital write
/// @param  RXBuffer buffer used for command and response
/// @param  maxTrials max number of trials
void runIDNCommandUntilNoError(int ssPin, byte *command, int length, byte *RXBuffer, int maxTrials) {

    int count = 0;
    bool success;
    do {

        #ifdef DEBUG
        Serial.printf("Before: Count: %d, success: %b, RXBuffer[0]: %x \r\n", count, success, RXBuffer[0]);
        #endif
        count++;

        // clear RXBuffer with zeros
        memset(RXBuffer, 0, sizeof(RXBuffer));

        // run SPI command
        sendPollReceiveSPINew(ssPin, command, sizeof(command), RXBuffer);
        success = idnResponseHasNoError(RXBuffer);

        #ifdef DEBUG
        Serial.printf("After: Count: %d, success: %b, RXBuffer[0]: %x \r\n", count, success, RXBuffer[0]);
        #endif
    } while ( !success && (count < maxTrials));
    delay(10);
    #ifdef DEBUG
    Serial.printf("Exiting at count: %d, RXBuffer[0]: %x \r\n", count, RXBuffer[0]);
    #endif
}


/// Runs the system information command several times, i.e. maxTrials times or until a response with no error is gotten, whatever happens first.
/// @param ssPin SS_PIN used for SPI
/// @param RXBuffer buffer used for send and receive
/// @param maxTrials maximum number of trials. If reached, the result is returned, even if there still is an error present
/// @brief  Get system information from BM019
/// @details  The response is read into RXBuffer and that's it, no matter if there is an error or not.
/// @n Command format for CR95HF:
/// @n 8 bits for request flags,
/// @n 8 bits for the get system information command,i.e 0x2B,
/// @n 64 bits for an optional UID (not used in non-addressed modes).
/// @details Example get system info command (CR95HF command embedded in BM019 command):
/// @details 0x04 ... BM019 send/receive command code
/// @details 0x02 ... BM019 length of CR95HF command data that follows
/// @details 0x02 ... CR95HF request flags byte (high data rate used)
/// @details 0x2B ... CR95HF get system information command
/// @n
/// @n
/// @details Example CR95HF response with no error:
/// @details 0x80      ... result code
/// @details 0x12      ... length of following data (12 bytes)
/// @details 0x00      ... response flags
/// @details 0x0F      ... info flags
/// @details 0x69 0x55 0x19 0x38 0x42 0x20 0x02 0xE0 ... data: UID
/// @details 0x00      ... DSFID (supported and field is present in response if bit 1 of info flags is set)
/// @details 0x00      ... AFI (supported and field is present in response if bit 2 of info flags is set)
/// @details 0x3F 0X03 ... memory size (supported and field is present in response if bit 3 of info flags is set)
/// @details 0x20      ... IC Ref (supported and field is present in response if bit 4 of info flags is set)
/// @details 0xB4 0xA9 ... CRC16
/// @details 0x00      ... error
/// @n
/// @details Example CR95HF response with error (bit 0 of response flag is set)
/// @details 0x01      ... response flags
/// @details 0x01      ... error code (returned when error bit is set)
/// @details 0xB4 0xA9 ... CRC16
/// @details 0X01      ... (error CRC16 or collision error bits)
void runSystemInformationCommandUntilNoError(int ssPin, byte *RXBuffer, int maxTrials) {
    #ifdef DEBUG
    Serial.printf("ssPin: %d, maxTrials: %d, RXBuffer[0]: %x \r\n", ssPin, maxTrials, RXBuffer[0]);
    #endif
    byte command[4];
    command[0] = 0x04;   // command code for send receive CR95HF command
    command[1] = 0x02;   // length of data that follows (3 bytes)
    command[2] = 0x03;   // request Flags byte, dual sub carrier
    //    command[2] = 0x02;   // request Flags byte, single sub carrier
    command[3] = 0x2B;   // get system information command for ISO/IEC 15693
    delay(10);
    #ifdef DEBUG
    Serial.printf("ssPin: %d, maxTrials: %d, RXBuffer[0]: %x \r\n", ssPin, maxTrials, RXBuffer[0]);
    #endif
    // run command until no error, but only max 10 times
    runSPICommandUntilNoError(ssPin, command, sizeof(command), RXBuffer, maxTrials);
}

/// Runs the inventory  command several times, i.e. maxTrials times or until a response with no error is gotten, whatever happens first.
/// @param ssPin SS_PIN used for SPI
/// @param RXBuffer buffer used for send and receive
/// @param maxTrials maximum number of trials. If reached, the result is returned, even if there still is an error present
void runIDNCommand(int ssPin, byte *RXBuffer, int maxTrials) {

    #ifdef DEBUG
    Serial.printf("ssPin: %d, maxTrials: %d, RXBuffer[0]: %x \r\n", ssPin, maxTrials, RXBuffer[0]);
    #endif

    byte command[2];
    command[0] = 0x01;   // command code for send receive CR95HF command
    command[1] = 0x00;   // length of data that follows (0 bytes)
    delay(10);

    #ifdef DEBUG
    Serial.printf("ssPin: %d, maxTrials: %d, RXBuffer[0]: %x \r\n", ssPin, maxTrials, RXBuffer[0]);
    #endif
    // run command until no error, but only max 10 times
    runIDNCommandUntilNoError(ssPin, command, sizeof(command), RXBuffer, maxTrials);
}


//Example response of CR95HF for inventory command and data positions/indices
//+------+--------+----------------------------------------------------------------------+
//|Result| Length | Data                                                                 |
//| code |        +--------+-----+---------------------------------------+---------+-----+
//|      |        |Response|     |                                       |         |     |
//|      |        | flags  |DSFID| UID                                   |CRC16    |Error|
//+------+--------+--------+---------------------------------------------+---------+-----+
//| 0x80 |0x0D(13)| 0x00   |0x00 |0x51 0x69 0x19 0x38 0x42 0x20 0x02 0xE0|0x84 0x28|0x00 |
//+------+--------+--------+-----+---------------------------------------+---------+-----+
//|   0  |  1     |   2    |  3  | 4    5    6    7    8    9    10   11 | 12   13 | 14  |
//+------+--------+--------+-----+---------------------------------------+---------+-----+
//                    0       1    2    3    4    5    6    7     8    9   10   11   12
//

/// Retreive idn data from RXBuffer from IDN command
/// This ist the struct later to be transmitted via bluetooth
/// @detail There is no error possible in the response since this is just pure device information and thus not rely on RFID and a tag in the field
/// @param RXBuffer buffer containing the response from the IDN command
/// @param idnType struct with retreived data
IDNDataType idnDataFromIDNResponse(byte *RXBuffer) {

    IDNDataType idnData;
    idnData.resultCode = RXBuffer[0];

    // Device ID has 13 bytes
    for (int i = 0; i < 13; i++) {
        idnData.deviceID[i] = RXBuffer[i + 2];
    }

    // ROM CRC are the last two bytes of the data
    int length = RXBuffer[2];
    idnData.romCRC[0] = RXBuffer[length - 2];
    idnData.romCRC[1] = RXBuffer[length - 1];

    return idnData;
}

/// Retreive system information from RXBuffer from system information command
/// This ist the struct later to be transmitted via bluetooth
/// @param RXBuffer buffer containing the response from the system information command
/// @param SystemInformationType struct with retreived data
SystemInformationDataType systemInformationDataFromGetSystemInformationResponse(byte *RXBuffer) {

    SystemInformationDataType systemInformationData;

    systemInformationData.resultCode = RXBuffer[0];
    systemInformationData.responseFlags = RXBuffer[2];

    // check for no error in result code and handle accordingly
    if (systemInformationData.resultCode == 0x80) { // no error in result code

        // check for no error in response flags
        if ((systemInformationData.responseFlags & 0x01) == 0) {
            // no error in response flags
            systemInformationData.infoFlags = RXBuffer[3];
            for (int i = 0; i < 8; i++) {
                systemInformationData.uid[i] = RXBuffer[11 - i];
            }
            systemInformationData.errorCode = RXBuffer[RXBuffer[1] + 2 - 1];
        } else {
            // error case
            systemInformationData.errorCode = RXBuffer[3];
        }
    } else {
        // error case
        clearBuffer(systemInformationData.uid);
        systemInformationData.errorCode = RXBuffer[3];
    }
    return systemInformationData;
}


/// Sends a packet of data (c-struct) via bluetooth to a smartphone application
/// @detail Any packet to be transfered is a c-struct. These structs are defined in data_types.h.
/// @param packetIdentifier identifier that can be used to treat a packet separately in the app. Identifiers are defined in constants.h
/// @param txFlags don't know yet, what these are fore
/// @param *packetBytes pointer on the c-struct, that is the packet
/// @param byteCount number of bytes to be transfered
bool pumpViaBluetooth(unsigned short packetIdentifier, UBP_TxFlags txFlags, const char *packetBytes, unsigned short byteCount) {

    bool success = UBP_queuePacketTransmission(packetIdentifier, txFlags, packetBytes, byteCount);

    delay(1);
    #ifdef DEBUG
    if (success) Serial.println("Packet queued successfully");
    else Serial.println("Failed to enqueue packet");
    #endif
    // put your main code here, to run repeatedly:
    while (UBP_isBusy() == true) UBP_pump();
}

/// Returns the voltage on the Simblee/RFduino VDD pin as a float in Volts.
/// Code is from the RFduino Forum, see http://forum.rfduino.com/index.php?topic=265.0 for details
float voltageOnVDD() {
    analogReference(VBG);                // Sets the Reference to 1.2V band gap
    analogSelection(VDD_1_3_PS);         // Selects VDD with 1/3 prescaling as the analog source
    int sensorValue = analogRead(1);     // the pin has no meaning, it uses VDD pin
    return sensorValue * (3.6 / 1023.0); // convert value to voltage;
}

/// Print all data of systemInformationData to serial console
void printSystemInformationData(SystemInformationDataType systemInformationData) {

    Serial.printf("System information data result code:    %x\r\n", systemInformationData.resultCode);
    Serial.printf("System information data response flags: %x\r\n", systemInformationData.responseFlags);

    Serial.printf("System information data uid:            %x", systemInformationData.uid[0]);
    for (int i = 1; i < 8; i++) {
        Serial.printf(":%x", systemInformationData.uid[i]);
    }
    Serial.printf("\r\nSystem information data error code:     %x\r\n", systemInformationData.errorCode);
}


void readHeader(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {

    int maxTrials = 10; // (Re)read a block max four times

    // read first three blocks (0 , 1, 2)
    for (byte block = 0; block < 3; block++) {
        (*sensor).resultCodes[block] =  readSingleBlock((byte) block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
        #ifdef DEBUG
        printf("resultCode 0x%x\n\r", (*sensor).resultCodes[block]);
        #endif
    }

    #ifdef DEBUG
    // Check crc16 for header
    if (!checkCRC16((*sensor).fram, (byte) 0)) {
        Serial.println("CRC16 check for header failed");
    } else {
        Serial.println("CRC16 check for header succeded");
    }
    #endif
}

// read complete body data
void readBody(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {

    int maxTrials = 10; // (Re)read a block max four times
    // read block 0x27 (i.e. 39) with minute counter and block 3 with indices on trend and history data
    (*sensor).resultCodes[39] =  readSingleBlock((byte) 39, maxTrials, RXBuffer, &(sensor)->fram[0], SS_PIN);
    (*sensor).resultCodes[3] =  readSingleBlock((byte) 3, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);

    // if no read erros continue with checking for new data
    if ((*sensor).resultCodes[3] == 0x80 && (*sensor).resultCodes[39] == 0x80) {

        (*sensor).oldNextTrend = (*sensor).nextTrend;
        (*sensor).nextTrend = (*sensor).fram[26];
        (*sensor).oldNextHistory = (*sensor).nextHistory;
        (*sensor).nextHistory = (*sensor).fram[27];

        #ifdef DEBUG
        printf("resultCode Block 3: 0x%x\n\r", (*sensor).resultCodes[3]);
        printf("oldNextTrend:       0x%x\n\r", (*sensor).oldNextTrend);
        printf("nextTrend:          0x%x\n\r", (*sensor).nextTrend);
        printf("oldNextHistory:     0x%x\n\r", (*sensor).oldNextHistory);
        printf("nextHistory:        0x%x\n\r", (*sensor).nextHistory);
        #endif

        (*sensor).oldMinutesSinceStart = (*sensor).minutesSinceStart;
        (*sensor).minutesSinceStart = ((uint16_t) (*sensor).fram[317] << 8) + (uint16_t) (*sensor).fram[316]; // bytes swapped
        uint16_t minutesSinceLastReading = (*sensor).minutesSinceStart - (*sensor).oldMinutesSinceStart;

        #ifdef DEBUG
        printf("resultCode Block 39:     0x%x\n\r", (*sensor).resultCodes[39]);
        printf("oldMinutesSinceStart:    0x%x\n\r", (*sensor).oldMinutesSinceStart);
        printf("minutesSinceStart:       0x%x\n\r", (*sensor).minutesSinceStart);
        printf("minutesSinceLastReading: 0x%x\n\r", minutesSinceLastReading);
        #endif

        // amount of new trend data
        byte trendDelta = (*sensor).nextTrend - (*sensor).oldNextTrend;
        byte trendsSinceLastReading = (trendDelta >= 0) ? trendDelta : trendDelta + 16; // compensate ring buffer
        if (minutesSinceLastReading > 15) trendsSinceLastReading = 16;  // Read all 16 trend data if more than 15 minute have passed

        #ifdef DEBUG
        printf("trendDelta:             0x%x\n\r", trendDelta);
        printf("trendsSinceLastReading: 0x%x\n\r", trendsSinceLastReading);
        #endif

        // if there is new trend data, read the new trend data, along the ring buffer from old to new
        if (trendsSinceLastReading > 0) {
            int firstTrend = (*sensor).oldNextTrend;
            int firstTrendByte = 28  +  (6 * firstTrend);
            int lastTrendByte = firstTrendByte + (6 * trendsSinceLastReading) -1; // Does not consider the ring buffer (this is considered later in the reading loop)

            byte firstTrendBlock = firstTrendByte / 8; // integer division without remainder
            byte lastTrendBlock = lastTrendByte / 8;   // integer division without remainder

            #ifdef DEBUG
            printf("firstTrend:      0x%x\n\r", firstTrend);
            printf("firstTrendByte:  0x%x\n\r", firstTrendByte);
            printf("lastTrendByte:   0x%x\n\r", lastTrendByte);
            printf("firstTrendBlock: 0x%x\n\r", firstTrendBlock);
            printf("lastTrendBlock:  0x%x\n\r", lastTrendBlock);
            #endif

            // read the trend blocks that are new since last reading, but no more than 100 times to avoid endless loop in error case
            byte trendBlock = firstTrendBlock;
            byte count = 0;
            while (count <= 100) {
                if (trendBlock != 3) { // block 3 was read already and can be skipped
                    (*sensor).resultCodes[trendBlock] =  readSingleBlock(trendBlock, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
                    #ifdef DEBUG
                    printf("resultCode Block 0x%x: 0x%x\n\r", trendBlock, (*sensor).resultCodes[trendBlock]);
                    #endif
                }
                if (trendBlock == lastTrendBlock) break;
                trendBlock++;
                if (trendBlock > 15) trendBlock = trendBlock -13; // block 16 -> start over at block 3
                count++;
            }
        }

        // amount of new history data
        byte historyDelta = (*sensor).nextHistory - (*sensor).oldNextHistory;
        byte historiesSinceLastReading = (historyDelta >= 0) ? historyDelta : historyDelta + 32; // compensate ring buffer

        #ifdef DEBUG
        printf("historyDelta:              0x%x\n\r", historyDelta);
        printf("historiesSinceLastReading: 0x%x\n\r", historiesSinceLastReading);
        #endif

        // if there is new trend data, read the new trend data
        if (historiesSinceLastReading > 0) {
            int firstHistory = (*sensor).oldNextHistory;
            int firstHistoryByte = 124  +  (6 * firstHistory);
            int lastHistoryByte = firstHistoryByte + (6 * historiesSinceLastReading) -1; // Does not consider the ring buffer (this is considered later in the reading loop)

            byte firstHistoryBlock = firstHistoryByte / 8; // integer division without remainder
            byte lastHistoryBlock = lastHistoryByte / 8;   // integer division without remainder

            #ifdef DEBUG
            printf("firstHistory:       0x%x\n\r", firstHistory);
            printf("firstHistoryByte:   0x%x\n\r", firstHistoryByte);
            printf("lastHistoryByte:    0x%x\n\r", lastHistoryByte);
            printf("firstHistoryBlock:  0x%x\n\r", firstHistoryBlock);
            printf("lastHistoryBlock:   0x%x\n\r", lastHistoryBlock);
            #endif

            // read the history blocks that are new since last reading, but no more than 100 times
            byte historyBlock = firstHistoryBlock;
            byte count = 0;
            while (count <= 100) {
                if (historyBlock != 39) { // block 39 was read already and can be skipped
                    (*sensor).resultCodes[historyBlock] =  readSingleBlock(historyBlock, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
                }
                if (historyBlock == lastHistoryBlock) break;
                historyBlock++;
                if (historyBlock > 39) historyBlock = historyBlock -25; // block 40 -> start over at block 15
                count++;
            }
        }

        #ifdef DEBUG
        // Check crc16
        if (!checkCRC16((*sensor).fram, (byte) 1)) {
            Serial.println("CRC16 check for body failed");
        } else {
            Serial.println("CRC16 check for body succeded");
        }
        #endif
    }
}


void readFooter(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {

    int maxTrials = 10; // (Re)read a block max four times
    // read three blocks of footer (40, 41, 42)
    for (byte block = 40; block < 43; block++) {
        (*sensor).resultCodes[block] =  readSingleBlock((byte) block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
        #ifdef DEBUG
        printf("resultCode 0x%x\n\r", (*sensor).resultCodes[block]);
        #endif

    }

    #ifdef DEBUG
    // Check crc16 for header
    if (!checkCRC16((*sensor).fram, (byte) 2)) {
        Serial.println("CRC16 check for footer failed");
    } else {
        Serial.println("CRC16 check for footer succeded");
    }
    #endif
}


void reReadBlocksWhereResultCodeStillHasEnError(byte *RXBuffer,  Sensor *sensor, int SS_Pin) {
    int maxTrials = 3;
    for (byte block = 0; block < 40; block++) {
        if ((*sensor).resultCodes[block] != 0x80) {
            (*sensor).resultCodes[block] =  readSingleBlock(block, maxTrials, RXBuffer, (*sensor).fram, SS_PIN);
            #ifdef DEBUG
            printf("Reread block #0x%x with result code 0x%x\n\r ", block, (*sensor).resultCodes[block]);
            #endif
        }
    }
}

///===================================================================================================
//   Arduino main routines
///===================================================================================================

void setup() {
    Serial.begin(9600);
    setupPins();
    setupSPI();
    initSensor(&sensor);

    Serial.println("Please provide power to the BM019 within the next 5 seconds ...");
    delay(5000);

    sendWakeupPulse(IRQ_PIN);  // wake up BM019 and set to SPI (since SS_0 is wired up to be HIGH)
    readWakeUPEventRegister(SS_PIN, RXBuffer);
    setupBluetoothConnection();

    // --------- Transmit NFC state via bluetooth if it changed
    NFCState nfcState;
    oldNFCReady = NFCReady;
    nfcState.nfcReady = NFCReady;
    nfcStateTransmissionSuccess = pumpViaBluetooth(NFC_STATE, UBP_TxFlagIsRPC, (char *) &nfcState, sizeof(NFCState));
    #ifdef DEBUG
    printf("In Setup: Transmitted nfc state 0x%x with success %s\n\r", NFCReady, nfcStateTransmissionSuccess  ? "true" : "false");
    #endif

}

void loop() {

    #ifdef DEBUG
    Serial.println("In the loop");
    #endif


    // --------- Transmit NFC state via bluetooth if it changed
    if (!nfcStateTransmissionSuccess | NFCReady != oldNFCReady) {
        oldNFCReady = NFCReady;
        NFCState nfcState;
        nfcState.nfcReady = NFCReady;
        bool nfcStateTransmissionSuccess = pumpViaBluetooth(NFC_STATE, UBP_TxFlagIsRPC, (char *) &nfcState, sizeof(NFCState));
        #ifdef DEBUG
        printf("Loop: Transmitted nfc state 0x%x with success %s\n\r", NFCReady, nfcStateTransmissionSuccess  ? "true" : "false");
        #endif
    }


    // Start up BM019 if not yet started
    if (NFCReady == 0) {

        delay(100);
        SetProtocol_Command(); // ISO 15693 settings
        #ifdef DEBUG
        Serial.println("After SetProtocoll_Command()");
        #endif
        delay(100);

    } else {


        // ------- Read system information from BM019 ----------------------------------------------------------------------------
        #ifdef DEBUG
        Serial.println("Get system information command ...");
        #endif
        runSystemInformationCommandUntilNoError(SS_PIN, RXBuffer, 10);

        delay(5);
        #ifdef DEBUG
        Serial.println("... retreive system information ...");
        #endif
        SystemInformationDataType systemInformationData = systemInformationDataFromGetSystemInformationResponse(RXBuffer);
        delay(5);


        // ------- Transmit system information data via bluetooth. By convention this is the first transmission -----------------

        bool ergo = pumpViaBluetooth(SYSTEM_INFORMATION_DATA, UBP_TxFlagIsRPC, (char *) &systemInformationData, sizeof(SystemInformationDataType));
        #ifdef DEBUG
        printSystemInformationData(systemInformationData);
        #endif




        //------- Read IDN information from BM019 (IDN Command) -------------------------------------------------------------------

        #ifdef DEBUG
        Serial.println("IDN command ...");
        #endif
        delay(5);

        runIDNCommand(SS_PIN, RXBuffer, 10);
        #ifdef DEBUG
        Serial.println("... retreive IDN information ...");
        #endif
        IDNDataType idnData = idnDataFromIDNResponse(RXBuffer);




        // ----------- Read 43 data blocks, but only blocks that have not been read yet ---------------

        if (true) {

            #ifdef DEBUG
            printf("Starting of new code section ...");
            #endif

            memcpy(sensor.uid, systemInformationData.uid, sizeof(sensor.uid));

            readHeader(RXBuffer, &sensor, SS_PIN);
            if (!checkCRC16(sensor.fram, (byte) 0)) readHeader(RXBuffer, &sensor, SS_PIN); // one more time

            readBody(RXBuffer, &sensor, SS_PIN);
            if (!checkCRC16(sensor.fram, (byte) 1)) readBody(RXBuffer, &sensor, SS_PIN); // one more time

            readFooter(RXBuffer, &sensor, SS_PIN);
            if (!checkCRC16(sensor.fram, (byte) 2)) readFooter(RXBuffer, &sensor, SS_PIN); // one more time

            // TODO: test the following block and also switch to pumpViaBluetooth and get rid of print section
            //reReadBlocksWhereResultCodeStillHasEnError(RXBuffer, &sensor, SS_PIN);

            // // try again for blocks that had read errors
            // int maxTrials = 3;
            // for (byte block = 0; block < 40; block++) {
            //     if (sensor.resultCodes[block] != 0x80) {
            //         sensor.resultCodes[block] =  readSingleBlock(block, maxTrials, RXBuffer, sensor.fram, SS_PIN);
            //         printf("Reread block #0x%x with result code 0x%x\n\r ", block, sensor.resultCodes[block]);
            //     }
            // }

            reReadBlocksWhereResultCodeStillHasEnError(RXBuffer, &sensor, SS_PIN);

            // print for debuging
            #ifdef DEBUG
            for (byte block = 0; block < 40; block++) {
                printf("sensor.fram block #0x%x with result code 0x%x", block, sensor.resultCodes[block]);
                for (byte i = 0; i < 8; i++) {
                    printf(" %x", sensor.fram[block*8 + i]);
                }
                printf("\n\r");
            }
            printf("... end of new code section.");
            #endif
        }

        // ----------- Read 43 data blocks into RXBuffer (OLD CODE) ---------------

        // IMPORTANT:
        // use dataBuffer for bluetooth transfer if this code section shall be used again
        if (false) {
            #ifdef DEBUG
            Serial.println("Read all data");
            #endif

            memset(RXBuffer, 0, sizeof(dataBuffer));
            memset(dataBuffer, 0, sizeof(dataBuffer));

            byte resultCode = 0;
            int trials = 0;
            int maxTrials = 10;
            for (int i = 0; i < 43; i++) { // Need only 43 of 244 blocks
                resultCode = ReadSingleBlockReturn(i);

                #ifdef DEBUG
                printf("resultCode 0x%x\n\r", resultCode);
                #endif
                if (resultCode != 0x80 && trials < maxTrials) {
                    #ifdef DEBUG
                    printf("Error 0x%x\n\r", resultCode);
                    #endif
                    i--;        // repeat same block if error occured, but
                    trials++;   // not more than maxTrials times per block
                } else if (trials >= maxTrials) {
                    break;
                } else {
                    trials = 0;

                    for (int j = 3; j < RXBuffer[1] + 3 - 4; j++) {
                        dataBuffer[i * 8 + j - 3] = RXBuffer[j];
                        #ifdef DEBUG
                        Serial.print(RXBuffer[j], HEX);
                        Serial.print(" ");
                        #endif
                    }
                }
            }
        }

        // ----------- All data collected, send BM019 to hibernate -------------------------

        #ifdef DEBUG
        Serial.println("Sending CR95HF to hibernate ...");
        #endif
        delay(5);

        sendCR95HFToHibernate(SS_PIN);


        //-------- Read Battery level and Simblee temperature and transmit via bluetooth ---------------------

        BatteryDataType batteryData;
        batteryData.voltage = voltageOnVDD();
        #ifdef SIMBLEE
        batteryData.temperature = Simblee_temperature(CELSIUS);
        #else
        batteryData.temperature = RFduino_temperature(CELSIUS);
        #endif

        #ifdef DEBUG
        Serial.printf("Battery voltage: %f\r\n", batteryData.voltage);
        #endif
        // Transmit via bluetooth
        bool success = UBP_queuePacketTransmission(BATTERY_DATA, UBP_TxFlagIsRPC, (char *) &batteryData, sizeof(BatteryDataType));
        #ifdef DEBUG
        if (success) Serial.println("Battery data packet queued successfully");
        else Serial.println("Failed to enqueue battery data packet");
        #endif
        // put your main code here, to run repeatedly:
        while (UBP_isBusy() == true) UBP_pump();

        #ifdef DEBUG
        Serial.printf("Sent Battery voltage: %f\r\n", batteryData.voltage);
        #endif

        //-------- transmit sensor.fream (was dataBuffer in old code) via bluetooth ---------------------------------------------------------

        AllBytesDataType allBytes;
        #ifdef DEBUG
        Serial.println("----about to send all data bytes packet");
        #endif

        memset(allBytes.allBytes, 0, sizeof(allBytes.allBytes));
        memcpy(allBytes.allBytes, sensor.fram, sizeof(sensor.fram));
//        memcpy(allBytes.allBytes, dataBuffer, sizeof(dataBuffer)); // old code

        success = UBP_queuePacketTransmission(ALL_BYTES, UBP_TxFlagIsRPC, (char *) &allBytes, sizeof(AllBytesDataType));
        #ifdef DEBUG
        if (success) Serial.println("----all data bytes packet queued successfully");
        else Serial.println("----Failed to enqueue all data bytes packet");
        #endif
        while (UBP_isBusy() == true) UBP_pump();



        //-------- transmit IDN data via bluetooth. By convention this is the last transmission -----------------

        // TODO: change code to pumpViaBluetooth here and at other places, too
        //ergo = pumpViaBluetooth(IDN_DATA, UBP_TxFlagNone, (char *) &idnData, sizeof(IDNDataType));
        success = UBP_queuePacketTransmission(IDN_DATA, UBP_TxFlagIsRPC, (char *) &idnData, sizeof(IDNDataType));
        delay(10);
        #ifdef DEBUG
        if (success) Serial.println("IDN data packet queued successfully");
        else Serial.println("Failed to enqueue IDN data packet");
        #endif
        // put your main code here, to run repeatedly:
        while (UBP_isBusy() == true) UBP_pump();

        #ifdef DEBUG
        Serial.printf("IDN: %x", idnData.deviceID[0]);
        for (int i = 1; i < 13; i++) {
            Serial.printf(":%x", idnData.deviceID[i]);
        }
        Serial.println("... done");
        #endif

        Serial.print("."); // Heartbeat for console

        //--------- send Simblee into ultra low power mode ---------------------------------------------------

        #ifdef DEBUG
        Serial.println("Sending Simblee/RFduino to sleep ...");
        #endif
        #ifdef SIMBLEE
        Simblee_ULPDelay(SLEEP_DURATION);
        #else
        RFduino_ULPDelay(SLEEP_DURATION);
        #endif


        //--------- send Simblee woke up again, now also wake up BM019 and repeat the loope cycle ------------

        #ifdef DEBUG
        Serial.println("... Simblee/RFduino woke up again");
        Serial.println("Wake up CR95HF with wake up pulse...");
        #endif
        sendWakeupPulse(IRQ_PIN);  // Wake up BM019 (low pulse on IRQ_PIN)
        readWakeUPEventRegister(SS_PIN, RXBuffer);
        setupSPI();

        delay(10);
        SetProtocol_Command(); // ISO 15693 settings
        delay(100);

        #ifdef DEBUG
        Serial.println("... CR95HF woke up again. Receiving wake up response");
        #endif
    }


}
