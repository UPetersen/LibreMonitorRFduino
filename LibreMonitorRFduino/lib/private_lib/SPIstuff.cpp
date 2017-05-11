//
// SPIstuff.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		BM019 Inventory
//
// Created by 	Uwe Petersen, 15.11.15 12:13
// 				Uwe Petersen
//
// Copyright 	(c) Uwe Petersen, 2015
// Licence		<#license#>
//
// See 			SPIstuff.h and ReadMe.txt for references
//


// Library header
#include <SPI.h>
#include "SPIstuff.h"

// Code

//#define DEBUG


//bool responseHasError(byte *RXBuffer) {
//    
//    Serial.printf("Response is RXBuffer[0]: %x, RXBuffer[2]: %x \r\n", RXBuffer[0], RXBuffer[2]);
//    // Check if result code has error
//    if (RXBuffer[0] != 0x80) {
//        // Check if error in response flag (i.e. bit 0 of response flag is set) is not zero
//        if ((RXBuffer[2] & 0x01) != 1) {
//            return false; // no errors, thus break and transmit via bluetooth
//        }
//    }
//    return true;
//}



void sendWakeupPulse(uint8_t wakeupPin) {
    
#ifdef DEBUG
    Serial.println("Send wake up pulse to BM019 and configure to use SPI...");
#endif
    // IRQ must be set to HIGH (or already be at HIGH) on start up, to later provide LOW pulse
    digitalWrite(wakeupPin, HIGH);
    
    // Wait at least 100 micro seconds after power up. Set to 10 ms here, just to be safe
    delay(10);
    
    // Send wake up pulse to put the BM019 into SPI or UART mode
    digitalWrite(wakeupPin, LOW);
    delayMicroseconds(100);
    digitalWrite(wakeupPin, HIGH);
    
    // First valid command will be accepted after 10 ms
    delay(10);
#ifdef DEBUG
    Serial.println("... done sending wake up pulse to BM019 and configuring to use SPI.");
#endif
}



bool responseHasNoError(byte *RXBuffer) {
    
#ifdef DEBUG
    Serial.printf("Response is RXBuffer[0]: %x, RXBuffer[2]: %x \r\n", RXBuffer[0], RXBuffer[2]);
#endif
    // Check if result code has no error, i.e. 0x80 for normal commands
    if (RXBuffer[0] == 0x80) {
        // Check if no error in response flag (i.e. bit 0 of response flag is not set, i.e. it is zero)
        if ((RXBuffer[2] & 0x01) == 0) {
            return true; // no errors, thus break
        }
    }
    return false;
}

bool idnResponseHasNoError(byte *RXBuffer) {
    
#ifdef DEBUG
    Serial.printf("IDN response is RXBuffer[0]: %x \r\n", RXBuffer[0]);
#endif
    // Check if result code has no error
    if (RXBuffer[0] == 0x00) { // 0x00 means no error for IDN command
        return true; // no errors, thus break
    }
    return false;
}

//void sendPollReceiveSPI(int ssPIN, byte *RXBuffer) {
//    
//    // step 1 send the command, which is stored in RXBuffer
//    sendSPICommand(ssPIN, RXBuffer, 4);
//    
//    // step 2, poll for data ready
//    pollSPIUntilResponsIsReady(ssPIN, RXBuffer);
//    
//    // step 3, read the data into RXBuffer
//    receiveSPIResponse(ssPIN, RXBuffer);
//}

void sendPollReceiveSPINew(int ssPIN, byte *command, int commandLength, byte *RXBuffer) {
    
    // step 1 send the command, which is stored in RXBuffer
    sendSPICommand(ssPIN, command, commandLength);
    
    // step 2, poll for data ready
    pollSPIUntilResponsIsReady(ssPIN, RXBuffer);
    
    // step 3, read the data into RXBuffer
    receiveSPIResponse(ssPIN, RXBuffer);
}

void readWakeUPEventRegister(int ssPIN, byte *RXBuffer) {
    int length = 5;
    byte command[length];
    command[ 0] = 0x08;   // command code for the read register command
    command[ 1] = 0x03;   // length of data that follows (3 bytes)
    command[ 2] = 0x62;   // register adress (0x62 for wake-up event register, 0x69 fo ARC_B register)
    command[ 3] = 0x01;   // register size
    command[ 4] = 0x00;   // ST Reserved
    
    sendPollReceiveSPINew(ssPIN, command, sizeof(command), RXBuffer);
 
#ifdef DEBUG
    printWakeUpRegisterResponseToSerial(RXBuffer);
#endif
}

void printWakeUpRegisterResponseToSerial(byte *RXBuffer) {

    // Print results of read wake up register to serial console

    Serial.printf("Printing result of read wake-up register command ...\r\n");
    Serial.printf("  Result code (byte 0): %x\r\n", RXBuffer[0]);
    
    Serial.printf("Length of data (byte 1): %d\r\n", RXBuffer[1]);
    if (RXBuffer[1] > 0) {
        for (int i=2; i<2+RXBuffer[1]; i++) {
            Serial.printf("   Data byte %d: %x\r\n", i, RXBuffer[i]);
        }
    }
    Serial.println("... finished printing wake-up register result.");
}

void sendCR95HFToHibernate(int ssPIN) {
    int length = 16;
    byte command[length];
    command[ 0] = 0x07;   // command code for idle CR95HF command
    command[ 1] = 0x0E;   // length of data that follows (3 bytes)
    command[ 2] = 0x08;   // wakeup source (0x01 time out and 0x08 low puls on IRQ_IN
    command[ 3] = 0x04;   // enter control 0x0400 hibernate (even clock doesn't run)
    command[ 4] = 0x00;   // enter control
    command[ 5] = 0x04;   // wakeup control 0x0400 hibernate
    command[ 6] = 0x00;   // wakeup control
    command[ 7] = 0x18;   // leave control 0x1800 hibernate
    command[ 8] = 0x00;   // leave control
    command[9 ] = 0x00;   // wakeup period
    command[10] = 0x00;   // OSC start
    command[11] = 0x00;   // DAC start
    command[12] = 0x00;   // DAC data
    command[13] = 0x00;   // DAC data
    command[14] = 0x00;   // swing count
    command[15] = 0x00;   // max sleep
    
    // run command until no error, but only max 10 times
    sendSPICommand(ssPIN, command, sizeof(command));
}

void receiveCR95HFWakeupResponse(int ssPin, byte *RXBuffer) {
    
    digitalWrite(ssPin, HIGH);

    // step 2, poll for data ready
    pollSPIUntilResponsIsReady(ssPin, RXBuffer);
    
    // step 3, read the data into RXBuffer
    receiveSPIResponse(ssPin, RXBuffer);
}


void sendSPICommand(int ssPin, byte *commandArray, int length) {
    digitalWrite(ssPin, LOW);
    
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    
    for (int i=0; i<length; i++) {
        SPI.transfer(commandArray[i]);
    }
    digitalWrite(ssPin, HIGH);
    delay(1);
}

void pollSPIUntilResponsIsReady(int ssPin, byte *RXBuffer) {
    
    digitalWrite(ssPin , LOW);
    
    while(RXBuffer[0] != 8) {
        RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
#ifdef DEBUG
        Serial.printf("SPI polling response byte:%x\r\n", RXBuffer[0]);
#endif
        RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set in response byte
    }
    digitalWrite(ssPin, HIGH);
    delay(1);
}

void receiveSPIResponse(int ssPin, byte *RXBuffer) {
    
    digitalWrite(ssPin, LOW);
    SPI.transfer(0x02);   // SPI control byte for read
    
    RXBuffer[0] = SPI.transfer(0);  // response code
    RXBuffer[1] = SPI.transfer(0);  // length of data
    
    for (byte i=0; i<RXBuffer[1]; i++)
        RXBuffer[i+2] = SPI.transfer(0);  // data
    
    digitalWrite(ssPin, HIGH);
    delay(1);
}

/// @brief clears the buffer by filling up with zeros
void clearBuffer(byte *RXBuffer) {
    
    memset(RXBuffer, 0, sizeof(RXBuffer));
}






