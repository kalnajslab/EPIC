/*
 *  EPICComm.h
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the PIB and the PU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef EPICComm_H
#define EPICComm_H

#include "LoRaComm.h"

/* Note for use with Teensy 3.X, the following lines in LoRa.cpp need to be modified:
Line 365:
    SPI.notUsingInterrupt((IRQ_NUMBER_t)digitalPinToInterrupt(_dio0));
Line 383: 
  SPI.notUsingInterrupt((IRQ_NUMBER_t)digitalPinToInterrupt(_dio0));
*/
#include <LoRa.h>
#include <SPI.h>     

enum PUMessages_t : uint8_t {
    EFU_NO_MESSAGE = 0,

    // MonDo -> EFU (no params)
    EFU_SEND_STATUS, //1
    EFU_RESET, //2

    // MonDo -> EFU (with params)
    EFU_SET_HEATERS, //3
    EFU_SET_DATA_RATE, //4
    EFU_SET_TX_RATE, //5

    // EFU -> MonDo (no params)
    EFU_DATA_RECORD,  //6 binary transfer

    // EFU -> MonDo (with params)
    EFU_STATUS, //7
    EFU_ERROR //8
};


class EPICComm : public LoRaComm {
public:
    EPICComm(Stream * serial_port);
    ~EPICComm() { };

    //Setup 
    bool Begin(long frequency);
    bool Begin(long frequency,int SSPin, int ResetPin, int InteruptPin);
    bool Begin(long frequency,int SSPin, int ResetPin, int InteruptPin, SPIClass& spi);
    void SetModulation(int SpreadingFactor, long BandWidth);
    void SetTXPower(int Power);



    // MonDo -> EPIC (with params) -----------------------
    bool TX_SetHeaters(float Heater1T); //Set the heater temperature (parameter not state)
    bool RX_SetHeaters(float * Heater1T);
    bool TX_SetDataRate(int32_t DataRate);
    bool RX_SetDataRate(int32_t * DataRate);
    bool TX_SetTxRate(int32_t TxRate);
    bool RX_SetTxRate(int32_t * TXRate);


    // EPIC -> MonDo (with params) -----------------------

    bool TX_Status(uint32_t EFUTime, float VBattery, float VTSEN, float T_PCB, float T_Battery, uint8_t HeaterStat);
    bool RX_Status(uint32_t * EFUTime, float * VBattery, float * VTSEN, float * T_PCB, float * T_Battery, uint8_t * HeaterStat);

    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);
};

#endif /* EPICComm_H */
