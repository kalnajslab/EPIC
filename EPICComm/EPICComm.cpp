/*
 *  EPICComm.cpp
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the PIB and PU. The class inherits its protocol from the SerialComm
 *  class which has been modified to the LoRaComm class to allow communications through the 
 *  LoRa.h library.   LoRa uses packetized communications, this library converts the Stream class that
 *  SerComm expects into a buffer using LoopbackStream, which can then be sent to the LoRa object (TX)
 *  and filled from the LoRa object (RX). 
 */

#include "EPICComm.h"

EPICComm::EPICComm(Stream * serial_port)
    : LoRaComm(serial_port)
{
}

// For use with default SPI and pins
bool EPICComm::Begin(long frequency) 
{
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

// If you need to use different pins (ie EPIC Board)
bool EPICComm::Begin(long frequency,int SSPin, int ResetPin, int InteruptPin)
{
    LoRa.setPins(SSPin, ResetPin,InteruptPin);
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

// If you need to use different pins and a different SPI (ie Profiler)
bool EPICComm::Begin(long frequency,int SSPin, int ResetPin, int InteruptPin, SPIClass& spi)
{
    LoRa.setPins(SSPin, ResetPin,InteruptPin);
    LoRa.setSPI(spi);
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

void EPICComm::SetModulation(int SpreadingFactor, long BandWidth)
{
    LoRa.setSpreadingFactor(SpreadingFactor);
    LoRa.setSignalBandwidth(BandWidth);
}

void EPICComm::SetTXPower(int Power)
{
    LoRa.setTxPower(Power);

}

// MonDo -> EPIC (with params) ---------------------------

bool EPICComm::TX_SetHeaters(float HeaterT)
{
    if (!Add_float(HeaterT)) return false;
   
    TX_ASCII(EFU_SET_HEATERS);

    return true;
}

bool EPICComm::RX_SetHeaters(float * HeaterT)
{
    float temp1;

    if (!Get_float(&temp1)) return false;
   
    *HeaterT = temp1;

    return true;
}

bool EPICComm::TX_SetDataRate(int32_t DataRate)
{
    if (!Add_int32(DataRate)) return false;
   
    TX_ASCII(EFU_SET_DATA_RATE);

    return true;
}

bool EPICComm::RX_SetDataRate(int32_t * DataRate)
{
    int32_t temp1;

    if (!Get_int32(&temp1)) return false;
   
    *DataRate = temp1;

    return true;
}

bool EPICComm::TX_SetTxRate(int32_t TxRate)
{
    if (!Add_int32(TxRate)) return false;
   
    TX_ASCII(EFU_SET_TX_RATE);

    return true;
}

bool EPICComm::RX_SetTxRate(int32_t * TxRate)
{
    int32_t temp1;

    if (!Get_int32(&temp1)) return false;
   
    *TxRate = temp1;

    return true;
}


bool EPICComm::TX_Status(uint32_t EFUTime, float VBattery, float VTSEN, float T_PCB, float T_Battery, uint8_t HeaterStat)
{
    if (!Add_uint32(EFUTime)) return false;
    if (!Add_float(VBattery)) return false;
    if (!Add_float(VTSEN)) return false;
    if (!Add_float(T_PCB)) return false;
    if (!Add_float(T_Battery)) return false;
    if (!Add_uint8(HeaterStat)) return false;
   
    TX_ASCII(EFU_STATUS);

    return true;
}

bool EPICComm::RX_Status(uint32_t * EFUTime, float * VBattery, float * VTSEN, float * T_PCB, float * T_Battery, uint8_t * HeaterStat)
{
    uint32_t temp1;
    float temp2, temp3, temp4, temp5;
    uint8_t temp6;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_float(&temp4)) return false;
    if (!Get_float(&temp5)) return false;
    if (!Get_uint8(&temp6)) return false;


    *EFUTime = temp1;
    *VBattery = temp2;
    *VTSEN = temp3;
    *T_PCB = temp4;
    *T_Battery = temp5;
    *HeaterStat = temp6;

    return true;
}

// // -- EFU to PIB error string

bool EPICComm::TX_Error(const char * error)
{
    if (Add_string(error)) return false;

    TX_ASCII(EFU_ERROR);

    return true;
}

bool EPICComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}


