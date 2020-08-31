
///Contains functions for the intizilation and operation of the End of Fiber Unit Peripherial and Interface Board

///Created by Doug Goetz June 2020


#ifndef EPICLibrary_h
#define EPICLibrary_h

//Arduino basic libraries
#include <Arduino.h>
#include "SPI.h"
#include "Wire.h"

//For RTC
#include <TimeLib.h>
#include <DS1307RTC.h>

//For LORA communiations
//#include "LoRaToolbox.h"
    //radiohead library called within the Loratoolbox

//For GPS decoding
#include "TinyGPS++.h"

//For SD card storage
#include <SdFat.h>

//For One Wire temperature sensing
//#include <OneWire.h>
#include "TSensor1WireBus.h"


//////////// Serial Port constants /////////////
#define TSENSerial  Serial1 //RX/TX to TSEN via RS232 converter
#define DaisySerial Serial2 //RX/TX for daisy chain communications
#define GPSSerial Serial4 //RX/TX to UBLOX GPS receiver
#define SpareSerial Serial5 //Spare TTL serial if RS232 converter is not used

///////// Definitions //////////////
#define GPS_EXTINT 28 //use if pullup isn't installed
#define GPS_RESET 29


//Solar board interface
#define SOLAR_PWR 25 //solar charger on/off. Only set when R15 pullup isn't used
#define SOLAR_SHUTDOWN 26 //solar input shutdown switch. Pulled low on start up
#define BATT_HEATER 27 //High = on

//LORA
#define RF_CS 19   // Chip Select
#define RF_RESET 18  // Reset
#define RF_INT 3   // Interupt
#define RF_FREQ 915.0

//RH_RF95::ModemConfig short_range = {0};
//char short_packet[41] = {0};
////RH_RF95 rf95(RF_CS, RF_INT, hardware_spi);


//TSEN
#define TSEN_EN 24

//T Sensors
#define TBATT 2
#define TSPARE 3
#define TPCB 4


//Analog channels
#define TEENSY_V  A1 //3v3 output from teensy
#define BATT_V A2 //Lithium ion voltage
#define DCDC_V A3 //Output from 6-9V DC to DC converter that powers TSEN

#define TXBUFSIZE 2000

//Constants for software reboot of 3.X
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))


class EPICLibrary
{
    public: EPICLibrary() {};

    enum AnalogChannel : uint8_t {
		TEENSY,
		BATT,
		TSEN	
	};


    float TBatt_Val;
    float TSpare_Val;
    float TPBC_Val;

    uint16_t TSEN_T;
    uint16_t TSEN_TP;
    uint16_t TSEN_P;

    void Setup();

    float GetAnalogVoltage(uint8_t channel);

    // Generic Solar Charger functions
    void SolarOff(); //only off during reset
    void SolarOn();

    void HeaterOn();
    void HeaterOff();

    void ListenForTSEN();
    void RequestTSEN();

    void TSENOn();
    void TSENOff();

    void ResetLoRa();

    // UBLOX GPS controls
    void GPSreset();
    void configureUblox(byte *);
    void calcChecksum(byte *, byte);
    void sendUBX(byte *, byte);
    byte getUBX_ACK(byte *);
    void setBaud(byte);
    bool gpsStatus[7] = {false, false, false, false, false, false, false};
    


};

#endif 







