
#include "EPICLibrary.h"


EPICLibrary EPIC;

RH_RF95::ModemConfig short_range = {0};
char short_packet[41] = {0};
RH_RF95 rf95(RF_CS, RF_INT, hardware_spi); //Initialize radiohead library

TSensor1Bus  TempPCB(TPCB); //initialize temperautre sensors
TSensor1Bus  TempBatt(TBATT);
TSensor1Bus  TempSpare(TSPARE);

//Used for storing 750ms temperature records
float TPCB_val;
float TBatt_val;
float TSpare_val;

//TinyGPSPlus gps;
byte UBLOXsettingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00}; //

long timer;




void setup() {

    Serial.begin(9600);
    EPIC.Setup();

    Serial.println("exit EPIC setup");

    
    //Setup UBLOX GPS reciever
   // EPIC.GPSreset();
   // EPIC.configureUblox(UBLOXsettingsArray);
    
    
    //check 1-wire temperature sensors
    if(!TempPCB.ValidateAddrCRC()){ 
        Serial.println(" PCB sensor crc bad, check sensor connection");
    }
    if(!TempBatt.ValidateAddrCRC()){  
        Serial.println("Batt sensor crc bad, check sensor connection");
   }
    if(!TempSpare.ValidateAddrCRC()){ 
        Serial.println("Spare sensor crc bad, check sensor connection");
    }

    //Setup LoRa
   // CreateModemConfig(&short_range, BW_500, CR_4_5, SF_11);
   // rf95.setModemRegisters(&short_range);
   // EPIC.ResetLoRa();

  //  while(!rf95.init()) {
  //    Serial.println("rf95.init failed");
   // }

    //// configure the radio
    //rf95.setFrequency(RF_FREQ);
    //rf95.setTxPower(TX_5dBm, false); //to do: find lowest power option for EFU
    //rf95.setModemRegisters(&short_range);
    //Serial.println(" LORA frequency, power, and rf parameters set");

    
    timer = millis();

    
}


void loop(){

    TempPCB.ManageState(TPCB_val); //use this in the main loop outside of any control timers. A non blocking internal 750ms timer is used to between the start of the sensor conversion and when the conversion is ready to be read.
    TempBatt.ManageState(TBatt_val); 
    TempSpare.ManageState(TSpare_val); 

//    while (GPSSerial.available())  //Check for GPS serial
//      {
//        char c = GPSSerial.read();
//        gps.encode(c);
//        Serial.print(c);
//       }

    if(millis()-timer>10000){

      EPIC.RequestTSEN();  
      Serial.print(TPCB_val);
      Serial.print(",");
      Serial.print(TBatt_val);
      Serial.print(",");  
      Serial.print(TSpare_val);
      Serial.print(","); 
      Serial.println(EPIC.TSEN_T);
      timer = millis();
        
    }
    
   EPIC.ListenForTSEN();
    
}
