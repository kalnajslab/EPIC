#include "EPICLibrary.h"
#include "EPICComm.h"
#include <TinyGPS++.h>
#include <Adafruit_SleepyDog.h>
#include <SdFat.h>
#include <SdFatConfig.h>

//Teensy 3.6 specific SD card config
#define USE_SDIO 1

#define DEBUG_SERIAL Serial
#define RECORD_LENGTH 9  //4 GPS ints + 4 TSEN Ints + 1 Status int - ie twice this in bytes (18KB)
#define EFU_RECORDS 32 // Number of records to hold in memory

enum AnalogChannel : uint8_t {
		TEENSY,
		BATT,
		TSEN	
	};

/* Variables that can be reconfigured */
float HeaterSetpoint = 0.0;
int DataRate = 5; //Period at which to collect GPS/TSEN data
int RecordsToSend = 6; //Number of data records to collect before LoRa TX

/* Other */
float DeadBand = .5;
long startTime = 0; // milliseconds to start next measurement
int recordInd = 0; //how many records have been accumulated
time_t GPSStartTime;  //Unix time_t of starttime of profile from PU clock
float GPSStartLat;  // Initial Latitude decimal degrees (float32) = 1m
float GPSStartLon; //Initial Longitude decimal degrees (float32) = 1m
uint16_t EFUData[EFU_RECORDS][RECORD_LENGTH] = {0};  // Buffer to hold EFU data records

uint8_t bytes[sizeof(float)]; //byte array for converting floats to uint16_t

float V_Battery;
float V_DCDC;
float V_3V3;

float TBatt_val;
float TSpare_val;
float TPCB_val;

uint16_t TSEN_T = 0x1234;
uint32_t TSEN_TP = 0x9ABCDEF0;
uint16_t TSEN_P = 0x5678;

bool HeaterStatus = false;
byte UBLOXsettingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};


LoopbackStream LoRaBuff;
TinyGPSPlus gps;
SdFatSdio SD;
EPICComm EC(&LoRaBuff);
EPICLibrary EPIC;

TSensor1Bus  TempPCB(TPCB); //initialize temperautre sensors


int counter = 0;

void setup() {
  EPIC.Setup(); //initialize pins and ports
  delay(2000);  //wait for serial port

  DEBUG_SERIAL.println("EPIC Test Sketch");

/*Set Up the watchdog with a 60s timeout */
  int countdownMS = Watchdog.enable(60000);
  DEBUG_SERIAL.print("Enabled the watchdog with max countdown of ");
  DEBUG_SERIAL.print(countdownMS, DEC);
  DEBUG_SERIAL.println(" milliseconds!");

     //Setup UBLOX GPS reciever
   EPIC.GPSreset();
   EPIC.configureUblox(UBLOXsettingsArray);
      
  if (!EC.Begin(915E6, 19, 18, 30)) {
    DEBUG_SERIAL.println("Starting LoRa failed!");
    while (1);
  }
  EC.SetModulation(11, 250E3);
  EC.SetTXPower(12);

  if(!TempPCB.ValidateAddrCRC()){ 
        DEBUG_SERIAL.println(" PCB sensor crc bad, check sensor connection");
    }

  DEBUG_SERIAL.println("SetUp Complete");  
}

void loop() {
  
  checkForSerial();

  if (millis() > startTime)
  {
    startTime = millis() + DataRate*1000;
    DEBUG_SERIAL.println("Collect Record: " + String(recordInd));
    checkForSerial();
    V_Battery = EPIC.GetAnalogVoltage(BATT);
    V_DCDC = EPIC.GetAnalogVoltage(TSEN);
    V_3V3 = EPIC.GetAnalogVoltage(TEENSY);
    TempPCB.ManageState(TPCB_val);
    recordInd = SaveRecord(recordInd);
   

    if (recordInd > RecordsToSend)
    {
      checkForSerial();
      DEBUG_SERIAL.println("Sending Binary");
      EC.AssignBinaryTXBuffer((uint8_t *) EFUData, (recordInd)*RECORD_LENGTH*2, (recordInd)*RECORD_LENGTH*2);
      EC.TX_Bin(EFU_DATA_RECORD);
      recordInd = 0;
      updateRTCfromGPS();
    }
  }
}


bool SerComRX()
 {
     /* Recieves commands from the PIB and parses them.  
     */
     int8_t tmp1;
     
     switch (EC.RX()) {
         case ASCII_MESSAGE:
            DEBUG_SERIAL.print("Received message: "); DEBUG_SERIAL.println(EC.ascii_rx.msg_id);
            DEBUG_SERIAL.print("Message Content: "); DEBUG_SERIAL.println(EC.ascii_rx.buffer);
            switch (EC.ascii_rx.msg_id){
                case EFU_SET_HEATERS:
                    tmp1 = EC.RX_SetHeaters(&HeaterSetpoint);
                    EC.TX_Ack(EFU_SET_HEATERS,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_HEATER: ");
                        DEBUG_SERIAL.println(HeaterSetpoint); 
                    }
                    return false;
                case EFU_SET_DATA_RATE:
                    tmp1 = EC.RX_SetDataRate(&DataRate);
                    EC.TX_Ack(EFU_SET_DATA_RATE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_DATA_RATE: ");
                        DEBUG_SERIAL.println(DataRate); 
                    }
                    return false;
                case EFU_SET_TX_RATE:
                    tmp1 = EC.RX_SetTxRate(&RecordsToSend);
                    EC.TX_Ack(EFU_SET_TX_RATE,tmp1);
                    if(tmp1)
                    {
                        DEBUG_SERIAL.print("Received EFU_SET_TX_RATE: ");
                        DEBUG_SERIAL.println(RecordsToSend); 
                    }
                    return false;
                case EFU_RESET:
                    EC.TX_Ack(EFU_RESET,true);
                    DEBUG_SERIAL.println("[warning] Rebooting in 2 seconds");
                    delay(2000);
                    WRITE_RESTART(0x5FA0004);
                    return false;
                case EFU_SEND_STATUS:
                    uint32_t tmp3 = now();
                    tmp1 = EC.TX_Status(tmp3,V_Battery,V_DCDC, TPCB_val, TPCB_val, HeaterStatus);
                    DEBUG_SERIAL.println("Received EFU_SEND_STATUS");
                    return false;
                
                default:
                    EC.TX_Ack(EC.ascii_rx.msg_id,false);
                    return false;   
            }
        case ACK_MESSAGE:
                DEBUG_SERIAL.print("ACK/NAK for msg: "); DEBUG_SERIAL.println(EC.ack_id);
                DEBUG_SERIAL.print("Value: ");
                EC.ack_value ? DEBUG_SERIAL.println("ACK") : DEBUG_SERIAL.println("NAK");
                return false;
        case BIN_MESSAGE:
               DEBUG_SERIAL.print("Binary message: "); Serial.println(EC.binary_rx.bin_id);
              DEBUG_SERIAL.print("Buffer: ");
              for (int i = 0; i < EC.binary_rx.bin_length; i++) {
                DEBUG_SERIAL.print((char) EC.binary_rx.bin_buffer[i]);
              }
              DEBUG_SERIAL.println();
             
              break;
        case NO_MESSAGE:
          default:
                return false;
        return false;                    

        }
 }

int SaveRecord(int dataindx)
{
    if (dataindx >= EFU_RECORDS) //check we don't go outside buffer
        dataindx = 1;
    
    if(dataindx == 0) //first line of array is header line
    {
        GPSStartTime = now(); //get time as time_t from Teensy clock (4 bytes)
        EFUData[0][0] = (uint16_t) (GPSStartTime);
        EFUData[0][1] = (uint16_t) (GPSStartTime >> 16);  

        GPSStartLat = gps.location.lat();
        float single = (float)GPSStartLat;  //convert double to single float
        *(float*)(bytes) = single;  // convert float to bytes
        EFUData[0][2] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        EFUData[0][3] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        GPSStartLon = gps.location.lng();
        single = (float)GPSStartLon;  //convert double to single float
        *(float*)(bytes) = single;  // convert float to bytes
        EFUData[0][4] = (uint16_t)((bytes[1] << 8) + bytes[0]); //low bytes (little endian)
        EFUData[0][5] = (uint16_t)((bytes[3] << 8) + bytes[2]); //high bytes (little endian)
        
        EFUData[0][6] = (uint16_t) (gps.altitude.meters()); //Altitude in meters
         
        EFUData[0][7] = (uint16_t)(V_Battery*1000);  //Voltage in mV
        EFUData[0][8] = (uint8_t)(V_DCDC*10) + ((uint8_t)(V_3V3*50) << 8);  //Voltage in 100mv and 20mV resolution
       
        DEBUG_SERIAL.println("Time: " + String(now()));
        DEBUG_SERIAL.println("GPS: " + String(GPSStartLat) + "," + String(GPSStartLon) + "," + String(gps.altitude.meters()));
        DEBUG_SERIAL.println("V Battery: " + String(V_Battery));
        DEBUG_SERIAL.println("V TSEN: " + String(V_DCDC));
        DEBUG_SERIAL.println("V Teensy: " + String(V_3V3));
        DEBUG_SERIAL.println("T PCB: " + String(TPCB_val));
        dataindx++;
    }
        EFUData[dataindx][0] = (uint16_t)(now() - GPSStartTime); // Elapsed time in seconds
        EFUData[dataindx][1] = (int16_t)((gps.location.lat() - GPSStartLat)*50000.0); //difference in lat 0.00005 degrees
        EFUData[dataindx][2] = (int16_t)((gps.location.lng() - GPSStartLon)*50000.0); //difference in lon 0.00005 degrees
        EFUData[dataindx][3] = (uint16_t)(gps.altitude.meters());  //Altitude in meters
        EFUData[dataindx][4] = (uint16_t)(TSEN_T);  //TSEN variable 1
        EFUData[dataindx][5] = (uint16_t)(TSEN_P);  //TSEN variable 2
        EFUData[dataindx][6] = (uint16_t)(TSEN_TP);  //TSEN variable 3
        EFUData[dataindx][7] = (uint16_t)(TSEN_TP>>16);  //TSEN variable 4
        EFUData[dataindx][8] = (uint8_t)((TPCB_val + 100) + (HeaterStatus << 8)); //Enum index and heater on/off bits 

    dataindx++;
    return dataindx;
}

bool checkForSerial()
{
  /* Checks for any new bytes on the various serial ports
   *  and processes them when necessary. 
   */
    Watchdog.reset(); 
//   if (DEBUG_SERIAL.available()) {
//    char DEBUG_Char = DEBUG_SERIAL.read();
   
//    if((DEBUG_Char == '\n') || (DEBUG_Char == '\r'))
//    {
//     parseCommand(DEBUG_Buff);
//     DEBUG_Buff = "";
//    } else
//    {
//     DEBUG_Buff += DEBUG_Char;
//    }   
//   }

  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
  }
   if (LoRa.parsePacket())
  {
      DEBUG_SERIAL.println("LoRa Waiting");
      SerComRX();
      DEBUG_SERIAL.println("Done with SerComRX");
  }
  return false;
}

bool updateRTCfromGPS()
{
    if ((gps.time.age() < 1500) && (gps.satellites.value() > 4))  //we have a GPS time fix within the last 1.5s
    {
        if(timeStatus() != timeSet) //if the time is not set, set it
        {
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            DEBUG_SERIAL.println("Setting RTC to GPS time "); 
            return true;
        }

        if (abs(minute() * 60 + second() - (gps.time.minute()*60 +gps.time.second())) > 2) //if the clock is more than 1 second off
        {
            DEBUG_SERIAL.printf("Updating RTC to GPS time from %d:%d:%d to %u:%u:%u\n", hour(), minute(), second(), gps.time.hour(), gps.time.minute(), gps.time.second());
            DEBUG_SERIAL.println((gps.time.hour()*3600 + gps.time.minute()*60 + gps.time.second()) - (hour()*3600 + minute()*60 + second()));
            setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
            return true;
        }

        return false;

    }
    return false;

}

int setHeater()
{
  /* Thermostat for both heaters, bases on global variables
   *  Heater1Setpoint, Heater2Setpoint and DeadBand
   *  Returns true if either heater is on
   */

  
   
  if (TBatt_val > -999) //if battery temperautre is working, control based on that
  {
    if(TBatt_val < HeaterSetpoint)
    {
        digitalWrite(BATT_HEATER, HIGH);
        HeaterStatus = true;
    }

    if(BATT_HEATER > (HeaterSetpoint + DeadBand))
    {
        digitalWrite(BATT_HEATER, LOW);
        HeaterStatus = false;
    }
    return HeaterStatus;
  }

  else if (TPCB_val > -999) //if PCB temperature is working, control based on that
  {
    if(TPCB_val < HeaterSetpoint)
    {
        digitalWrite(BATT_HEATER, HIGH);
        HeaterStatus = true;
    }

    if(BATT_HEATER > (HeaterSetpoint + DeadBand))
    {
        digitalWrite(BATT_HEATER, LOW);
        HeaterStatus = false;
    }
    return HeaterStatus;
  }

  else //if both thermistors fail go to 50% duty cycle
  {
      digitalWrite(BATT_HEATER, !digitalRead(BATT_HEATER)); 
  }
return HeaterStatus;
  
}