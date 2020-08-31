
#include "EPICLibrary.h"

//EPICLibrary::EPICLibrary(){
//}

void EPICLibrary::Setup(){

//Comms ports
    Serial.begin(9600);
    GPSSerial.begin(9600);
    TSENSerial.begin(9600);

//TSEN

    pinMode(TSEN_EN, OUTPUT);
    TSENOn();

//LORA
    pinMode(RF_CS, OUTPUT);
    pinMode(RF_RESET, OUTPUT); 

    digitalWrite(RF_CS, HIGH);
    delay(100);

    ResetLoRa();

//Solar
    pinMode(SOLAR_SHUTDOWN,OUTPUT);
    pinMode(BATT_HEATER, OUTPUT);
    digitalWrite(SOLAR_SHUTDOWN,LOW);
    digitalWrite(BATT_HEATER,LOW);

//GPS    
    pinMode(GPS_RESET, OUTPUT);
   //pinMode(GPS_EXTINT, OUTPUT);
    
//Analog voltage monitoring   
    analogReference(EXTERNAL);
}


void EPICLibrary::SolarOff(){
  digitalWrite(SOLAR_SHUTDOWN,LOW); 
  //SolarChargeState = 0;
}

void EPICLibrary::SolarOn(){
  digitalWrite(SOLAR_SHUTDOWN,HIGH); 
  //SolarChargeState = 1; 
}

void EPICLibrary::HeaterOn(){
  digitalWrite(BATT_HEATER,HIGH); 
  //BattHeaterState = 1;
}

void EPICLibrary::HeaterOff(){
  digitalWrite(BATT_HEATER,LOW); 
  //BattHeaterState = 0;
}

void EPICLibrary::TSENOn(){
  digitalWrite(TSEN_EN,HIGH); 
}

void EPICLibrary::TSENOff(){
  digitalWrite(TSEN_EN,LOW);
}

void EPICLibrary::ResetLoRa(){

    digitalWrite(RF_RESET, LOW);      
    delay(10);                         
    digitalWrite(RF_RESET, HIGH);      
    delay(10);

}

void EPICLibrary::ListenForTSEN(){

    char TSEN_Buff[19];   
    char header = '#';
    char messstart;
    char Tstr[3];
    char TPstr[6];
    char Pstr[6];
    char * pEnd;


    if(TSENSerial.available()){

        TSENSerial.readBytes(TSEN_Buff,19);
        sscanf(TSEN_Buff, "%1c %s %s %s", &messstart, Tstr, TPstr, Pstr);

        if(strcmp (&messstart,&header) != 0){

            TSEN_T = strtoul(Tstr, &pEnd, 16);
            TSEN_TP = strtoul(TPstr, &pEnd, 16);
            TSEN_P = strtoul(Pstr, &pEnd, 16);
        
        }

        else{
            Serial.print("bad TSEN start character = ");
            Serial.println(messstart);
            TSEN_T = -999;
            TSEN_TP = -999;
            TSEN_P = -999;

        }

    }

}

void EPICLibrary::RequestTSEN(){

  //Serial.println("send TSEN request");
  TSENSerial.print("*01A?");
  TSENSerial.write('\r');
  TSENSerial.flush();

}


float EPICLibrary::GetAnalogVoltage(uint8_t channel){

  float Vout;
  uint16_t val;

  switch(channel){

    case TEENSY:
      val = analogRead(TEENSY_V); 
      Vout = float(val)*(3.3/1023.0)*((4700.0+4700.0)/4700.0); 
      break;
    case BATT:
      val = analogRead(BATT_V);    
      Vout = float(val)*(3.3/1023.0)*((4700.0+4700.0)/4700.0);
      break;
    case TSEN:
      val = analogRead(DCDC_V);
      Vout = float(val)*(3.3/1023.0)*((10000.0+4700.0)/4700.0);
      break;
    default:
      Vout = -9999.9;
      break;

  
  return Vout;
  }
  return Vout;
}


//GPS Functions

void EPICLibrary::GPSreset(){
  digitalWrite(GPS_RESET,LOW);
  delay(50);
  digitalWrite(GPS_RESET,HIGH);
  //configureUblox(settingsArrayPointer);
}


void EPICLibrary::configureUblox(byte *settingsArrayPointer) {
    byte gpsSetSuccess = 0;
    SerialUSB.println("Configuring u-Blox GPS initial state...");
    
    //Generate the configuration string for Navigation Mode
    byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    calcChecksum(&setNav[2], sizeof(setNav) - 4);
    
    //Generate the configuration string for Data Rate
    byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
    calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);
    
    //Generate the configuration string for Baud Rate
    byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);
    
    byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
    byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
    byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
    byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
    byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
    
    delay(2500);
    
    while(gpsSetSuccess < 3)
    {
        SerialUSB.print("Setting Navigation Mode... ");
        sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
        gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
        if (gpsSetSuccess == 5) {
            gpsSetSuccess -= 4;
            setBaud(settingsArrayPointer[4]);
            delay(1500);
            byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
            sendUBX(lowerPortRate, sizeof(lowerPortRate));
            GPSSerial.begin(9600);
            delay(2000);
        }
        if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
        if (gpsSetSuccess == 10) gpsStatus[0] = true;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("Navigation mode configuration failed.");
    gpsSetSuccess = 0;
    while(gpsSetSuccess < 3) {
        SerialUSB.print("Setting Data Update Rate... ");
        sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
        gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function
        if (gpsSetSuccess == 10) gpsStatus[1] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("Data update mode configuration failed.");
    gpsSetSuccess = 0;
    
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
        SerialUSB.print("Deactivating NMEA GLL Messages ");
        sendUBX(setGLL, sizeof(setGLL));
        gpsSetSuccess += getUBX_ACK(&setGLL[2]);
        if (gpsSetSuccess == 10) gpsStatus[2] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GLL Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
        SerialUSB.print("Deactivating NMEA GSA Messages ");
        sendUBX(setGSA, sizeof(setGSA));
        gpsSetSuccess += getUBX_ACK(&setGSA[2]);
        if (gpsSetSuccess == 10) gpsStatus[3] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GSA Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
        SerialUSB.print("Deactivating NMEA GSV Messages ");
        sendUBX(setGSV, sizeof(setGSV));
        gpsSetSuccess += getUBX_ACK(&setGSV[2]);
        if (gpsSetSuccess == 10) gpsStatus[4] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA GSV Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
        SerialUSB.print("Deactivating NMEA RMC Messages ");
        sendUBX(setRMC, sizeof(setRMC));
        gpsSetSuccess += getUBX_ACK(&setRMC[2]);
        if (gpsSetSuccess == 10) gpsStatus[5] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA RMC Message Deactivation Failed!");
    gpsSetSuccess = 0;
    
    while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
        SerialUSB.print("Deactivating NMEA VTG Messages ");
        sendUBX(setVTG, sizeof(setVTG));
        gpsSetSuccess += getUBX_ACK(&setVTG[2]);
        if (gpsSetSuccess == 10) gpsStatus[6] = true;
        if ((gpsSetSuccess == 5) | (gpsSetSuccess == 6)) gpsSetSuccess -= 4;
    }
    if (gpsSetSuccess == 3) SerialUSB.println("NMEA VTG Message Deactivation Failed!");
    
    gpsSetSuccess = 0;
    if (settingsArrayPointer[4] != 0x25) {
        SerialUSB.print("Setting Port Baud Rate... ");
        sendUBX(&setPortRate[0], sizeof(setPortRate));
        setBaud(settingsArrayPointer[4]);
        SerialUSB.println("Success!");
        delay(500);
    }
}


void EPICLibrary::calcChecksum(byte *checksumPayload, byte payloadSize) {
    byte CK_A = 0, CK_B = 0;
    for (int i = 0; i < payloadSize ;i++) {
        CK_A = CK_A + *checksumPayload;
        CK_B = CK_B + CK_A;
        checksumPayload++;
    }
    *checksumPayload = CK_A;
    checksumPayload++;
    *checksumPayload = CK_B;
}

void EPICLibrary::sendUBX(byte *UBXmsg, byte msgLength) {
    for(int i = 0; i < msgLength; i++) {
        GPSSerial.write(UBXmsg[i]);
        GPSSerial.flush();
    }
    GPSSerial.println();
    GPSSerial.flush();
}


byte EPICLibrary::getUBX_ACK(byte *msgID) {
    byte CK_A = 0, CK_B = 0;
    byte incoming_char;
    //boolean headerReceived = false;
    unsigned long ackWait = millis();
    byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int i = 0;
    while (1) {
        if (GPSSerial.available()) {
            incoming_char = GPSSerial.read();
            if (incoming_char == ackPacket[i]) {
                i++;
            }
            else if (i > 2) {
                ackPacket[i] = incoming_char;
                i++;
            }
        }
        if (i > 9) break;
        if ((millis() - ackWait) > 1500) {
            SerialUSB.println("ACK Timeout");
            return 5;
        }
        if (i == 4 && ackPacket[3] == 0x00) {
            SerialUSB.println("NAK Received");
            return 1;
        }
    }
    
    for (i = 2; i < 8 ;i++) {
        CK_A = CK_A + ackPacket[i];
        CK_B = CK_B + CK_A;
    }
    if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
        //    Serial.println("Success!");
        //    Serial.print("ACK Received! ");
        //    printHex(ackPacket, sizeof(ackPacket));
        return 10;
    }
    else {
        //    Serial.print("ACK Checksum Failure: ");
        //    printHex(ackPacket, sizeof(ackPacket));
        //    delay(1000);
        return 1;
    }
}


void EPICLibrary::setBaud(byte baudSetting) {
    if (baudSetting == 0x12) GPSSerial.begin(4800);
    if (baudSetting == 0x4B) GPSSerial.begin(19200);
    if (baudSetting == 0x96) GPSSerial.begin(38400);
    if (baudSetting == 0xE1) GPSSerial.begin(57600);
    if (baudSetting == 0xC2) GPSSerial.begin(115200);
    if (baudSetting == 0x84) GPSSerial.begin(230400);
}

