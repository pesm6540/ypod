/*******************************************************************************
 * @project HAQ Lab Updated AQIQ Firmware
 *
 * @file    YPOD_V3.0.0.ino
 * @brief   Updated AQIQ Firmware 
 *
 * @author 	Percy Smith
 * @date 	  October 19, 2023
 ******************************************************************************/

//Before Verifying Script: Downgrade Adafruit ADS1X15 to Version 1.0, RTCLib 2.1.1

#include "YPOD_node.h"

#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <SdFat.h>  
#include <arduino.h> //in '#include <Arduino.h>' in OneWire.cpp and DallasTemperature.cpp
#include <SFE_BMP180.h>
#include <RTClib.h>
#include <mcp3424.h>
#include <Adafruit_ADS1015.h>

String ypodID = "YPODZ1";

RTC_DS3231 RTC;
SdFat sd;
SdFile file;
String fileName;

word setFileName = 1;
String dlm = ",";

//On-Board ADC instances and variables
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2(B1001001);
int ADC1;

//BMP Temp and Pressure Variables
SFE_BMP180 BMP;
//SHT2 Temp and Humidity Variables
unsigned int temperature_board, humidity_board;

#if PMS_ENABLED
  SoftwareSerial pmsSerial(PM_SERIAL_RX, PM_SERIAL_TX);
#endif

void setup()
{
  #if SERIAL_LOG_ENABLED
    Serial.begin(9600);
  #endif
  #if PMS_ENABLED
    pmsSerial.begin(9600);
  #endif
  #if QUAD_ENABLED
    //Quadstat ADC instances and variables
    mcp3424 alpha_one;
    mcp3424 alpha_two;
    float alpha_value;
  #endif

  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(PCB_LED_RED, OUTPUT);
  pinMode(PCB_LED_GREEN, OUTPUT);

  sd.begin(SD_CS_PIN);
  Wire.begin();
  SPI.begin();
  RTC.begin();
  BMP.begin();
  ads1.begin();
  ads2.begin();

  DateTime now = RTC.now();
  fileName = ypodID + "_" + String(now.year()) + "_" + String(now.month()) + "_" + String(now.day()) + ".txt";
  char fileNameArray[fileName.length()+1];
  fileName.toCharArray(fileNameArray, sizeof(fileNameArray)); //Well damn, that function is nice.

  while (!sd.begin(SD_CS_PIN)) 
  {
    #if SERIAL_LOG_ENABLED
      Serial.println("Cannot see SD Card");
    #endif
    sd.begin(SD_CS_PIN);
  }

  #if QUAD_ENABLED
    alpha_one.GetAddress('G', 'F'); //user defined address for the alphasense pstat array (4-stat)
    alpha_two.GetAddress('H', 'H') ;
  #endif
}

#if PMS_ENABLED
  struct pms5003data 
  {
    uint16_t framelen;
    uint16_t pm10_standard, pm25_standard, pm100_standard;
    uint16_t pm10_env, pm25_env, pm100_env;
    uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
    uint16_t unused;
    uint16_t checksum;
  };
  struct pms5003data PMS_data;
#endif

void loop()
{

  String data;
  data += ypodID + dlm;

  char fileNameArray[fileName.length()+1];
  fileName.toCharArray(fileNameArray, sizeof(fileNameArray));
  DateTime now = RTC.now();
  data +=  String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + dlm + String(now.hour()) + ":" + String(now.minute()) + ":" +String(now.second()) + dlm; 

  //Get SHT data
  const byte temp_command = B11100011;
  const byte hum_command = B11100101;
  temperature_board = read_wire(temp_command);
  humidity_board = read_wire(hum_command);
  float humidity_SHT = ((125 * (float)humidity_board) / (65536)) - 6.00;
  float temperature_SHT = ((175.72 * (float)temperature_board) / (65536)) - 46.85;

  //Get BMP data
  double T, P;
  char status;
  status = BMP.startTemperature();
  if (status != 0)  
  {
    //Serial.println(status);
    delay(status);
    status = BMP.getTemperature(T);
    status = BMP.startPressure(3);
    if (status != 0)  
    {
      delay(status);
      status = BMP.getPressure(P, T);
    }
    else  
    { 
      //if good temp; but can't compute P
      P = -99;
    }
  }
  else  
  { 
    //if bad temp; then can't compute temp or preGPSure
    T = -99;
    P = -99;
  }
  data += String(T) + dlm + String(P) + dlm + String(temperature_SHT) + dlm + String(humidity_SHT) + dlm;
  
  data += String(getS300CO2()) + dlm;

  #if PMS_ENABLED
    if (readPMSdata(&pmsSerial)) 
    {
      data +=  String(PMS_data.pm10_env) + dlm + String(PMS_data.pm25_env) + dlm + String(PMS_data.pm100_env) + dlm;
    }
  #endif

  #if MET_ENABLED
  #else if
    data += dlm + dlm;
  #endif

  //Read ADCs on-board and on-Quadstat
  for (int i = 1; i <= 16; i++) 
  {
    if (i <= 4) data += 
    #if QUAD_ENABLED
      alpha_one.GetValue(i) + 
    #endif
    dlm;
    
    else if (i <= 8) data += 
    #if QUAD_ENABLED
      alpha_two.GetValue(i - 4) + 
    #endif 
    dlm;
    else if (i <= 12) data += ads1.readADC_SingleEnded(i - 9) + dlm;
    else if (i <= 16) data += ads2.readADC_SingleEnded(i - 13) + dlm;
  }

  while(!sd.begin(SD_CS_PIN))
  {
      #if SERIAL_LOG_ENABLED
        Serial.println("Issues Finding SD in Loop");
      #endif
      sd.begin(SD_CS_PIN);
  }

  if (sd.begin(SD_CS_PIN))   // very important - reinitialize SD card on the SPI bus
  {
    file.open(fileNameArray, O_CREAT | O_APPEND | O_WRITE);
    file.println(data); 
    file.close();

    #if SERIAL_LOG_ENABLED
      Serial.println(data);
    #endif
   
    delay(250);
  }
  delay(2000);
}

/********PMS5003 Datalogging function********/
#if PMS_ENABLED
  boolean readPMSdata(Stream *s) 
  {  
    if (! s->available()) 
    {
      return false;
    }
    
    // Read a byte at a time until we get to the special '0x42' start-byte
    if (s->peek() != 0x42) 
    {
      s->read();
      return false;
    }
  
    // Now read all 32 bytes
    if (s->available() < 32) 
    {
        return false;
    }
      
    uint8_t buffer[32];    
    uint16_t sum = 0;
    s->readBytes(buffer, 32);
  
    // get checksum ready
    for (uint8_t i=0; i<30; i++) 
    {
      sum += buffer[i];
    }
    
    // The data comes in endian'd, this solves it so it works on all platforms
    uint16_t buffer_u16[15];
    for (uint8_t i=0; i<15; i++) 
    {
      buffer_u16[i] = buffer[2 + i*2 + 1];
      buffer_u16[i] += (buffer[2 + i*2] << 8);
    }
  
    // put it into a nice struct :)
    memcpy((void *)&PMS_data, (void *)buffer_u16, 30);
  
    if (sum != PMS_data.checksum) 
    {
      Serial.println("Checksum failure");
      return false;
    }
    // success!
    return true;
  }
  #endif
/********PMS5003 Datalogging function********/

/********S300CO2 Datalogging function********/
float getS300CO2()  
  {
    int i = 1;
    long reading;
    wire_setup(0x31, 0x52, 7);

    while (Wire.available())
    {
      byte val = Wire.read();
      if (i == 2)  
      {
        reading = val;
        reading = reading << 8;
      }
      if (i == 3)  
      {
        reading = reading | val;
      }
      i = i + 1;
    }
    return reading;
  }
/********S300CO2 Datalogging function********/

/********Wire Setup function********/
void wire_setup(int address, byte cmd, int from) 
  {
    Wire.beginTransmission(address);
    Wire.write(cmd);
    Wire.endTransmission();
    Wire.requestFrom(address, from);
  }
/********Wire Setup function********/

/********Read Wire Datalogging function********/
unsigned int read_wire(byte cmd) 
  {
    const int SHT2x_address = 64;
    const byte mask = B11111100;
    byte byte1, byte2, byte3;

    wire_setup(SHT2x_address, cmd, 3);

    byte1 = Wire.read();
    byte2 = Wire.read();
    byte3 = Wire.read();

    //HUM_byte1 shifted left by 1 byte, (|) bitwise inclusize OR operator
    return ( (byte1 << 8) | (byte2) & mask );
  }
/********Read Wire Datalogging function********/
