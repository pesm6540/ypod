/*******************************************************************************
 * @project HAQ Lab YPOD
 *
 * @file    YPOD_V3.1.2.cpp
 * @author  Percy Smith
 * @brief   Should Fix the YPOD library problem !! I STG THIS TIME !!!!!
 * 
 * @date    March 19, 2024
 * @version V3.1.2
 * @log     Hopefully resolves version issue with libraries        
 * ******************************************************************************/
 #include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include <SdFat.h>  
#include <arduino.h> in '#include <Arduino.h>' in OneWire.cpp and DallasTemperature.cpp
#include <SFE_BMP180.h>
#include <RTC_DS3231.h>
#include <mcp3424.h>
#include <Adafruit_ADS1015.h>
// #include "hardware_logging.h"

#define PMS_Sensing 0
#define SerialStream 1
#define GPS_used 0
#define MetStation 0
#define QuadStat  0 //auxillary 4-stat array, uses 2 MCP3424s

#define SD_Card_Write_Time 5 //"SD_Card_Write_Time" gives a delay 1 sec or 2 secs less than the given value.
/*****UPOD model indicator. Modify the 5th and 6th character to denote which YPOD you are using.*****/
String ypodID = "YPODA1";
String version = "Y3.2.0";
String fileName;

word setFileName = 1;

const word chipSelect = 4;
String delimiter = ",";

#if GPS_used
SoftwareSerial GPS(8, 9);
#endif

//UART Communication for PMS5003
#if PMS_Sensing
SoftwareSerial pmsSerial(2,3);//(RX,TX)
#endif

RTC_DS3231 RTC;
SdFat sd;
SdFile file;

//On-Board ADC instances and variables
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2(B1001001);
int ADC1;

#if QuadStat
//Quadstat ADC instances and variables
mcp3424 alpha_one;
mcp3424 alpha_two;
float alpha_value;
#endif

//BMP Temp and Pressure Variables
SFE_BMP180 BMP;

//SHT2 Temp and Humidity Variables
unsigned int temperature_board, humidity_board;

#if GPS_used
//Interrupt for GPS
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
uint32_t timer = millis();
String gps_data;
bool gps_available = false;
#endif

#if MetStation
//Wind direction sensor(Potentiometer) on analog pin 0
const byte WDIR = A0;
//Wind speed variables
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
#endif

#if MetStation
// Function called anemometer interrupt (2 ticks per rotation), attached to input D4
void wspeedIRQ()  
{
  if (millis() - lastWindIRQ > 10)  
  { // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}
#endif

void setup() 
{ 
  #if SerialStream
  Serial.begin(9600);
  #endif
  #if GPS_used
  GPS.begin(4800);
  #endif

  
  //PMS5003 sensor baud rate is 9600
  #if PMS_Sensing
  pmsSerial.begin(9600);
  #endif
  
  Wire.begin();
  SPI.begin();
  RTC.begin();
  BMP.begin();
  ads1.begin();
  ads2.begin();
  
  pinMode(chipSelect, OUTPUT);
  pinMode(10, OUTPUT);

  DateTime now = RTC.now();
  fileName = ypodID + "_" + String(now.year()) + "_" + String(now.month()) + "_" + String(now.day()) + ".txt";
  char fileNameArray[fileName.length()+1];
  fileName.toCharArray(fileNameArray, sizeof(fileNameArray)); //Well damn, that function is nice.

  while (!sd.begin(chipSelect)) 
  {
    #if SerialStream
    Serial.println("insert sd card to begin");
    #endif
    sd.begin(chipSelect);
  }

  digitalWrite(10, LOW);

  #if QuadStat
  alpha_one.GetAddress('G', 'F'); //user defined address for the alphasense pstat array (4-stat)
  alpha_two.GetAddress('H', 'H') ;
  #endif
  
  #if MetStation
  attachInterrupt(4, wspeedIRQ, FALLING); //anemometer reed switch on pin 7--> interrupt# 4
  #endif
  
  #if GPS_used
    useInterrupt(true);
    //GPS.println("$PTNLSNM,0021,02"); //Address for ZDA output (UTC and date, no coordinates).
    GPS.println("$PTNLSNM,0001,02"); //Address for GGA output (coordinates, no date or time). 
    //GPS.println("$PTNLSNM,0101,02");//Address for RMC output (UTC and coordinates, no date!!).
      
    delay(500);
    //GPS.println("$PTNLSNM,0021,02"); //Address for ZDA output (UTC and date, no coordinates).
    GPS.println("$PTNLSNM,0001,02");//Address for GGA output (coordinates, no time or date). 
    //GPS.println("$PTNLSNM,0101,02");//Address for RMC output (UTC and coordinates, no date!!).
  #endif
}



#if GPS_used
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  if (GPS.available()) {
    char c = GPS.read();
    //    Serial.print(c);
    if (!gps_available) {
      gps_data += c;
      if (c == '\n') gps_available = true;
    }
  }
}
#endif

#if PMS_Sensing
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
   
  static int SD_Write_Count=0;
  String particle_10 = "P10";
  String particle_25 = "P25";
  String particle_100 = "P100";
  
  String data;
  
  
  String combined_data;
    
  char fileNameArray[fileName.length()+1];
  fileName.toCharArray(fileNameArray, sizeof(fileNameArray));
  DateTime now = RTC.now();
  
   
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

   
  data += ypodID + delimiter + String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + delimiter + String(now.hour()) + ":" + String(now.minute()) + ":" +String(now.second()) + delimiter + T + delimiter + P + delimiter + temperature_SHT + delimiter + humidity_SHT + delimiter + String(getS300CO2()) + delimiter;
  //data += ypodID + delimiter + String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + delimiter + String(now.hour()) + ":" + String(now.minute()) + ":" +String(now.second()) + temperature_SHT + delimiter + humidity_SHT + delimiter + String(getS300CO2()) + delimiter;


  #if PMS_Sensing
      if (readPMSdata(&pmsSerial)) 
      {
        data +=  PMS_data.pm10_env + delimiter + PMS_data.pm25_env + delimiter;// + PMS_data.pm100_env + delimiter;
        data += PMS_data.pm100_env + delimiter;
        ++SD_Write_Count;
      }
  #else 
        ++SD_Write_Count;
  #endif

  
  #if MetStation
  data += String(get_wind_speed()) + delimiter + String(analogRead(A0)) + delimiter;
  #else if
  data += delimiter + delimiter;
  #endif
  
        
    
  //Read ADCs on-board and on-Quadstat
  for (int i = 1; i <= 16; i++) 
  {
    if (i <= 4) data += 
    #if QuadStat
      alpha_one.GetValue(i) + 
    #endif
    delimiter;
    
    else if (i <= 8) data += 
    #if QuadStat
      alpha_two.GetValue(i - 4) + 
    #endif 
    delimiter;
    else if (i <= 12) data += ads1.readADC_SingleEnded(i - 9) + delimiter;
    else if (i <= 16) data += ads2.readADC_SingleEnded(i - 13) + delimiter;
  }



  #if GPS_used
  //Get GPS data
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  if (millis() - timer > 2000) {
    timer = millis();
    if (gps_available) {
      data += gps_data;
      gps_data = "";
      gps_available = false;
    }
  }
  data.replace("\n","");
  data.replace("\r",""); // GPS adds carriage return, newline. We need to strip it from data before printing to file.
  #endif

     
  while(!sd.begin(chipSelect))
  {
      #if SerialStream
      Serial.println("error in loop");
      #endif
      sd.begin(chipSelect);
  }
  digitalWrite(10, LOW);

  if (sd.begin(chipSelect))   // very important - reinitialize SD card on the SPI bus
  {
    #if QuadStat
            
        digitalWrite(10, HIGH);
        file.open(fileNameArray, O_CREAT | O_APPEND | O_WRITE);
        file.println(data);
        #if SerialStream
        Serial.println(data);
        #endif
        file.close();
        delay(250);
        digitalWrite(10,LOW);

    #else
    
        if(SD_Write_Count == SD_Card_Write_Time)
        {
        digitalWrite(10, HIGH);
        file.open(fileNameArray, O_CREAT | O_APPEND | O_WRITE);
        file.println(data);
        
        #if SerialStream
        Serial.println(data);
        #endif
        
        file.close();
        delay(250);
        digitalWrite(10,LOW);
        
        SD_Write_Count=0;
       }
    #endif   
  }

  //QuadStat takes ~11 seconds to sample. No need for delay in main loop
 
 }

/********PMS5003 Datalogging function********/
#if PMS_Sensing
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

float getS300CO2()  
{
  int i = 1;
  long reading;
  //float CO2val;
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

  //Shift Calculation to Atheros
  //    CO2val = reading / 4095.0 * 5000.0;
  //    CO2val = reading;
  return reading;
}

void wire_setup(int address, byte cmd, int from) 
{
  Wire.beginTransmission(address);
  Wire.write(cmd);
  Wire.endTransmission();
  Wire.requestFrom(address, from);
}

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

#if MetStation
//Returns the instataneous wind speed
  float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  return (windSpeed);
}
#endif
