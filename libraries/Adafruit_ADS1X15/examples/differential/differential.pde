#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2(B1001001);  //ADDR to VDD
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

void setup(void)
{
  Serial.begin(9600);
//  Serial.println("Hello!");
//  
//  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
//  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   ads1.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
   ads2.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads1.begin();
  ads2.begin();
}

void loop(void)
{
  //ADS1
  int16_t thermistor1; //Differential output between 0 and 1
  int16_t thermsitor2;  //Differential output between 2 and 3
  //ADS2
  int16_t thermistor3; //Differential output between 0 and 1
  int16_t thermsitor4;  //Differential output between 2 and 3
  
  /* Be sure to update this value based on the IC and the gain settings! */
  //float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
  //float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
  float multiplier = 0.0625F; 
  thermistor1 = ads1.readADC_Differential_0_1();  
  thermsitor2 = ads1.readADC_Differential_2_3();
  thermistor3 = ads2.readADC_Differential_0_1();  
  thermsitor4 = ads2.readADC_Differential_2_3();
    
  Serial.print("Thermistor 1, ADC value: "); Serial.print(thermistor1); Serial.print("("); Serial.print(thermistor1 * multiplier); Serial.println("mV)");
  Serial.print("Thermistor 2, ADC value: "); Serial.print(thermsitor2); Serial.print("("); Serial.print(thermsitor2 * multiplier); Serial.println("mV)");
  Serial.print("Thermistor 3, ADC value: "); Serial.print(thermistor3); Serial.print("("); Serial.print(thermistor3 * multiplier); Serial.println("mV)");
  Serial.print("Thermistor 4, ADC value: "); Serial.print(thermsitor4); Serial.print("("); Serial.print(thermsitor4 * multiplier); Serial.println("mV)");

  delay(1000);
}
