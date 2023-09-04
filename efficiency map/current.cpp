#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

void setup(void)
{
Serial.begin(9600);

ads.setGain(GAIN_SIXTEEN);    // 16x gain +/- 0.256V 1 bit = 0.125mV 0.0078125mV

ads.begin();
}

void loop(void)
{
int16_t results;

results = ads.readADC_Differential_0_1();

Serial.print(“Amps: “);

float amps = ((float)results * 256.0) / 32768.0;//100mv shunt
//amps = amps * 1.333; //uncomment for 75mv shunt
//amps = amps * 2; //uncomment for 50mv shunt

Serial.println(amps);

delay(5000);
}

