#include <Adafruit_ADS1X15.h>
#include <Servo.h>

Servo motor;
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");

  motor.attach(A3);


  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    // while (1);
  }
}

void loop(void)
{

  // motor.writeMicroseconds(1500);
  int16_t ads_shunt_dif;
  double current, volts;

  ads_shunt_dif = ads.readADC_Differential_0_1();

  volts = ads.computeVolts(ads_shunt_dif);
  current = volts * 50/0.075;

  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts*1000); Serial.print("mV"), Serial.print(current); Serial.println("A");;
}
