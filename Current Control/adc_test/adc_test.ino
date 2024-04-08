#include <ADS1X15.h>

ADS1115 ADS(0x48);

unsigned long time_now = 0;
unsigned long time_before = 0;

void setup() 
{
    Serial.begin(115200);
    Serial.println(__FILE__);
    Serial.print("ADS1X15_LIB_VERSION: ");
    Serial.println(ADS1X15_LIB_VERSION);

    Wire.begin();

    if (!ADS.begin())
    {
        Serial.println("ADS1115 not connected. Please verify connections and try again.");
        while (1);
    }
    ADS.setGain(0);
}


void loop() 
{
    // Time
    time_now = millis();

    int16_t val_01 = ADS.readADC_Differential_0_1();  
    float volts_01 = ADS.toVoltage(val_01); 

    Serial.print("\tRaw Val: "); Serial.print(val_01); Serial.print("\t Voltage: "); Serial.print(volts_01, 3);
    Serial.print("\t Frequency: "); Serial.print(1000/(time_now - time_before)); Serial.print(" Hz");
    Serial.println();


    time_before = time_now;
    delay(10);
}


//  -- END OF FILE --
