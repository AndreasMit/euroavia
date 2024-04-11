#include <Servo.h>
#include <ADS1X15.h>

Servo motor;
ADS1115 ADS(0x48);

/*  Interrupt throttle reading variables  */
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;


// Pins Connected -------------------------------
const uint8_t motor_esc_pin = A1;     // Reading PWM Throttle 
const uint8_t RECEIVE_PIN = 2;
// Variables ------------------------------------
// Variables to store pulse width and throttle position
unsigned int pulse_width;
unsigned int throttle_value;

// Current Sensor
int16_t current_adc_reading = 0;
float current_value = 0;
float current_offset = 0;
float f;

unsigned long time_now = 0;
unsigned long time_before = 0;
bool first_run_flag = true;

 // Sample time and percentage arrays
// long timestamp[] =  {0, 14000, 15000, 18000, 22000, 26000, 29000, 32000, 32100, 37000, 39000}; // in milliseconds
// int percentage[] =  {0, 0,     30,    30,    60,    60,    0,     0,     20,    20,    0};
// long timestamp[] =  {0, 14000, 15000, 18000, 20000, 23000, 25000, 30000, 32000}; // in milliseconds
// int percentage[] = {0, 0,     20,    18,    0,     0,     40,    40,    0};
// long timestamp[] = {0, 14000, 14010, 17000, 18000};
// int percentage[] = {0, 0,     60,    60,    0};
long timestamp[] = {0, 14000, 14100, 15000, 15100, 16000, 16100, 17000, 17100, 18000, 18100, 19000, 19100, 20000, 21100, 22000, 22100, 23000, 23100,  24000, 24100, 25000, 28000, 29000, 32000, 35000};
int percentage[] = {0, 0,     10,    10,    20,    20,    30,    30,    40,    40,    50,    50,    60,    60,    70,    70,    80,    80,    90,     90,    100,   100,   0,     0,     100,   0};
int arraySize = sizeof(timestamp) / sizeof(timestamp[0]);


// Setup ----------------------------------------

void setup(){

  // Serial Configuration
  Serial.begin(115200); delay(1000);
  Serial.println("Starting Setup...");

  // Motor Configuration
  motor.attach(motor_esc_pin);
  motor.writeMicroseconds(1000);
  delay(2000);

  // Initialise ADS
  Wire.begin();
  if (!ADS.begin()){
    Serial.println("ADS1015 not connected. Please verify connections and try again.");
    while (1);
  }
  ADS.setGain(1);
  Serial.print("Data Rate : "); Serial.println(ADS.getDataRate());
  f = ADS.toVoltage(1);  // Voltage factor

  // Current Sensor Configuration
  current_offset = calculateCurrentOffset(200);
  Serial.println("Begging Data Collection in 2 Seconds...");
  delay(2000);

  /*  Attaching interrupt */
  attachInterrupt(digitalPinToInterrupt(RECEIVE_PIN), fastPulseIn, CHANGE);

}

// Loop -----------------------------------------

void loop(){

    // Time
    time_now = millis();




    // SETTING PWM THROTTLE --------------------------
    // throttle_value = interpolate(timestamp, percentage, arraySize, millis());  // 0 -> 100%
    // throttle_value = int(float(analogRead(A3))*100/498);
    // throttle_value = int(float(throttle_value)/100*1000 + 1000); // 1000 -> 2000
    // unsigned int pulse_width = pulseIn(A0, HIGH);
    if (newPulseDurationAvailable)
      unsigned int pulse_width = pulseInTimeEnd - pulseInTimeBegin;
    throttle_value = map(pulse_width, 1000, 2000, 1000, 2000);

    Serial.print("Pulse width:"); Serial.println(pulse_width);
    Serial.print("Throttle value:"); Serial.println(throttle_value);
    // motor.writeMicroseconds(throttle_value);


    // READING CURRENT SENSOR ------------------------
    current_adc_reading = ADS.readADC_Differential_0_1(); //Raw data reading // 0 -> 4095
    current_value = ADS.toVoltage(current_adc_reading) / 0.04 - current_offset; //0.04V/A -->[A]





    // PRINTING VALUES --------------------------------
    // printDebugStatements();




    time_before = time_now;
    
    // Manipulate Delay Time

    // while(millis() - time_now < 2){
    //   // Do Nothing
    // }
}


void printDebugStatements(){
  if (first_run_flag){
    Serial.println("Time[ms], Throttle[1000 -> 2000], Current[A], Frequency [Hz]");
    first_run_flag = false;
  }
  else {
    Serial.print(time_now); Serial.print(",");
    Serial.print(throttle_value); Serial.print(",");
    Serial.print(current_value); Serial.print(",");
    Serial.print(1000/(time_now - time_before)); Serial.println();
  }
}




float calculateCurrentOffset(int numOfSamples){
  Serial.println("Calculating Current Offset...");

  float sum = 0;
  for(int i = 0; i < numOfSamples; i++){
    int16_t raw_adc = ADS.readADC_Differential_0_1();
    float voltage = ADS.toVoltage(raw_adc);
    sum += voltage / 0.04;;
    delay(10);
  }
  float currentOffset = sum / numOfSamples;

  Serial.print("Current Offset: "); Serial.println(currentOffset);
  return currentOffset;
}

// Function to interpolate linearly between given timestamps and percentage values
int interpolate(long timestamp[], int percentage[], int arraySize, int currentTime) {

  // Ensure current time is within the range of timestamps
  if (currentTime <= timestamp[0]) {
    return percentage[0];
  }
  if (currentTime >= timestamp[arraySize - 1]) {
    return percentage[arraySize - 1];
  }
  
  // Find the index of the first timestamp greater than or equal to current time
  int index = 0;
  while (index < arraySize && timestamp[index] < currentTime) {
    index++;
  }

  // Interpolate between the two closest timestamps
  float timeFraction = (float)(currentTime - timestamp[index - 1]) / (timestamp[index] - timestamp[index - 1]);
  int interpolatedPercentage = percentage[index - 1] + timeFraction * (percentage[index] - percentage[index - 1]);
  return interpolatedPercentage;
}


void fastPulseIn() {

  if (digitalRead(6) == HIGH)
    pulseInTimeBegin = micros()
  else {
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}