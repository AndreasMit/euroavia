#include <Servo.h>
#include <ADS1X15.h>

#define CONTROL_ENABLE_THREASHOLD 1700
#define TARGET_FREQUENCY 250

#define TARGET 25
#define CTRL_CENTER_U 0.85


Servo motor;
ADS1015 ADS(0x48);

/*  Interrupt throttle reading variables  */
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
volatile bool controlEnabled = false;
volatile unsigned long controlPulseStart = micros();
// Pins Connected -------------------------------
const uint8_t motor_esc_pin = A1;     // Reading PWM Throttle 
const uint8_t RECEIVE_PIN = 2;
const uint8_t CONTROL_PIN = 3;
// Variables ------------------------------------
// Variables to store pulse width and throttle position
unsigned long pulse_width;
unsigned long throttle_value;
float throttle_value_01;
bool first_time_control_flag = true;
float int_sum = 0;
float error = 0;
float prev_error = 0;
float dt = 0;
float freq = 1;


// Current Sensor
int16_t current_adc_reading = 0;
float current_value = 0;
float current_value_real = 0;
float current_offset = 0;
float f;

unsigned long time_now = 0;
unsigned long time_before = 0;
bool first_run_flag = true;

#define ARRAY_SIZE 10
float last_c_measured[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Setup ----------------------------------------

void setup(){

  // Serial Configuration
  Serial.begin(115200); delay(1000);
  Serial.println("Starting Setup...");

  // Motor Configuration
  motor.attach(motor_esc_pin);
  motor.writeMicroseconds(1000);
  delay(2000);

  // Setup Built in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise ADS
  Wire.begin();
  if (!ADS.begin()){
    Serial.println("ADS1015 not connected. Please verify connections and try again.");
    while (1);
  }
  ADS.setDataRate(7);
  ADS.setMode(0);
  ADS.setGain(1);
  Serial.print("Data Rate : "); Serial.println(ADS.getDataRate());
  f = ADS.toVoltage(1);  // Voltage factor

  // Current Sensor Configuration
  current_offset = calculateCurrentOffset(200);
  Serial.println("Begging Data Collection in 2 Seconds...");
  delay(2000);

  /*  Attaching interrupt */
  pinMode(RECEIVE_PIN, INPUT_PULLUP);
  pinMode(CONTROL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RECEIVE_PIN), fastPulseIn, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CONTROL_PIN), controlChangeEvent, CHANGE);
}

// Loop -----------------------------------------

void loop() {

    // Time
    time_now = millis();


    // READING CURRENT SENSOR ------------------------
    current_adc_reading = ADS.readADC_Differential_0_1(); //Raw data reading // 0 -> 4095
    current_value = ADS.toVoltage(current_adc_reading) / 0.04 - current_offset; //0.04V/A -->[A]
    current_value_real = current_value;
    current_value = updateAndAverage(last_c_measured, current_value);

    // SETTING PWM THROTTLE --------------------------
    // throttle_value = interpolate(timestamp, percentage, arraySize, millis());  // 0 -> 100%
    // throttle_value = int(float(analogRead(A3))*100/498);
    // throttle_value = int(float(throttle_value)/100*1000 + 1000); // 1000 -> 2000
    // unsigned int pulse_width = pulseIn(A0, HIGH);
    if (newPulseDurationAvailable) {
      pulse_width = pulseInTimeEnd - pulseInTimeBegin;
      throttle_value = map(pulse_width, 1000, 2000, 1000, 2000);
    }


  // CONTROL LOGIC -------------------------------

  if (controlEnabled){
    // Turn on the builtin LED
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

    throttle_value = constrain(throttle_value, 1000, 2000);
    throttle_value_01 = float(throttle_value - 1000)/1000;
    float uc = 0;
    float dt = 1/freq;  // [s]

    error = TARGET - current_value;
    float de_dt = (error - prev_error)/dt;
    int_sum += error*dt;


    if (current_value > TARGET || (current_value < TARGET && throttle_value_01 > 0.7)){
      if (first_time_control_flag){
        int_sum = 0;
        first_time_control_flag = false;
      }
      uc = CTRL_CENTER_U + 0.008*error + 0.01*int_sum + 0*de_dt;
      
    }
    else {
      first_time_control_flag = true;
      uc = throttle_value_01;
    }

    if (uc > throttle_value_01){
      first_time_control_flag = true;
      uc = throttle_value_01;
    }

    uc = constrain(uc, 0, 1);

    throttle_value_01 = uc;
    throttle_value = constrain(int(throttle_value_01 * 1000 + 1000), 1000, 2000);
    
  }
  else {
    // Turn off the builtin LED
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
  }


  // ---------------------------------------------

    motor.writeMicroseconds(throttle_value);

    freq = 1000/(time_now - time_before);
    // PRINTING VALUES --------------------------------
    printDebugStatements();

    // Keeping Frequency Constant
    while(1000/(millis() - time_now) > TARGET_FREQUENCY ){
      // Do nothing
      ;
    }

    time_before = time_now;
}




void printDebugStatements(){
  if (first_run_flag) {
    Serial.println("Time[ms], Throttle[1000 -> 2000], Current[A], Frequency [Hz], Control Enabled");
    first_run_flag = false;
  }
  else {
    Serial.print(time_now); Serial.print(",");
    Serial.print(throttle_value); Serial.print(",");
    Serial.print(current_value); Serial.print(",");
    Serial.print(freq); Serial.print(",");
    Serial.print(controlEnabled); Serial.print(",");
    Serial.print(current_value_real);
    Serial.println();
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

void controlChangeEvent() {
  if (digitalRead(CONTROL_PIN) == HIGH) {
    controlPulseStart = micros();
  } else {
    	unsigned long control_pulse_width = micros() - controlPulseStart;
      // Serial.println(control_pulse_width);
      if (control_pulse_width >= CONTROL_ENABLE_THREASHOLD)
        controlEnabled = true;
      else
        controlEnabled = false;
  }

}


void fastPulseIn() {
  // Serial.println("interrupt runninggggggggggggggggg");
  if (digitalRead(RECEIVE_PIN) == HIGH) {
    pulseInTimeBegin = micros();
    newPulseDurationAvailable = false;
  }
  else {
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}



float updateAndAverage(float last_u[], float newNumber) {
  // Shift elements to the left to remove the last element
  for (int i = 0; i < ARRAY_SIZE - 1; i++) {
    last_u[i] = last_u[i + 1];
  }
  // Add the new number at the end of the array
  last_u[ARRAY_SIZE - 1] = newNumber;

  // Calculate the sum of the elements
  float sum = 0;
  for (int i = 0; i < ARRAY_SIZE; i++) {
    sum += last_u[i];
  }
  
  // Calculate the average
  float average = sum / ARRAY_SIZE;
  
  return average;
}