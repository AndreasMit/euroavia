#include <Servo.h>
#include <ADS1X15.h>

#define TARGET_FREQUENCY 250

#define TARGET_LOW 20          // ControlEnabledFlag = 1
#define CTRL_CENTER_U_LOW 0.75
#define TARGET_HIGH 25         // ControlEnabledFlag = 2
#define CTRL_CENTER_U_HIGH 0.8
float target = 0;

#define KP_GAIN 0.008
#define KI_GAIN 0.055
#define KD_GAIN 0.0

Servo motor;
ADS1015 ADS(0x48);

/*  Interrupt throttle reading variables  */
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
volatile int control_switch_pos = 0;
volatile unsigned long controlPulseStart = micros();
// Pins Connected -------------------------------
const uint8_t motor_esc_pin = A1;     // Writing PWM Throttle 
const uint8_t RECEIVE_PIN = 2;        // Reading PWM Throttle
const uint8_t CONTROL_PIN = 3;        // Reading Switch Status
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
float current_value_raw = 0;
float current_offset = 0;

// Current Moving Average Filter
#define MA_WINDOW_SIZE 10
int buffer_index = 0;
float last_c_measured[MA_WINDOW_SIZE] = {0};

// Time Variables
unsigned long time_now = 0;
unsigned long time_before = 0;
bool first_run_flag = true;


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
  current_value_raw = current_value;
  current_value = updateAndAverage(last_c_measured, &buffer_index, current_value);



  // READING PWM THROTTLE --------------------------
  // unsigned int pulse_width = pulseIn(A0, HIGH);
  if (newPulseDurationAvailable) {
    pulse_width = pulseInTimeEnd - pulseInTimeBegin;
    throttle_value = map(pulse_width, 1000, 2000, 1000, 2000);
  }

  throttle_value = constrain(throttle_value, 1000, 2000);


  // CONTROL LOGIC -------------------------------------------------------------------

  if (control_switch_pos>0){
    // Turn on the builtin LED
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

    // Map to 0 -> 1
    throttle_value_01 = float(throttle_value - 1000)/1000;
    float uc = 0;
    float dt = 1/freq;  // [s]

    // Define Error - dE/dt - ΣeΔt
    if (control_switch_pos == 1)
      error = TARGET_LOW - current_value;
      target = TARGET_LOW;
    else if (control_switch_pos == 2)
      error = TARGET_HIGH - current_value;
      target = TARGET_HIGH;

    float de_dt = (error - prev_error)/dt;
    int_sum += error*dt;
    if (first_time_control_flag){int_sum = 0;}


    if (current_value > target || (current_value < target && throttle_value_01 > 0.6)){
      if (first_time_control_flag){
        int_sum = 0;
        first_time_control_flag = false;
      }
      if (control_switch_pos == 1)
        uc = CTRL_CENTER_U_LOW  + KP_GAIN*error + KI_GAIN*int_sum + KD_GAIN*de_dt;
      else if (control_switch_pos == 2)
        uc = CTRL_CENTER_U_HIGH + KP_GAIN*error + KI_GAIN*int_sum + KD_GAIN*de_dt;
    }
    else {
      first_time_control_flag = true;
      uc = throttle_value_01;
    }

    if (uc > throttle_value_01){
      // first_time_control_flag = true;
      uc = throttle_value_01;
    }
    if (current_value < target * 0.5){
      first_time_control_flag = true;
    }

    uc = constrain(uc, 0, 1);

    throttle_value = constrain(int(uc * 1000 + 1000), 1000, 2000);
    
  }
  else {
    // Turn off the builtin LED
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
  }


  // -----------------------------------------------------------------------------

  // Write Motor Speed to the ESC
  motor.writeMicroseconds(throttle_value);

  // Frequency Calculation
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
    Serial.println("Time[ms], Throttle[1000 -> 2000], Current_MA[A], Frequency [Hz], Control Enabled Flag, Current Raw[A], I Term");
    first_run_flag = false;
  }
  else {
    Serial.print(time_now); Serial.print(",");
    Serial.print(throttle_value); Serial.print(",");
    Serial.print(current_value); Serial.print(",");
    Serial.print(freq); Serial.print(",");
    Serial.print(control_switch_pos); Serial.print(",");
    Serial.print(current_value_raw); Serial.print(",");
    Serial.print(int_sum * KI_GAIN);
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


void controlChangeEvent() {
  if (digitalRead(CONTROL_PIN) == HIGH) {
    controlPulseStart = micros();
  } else {
    unsigned long control_pulse_width = micros() - controlPulseStart;
    if (control_pulse_width >= 1700) {
      control_switch_pos = 2;
    }
    else if (control_pulse_width < 1700 && control_pulse_width > 1400) {
      control_switch_pos = 1;
    }
    else {
      control_switch_pos = 0;
    }
  }

}


void fastPulseIn() {
  if (digitalRead(RECEIVE_PIN) == HIGH) {
    pulseInTimeBegin = micros();
    newPulseDurationAvailable = false;
  }
  else {
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}


float updateAndAverage(float last_c[], int *buffer_index, float newNumber) {
  // Add the new number at the current buffer index
  last_c[*buffer_index] = newNumber;
  // Shift the buffer index
  *buffer_index = (*buffer_index + 1) % MA_WINDOW_SIZE;

  // Calculate the sum of the elements
  float sum = 0;
  for (int i = 0; i < MA_WINDOW_SIZE; i++) {
    sum += last_c[i];
  }
  
  // Calculate the average
  float average = sum / MA_WINDOW_SIZE;
  
  return average;
}
