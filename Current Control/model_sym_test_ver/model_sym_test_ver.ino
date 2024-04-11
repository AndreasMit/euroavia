#include <Servo.h>
#include <ADS1X15.h>

/* Current (A) value*/
#define TARGET 28

Servo motor;
ADS1015 ADS(0x48);

// Pins Connected -------------------------------
const uint8_t motor_esc_pin = A1; // Reading PWM Throttle 

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
long timestamp[] = {
  0,
  14000,
  14300,
  14310,
  14600,
  14610,
  14900,
  14910,
  15200,
  15210,
  15500,
  15510,
  15800,
  15810,
  16100,
  16110,
  16400,
  16410,
  16700,
  16710,
  17000,
  17010,
  18000,
  18010,
  19000,
  19010,
  19500,
  19510,
  20000,
  20010,
  21000,
  21010,
  21300,
  21310,
  21800,
  21810,
  22500,
  22510,
  22900,
  22910,
  23500,
  23510,
  24000,
  24010,
  24300,
  24310,
  28000,
  28010,
  29000,
  29010,
  29500,
  29510,
  30000,
  30010,
  30500,
  30510,
  32000
};
int percentage[] = {
  0,
  0,
  0,
  10,
  10,
  20,
  20,
  30,
  30,
  40,
  40,
  50,
  50,
  60,
  60,
  70,
  70,
  80,
  80,
  90,
  90,
  100,
  100,
  0,
  0,
  80,
  80,
  20,
  20,
  0,
  0,
  60,
  60,
  40,
  40,
  90,
  90,
  100,
  100,
  90,
  90,
  80,
  80,
  55,
  55,
  80,
  80,
  100,
  100,
  30,
  30,
  80,
  80,
  50,
  50,
  0,
  0
};
int arraySize = sizeof(timestamp) / sizeof(timestamp[0]);

bool first_time_control_flag = true;
int f_t_c_t = 0;
float error = 0;
float error_prev = 0;
float de_dt = 0;
float int_sum = 0;
unsigned long k = 0;
float uc = 0;

// Setup ----------------------------------------

void setup() {

  /*  Serial configuration begin  */
  Serial.begin(115200);
  Serial.println("Starting Setup...");
  /*  Serial configuration end  */


  /*  Motor configuration begin */
  motor.attach(motor_esc_pin);
  motor.writeMicroseconds(1000);
  /*  Motor configuration end */

  /*  Initialise ADS Begin */
  Wire.begin();
  if (!ADS.begin()) {
    Serial.println("ADS1015 not connected. Please verify connections and try again.");
    while (1);
  }
  // Sets data rate to **fastest**
  ADS.setDataRate(7);
  // Sets **continuous** mode
  ADS.setMode(0);
  ADS.setGain(1);
  f = ADS.toVoltage(1); // Voltage factor
  delay(500);
  Serial.print("ADS mode: ");
  Serial.println(ADS.getMode() ? "Single" : "Continuous");
  Serial.print("Data Rate : ");
  Serial.println(ADS.getDataRate());
  delay(500);
  /*  Initialise ADS End */

  // Current Sensor Configuration
  current_offset = calculateCurrentOffset(200);
  Serial.println("Beginning Data Collection in 2 Seconds...");
  delay(2000);
}

// Loop -----------------------------------------

void loop() {

  time_now = millis();
  float dt = (time_now - time_before) / 1000.0; // [s]

  // SETTING PWM THROTTLE --------------------------
  throttle_value = interpolate(timestamp, percentage, arraySize, millis()); // 0 -> 100%
  throttle_value = int(float(throttle_value) / 100); // 0 -> 1
  // throttle_value = int(float(throttle_value)/100*1000 + 1000); // 1000 -> 2000
  // throttle_value = int(float(analogRead(A3))*100/498);
  // unsigned int pulse_width = pulseIn(A0, HIGH);
  // throttle_value = map(pulse_width, 1000, 2000, 0, 1);

  // Control Logic Here
  error = float(TARGET) - current_value;
  de_dt = (error - error_prev) / dt;
  int_sum += error * dt;

  if (current_value > TARGET || (current_offset < TARGET && throttle_value > 0.7)) {
    if (first_time_control_flag) {
      int_sum = 0;
      first_time_control_flag = false;
      f_t_c_t = k;
    }
    uc = 0.85 + 0.01 * error + 0.1 * int_sum + 0.0003 * de_dt;
  } else {
    uc = throttle_value;
  }

  if (uc > throttle_value) {
    uc = throttle_value;
  }

  if (uc < 0.5 || (k - f_t_c_t) < 40) {
    first_time_control_flag = true;
  }

  // Clip to 0 -> 1
  throttle_value = constrain(uc, 0, 1)
  throttle_value = int(throttle_value * 1000 + 1000); // 1000 -> 2000

  error_prev = error;

  motor.writeMicroseconds(throttle_value);

  // READING CURRENT SENSOR ------------------------
  current_adc_reading = ADS.readADC_Differential_0_1(); //Raw data reading // 0 -> 4095
  current_value = ADS.toVoltage(current_adc_reading) / 0.04 - current_offset; //0.04V/A -->[A]

  // PRINTING VALUES --------------------------------
  printDebugStatements();

  time_before = time_now;

  // Update iteration number
  ++k;

  // if k too large to store reset it to zero
  if (k > 1000000)
    k = 0;
}

void printDebugStatements() {
  if (first_run_flag) {
    Serial.println("Time[ms], Throttle[1000 -> 2000], Current[A], Frequency [Hz]");
    first_run_flag = false;
  } else {
    Serial.print(time_now);
    Serial.print(",");
    Serial.print(throttle_value);
    Serial.print(",");
    Serial.print(current_value);
    Serial.print(",");
    Serial.print(1000 / (time_now - time_before));
    Serial.println();
  }
}

float calculateCurrentOffset(int numOfSamples) {
  Serial.println("Calculating Current Offset...");

  float sum = 0;
  for (int i = 0; i < numOfSamples; ++i) {
    int16_t raw_adc = ADS.readADC_Differential_0_1();
    float voltage = ADS.toVoltage(raw_adc);
    sum += voltage / 0.04;
    delay(10);
  }
  float currentOffset = sum / numOfSamples;

  Serial.print("Current Offset: ");
  Serial.println(currentOffset);
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