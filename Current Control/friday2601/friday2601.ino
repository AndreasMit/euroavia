#include <Servo.h>

Servo motor;

// Pins Connected -------------------------------
int motor_esc_pin = A0;     // Reading PWM Throttle 
int throttle_rcvr_pin = A2; // Reading PWM Throttle
int current_sensor_pin = A3;   // Reading Current Sensor


// Variables ------------------------------------

// Variables to store pulse width and throttle position
unsigned int pulse_width;
unsigned int throttle_value;

// Current Sensor
int current_adc_reading = 0;
float current_value = 0;
float current_offset = 0;

// Control Variables
float TARGET_CURRENT = 3;     // [A]
float CENTER_THROTTLE_POS = 1500.0; // [us]
float Kp = 0;
float Kd = 0;
float Ki = 0;
float error = 0;
float error_prev = 0;
float error_sum = 0;
unsigned long time_now = millis();
unsigned long time_before = millis();

float pid_p = 0;
float pid_d = 0;
float pid_i = 0;

bool first_run_flag = true;


// Setup ----------------------------------------

void setup(){

  // Serial Configuration
  Serial.begin(115200); delay(1000);

  // Motor Configuration
  motor.attach(motor_esc_pin);
  motor.writeMicroseconds(1000);
  delay(2000);

  // Current Sensor Configuration
  current_offset = calculateCurrentOffset(200);
  delay(2000);

}

// Loop -----------------------------------------

void loop(){

    // Time
    time_now = millis();




    // READING PWM THROTTLE --------------------------
    
    // Read pulse width from the throttle channel
    // pulse_width = pulseIn(throttle_rcvr_pin, HIGH);+++++++++++++
    // throttle_value = map(pulse_width, 1000, 2000, 1000, 2000);

    // Read Throttle from potentiometer
    throttle_value = map(analogRead(throttle_rcvr_pin), 0, 1023, 1000, 1500);





    // READING CURRENT SENSOR ------------------------

    current_adc_reading = analogRead(current_sensor_pin); //Raw data reading // 0 -> 1023
    current_value = (float(current_adc_reading)-512)* 5/1024 / 0.04 - current_offset; //0.04V/A -->[A]





    // CONTROL ---------------------------------------
    error = TARGET_CURRENT - current_value;
    error_sum += error;
    pid_p = Kp * error;
    pid_d = Kd * (error - error_prev) / (time_now - time_before);
    pid_i = Ki * error_sum;

    throttle_value = CENTER_THROTTLE_POS + pid_p + pid_d + pid_i;
    throttle_value = constrain(throttle_value, 1000, 2000);

    // Writing PWM to motor
    motor.writeMicroseconds(int(throttle_value)); 





    // PRINTING VALUES --------------------------------
    printDebugStatements(2);
    // Serial.print("DG 5 = "); Serial.println(analogRead(A7));






    time_before = time_now;
    error_prev = error;
}









void printDebugStatements(int mode){
  if (mode == 1){
    //if first time, print header, else print values
    if(first_run_flag){
      Serial.print("Time,");
      Serial.print("PWM,"); Serial.print("Throttle,"); 
      Serial.print("Current,"); Serial.print("Error,"); 
      Serial.print("PID_P,"); Serial.print("PID_D,"); Serial.print("PID_I,"); Serial.print("PID");
      Serial.println();
      first_run_flag = false;
    }
    else {
      Serial.print(time_now); Serial.print(",");
      Serial.print(pulse_width); Serial.print(","); 
      Serial.print(throttle_value); Serial.print(","); 
      Serial.print(current_value); Serial.print(","); 
      Serial.print(error); Serial.print(","); 
      Serial.print(pid_p); Serial.print(","); 
      Serial.print(pid_d); Serial.print(","); 
      Serial.print(pid_i); Serial.print(","); 
      Serial.print(pid_p + pid_d + pid_i);
      Serial.println();
    }
  }
  else if (mode == 2){
    Serial.print("PWM: "); Serial.print(pulse_width);
    Serial.print(" | Throttle: "); Serial.print(throttle_value);
    Serial.print(" | Current: "); Serial.print(current_value); Serial.print(" A");
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | PID_P: "); Serial.print(pid_p);
    Serial.print(" | PID_D: "); Serial.print(pid_d);
    Serial.print(" | PID_I: "); Serial.print(pid_i);
    Serial.print(" | PID: "); Serial.print(pid_p + pid_d + pid_i);
    Serial.println();
    Serial.println(current_value);
  }
}




float calculateCurrentOffset(int numOfSamples){
  Serial.println("Calculating Current Offset...");

  float sum = 0;
  for(int i = 0; i < numOfSamples; i++){
    sum += (float(analogRead(current_sensor_pin))-512)* 5/1024 / 0.04;;
    delay(10);
  }
  float currentOffset = sum / numOfSamples;

  Serial.print("Current Offset: "); Serial.println(currentOffset);
  return currentOffset;
}