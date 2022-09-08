#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// --- Controller Constants ----

#define K_P 2.0 // P constant
#define K_D 1.5 // D constant
#define K_I 1.2 // I constant

// -----------------------------

MPU6050 mpu6050(Wire);
Servo motor_L; 
Servo motor_R;

//Variables
float desired_angle = 0;
float throttle = 1200; //initial speed of the motors, used in the end - RANDOM
float error; float prev_error = 0; float error_sum = 0; //cumulative error for the Integral part
float c_time; //current time
float prev_time, elapsed_time;
float pid_p, pid_i, pid_d, PID;
float motor_L_speed, motor_R_speed;

//Motor pin numbers
int motor_left_esc = 9; 
int motor_right_esc = 8;

void setup(){

    //Initialization
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    motor_L.attach(motor_left_esc);
    motor_R.attach(motor_right_esc);

    //Start counting time in milliseconds
    c_time = millis();

    //MOTOR ARM
    //Sending initial stop signal to the motors
    delay(1000);
    motor_L.writeMicroseconds(1000);
    motor_R.writeMicroseconds(1000);

    delay(4000);
    //They say that we have to give 1000ms to the ESCs and after that we connect to battery
    //Otherwise they might get into configuration mode or not activate at all.

}

void loop(){

    //Keeping track of time
    prev_time = c_time;
    c_time = millis();
    elapsed_time = (c_time - prev_time) / 1000; //time passed in [sec]

    //Reading sensor angle
    mpu6050.update();
    float c_angle = mpu6050.getAngleX(); //current angle

    //Calculating error
    error = desired_angle - c_angle; //current error
    error_sum = error_sum + error * elapsed_time; //Integration method: Midpoint rule

    // ----- P I D CONTROLLER --------
    
    pid_p = K_P * error;
    pid_d = K_D * (error - prev_error) / elapsed_time;
    pid_i = K_I * error_sum;

    PID = pid_p + pid_i + pid_d;

    // -----------------------------

    /*
        Since motor speed goes from 1000 to 2000, the maximun we could add from 
        stop (1000) is 1000 and the max we could substract from top speed (2000)
        is -1000. So, we expect the pid result values to vary from -1000 to 1000.
    */
    PID = constrain(PID, -1000, 1000);

    //Writing final calculated motor speed
    motor_L_speed = constrain(throttle + PID, 1000, 2000); //Clamping saturation limit
    motor_R_speed = constrain(throttle - PID, 1000, 2000);
    motor_L.writeMicroseconds(motor_L_speed); 
    motor_R.writeMicroseconds(motor_R_speed);


    //Serial Monitor Data Output
    Serial.print("Angle: "); Serial.println(c_angle);
    Serial.print("Error: "); Serial.println(error);
    Serial.print("Speed Left: "); Serial.println(motor_L_speed);
    Serial.print("Speed Right: "); Serial.println(motor_R_speed); 
    Serial.println();

    //Keeping current error in memory
    prev_error = error;

    //Final delay before next iteration
    delay(400);
}
