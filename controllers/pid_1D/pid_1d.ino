/* 
    TODO:
        - Check if it works
        - throttle value? Random?
        - cut final output before 2000, for example at 1600.
          Motors are powerfull and I wouldn't want to be near them running full speed.
          After all, we dont need so much power.
        - Implement "D" from pid
        - Implement "I" from pid 
        - [throttle + PID] to right or [throttle - PID]?

*/

#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// --- Controller Constants ----

#define P_CONST 2;

// -----------------------------

MPU6050 mpu6050(Wire);
Servo motor_L; 
Servo motor_R;

//Variables
float desired_angle = 0;
float throttle = 1200; //initial speed of the motors, used in the end - RANDOM

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

    //Reading sensor angle
    mpu6050.update();
    float c_angle = mpu6050.getAngleX(); //current angle

    float error = desired_angle - c_angle;

    // ----- P I D CONTROLLER --------
    
    float pid_p = P_CONST * error;
    float pid_d = 0;
    float pid_i = 0;

    float PID = pid_p + pid_i + pid_d;

    // -----------------------------

    /*
        Since motor speed goes from 1000 to 2000, the maximun we could add from 
        stop (1000) is 1000 and the max we could substract from top speed (2000)
        is -1000. So, we expect the pid result values to vary from -1000 to 1000.
    */
    PID = cutBound(PID, -1000, 1000);

    //Writing final calculated motor speed
    motor_L_speed = cutBound(throttle + PID, 1000, 2000);
    motor_R_speed = cutBound(throttle - PID, 1000, 2000);
    motor_L.writeMicroseconds(motor_L_speed); 
    motor_R.writeMicroseconds(motor_R_speed);


    //Serial Monitor Data Output
    Serial.print("Angle: "); Serial.println(c_angle);
    Serial.print("Error: "); Serial.println(error);
    Serial.print("Speed Left: "); Serial.println(motor_L_speed);
    Serial.print("Speed Right: "); Serial.println(motor_R_speed); 
    Serial.println();


    delay(400);
}



// --------- ASSISTING FUNCTIONS ------------

/*
    Contains a certain number between 2 boundaries by cutting.
    Eg:
        1200 in [1000, 2000] => 1200
        900 in [1000, 2000] => 1000
        2500 in [1000, 2000] => 2000
    Contains "value" in [down, up]
*/
float cutBound(float value, float down, float up){
    if (value > up){
        return up;
    } else if (value < down){
        return down;
    } else {
        return value;
    }
}