#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire, 0.02, 0.98); // Complementary filter coeffs [0.02 ACC, 0.98 GYRO]

// --- Controller Constants ----

#define K_P 10 // P constant
#define K_D 0 // D constant
#define K_I 0 // I constant

// -----------------------------

#define THROTTLE_BASE_VAL 1400.0

//Motor pin numbers
int motor_left_esc = 9; 
int motor_right_esc = 3;

Servo motor_L; 
Servo motor_R;

//Variables
float desired_angle = 0;
float throttle = THROTTLE_BASE_VAL; //initial speed of the motors, used in the end - RANDOM

float error; float prev_error = 0; float error_sum = 0; //cumulative error for the Integral part

float c_time; //current time
float prev_time, elapsed_time;

float pid_p, pid_i, pid_d, PID;
float motor_L_speed, motor_R_speed;

long acc_x, acc_y, acc_z;
float c_angle; //current angle (most recent measurement)
float angle_x, angle_y;


void setup(){

    //MOTOR ARM
    //Sending initial stop signal to the motors
    motor_L.attach(motor_left_esc);
    motor_R.attach(motor_right_esc);
    delay(1000);
    motor_L.writeMicroseconds(1000);
    motor_R.writeMicroseconds(1000);

    //Initialization
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();

    //Configuring DLPF (Digital Low Pass Filter)
    Wire.beginTransmission(0b1101000); //Choosing 4th filter available (Sec. 4.6 Register 26)
    Wire.write(0x1A);
    Wire.write(0b00000100);
    Wire.endTransmission(); delay(100);

    mpu6050.calcGyroOffsets(true);

    //Start counting time in milliseconds
    c_time = millis();

    delay(1000);
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
    c_angle = mpu6050.getAngleX(); //current angle

    //Calculating error
    error = desired_angle - c_angle; //current error
    error_sum += (error + prev_error) / 2 * elapsed_time; //Integration method: Trapezoidal Rule

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
    motor_L_speed = constrain(throttle + PID, 1000, 1700); //Clamping saturation limit
    motor_R_speed = constrain(throttle - PID, 1000, 1700);
    motor_L.writeMicroseconds(motor_L_speed); 
    motor_R.writeMicroseconds(motor_R_speed);


    //Serial Monitor Data Output
    Serial.print("Angle: "); Serial.print(c_angle);
    Serial.print("  Error: "); Serial.print(error);
    Serial.print("  Speed Left: "); Serial.print(motor_L_speed);
    Serial.print("  Speed Right: "); Serial.print(motor_R_speed); 
    Serial.println();

    //Keeping current error in memory
    prev_error = error;

    //Final delay before next iteration
    delay(85);
}
