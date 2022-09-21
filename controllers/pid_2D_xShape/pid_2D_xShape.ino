#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire, 0.02, 0.98); // Complementary filter coeffs [0.02 ACC, 0.98 GYRO]

// --- Controller Constants ----

// X coordinate

#define K_P_x 2 // P constant 
#define K_D_x 0   // D constant 
#define K_I_x 0 // I constant 

// Y coordinate

#define K_P_y 2 // P constant 
#define K_D_y 0   // D constant 
#define K_I_y 0 // I constant 

// -----------------------------

#define THROTTLE_BASE_VAL 1250.0 //initial speed of the motors, used in the end - RANDOM
#define MIN_MOTOR_VAL 1000
#define MAX_MOTOR_VAL 1500 //Motor clamping saturation limit

//Motor pin numbers
int motor_a_esc = 9; 
int motor_b_esc = 3;
int motor_c_esc = 2;
int motor_d_esc = 5;

Servo motor_a; 
Servo motor_b;
Servo motor_c;
Servo motor_d;

//Variables
float desired_angle_x = 0.0;
float desired_angle_y = 0.0;

float error_x, error_y; 
float prev_error_x = 0; float error_sum_x = 0; //cumulative error for the Integral part
float prev_error_y = 0; float error_sum_y = 0; //cumulative error for the Integral part

float c_time; //current time
float prev_time, elapsed_time;

float pid_p_x, pid_i_x, pid_d_x, PID_x;
float pid_p_y, pid_i_y, pid_d_y, PID_y;
float motor_a_speed, motor_b_speed, motor_c_speed, motor_d_speed;

float c_angle_x, c_angle_y; //current angle (most recent measurement)


void setup(){

    //MOTOR ARM
    //Sending initial stop signal to the motors
    // digitalWrite(motor_left_esc, 0); //We need to have them low before attaching, otherwise 
    // digitalWrite(motor_right_esc, 0); //when arduino powers on they go crazy and then turn off (Safety Measure!!)
    // delay(100);
    motor_a.attach(motor_a_esc);
    motor_b.attach(motor_b_esc);
    motor_c.attach(motor_c_esc);
    motor_d.attach(motor_d_esc);
    motor_a.writeMicroseconds(1000);
    motor_b.writeMicroseconds(1000);
    motor_c.writeMicroseconds(1000);
    motor_d.writeMicroseconds(1000);
    delay(1000);
    

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

    delay(500);
}

void loop(){

    //Keeping track of time
    prev_time = c_time;
    c_time = millis();
    elapsed_time = (c_time - prev_time) / 1000; //time passed in [sec]

    //Reading sensor angle
    mpu6050.update();
    getAnglesFiltered(&c_angle_x, &c_angle_y);

    //Calculating error
    error_x = desired_angle_x - c_angle_x; //current error
    error_y = desired_angle_y - c_angle_y;
    error_sum_x += (error_x + prev_error_x) / 2 * elapsed_time; //Integration method: Trapezoidal Rule
    error_sum_y += (error_y + prev_error_y) / 2 * elapsed_time;

    // ----- P I D CONTROLLER --------
    
    // X direction
    pid_p_x = K_P_x * error_x;
    pid_d_x = K_D_x * (error_x - prev_error_x) / elapsed_time;
    pid_i_x = K_I_x * error_sum_x;

    PID_x = pid_p_x + pid_i_x + pid_d_x;

    // Y direction
    pid_p_y = K_P_y * error_y;
    pid_d_y = K_D_y * (error_y - prev_error_y) / elapsed_time;
    pid_i_y = K_I_y * error_sum_y;

    PID_y = pid_p_y + pid_i_y + pid_d_y;

    // -----------------------------

    /*
        Since motor speed goes from 1000 to 2000, the maximun we could add from 
        stop (1000) is 1000 and the max we could substract from top speed (2000)
        is -1000. So, we expect the pid result values to vary from -1000 to 1000.
    */
    PID_x = constrain(PID_x, -500, 500);
    PID_y = constrain(PID_y, -500, 500);

    //Writing final calculated motor speed
    motor_a_speed = constrain(THROTTLE_BASE_VAL + PID_x - PID_y, MIN_MOTOR_VAL, MAX_MOTOR_VAL); //Clamping saturation limit
    motor_b_speed = constrain(THROTTLE_BASE_VAL + PID_x + PID_y, MIN_MOTOR_VAL, MAX_MOTOR_VAL);
    motor_c_speed = constrain(THROTTLE_BASE_VAL - PID_x - PID_y, MIN_MOTOR_VAL, MAX_MOTOR_VAL);
    motor_d_speed = constrain(THROTTLE_BASE_VAL - PID_x + PID_y, MIN_MOTOR_VAL, MAX_MOTOR_VAL);

    motor_a.writeMicroseconds(motor_a_speed);
    motor_b.writeMicroseconds(motor_b_speed);
    motor_c.writeMicroseconds(motor_c_speed);
    motor_d.writeMicroseconds(motor_d_speed);

    //Serial Monitor Data Output
    serialPrintData();

    //Keeping current error in memory
    prev_error_x = error_x;
    prev_error_y = error_y;

    //Final delay before next iteration
    delay(10);
}


//---------- ASSISTING FUNCTIONS -------------

/*
    MOVING AVERAGE FILTER
    Just measures some angles and returns the average.
    For eg, if count = 3:

                 angle[t] +  angle[t+1]  +  angle[t+2]
        result = -------------------------------------
                                 3

    It can be done with as many angle measurements as we want.
    There is a delay of 5 mSec in between (randomly choosen small value)

*/
void getAnglesFiltered(float *angle_x, float *angle_y){

    int count = 3;
    float result_angle_x = 0;
    float result_angle_y = 0;

    for (int i = 0; i < count; ++i)
    {
        mpu6050.update();
        result_angle_x += mpu6050.getAngleX();
        result_angle_y += mpu6050.getAngleY();
        
        delay(10);
    }

    *(angle_x) = result_angle_x / count;
    *(angle_y) = result_angle_y / count;

}

/*
    Data Export for Serial Studio
*/
void serialPrintData(){
    //ORDER:
    // time(1), angle_x(2), angle_y(3), speed_a(4), speed_b(5), speed_c(6), speed_d(7), PID_x(8), PID_y(9)

    Serial.print("/*");
    Serial.print(millis()); //ms since system started
    Serial.print(",");
    Serial.print(c_angle_x); 
    Serial.print(",");
    Serial.print(c_angle_y); 
    Serial.print(","); 
    Serial.print(motor_a_speed); 
    Serial.print(",");
    Serial.print(motor_b_speed); 
    Serial.print(",");
    Serial.print(motor_c_speed); 
    Serial.print(",");
    Serial.print(motor_d_speed); 
    Serial.print(",");
    Serial.print(PID_x); 
    Serial.print(",");
    Serial.print(PID_y); 
    Serial.println("*/");
}