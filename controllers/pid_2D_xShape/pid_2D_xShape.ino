#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

MPU6050 mpu6050(Wire, 0.02, 0.98); // Complementary filter coeffs [0.02 ACC, 0.98 GYRO]

/*  Address through which two modules communicate.
    The pipe address does not have to be “alex0”, 
    it can be any 5-character string such as “0s08d” 
    as long as the transmitter and receiver both use the same address.
*/
// const byte address[6] = "alex0"; 
const uint64_t address = 0xE8E8F0F0E1LL;

#define THROTTLE_BASE_VAL 1250.0 //initial speed of the motors, used in the end - RANDOM
#define MIN_MOTOR_VAL 1000
#define MAX_MOTOR_VAL 1500 //Motor clamping saturation limit

//Motor pin numbers
int motor_a_esc = 8; 
int motor_b_esc = 3;
int motor_c_esc = 2;
int motor_d_esc = 5;

Servo motor_a; 
Servo motor_b;
Servo motor_c;
Servo motor_d;

RF24 radio(6, 9); // CE , CSN pins

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channels
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;

  float K_P_x, K_I_x, K_D_x;
  float K_P_y, K_I_y, K_D_y;
};
MyData data;

//Variables
float desired_angle_x = 0.0;
float desired_angle_y = 0.0;

float error_x, error_y; 
float prev_error_x = 0; float error_sum_x = 0; //cumulative error for the Integral part
float prev_error_y = 0; float error_sum_y = 0; 

float flag_x = 1.0; //Integrators flag. Can be either 1 or 0. If 0, the "I" part of pids turn off
float flag_y = 1.0;

float c_time; //current time
float prev_time, elapsed_time;
unsigned long lastRecvTime = 0; //last time we received commands from transmitter

float pid_p_x, pid_i_x, pid_d_x, PID_x;
float pid_p_y, pid_i_y, pid_d_y, PID_y;
float motor_a_speed, motor_b_speed, motor_c_speed, motor_d_speed;
float K_P_x = 0; float K_D_x = 0; float K_I_x = 0;
float K_P_y = 0; float K_D_y = 0; float K_I_y = 0;

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
    delay(500);
    

    //Initialization
    //Serial
    Serial.begin(9600);
    //I2C for mpu6050
    Wire.begin();
    mpu6050.begin();
    //Radio for nrf24
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.openReadingPipe(1, address); //set receiving address
    radio.startListening(); //Set module as receiver
    resetRFData();

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
    
    //Receiving data from transmitter
    recvData();

    //Check if transmitter changed the pid constants from the last iteration
    //If yes, we need to reset the integral sum part to start all over again
    if (pidConstsChanged_X()){
        error_sum_x = 0;
        //Setting received PID values to be used
        K_P_x = data.K_P_x;
        K_I_x = data.K_I_x;
        K_D_x = data.K_D_x;
    }
    if (pidConstsChanged_Y()){
        error_sum_y = 0;
        //Setting received PID values to be used
        K_P_y = data.K_P_y;
        K_I_y = data.K_I_y;
        K_D_y = data.K_D_y;
    }

    //Calculating angles from data received
    desired_angle_x = map(data.pitch, 0, 255, -10, 10); //Angle mapping values can be changed
    desired_angle_y = map(data.roll, 0, 255, -10, 10);

    //Calculating error
    error_x = desired_angle_x - c_angle_x; //current error
    error_y = desired_angle_y - c_angle_y;
    error_sum_x += (error_x + prev_error_x) / 2 * elapsed_time; //Integration method: Trapezoidal Rule
    error_sum_y += (error_y + prev_error_y) / 2 * elapsed_time;

    // ----- P I D CONTROLLER --------
    
    // X direction
    pid_p_x = K_P_x * error_x;
    pid_d_x = K_D_x * (error_x - prev_error_x) / elapsed_time;
    pid_i_x = K_I_x * error_sum_x * flag_x;

    PID_x = pid_p_x + pid_i_x + pid_d_x;

    // Y direction
    pid_p_y = K_P_y * error_y;
    pid_d_y = K_D_y * (error_y - prev_error_y) / elapsed_time;
    pid_i_y = K_I_y * error_sum_y * flag_y;

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

    //Anti-Windup Method
    integrators_anti_windup();

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
    // time(1), angle_x(2), angle_y(3), speed_a(4), speed_b(5), speed_c(6), speed_d(7), 
    // PID_x(8), PID_y(9), Connection Lost?(10), Clamping?(11)
    // K_P_x(12), K_I_x(13), K_D_x(14)
    // K_P_y(15), K_I_y(16), K_D_y(17)

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
    Serial.print(",");
    Serial.print(millis() - lastRecvTime < 100?1:0);
    Serial.print(",");
    Serial.print((flag_x==0 || flag_y==0)?1:0);
    Serial.print(",");
    Serial.print(K_P_x);
    Serial.print(",");
    Serial.print(K_I_x);
    Serial.print(",");
    Serial.print(K_D_x);
    Serial.print(",");
    Serial.print(K_P_y);
    Serial.print(",");
    Serial.print(K_I_y);
    Serial.print(",");
    Serial.print(K_D_y);
    Serial.println("*/");
}

void integrators_anti_windup(){

    /* Check whether contrain clamped the output or not
       not constrained -> bool = 0
       constrained     -> bool = 1 
    */
    bool out_1 = 
        (motor_a_speed != THROTTLE_BASE_VAL + PID_x - PID_y) ||
        (motor_b_speed != THROTTLE_BASE_VAL + PID_x + PID_y) ||
        (motor_c_speed != THROTTLE_BASE_VAL - PID_x - PID_y) ||
        (motor_d_speed != THROTTLE_BASE_VAL - PID_x + PID_y) ;  

    
    /* Check if the error and the pid output have the same sign
       If yes, that means that the integrator is adding to the output
       wich makes our situation worse
       same sign      -> bool = 1
       different sign -> bool = 0
    */
    bool out_2_x = (PID_x * error_x) >= 0 ;
    bool out_2_y = (PID_y * error_y) >= 0 ;

    // If we're getting worse, turn off the "I" part from the pid
    flag_x = (out_1 && out_2_x)?0:1;
    flag_y = (out_1 && out_2_y)?0:1;

}

 
/* "Safe" values to use when no radio input is detected.
   Throttle is 0 in order to stop the motors
   127 is the middle value (0 - 255)
*/
void resetRFData() {
    data.throttle = 0;
    data.yaw = 127;
    data.pitch = 127;
    data.roll = 127;
    data.AUX1 = 0;
    data.AUX2 = 0;
}

//If there are available data received then read them
void recvData(){
    if ( radio.available() ){
        radio.read(&data, sizeof(MyData));
        lastRecvTime = millis();
    } 
    //Connection Lost?
    else if (millis() - lastRecvTime > 1000){ 
        //If no commands received for over 1 sec
        //Reset data to make drone hover in the air
        resetRFData();
    }
}

//Check if transmitter changed the pid constants from the last iteration
bool pidConstsChanged_X(){
    //If changed => return true
    if (K_P_x != data.K_P_x ||
        K_I_x != data.K_I_x ||
        K_D_x != data.K_D_x ) {
            return true;            
    } else {
        return false;
    }
}
bool pidConstsChanged_Y(){
    //If changed => return true
    if (K_P_y != data.K_P_y ||
        K_I_y != data.K_I_y ||
        K_D_y != data.K_D_y ) {
            return true;            
    } else {
        return false;
    }
}