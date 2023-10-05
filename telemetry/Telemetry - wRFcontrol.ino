#include <Servo.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <Adafruit_ADS1X15.h>

// For Current Measurement
Adafruit_ADS1115 ads; 

// Create a TinyGPS++ object
TinyGPSPlus gps;

RF24 radio(6, 9); // CE , CSN pins
uint8_t pipeNum; // Variable to store the pipe number

const uint64_t transmitter_address = 0xE8E8F0F0E1LL; 
const uint64_t GS_nrf_address = 0xF0F0F0F0D2LL; // Ground Station NRF Address

//Buzzer Pin
const byte buzzer_pin = 7;

//MPU9250
#define MPU9250_ADDR 0x68
MPU9250_WE mpu9250 = MPU9250_WE(MPU9250_ADDR);

//Voltage Measurement
#define R1 173400.0
#define R2 36800.0

#define MIN_MOTOR_VAL 1000
#define MAX_MOTOR_VAL 1500 //Motor clamping saturation limit

//Motor pin numbers
const byte motor_esc = A0;
const byte servo_roll_esc = A1;
const byte servo_pitch_esc = A2;
const byte servo_yaw_esc = A3;

Servo motor;
Servo servo_roll;
Servo servo_pitch;
Servo servo_yaw;


// ----------- Structs --------------------
/*
    Struct size <= 32 bytes
        byte    : 1 byte    [0  : 255]
        int8_t  : 1 byte    [-128 : 127]
        int     : 2 bytes
        float   : 4 bytes
        double  : 4 bytes
        unsigned long: 4 bytes
*/

/*
  GPS status:
    -1  -  init status
    0   -  no signal
    1   -  signal
    2   -  timed out

*/
struct TelemetryData {
    unsigned long c_time; //current time in millis()
    
    float angle_x;
    float angle_y;

    // unsigned long loop_freq;

    //GPS 
    byte gps_status = -1;
    float lat = 0; // latitude
    float lng = 0; // longitude
    float altitude;
    // float speedGps; //[m/s]
    // float courseAngle; //[deg]

    //Battery
    double current = 0;
    // double voltage = 0;

    //Motors
    byte motorsArmed = 0;
    byte throttle;

};
TelemetryData telemetry;

/*
    Struct to store Ground Station Data
    gs_command = 1 : arm motors
    gs_command = 2 : disarm motors
    gs_command = 0 : neutral - do nothing
*/
struct GroundStationData {
    int gs_command; //Number sent from ground to choose action 

    // float K_P_x, K_I_x, K_D_x;
    // float K_P_y, K_I_y, K_D_y;
};
GroundStationData gs_data;

//Custom Transmitter Data
struct TransmitterData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};
TransmitterData rf_data;



// ---------------  Variables Needed ------------------

float c_time; //current time
float prev_time, elapsed_time;
unsigned long lastRecvTime = 0;
unsigned long lastGpsAquiredTime = 0;

byte mpu_exists = 0;

byte ads_exists = 0;
float bat_voltage = 0;

bool motorsArmed = false;
unsigned long lastArmedTime = 0; //last motors were armed



void setup(){

    //Setting pinModes
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(motor_esc, OUTPUT);
    pinMode(servo_roll_esc, OUTPUT);
    pinMode(servo_pitch_esc, OUTPUT);
    pinMode(servo_yaw_esc, OUTPUT);

    //ARM
    motor.attach(motor_esc);
    servo_roll.attach(servo_roll_esc);
    servo_pitch.attach(servo_pitch_esc);
    servo_yaw.attach(servo_yaw_esc);

    motor.writeMicroseconds(1000);
    servo_roll.writeMicroseconds(1000);
    servo_pitch.writeMicroseconds(1000);
    servo_yaw.writeMicroseconds(1000);
    delay(500);


    // --------- Initialization ----------
    //Serial for GPS
    Serial.begin(9600);

    //Radio for nrf24
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.openWritingPipe(GS_nrf_address);
    radio.openReadingPipe(1, transmitter_address);
    radio.openReadingPipe(2, GS_nrf_address);
    radio.startListening(); //Set module as receiver
    
    //I2C for mpu9250
    Wire.begin();
    mpu_exists = mpu9250.init()?1:0;
    Serial.println(mpu_exists==1?"MPU Connected":"MPU Not Found");
    Serial.print("MPU Calibrating...");
    delay(1000);
    mpu9250.autoOffsets();
    Serial.println("\tDone");
    mpu9250.setAccRange(MPU9250_ACC_RANGE_4G); //2, 4, 8, 16
    mpu9250.enableAccDLPF(true);
    mpu9250.setAccDLPF(MPU9250_DLPF_2);


    //ADS Init
    delay(200);
    ads.setGain(GAIN_SIXTEEN);  // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads_exists = ads.begin()?1:0;
    if (ads_exists == 0)
        Serial.println("Failed to initialize ADS.");
    else
        Serial.println("ADS initialized.");


    //Start counting time in milliseconds
    c_time = millis();

    //Buzzer beep
    starting_beep(3, 200, 200, 1000); 
    delay(1000);
}





void loop(){

    //Keeping track of time
    prev_time = c_time;
    c_time = millis();
    elapsed_time = (c_time - prev_time) / 1000; //time passed in [sec]
    // telemetry.loop_freq = (unsigned long)(1000 / (c_time - prev_time));



    //Reading MPU9250 angle
    if (mpu_exists == 1){
        xyzFloat angle = mpu9250.getAngles();
        telemetry.angle_x = angle.x;
        telemetry.angle_y = angle.y;
        // telemetry.angle_x = mpu9250.getPitch();
        // telemetry.angle_y = mpu9250.getRoll();
        // getAnglesFiltered(&telemetry.angle_x, &telemetry.angle_y, 3);
    }



    //Reading GPS
    if (Serial.available() > 0){
        if (gps.encode(Serial.read())){
            parseGPSdata();
            lastGpsAquiredTime = millis();
        }
    }
    // If 5 seconds pass and there are no characters coming in
    // over the serial port, show a "No GPS detected" error
    if (millis() - lastGpsAquiredTime > 5000 && gps.charsProcessed() < 10){
        //Tell them "No GPS detected"
        telemetry.gps_status = 2;
    }


    //Reading CURRENT
    if (ads_exists == 1){
        ads.setGain(GAIN_SIXTEEN);
        double volts_shunt = ads.computeVolts(ads.readADC_Differential_0_1());
        telemetry.current = volts_shunt * 50/0.075;  // I = V / R_shunt
    }

    //Reading Voltage
    if (ads_exists == 1){
        ads.setGain(GAIN_TWOTHIRDS);
        double adc_voltage = ads.computeVolts(ads.readADC_SingleEnded(3));
        bat_voltage = adc_voltage * (R1 + R2) / R2;
    }

    
    //Receiving data from Transmitter and Ground Station
    // recvData();
    //Sending Telemetry data to ground
    // sendTelemetryData();



    //Serial Monitor Data Output
    serialPrintData(); //Disconnect GPS before uncommenting


    //Write motor Speed
    writeMotorAndServos();


    //Checking fast safe switch
    //if it's flipped -> disarm the motors
    if (motorsArmed == true && rf_data.AUX1 == 1 && millis() - lastArmedTime > 1000){
        motorsArmed = false;
    }
    //if motors disarmed, check for arming sequence
    else if (motorsArmed == false && rf_data.throttle <= 3 && rf_data.AUX1 == 1 && rf_data.AUX2 == 1 && millis() - lastRecvTime < 100){
        motorsArmed = true;
        lastArmedTime = millis();
    }

    
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
void getAnglesFiltered(float *angle_x, float *angle_y, int count){

    float result_angle_x = 0;
    float result_angle_y = 0;

    for (int i = 0; i < count; ++i)
    {
        xyzFloat angle = mpu9250.getAngles();
        result_angle_x += angle.x;
        result_angle_y += angle.y;
        
        delay(10);
    }

    *(angle_x) = result_angle_x / count;
    *(angle_y) = result_angle_y / count;

}

/*
    Data Export for Serial Studio
*/
void serialPrintData(){

    Serial.print("/*");
    Serial.print(millis()/1000); //number of [sec] since system started
    Serial.print(",");
    Serial.print(telemetry.angle_x); 
    Serial.print(",");
    Serial.print(telemetry.angle_y); 
    // Serial.print(","); 
    // Serial.print(millis() - lastRecvTime < 100?1:0);
    // Serial.print(",");
    // Serial.print(1000 / elapsed_time); //Loop Frequency [Hz]
    // Serial.print(",");
    Serial.print(telemetry.gps_status);
    Serial.print(",");
    Serial.print(telemetry.lat);
    Serial.print(",");
    // Serial.print(telemetry.lng);
    // Serial.print(",");
    // Serial.print(telemetry.altitude);
    // Serial.print(",");
    Serial.print(bat_voltage);
    Serial.println("*/");
}


 
//If there are available data received then read them
void recvData(){

    if (radio.available(&pipeNum)){

        if (pipeNum == 1){
            // Read motion commands from transmitter
            radio.read(&rf_data, sizeof(TransmitterData));
            lastRecvTime = millis();
        }
        if (pipeNum == 2){
            // Read Commands from Ground Station
            radio.read(&gs_data, sizeof(GroundStationData));
            processGroundStationInput();
        }


    }
    else if (millis() - lastRecvTime > 1000){ 
        //If no commands received from RF for over 1 sec
        //Connection Lost ?
        motorsArmed = false;
    }


// -----------------

    if (radio.available()){
        // Read Commands from Ground Station
        radio.read(&gs_data, sizeof(GroundStationData));
        processGroundStationInput();
        lastRecvTime = millis();

    }
    else if (millis() - lastRecvTime > 1000){ 
        //Connection lost?
    }

}


/*  Blink builtin led
    eg. blinkLED(5, 100, 100); //blink 5 times, 100mSec on, 100mSec off  
*/
void blinkLED(int blinkTimes, int on_mSecs, int off_mSecs){
    for (int i = 0; i < blinkTimes; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(on_mSecs);
        digitalWrite(LED_BUILTIN, LOW);
        delay(off_mSecs);
    }
}


/* Transmitter beeps 
   eg. starting_beep(3, 200, 300, 1000)
      => beeps with the buzzer for 3 times, 200 ms ON, 300 ms OFF, at 1000 Hz
*/
void starting_beep(int beepTimes, int on_time, int off_time, int tone_freq){
  for (int i = 0; i < beepTimes; ++i){
    tone(buzzer_pin, tone_freq);
    delay(on_time);
    noTone(buzzer_pin);
    delay(off_time);
  }
}

// Function to interprete the commands sent from Ground Station
void processGroundStationInput(){
    if (gs_data.gs_command == 1) {
        motorsArmed = true;
    }
    else if (gs_data.gs_command == 2){
        motorsArmed = false;
    }
}

// Stops listening and sends telemetry data back toac Ground Station
void sendTelemetryData(){

    telemetry.c_time = millis();
    
    telemetry.throttle = rf_data.throttle;
    telemetry.motorsArmed = motorsArmed?1:0;
    // telemetry.yaw = rf_data.yaw;
    // telemetry.pitch = rf_data.pitch;
    // telemetry.roll = rf_data.roll;


    radio.stopListening();
    radio.write(&telemetry, sizeof(TelemetryData));
    radio.startListening();
}


// Save received gps data to my variables
void parseGPSdata(){
    if (gps.location.isValid()){
        telemetry.gps_status = 1;
        telemetry.lat = gps.location.lat();
        telemetry.lng = gps.location.lng();
        telemetry.altitude = gps.altitude.meters();
    } else {
        telemetry.gps_status = 0;
    }
    // if (gps.speed.isUpdated()){
    //     telemetry.speedGps = gps.speed.mps();
    //     telemetry.courseAngle = gps.course.deg();
    // }
}

void writeMotorAndServos(){
    motor.writeMicroseconds(motorsArmed?map(rf_data.throttle, 0, 255, MIN_MOTOR_VAL, MAX_MOTOR_VAL):MIN_MOTOR_VAL);

    servo_roll.write(map(rf_data.roll, 0, 255, 60, 144));
    servo_pitch.write(map(rf_data.pitch, 0, 255, 35, 150));
    servo_yaw.write(map(rf_data.yaw, 0, 255, 57, 140));
}