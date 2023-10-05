#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <Adafruit_ADS1X15.h>
#include <SdFat.h>

// For Current Measurement
Adafruit_ADS1115 ads; 

// Create a TinyGPS++ object
TinyGPSPlus gps;

//SD Card File object
SdFat SD;
SdFile myFile;


MPU6050 mpu6050(Wire, 0.02, 0.98); // Complementary filter coeffs [0.02 ACC, 0.98 GYRO]

const uint64_t GS_nrf_address = 0xF0F0F0F0D2LL; // Ground Station NRF Address

//Buzzer Pin
const byte buzzer_pin = 7;

//SD CS Pin
const byte SD_chipSelect = 10;
#define SD_REOPEN_TIME 10000


RF24 radio(6, 9); // CE , CSN pins


// ----------- Structs --------------------

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

    //GPS 
    byte gps_status = -1;
    float lat = 0; // latitude
    float lng = 0; // longitude
    float altitude;
    // float speedGps; //[m/s]
    // float courseAngle; //[deg]

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

    float K_P_x, K_I_x, K_D_x;
    float K_P_y, K_I_y, K_D_y;
};
GroundStationData gs_data;



// ---------------  Variables Needed ------------------

float c_time; //current time
float prev_time, elapsed_time;
unsigned long lastRecvTime = 0;
unsigned long lastGpsAquiredTime = 0;

float c_angle_x, c_angle_y; //current angle (most recent measurement)

double volts_shunt = 0;
double current = 0;
byte ads_exists = 1;

int file_offset = 0;  // This is the file offset that will be appended to the filename.
unsigned long sd_last_opened = millis();
unsigned long sd_last_restarted = millis();
char filename[20];
bool sd_connected = false;


void setup(){

    //Setting pinModes
    pinMode(LED_BUILTIN, OUTPUT);


    // --------- Initialization ----------
    //Serial for GPS
    Serial.begin(9600);


    //Radio for nrf24
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.openWritingPipe(GS_nrf_address);
    radio.openReadingPipe(1, GS_nrf_address);
    radio.startListening(); //Set module as receiver


    //I2C for mpu6050
    Wire.begin();
    mpu6050.begin();
    //Configuring DLPF (Digital Low Pass Filter)
    Wire.beginTransmission(0b1101000); //Choosing 4th filter available (Sec. 4.6 Register 26)
    Wire.write(0x1A);
    Wire.write(0b00000100);
    Wire.endTransmission(); delay(100);
    //Calculating gyro offsets
    mpu6050.calcGyroOffsets(true); 


    //ADS Init
    ads.setGain(GAIN_SIXTEEN);  // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    if (!ads.begin()) {
        Serial.println(F("Failed to initialize ADS."));
        ads_exists = 0;
    }

    Serial.println(F("Before sd init"));
    //SD card init
    sd_connected = SD.begin(SD_chipSelect);
    Serial.println(sd_connected?F("SD Init Completed"):F("SD Init Failed"));
    Serial.println(F("Before opening file"));
    if (sd_connected)
        sdOpenFile(0);
    delay(500);

    //Start counting time in milliseconds
    c_time = millis();

    //Buzzer beep
    starting_beep(3, 200, 200, 1000); 
    delay(1000);

    sd_last_opened = millis();
    sd_last_restarted = millis();
}





void loop(){

    //Keeping track of time
    prev_time = c_time;
    c_time = millis();
    elapsed_time = (c_time - prev_time) / 1000; //time passed in [sec]



    //Reading sensor angle
    mpu6050.update();
    getAnglesFiltered(&c_angle_x, &c_angle_y);



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
      volts_shunt = ads.computeVolts(ads.readADC_Differential_0_1());
      current = volts_shunt * 50/0.075;  // I = V / R_shunt
    }

    
    //Receiving data from Transmitter and Ground Station
    recvData();
    //Sending Telemetry data to ground
    sendTelemetryData();



    //Serial Monitor Data Output
    serialPrintData(); //Disconnect GPS before uncommenting

    //Writing to SD card
    writeDataToSD();

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

    // Serial.print(F("/*"));
    // Serial.print(millis()/1000); //number of [sec] since system started
    // Serial.print(F(","));
    // Serial.print(c_angle_x); 
    // Serial.print(F(","));
    // Serial.print(c_angle_y); 
    // // Serial.print(","); 
    // // Serial.print(millis() - lastRecvTime < 100?1:0);
    // // Serial.print(",");
    // // Serial.print(1000 / elapsed_time); //Loop Frequency [Hz]
    // // Serial.print(",");
    // Serial.print(telemetry.gps_status);
    // Serial.print(F(","));
    // Serial.print(telemetry.lat);
    // Serial.print(F(","));
    // // Serial.print(telemetry.lng);
    // // Serial.print(",");
    // // Serial.print(telemetry.altitude);
    // // Serial.print(",");
    // Serial.println(F("*/"));

    Serial.print(1);
    Serial.print(",");
    Serial.println(2);
}


 
//If there are available data received then read them
void recvData(){

    if (radio.available()){
        // Read Commands from Ground Station
        radio.read(&gs_data, sizeof(gs_data));
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
        // motorsArmed = true;
    }
    else if (gs_data.gs_command == 2){
        // motorsArmed = false;
    }
}

// Stops listening and sends telemetry data back toac Ground Station
void sendTelemetryData(){

    telemetry.c_time = millis();
    telemetry.angle_x = c_angle_x;
    telemetry.angle_y = c_angle_y;
    // telemetry.throttle = data.throttle;
    // telemetry.yaw = data.yaw;
    // telemetry.pitch = data.pitch;
    // telemetry.roll = data.roll;

    radio.stopListening();
    radio.write(&telemetry, sizeof(telemetry));
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

//------------------------ SD CARD --------------------------------------

// // This function returns the correct filename 
// //of the measurements file given a certain offset.
// String get_filename(int file_offset) {
//   return "drone" + String(file_offset) + ".csv";
// }
void get_filename(int file_offset, char* filename) {
  sprintf(filename, "drone%d.csv", file_offset);
}



/*
    If reopenLast = 1 -> try to open the last remembered filename
*/
void sdOpenFile(byte reopenLast){

    if (reopenLast == 0){
        // Find latest file offset for the filename  
        get_filename(file_offset, filename);
        while (SD.exists(filename)){
            file_offset += 1;
            get_filename(file_offset, filename);
        }
    }
    // try opening the file...
    get_filename(file_offset, filename);
    myFile.open(filename, O_WRITE | O_CREAT);

    while (!myFile) {
        for (int i = 0; i < 5; i++){
            get_filename(file_offset, filename);
            myFile.open(filename, O_WRITE | O_CREAT);
            delay(200);
            Serial.print(F("Reopening Loop"));
            Serial.println(filename);
            if (myFile) break;
        }
        file_offset += 1;
    }
    

    if (myFile){
        Serial.print(F("Measurements will be saved in file: "));
        Serial.println(filename);
    }
    else {
        Serial.print(F("Something went wrong while trying to open the file "));
        Serial.println(filename);
    }

    if (myFile && reopenLast != 0)
        Serial.println(F("Succesfully Reopened!"));
    if (!myFile && reopenLast != 0)
        Serial.println(F("Could not be Reopened!"));

    file_offset = myFile?file_offset:file_offset+1;
    
}


void storeDataSD(){
    // firt we check that the file is open and working
    if (myFile) {
        myFile.print(1);
        myFile.print(",");
        myFile.println(2);
        // myFile.sync(); 
        // myFile.flush(); 
    } else {
        get_filename(file_offset, filename);
        Serial.print(F("There seems to be an error with the SD File "));
        Serial.println(filename);

    }
    delay(500);

}

void closeSD(){

    Serial.println(myFile?F("True Before Close"):F("False Before Close"));
    myFile.close();
    Serial.println(myFile?F("True After Close"):F("False After Close"));


    get_filename(file_offset, filename);
    Serial.print(F("File: "));
    Serial.print(filename);
    Serial.println(F(" closed. Have fun :)"));

}

void writeDataToSD(){
    if (sd_connected == true){

        storeDataSD();  

        if (millis() - sd_last_opened > SD_REOPEN_TIME){
            sd_last_opened = millis();
            Serial.println(F("Syncing"));
            myFile.sync();
        }
        if (millis() - sd_last_restarted > 3 * SD_REOPEN_TIME){
            sd_last_restarted = millis();
            Serial.println(F("Restarting"));
            delay(500);
            closeSD();
            delay(500);
            myFile = SdFile();
            sd_connected = SD.begin(SD_chipSelect); // re-initialize SD card
            Serial.println(sd_connected?F("Reinit OK"):F("Reinit FAILED"));
            delay(500);
            file_offset += 1;
            sdOpenFile(0); //reopen
        }
    }
}