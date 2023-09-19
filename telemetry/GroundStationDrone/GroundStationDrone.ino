#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*  Address through which two modules communicate.
    The pipe address does not have to be “alex0”, 
    it can be any 5-character string such as “0s08d” 
    as long as the transmitter and receiver both use the same address.
*/
const uint64_t drone_address = 0xF0F0F0F0D2LL; // Ground Station NRF Address

RF24 radio(6, 9); //CE - CSN

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

    //Transmitter Data
    // byte throttle;
    // byte yaw;
    // byte pitch;
    // byte roll;
    byte connection_status; //connection with transmitter on the ground

    byte motorsArmed;

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

//Variables
float last_time_sent_arm = 0; //It stores the last time "arm" or "disarm" command was sent via terminal
float lastRecvTime = 0;

void setup() {
    Serial.begin(9600);
    Serial.print("Struct Size: "); Serial.println(sizeof(TelemetryData));

    resetRFData();

    //NRF setup
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.openWritingPipe(drone_address); // Set the address to send to here (drone)
    radio.openReadingPipe(1, drone_address); // Set the address to listen from here (drone)
    radio.startListening();

    delay(1000);
}

void loop() {

    //Receive data from drone
    rcv_telemetry();

    //Printing serial
    serialPrintTelemetryData();

    //Checking for input commands through serial port
    checkInputFromTerminal();

    delay(100);
    
}



// --------- FUNCTIONS NEEDED -------------

void rcv_telemetry(){
    if (radio.available()) {
        radio.read(&telemetry, sizeof(telemetry));
        lastRecvTime = millis();
    }
    else if (millis() - lastRecvTime > 1000){ 
        resetRFData();
    }
}

void resetRFData(){
    telemetry.c_time = -1;
    telemetry.angle_x = 0;
    telemetry.angle_y = 0;
    telemetry.connection_status = millis() - lastRecvTime < 100?1:0;
    // telemetry.throttle = 0;
    // telemetry.yaw = 0;
    // telemetry.pitch = 0;
    // telemetry.roll = 0;
    telemetry.motorsArmed = 0;
}

void serialPrintTelemetryData() {
    // Display the received telemetry data
    Serial.print("/*");
    
    Serial.print(telemetry.c_time/1000); //time the drone controller is up and running [sec] 
    Serial.print(",");
    Serial.print(telemetry.angle_x); 
    Serial.print(",");
    Serial.print(telemetry.angle_y); 
    Serial.print(",");
    // Serial.print(telemetry.throttle); 
    // Serial.print(",");
    // Serial.print(telemetry.roll); 
    // Serial.print(",");
    // Serial.print(telemetry.pitch); 
    // Serial.print(",");
    // Serial.print(telemetry.yaw); 
    Serial.print(",");
    Serial.print(telemetry.connection_status); 
    Serial.print(",");
    Serial.print(telemetry.motorsArmed); 
    Serial.print(",");
    Serial.print(telemetry.lat); 
    Serial.print(",");
    Serial.print(telemetry.lng); 
    Serial.print(",");
    Serial.print(telemetry.altitude); 
    Serial.print(",");
    Serial.print(telemetry.gps_status); 
    // Serial.print(",");
    // Serial.print(telemetry.speedGps);
    // Serial.print(",");
    // Serial.print(telemetry.courseAngle);  
    
    Serial.println("*/");
}

void checkInputFromTerminal(){
    // Check for input in the terminal
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // Read the input string until a newline character is encountered

        if (input == "arm") {
            gs_data.gs_command = 1;
            last_time_sent_arm = millis();

        } else if (input == "disarm") {
            gs_data.gs_command = 2;
            last_time_sent_arm = millis();

        } else {
            interpreteTerminalInput(input);
        }

        sendValuesToDrone();
    }
}

void interpreteTerminalInput(String input){

    // Extract individual values from the input string
    int kpx, kix, kdx, kpy, kiy, kdy;
    sscanf(input.c_str(), "%d,%d,%d,%d,%d,%d", &kpx, &kix, &kdx, &kpy, &kiy, &kdy);


    gs_data.K_P_x = kpx;
    gs_data.K_I_x = kix;
    gs_data.K_D_x = kdx;
    gs_data.K_P_y = kpy;
    gs_data.K_I_y = kiy;
    gs_data.K_D_y = kdy;

    gs_data.gs_command = 0; //neutral position

}

void sendValuesToDrone() {

    // Send motion commands to the drone
    radio.stopListening();
    radio.write(&gs_data, sizeof(GroundStationData));
    radio.startListening();
}

