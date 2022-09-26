#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*  Address through which two modules communicate.
    The pipe address does not have to be “alex0”, 
    it can be any 5-character string such as “0s08d” 
    as long as the transmitter and receiver both use the same address.
*/
// const byte address[6] = "alex0"; 
const uint64_t address = 0xE8E8F0F0E1LL;

RF24 radio(6, 9); // CE , CSN pins

float loop_freq = 0; // Hz

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channels
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};
MyData data;

float c_time; //current time
float prev_time, elapsed_time;
unsigned long lastRecvTime = 0; //last time we received commands from transmitter

void setup(){

    //Initialization
    //Serial
    Serial.begin(9600);
    //Radio for nrf24
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
    radio.openReadingPipe(1, address); //set receiving address
    radio.startListening(); //Set module as receiver
    resetRFData();

    //Start counting time in milliseconds
    c_time = millis();

    delay(500);
}

void loop(){

    //Keeping track of time
    prev_time = c_time;
    c_time = millis();
    elapsed_time = (c_time - prev_time); //time passed in [msec]

    //Receiving data from transmitter
    recvData();

    //Calc loop freq
    loop_freq = 1 / elapsed_time * 1000; //Hz

    //Serial Monitor Data Output
    serialPrintData();
}


//---------- ASSISTING FUNCTIONS -------------

/*
    Data Export for Serial Studio
*/
void serialPrintData(){

    // ORDER:
    // time(1), received_flag(2), data.throttle(3), data.roll(4), data.pitch(5), data.yaw(6), data.AUX1(7), data.AUX2(8), loop_freq(9)

    Serial.print("/*");
    Serial.print(millis()/1000); //sec since system started
    Serial.print(",");
    Serial.print(millis() - lastRecvTime < 100?1:0);
    Serial.print(",");
    Serial.print(data.throttle);
    Serial.print(",");
    Serial.print(data.roll);
    Serial.print(",");
    Serial.print(data.pitch);
    Serial.print(",");
    Serial.print(data.yaw);
    Serial.print(",");
    Serial.print(1- data.AUX1);
    Serial.print(",");
    Serial.print(data.AUX2);
    Serial.print(",");
    Serial.print(loop_freq);
    Serial.println("*/");

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