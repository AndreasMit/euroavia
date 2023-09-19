#include <TinyGPS++.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t nrf_address = 0xE8E8F0F0E1LL; 

RF24 radio(6, 9); // CE , CSN pins

// Create a TinyGPS++ object
TinyGPSPlus gps;


/*
  status:
    -1  -  init status
    0   -  no signal
    1   -  signal
    2   -  timed out

*/
struct TelemetryData{
  int status = -1;
  double lat = 0.0;
  double lot = 0.0;
  double speed = 0.0;
  // double alt = 0.0;
};
TelemetryData data;


unsigned long prev_loop_time = millis();
unsigned long prev_time_gps = millis();


void setup(){
  Serial.begin(9600);

  //Radio for nrf24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.openWritingPipe(nrf_address);
  // radio.openReadingPipe(1, nrf_address); //set receiving address
  // radio.startListening(); //Set module as receiver
}

void loop(){

  prev_loop_time = micros();

  // This sketch displays information every time a new sentence is correctly encoded.
  if (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      sendInfo();
      prev_time_gps = millis();
    }
  }

  // If 5000 milliseconds pass and there are no characters coming in
  // over the serial port, show a "No GPS detected" error
  if (millis() - prev_time_gps > 5000 && gps.charsProcessed() < 10)
  {
    data.status = 2;
    sendInfo();
    delay(500);
  }
  
}

void sendInfo(){
  if (gps.location.isValid()){
    data.status = 1;
    data.lat = (double)gps.location.lat();
    data.lot = (double)gps.location.lng();
    // data.alt = (double)gps.altitude.meters();
  }
  else {
    data.status = 0;
  }
  if (gps.speed.isUpdated()){
    data.speed = (double)gps.speed.mps();
  }

  radio.write(&data, sizeof(TelemetryData));
  delay(10);

}
