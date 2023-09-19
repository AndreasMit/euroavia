#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t nrf_address = 0xE8E8F0F0E1LL; 

RF24 radio(6, 9); // CE , CSN pins

/*
  status:
    0 - no signal
    1 - signal
    2 - timed out

*/
struct TelemetryData{
  int status = -1;
  double lat = 0.0;
  double lot = 0.0;
  double speed = 0.0;
  // double alt = 0.0;
  float freq = 0;
};
TelemetryData data;


void setup(){
  Serial.begin(9600);
  delay(1500);

  //Radio for nrf24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  // radio.openWritingPipe(GS_nrf_address);
  radio.openReadingPipe(1, nrf_address); //set receiving address
  radio.startListening(); //Set module as receiver

  Serial.print("Struct Size: "); Serial.println(sizeof(TelemetryData));

}

void loop(){
  if ( radio.available() ){
    radio.read(&data, sizeof(TelemetryData));
    printNRF();
  } 
  delay(100);
}

void printNRF(){
  Serial.print("Status: "); Serial.println(data.status);
  Serial.print("Lat: "); Serial.println(data.lat);
  Serial.print("Lng: "); Serial.println(data.lot);
  Serial.print("Speed: "); Serial.println(data.speed);
  // Serial.print("Alt: "); Serial.println(data.alt);
  Serial.print("Freq: "); Serial.println(data.freq);
  Serial.println();
}