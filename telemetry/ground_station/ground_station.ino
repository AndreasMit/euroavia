//     Euroavia - ground station
//
// the setup consists of an arduino with NRF sensor 
// connected trough serial port to a computer 
// Serial studio is running on that computer
// we publish csv format of data and then serial studio converts to json format
// to visualize that data with widgets

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipe = 0xE8E8F0F0E1LL;     
RF24 radio(9, 10);  // CE ,CSN

int alt_offset = 0;
bool first_time = true;

struct Payload {
  //BATTERY
  short int battery;
  
  //SIGNAL STRENGTH
  
  //GPS
  //  NeoGPS::time_t  dateTime;
  int32_t latitude;       //4 bytes
  int32_t longitude; 
  short int altitude;     //2 bytes
  short int speed_kph;         
  
  //IMU
  short int AccX;
  short int AccY;
  short int AccZ;
  short int GyroX;   //rads
  short int GyroY;
  short int GyroZ;
  
  //BARO
  short int alt;
  short int temp;
  
  //PITTOT
  
};

Payload data;

void setup()
{
  Serial.begin(9600);
  
  //We reset the received values
  data.latitude = 0;
  data.longitude = 0;
  data.altitude = 0;
  data.speed_kph = 0;
  data.AccX = 0;
  data.AccY = 0;
  data.AccZ = 0;
  data.GyroX = 0;
  data.GyroY = 0;
  data.GyroZ = 0;
//  data.MagX = 0;
//  data.MagY = 0;
//  data.MagZ = 0;
//  data.temp = 0;
  data.alt = 0;
 
  //Once again, begin and radio configuration
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipe); //or 0??
  
  //radio.setPALevel(RF24_PA_MIN);
  
  //We start the radio comunication1
  radio.startListening();
}

void loop(){
  //Receive the radio data
  if(radio.available()) { //check again if this is the right way to do it/ or while?
    radio.read(&data, sizeof(Payload));
  }
  
  double longitude = data.longitude;
  longitude/=10000000;
  double latitude = data.latitude;
  latitude/=10000000;
  
  Serial.print("/*");      // frame start sequence (json)
  Serial.print(millis());  //ms since system started
  Serial.print(",");
  Serial.print(data.battery/107.4); //battery //calibrate *2 (voltage div) *5 (Vref) /1024 resolution
  Serial.print(",");
  
  Serial.print(longitude,7); //show 7 decimal digits
  Serial.print(",");
  Serial.print(latitude,7);
  Serial.print(",");
  Serial.print(data.altitude);
  Serial.print(",");
  Serial.print(data.speed_kph*(1.0)/100);
  Serial.print(",");
  
  Serial.print(data.AccX*(1.0)/100);
  Serial.print(",");
  Serial.print(data.AccY*(1.0)/100);
  Serial.print(",");
  Serial.print(data.AccZ*(1.0)/100);
  Serial.print(",");
  Serial.print(data.GyroX*(1.0)/100);
  Serial.print(",");
  Serial.print(data.GyroY*(1.0)/100);
  Serial.print(",");
  Serial.print(data.GyroZ*(1.0)/100);
  Serial.print(",");
  
  if(data.altitude != 0 and first_time){
    alt_offset = data.altitude;
    first_time = false;
  }
  Serial.print(data.alt*(1.0)/100 + alt_offset);   // +-300 meters from ground station
  Serial.print(",");
  Serial.print(data.temp*(1.0)/100); //temperature
  Serial.print(",");
  
  Serial.println("35");//air speed later 
//  Serial.print("*/"); // frame end sequence (json)
}
