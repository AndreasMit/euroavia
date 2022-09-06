//  Euroavia - data logging 
//
//  this script is using libraries given under the GNU General Public License/ BSD license.
//  purpose of this script:
//      read and transmit to ground station the following values:
//      -airspeed
//      -groundspeed
//      -altitude
//      -coordinates
//      -angular and liner accelerations
//      -temperature
//      -angle of attack
//
//  Written by Mitakidis Andreas

//the schematic is given in the different file.
//the connections we use are:
//IMU , BARO (I2C)  - (SDA,SCL) -> (A4,A5)
//NRF (SPI)         - (CNS,CE)  -> (10,9)
//GPS (SERIAL)      - (TX,RX)   -> (RX,TX)
//PITTOT (ANALOG)   - (A0)
//
// IMU -> MPU6050
// BARO -> BMP280
// GPS -> UBLOX NEO6M
// NRF -> NRF24L01 + LNA
// PITTOT -> MPXV7002DP ??

//################################# LIBRARIES #########################################
#include <NMEAGPS.h>
//for the gps we use interrupts and thus we have edited the GPS_port and NMEAGPS_cfg files
//dont forget to add to src/ the Hardware Serial files.
//also edited MPU6050_tockn.cpp: deleted Serial.println lines
//same for Adafruit_sensor.cpp which is used by bmp280 sensor (called in Adafruit_BMP280.h)

/*!! reduce object size of fix to reduce gps latency*/

#include <GPSport.h>
#include <Streamers.h>
#include <MPU6050_tockn.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BMP280.h>

#ifndef NMEAGPS_INTERRUPT_PROCESSING
  #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

//##############################  DEFINITIONS   ########################################
MPU6050 mpu6050(Wire);
static NMEAGPS   gps;
static gps_fix fix;
const uint64_t pipe = 0xE8E8F0F0E1LL;  //address of nrf channel transmission
RF24 radio(9, 10);    //CE,CSN
Adafruit_BMP280 bmp; // use I2C interface
int alt_0 = 0;
bool first_time = true;

//PAYLOAD
// The sizeof this struct should not exceed 32 bytes
struct Payload {
  //BATTERY - 2 bytes
  short int battery;
  
  //SIGNAL STRENGTH
  
  //GPS - 12bytes
  //  NeoGPS::time_t  dateTime;
  int32_t latitude;       //4 bytes
  int32_t longitude; 
  short int altitude;     //2 bytes
  short int speed_kph;         
  
  //IMU - 12 bytes
  short int AccX;
  short int AccY;
  short int AccZ;
  short int GyroX;   //rads
  short int GyroY;
  short int GyroZ;
  
  //BARO - 4 bytes
  short int alt; //use int for alt and remove temperature
  short int temp;
  
  //PITTOT - 2 bytes
//  short int airspeed;
};
//exceeding 32 bytes(limit) => data casting to short int = 2 bytes, 
//float = 4 bytes

Payload data;

// GPSisr :stores the new data in the fix data structure
static void GPSisr( uint8_t c ){
  gps.handle( c );
}


//####################### SETUP ###################################
void setup(){
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;
    
  //for IMU
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  //for BMP
  if (!bmp.begin()) {
    DEBUG_PORT.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //for GPS
  DEBUG_PORT.print( F("NMEA_isr.INO: started\n") );
  DEBUG_PORT.print( F("fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("NMEAGPS object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  
  trace_header( DEBUG_PORT );
  
  DEBUG_PORT.flush();
  
  gpsPort.attachInterrupt( GPSisr );
  gpsPort.begin( 9600 );

  //for NRF
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe);
}


//############################## LOOP ####################################
void loop(){
  // it seems like we have a problem when losing and finding again gps signal-> we want to continue data transmission 
  //of others sensor measurements
  mpu6050.update();
  //GPS
  if (gps.available()) {
    //find the data type and names of fix components in the GPSfix.h file
    fix = gps.read();
    //data.dateTime = fix.dateTime;
    data.latitude = fix.latitudeL();
    data.longitude = fix.longitudeL();
    data.altitude = fix.altitude();  //no decimal resolution - imprecise
    data.speed_kph = fix.speed_kph()*100;
  }
  //IMU
  data.AccX = mpu6050.getAccX()*100;
  data.AccY = mpu6050.getAccY()*100;
  data.AccZ = mpu6050.getAccZ()*100;

  data.GyroX = mpu6050.getAngleX()*100;
  data.GyroY = mpu6050.getAngleY()*100;
  data.GyroZ = mpu6050.getAngleZ()*100;
//  data.GyroX = mpu6050.getGyroX()*100;
//  data.GyroY = mpu6050.getGyroY()*100;
//  data.GyroZ = mpu6050.getGyroZ()*100;  
  
  //BMP
  if(first_time){
    alt_0 = bmp.readAltitude(1024);
    first_time = false;
  }
  data.alt = (bmp.readAltitude(1024) - alt_0)*100; //1013.25 original
  data.temp = bmp.readTemperature()*100;
  
  //BATTERY 
  data.battery = analogRead(A1);
  
  //PITTOT
  //data.airspeed = analogRead(A0);
  
  //tramsmit data
  radio.write(&data, sizeof(Payload));
}

//delete later
//  if (gps.overrun()) {
//    gps.overrun( false );
//    DEBUG_PORT.println( F("DATA OVERRUN: took too long to print GPS data!") );
//  } //i dont think i need this one
