/*A basic 6 channel transmitter using the nRF24L01 module.*/
/* Like, share and subscribe, ELECTRONOOBS */
/* http://www.youtube/c/electronoobs */

/* First we include the libraries. Download it from 
   my webpage if you donw have the NRF24 library */
 
#include <SPI.h>
#include <nRF24L01.h>             //Downlaod it here: https://www.electronoobs.com/eng_arduino_NRF24_lib.php
#include <RF24.h>              

/*Create a unique pipe out. The receiver has to 
  wear the same unique code*/
  
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!

RF24 radio(8,7); // select  CE and CSN  pins

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data;

void resetData() 
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.
    
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

void setup()
{
  Serial.begin(9600);
//  Serial.println("yo whats wrong");
  //Start everything up
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
  pinMode(2,INPUT);
  digitalWrite(2,HIGH);
  //pinMode(5,OUTPUT);
  //digitalWrite(5,HIGH);
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.
  
  data.throttle = mapJoystickValues( analogRead(A0),  0, 505, 980, true ); //play with these values so that you get 0,128,255 values from the joystick
  data.yaw      = mapJoystickValues( analogRead(A1),  0, 523, 980, true ); //analogRead(A0)*5/3.3 if u use a 5v processor
  data.pitch    = mapJoystickValues( analogRead(A2),  0, 517, 980, true );
  data.roll     = mapJoystickValues( analogRead(A3),  0, 502, 980, true );
  data.AUX1     = digitalRead(10);
  data.AUX2     = digitalRead(2);

//  Serial.print( data.throttle);
//  Serial.print(',');
//  Serial.print( data.yaw);
//  Serial.print(',');
//  Serial.print( data.pitch);
//  Serial.print(',');
//  Serial.println( data.roll);
//  Serial.print(',');
//  Serial.print( data.AUX1);
//  Serial.print(',');
//  Serial.println( data.AUX2);
  
  
  radio.write(&data, sizeof(MyData));
}
