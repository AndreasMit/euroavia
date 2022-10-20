/*A basic 6 channel transmitter using the nRF24L01 module.*/
/* Like, share and subscribe, ELECTRONOOBS */
/* http://www.youtube/c/electronoobs */

/* First we include the libraries. Download it from 
   my webpage if you donw have the NRF24 library */
 
#include <SPI.h>
#include <nRF24L01.h>             //Downlaod it here: https://www.electronoobs.com/eng_arduino_NRF24_lib.php
#include <RF24.h>  
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
            
#define OLED_RESET     3 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/*Create a unique pipe out. The receiver has to 
  wear the same unique code*/
  
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!
const uint64_t pipeIn = 0xE8E8F0F1E1LL;

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
struct RF24Ack{
  int bat; //battery voltage
};

MyData data;
RF24Ack ack;

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
void resetAck(){
  ack.bat = 188;
}
void setup()
{
  Serial.begin(9600);
  
  //Start everything up
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.openReadingPipe(1, pipeIn);
  resetData();
  resetAck();
  pinMode(2,INPUT);
  digitalWrite(2,HIGH);
  //pinMode(5,OUTPUT);
  //digitalWrite(5,HIGH);
  display.display();
  delay(1000);
  display.clearDisplay();
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
  radio.stopListening();
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

  //reading from drone
  float battery;
  radio.startListening();
  if(radio.available()){
    radio.read(&ack, sizeof(RF24Ack));
  }
//  if(ack.bat<255){
//    ack.bat+=1;
//  }
  battery = (ack.bat*5.0/255);             //from analog to voltage
  battery = 100*(battery-3.5)/(4.2-3.5); //4.2-3.5 volt
  //print to screen the battery message
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Battery:"));
  display.print(battery);
  display.println(F(" %"));
  display.display();
//  delay(50);
  

}
