#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*  Address through which two modules communicate.
    The pipe address does not have to be “alex0”, 
    it can be any 5-character string such as “0as1s” 
    as long as the transmitter and receiver both use the same address.
*/
// const byte address[6] = "alex0"; 
const uint64_t address = 0xE8E8F0F0E1LL; 

RF24 radio(6, 9); // CE , CSN pins

//Arduino Pins
int right_switch_pin = A5;
int left_switch_pin = A0;
int roll_pin = A4;
int pitch_pin = A3;
int yaw_pin = A2;
int throttle_pin = A1;
int buzzer_pin = 3;

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 x 8 bits channels
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data;


void setup()
{
  //Start everything up
  Serial.begin(9600);
  //NRF Radio Communication
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  //radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  resetRFData();

  //Buzzer
  pinMode(buzzer_pin, OUTPUT);
  //Button AUX1
  pinMode(right_switch_pin, INPUT_PULLUP);
  pinMode(left_switch_pin, INPUT_PULLUP);

  delay(200);
  //digitalWrite(17, HIGH); // Turning off builtin led in arduino

  starting_beep(3, 200, 200, 1000);
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.  
  data.yaw      = mapJoystickValues( analogRead(yaw_pin),  195, 522, 830, false );
  data.pitch    = mapJoystickValues( analogRead(pitch_pin), 213, 526, 825, true );
  data.roll     = mapJoystickValues( analogRead(roll_pin), 205, 530, 845, false );
  data.throttle = mapJoystickValues( analogRead(throttle_pin), 228, 526, 829, true );
  data.AUX1     = 1 - digitalRead(left_switch_pin);
  data.AUX2     = digitalRead(right_switch_pin);        
  
  radio.write(&data, sizeof(MyData));

  serialPrintDebug();

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

void resetRFData() {
    data.throttle = 0;
    data.yaw = 127;
    data.pitch = 127;
    data.roll = 127;
    data.AUX1 = 0;
    data.AUX2 = 0;
}

//Sending status to Serial Port for Debugging purposes
void serialPrintDebug(){

  /* ORDER:
      time(1) [sec], data.throttle(2), analogRead(throttle_pin)(3),
      data.roll(4), analogRead(roll_pin)(5)
      data.pitch(6), analogRead(pitch_pin)(7),
      data.yaw(8), analogRead(yaw_pin)(9),
      data.AUX1(10), data.AUX2(11)
  */
  Serial.print("/*");
  Serial.print(millis()/1000);
  Serial.print(",");
  Serial.print(data.throttle);
  Serial.print(",");
  Serial.print(analogRead(throttle_pin));
  Serial.print(",");
  Serial.print(data.roll);
  Serial.print(",");
  Serial.print(analogRead(roll_pin));
  Serial.print(",");
  Serial.print(data.pitch);
  Serial.print(",");
  Serial.print(analogRead(pitch_pin));
  Serial.print(",");
  Serial.print(data.yaw);
  Serial.print(",");
  Serial.print(analogRead(yaw_pin));
  Serial.print(",");
  Serial.print(data.AUX1);
  Serial.print(",");
  Serial.print(data.AUX2);
  Serial.println("*/");

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