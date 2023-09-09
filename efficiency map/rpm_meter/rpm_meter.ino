int rpm;
unsigned long int prev_time;
volatile float objects = 0;
unsigned long int time;

#include <Servo.h>

Servo motor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  motor.attach(A3);

  //Wire.begin();
  // Wire.begin(8);
  // Wire.onReceive(receiveEvent);
  attachInterrupt(digitalPinToInterrupt(2), count, FALLING);
  delay(1000);
  pinMode(2, INPUT);
}

void count() {
  ++objects;
}

// void receiveEvent(int bytes) {
//   Wire.write(rpm); // respond with message of 6 bytes as expected by master
// }

void loop () {

  motor.writeMicroseconds(1400);
  // while loop is on delay interrupts are happening...
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(2));
  // time = 2000 but whatever...
  time = millis() - prev_time;
  // Serial.print("Objects detected: ");
  // Serial.println(objects);
  rpm = objects/time * 60000;
  prev_time = millis();
  objects = 0;
  Serial.print("RPM: ");
  Serial.println(rpm);
//  Wire.beginTransmission(9); // transmit to device #9
//  Wire.write(rpm);              // sends x 
//  Wire.endTransmission();    // stop transmittin
  
  attachInterrupt(digitalPinToInterrupt(2), count, FALLING);
}
