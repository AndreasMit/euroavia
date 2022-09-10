#include <Wire.h>

long acc_x, acc_y, acc_z;
float angle_x, angle_y, angle_z;


void setup() {

  Serial.begin(9600);
  Wire.begin();

  mpu6050_setup();  

}

void loop() {
  
  recordAccelRegisters();
  calc_angles();
  print_results();
  delay(85);

}

void mpu6050_setup(){
  //Setting sensor into reading mode (awaken from sleep)
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  
  //Setting Accelerometer Mode
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 

  //Configuring DLPF (Digital Low Pass Filter)
  Wire.beginTransmission(0b1101000); //Choosing 4th filter available (Sec. 4.6 Register 26)
  Wire.write(0x1A);
  Wire.write(0b00000100);
  Wire.endTransmission();

  delay(200);
}

void recordAccelRegisters() {

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  acc_x = Wire.read()<<8|Wire.read(); //Store first two bytes into acc_x
  acc_y = Wire.read()<<8|Wire.read(); //Store middle two bytes into acc_y
  acc_z = Wire.read()<<8|Wire.read(); //Store last two bytes into acc_z

}

void calc_angles(){

  //Converting ACC readings into angle measurements
  angle_x = atan2(acc_y, acc_z) * 180 / PI;
  angle_y = atan2(acc_x, acc_z) * 180 / PI;

}

void print_results(){
  Serial.println(angle_z);
}