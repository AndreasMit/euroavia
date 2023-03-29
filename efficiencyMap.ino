#include <HX711.h>
#include <SPI.h>
#include <SD.h>


HX711 scale;

float airspeed = 0;
float d = 1.204; // air density (kg/m3)

int offset = 0; // offset if there is flow when starting pitot
int offset_times = 10; 

const int SDpin = 10; // SD CS pin 
File myFile; // we create a file to store the data

void setup(){
  Serial.begin(38400); // too high for pitot maybe??
  scale.begin(2,13); // hx711 pins 

// initialise SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("test.csv", FILE_WRITE); // opens or creates SD file 


  scale.set_scale(2000.0); // adding calibration factor to dyno (reading/known weight) (not determined yet)
  scale.tare(); // resets to zero

// we test for offset 10 times and we take the average offset

  for(int i=0; i<offset_size; i++){
    offset += analogRead(A0) - (1023/2);
  }
  offset /= offset_size;
}


void loop(){
  float v_read = analogRead(A0)-offset; //pitot voltage read minus offset

// if it reads below 512 we need to equate to a negative velocity

  if(v_read<512){
    airspeed = -sqrt((-10000.0*((v_read/1023.0)-0.5))/d);
  }else{
    airspeed = sqrt((-10000.0*((v_read/1023.0)-0.5))/d);
  }
  

  // pitot should delay 10secs for stability
  Serial.print(scale.get_units(),3); // reading minus tare and divided by calibration parameter
  Serial.print("\t");
  Serial.print(airspeed);
  Serial.print("\n")

// store data in SD card
// firt we check that the file is open and working
   if (myFile) {
    myFile.println(scale.get_units(),3);
    myFile.print("\t");
    myFile.print(airspeed);
    myFile.print("\n")
    myFile.close(); // ensures the data is saved
    
  } else {
    Serial.println("error opening test.txt");
  }
}