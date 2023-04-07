#include <HX711.h>
#include <SPI.h>
#include <SD.h>

HX711 scale;
File myFile; 

// Input Variables --------------

const int SDpin = 10; // SD CS pin 
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 13;
const int PITOT_PIN = A0;

float air_density = 1.204; // air density (kg/m3)
float loadcell_scale = 2000.0;

char filename[] = "Measurements.csv";

// ------------------------------

float airspeed = 0;
int pitot_offset = 0; // offset if there is flow when starting pitot
int pitot_offset_times = 10; 


void setup(){
  Serial.begin(38400); // too high for pitot maybe??

  // Initialise SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open(filename, FILE_APPEND); // opens or creates SD file 
  myFile.println("--------\n")
  myFile.close()

  //Setting up HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // hx711 pins 
  scale.set_scale(loadcell_scale); // adding calibration factor to dyno (reading/known weight) (not determined yet)
  scale.tare(); // resets to zero

  //Setting up PITOT
  // we test for offset 10 times and we take the average offset
  for(int i=0; i<pitot_offset_times; i++){
    pitot_offset += analogRead(PITOT_PIN) - (1023/2);
  }
  pitot_offset /= pitot_offset_times;

}


void loop(){

  // Reading pitot
  float v_read = analogRead(PITOT_PIN)-pitot_offset; 

  // if it reads below 512 we need to equate to a negative velocity
  if(v_read<512){
    airspeed = -sqrt((-10000.0*((v_read/1023.0)-0.5))/air_density);
  }else{
    airspeed = sqrt((-10000.0*((v_read/1023.0)-0.5))/air_density);
  }
  
  // pitot should delay 10secs for stability
  Serla.print("/*") //Format for Serial Studio
  Serial.print(scale.get_units(),3); // reading minus tare and divided by calibration parameter
  Serial.print(",");
  Serial.print(airspeed);
  Serial.print("*/\n")

  // Store data in SD card
  // firt we check that the file is open and working
  myFile = SD.open(filename, FILE_APPEND);
  if (myFile) {
    myFile.println(scale.get_units(),3);
    myFile.print("\t");
    myFile.print(airspeed);
    myFile.print("\n")
    myFile.close(); // ensures the data is saved
    
  } else {
    Serial.println("Error opening file for writing!");
  }
}
