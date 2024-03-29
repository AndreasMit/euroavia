#include <SPI.h>
#include <SD.h>
#include <HX711.h>
#include <Servo.h>

HX711 scale;
File myFile;
Servo motor;

// Input Variables --------------
const byte chipSelect = 5;
const byte LOADCELL_DOUT_PIN = A4; 
const byte LOADCELL_SCK_PIN = A5;
const byte PITOT_PIN = A7;
const byte motor_pin = A3;

double air_density = 1.204; // air density (kg/m3)
double loadcell_scale = 2000.0;
// --------------------


double pitot_airspeed = 0;
double pitot_offset = 0; // offset if there is flow when starting pitot
const byte pitot_offset_times = 10; 
double dyn_force = 0; //  force measured from dynamometer

// This is the file offset that will be appended to the filename.
int file_offset = 0;

// Arduino Control Through Serial Monitor
char init_sd_input = 'n'; // Should be 'y' in order to start the SD card initialization...
char measurements_input = 'n'; // Should be 'y' in order to start measuring...
char close_input = 'n'; // If 'y' then we should close the measurements file...


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // while (!Serial.available());

  motor.attach(motor_pin);
  motor.writeMicroseconds(1000);
  delay(500);

  // SD CARD BEGIN INITIALIZATION
  Serial.println("Should I initialize the SD Card?");

  // Scale Init
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Once 'y' is typed in the Serial monitor the program should go on...
  while (init_sd_input != 'y') {
    if (Serial.available())
      init_sd_input = Serial.read();
  }




  Serial.println("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // Find latest file offset for the filename  
  while (SD.exists(get_filename(file_offset))) ++file_offset;

  // try opening the file...
  myFile = SD.open(get_filename(file_offset), FILE_WRITE);

  // if file is opened
  if (myFile)
    Serial.println("Measurements will be saved in file: " + get_filename(file_offset));
  else 
    Serial.println("Something went wrong while trying to open the file" + get_filename(file_offset));

  // SD CARD END INITIALIZATION


  // PITOT BEGIN INITIALIZATION

  // Setting up PITOT
  // We test for offset 10 times and we take the average offset
  for(int i=0; i < pitot_offset_times; ++i){
    pitot_offset += analogRead(PITOT_PIN) - (1023 / 2);
  }
  pitot_offset /= pitot_offset_times;

  // PITOT END INITIALIZATION

}

void loop()
{
  // If we have closed the measurements file then don't execute the rest of the while loop.
  // We have decided we don't want any more measurements.
  if (close_input == 'y'){
    motor.writeMicroseconds(1000);
    return;
  } 



  // If 'y' is typed in the Serial monitor then we should start getting measurements...
  while(measurements_input != 'y') {
    if (Serial.available())
        measurements_input = Serial.read();
  }
  motor.writeMicroseconds(1400);


  while(Serial.available()) { //maybe this is not right
    close_input = Serial.read();
    if (close_input != 'y') continue; //check whether it skips the whole while loop and not just this loop

    // if close_input == 'y'
    myFile.close();
    Serial.println("File: " + get_filename(file_offset) + " closed. Have fun :)");
  }

  // Reading pitot
  double v_read = analogRead(PITOT_PIN) - pitot_offset; 

  // if it reads below 512 we need to equate to a negative velocity
  if (v_read < 512)
    pitot_airspeed = -sqrt((-10000.0*((v_read / 1023.0) - 0.5)) / air_density); //maybe we could simplify these operations
  else
    pitot_airspeed = +sqrt((+10000.0*((v_read / 1023.0) - 0.5)) / air_density);

  // Reading Scale
  dyn_force = scale.get_units();

  // Sending to Serial Studio
  Serial.print("/*"); // Format for Serial Studio
  Serial.print(dyn_force); // Reading minus tare and divided by calibration parameter
  Serial.print(",");
  Serial.print(pitot_airspeed);
  Serial.print("*/\n");

  // Store data in SD card
  // firt we check that the file is open and working

  if (myFile) {
    myFile.print(dyn_force);
    myFile.print(",");
    myFile.print(pitot_airspeed);
    myFile.print("\n");
  } else {
    Serial.println("There seems to be an error with the SD File" + get_filename(file_offset));
  }


}




// This function returns the correct filename 
//of the measurements file given a certain offset.
String get_filename(int file_offset) {
  return "test" + String(file_offset) + ".csv";
}

