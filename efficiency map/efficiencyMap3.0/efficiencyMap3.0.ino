#include <SPI.h>
#include <SD.h>
#include <HX711.h>

HX711 scale;
File myFile;

const uint64_t address_1 = 0xE8E8F0F0E2LL;
const uint64_t address_2 = 0xE8E8F0F0E3LL;
RF24 radio(6, 9); // CE , CSN pins

struct MyData {
  char[10] message;
  double dyn_force;
  double pitot_airspeed;
};
MyData data;

struct Command {
  byte init_sd;
  byte close_input;
  byte measurements_input;
};
Command command;

// Input Variables --------------
const byte chipSelect = 10;
const byte LOADCELL_DOUT_PIN = 2; 
const byte LOADCELL_SCK_PIN = 13;
const byte PITOT_PIN = A0;

double air_density = 1.204; // air density (kg/m3)
double loadcell_scale = 2000.0;
// --------------------


// double pitot_airspeed = 0;
double pitot_offset = 0; // offset if there is flow when starting pitot
const byte pitot_offset_times = 10; 
// double dyn_force = 0; //  force measured from dynamometer

// This is the file offset that will be appended to the filename.
int file_offset = 0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // while (!Serial.available());

  // NRF BEGIN INITIALIZATION

  //Radio for nrf24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.openReadingPipe(1, address_1); //set receiving address
  radio.openWritingPipe(address_2);
  resetData();
  resetCommand();
  
  // NRF END INITIALIZATION

  // PITOT BEGIN INITIALIZATION

  // Setting up PITOT
  // We test for offset 10 times and we take the average offset
  for(int i=0; i < pitot_offset_times; ++i){
    pitot_offset += analogRead(PITOT_PIN) - (1023 / 2);
  }
  pitot_offset /= pitot_offset_times;

  // PITOT END INITIALIZATION

  setup2();
}

void setup2(){
  //send message that you are ready
  radio.stopListening();
  data.message = "I am ready";
  radio.write(&data, sizeof(MyData));

  radio.startListening();
  recvCommand();
  while(command.init_sd != 'y'){ 
    recvCommand(); //waiting for command to start measurements
  }
  Serial.println("Initializing SD card..."); //send message back too
  
  delay(1000);
  radio.stopListening();
  if (!SD.begin()) {
    Serial.println("Initialization failed!"); //send message back too
    data.message = 'init faill';
    radio.write(&data, sizeof(MyData));
    return;
  }
  Serial.println("Initialization done."); //send message back
  data.message = 'init donee';
  radio.write(&data, sizeof(MyData));

  // Find latest file offset for the filename  
  while (SD.exists(get_filename(file_offset))) ++file_offset;

  // try opening the file...
  myFile = SD.open(get_filename(file_offset), FILE_WRITE);

  // if file is opened
  if (myFile) //don't send these. it's fine
    Serial.println("Measurements will be saved in file: " + get_filename(file_offset));
  else 
    Serial.println("Something went wrong while trying to open the file" + get_filename(file_offset));

  // SD CARD END INITIALIZATION
}

void loop()
{
  // If we have closed the measurements file then don't execute the rest of the while loop.
  // We have decided we don't want any more measurements.
  if (command.close_input == 'y') return;
  
  radio.startListening();
  recvCommand(); 
  //waiting for command to start measurements
  while(command.measurements_input != 'y'){
    recvCommand();
  }

  if(command.close_input == 'y') { 
    myFile.close();
    Serial.println("File: " + get_filename(file_offset) + " closed. Have fun :)");
  }

  // Reading pitot
  double v_read = analogRead(PITOT_PIN) - pitot_offset; 

  // if it reads below 512 we need to equate to a negative velocity
  if (v_read < 512)
    data.pitot_airspeed = -sqrt((-10000.0*((v_read / 1023.0) - 0.5)) / air_density); //maybe we could simplify these operations
  else
    data.pitot_airspeed = +sqrt((+10000.0*((v_read / 1023.0) - 0.5)) / air_density);

  // Reading Scale
  data.dyn_force = scale.get_units();

  // Sending to control station
  Serial.print("/*"); // Format for Serial Studio
  Serial.print(data.dyn_force); // Reading minus tare and divided by calibration parameter
  Serial.print(",");
  Serial.print(data.pitot_airspeed);
  Serial.print("*/\n");

  radio.stopListening();
  radio.write(&data, sizeof(MyData));

  // Store data in SD card
  // firt we check that the file is open and working

  if (myFile) {
    myFile.print(data.dyn_force);
    myFile.print(",");
    myFile.print(data.pitot_airspeed);
    myFile.print("\n");
  } else {
    Serial.println("There seems to be an error with the SD File" + get_filename(file_offset));
  }


}

void recvCommand(){
  // radio.startListening();
    if ( radio.available() ){
        radio.read(&command, sizeof(Command));
    } 
}

// This function returns the correct filename 
//of the measurements file given a certain offset.
String get_filename(int file_offset) {
  return "test" + String(file_offset) + ".csv";
}

void resetData() {
  data.message = "";
  data.dyn_force = 0;
  data.pitot_airspeed = 0;
}
void resetCommand() {
  command.init_sd = 'n'; // Should be 'y' in order to start the SD card initialization...
  command.close_input = 'n'; // If 'y' then we should close the measurements file...
  command.measurements_input = 'n'; // Should be 'y' in order to start measuring...
}