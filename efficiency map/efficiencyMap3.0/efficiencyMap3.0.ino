#include <SPI.h>
#include <SD.h>
#include <HX711.h>
#include <RF24.h>
#include <Servo.h>

HX711 scale;
File myFile;
Servo motor;

// Input Variables --------------
const byte chipSelect = 10; //5
const byte LOADCELL_DOUT_PIN = A5;
const byte LOADCELL_SCK_PIN = A4; 
const byte PITOT_PIN = A0; //A7
const byte motor_pin = A3;

double air_density = 1.204; // air density (kg/m3)
double loadcell_scale = 2000.0;

#define SCALE_OFFSET (207762.2)
#define SCALE_SLOPE (192.0143)
// --------------------

const uint64_t address_1 = 0xE8E8F0F0E2LL;
const uint64_t address_2 = 0xE8E8F0F0E3LL;
RF24 radio(6, 9); // CE , CSN pins


struct WindTunnelData {
  char message[11];
  double dyn_force = 0;
  double pitot_airspeed = 0;
  float loop_freq = 0;  // Frequency [Hz]
};
WindTunnelData data;

struct Command {
  byte init_sd = 'n';
  byte close_input = 'n';
  byte measurements_input = 'n';
  int motor_speed = 0;
  byte restart = 'n'; //used to restart the experiment when measurements closed
};
Command command;

//Variables Needed
double pitot_offset = 0;  // offset if there is flow when starting pitot
const byte pitot_offset_times = 10; 

unsigned long prev_time = 0;

int file_offset = 0;  // This is the file offset that will be appended to the filename.



void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // Motor Configuration
  motor.attach(motor_pin);
  motor.writeMicroseconds(1000);
  delay(500);

  // Scale Init
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale(192.0143);
  // scale.set_offset(207762.2);

  // NRF BEGIN INITIALIZATION ----------------
  //Radio for nrf24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.openReadingPipe(1, address_1); //set receiving address
  radio.openWritingPipe(address_2);
  resetData();
  resetCommand();

  // PITOT BEGIN INITIALIZATION ----------------
  // We test for offset 10 times and we take the average offset
  for(int i=0; i < pitot_offset_times; ++i){
    pitot_offset += analogRead(PITOT_PIN) - (1023 / 2);
  }
  pitot_offset /= pitot_offset_times;



  //send message that you are ready
  radio.stopListening();
  strcpy(data.message, "I am ready");
  Serial.println("I am ready");
  radioSendCommands(20);

  radio.startListening();
  recvCommand();

  // SD CARD INITIALIZATION ----------------
  //waiting for command to initialize SD
  Serial.println("waiting for command to initialize SD");
  while(command.init_sd != 'y'){ 
    recvCommand(); 
  }
  Serial.println("Initializing SD card..."); //send message back too
  
  delay(1000);
  radio.stopListening();

  if (!SD.begin()) {
    Serial.println("Initialization failed!"); //send message back too
    strcpy(data.message, "init fail");
    radio.write(&data, sizeof(WindTunnelData));
    // return;
  }
  Serial.println("Initialization done."); //send message back
  strcpy(data.message, "init done");
  radio.write(&data, sizeof(WindTunnelData));


  // sdOpenFile();
  // SD CARD END INITIALIZATION

  Serial.println("Entering Main Loop");
}


void loop()
{
  //Store time
  prev_time = millis();


  // If we have closed the measurements file then don't execute the rest of the while loop.
  // We have decided we don't want any more measurements.
  if (command.close_input == 'y'){
    motor.writeMicroseconds(1000); //motor stop
    // myFile.close();
    // Serial.println("File: " + get_filename(file_offset) + " closed. Have fun :)");
    Serial.println("File Closed");
    Serial.println("Measurements Closed");
    printCommand();  //DEBUG
    
    //Check for restarting
    radio.startListening();
    while(command.restart != 'y'){
      // waiting to start again
      recvCommand(); 
    }
    
    //When we decide to restart:
    // sdOpenFile();
    command.restart = 'n';
    command.close_input = 'n';    
    command.measurements_input = 'y';

    Serial.println("Restarting in 5 seconds...");
    delay(5000);

  }
  
  radio.startListening();
  recvCommand(); 
  //waiting for command to start measurements
  while(command.measurements_input != 'y'){
    Serial.println("Waiting for command to start measurements");
    recvCommand();
  }

  //Start motor 
  motor.writeMicroseconds(command.motor_speed); //TODO: Might take a while for the motor to reach desired RPM

  // Reading pitot
  double v_read = analogRead(PITOT_PIN) - pitot_offset; 

  // if it reads below 512 we need to equate to a negative velocity
  if (v_read < 512)
    data.pitot_airspeed = -sqrt((-10000.0*((v_read / 1023.0) - 0.5)) / air_density); //maybe we could simplify these operations
  else
    data.pitot_airspeed = +sqrt((+10000.0*((v_read / 1023.0) - 0.5)) / air_density);

  // Reading Scale
  // data.dyn_force = scale.get_units(); //faster than 330 Hz
  if (scale.is_ready()){
    data.dyn_force = (scale.read() - SCALE_OFFSET)/(SCALE_SLOPE);
  }


  // Sending to control station
  // Serial.print("/*"); // Format for Serial Studio
  // Serial.print(millis());
  // Serial.print(",");
  // Serial.print(command.motor_speed);
  // Serial.print(",");
  // Serial.print(data.dyn_force); // Reading minus tare and divided by calibration parameter
  // Serial.print(",");
  // Serial.print(data.pitot_airspeed);
  // Serial.print("*/\n");
  Serial.flush();

  //Storing Freq
  data.loop_freq = (float)(1000/(millis() - prev_time)); // Frequency [Hz]
  
  radio.stopListening();
  radio.write(&data, sizeof(WindTunnelData));

  // Store data in SD card
  // firt we check that the file is open and working

  // if (myFile) {
  //   myFile.print(data.dyn_force);
  //   myFile.print(",");
  //   myFile.print(data.pitot_airspeed);
  //   myFile.print("\n");
  // } else {
  //   Serial.println("There seems to be an error with the SD File" + get_filename(file_offset));
  // }


}

void recvCommand(){
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
  strcpy(data.message, "");
  data.dyn_force = 0;
  data.pitot_airspeed = 0;
}
void resetCommand() {
  command.init_sd = 'n'; // Should be 'y' in order to start the SD card initialization...
  command.close_input = 'n'; // If 'y' then we should close the measurements file...
  command.measurements_input = 'n'; // Should be 'y' in order to start measuring...
}
//Printing the contents of the command struct 
//for debugging purposes
void printCommand(){
  Serial.println("Printing Command Sent ----------------");
  Serial.print("[byte] init_sd = ");
  Serial.println(command.init_sd);
  Serial.print("[byte] close_input = ");
  Serial.println(command.close_input);
  Serial.print("[byte] measurements_input = ");
  Serial.println(command.measurements_input);
  Serial.println("--------------------------------------");
}

void sdOpenFile(){
  // Find latest file offset for the filename  
  while (SD.exists(get_filename(file_offset))) ++file_offset;

  // try opening the file...
  myFile = SD.open(get_filename(file_offset), FILE_WRITE);

  if (myFile)
    Serial.println("Measurements will be saved in file: " + get_filename(file_offset));
  else 
    Serial.println("Something went wrong while trying to open the file" + get_filename(file_offset));
}

//Function to send the radio commands 
//multiple times for redundancy if needed
void radioSendCommands(int times){
  radio.stopListening();
  for (int i = 0; i < times; i++){
    radio.write(&data, sizeof(WindTunnelData));
    delay(10);
  }
}