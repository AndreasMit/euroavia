#include <SPI.h>

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

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // while (!Serial.available());

  // NRF BEGIN INITIALIZATION

  //Radio for nrf24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.openReadingPipe(1, address_2); //set receiving address
  radio.openWritingPipe(address_1);
  resetCommand();
  resetData();
  
  // NRF END INITIALIZATION
  
  // wait for confirmation from the other arduino that it is ready to begin
  radio.startListening();
  recvData();
  //receive and print message
  while (data.message == ""){
      recvData();
      Serial.println("Waiting for wind tunnel arduino...")
  }
  Serial.println(data.message);
  resetData();

  // SD CARD BEGIN INITIALIZATION - send this to 
  Serial.println("Should I initialize the SD Card?");
  // Once 'y' is typed in the Serial monitor the program should go on...
  while (init_sd_input != 'y') {
    if (Serial.available())
      command.init_sd = Serial.read();
  }
  delay(1000);
  //send command with nrf - init sd
  radio.stopListening();
  radio.write(&command, sizeof(Command));

  radio.startListening();
  recvData();
  while (data.message == ""){
      recvData();
      Serial.println("Waiting for SD initialization...")
  }
  Serial.println(data.message);
  resetData();
}

void loop(){
  // If 'y' is typed in the Serial monitor then we should start getting measurements...
  while(command.measurements_input != 'y') {
    if (Serial.available())
        command.measurements_input = Serial.read();
  }
  //send command with nrf - start input
  radio.stopListening();
  radio.write(&command, sizeof(Command));

  while(Serial.available()) { 
    command.close_input = Serial.read();
    if (command.close_input != 'y') continue;
    esle {
      radio.write(&command, sizeof(Command));
    }

  //receive data from the receiver
  radio.startListening();
  recvData();

  // Sending to Serial Studio
  Serial.print("/*"); // Format for Serial Studio
  Serial.print(data.dyn_force); // Reading minus tare and divided by calibration parameter
  Serial.print(",");
  Serial.print(data.pitot_airspeed);
  Serial.print("*/\n");
}

void recvData(){
    if ( radio.available() ){
        radio.read(&data, sizeof(MyData));
    } 
}

void resetCommand() {
  command.init_sd = 'n'; // Should be 'y' in order to start the SD card initialization...
  command.close_input = 'n'; // If 'y' then we should close the measurements file...
  command.measurements_input = 'n'; // Should be 'y' in order to start measuring...
}
void resetData() {
  data.message = "";
  data.dyn_force = 0;
  data.pitot_airspeed = 0;
}