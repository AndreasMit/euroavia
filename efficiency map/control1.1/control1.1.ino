#include <RF24.h>
// #include <RF24_config.h>
// #include <nRF24L01.h>
#include <SPI.h>

const uint64_t address_1 = 0xE8E8F0F0E2LL; //Transmitting address
const uint64_t address_2 = 0xE8E8F0F0E3LL; //Receiving address
RF24 radio(9, 10); // CE , CSN pins

//Declaring the function manually cause i had trouble with the default value not being recognised
void radioSendCommands(int times = 1);

// Struct to hold the measurements received from the wind tunnel
struct WindTunnelData {
  char message[11];
  double dyn_force = 0;
  double pitot_airspeed = 0;
  float loop_freq = 0;  // Frequency [Hz]
  float voltage = 0;
  float rpm = 0;
};
WindTunnelData data;

// Struct to hold commands sent to control the module inside the tunnel from the serial monitor
struct Command {
  byte init_sd = 'n';
  byte close_input = 'n';
  byte measurements_input = 'n';
  int motor_speed = 0;
  byte restart = 'n'; //used to restart the experiment when measurements closed
};
Command command;

//Variables needed
String date_time = "";
unsigned long currentMillis = millis();


void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600); delay(1000);

  // Getting time and date from user
  Serial.println("Enter time and date in the following format: 2023-08-30 14:25:45");
  readDateTimeFromSerial();

  // NRF BEGIN INITIALIZATION ----------------
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.openReadingPipe(1, address_2); //set receiving address
  radio.openWritingPipe(address_1);
  resetData();
  // NRF END INITIALIZATION ----------------
  
  // wait for confirmation from the other arduino that it is ready to begin
  radio.startListening();
  recvData();

  //receive and print message
  delay(1000); 
  Serial.println("Waiting for wind tunnel arduino...");
  while (strcmp(data.message, "I am ready") != 0){
      recvData();
  }
  Serial.print("Responded: "); Serial.println(data.message);
  resetData();

  // SD CARD BEGIN INITIALIZATION ----------------
  Serial.println("Should I initialize the SD Card?");  // Once 'y' is typed in the Serial monitor the program should go on...
  
  Serial.flush();
  while (command.init_sd != 'y') {
    while (Serial.available() == 0) {
      // Wait for user input
    }
    
    command.init_sd = Serial.read();
    Serial.flush();
  }
  delay(1000);

  //send command with nrf - init sd
  radioSendCommands();

  // printCommand();
  
  radio.startListening();
  recvData();

  Serial.println("Waiting for SD initialization...");

  while (strcmp(data.message, "init done") != 0){ // data.message initialized somewhere?
      recvData();
      // Serial.println(data.message);
  }
  Serial.println("SD Card Initialized");
  resetData();

  //Asking user for motor speed input
  askForMotorSpeed();

  Serial.println("Format: Time[ms] / MotorSpeed [1000 - 2000] / dyn_force / pitot_airspeed");

}

void loop(){

  //Get time from start
  currentMillis = millis();
  
  // If 'y' is typed in the Serial monitor then we should start getting measurements...
  Serial.flush();
  while(command.measurements_input != 'y') {
    Serial.println("Start getting measurements?");
    while (Serial.available() == 0) {
      // Wait for user input
    }
    
    command.measurements_input = Serial.read();
    Serial.flush();
  }


  //send command to nrf
  radioSendCommands();

  //receive data from the receiver
  radio.startListening();
  recvData();

  // Sending to Serial Studio
  Serial.print("/*"); // Format for Serial Studio
  Serial.print(currentMillis);
  Serial.print(",");
  Serial.print(data.loop_freq);  //Frequency of Wind Tunnel Arduino Loop[Hz]
  Serial.print(",");
  Serial.print(command.motor_speed);
  Serial.print(",");
  Serial.print(data.dyn_force); // Reading minus tare and divided by calibration parameter
  Serial.print(",");
  Serial.print(data.pitot_airspeed);
  Serial.print(",");
  Serial.print(data.voltage);
  Serial.print(",");
  Serial.print(data.rpm);
  Serial.print("*/\n");


  //Close measurements?
  if(Serial.available()) { 
    command.close_input = Serial.read();

    //Initializing Close Procedure
    if (command.close_input == 'y'){

      radioSendCommands(20);
      // printCommand();

      command.measurements_input = 'n';
      Serial.println("Measurements Done");
      
      // myFile.close();
      // Serial.println("File: " + get_filename(file_offset) + " closed. Have fun :)");
      Serial.println("File Closed");


      //LOOP TO GET STUCK
      Serial.println("Begin another session?");
      Serial.flush();
      while(command.measurements_input != 'y') {
        while (Serial.available() == 0) {
          // Wait for user input
        }
        command.measurements_input = Serial.read();
        Serial.flush();
      }

      //Asking for motor speed input
      askForMotorSpeed();

      //when he said yes, we init the sd card again and start over with the measurements
      command.restart = 'y';
      command.close_input = 'n';
      
      radioSendCommands(10);

      command.restart = 'n';

      resetData();
      Serial.println("Starting again in 2 seconds...");
      delay(2000);
      Serial.println("Format: Time[ms] / MotorSpeed [1000 - 2000] / dyn_force / pitot_airspeed");

    }
  }


}

void readDateTimeFromSerial(){
  Serial.flush();
  while (Serial.available() == 0) {
    // Wait for user input
  }
  date_time = Serial.readString();
  Serial.flush();
  Serial.println("---------------------");
  Serial.println(date_time);
  Serial.println("---------------------");
  delay(1000);
}

void askForMotorSpeed(){
  int input = 0;
  while (input < 1000 || input > 2000){
    Serial.println("Input desired Motor Speed (eg. 1500): ");
    Serial.flush();
    while (Serial.available() == 0) {
      // Wait for user input
    }
    input = Serial.parseInt();
    Serial.flush();
    Serial.println("---------------------");
    Serial.print("Motor Speed: "); Serial.println(input);
    Serial.println("---------------------");
    delay(1000);
  }
  command.motor_speed = input;
}

void recvData(){
  if ( radio.available() ){
      radio.read(&data, sizeof(WindTunnelData));
  } 
}

//Function to send the radio commands 
//multiple times for redundancy if needed
void radioSendCommands(int times){
  radio.stopListening();
  for (int i = 0; i < times; i++){
    radio.write(&command, sizeof(Command));
    delay(10);
  }
}

void resetCommand() {
  command.init_sd = 'n'; // Should be 'y' in order to start the SD card initialization...
  command.close_input = 'n'; // If 'y' then we should close the measurements file...
  command.measurements_input = 'n'; // Should be 'y' in order to start measuring...
}

void resetData() {
  strcpy(data.message, "");
  data.dyn_force = 0;
  data.pitot_airspeed = 0;
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
