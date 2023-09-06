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
const byte PITOT_PIN = A2;
const byte motor_pin = A3;

const byte VOLTAGE_PIN = A0;

const byte RPM_PIN = 2;

const byte pitot_offset_times = 50; // how many times to test for offset at the beginning of the code
const byte pitot_readings_per_loop = 10;

#define SCALE_OFFSET (207762.2)
#define SCALE_SLOPE (192.0143)

#define RPM_MEASURE_TIME 20000
// --------------------

//Variables Needed
double pitot_offset = 0;  // offset if there is flow when starting pitot
double pitot_airspeed = 0;
double pitot_pressure = 0;
double pitot_adc_raw = 0;
double pitot_voltage = 0;
double air_density = 1;

int rpm_flag = 1; //flag used to detect only one measurement per speed change

int file_offset = 0;  // This is the file offset that will be appended to the filename.

double dyn_force = 0;

String comments = "";
unsigned long currentMillis = millis();
unsigned long int prev_time_interups;
unsigned long prev_time = 0;
float wt_loop_freq = 0;  //wind tunnel arduino loop freq
int motor_speed = 1000;

int rpm = 0;
volatile float objects = 0;

float adc_voltage = 0.0;
float battery_voltage = 0.0;
// Floats for resistor values in divider (in ohms)
float R1 = 27900.0;
float R2 = 7350.0; //corrected from 7500 using the voltmeter device
float ref_voltage = 4.9; //4.965
int adc_value = 0;


//Commands
byte serial_command = 'n';
byte command_measurements_input = 'n';

void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600); delay(1000);

  // Motor Configuration
  motor.attach(motor_pin);
  motor.writeMicroseconds(1000);
  delay(500);

  // RPM Config
  attachInterrupt(digitalPinToInterrupt(2), count_rpm, FALLING);
  delay(1000);
  pinMode(RPM_PIN, INPUT);

  // Getting time and date from user
  Serial.println("Enter any comments regarding the setup of the experiment: ");
  readCommentsTimeFromSerial();


  //Get air density from user
  readDensityFromSerial();
  
  
  // Scale Init
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale(192.0143);
  // scale.set_offset(207762.2);


  // PITOT BEGIN INITIALIZATION ----------------
  // We test for offset 10 times and we take the average offset
  for(int i=0; i < pitot_offset_times; ++i){
    pitot_offset += analogRead(PITOT_PIN) - (1023 / 2);
  }
  pitot_offset /= pitot_offset_times;


  // SD CARD INIT
  // sdCardInit();


  //Asking user for motor speed input
  askForMotorSpeed();
  
  //Printing Status
  Serial.println("Format: ");
  Serial.println("Time[ms] / Loop Freq [Hz] / MotorSpeed [1000 - 2000] / Dyn_force [g] / RPM / Pitot_adc / Pitot_voltage [V]/ Pitot_pressure [Pa] / Pitot_speed [m/s] / RPM_flag / Battery_voltage [V]");

}

void loop(){

	prev_time = millis();

	// If 'y' is typed in the Serial monitor then we should start getting measurements...
 	Serial.flush();
 	while(command_measurements_input != 'y') {
  	Serial.println("Start getting measurements?");
  	while (Serial.available() == 0) {
  	  // Wait for user input
  	}
		command_measurements_input = Serial.read();
		Serial.flush();

		delay(200);
	}


	//Start motor 
	motor.writeMicroseconds(motor_speed); //TODO: Might take a while for the motor to reach desired RPM


	//RPM Measurement
	if (rpm_flag == 1){
		startRPMMeasurements();
	}
	rpm_flag = 0;


	// Reading pitot
	double v_read = 0; 
	for (int i = 0; i < pitot_readings_per_loop; i++){
		v_read += analogRead(PITOT_PIN) - pitot_offset;
	}
	pitot_adc_raw = v_read/pitot_readings_per_loop;
	pitot_voltage = pitot_adc_raw / 1023 * 5;
	pitot_pressure = 1000 * (pitot_voltage - 2.5);  // [Pa]

	// if it reads above 512 we need to equate to a negative velocity
	if (v_read < 512)
	  pitot_airspeed = -sqrt(2 * abs(pitot_pressure) / air_density); //maybe we could simplify these operations
	else
	  pitot_airspeed = +sqrt(2 * abs(pitot_pressure) / air_density);


	// Reading Scale
	// data.dyn_force = scale.get_units(); //faster than 330 Hz
	if (scale.is_ready()){
		dyn_force = (scale.read() - SCALE_OFFSET)/(SCALE_SLOPE);
	}

	//Storing Freq
	wt_loop_freq = (float)(1000/(millis() - prev_time)); // Frequency [Hz]


	//READING BATTERY VOLTAGE
	adc_value = analogRead(VOLTAGE_PIN);  // Read the Analog Input
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;  // Determine voltage at ADC input
  battery_voltage = adc_voltage*(R1+R2)/R2;  // Calculate voltage at divider input
  
	// Sending to Serial Studio
	serialStudioPrint();

	// Store data in SD card
	// storeDataSD();


	//Close measurements?
	if(Serial.available()) { 
		serial_command = Serial.read();

		//Initializing Close Procedure
		if (serial_command == 'y'){
			serial_command = 'n';

			motor.writeMicroseconds(1000); // motor stop

			command_measurements_input = 'n';

			// command_measurements_input = 'n';
			Serial.println("Measurements Done");

			// myFile.close();
			// Serial.println("File: " + get_filename(file_offset) + " closed. Have fun :)");

			Serial.println("File Closed");


			//LOOP TO GET STUCK
			Serial.println("Begin another session?");
			Serial.flush();
			while(serial_command != 'y') {
				while (Serial.available() == 0) {
					// Wait for user input
				}
				serial_command = Serial.read();
				Serial.flush();

				//If 'r' is typed in the terminal before 'y' -> we want to measure rpm in the next run
				rpm_flag = (rpm_flag == 1)?1:((serial_command == 'r')?1:0);
			}
			serial_command = 'n';

			//Asking for motor speed input
			askForMotorSpeed();


			//Printing Status
		  Serial.println("Format: ");
		  Serial.println("Time[ms] / Loop Freq [Hz] / MotorSpeed [1000 - 2000] / Dyn_force [g] / RPM / Pitot_adc / Pitot_voltage [V]/ Pitot_pressure [Pa] / Pitot_speed [m/s] / RPM_flag / Battery_voltage [V]");

		}
	}


}


// This function returns the correct filename 
//of the measurements file given a certain offset.
String get_filename(int file_offset) {
  return "test" + String(file_offset) + ".csv";
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

void readCommentsTimeFromSerial(){
	Serial.flush();
	while (Serial.available() == 0) {
		// Wait for user input
	}
	comments = Serial.readString();
	Serial.flush();
	Serial.println("---------------------");
	Serial.println(comments);
	Serial.println("---------------------");
	delay(1000);
}

void readDensityFromSerial(){
	float input = 0;

	Serial.println("Choose AIR DENSITY for the experiment:");
	Serial.println("http://meteo.ntua.gr/");
	Serial.println("https://www.calctool.org/atmospheric-thermodynamics/air-density");
	Serial.flush();
	while (Serial.available() == 0) {
		// Wait for user input
	}
	input = Serial.parseFloat();
	input = (double)(input);
	Serial.flush();
	Serial.println("---------------------");
	Serial.print("Air Density: "); Serial.println(input, 6);
	Serial.println("---------------------");
	delay(1000);

	air_density = input;

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
	motor_speed = input;
}

void sdCardInit(){

	// SD CARD BEGIN INITIALIZATION ----------------
  Serial.println("Should I initialize the SD Card?");  // Once 'y' is typed in the Serial monitor the program should go on...
  
  Serial.flush();
  while (serial_command != 'y') {
    while (Serial.available() == 0) {
      // Wait for user input
    }
    
    serial_command = Serial.read();
    Serial.flush();
  }
  serial_command = 'n';
  delay(1000);

  Serial.println("Initializing SD card..."); //send message back too
  
  delay(200);

  if (!SD.begin()) {
    Serial.println("Initialization failed!"); //send message back too
    delay(1000);
    // return;
  }
  else {
  	Serial.println("Initialization done."); //send message back
  }
  delay(200);

  // sdOpenFile();
  // SD CARD END INITIALIZATION

}

void count_rpm() {
  ++objects;
}

void startRPMMeasurements(){
	unsigned long started_time = millis();
	while (millis() - started_time < RPM_MEASURE_TIME){
		delay(1500);
	  detachInterrupt(digitalPinToInterrupt(RPM_PIN));
	  rpm = objects/(millis() - prev_time_interups) * 60000;
		prev_time_interups = millis();
	  objects = 0;
	  attachInterrupt(digitalPinToInterrupt(RPM_PIN), count_rpm, FALLING);

	  serialStudioPrint();
	}
	delay(500);
}

void serialStudioPrint(){
	Serial.print("/*"); // Format for Serial Studio
	Serial.print(millis());
	Serial.print(",");
	Serial.print(wt_loop_freq);  //Frequency of Wind Tunnel Arduino Loop[Hz]
	Serial.print(",");
	Serial.print(motor_speed);
	Serial.print(",");
	Serial.print(dyn_force); // Reading minus tare and divided by calibration parameter
	Serial.print(",");
	Serial.print(rpm);
	Serial.print(",");
	Serial.print(pitot_adc_raw);
	Serial.print(",");
	Serial.print(pitot_voltage);
	Serial.print(",");
	Serial.print(pitot_pressure);
	Serial.print(",");
	Serial.print(pitot_airspeed);
	Serial.print(",");
	Serial.print(rpm_flag?1:0);
	Serial.print(",");
	Serial.print(battery_voltage);
	Serial.print("*/\n");
}

void storeDataSD(){
	// Store data in SD card
  // firt we check that the file is open and working

  if (myFile) {
    myFile.print("/*"); // Format for Serial Studio
		myFile.print(millis());
		myFile.print(",");
		myFile.print(wt_loop_freq);  //Frequency of Wind Tunnel Arduino Loop[Hz]
		myFile.print(",");
		myFile.print(motor_speed);
		myFile.print(",");
		myFile.print(dyn_force); // Reading minus tare and divided by calibration parameter
		myFile.print(",");
		myFile.print(rpm);
		myFile.print(",");
		myFile.print(pitot_adc_raw);
		myFile.print(",");
		myFile.print(pitot_voltage);
		myFile.print(",");
		myFile.print(pitot_pressure);
		myFile.print(",");
		myFile.print(pitot_airspeed);
		myFile.print(",");
		myFile.print(rpm_flag?1:0);
		myFile.print(",");
		myFile.print(battery_voltage);
		myFile.print("*/\n");
  } else {
    Serial.println("There seems to be an error with the SD File" + get_filename(file_offset));
  }

}