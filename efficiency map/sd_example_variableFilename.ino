#include <SPI.h>
#include <SD.h>

File myFile;

#define SD_REOPEN_TIME 3000

const byte SD_chipSelect = 10;
int file_offset = 0;  // This is the file offset that will be appended to the filename.
unsigned long sd_last_opened = millis();
char filename[20];


bool sd_connected = false;





void setup(){

    Serial.begin(9600);

    sd_connected = SD.begin(SD_chipSelect);

    Serial.println(sd_connected?"Init Completed":"Init Failed");

    sdOpenFile(0);

}






void loop(){

    if (sd_connected == true){

        storeDataSD();  

        if (millis() - sd_last_opened > SD_REOPEN_TIME){
            sd_last_opened = millis();
            Serial.println("Reopening");
            closeSD();
            delay(10);
            sdOpenFile(1); //reopen
        }
    }

    delay(200);

}





// // This function returns the correct filename 
// //of the measurements file given a certain offset.
// String get_filename(int file_offset) {
//   return "drone" + String(file_offset) + ".csv";
// }
void get_filename(int file_offset, char* filename) {
  sprintf(filename, "drone%d.csv", file_offset);
}




/*
    If reopenLast = 1 -> try to open the last remembered filename
*/
void sdOpenFile(byte reopenLast){
    if (reopenLast == 0){
        // Find latest file offset for the filename  
        get_filename(file_offset, filename);
        while (SD.exists(filename)){
            file_offset += 1;
            get_filename(file_offset, filename);
        }
    }
    // try opening the file...
    get_filename(file_offset, filename);
    myFile = SD.open(filename, FILE_WRITE);

    if (myFile){
        Serial.print("Measurements will be saved in file: ");
        Serial.println(filename);
    }
    else {
        Serial.print("Something went wrong while trying to open the file ");
        Serial.println(filename);
    }

    if (myFile && reopenLast != 0)
        Serial.println("Succesfully Reopened!");
    if (!myFile && reopenLast != 0)
        Serial.println("Could not be Reopened!");

}

void storeDataSD(){
    // firt we check that the file is open and working
    if (myFile) {
        myFile.print(1);
        myFile.print(",");
        myFile.print(2);
        myFile.print("\n");
    } else {
        get_filename(file_offset, filename);
        Serial.print("There seems to be an error with the SD File ");
        Serial.println(filename);

    }
}

void closeSD(){
    myFile.close();
    get_filename(file_offset, filename);
    Serial.print("File: ");
    Serial.print(filename);
    Serial.println(" closed. Have fun :)");

}
