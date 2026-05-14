#include <SPI.h>
#include <SD.h>

#define SD_CS 22

#define SCK 18
#define MISO 19
#define MOSI 23
#define SD_DET 21

void setup() {
  Serial.begin(74880);
  delay(1000);

  Serial.println("\n=== SD CARD TEST ONLY ===");
  
   //Check if SD Card is in the module. 
  int val = digitalRead(SD_DET);
  if(val){
		 Serial.printf("SD CARD is in the hole %d\n", val);
	}
	else{
		Serial.printf("SD CARD is NOT in the hole %d\n", val);
	}
  delay(500);

  // Required for stable SPI
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(300);

  SPI.begin(SCK, MISO, MOSI);

  Serial.print("SD.begin... ");
  if (!SD.begin(SD_CS)) {
    Serial.println("FAIL");
    return;
  }
  Serial.println("OK");

  // Create test file
  Serial.println("Writing test.txt...");
  File f = SD.open("/test.txt", FILE_WRITE);
  if (!f) {
    Serial.println("FILE OPEN FAIL");
    return;
  }
  f.println("Hello from SD test!");
  f.close();

  // Read test file
  Serial.println("Reading test.txt...");
  f = SD.open("/test.txt");
  if (!f) {
    Serial.println("READ FAIL");
    return;
  }

  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();

  Serial.println("\nSD TEST COMPLETE");
}

void loop() {}
