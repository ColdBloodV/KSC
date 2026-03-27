// Ground T-SATA No Altitude 
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RF95_CS   5
#define RF95_INT  2
#define RF95_RST  4
#define BUTTON_PIN 25
#define SERIAL_BAUD 74880  // Make sure Serial Monitor matches

RH_RF95 rf95(RF95_CS, RF95_INT);


int buttonValue = 0;
int lastButtonValue = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  Serial.println("\n=== Initializing Ground Station ===");
  
  // Initialize SPI
  SPI.begin();
  
  // Reset radio
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, LOW);
  delay(10);
  digitalWrite(RF95_RST, HIGH);
  delay(10);
  
  // Button setup
  pinMode(BUTTON_PIN, INPUT);  // Adjust based on your wiring
  
  // Initialize radio
  if (!rf95.init()) {
    Serial.println("LoRa init FAILED!");
    while (1) delay(1000);
  }
  Serial.println("LoRa init SUCCESS");
  
  if (!rf95.setFrequency(915.0)) {
    Serial.println("Frequency set FAILED!");
    while (1);
  }
  Serial.println("Frequency: 915 MHz");
  
  rf95.setTxPower(13, false);
  Serial.println("TX Power: 13 dBm");
  
  Serial.println("\n=== Ground Station Ready ===");
  Serial.println("Press button to send 'Detach'\n");
}

void loop() {
  buttonValue = digitalRead(BUTTON_PIN);
  
  // Button pressed (rising edge)
  if (buttonValue == HIGH && lastButtonValue == LOW) {
    Serial.println("\n>>> BUTTON PRESSED <<<");
    Serial.println("Sending 'Detach' command...");
    
    uint8_t detachMsg[] = "Detach";
    rf95.send(detachMsg, sizeof(detachMsg) - 1);  // Don't send null terminator
    rf95.waitPacketSent();
    
    Serial.println("Message sent!");
    delay(500); //let tsat finish loop 
    Serial.println("Waiting for reply...");
    
    // WAIT LONGER FOR REPLY - satellite needs time to process
    unsigned long replyTimeout = millis() + 10000;  // Wait up to 5 seconds
    bool gotReply = false;
    
    while (millis() < replyTimeout && !gotReply) {
      if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        if (rf95.recv(buf, &len)) {
          buf[len] = '\0';
          Serial.print("<<< REPLY RECEIVED: ");
          Serial.println((char*)buf);
          Serial.print("<<< RSSI: ");
          Serial.println(rf95.lastRssi());
          gotReply = true;
        }
      }
      delay(10);  // Small delay to prevent tight loop
    }
    
    if (!gotReply) {
      Serial.println("!!! No reply received (timeout)");
    }
    
    // Debounce
    delay(300);
    lastButtonValue = HIGH; //changed
  }
  
  lastButtonValue = buttonValue;
  
  //listen contniously 
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';
      Serial.print("<<< Unexpected message: ");
      Serial.println((char*)buf);
      Serial.print("<<< RSSI: ");
      Serial.println(rf95.lastRssi());
    }
  }
  
  delay(10);  // Prevent tight loop
}
