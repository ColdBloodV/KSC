// TSAT A Ground Better
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RF95_CS   5
#define RF95_INT  2
#define RF95_RST  4
#define BUTTON_PIN 25
#define SERIAL_BAUD 74880  

//Globals

//Receive and send 
RH_RF95 rf95(RF95_CS, RF95_INT);

int buttonValue = 0;
int lastButtonValue = 0;
int currentlySending = 0; 
int noreplycount = 1;

unsigned long lastPacketTime = 0; 

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
  pinMode(BUTTON_PIN, INPUT); 
  
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

  //only send one message at a time
  if(!currentlySending){
    buttonValue = digitalRead(BUTTON_PIN);
  
    // Button pressed (rising edge)
    if (buttonValue == HIGH && lastButtonValue == LOW) {
      currentlySending = 1; 

      //Wait to prevent packet corruption 
      unsigned long waitStart = millis();
      while(millis() - lastPacketTime < 200){ //200ms 
        delay(10);

        //flush any straggling packets
        if(rf95.available()){

          uint8_t flush[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t flen = sizeof(flush);
          rf95.recv(flush, &flen);
          lastPacketTime = millis(); 
          Serial.println("Killing before detach...");
        }
      }

      WaitACK();
      currentlySending = 0; 
      // Debounce
      delay(300);
      lastButtonValue = HIGH; //changed
    }

    lastButtonValue = buttonValue;
  
    //listen contniously for Altitude
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len)) {

        //Track when we last received a packet
        lastPacketTime = millis(); 

        if(len < RH_RF95_MAX_MESSAGE_LEN){

          buf[len] = '\0';
        }
        if(strncmp((char*)buf, "Alt:", 4) == 0){
          Serial.println((char*)buf);
          Serial.print("<<< RSSI: ");
          Serial.println(rf95.lastRssi());

        }
        else{
          Serial.println("ignored packet");
        }
       
      }
    }
  }
  delay(10); 
}

void WaitACK(){

  Serial.println("Sending 'Detach' message...");
  bool gotReply = false;

  flush();
  delay(1000);//let packet finish 
  flush();
  
  uint8_t detachMsg[] = "Detach";
  rf95.send(detachMsg, sizeof(detachMsg) - 1); 
  rf95.waitPacketSent();
  rf95.setModeRx();
  
  Serial.println("Message sent!");
  delay(500); //prev 500ms

  //Do not listen for Altitude while sending or receving from/to T-SAT
  flush();

  unsigned long startTime = millis();
  Serial.println("Waiting for reply...");
  //Wait for reply for 25 seconds (needs time to deploy servo)
  while (millis() - startTime < 25000) { 

    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len)) {

        if(len < RH_RF95_MAX_MESSAGE_LEN){

          buf[len] = '\0';
        }
        //Make sure its ACK
        if(strcmp((char*)buf, "Detached") == 0){

          Serial.print("<<< REPLY RECEIVED: ");
          Serial.println((char*)buf);
          Serial.print("<<< RSSI: ");
          Serial.println(rf95.lastRssi());

          gotReply = true;
          break; 
        }
        else{
          Serial.println("(ignored non-ACK packet)");
        }
   
      }
    }
    delay(10);
  }
    
  if (!gotReply) {
    Serial.printf("No reply received %d\n", noreplycount);
    noreplycount++;
    return; 
  }
  //Servo Cooldown
  Serial.println("Cooldown: 30 seconds before you can send again...");
  unsigned long cooldownStart = millis();
  while(millis() - cooldownStart < 30000){
    int remaining = 30 - (int)((millis() - cooldownStart) / 1000);
    Serial.printf("%d seconds\r\n", remaining);
    delay(1000);
  }
  Serial.println("\n>>> You can now send Detach <<<");
  flush(); 
}

//flush altitude packets to avoid corruption
void flush(){
    while(rf95.available()){
      uint8_t flush[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t flen = sizeof(flush);
      rf95.recv(flush, &flen);
      Serial.println("packet got flushed ;(");
  }
}

