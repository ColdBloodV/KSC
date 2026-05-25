#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 rf95(5, 21);

// Button Pin and Variables
int ButtonValue = 0; 
int lastButtonValue = 0;
int Button = 3;          // Pin where the button is connected
int led = LED_BUILTIN;   // Built-in LED ESP32

// Message to be sent when the button is pressed
uint8_t burn[] = "BurnWire"; 

void setup() 
{
  pinMode(Button, INPUT);  // Set button pin as input
  pinMode(led, OUTPUT);    // Set LED pin as output
  
  Serial.begin(115200);
  while (!Serial) ;  // Wait for serial port to be available
  
  if (!rf95.init()) {
    Serial.println("init failed");
    while (1);  // if radio inti fails then it stops  
  }

  rf95.setFrequency(915.0);  // Set frequency 
  Serial.println("Ground Station Ready");
}

void loop()
{
  ButtonValue = digitalRead(Button);

  // Check if button is pressed
  if (ButtonValue == HIGH && lastButtonValue == LOW) {
    // Button was pressed, send "BurnWire" message
    Serial.println("Sending 'BurnWire' message...");
    rf95.send(burn, sizeof(burn));  // Send the message
    rf95.waitPacketSent();  // Wait for transmission to complete

    //Turn on LED to indicate its sending 
    digitalWrite(led, HIGH); 
    delay(5000);               // Keep LED on for 5 secs.
    digitalWrite(led, LOW);   

    // Print confirmation of message
    Serial.println("Message sent to T-Sat: 'BurnWire'");
  }

  // Store current state of button
  lastButtonValue = ButtonValue;

  //Wait for the reply 
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.waitAvailableTimeout(3000)) // Wait for a reply for 3 seconds
  {  
    if (rf95.recv(buf, &len)) {
      Serial.print("Received reply: ");
      Serial.println((char*)buf);
    } 
    else 
    {
      Serial.println("Receive failed"); // signal detected but fail to rec
    }
  } 
  else 
  {
    Serial.println("No reply received is rf95_server running?"); //no signal at all
  }
  delay(50);  // Add delay for button press
}
