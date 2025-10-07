#include <SPI.h>
#include <RH_RF95.h>

//Air
 
// Singleton instance of the radio driver
RH_RF95 rf95(4,5);
 
// LED and timing variables
const int ledPin = LED_BUILTIN;  // Built-in LED pin 
unsigned long burnTime = 5000;   // Burn time 5 seconds
uint8_t reply[] = "Burning the wire...";  // msg back to ground

void setup() 
{
  Serial.begin(115200);

  Serial.println("Initializing");

  while (!Serial);  // Wait for the serial port to be available

  if (!rf95.init()) {
    Serial.println("RF95 module initialization failed!");
    while (1);  // Halt if radio initialization fails
  }

  rf95.setFrequency(915.0);  // Set frequency 
  
  pinMode(ledPin, OUTPUT);  // Set the LED pin as output
  digitalWrite(ledPin, LOW);  // LED is initially off

  Serial.println("T-Sat isready to receive messages.");
}

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Wait for a message from Ground Station
  if (rf95.waitAvailableTimeout(3000)) { 
    //Checks if message was received 
    if (rf95.recv(buf, &len)) 
    {
      // Received a message, print it out and trigger LED action
      Serial.print("Received message: ");
      Serial.println((char*)buf);  // Print the received message

      if (strcmp((char*)buf, "BurnWire") == 0) 
      {
        // Message is "BurnWire", trigger LED and send response

        // Turn on the LED
        digitalWrite(ledPin, HIGH);
        Serial.println("LED is ON (burning)");

        // Delay for burnTime duration
        delay(burnTime);

        // Turn off the LED
        digitalWrite(ledPin, LOW);
        Serial.println("LED is OFF (burn complete)");

        // Send response to Ground Station
        Serial.println("Sending reply: 'Burning the wire...'");
        rf95.send(reply, sizeof(reply));
        rf95.waitPacketSent();  // Wait for transmission to complete
      }
    } 
    else
    {
      Serial.println("Receive failed");
    }
  } 
  else 
  {
    Serial.println("No message received from Ground Station");
  }

  delay(1000);  //optinal delay before checking again 
}

