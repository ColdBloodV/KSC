#define MOSFET_PIN 2 //PIN 4


void setup() {
    Serial1.begin(115200);

    pinMode(MOSFET_PIN, OUTPUT);

    delay(1000);

    digitalWrite(MOSFET_PIN, HIGH); //Give power to the 2W 

    Serial.println("PICO READY");

}

void loop(){

    Shutoff();
}


void Shutoff(void) {

    // Pico initiates shutdown
    Serial1.println("SHUTOFF");

    // Wait for Pi to confirm it is done
    unsigned long startTime = millis();

    while (millis() - startTime < 30000) {

        if (Serial1.available()) {

            String response = Serial1.readStringUntil('\n');
            response.trim();

            if (response == "DONE") {

                // Give the Pi time to shut down
                delay(10000);

                // Cut power to Pi
                digitalWrite(MOSFET_PIN, LOW);

                return;
            }
        }
    }
}
