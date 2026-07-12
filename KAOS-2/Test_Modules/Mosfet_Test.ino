#define PICO_T  2

void setup() {
  Serial.begin(115200);
  pinMode(PICO_T, OUTPUT);

}

void loop() {
  digitalWrite(PICO_T, HIGH);
  delay(500);
  digitalWrite(PICO_T, LOW);
  delay(500);
}
