// SCD-30 test for Raspberry Pi Pico
#include <Wire.h>
#include <Adafruit_SCD30.h>

#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5
#define SCD30_I2C_ADDRESS 0x61

Adafruit_SCD30 scd30;
bool scd30Found = false;

void scanI2C() {
  Serial.println("Scanning I2C bus...");

  int devicesFound = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found. Check power, ground, SDA, and SCL.");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(15000);

  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println();
  Serial.println("=== SCD-30 Pico sketch booted ===");
  Serial.println("Starting SCD-30 test on Raspberry Pi Pico...");
  Serial.println("Using SDA=GP4, SCL=GP5");
  scanI2C();

  if (!scd30.begin(SCD30_I2C_ADDRESS, &Wire)) {
    Serial.println("ERROR: Failed to find SCD-30 sensor. Check wiring.");
    Serial.println("Continuing anyway so Serial/LED diagnostics keep running.");
    scd30Found = false;
    return;
  }

  scd30Found = true;
  Serial.println("SCD-30 found.");
  Serial.print("Measurement interval: ");
  Serial.print(scd30.getMeasurementInterval());
  Serial.println(" seconds");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  static bool ledOn = false;
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

  Serial.println("loop alive");

  if (!scd30Found) {
    Serial.println("SCD-30 not initialized. Check scanner output and wiring.");
    delay(1000);
    return;
  }

  if (scd30.dataReady()) {
    if (!scd30.read()) {
      Serial.println("Error reading SCD-30 data");
      delay(500);
      return;
    }

    Serial.print("CO2: ");
    Serial.print(scd30.CO2, 3);
    Serial.print(" ppm | Temp: ");
    Serial.print(scd30.temperature);
    Serial.print(" C | Humidity: ");
    Serial.print(scd30.relative_humidity);
    Serial.println(" %");
  } else {
    Serial.println("Waiting for SCD-30 data...");
  }

  delay(1000);
}
