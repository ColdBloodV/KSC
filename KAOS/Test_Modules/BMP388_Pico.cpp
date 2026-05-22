// BMP388 test for Raspberry Pi Pico
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5

Adafruit_BMP3XX bmp;

float groundPressure_hPa = 0.0;

float calculateAltitudeAboveBaseline(float pressurePa, float baselinePressureHpa) {
  float pressureHpa = pressurePa / 100.0;
  return 44330.0 * (1.0 - pow(pressureHpa / baselinePressureHpa, 0.1903));
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();

  Serial.println("Starting BMP388 test on Raspberry Pi Pico...");
  Serial.println("Using SDA=GP4, SCL=GP5");

  if (!bmp.begin_I2C(0x77, &Wire)) {
    Serial.println("BMP388 not found at 0x77, trying 0x76...");

    if (!bmp.begin_I2C(0x76, &Wire)) {
      Serial.println("ERROR: Could not find BMP388 sensor. Check wiring.");
      while (1) {
        delay(10);
      }
    }
  }

  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  for (int i = 0; i < 5; i++) {
    bmp.performReading();
    delay(100);
  }

  float pressureSum = 0.0;
  int validReadings = 0;

  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) {
      pressureSum += bmp.pressure;
      validReadings++;
    }
    delay(50);
  }

  if (validReadings == 0) {
    Serial.println("ERROR: Could not read baseline pressure.");
    while (1) {
      delay(10);
    }
  }

  groundPressure_hPa = (pressureSum / validReadings) / 100.0;

  Serial.print("Launch baseline pressure: ");
  Serial.print(groundPressure_hPa);
  Serial.println(" hPa");

  Serial.println("BMP388 ready.");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Read fail");
    delay(500);
    return;
  }

  float pressureHpa = bmp.pressure / 100.0;
  float altitudeAboveLaunch = calculateAltitudeAboveBaseline(
    bmp.pressure,
    groundPressure_hPa
  );

  Serial.print("Pressure: ");
  Serial.print(pressureHpa);
  Serial.print(" hPa | Temp: ");
  Serial.print(bmp.temperature);
  Serial.print(" C | Alt above launch: ");
  Serial.print(altitudeAboveLaunch);
  Serial.println(" m");

  delay(1000);
}
