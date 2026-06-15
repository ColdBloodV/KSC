// =====================================================================
//  FLIGHT COMPUTER — Raspberry Pi Pico
//  Sensors  : BMP388 (pressure/temp/altitude)
//             SCD30  (CO2/temp/humidity)
//  Display  : SSD1306 OLED 128x64
//  Logging  : PCF8523 RTC  +  microSD (flight.csv)
//
//  I2C Bus  : GP4 (SDA)  GP5 (SCL)
//  SPI Bus  : GP18 (CLK) GP19 (MOSI) GP16 (MISO) GP17 (CS)
//
//  I2C Addresses:
//    SSD1306  0x3C
//    BMP388   0x77 (fallback 0x76)
//    SCD30    0x61
//    PCF8523  0x68  (handled internally by RTClib)
//
//  Libraries required (install via Arduino Library Manager):
//    Adafruit BMP3XX Library
//    Adafruit SCD30
//    Adafruit GFX Library
//    Adafruit SSD1306
//    Adafruit Unified Sensor
//    RTClib  (by Adafruit)
//    SD      (Arduino built-in)
// =====================================================================

// ===================== LIBRARIES =====================
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== I2C PINS =====================
#define PICO_SDA_PIN  4
#define PICO_SCL_PIN  5

// ===================== SPI / SD PINS =====================
#define SD_CS         17   // GP17
// GP18 = CLK, GP19 = MOSI, GP16 = MISO (hardware SPI defaults on Pico)

// ===================== OLED =====================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===================== SENSORS =====================
Adafruit_BMP3XX  bmp;
Adafruit_SCD30   scd30;

// ===================== RTC =====================
RTC_PCF8523 rtc;

// ===================== FLAGS =====================
bool scd30Found = false;
bool rtcFound   = false;
bool sdFound    = false;

// ===================== BASELINE =====================
float groundPressure_hPa = 0.0;

// =====================================================================
//  ALTITUDE CALCULATION
//  Uses barometric formula relative to ground-level baseline pressure.
//  Returns altitude in metres above launch point.
// =====================================================================
float calculateAltitudeAboveBaseline(float pressurePa, float baselinePressureHpa) {
  float pressureHpa = pressurePa / 100.0;
  return 44330.0 * (1.0 - pow(pressureHpa / baselinePressureHpa, 0.1903));
}

// =====================================================================
//  SD LOGGING
//  Appends one CSV row per call.  Silently skips if SD or RTC failed.
//  CSV columns: Date, Time, Pressure_hPa, Temperature_C, Altitude_m, CO2_ppm
// =====================================================================
void logToSD(float pressureHpa, float tempC, float altitude, float co2) {

  if (!sdFound || !rtcFound) return;

  DateTime now = rtc.now();

  File file = SD.open("flight.csv", FILE_WRITE);
  if (!file) {
    Serial.println("[SD] Could not open flight.csv for write");
    return;
  }

  // Date  YYYY-M-D
  file.print(now.year());  file.print("-");
  file.print(now.month()); file.print("-");
  file.print(now.day());   file.print(",");

  // Time  H:MM:SS  (pad minutes and seconds to two digits)
  file.print(now.hour());  file.print(":");
  if (now.minute() < 10) file.print("0");
  file.print(now.minute()); file.print(":");
  if (now.second() < 10) file.print("0");
  file.print(now.second()); file.print(",");

  // Sensor data
  file.print(pressureHpa); file.print(",");
  file.print(tempC);        file.print(",");
  file.print(altitude);     file.print(",");

  if (co2 < 0) {
    file.println("N/A");
  } else {
    file.println(co2);
  }

  file.close();
}

// =====================================================================
//  OLED UPDATE
//  Displays all four readings.  CO2 shows "Wait..." until SCD30 ready.
// =====================================================================
void updateOLED(float pressureHpa, float tempC, float altitude, float co2) {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("P:   ");
  display.print(pressureHpa, 1);
  display.println(" hPa");

  display.print("T:   ");
  display.print(tempC, 1);
  display.println(" C");

  display.print("Alt: ");
  display.print(altitude+160, 1);
  display.println(" m");

  display.print("CO2: ");
  if (co2 < 0) {
    display.println("Wait...");
  } else {
    display.print(co2, 0);
    display.println(" ppm");
  }

  // If RTC found, show timestamp on bottom line
  if (rtcFound) {
    DateTime now = rtc.now();
    display.setCursor(0, 56);
    display.print(now.hour());   display.print(":");
    if (now.minute() < 10) display.print("0");
    display.print(now.minute()); display.print(":");
    if (now.second() < 10) display.print("0");
    display.print(now.second());
  }

  display.display();
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // --- I2C ---
  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("=== FLIGHT COMPUTER BOOT ===");

  // ── OLED ─────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] FAIL — halting");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
  Serial.println("[OLED] OK");

  // ── BMP388 ───────────────────────────────────────
  if (!bmp.begin_I2C(0x77, &Wire)) {
    if (!bmp.begin_I2C(0x76, &Wire)) {
      Serial.println("[BMP388] FAIL — halting");
      while (1);
    }
  }
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("[BMP388] OK");

  // Baseline pressure — average 10 readings
  float sum  = 0;
  int   count = 0;
  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) {
      sum += bmp.pressure;
      count++;
    }
    delay(50);
  }
  groundPressure_hPa = (count > 0) ? (sum / count) / 100.0 : 1013.25;
  Serial.print("[BMP388] Baseline pressure: ");
  Serial.print(groundPressure_hPa);
  Serial.println(" hPa");

  // ── SCD30 ────────────────────────────────────────
  if (!scd30.begin(0x61, &Wire)) {
    Serial.println("[SCD30] FAIL — continuing without CO2");
    scd30Found = false;
  } else {
    scd30Found = true;
    Serial.println("[SCD30] OK");
  }

  // ── PCF8523 RTC ──────────────────────────────────
  if (!rtc.begin()) {
    Serial.println("[RTC] FAIL — timestamps unavailable");
    rtcFound = false;
  } else {
    rtcFound = true;
    if (!rtc.initialized() || rtc.lostPower()) {
      Serial.println("[RTC] Power lost — setting from compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("[RTC] OK");
  }

  // ── microSD ──────────────────────────────────────
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] FAIL — logging disabled");
    sdFound = false;
  } else {
    sdFound = true;
    Serial.println("[SD] OK");

    // Write CSV header only if file is empty / new
    File file = SD.open("flight.csv", FILE_WRITE);
    if (file) {
      if (file.size() == 0) {
        file.println("Date,Time,Pressure_hPa,Temperature_C,Altitude_m,CO2_ppm");
        Serial.println("[SD] Header written to flight.csv");
      }
      file.close();
    } else {
      Serial.println("[SD] Could not open flight.csv on boot");
    }
  }

  // ── Boot complete ─────────────────────────────────
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Boot complete");
  display.print("BMP388: "); display.println("OK");
  display.print("SCD30:  "); display.println(scd30Found ? "OK" : "FAIL");
  display.print("RTC:    "); display.println(rtcFound   ? "OK" : "FAIL");
  display.print("SD:     "); display.println(sdFound    ? "OK" : "FAIL");
  display.display();
  delay(2000);

  Serial.println("=== LOOP START ===");
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {

  // ── BMP388 read ──────────────────────────────────
  if (!bmp.performReading()) {
    Serial.println("[BMP388] Read failed — skipping cycle");
    delay(500);
    return;
  }

  float pressureHpa = bmp.pressure / 100.0;
  float tempC       = bmp.temperature;
  float altitude    = calculateAltitudeAboveBaseline(bmp.pressure, groundPressure_hPa);

  // ── SCD30 read ───────────────────────────────────
  float co2 = -1.0;

  if (scd30Found && scd30.dataReady()) {
    if (scd30.read()) {
      co2 = scd30.CO2;
      Serial.print("[SCD30] CO2: "); Serial.print(co2);
      Serial.print(" ppm | Temp: "); Serial.print(scd30.temperature);
      Serial.print(" C | RH: ");    Serial.print(scd30.relative_humidity);
      Serial.println(" %");
    }
  }

  // ── Serial output ────────────────────────────────
  Serial.print("[BMP388] P: "); Serial.print(pressureHpa);
  Serial.print(" hPa | T: ");  Serial.print(tempC);
  Serial.print(" C | Alt: ");  Serial.print(altitude);
  Serial.println(" m");
                                                                                                                                
  // ── OLED update ──────────────────────────────────
  updateOLED(pressureHpa, tempC, altitude, co2);

  // ── SD log ───────────────────────────────────────
  logToSD(pressureHpa, tempC, altitude, co2);

  delay(1000);
}
