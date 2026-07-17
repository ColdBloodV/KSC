// =====================================================================
//  FLIGHT COMPUTER — Raspberry Pi Pico
//  Sensors  : BMP388 (pressure/temp/altitude)
//             SCD30  (CO2/temp/humidity)
//             ICM-20948 (accel/gyro)
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
//    ICM20948 0x69 preferred, 0x68 fallback if no RTC is present
//    PCF8523  0x68  (handled internally by RTClib)
//
//  Libraries required (install via Arduino Library Manager):
//    Adafruit BMP3XX Library
//    Adafruit SCD30
//    Adafruit ICM20X
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
#include <Adafruit_ICM20948.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== I2C PINS =====================
#define PICO_SDA_PIN  4
#define PICO_SCL_PIN  5

// ===================== SPI / SD PINS =====================
#define SD_CS         17   // GP17
// GP18 = CLK, GP19 = MOSI, GP16 = MISO (hardware SPI defaults on Pico)

// ===================== APRS LINK / PAYLOAD SHUTDOWN =====================
// APRS sends compact scaled-integer I2C packets such as:
// G,2860159,-8119910,596,10,1
#define APRS_I2C_SDA_PIN         2   // Pico GP2 / I2C1 SDA
#define APRS_I2C_SCL_PIN         3   // Pico GP3 / I2C1 SCL
#define APRS_I2C_ADDRESS         0x42
#define PAYLOAD_SHUTDOWN_PIN     14  // Pico GP14 -> Pi physical pin 38 / GPIO20
#define PAYLOAD_SHUTDOWN_ACTIVE  LOW

#define ICM_I2C_ADDRESS 0x69  // Keep high so RTC can use 0x68.

// ===================== OLED =====================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===================== SENSORS =====================
Adafruit_BMP3XX  bmp;
Adafruit_SCD30   scd30;
Adafruit_ICM20948 icm;

// ===================== RTC =====================
RTC_PCF8523 rtc;

// ===================== FLAGS =====================
bool scd30Found = false;
bool imuFound   = false;
bool rtcFound   = false;
bool sdFound    = false;
bool shutdownRequested = false;

// ===================== BASELINE =====================
float groundPressure_hPa = 0.0;
const unsigned long BASELINE_SAMPLE_MS = 30000;
const unsigned long BASELINE_SAMPLE_INTERVAL_MS = 100;

// ===================== LOGGING / SAMPLING =====================
const unsigned long IMU_LOG_INTERVAL_MS = 10;      // 100 Hz target
const unsigned long ENV_SAMPLE_INTERVAL_MS = 100;  // 10 BMP samples/sec
const unsigned long ENV_UPDATE_INTERVAL_MS = 1000; // 1 Hz BMP/SCD/OLED update
const unsigned long SD_FLUSH_INTERVAL_MS = 1000;

// ===================== LATEST SENSOR STATE =====================
File logFile;
float latestPressureHpa = NAN;
float latestTempC = NAN;
float latestAltitudeM = NAN;
float latestCo2 = -1.0;
float latestScd30TempC = NAN;
float latestScd30Rh = NAN;
float latestGpsLatitude = NAN;
float latestGpsLongitude = NAN;
float latestGpsAltitudeM = NAN;
int latestGpsSatellites = 0;
bool latestGpsValid = false;
unsigned long lastGpsUpdateMs = 0;
float bmpPressureSumPa = 0.0;
float bmpTempSumC = 0.0;
int bmpSampleCount = 0;
float latestAccXmg = NAN;
float latestAccYmg = NAN;
float latestAccZmg = NAN;
float latestGyroXdps = NAN;
float latestGyroYdps = NAN;
float latestGyroZdps = NAN;
unsigned long lastImuLogMs = 0;
unsigned long lastEnvSampleMs = 0;
unsigned long lastEnvUpdateMs = 0;
unsigned long lastSdFlushMs = 0;
volatile char aprsLine[96];
volatile uint8_t aprsLineLength = 0;
volatile bool aprsLineReady = false;

// =====================================================================
//  ALTITUDE CALCULATION
//  Uses barometric formula relative to ground-level baseline pressure.
//  Returns altitude in metres above launch point.
// =====================================================================
float calculateAltitudeAboveBaseline(float pressurePa, float baselinePressureHpa) {
  float pressureHpa = pressurePa / 100.0;
  return 44330.0 * (1.0 - pow(pressureHpa / baselinePressureHpa, 0.1903));
}

void sampleBMPForEnvironment() {
  if (bmp.performReading()) {
    bmpPressureSumPa += bmp.pressure;
    bmpTempSumC += bmp.temperature;
    bmpSampleCount++;
  }
}

bool readIMU() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  if (!icm.getEvent(&accel, &gyro, &temp, &mag)) return false;

  latestAccXmg = accel.acceleration.x * (1000.0 / 9.80665);
  latestAccYmg = accel.acceleration.y * (1000.0 / 9.80665);
  latestAccZmg = accel.acceleration.z * (1000.0 / 9.80665);
  latestGyroXdps = gyro.gyro.x * (180.0 / PI);
  latestGyroYdps = gyro.gyro.y * (180.0 / PI);
  latestGyroZdps = gyro.gyro.z * (180.0 / PI);

  return true;
}

// =====================================================================
//  APRS GPS LINK / SHUTDOWN INPUT
// =====================================================================
void receiveAprsI2c(int byteCount) {
  uint8_t length = 0;

  while (Wire1.available() && length < sizeof(aprsLine) - 1) {
    aprsLine[length++] = static_cast<char>(Wire1.read());
  }

  while (Wire1.available()) {
    Wire1.read();
  }

  aprsLine[length] = '\0';
  aprsLineLength = length;
  aprsLineReady = (length > 0);
}

void handleAprsLine(const char *line) {
  if (strcmp(line, "SHUTDOWN") == 0) {
    shutdownRequested = true;
    return;
  }

  float latitude = NAN;
  float longitude = NAN;
  float altitudeM = NAN;
  int satellites = 0;
  int valid = 0;

  long latitudeE5 = 0;
  long longitudeE5 = 0;
  long altitudeMInt = 0;
  int parsed = sscanf(line, "G,%ld,%ld,%ld,%d,%d", &latitudeE5,
                      &longitudeE5, &altitudeMInt, &satellites, &valid);
  if (parsed == 5) {
    latitude = latitudeE5 / 100000.0;
    longitude = longitudeE5 / 100000.0;
    altitudeM = altitudeMInt;
  }
  if (parsed != 5) {
    // Accept the older UART test format too.
    parsed = sscanf(line, "GPS,%f,%f,%f,%d,%d", &latitude, &longitude,
                    &altitudeM, &satellites, &valid);
  }

  if (parsed == 5) {
    latestGpsLatitude = latitude;
    latestGpsLongitude = longitude;
    latestGpsAltitudeM = altitudeM;
    latestGpsSatellites = satellites;
    latestGpsValid = (valid != 0);
    lastGpsUpdateMs = millis();

    Serial.print("[APRS] GPS received: ");
    Serial.print(latestGpsLatitude, 6);
    Serial.print(",");
    Serial.print(latestGpsLongitude, 6);
    Serial.print(" alt=");
    Serial.print(latestGpsAltitudeM, 1);
    Serial.print(" sats=");
    Serial.println(latestGpsSatellites);
  }
}

void pollAprsLink() {
  char line[sizeof(aprsLine)];
  uint8_t length = 0;

  noInterrupts();
  if (aprsLineReady) {
    length = aprsLineLength;
    memcpy(line, (const void *)aprsLine, length + 1);
    aprsLineReady = false;
  }
  interrupts();

  if (length > 0) {
    handleAprsLine(line);
  }
}

void completePayloadShutdown() {
  if (!shutdownRequested) return;

  Serial.println("[SHUTDOWN] APRS shutdown request received");

  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  // Tell the Pi to stop recording and shut down cleanly.
  digitalWrite(PAYLOAD_SHUTDOWN_PIN, PAYLOAD_SHUTDOWN_ACTIVE);

  // Keep this controller alive while APRS/speaker power remains on.
  while (true) {
    delay(1000);
  }
}

// =====================================================================
//  SD LOGGING
//  Keeps the file open during flight so 100 Hz IMU rows do not spend most
//  of their time opening and closing the filesystem.
// =====================================================================
void openLogFile() {
  if (!sdFound) return;

  logFile = SD.open("flight.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("[SD] Could not open flight.csv for write");
    sdFound = false;
    return;
  }

  if (logFile.size() == 0) {
    logFile.println("Time_ms,Pressure_hPa,BMP_Temp_C,Altitude_m,CO2_ppm,SCD30_Temp_C,SCD30_RH,AccX_mg,AccY_mg,AccZ_mg,GyroX_dps,GyroY_dps,GyroZ_dps,GPS_Lat,GPS_Lon,GPS_Alt_m,GPS_Sats,GPS_Valid,GPS_Age_ms");
    logFile.flush();
    Serial.println("[SD] Header written to flight.csv");
  }
}

void logFastRow() {
  if (!sdFound || !logFile) return;

  float accX = NAN, accY = NAN, accZ = NAN;
  float gyroX = NAN, gyroY = NAN, gyroZ = NAN;

  if (imuFound && readIMU()) {
    accX = latestAccXmg;
    accY = latestAccYmg;
    accZ = latestAccZmg;
    gyroX = latestGyroXdps;
    gyroY = latestGyroYdps;
    gyroZ = latestGyroZdps;
  }

  logFile.print(millis()); logFile.print(",");
  logFile.print(latestPressureHpa); logFile.print(",");
  logFile.print(latestTempC); logFile.print(",");
  logFile.print(latestAltitudeM); logFile.print(",");
  if (latestCo2 < 0) logFile.print("N/A"); else logFile.print(latestCo2);
  logFile.print(",");
  logFile.print(latestScd30TempC); logFile.print(",");
  logFile.print(latestScd30Rh); logFile.print(",");
  logFile.print(accX); logFile.print(",");
  logFile.print(accY); logFile.print(",");
  logFile.print(accZ); logFile.print(",");
  logFile.print(gyroX); logFile.print(",");
  logFile.print(gyroY); logFile.print(",");
  logFile.print(gyroZ); logFile.print(",");
  logFile.print(latestGpsLatitude); logFile.print(",");
  logFile.print(latestGpsLongitude); logFile.print(",");
  logFile.print(latestGpsAltitudeM); logFile.print(",");
  logFile.print(latestGpsSatellites); logFile.print(",");
  logFile.print(latestGpsValid ? 1 : 0); logFile.print(",");
  if (lastGpsUpdateMs == 0) logFile.println("N/A");
  else logFile.println(millis() - lastGpsUpdateMs);
}

void flushLogIfNeeded(unsigned long nowMs) {
  if (!sdFound || !logFile) return;
  if (nowMs - lastSdFlushMs >= SD_FLUSH_INTERVAL_MS) {
    logFile.flush();
    lastSdFlushMs = nowMs;
  }
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

  // APRS I2C slave bus and Pi shutdown signal.
  Wire1.setSDA(APRS_I2C_SDA_PIN);
  Wire1.setSCL(APRS_I2C_SCL_PIN);
  Wire1.begin(APRS_I2C_ADDRESS);
  Wire1.onReceive(receiveAprsI2c);
  pinMode(PAYLOAD_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(PAYLOAD_SHUTDOWN_PIN, HIGH);

  // --- I2C ---
  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

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

  // Baseline pressure: average readings over a longer ground-still window.
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BMP baseline...");
  display.println("Keep payload still");
  display.display();

  float sum = 0;
  int count = 0;
  unsigned long baselineStart = millis();
  unsigned long lastProgress = 0;

  while (millis() - baselineStart < BASELINE_SAMPLE_MS) {
    if (bmp.performReading()) {
      sum += bmp.pressure;
      count++;
    }

    unsigned long elapsed = millis() - baselineStart;
    if (elapsed - lastProgress >= 1000) {
      lastProgress = elapsed;
      Serial.print("[BMP388] Baseline sampling ");
      Serial.print(elapsed / 1000);
      Serial.print("/");
      Serial.print(BASELINE_SAMPLE_MS / 1000);
      Serial.print(" s, samples=");
      Serial.println(count);
    }

    delay(BASELINE_SAMPLE_INTERVAL_MS);
  }
  groundPressure_hPa = (count > 0) ? (sum / count) / 100.0 : 1013.25;
  Serial.print("[BMP388] Baseline pressure: ");
  Serial.print(groundPressure_hPa);
  Serial.println(" hPa");

  // ── ICM-20948 IMU ───────────────────────────────
  if (icm.begin_I2C(ICM_I2C_ADDRESS, &Wire)) {
    imuFound = true;
    icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setMagDataRate(AK09916_MAG_DATARATE_SHUTDOWN);
    Serial.println("[ICM20948] OK at 0x69 via Adafruit library");
  } else {
    imuFound = false;
    Serial.println("[ICM20948] FAIL at 0x69");
  }

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

    openLogFile();
  }

  // ── Boot complete ─────────────────────────────────
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Boot complete");
  display.print("BMP388: "); display.println("OK");
  display.print("IMU:    "); display.println(imuFound ? "OK" : "FAIL");
  display.print("SCD30:  "); display.println(scd30Found ? "OK" : "FAIL");
  display.print("RTC:    "); display.println(rtcFound   ? "OK" : "FAIL");
  display.print("SD:     "); display.println(sdFound    ? "OK" : "FAIL");
  display.display();
  delay(2000);

  Serial.println("=== LOOP START ===");
  lastImuLogMs = millis();
  lastEnvSampleMs = millis();
  lastEnvUpdateMs = millis();
  lastSdFlushMs = millis();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  unsigned long nowMs = millis();

  pollAprsLink();
  completePayloadShutdown();

  // ── Fast 100 Hz IMU / SD row ─────────────────────
  if (nowMs - lastImuLogMs >= IMU_LOG_INTERVAL_MS) {
    lastImuLogMs += IMU_LOG_INTERVAL_MS;
    logFastRow();
  }

  // ── Background BMP accumulation ──────────────────
  if (nowMs - lastEnvSampleMs >= ENV_SAMPLE_INTERVAL_MS) {
    lastEnvSampleMs += ENV_SAMPLE_INTERVAL_MS;
    sampleBMPForEnvironment();
  }

  // ── Slow 1 Hz environment / display lane ─────────
  if (nowMs - lastEnvUpdateMs >= ENV_UPDATE_INTERVAL_MS) {
    lastEnvUpdateMs += ENV_UPDATE_INTERVAL_MS;

    if (bmpSampleCount > 0) {
      float pressurePa = bmpPressureSumPa / bmpSampleCount;
      latestPressureHpa = pressurePa / 100.0;
      latestTempC = bmpTempSumC / bmpSampleCount;
      latestAltitudeM = calculateAltitudeAboveBaseline(pressurePa, groundPressure_hPa);

      bmpPressureSumPa = 0.0;
      bmpTempSumC = 0.0;
      bmpSampleCount = 0;
    } else {
      Serial.println("[BMP388] No samples in last environment window");
    }

    if (scd30Found && scd30.dataReady()) {
      if (scd30.read()) {
        latestCo2 = scd30.CO2;
        latestScd30TempC = scd30.temperature;
        latestScd30Rh = scd30.relative_humidity;
      }
    }

    Serial.print("[BMP388] P: "); Serial.print(latestPressureHpa);
    Serial.print(" hPa | T: ");  Serial.print(latestTempC);
    Serial.print(" C | Alt: ");  Serial.print(latestAltitudeM);
    Serial.print(" m | IMU: ");  Serial.println(imuFound ? "OK" : "FAIL");

    updateOLED(latestPressureHpa, latestTempC, latestAltitudeM, latestCo2);
  }

  flushLogIfNeeded(nowMs);
}
