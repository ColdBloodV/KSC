// KAOS-2 

//Libraries
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
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>
#include <hardware/sync.h>

//I2S (PCM5122)
#define I2S_BCLK  26
#define I2S_LRCLK 27
#define I2S_DOUT  28

// I2C 
#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5

// SPI / SD 
#define SD_CS 17

// MOSFET
#define MOSFET_PIN 2 //PIN 4

// 2W shutdown signal
#define PI_ZERO_SHUTDOWN_PIN 14
#define PI_ZERO_SHUTDOWN_ACTIVE LOW

// Sensors 
Adafruit_BMP3XX bmp;
Adafruit_SCD30  scd30;
RTC_PCF8523     rtc;
Adafruit_ICM20948 icm;

bool scd30Found = false;
bool rtcFound   = false;
bool sdFound    = false;
bool imuFound   = false;
File logFile;

float groundPressure_hPa = 0.0;

enum FlightPhase : uint8_t {
  BASELINING,
  WAITING_FOR_ASCENT,
  ASCENT_CONFIRMED,
  DESCENT_CONFIRMED,
  GROUND_CANDIDATE,
  LANDED
};

FlightPhase flightPhase = BASELINING;
float baselinePressureSum = 0.0f;
uint16_t baselinePressureCount = 0;
float minimumFlightPressureHpa = NAN;
float landingPressureMinHpa = NAN;
float landingPressureMaxHpa = NAN;
unsigned long baselineStartedMs = 0;
unsigned long ascentDropStartedMs = 0;
unsigned long descentStartedMs = 0;
unsigned long groundCandidateStartedMs = 0;

bool payloadShutdownStarted = false;
bool piShutdownSignaled = false;
volatile bool loggerShutdownRequested = false;
volatile bool loggerShutdownComplete = false;

#define SERIAL_COMMAND_SIZE 32
char serialCommand[SERIAL_COMMAND_SIZE];
uint8_t serialCommandLength = 0;

// Audio
#define MAX_SONGS 20
char songList[MAX_SONGS][64];
int  songCount   = 0;
int  currentSong = 0;

AudioFileSourceSD *source = nullptr;
AudioGeneratorMP3 *mp3    = nullptr;
AudioOutputI2S    *output = nullptr;
bool audioReady = false;

// State 
enum State { FLIGHT, GROUND };
State currentState = FLIGHT;
bool groundModeTriggered = false;

//Timers
unsigned long lastSensorRead = 0;
unsigned long lastLogTime    = 0;
#define SENSOR_INTERVAL 1000
#define IMU_LOG_INTERVAL 10
#define SD_FLUSH_INTERVAL 10000
#define LOG_SEGMENT_INTERVAL (15UL * 60UL * 1000UL)

//Currently set for (3hrs and 30 mins)
const unsigned long TIMER_DURATION = 3UL * 60UL * 60UL * 1000UL
                                   + 30UL * 60UL * 1000UL;
unsigned long startTime;

// Cached sensor values 
float lastPressureHpa = 0;
float lastTempC       = 0;
float lastAltitude    = 0;
float lastCO2         = -1;
uint32_t lastRtcDate  = 0;
uint32_t lastRtcTime  = 0;

float lastAccelX      = NAN;
float lastAccelY      = NAN;
float lastAccelZ      = NAN;
float lastGyroX       = NAN;
float lastGyroY       = NAN;
float lastGyroZ       = NAN;
float lastMagX        = NAN;
float lastMagY        = NAN;
float lastMagZ        = NAN;
float lastImuTempC    = NAN;

struct ImuLogSample {
  uint32_t elapsedMs;
  uint32_t rtcDate;
  uint32_t rtcTime;
  uint8_t flightPhase;
  float pressureHpa;
  float temperatureC;
  float altitudeM;
  float co2Ppm;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float magX;
  float magY;
  float magZ;
  float imuTempC;
};

// Sensor helpers
float calculateAltitudeAboveBaseline(float pressurePa, float baselinePressureHpa) {
  float pressureHpa = pressurePa / 100.0;
  return 44330.0 * (1.0 - pow(pressureHpa / baselinePressureHpa, 0.1903));
}

void logToSD(float pressureHpa, float tempC, float altitude, float co2) {
  if (!sdFound || !rtcFound) return;
  DateTime now = rtc.now();
  File file = SD.open("flight.csv", FILE_WRITE);
  if (!file) return;
  file.print(now.year());   file.print("-");
  file.print(now.month());  file.print("-");
  file.print(now.day());    file.print(",");
  file.print(now.hour());   file.print(":");
  if (now.minute() < 10) file.print("0");
  file.print(now.minute()); file.print(":");
  if (now.second() < 10) file.print("0");
  file.print(now.second()); file.print(",");
  file.print(pressureHpa);  file.print(",");
  file.print(tempC);        file.print(",");
  file.print(altitude);     file.print(",");
  if (co2 < 0) file.println("N/A"); else file.println(co2);
  file.close();
}

// Audio helpers
void scanForMP3s() {
  songCount = 0;
  File root = SD.open("/");
  if (!root || !root.isDirectory()) return;
  while (songCount < MAX_SONGS) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      String name = String(f.name());
      name.toLowerCase();
      if (name.endsWith(".mp3") && !String(f.name()).startsWith("._")) {
        String fullPath = "/" + String(f.name());
        fullPath.toCharArray(songList[songCount], 64);
        Serial.printf("  [%d] %s\n", songCount, songList[songCount]);
        songCount++;
      }
    }
    f.close();
  }
  root.close();
  Serial.printf("Found %d MP3 file(s)\n", songCount);
}

void stopCurrent() {
  if (mp3) {
    if (mp3->isRunning()) mp3->stop();
    delete mp3; mp3 = nullptr;
  }
  if (source) { delete source; source = nullptr; }
}

void startRecoveryAudio(int index) {
  stopCurrent();
  Serial.printf("Playing [%d/%d]: %s\n", index + 1, songCount, songList[index]);
  source = new AudioFileSourceSD(songList[index]);
  mp3    = new AudioGeneratorMP3();
  if (!mp3->begin(source, output)) {
    Serial.println("ERROR: MP3 failed to start, skipping...");
    currentSong = (currentSong + 1) % songCount;
    startRecoveryAudio(currentSong);
  }
}

//Shutoff Cams
void Shutoff_Cameras(void) {

    // Pico initiates shutdown
    Serial1.println("Shutting off Cameras");

    // Wait for Pi to confirm it is done
    unsigned long startTime = millis();

    while (millis() - startTime < 30000) {

        if (Serial1.available()) {

            String response = Serial1.readStringUntil('\n');
            response.trim();

            if (response == "DONE") {

                // Give the Pi time to shut down
                delay(20000);

                // Cut power to Pi
                digitalWrite(MOSFET_PIN, LOW);

                return;
            }
        }
    }
}

// Enter Gound Mode
void enterGroundMode() {
  Serial.println("Entering Ground mode...");

  // Flush any buffered IMU log data before we leave flight mode
  flushLogBuffer();
  if (logFile) logFile.flush();

  // Turn off Cameras
  Shutoff_Cameras();

  // Init I2S once
  if (!audioReady) {
    output = new AudioOutputI2S();
    output->SetPinout(I2S_BCLK, I2S_LRCLK, I2S_DOUT);
    output->SetChannels(2);
    output->SetGain(0.05);
    audioReady = true;
  }

  scanForMP3s();
  if (songCount == 0) {
    Serial.println("No MP3s found, staying in sensor mode");
    return;
  }

  currentSong = 0;
  startRecoveryAudio(currentSong);
  currentState = GROUND;
}

bool readImu() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;

  if (!icm.getEvent(&accel, &gyro, &temp, &mag)) return false;

  lastAccelX = accel.acceleration.x;
  lastAccelY = accel.acceleration.y;
  lastAccelZ = accel.acceleration.z;
  lastGyroX = gyro.gyro.x;
  lastGyroY = gyro.gyro.y;
  lastGyroZ = gyro.gyro.z;
  lastMagX = mag.magnetic.x;
  lastMagY = mag.magnetic.y;
  lastMagZ = mag.magnetic.z;
  lastImuTempC = temp.temperature;
  return true;
}

void appendImuLog(const ImuLogSample &sample) {
  if (!sdFound || !logFile || !imuFound) return;

  char row[256];
  int written = snprintf(
      row, sizeof(row),
      "%lu,%08lu,%06lu,%u,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      sample.elapsedMs, sample.rtcDate, sample.rtcTime, sample.flightPhase, sample.pressureHpa, sample.temperatureC,
      sample.altitudeM, sample.co2Ppm, sample.accelX, sample.accelY,
      sample.accelZ, sample.gyroX, sample.gyroY, sample.gyroZ,
      sample.magX, sample.magY, sample.magZ, sample.imuTempC);

  if (written <= 0 || written >= static_cast<int>(sizeof(row))) return;
  if (logBufferUsed + static_cast<size_t>(written) > LOG_BUFFER_SIZE) {
    flushLogBuffer();
  }
  if (static_cast<size_t>(written) <= LOG_BUFFER_SIZE - logBufferUsed) {
    memcpy(logBuffer + logBufferUsed, row, written);
    logBufferUsed += written;
  }
}

// Builds an ImuLogSample from the current cached readings and queues it.
// This is what loop() calls at the IMU_LOG_INTERVAL cadence.
void enqueueImuSample(unsigned long elapsedMs) {
  ImuLogSample sample;
  sample.elapsedMs    = elapsedMs;
  sample.rtcDate       = lastRtcDate;
  sample.rtcTime       = lastRtcTime;
  sample.flightPhase   = static_cast<uint8_t>(flightPhase);
  sample.pressureHpa   = lastPressureHpa;
  sample.temperatureC  = lastTempC;
  sample.altitudeM     = lastAltitude;
  sample.co2Ppm        = lastCO2;
  sample.accelX = lastAccelX; sample.accelY = lastAccelY; sample.accelZ = lastAccelZ;
  sample.gyroX  = lastGyroX;  sample.gyroY  = lastGyroY;  sample.gyroZ  = lastGyroZ;
  sample.magX   = lastMagX;   sample.magY   = lastMagY;   sample.magZ   = lastMagZ;
  sample.imuTempC = lastImuTempC;
  appendImuLog(sample);
}

// Creates /FLIGHTS/FLTxxx/imu.csv and opens it for buffered writing. Was
// called from setup() but never defined — added a minimal version here.
bool startFlightLog() {
  if (!createFlightDirectory()) return false;

  char imuPath[48];
  snprintf(imuPath, sizeof(imuPath), "%s/imu.csv", flightDirectory);
  logFile = SD.open(imuPath, FILE_WRITE);
  if (!logFile) {
    Serial.println("[SD] Could not open IMU log file");
    return false;
  }
  logFile.println("elapsed_ms,rtc_date,rtc_time,flight_phase,pressure_hPa,temp_C,altitude_m,co2_ppm,ax,ay,az,gx,gy,gz,mx,my,mz,imu_temp_C");
  logFile.flush();
  return true;
}


bool createFlightDirectory() {
  if (!SD.exists("/FLIGHTS") && !SD.mkdir("/FLIGHTS")) {
    Serial.println("[SD] Could not create /FLIGHTS");
    return false;
  }

  for (uint16_t index = 0; index < 1000; ++index) {
    snprintf(flightDirectory, sizeof(flightDirectory), "/FLIGHTS/FLT%03u", index);
    if (!SD.exists(flightDirectory)) {
      if (SD.mkdir(flightDirectory)) {
        Serial.print("[SD] Flight folder: ");
        Serial.println(flightDirectory);
        return true;
      }
      Serial.println("[SD] Could not create flight folder");
      return false;
    }
  }

  Serial.println("[SD] No unused flight folder number");
  return false;
}

void flushLogBuffer() {
  if (!sdFound || !logFile || logBufferUsed == 0) return;
  logFile.write(reinterpret_cast<const uint8_t *>(logBuffer), logBufferUsed);
  logBufferUsed = 0;
}

// Refresh the cached RTC date/time used to stamp IMU rows (call this ~1 Hz)
void updateRtcCache() {
  if (!rtcFound) return;
  DateTime now = rtc.now();
  lastRtcDate = static_cast<uint32_t>(now.year()) * 10000UL
              + static_cast<uint32_t>(now.month()) * 100UL
              + static_cast<uint32_t>(now.day());
  lastRtcTime = static_cast<uint32_t>(now.hour()) * 10000UL
              + static_cast<uint32_t>(now.minute()) * 100UL
              + static_cast<uint32_t>(now.second());
}



// Setup
void setup() {
  Serial.begin(115200);
  delay(2000);

  //Remove before flight 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  //MOSFET
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH); //Give power to the 2W 

  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("=== KAOS-2 Flight Computer ===");

  // ICM-20948 shares the 100 kHz primary I2C bus. Continue without it if
  // absent so the rest of the payload can still run during bring-up.
  imuFound = icm.begin_I2C(0x69, &Wire);
  if (!imuFound) imuFound = icm.begin_I2C(0x68, &Wire);
  if (imuFound) {
    icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setAccelRateDivisor(10); // 1125 / (1 + 10) = about 102 Hz.
    icm.setGyroRateDivisor(10);  // 1100 / (1 + 10) = 100 Hz.
    Serial.println("[ICM20948] OK: 100 Hz gyro target");
  } else {
    Serial.println("[ICM20948] FAIL — continuing without IMU logging");
  }

  // BMP388
  if (!bmp.begin_I2C(0x77, &Wire)) {
    if (!bmp.begin_I2C(0x76, &Wire)) {
      Serial.println("[BMP388] FAIL"); while (1);
    }
  }
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("[BMP388] OK");

  // Baseline pressure
  float sum = 0; int cnt = 0;
  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) { sum += bmp.pressure; cnt++; }
    delay(50);
  }
  groundPressure_hPa = (cnt > 0) ? (sum / cnt) / 100.0 : 1013.25;
  Serial.print("[BMP388] Baseline: ");
  Serial.print(groundPressure_hPa);
  Serial.println(" hPa");

  // SCD30
  scd30Found = scd30.begin(0x61, &Wire);
  Serial.println(scd30Found ? "[SCD30] OK" : "[SCD30] FAIL — continuing");

  // RTC
  rtcFound = rtc.begin();
  if (rtcFound) {
    if (!rtc.initialized() || rtc.lostPower())
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("[RTC] OK");
  } else {
    Serial.println("[RTC] FAIL");
  }

  // SD
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] FAIL — logging disabled");
  } else {
    sdFound = true;
    Serial.println("[SD] OK");
    if (!startFlightLog()) {
      sdFound = false;
      Serial.println("[SD] Flight log setup failed — logging disabled");
    }
  }

  // Start Timer
  startTime = millis();
  lastImuLogTime = millis();
  loggerLastFlushMs = lastImuLogTime;
  loggerCoreReady = sdFound; 

  Serial.println("Sensors are ok, starting loop...");

}

// LOOP
void loop() {

  unsigned long now = millis();

  if (currentState == FLIGHT) {

    // Continuously read + log sensors during flight (1 Hz)
    if (now - lastSensorRead >= SENSOR_INTERVAL) {
      lastSensorRead = now;
      if (bmp.performReading()) {
        lastPressureHpa = bmp.pressure / 100.0;
        lastTempC       = bmp.temperature;
        lastAltitude    = calculateAltitudeAboveBaseline(bmp.pressure, groundPressure_hPa);
      }
      if (scd30Found && scd30.dataReady() && scd30.read()) {
        lastCO2 = scd30.CO2;
      }
      updateRtcCache();
    }

    // IMU sampling at its own ~100 Hz cadence — pulled out of the 1 Hz
    // sensor block above, where it was previously only evaluated once a second.
    if (imuFound && now - lastImuLogTime >= IMU_LOG_INTERVAL) {
      lastImuLogTime += IMU_LOG_INTERVAL;
      if (now - lastImuLogTime >= IMU_LOG_INTERVAL) {
        // Recover cleanly if a slow SD flush delayed the loop.
        lastImuLogTime = now;
      }
      if (readImu()) enqueueImuSample(now);
    }

    if (now - lastLogTime >= LOG_INTERVAL) {
      lastLogTime = now;
      logToSD(lastPressureHpa, lastTempC, lastAltitude, lastCO2);
    }

    // Periodically flush the buffered IMU log to SD
    if (now - loggerLastFlushMs >= SD_FLUSH_INTERVAL) {
      loggerLastFlushMs = now;
      flushLogBuffer();
      if (logFile) logFile.flush();
    }

    // Once the ascent/descent timer is up, switch to ground/audio mode
    if (now - startTime >= TIMER_DURATION && !groundModeTriggered) {
      Serial.println("3 hours 30 minutes is up!");
      groundModeTriggered = true;
      enterGroundMode(); // sets currentState = GROUND on success
    }

  } else if (currentState == GROUND) {

    // Keep decoder fed (no delays ever in this branch)
    if (mp3 && mp3->isRunning()) {
      if (!mp3->loop()) {
        // Track finished → play next
        startRecoveryAudio((currentSong + 1) % songCount);
      }
    }
  }
}
