//  KAOS-2
//  FLIGHT COMPUTER — Pico
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
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>
#include <hardware/sync.h>

// ── I2S (PCM5122) ───────────────────────────────────────────────
#define I2S_BCLK  26
#define I2S_LRCLK 27
#define I2S_DOUT  28

// ── I2C ─────────────────────────────────────────────────────────
#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5

// The ICM-20948 shares the primary I2C bus with the BMP388, SCD30, and OLED:
// GP4 (physical pin 6) is SDA/SDI and GP5 (physical pin 7) is SCL/SCLK.

// ── Pi Zero graceful shutdown signal ────────────────────────────
// GP14 -> Pi GPIO20. The Pi listener treats LOW as a shutdown request.
#define PI_ZERO_SHUTDOWN_PIN 14
#define PI_ZERO_SHUTDOWN_ACTIVE LOW

// ── SPI / SD ────────────────────────────────────────────────────
#define SD_CS 17

// ── OLED ────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Sensors ─────────────────────────────────────────────────────
Adafruit_BMP3XX bmp;
Adafruit_SCD30  scd30;
RTC_PCF8523     rtc;
Adafruit_ICM20948 icm;

bool scd30Found = false;
bool rtcFound   = false;
volatile bool sdFound = false;
bool imuFound   = false;
File logFile;

float groundPressure_hPa = 0.0;

// ── Pressure-only landing detector ──────────────────────────────
// These thresholds are intentionally used only in the BMP388's useful
// lower-atmosphere range. IMU data is not part of this first decision path.
#define BASELINE_DURATION_MS (60UL * 1000UL)
#define ASCENT_DROP_HPA 150.0f
#define ASCENT_CONFIRM_MS (60UL * 1000UL)
#define DESCENT_RISE_HPA 100.0f
#define DESCENT_CONFIRM_MS (60UL * 1000UL)
#define NEAR_GROUND_HPA 20.0f
#define LANDING_CONFIRM_MS (5UL * 60UL * 1000UL)
#define LANDING_PRESSURE_RANGE_HPA 2.0f

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

// ── Audio ────────────────────────────────────────────────────────
const char *const RECOVERY_AUDIO_FILE = "/AlarmF.mp3";

AudioFileSourceSD *source = nullptr;
AudioGeneratorMP3 *mp3    = nullptr;
AudioOutputI2S    *output = nullptr;
bool audioReady = false;
bool recoveryAudioStarted = false;
bool recoveryAudioFailed = false;

// ── Timers ───────────────────────────────────────────────────────
unsigned long lastSensorRead = 0;
unsigned long lastImuLogTime = 0;
#define SENSOR_INTERVAL 1000
#define IMU_LOG_INTERVAL 10
#define SD_FLUSH_INTERVAL 10000
#define LOG_SEGMENT_INTERVAL (15UL * 60UL * 1000UL)

// Core 1 writes small batches; an expensive filesystem sync happens every 10 s.
#define LOG_BUFFER_SIZE 2048
char logBuffer[LOG_BUFFER_SIZE];
size_t logBufferUsed = 0;
char flightDirectory[32];
uint16_t logSegmentIndex = 0;
unsigned long logSegmentStartedMs = 0;
unsigned long loggerLastFlushMs = 0;

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

// Core 0 produces samples. Core 1 owns all SD work and consumes samples.
#define IMU_QUEUE_CAPACITY 512
ImuLogSample imuQueue[IMU_QUEUE_CAPACITY];
volatile uint16_t imuQueueHead = 0;
volatile uint16_t imuQueueTail = 0;
volatile uint32_t droppedImuSamples = 0;
volatile bool loggerCoreReady = false;
volatile bool loggerFlushRequested = false;

const char *LOG_HEADER =
    "Elapsed_ms,RTC_Date,RTC_Time,Flight_Phase,Pressure_hPa,Temperature_C,Altitude_m,CO2_ppm,"
    "AccelX_mps2,AccelY_mps2,AccelZ_mps2,"
    "GyroX_rads,GyroY_rads,GyroZ_rads,"
    "MagX_uT,MagY_uT,MagZ_uT,IMU_Temp_C";

// ── Cached sensor values ─────────────────────────────────────────
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

// ────────────────────────────────────────────────────────────────
//  Sensor helpers
// ────────────────────────────────────────────────────────────────
float calculateAltitudeAboveBaseline(float pressurePa, float baselinePressureHpa) {
  float pressureHpa = pressurePa / 100.0;
  return 44330.0 * (1.0 - pow(pressureHpa / baselinePressureHpa, 0.1903));
}

void updateRtcCache() {
  if (!rtcFound) return;
  DateTime now = rtc.now();
  lastRtcDate = static_cast<uint32_t>(now.year()) * 10000UL
              + static_cast<uint32_t>(now.month()) * 100UL
              + now.day();
  lastRtcTime = static_cast<uint32_t>(now.hour()) * 10000UL
              + static_cast<uint32_t>(now.minute()) * 100UL
              + now.second();
}

const char *flightPhaseName(FlightPhase phase) {
  switch (phase) {
    case BASELINING: return "BASELINING";
    case WAITING_FOR_ASCENT: return "WAITING_FOR_ASCENT";
    case ASCENT_CONFIRMED: return "ASCENT_CONFIRMED";
    case DESCENT_CONFIRMED: return "DESCENT_CONFIRMED";
    case GROUND_CANDIDATE: return "GROUND_CANDIDATE";
    case LANDED: return "LANDED";
  }
  return "UNKNOWN";
}

void setFlightPhase(FlightPhase nextPhase) {
  if (flightPhase == nextPhase) return;
  flightPhase = nextPhase;
  Serial.print("[LANDING] Phase: ");
  Serial.println(flightPhaseName(flightPhase));
}

void beginPayloadShutdown() {
  if (payloadShutdownStarted) return;

  payloadShutdownStarted = true;
  loggerFlushRequested = true;
  loggerShutdownRequested = true;
  Serial.println("[LANDING] Landing confirmed; stopping sensors and closing log");

  if (!loggerCoreReady) {
    loggerShutdownComplete = true;
  }
}

void updateLandingDetector(unsigned long now, float pressureHpa) {
  if (payloadShutdownStarted || !isfinite(pressureHpa)) return;

  if (flightPhase == BASELINING) {
    if (baselineStartedMs == 0) baselineStartedMs = now;
    baselinePressureSum += pressureHpa;
    ++baselinePressureCount;

    if (now - baselineStartedMs >= BASELINE_DURATION_MS && baselinePressureCount > 0) {
      groundPressure_hPa = baselinePressureSum / baselinePressureCount;
      minimumFlightPressureHpa = groundPressure_hPa;
      Serial.print("[LANDING] Launch baseline: ");
      Serial.print(groundPressure_hPa, 2);
      Serial.println(" hPa");
      setFlightPhase(WAITING_FOR_ASCENT);
    }
    return;
  }

  if (flightPhase == WAITING_FOR_ASCENT) {
    if (pressureHpa <= groundPressure_hPa - ASCENT_DROP_HPA) {
      if (ascentDropStartedMs == 0) {
        ascentDropStartedMs = now;
        Serial.println("[LANDING] Large pressure drop seen; confirming ascent");
      }
      if (now - ascentDropStartedMs >= ASCENT_CONFIRM_MS) {
        minimumFlightPressureHpa = pressureHpa;
        setFlightPhase(ASCENT_CONFIRMED);
      }
    } else {
      ascentDropStartedMs = 0;
    }
    return;
  }

  if (pressureHpa < minimumFlightPressureHpa) {
    minimumFlightPressureHpa = pressureHpa;
  }

  if (flightPhase == ASCENT_CONFIRMED) {
    if (pressureHpa >= minimumFlightPressureHpa + DESCENT_RISE_HPA) {
      if (descentStartedMs == 0) {
        descentStartedMs = now;
        Serial.println("[LANDING] Pressure is rising; confirming descent");
      }
      if (now - descentStartedMs >= DESCENT_CONFIRM_MS) {
        setFlightPhase(DESCENT_CONFIRMED);
      }
    } else {
      descentStartedMs = 0;
    }
    return;
  }

  bool nearGround = fabsf(pressureHpa - groundPressure_hPa) <= NEAR_GROUND_HPA;
  if (flightPhase == DESCENT_CONFIRMED && nearGround) {
    groundCandidateStartedMs = now;
    landingPressureMinHpa = pressureHpa;
    landingPressureMaxHpa = pressureHpa;
    setFlightPhase(GROUND_CANDIDATE);
    Serial.println("[LANDING] Near launch pressure; starting stability timer");
    return;
  }

  if (flightPhase == GROUND_CANDIDATE) {
    if (!nearGround) {
      groundCandidateStartedMs = 0;
      setFlightPhase(DESCENT_CONFIRMED);
      Serial.println("[LANDING] Pressure left ground window; resuming descent watch");
      return;
    }

    if (pressureHpa < landingPressureMinHpa) landingPressureMinHpa = pressureHpa;
    if (pressureHpa > landingPressureMaxHpa) landingPressureMaxHpa = pressureHpa;

    if (now - groundCandidateStartedMs >= LANDING_CONFIRM_MS) {
      float pressureRange = landingPressureMaxHpa - landingPressureMinHpa;
      if (pressureRange <= LANDING_PRESSURE_RANGE_HPA) {
        setFlightPhase(LANDED);
        beginPayloadShutdown();
      } else {
        groundCandidateStartedMs = now;
        landingPressureMinHpa = pressureHpa;
        landingPressureMaxHpa = pressureHpa;
        Serial.println("[LANDING] Ground window unstable; restarting stability timer");
      }
    }
  }
}

void updatePayloadShutdown() {
  if (!payloadShutdownStarted || !loggerShutdownComplete) return;

  if (!piShutdownSignaled) {
    digitalWrite(PI_ZERO_SHUTDOWN_PIN, PI_ZERO_SHUTDOWN_ACTIVE);
    piShutdownSignaled = true;
    Serial.println("[SHUTDOWN] Pi Zero shutdown request sent");
  }

  serviceRecoveryAudio();
}

void printFlightStatus() {
  Serial.print("[STATUS] phase=");
  Serial.print(flightPhaseName(flightPhase));
  Serial.print(" pressure_hPa=");
  Serial.print(lastPressureHpa, 2);
  Serial.print(" baseline_hPa=");
  Serial.print(groundPressure_hPa, 2);
  Serial.print(" dropped_imu=");
  Serial.println(droppedImuSamples);
}

void handleSerialCommand() {
  serialCommand[serialCommandLength] = '\0';

  if (strcmp(serialCommand, "shutdown") == 0) {
    Serial.println("[SERIAL] Manual shutdown requested");
    setFlightPhase(LANDED);
    beginPayloadShutdown();
  } else if (strcmp(serialCommand, "status") == 0) {
    printFlightStatus();
  } else if (serialCommandLength > 0) {
    Serial.println("[SERIAL] Unknown command");
  }

  serialCommandLength = 0;
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      handleSerialCommand();
      continue;
    }
    if (serialCommandLength < SERIAL_COMMAND_SIZE - 1) {
      serialCommand[serialCommandLength++] = incoming;
    }
  }
}

void flushLogBuffer() {
  if (!sdFound || !logFile || logBufferUsed == 0) return;
  logFile.write(reinterpret_cast<const uint8_t *>(logBuffer), logBufferUsed);
  logBufferUsed = 0;
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

bool openLogSegment() {
  if (logFile) {
    flushLogBuffer();
    logFile.flush();
    logFile.close();
  }

  char logPath[64];
  snprintf(logPath, sizeof(logPath), "%s/IMU%03u.CSV", flightDirectory,
           logSegmentIndex);
  logFile = SD.open(logPath, FILE_WRITE);
  if (!logFile) {
    Serial.print("[SD] Could not open ");
    Serial.println(logPath);
    return false;
  }

  logFile.println(LOG_HEADER);
  logFile.flush();
  logSegmentStartedMs = millis();
  Serial.print("[SD] Logging to ");
  Serial.println(logPath);
  return true;
}

bool startFlightLog() {
  logSegmentIndex = 0;
  return createFlightDirectory() && openLogSegment();
}

void rollLogSegmentIfNeeded(unsigned long now) {
  if (!sdFound || !logFile || now - logSegmentStartedMs < LOG_SEGMENT_INTERVAL) {
    return;
  }

  ++logSegmentIndex;
  if (!openLogSegment()) {
    sdFound = false;
    Serial.println("[SD] Logging disabled after segment rollover failure");
  }
}

bool enqueueImuSample(unsigned long elapsedMs) {
  uint16_t head = imuQueueHead;
  uint16_t nextHead = (head + 1) % IMU_QUEUE_CAPACITY;
  if (nextHead == imuQueueTail) {
    ++droppedImuSamples;
    return false;
  }

  ImuLogSample &sample = imuQueue[head];
  sample.elapsedMs = elapsedMs;
  sample.rtcDate = lastRtcDate;
  sample.rtcTime = lastRtcTime;
  sample.flightPhase = static_cast<uint8_t>(flightPhase);
  sample.pressureHpa = lastPressureHpa;
  sample.temperatureC = lastTempC;
  sample.altitudeM = lastAltitude;
  sample.co2Ppm = lastCO2;
  sample.accelX = lastAccelX;
  sample.accelY = lastAccelY;
  sample.accelZ = lastAccelZ;
  sample.gyroX = lastGyroX;
  sample.gyroY = lastGyroY;
  sample.gyroZ = lastGyroZ;
  sample.magX = lastMagX;
  sample.magY = lastMagY;
  sample.magZ = lastMagZ;
  sample.imuTempC = lastImuTempC;

  __dmb();
  imuQueueHead = nextHead;
  return true;
}

bool dequeueImuSample(ImuLogSample &sample) {
  uint16_t tail = imuQueueTail;
  if (tail == imuQueueHead) return false;

  __dmb();
  sample = imuQueue[tail];
  __dmb();
  imuQueueTail = (tail + 1) % IMU_QUEUE_CAPACITY;
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

void updateOLED_Sensors(float pressureHpa, float tempC, float altitude, float co2) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("P:   "); display.print(pressureHpa, 1); display.println(" hPa");
  display.print("T:   "); display.print(tempC, 1);       display.println(" C");
  display.print("Alt: "); display.print(altitude + 160, 1); display.println(" m");
  display.print("CO2: ");
  if (co2 < 0) display.println("Wait...");
  else { display.print(co2, 0); display.println(" ppm"); }
  if (rtcFound) {
    DateTime now = rtc.now();
    display.setCursor(0, 56);
    display.print(now.hour()); display.print(":");
    if (now.minute() < 10) display.print("0");
    display.print(now.minute()); display.print(":");
    if (now.second() < 10) display.print("0");
    display.print(now.second());
  }
  display.display();
}

void updateOLED_RecoveryAudio() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Recovery audio");
  display.println("");
  display.println("Landing confirmed");
  display.println("Pi shutdown sent");
  display.println("Playing AlarmF.mp3");
  display.setCursor(0, 56);
  display.println("Recovery beacon active");
  display.display();
}


//  Audio helpers
void stopCurrent() {
  if (mp3) {
    if (mp3->isRunning()) mp3->stop();
    delete mp3; mp3 = nullptr;
  }
  if (source) { delete source; source = nullptr; }
}

void startRecoveryAudio() {
  if (recoveryAudioFailed) return;

  if (!SD.exists(RECOVERY_AUDIO_FILE)) {
    Serial.println("[AUDIO] Missing /AlarmF.mp3; recovery audio disabled");
    recoveryAudioFailed = true;
    return;
  }

  if (!audioReady) {
    output = new AudioOutputI2S();
    output->SetPinout(I2S_BCLK, I2S_LRCLK, I2S_DOUT);
    output->SetChannels(2);
    output->SetGain(0.5);
    audioReady = true;
  }

  stopCurrent();
  Serial.println("[AUDIO] Playing /AlarmF.mp3");
  source = new AudioFileSourceSD(RECOVERY_AUDIO_FILE);
  mp3    = new AudioGeneratorMP3();
  if (!mp3->begin(source, output)) {
    Serial.println("[AUDIO] Could not start /AlarmF.mp3");
    stopCurrent();
    recoveryAudioFailed = true;
  }
}

void serviceRecoveryAudio() {
  if (recoveryAudioFailed) return;

  if (!recoveryAudioStarted) {
    recoveryAudioStarted = true;
    updateOLED_RecoveryAudio();
    startRecoveryAudio();
    return;
  }

  if (!mp3 || !mp3->isRunning() || !mp3->loop()) {
    startRecoveryAudio();
  }
}

//  SETUP
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(PI_ZERO_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(PI_ZERO_SHUTDOWN_PIN, HIGH);

  Wire.setSDA(PICO_SDA_PIN);
  Wire.setSCL(PICO_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("=== FLIGHT COMPUTER BOOT ===");

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] FAIL"); while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
  Serial.println("[OLED] OK");

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
    DateTime rtcNow = rtc.now();
    bool rtcDateInvalid = rtcNow.year() < 2025 || rtcNow.year() > 2040;
    if (!rtc.initialized() || rtc.lostPower() || rtcDateInvalid) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("[RTC] Set from sketch build time");
    }
    updateRtcCache();
    Serial.println("[RTC] OK");
    Serial.printf("[RTC] Current: %04lu-%02lu-%02lu %02lu:%02lu:%02lu\n",
                  lastRtcDate / 10000UL, (lastRtcDate / 100UL) % 100UL,
                  lastRtcDate % 100UL, lastRtcTime / 10000UL,
                  (lastRtcTime / 100UL) % 100UL, lastRtcTime % 100UL);
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

  // Boot screen
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("Boot complete");
  display.print("BMP388: "); display.println("OK");
  display.print("ICM20948: "); display.println(imuFound ? "OK" : "FAIL");
  display.print("SCD30:  "); display.println(scd30Found ? "OK" : "FAIL");
  display.print("RTC:    "); display.println(rtcFound   ? "OK" : "FAIL");
  display.print("SD:     "); display.println(sdFound    ? "OK" : "FAIL");
  display.display();
  delay(2000);

  lastImuLogTime = millis();
  loggerLastFlushMs = lastImuLogTime;
  loggerCoreReady = sdFound;

  Serial.println("[LANDING] Collecting 60 second launch-pressure baseline");
  Serial.println("=== LOOP START ===");
}

//  LOOP
void loop() {
  unsigned long now = millis();
  pollSerialCommands();

  if (payloadShutdownStarted) {
    updatePayloadShutdown();
    return;
  }

  // Non-blocking sensor read
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    updateRtcCache();
    if (bmp.performReading()) {
      lastPressureHpa = bmp.pressure / 100.0;
      lastTempC       = bmp.temperature;
      lastAltitude    = calculateAltitudeAboveBaseline(bmp.pressure, groundPressure_hPa);
      updateLandingDetector(now, lastPressureHpa);
    }
    if (scd30Found && scd30.dataReady() && scd30.read()) {
      lastCO2 = scd30.CO2;
    }
    updateOLED_Sensors(lastPressureHpa, lastTempC, lastAltitude, lastCO2);
  }

  // Sample and queue the IMU row every 10 ms. Environment values are the
  // most recent 1 Hz readings, while IMU values are freshly sampled.
  if (imuFound && now - lastImuLogTime >= IMU_LOG_INTERVAL) {
    lastImuLogTime += IMU_LOG_INTERVAL;
    if (now - lastImuLogTime >= IMU_LOG_INTERVAL) {
      // Recover cleanly if a slow SD flush delayed the loop.
      lastImuLogTime = now;
    }
    if (readImu()) enqueueImuSample(now);
  }
}

// Core 1 owns SD formatting, writes, flushes, and segment rollover. This
// prevents occasional SD-card latency from interrupting the 100 Hz IMU loop.
void setup1() {}

void loop1() {
  if (!loggerCoreReady) return;

  ImuLogSample sample;
  while (dequeueImuSample(sample)) {
    appendImuLog(sample);
  }

  if (loggerShutdownRequested && imuQueueTail == imuQueueHead) {
    flushLogBuffer();
    if (logFile) {
      logFile.flush();
      logFile.close();
    }
    loggerShutdownComplete = true;
    loggerCoreReady = false;
    return;
  }

  unsigned long now = millis();
  if (loggerFlushRequested || now - loggerLastFlushMs >= SD_FLUSH_INTERVAL) {
    flushLogBuffer();
    if (logFile) logFile.flush();
    loggerLastFlushMs = now;
    loggerFlushRequested = false;
  }

  rollLogSegmentIfNeeded(now);
}
