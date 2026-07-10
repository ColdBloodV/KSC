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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

// ── I2S (PCM5122) ───────────────────────────────────────────────
#define I2S_BCLK  26
#define I2S_LRCLK 27
#define I2S_DOUT  28

// ── Button ──────────────────────────────────────────────────────
#define BUTTON_PIN 15

// ── I2C ─────────────────────────────────────────────────────────
#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5

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

bool scd30Found = false;
bool rtcFound   = false;
bool sdFound    = false;

float groundPressure_hPa = 0.0;

// ── Audio ────────────────────────────────────────────────────────
#define MAX_SONGS 20
char songList[MAX_SONGS][64];
int  songCount   = 0;
int  currentSong = 0;

AudioFileSourceSD *source = nullptr;
AudioGeneratorMP3 *mp3    = nullptr;
AudioOutputI2S    *output = nullptr;
bool audioReady = false;

// ── State ────────────────────────────────────────────────────────
enum State { SENSORS, AUDIO };
State currentState = SENSORS;

// ── Timers ───────────────────────────────────────────────────────
unsigned long lastSensorRead = 0;
unsigned long lastLogTime    = 0;
unsigned long lastPressTime  = 0;
#define SENSOR_INTERVAL 1000
#define LOG_INTERVAL    1000

// ── Cached sensor values ─────────────────────────────────────────
float lastPressureHpa = 0;
float lastTempC       = 0;
float lastAltitude    = 0;
float lastCO2         = -1;

// ── Button state ─────────────────────────────────────────────────
int buttonValue     = 0;
int lastButtonValue = 0;

// ────────────────────────────────────────────────────────────────
//  Sensor helpers
// ────────────────────────────────────────────────────────────────
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

void updateOLED_Audio() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("  >> AUDIO MODE <<");
  display.println("");
  display.print("Track ");
  display.print(currentSong + 1);
  display.print(" / ");
  display.println(songCount);
  display.println("");
  String name = String(songList[currentSong]);
  if (name.startsWith("/")) name = name.substring(1);
  display.println(name);
  display.setCursor(0, 56);
  display.println("Press btn to stop");
  display.display();
}


//  Audio helpers
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

void playSong(int index) {
  stopCurrent();
  Serial.printf("Playing [%d/%d]: %s\n", index + 1, songCount, songList[index]);
  source = new AudioFileSourceSD(songList[index]);
  mp3    = new AudioGeneratorMP3();
  if (!mp3->begin(source, output)) {
    Serial.println("ERROR: MP3 failed to start, skipping...");
    currentSong = (currentSong + 1) % songCount;
    playSong(currentSong);
  }
}

//  Mode switching
void enterAudioMode() {
  Serial.println(">> Entering AUDIO mode — sensors paused");

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
  playSong(currentSong);
  currentState = AUDIO;
  updateOLED_Audio();
}

void exitAudioMode() {
  Serial.println(">> Exiting AUDIO mode — resuming sensors");
  stopCurrent();
  currentState = SENSORS;
  lastSensorRead = 0; // force immediate sensor read on return
  lastLogTime    = 0;
}

//  SETUP
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
    File file = SD.open("flight.csv", FILE_WRITE);
    if (file) {
      if (file.size() == 0)
        file.println("Date,Time,Pressure_hPa,Temperature_C,Altitude_m,CO2_ppm");
      file.close();
    }
  }

  // Boot screen
  display.clearDisplay(); display.setCursor(0, 0);
  display.println("Boot complete");
  display.print("BMP388: "); display.println("OK");
  display.print("SCD30:  "); display.println(scd30Found ? "OK" : "FAIL");
  display.print("RTC:    "); display.println(rtcFound   ? "OK" : "FAIL");
  display.print("SD:     "); display.println(sdFound    ? "OK" : "FAIL");
  display.display();
  delay(2000);

  Serial.println("=== LOOP START ===");
}

//  LOOP
void loop() {

  unsigned long now = millis();

  // Button debounce
  buttonValue = digitalRead(BUTTON_PIN);
  bool buttonPressed = (buttonValue == LOW
                     && lastButtonValue == HIGH
                     && now - lastPressTime > 250);
  if (buttonPressed) lastPressTime = now;

  // SENSOR MODE 
  if (currentState == SENSORS) {

    // Button → switch to audio, sensors/logging stop automatically
    if (buttonPressed) {
      enterAudioMode();
      lastButtonValue = buttonValue;
      return;
    }

    // Non-blocking sensor read
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
      updateOLED_Sensors(lastPressureHpa, lastTempC, lastAltitude, lastCO2);
    }

    // Non-blocking SD log
    if (now - lastLogTime >= LOG_INTERVAL) {
      lastLogTime = now;
      logToSD(lastPressureHpa, lastTempC, lastAltitude, lastCO2);
    }
  }

  // AUDIO MODE 
  else if (currentState == AUDIO) {

    // Button → stop audio, go back to sensors
    if (buttonPressed) {
      exitAudioMode();
      lastButtonValue = buttonValue;
      return;
    }

    // Keep decoder fed — no delays ever in this branch!
    if (mp3 && mp3->isRunning()) {
      if (!mp3->loop()) {
        // Track finished → play next
        currentSong = (currentSong + 1) % songCount;
        playSong(currentSong);
        updateOLED_Audio();
      }
    }
  }

  lastButtonValue = buttonValue;
}
