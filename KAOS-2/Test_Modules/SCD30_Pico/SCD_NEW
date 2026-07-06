#include <Adafruit_SCD30.h>
#include <Wire.h>

Adafruit_SCD30 scd30;

// =====================================================================
// 1. SET YOUR CALIBRATION PARAMETERS HERE:
// =====================================================================
const uint16_t MY_ALTITUDE_METERS = 30;      // Enter your local elevation above sea level in meters
const uint16_t MY_TEMP_OFFSET_TICKS = 150;   // 150 = 1.50 degrees C offset (Set to 0 if unsure!)
const bool     ENABLE_AUTO_CALIBRATION = true; // True for 7-day auto-calibration

// Set to true ONLY when standing outdoors in fresh air for 3+ minutes!
const bool     TRIGGER_FORCED_RECALIBRATION = false; 
const uint16_t OUTDOOR_REFERENCE_CO2 = 415;  // Global average outdoor CO2 ppm
// =====================================================================

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("--- SCD-30 Calibration Utility ---");

  // Assign I2C0 pins for Raspberry Pi Pico (GP0 / GP1)
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  if (!scd30.begin()) {
    Serial.println("ERROR: Failed to find SCD30 chip. Check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("SCD-30 Detected Successfully!");

  // 1. Apply Altitude Compensation
  scd30.setAltitudeOffset(MY_ALTITUDE_METERS);
  Serial.print("Altitude Offset set to: ");
  Serial.print(MY_ALTITUDE_METERS);
  Serial.println(" meters");

  // 2. Apply Temperature Offset (in hundredths of a degree C)
  scd30.setTemperatureOffset(MY_TEMP_OFFSET_TICKS);
  Serial.print("Temperature Offset set to: ");
  Serial.print(MY_TEMP_OFFSET_TICKS / 100.0);
  Serial.println(" degrees C");

  // 3. Configure Automatic Self-Calibration (ASC)
  scd30.selfCalibrationEnabled(ENABLE_AUTO_CALIBRATION);
  Serial.print("Automatic Self-Calibration (ASC): ");
  Serial.println(ENABLE_AUTO_CALIBRATION ? "ENABLED" : "DISABLED");

  // 4. Execute Forced Recalibration (FRC) if requested
  if (TRIGGER_FORCED_RECALIBRATION) {
    Serial.println("\n[WARNING] Executing Forced Recalibration in 5 seconds...");
    Serial.println("Ensure sensor is in steady outdoor air (~415 ppm)!");
    delay(5000);
    
    if (scd30.forceRecalibrationWithReference(OUTDOOR_REFERENCE_CO2)) {
      Serial.print("SUCCESS: Forced baseline calibrated to ");
      Serial.print(OUTDOOR_REFERENCE_CO2);
      Serial.println(" ppm!");
    } else {
      Serial.println("ERROR: Forced recalibration failed to write.");
    }
  }

  Serial.println("\nStarting live sensor readings...\n");
}

void loop() {
  if (scd30.dataReady()){
    if (!scd30.read()){ 
      Serial.println("Error reading sensor data"); 
      return; 
    }

    Serial.print("Temp: ");
    Serial.print(scd30.temperature, 2);
    Serial.print(" °C | RH: ");
    Serial.print(scd30.relative_humidity, 2);
    Serial.print(" % | CO2: ");
    Serial.print(scd30.CO2, 1);
    Serial.println(" ppm");
  }

  delay(2000); // Read every 2 seconds
}
