//BMP
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1000) //constant given for daily pressure
float groundPressure_hPa = 0;
Adafruit_BMP3XX bmp;


float calculateAltitude(float atmospheric, float avg) {
  atmospheric = atmospheric / 100.0;
  return 44330.0 * (1.0 - pow(atmospheric / avg, 0.1903));
} //Stolen Valor


void setup(){
    Serial.begin(115200);
    while (!Serial) { ; }                 // optional on some boards
    if (!bmp.begin_I2C(0x77)) {
        Serial.println(F("BMP388 not found at 0x77, trying 0x76..."));  //Standard addresses for the I2C PINS SDA 21 AND SCL 22
        if (!bmp.begin_I2C(0x76)) {
            Serial.println(F("ERROR: Could not find BMP388 sensor, CHECK WIRING"));
            while (1) delay(10);
        }
    }

    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // 1. Discard the first few readings while the sensor stabilizes
    for(int i = 0; i < 5; i++) {
       bmp.performReading();
       //delay(100);
    }
    delay(100);
    // 2. Take 10 readings and add them together
    float pressureSum = 0;
    for(int i = 0; i < 10; i++) {
        bmp.performReading();
        pressureSum += bmp.pressure; 
        //delay(50);
    }

    // 3. Find the average and convert from Pascals to hPa
    groundPressure_hPa = (pressureSum / 10.0) / 100.0;

}

void loop(){
    if (!bmp.performReading()) { Serial.println("Read fail"); delay(200); return; }
        float atmospheric = bmp.pressure;
        Serial.print("Altitude is: ");Serial.println(calculateAltitude(atmospheric, groundPressure_hPa),2);
        delay(1000);
}
