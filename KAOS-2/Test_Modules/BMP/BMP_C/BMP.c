#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp3.h"
#include <math.h>

// Set your I2C pins
// By default, Adafruit BMP3XX boards use I2C address 0x77
#define BMP3_I2C_ADDR 0x77
#define I2C_PORT i2c0 
#define I2C_SDA_PIN 4      // <--- Make sure these match the GPIO numbers you are currently plugged into
#define I2C_SCL_PIN 5

// 1. The I2C Read Wrapper
BMP3_INTF_RET_TYPE pico_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    // Tell the sensor which register we want to read
    i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true);
    // Read the data from that register
    i2c_read_blocking(I2C_PORT, dev_addr, reg_data, len, false);
    
    return BMP3_INTF_RET_SUCCESS;
}

// 2. The I2C Write Wrapper
BMP3_INTF_RET_TYPE pico_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    
    // The Pico requires the register address and data to be sent in one continuous array
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for(uint32_t i = 0; i < len; i++) {
        buf[i + 1] = reg_data[i];
    }
    
    i2c_write_blocking(I2C_PORT, dev_addr, buf, len + 1, false);
    return BMP3_INTF_RET_SUCCESS;
}

// 3. The Delay Wrapper
void pico_delay_us(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

float calculateAltitude(float atmospheric, float avg) {
  return 44330.0 * (1.0 - pow(atmospheric / avg, 0.1903));
}

int main() {
    stdio_init_all();
    
    // Give yourself 2 seconds to open the Serial Monitor after plugging it in
    sleep_ms(2000); 
    printf("Starting BMP3XX I2C Test...\n");

    i2c_init(I2C_PORT, 100 * 1000); // Initialize I2C at 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    struct bmp3_dev dev;
    uint8_t dev_addr = BMP3_I2C_ADDR;

    dev.intf = BMP3_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.read = pico_i2c_read;
    dev.write = pico_i2c_write;
    dev.delay_us = pico_delay_us;

    int8_t rslt = bmp3_init(&dev);
    if (rslt != BMP3_OK) {
        printf("BMP3 initialization failed with code: %d\n", rslt);
        while (1) {
            sleep_ms(1000); // Keep the program running to allow for debugging
        }
    }
    printf("BMP3 initialized successfully!\n");

    struct bmp3_settings settings = {0};
    
    // Select which settings we want to change
    uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
    
    // Turn on pressure and temperature
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    // Set oversampling (x8 pressure, x1 temp)
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    // Set output data rate to 50Hz
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    
    // Apply the settings to the sensor
    bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    
    // Set to Normal Mode (continuous reading)
    settings.op_mode = BMP3_MODE_NORMAL;
    bmp3_set_op_mode(&settings, &dev);

    // 5. The Infinite Loop
    struct bmp3_data data = {0};

    // Throw away the first 10 readings to allow the sensor to stabilize
    for (int i = 0; i < 10; i++) {
        bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
        sleep_ms(100);
    }
    // Take 20 readings and average them to get a more accurate sea level pressure for altitude calculations
    float seaLevelPressure = 0.0;
    int samples = 20;
    
    for (int i = 0; i < samples; i++) {
        bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
        seaLevelPressure += data.pressure;
        sleep_ms(100);
    }

    // Save this variable! It is your new "Sea Level"
    double launch_pad_pa = seaLevelPressure / samples; 
    
    printf("Baseline Locked: %.2f hPa\n", launch_pad_pa / 100.0);
    printf("--- CALIBRATION COMPLETE ---\n\n");
    sleep_ms(1000);

    // 3. The Infinite Flight Loop
    while (true) {
        rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
        
        if (rslt == BMP3_OK) {
            // Pass your custom launch_pad_pa instead of 101325.0!
            float relative_alt = calculateAltitude(data.pressure, launch_pad_pa);
            
            printf("Temp: %.2f *C | Press: %.2f hPa | Relative Alt: %.2f m\n", 
                   data.temperature, 
                   data.pressure / 100.0, 
                   relative_alt);
        } else {
            printf("Failed to read sensor data.\n");
        }
        
        sleep_ms(100); // Faster read rate for flight!
    }

    return 0;
}