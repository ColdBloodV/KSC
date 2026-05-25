# T-SAT A Flight Software Documentation

# T-SAT A System Overview 

T-SAT A handles 4 main objectives: 

1. Dual Camera Operation
2. Altidue readings
3. Servo Action
4. LoRa Radio Transmision. 

# Cameras 

This T-SAT handles dual Cameras (CAM1 & CAM2) 

On average we get 5 pictures per minute so the T-SAT takes a picture every 12 seconds that is for both CAM1 and CAM2. 
Pictures are logged and saved onto the SD card. 

** Important **
SD card and Cameras must be on seperate SPI buses. Saving Images to the SD card takes a long time and causes SPI conflicts. 

# Altitude 

Altidue readings are:
  * Transmitted to GND station 
  * Logged and saved to the SD card

The T-SAT can only send altitude readings every 25 seconds, so logging locally ensures that the altitude data is not lost between transmissions. 

# Servo 

The servo triggers around 15-25 seconds after the button on GND station is pressed. 

## Reasons for the Delay 
The main loop may be:
  * Saving images to SD card
  * Reading sensors
  * Handling LoRa communication 

Image saving to the SD card is the main cause of the delay because it takes the longest. 

# LoRa RF 

The LoRa RF module shares an SPI bus with the Cameras. This module is only meant to send or receive data it is not meant to act as a transceiver. 

** Important ** 
Main issue with the LoRa is that the Cameras and the RF module share the same SPI when we receive the 'Detach' message from GND it can disrupt the Cameras SPI bus and cause it to freeze.
Timing is crucial to prevent any conflicts between GND and T-SAT. 

## Timing Solution 
A flag system was implemented in the main loop: 
  * When cameras are using SPI -> LoRa send/receive is disabled
  * When SPI is free -> LoRa communication resumes
    
A flag system was implemented in the checkLora function:
  * When the servo is active stop sending altitude readings
  * Prevents packets from trasmitting while receviing and allows the 'detached' ACK to be sent. 

In order to send and receive data from the T-SAT some timers/delays were implemented to prevent it from freezing and stopping all action. 
A 3 second delay in the main loop is crucial for timing accuracy that allows the RF to trasmit and receive. 

Different delay values were tested: 
  * Shorter delays -> RF communication unreliable
  * Longer delays -> System timing inaccurate
  * 3 seconds provided the most stable performance

A 25 second delay in the checkLora function was added to give both modules on the T-SAT and GND station from colliding packets while sending the altitude readings.

# Timing Summary

| System Task            | Interval / Delay | Notes |
|------------------------|------------------|------|
| Camera Capture         | 12 seconds       | Alternates between CAM1 and CAM2 |
| Pictures Per Minute    | 5 pictures       | Combined total from both cameras |
| Altitude Logging       | Every loop       | Saved to SD card |
| Altitude Transmission  | 25 seconds       | Sent to Ground Station |
| Servo Trigger Delay    | 15–25 seconds    | Delay caused by SD writes and loop timing |
| Main Loop Delay        | 3 seconds        | Required for stable LoRa communication |
| LoRa Altitude Timer    | 25 seconds       | Prevents packet collisions |

# SPI Bus Configuration

| Device      | SPI Bus |
|-------------|---------|
| CAM1        | SPI Bus 1 |
| CAM2        | SPI Bus 1 |
| SD Card     | SPI Bus 2 |
| LoRa RF     | SPI Bus 1 |





