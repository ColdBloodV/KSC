# KAOS-2 Documentation 


# Mechanical

## OpenSCAD

# Electrical

## Regulator

## Power 


# Software
KAOS-2 is using one raspberry pi pico and two raspberry pi 2 zero W. 

## MCUS
### Pico 
Set up MCU: https://github.com/raspberrypi/pico-sdk
https://github.com/raspberrypi/pico-examples

## Video Capture

## Pictures

# Communications


## Antenna 
Due to the failure of communications on KAOS-1 the antennas might have to redisgned and implemented. 

## Light APRS 
It is able to report location, altitude, temperature and pressure via internet/Amateur Radio

Communicates via: I2C, SPI

Further info: https://github.com/lightaprs/LightAPRS-1.0

V-Dipole Antenna: https://apbouwens.github.io/V-dipole-rad-pat/

## How the APRS Protocol Works 

## GND
A simple magnectic roof antenna on the car to improve recovery performance for receiving on GND. 

Using RTL-SDR connected to software DireWolf(virtual TNC) to receive and test APRS packets.

## Airtag 
To be used as a close range backup recovery tool. Works over Bluetooth. Ranges from 100feet (30 meters) outdoors

Other Option: Tile trackers which range from 100-500ft. 
