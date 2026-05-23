# KAOS-2 

1. Hydrogen Vs. Helium 
2. Housing Vent for Overheating
3. Trajectory for a July Launch. 
4. Insta 360 
5. APRS
6. Cameras (corrupted images). Ardu Mega
7. Backup GPS Airtag(student Discount). 
8. Antenna's (Hit up travis) 
9. Hit up Diab to see if we can use GND station. 
10. Spongy Black Foam. 

## Modules 
1. SCD Temp & Humdity Sensor -$60
2. BMP 
3. Gyroscope The MPU-6050 (often labeled GY-521) is a 6-axis motion tracking device combining a 3-axis gyroscope and a 3-axis accelerometer on a single chip. -$12.95
4. SD Card 
5. Buzzer(Speaker) -$1.95
6. Clock (Adafruit).
7. APRs 

The first thing to solve and extensively test is GOS acquisition and aprs transmission.
Secondly, I’d like to explore why the Arduino-Cams keep producing a grey box on the pictures.
I think we could try to motive the Ground Station team to use the ground station during the flight of KAOS-2.
I’d also like to fly the Astronomy Society on KAOS-2 as a tandem payload since that is something we promised them.

Reoganizing the inside of the 4U box to fit other payloads from other clubs/people
Expanding the size of KAOS to a 6U to accommodate other clubs launching with us without needing to switch to a 2 payload balloon set up
Look into using Dove's old gopros for additional views
Create a new bus for KAOS for our customers to hook onto where we supply power, telemetry, and comms.

## Data Collection
* Altitude
* CO2
* Pictures
* Video
* Temp

## PCBs
OSHPARK PCBs (USA) or PCB Way. $100
JCLPCB (This for fatty pcb 7x7). 

MCU no pins

Comms Board, Main Board.

Sensor Board Recycling
1. BMP, Gyro, & SCD.

## Power 
Add Solar Panels
Wind Fan 

## Housing
Cheap wood $10 

## MCU 
2 picos
1 Zero $28

## Camera's
Fish eye $29

## Antenna's
PVC 
Copper wire
Antenna was chopped due to PCB material. Test to see if using plastic or wood will solve.

## Weight
### KAOS-1 
Kaymont Balooon: 2000g
Payload Weight: 884g
Parachute Weight: 196g
O-right weight: less than 1g
Carabiner Weight: 12g
Metal Eye Bolt Weight: 102g
Total Weight: 1218g
Estimate Tolerance: 200g
Total Weight for calculations: 1418g

### KAOS-2 
Max Payload Weight: 6 pounds (2721g)


## Parachute
Reusing T-SAT A parachute

## Flight Estimate
3 hr ascend. 
30 minute descend. 

## Trajectory/Recovery
https://predict.sondehub.org/

https://github.com/csete/gpredict/releases/tag/v2.5.1

## Module Ideas/Links:

https://www.adafruit.com/product/3886

https://www.adafruit.com/product/3295 - Clck 

https://github.com/lightaprs/LightAPRS-1.0

https://makerworld.com/en/models/34707-multicolor-kerbal-from-kerbal-space-program-for-ba?from=search#profileId-129653

https://www.arducam.com/multi-camera-v2-1-adapter-raspberry-pi.html

https://www.arducam.com/mega-5mp-color-rolling-shutter-camera-module-with-m12-lens-for-any-microcontroller.html -Cams KAOS-1 Used.

https://www.adafruit.com/product/4867 

https://www.adafruit.com/product/3886

https://www.adafruit.com/product/1314?srsltid=AfmBOoqa2Y0eFxMBJhQJ2VhBf37QU8frjgeTe-pMDWSDV-0_Tz_C6Noe1dw

### MCU's

https://www.amazon.com/EC-Buying-Pi-Zero-Development/dp/B0FN4CN7TM/ref=sr_1_8?crid=RJYO2K17153U&dib=eyJ2IjoiMSJ9.e5njIqODvl6b9IKwVms-2052_VKm7f8HL6kTDEUeyjBf7_YxKQ8hajdGwvwMgjmoQPK5c6-g1trus2oke-6Z3zKntStBJFPEO0EzZGzMOpWLx-FsQy37CzgaQacQSjqUmOBLnsynvZjGtfiEhHCZioWvf3RslCAMppYBaox513KEm3U1Pqze8GlQJyJfZD2zZ2vmJaLE6Nu22hpfaY18W-__-YK45amULSCbR93REME.IQmlkUSf6EHdcJ4jEPzYcAGavXIu_KpR0BdQQQbFe4U&dib_tag=se&keywords=Raspberry+Pi+Zero+2+W&qid=1778803179&sprefix=raspberry+pi+zero+2+w%2Caps%2C365&sr=8-8

### Battery 

https://www.adafruit.com/product/1781 - Battery 

https://www.amazon.com/PKCELL-CR1220-Battery-Lithium-Count/dp/B0D8T4C5PK/ref=sr_1_1_sspa?crid=28EXCR2BK2VGR&dib=eyJ2IjoiMSJ9.FLJIQjx9Dx8y5aQQgHqGaUFY0QkYy109Q1dFfp1hFL9I391HHaPnWKxhBdYaqt4l3B6ucj9OCtWWXpVAmvWY0tvinOsbykjKb13CNnlnrUxBTGRlBEmsaKb56YADpUI5DC-8RJbs4QM6uy6j1GOeLc9AxVIXI3EG_GqDAX7p7k3TizAdZTegwOwMxkllHjePul6dgvxYim0tevATmnCYanbWBfvB3ooAmHdOZCTlwhiLHnfrca_dcxOz6ZUPRibM5U2r7OKhcJSjI9eqJT8bY9CE5-XKSzsE7LGkh0ERsL8.68JPXEiPomfiY5k25ScScNfUeLMfdHOi4FF8PYhVzMs&dib_tag=se&keywords=CR1220&qid=1779567721&sprefix=cr1220%2Caps%2C143&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1 - Battery for Clk

### Buzzer/Speaker

https://www.adafruit.com/product/1313 -8 ohm speaker

### Radio 

https://www.verotelecom.com/Products/VGC-VR-N76-Dual-Band-Ham-Radio-KISS-TNC-Bluetooth-APRS-Satellite-Tracking-p2511333.html

https://www.amazon.com/RTL-SDR-Blog-RTL2832U-Software-Defined/dp/B0BMKB3L47?th=1

### Possible Camera's 
https://www.arducam.com/arducam-ultra-wide-angle-fisheye-5mp-ov5647-camera-for-raspberry-pi.html -Fish eye Cam

https://www.arducam.com/arducam-raspberry-pi-camera-v2-8mp-ixm219-b0103.html -8MP 


https://www.seeedstudio.com/IMX219-160-Camera-160-FOV-Applicable-for-Jetson-Nano-p-4603.html?gad_source=1&gad_campaignid=12740460396&gclid=CjwKCAjw5ZXQBhBdEiwAI5XVWT9q29YHWw0daC5d7V-952_xm-jBUq4S1vEw0P4yQzej9PaIomw-WBoCXRIQAvD_BwE

### Camera Attachments 

https://www.pishop.us/product/camera-cable-joinerextender-for-raspberry-pi/ 

https://www.arducam.com/multi-camera-v2-1-adapter-raspberry-pi.html

### Solar 

https://www.adafruit.com/product/5855?gad_source=1&gad_campaignid=21079227318&gclid=CjwKCAjw2rrQBhBuEiwAarLWHTQTQLF9XBSzwCDK9d-VbhUqb3OOivSrNKz0ZjxTm2qZvzlb-eDjMBoCr6sQAvD_BwE

### No longer Need
https://www.newark.com/raspberry-pi/rpi-hq-camera/high-quality-camera-12-3mp/dp/67AH5587?CMP=KNC-GUSA-STANDARD-SHOPPING-RPI-OEM&gad_source=1&gad_campaignid=22967479914&gclid=CjwKCAjw5ZXQBhBdEiwAI5XVWZQUM6AR_DOo7mSqUEMH4859y6xts9W0Hz8xK0rxeWI4RQNH82QOdhoCQyQQAvD_BwE

# Recovery 

if it lands in a hiking trail we can rent bikes. 

Bring Change of Clothes, Shoes, Water, Snacks, Sun Protection. Wear Shorts.

shoe covers for recovery https://www.amazon.com/Covers-Reusable-Waterproof-Cover-Non-Slip/dp/B0CQS49P6K/ref=sr_1_31?crid=2VRFRZJH75KB1&dib=eyJ2IjoiMSJ9.yIWybTD5ituqeCL6H3gq8ANSorAQs3GZD8hsBRiBxZ-zoKIDBR25od0hTTcEcycShp40imCrhlSi_AB-ErU8di4LhO7lSroZmq4XOkow2ISv57m5Cw-blxTXnAnrYAZ9kGfSylK92M-mhlsK4Ucyby9gH_s3X5QJsEky3PMVBbVmQmKo9YrFJImJ3ys7bUqg50rST6zAt5beCAiOkNVlK0_SabqWYksGYDZmhVuHLjVssa9T0gzdyJdzTGcOYbbBP6Mew6YcgADxOXmLHpudZb0Cr8heS-Uq9ub4skfIUUA.N7MRogix1sR5NrJ-7FswC5IwEPUmiHJ2jrTwwHLSsKo&dib_tag=se&keywords=Waterproof%2BShoe%2BCovers&qid=1779566752&sprefix=waterproof%2Bshoe%2Bcovers%2Caps%2C228&sr=8-31&th=1 

