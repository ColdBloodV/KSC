//T-SAT Air 

//libraries
#include <Arducam_Mega.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
//RF and Servo
#include <RH_RF95.h>
#include <ESP32Servo.h>

//BMP
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

//Alt sensor 
#define SEALEVELPRESSURE_HPA (1017.25) //constant given for daily pressure
Adafruit_BMP3XX bmp;

//Camera SPI pins (VSPI)
#define CAM2_CS 27
#define CAM1_CS 25 
#define SCK 18
#define MISO 19
#define MOSI 23

//SD Card SPI pins (HSPI)
#define SD_CS 15
#define SCK_SD 14
#define MISO_SD 12
#define MOSI_SD 13

//RF and Servo 
#define RF95_CS   5
#define RF95_INT  2 //GO
#define RF95_RST  4

#define SERVO_PIN   32 
#define SERVO_INITIAL_POS 0
#define SERVO_DEPLOY_POS 90

#define SPI_CLOCK_FREQ 8000000
#define PIC_BUFFER_SIZE 4096 
#define SERIAL_BAUD 74880

//Globals

//for RF
RH_RF95 rf95(RF95_CS, RF95_INT);

bool loraSendAck = false;
bool detachCommandReceived = false; 

//Create separate SPI instances (HSPI SD)
SPIClass spiSD(HSPI);  

//Declaring Cams
Arducam_Mega myCAM1(CAM1_CS);
Arducam_Mega myCAM2(CAM2_CS);

//for picture capture
uint8_t image_buf[PIC_BUFFER_SIZE];
int pic_num = 0;
int cam_num; 

//for SPI 
bool spiBusy = false; 

//bmp
float altitude;
unsigned long lastAltSend = 0; 
#define ALT_SEND_TIME 25000  // send every 25 seconds

//Servo 
Servo myservo; 
bool servoActive = false; 

//Initializing functions 
void write_pic(Arducam_Mega &cam, File &dest);
float calculateAltitude(float atmospheric);
void triggerDetach();
void checkLora();
void sendLoraAck();
void checkBMP();
void TakePictures();
void SavePictures(Arducam_Mega &cam, int cam_num, int pic_num);


void setup(){
  delay(1000);
  
  // 1. Init SPI Buses 
  
  // Initialize VSPI for camera and RF (default SPI)
  SPI.begin(SCK, MISO, MOSI, -1);
  pinMode(CAM1_CS, OUTPUT);
  digitalWrite(CAM1_CS, HIGH);
  pinMode(CAM2_CS, OUTPUT);
  digitalWrite(CAM2_CS, HIGH);
  pinMode(RF95_CS, OUTPUT);
  digitalWrite(RF95_CS, HIGH);
  delay(100);

  // Initialize 2nd SPI bus (HSPI) for SD card
  spiSD.begin(SCK_SD, MISO_SD, MOSI_SD, SD_CS);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(100);

  //2. Initalize RF module 

  // Reset radio
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, LOW);
  delay(10);
  digitalWrite(RF95_RST, HIGH);
  delay(10);

  // Initialize radio
  if (!rf95.init()) {
    Serial.println("RF INIT FAIL - continuing without radio");
    // continue mission without comms
  } else {
    Serial.println("RF module initialized");
    
    if (!rf95.setFrequency(915.0)) {
      Serial.println("RF frequency set FAIL");
    } else {
      Serial.println("RF frequency: 915 MHz");
    }
    
    rf95.setTxPower(13, false);
    Serial.println("RF module ready");
  }

  //3. Initialzie Cameras

  // Cam 1 init (VSPI)
  SPI.setFrequency(SPI_CLOCK_FREQ);
  uint8_t r1= myCAM1.begin();
  if (r1 != CAM_ERR_SUCCESS) {
    while (1) delay(1000);
  }

  //Cam 2 Init
  uint8_t r2 = myCAM2.begin();
  if (r2 != CAM_ERR_SUCCESS) {
    while (1) delay(1000);
  }

  // 4. Init SD card on HSPI
  if (!SD.begin(SD_CS, spiSD)) {
    while (1) delay(1000);
  }
 
  // Re-configure camera SPI after SD init
  SPI.setFrequency(SPI_CLOCK_FREQ);
  SD.mkdir("/images");

  //5. Init I2C LAST
  delay(500); // Stabilizing delay
  // Initialize BMP sensor
  bool bmp_ok = false;
  if (bmp.begin_I2C(0x77)) {
    bmp_ok = true;
  } else if (bmp.begin_I2C(0x76)) {
    bmp_ok = true;
  } else {
    while (1) delay(1000);
  }

  if (bmp_ok) {
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    bmp.performReading(); // Discard first reading
    delay(100);
    bmp.performReading(); // Start using readings after this
  }

}

void loop() {

  //1. Check RF
  checkLora();

  //2. BMP
  checkBMP();

  //3. Cameras
  spiBusy = true;

  TakePictures();

  SavePictures(myCAM1, 1, pic_num);
  delay(100);
  SavePictures(myCAM2, 2, pic_num);

  spiBusy = false;
  pic_num++;

  //Handles Transmission
  if(loraSendAck){
    sendLoraAck();
    loraSendAck = false; 
  }
  //Handles servo 
  if(detachCommandReceived){
    triggerDetach();
    detachCommandReceived = false;
    rf95.setModeRx();
  }
  delay(3000);
}

void write_pic(Arducam_Mega &cam, File &dest){
  uint8_t prev_byte = 0;
  uint8_t cur_byte = 0;
  bool head_flag = false;
  int read_len = 0;
  int start_offset = 0;
  int bytes_written = 0;
  //safety count
  int safety_count = 0;         
  const int MAX_LOOPS = 2000;  

  while (cam.getReceivedLength() && safety_count < MAX_LOOPS) {

    read_len = 0;

    // Fill image_buf in chunks (max 254 bytes at a time per ArduCAM limitation)
    while (read_len < PIC_BUFFER_SIZE && cam.getReceivedLength()) {
      uint32_t chunk_len = min(254, PIC_BUFFER_SIZE - read_len);
      uint32_t actual_read = cam.readBuff(image_buf + read_len, chunk_len);
      
      if (actual_read == 0) {
        break;
      }
      read_len += actual_read;
    }

    start_offset = 0;

    // Process buffer to find JPEG markers
    for (int i = 0; i < read_len; i++) {
      prev_byte = cur_byte;
      cur_byte = image_buf[i];

      // Found JPEG start marker (0xFF 0xD8)
      if (prev_byte == 0xff && cur_byte == 0xd8) {
        head_flag = true;
        start_offset = i + 1;
        
        bytes_written += dest.write(0xff);
        bytes_written += dest.write(0xd8);
        dest.flush();
      }

      // Found JPEG end marker (0xFF 0xD9)
      if (head_flag && prev_byte == 0xff && cur_byte == 0xd9) {
        
        // Write remaining data up to and including 0xD9
        int chunk_len = i + 1 - start_offset;
        bytes_written += dest.write(image_buf + start_offset, chunk_len);
        dest.close();
        return;
      }
    }

    // Write the buffer data if we've found the header
    if (head_flag) {
      int chunk_len = read_len - start_offset;
      int retval = dest.write(image_buf + start_offset, chunk_len);
      
      // Handle write failure by reopening file
      if (retval == 0 && chunk_len > 0) {
        char fp[40];
        strcpy(fp, dest.path());
        dest.close();
        delay(10);
        dest = SD.open(fp, FILE_APPEND);
        
        if (dest) {
          retval = dest.write(image_buf + start_offset, chunk_len);
        } else {
          return;
        }
      }
      
      bytes_written += retval;
      dest.flush();
    }
    safety_count++;
  }
  dest.close();
}
//for bmp 
float calculateAltitude(float atmospheric) {
  atmospheric = atmospheric / 100.0;
  return 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));
}

//Trigger servo 
void triggerDetach() {
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_INITIAL_POS);
  delay(3000); //delay 3 seconds
  myservo.write(SERVO_DEPLOY_POS);
  servoActive = false; 
}
//Check Lora
void checkLora(){

  //ignore if we are currently saving a picture
  if(spiBusy){
    return;
  }

  if (rf95.available()){ 

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if(rf95.recv(buf, &len)){

    if(len < RH_RF95_MAX_MESSAGE_LEN){

      buf[len] = '\0';
    }

    if (strcmp((char*)buf, "Detach") == 0) {

      servoActive = true; 
      loraSendAck = true; 
      detachCommandReceived = true; 
      }
    }
  }
  if(!servoActive){

    if(millis() - lastAltSend > ALT_SEND_TIME){
    //send Altitude to GND
    char alt[20];
    snprintf(alt, sizeof(alt), "Alt: %.2f m", altitude);
    rf95.send((uint8_t*)alt, strlen(alt));

    rf95.waitPacketSent();
    rf95.setModeRx(); 
    lastAltSend = millis();
    }
  }
}

//Send message back to gnd
void sendLoraAck(){

  spiBusy = true; 

  //Wake up radio 
  rf95.available();

  //make sure everything is paused
  digitalWrite(CAM1_CS, HIGH);
  digitalWrite(CAM2_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RF95_CS, LOW);

  const char reply[] = "Detached";
  rf95.send((uint8_t*)reply, strlen(reply));
  rf95.waitPacketSent();
  rf95.setModeRx(); 
  spiBusy = false;
}

//Check BMP 
void checkBMP(){

  if (bmp.performReading()) { 
    float atmospheric = bmp.pressure;
    altitude = calculateAltitude(atmospheric);
    //save it to SD card
    File f = SD.open("/Altitude.txt", FILE_APPEND);
    if(!f){
      return; 
    }
    f.println(String(altitude) + "m");
    f.close(); 

  } 
}

//Cameras
void TakePictures(){

  digitalWrite(RF95_CS, HIGH);
  digitalWrite(CAM1_CS, HIGH);
  digitalWrite(CAM2_CS, HIGH);

  // Trigger both cameras as quickly as possible
  myCAM1.takePicture(CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
  myCAM2.takePicture(CAM_IMAGE_MODE_FHD, CAM_IMAGE_PIX_FMT_JPG);
  
  // Wait for both cameras to finish capturing
  delay(200);
}

void SavePictures(Arducam_Mega &cam, int cam_num, int pic_num){

  char fp[32];
  // Read and save pictures
  if(cam_num == 1){
    sprintf(fp, "/images/cam1_pic%d.jpg", pic_num);
  }
  else{
    sprintf(fp, "/images/cam2_pic%d.jpg", pic_num);
  }
  
  File file1 = SD.open(fp, FILE_WRITE);
  if (!file1) {
    return;
  } 
  write_pic(cam, file1); //debug

  File check = SD.open(fp);
  if (check) {
    check.close();
  }
}
