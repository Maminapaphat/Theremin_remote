#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <Wire.h>

const float alpha = 0.98f; 
const float dt = 0.005f;    // 10ms loop time (200Hz)

float pitch = 0.0f;
float roll = 0.0f;

const byte cfig[] = {0,8,16,24};      // gyro and acellerometer range settings
uint8_t I2C_ADDR = 0x68;              // I2C address of the MPU6050 chip
int channel, hand;
uint8_t broadcastAddress[] = {0xa0, 0xb7, 0x65, 0x4e, 0xf5, 0x4c};
uint8_t thereminState[10];
float *tsi = (float *)thereminState;

byte writeToI2C(uint8_t chipRegister, uint8_t byte){  // write a single byte to the 6050
  Wire.beginTransmission (I2C_ADDR);  // request the bus, send I2C address with write bit
  Wire.write(chipRegister);           // starting register address to write to
  Wire.write(byte);                   // send the data
  return Wire.endTransmission();      // release the bus
}

byte readFromI2C(uint8_t chipRegister, uint8_t count, uint8_t *buf){  // read one or more bytes froman I2C device
  Wire.beginTransmission (I2C_ADDR);  // request the bus, send I2C address with write bit
  Wire.write(chipRegister);           // starting register address to read from
  Wire.endTransmission();             // release the bus

  Wire.requestFrom(I2C_ADDR, count);  // request the I2C bus, starting address with read bit and number of bytes to read
  while(Wire.available())
    while(count--)                    // for each byte expected
      *buf++ = Wire.read();           // put the byte in the buffer
  return Wire.endTransmission();      // release the I2C bus
}

void IRAM_ATTR OnDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  if(status != ESP_NOW_SEND_SUCCESS)  Serial.println("send failed");
}

void setup() {
  setCpuFrequencyMhz(80); // reduce speed to save energy and reduce chip temp- 67.8C @ 160MHz, 55.8 @ 80MHz
  memset(broadcastAddress, 0xff, 6);
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) Serial.println("esp_now_init failed");
  else Serial.println("esp_now_init success");

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_max_tx_power(30);      // 80 is max but may cause spikes

//  if(esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent)))   Serial.println("esp_now_register_send_cb failed");
//  else Serial.println("esp_now_register_send_cb success");

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo,0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA; // Explicitly tell it to use the Station interface  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)    Serial.println("Failed to add peer");
  else Serial.println("esp_now_add_peer success");

  pinMode(5,INPUT_PULLUP);  // channel 1=RIGHT, 0=LEFT
  pinMode(6,INPUT_PULLUP);  // 1=freq control
  pinMode(7,INPUT_PULLUP);  // 1=volume control
  pinMode(0,INPUT);         // accelerometer data ready
  pinMode(1, INPUT_PULLUP); // extra octave if grounded
  Serial.println("Wifi initialize");

// SETUP MPU6050 accelerometer

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.
  byte who_am_i, rc = readFromI2C(0x75, 1, &who_am_i);  // WHO_AM_I register has chip's I2C addr, unless this is a clone
  Serial.println(who_am_i, 16);
  writeToI2C(0x6b, 1);            // use Gyro Z clock
  writeToI2C(0x19, 7);
  writeToI2C(0x1a, 0);
  writeToI2C(0x1b, cfig[0]);      // gyro range +/- 250 degress/s  LSB = 250/32768 deg/s
  writeToI2C(0x1c, cfig[0]);      // accelerometer range +/- 2g    LSB = 2/32768 g
  writeToI2C(0x38, 1);
  Serial.println("Setup complete");
}

void loop() {
  static uint32_t lastRead, counter;
  int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ;

  uint8_t buf[14];
  if(millis() > lastRead){
    lastRead = millis() + 4;              // ms per read
    readFromI2C(0x3b, sizeof(buf), buf);  // read acc, not temp, gyro data

    rawAccX = (buf[0] << 8) | buf[1];
    rawAccY = (buf[2] << 8) | buf[3];
    rawAccZ = (buf[4] << 8) | buf[5];
    rawGyroX = (buf[8] << 8) | buf[9];
    rawGyroY = (buf[10] << 8) | buf[11];
    rawGyroZ = (buf[12] << 8) | buf[13];

    // 1. Convert Accel to Angles (Pitch/Roll)
    // Using atan2 for stability. 16384.0 is the LSB sensitivity for +/- 2g
    float accPitch = atan2f((float)rawAccY, (float)rawAccZ) * 57.295f; 
    float accRoll  = atan2f((float)-rawAccX, sqrtf((float)rawAccY * rawAccY + (float)rawAccZ * rawAccZ)) * 57.295f;

    // 2. Convert Gyro to Degrees/sec
    // 131.0 is LSB sensitivity for 250 deg/s
    float gyroXrate = (float)rawGyroX / 131.0f;
    float gyroYrate = (float)rawGyroY / 131.0f;

    // 3. The Complementary Filter
    // Pitch = 98% * (Previous Pitch + Gyro Delta) + 2% * (Accel Pitch)
    pitch = alpha * (pitch + gyroXrate * dt) + (1.0f - alpha) * accPitch;
    roll  = alpha * (roll + gyroYrate * dt) + (1.0f - alpha) * accRoll;

    // 4. send pitch, roll, and config bits to Theremin base
    tsi[0] = pitch;
    tsi[1] = roll;
    thereminState[8] = digitalRead(5) | digitalRead(6)<<1 | digitalRead(7)<<2 | digitalRead(1) << 3;
    thereminState[9] = 0x7d;
    esp_now_send(broadcastAddress, thereminState, sizeof(thereminState));
//    return;


    if(counter++ % 1000)  return;
    Serial.printf("pitch (freq) %5.1f, roll (volume)  %5.1f, options %1d  ",tsi[0], tsi[1], thereminState[8]);

    float temp_celsius = temperatureRead(); 
    Serial.print("Internal Chip Temp: ");
    Serial.print(temp_celsius);
    Serial.println(" °C");
  }
}
