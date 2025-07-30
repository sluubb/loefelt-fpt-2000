#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>

#define DEBUG 26
#define STATUS_LED 13

bool debugMode = false;

// LoRa module
#define LORA_CS 1
#define LORA_INT 24
#define LORA_RST 25
const float loraFrequency = 433.0; // MHz
const int loraBandwidth = 125;     // kHz
const int loraTxPower = 20;        // dBm
RH_RF95 lora(LORA_CS, LORA_INT);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 

//BMP280
Adafruit_BMP280 bmp;
#define Sp_HPa 1013.25

//ADXL345
#define ADXL345_ADDR 0x53
int16_t X_cords, Y_cords, Z_cords;

//LIS3MDL
Adafruit_LIS3MDL lis3mdl;

void setup() {
    pinMode(DEBUG, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

    if (digitalRead(DEBUG) == HIGH) {
	debugMode = true;

	Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2D);
    Wire.write(8);
    Wire.endTransmission();

	while (!Serial);

	Serial.println("DEBUG MODE ENABLED");
    }

    delay(1000);

    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    if (!lora.init()) {
	error("Failed to init LoRa");
    }

    lora.setFrequency(loraFrequency);
    lora.setSignalBandwidth(loraBandwidth);
    lora.setTxPower(loraTxPower, false);

    if (!bmp.begin(0x76)) { 
    Serial.println("BMP280 hittades inte");
    while (1);
  }

    if (!lis3mdl.begin_I2C()) {
      Serial.println("LIS3MDL hittas inte");
      while (1);
}
    Serial.println("LIS3MDL initierad");
    lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWER);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
}

void loop() {
    log("Aladab");
    Adxl_345();
    BMP_280();
    LIS3MDL();
    delay(1000);
}

void log(String msg) {
    int len = msg.length();
    msg.toCharArray((char *)buf, sizeof(buf));

    lora.send(buf, len);

    digitalWrite(STATUS_LED, HIGH);
    delay(10);
    digitalWrite(STATUS_LED, LOW);
}

void error(String msg) {
    if (debugMode) {
	Serial.print("ERROR :: ");
	Serial.println(msg);
    }

    while (1) {
	digitalWrite(STATUS_LED, HIGH);
	delay(500);
	digitalWrite(STATUS_LED, LOW);
	delay(500);
    }
}

void Adxl_345() {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, 6, true);

  X_cords = (Wire.read() | Wire.read() << 8);
  Y_cords = (Wire.read() | Wire.read() << 8);
  Z_cords = (Wire.read() | Wire.read() << 8);


  Serial.print("X: "); Serial.print(X_cords);
  Serial.print("  Y: "); Serial.print(Y_cords);
  Serial.print("  Z: "); Serial.println(Z_cords);
}

void BMP_280(){
  float te = bmp.readTemperature(); 
  float pr = bmp.readPressure() / 100.0F;
  float al = bmp.readAltitude(Sp_HPa);

  Serial.print("Temp: ");
  Serial.print(te);
  Serial.print(" °C   Tryck: ");
  Serial.print(pr);
  Serial.print(" hPa   Höjd: ");
  Serial.print(al);
  Serial.println(" m");
}


void LIS3MDL() {
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  Serial.print("Magnet X: "); Serial.print(event.magnetic.x);
  Serial.print("  Y: "); Serial.print(event.magnetic.y);
  Serial.print("  Z: "); Serial.println(event.magnetic.z);

}