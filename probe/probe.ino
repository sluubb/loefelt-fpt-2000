#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <LittleFS.h>

#define DEBUG 26
#define STATUS_LED 13

bool debugMode = false;

// LoRa
#define LORA_CS 1
#define LORA_INT 24
#define LORA_RST 25
const uint8_t loraTxPower = 10;      // dBm
const float loraFrequency = 433.0;   // MHz
const long loraBandwidth = 125;      // kHz
const uint8_t loraSpreadingFactor = 7;
RH_RF95 lora(LORA_CS, LORA_INT);

uint8_t currBufPos = 0;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 

// Barometer
Adafruit_BMP280 bmp;
float temp, pressure;

// Accelerometer
#define ADXL345_ADDR 0x53
float accelX, accelY, accelZ;

// Magnetometer
Adafruit_LIS3MDL lis3mdl;
float magnetX, magnetY, magnetZ;

void setup() {
    pinMode(DEBUG, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

    Wire.begin();

    if (digitalRead(DEBUG) == HIGH) {
	debugMode = true;

	Serial.begin(9600);

	while (!Serial);

	Serial.println("DEBUG MODE ENABLED");
    }

    delay(1000);

    initFS();
    initLoRa();
    initAccel();
    initPressure();
//    initMagnet();
}

void loop() {
    getAccel();
    getPressure();
//    getMagnet();

    String msg = String(temp)+";"+String(pressure)+";"+String(accelX)+";"+String(accelY)+";"+String(accelZ)+";"+String(magnetX)+";"+String(magnetY)+";"+String(magnetZ)+"\n";

    digitalWrite(STATUS_LED, HIGH);

    log(msg);

    if (lora.mode() != RHModeTX)
	digitalWrite(STATUS_LED, LOW);
}

void initFS() {
    if (!LittleFS.begin()) {
	error("Failed to mount LittleFS");
    }
}

void initLoRa() {
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    if (!lora.init()) {
	error("Failed to init LoRa");
    }

    lora.setFrequency(loraFrequency);
    lora.setSignalBandwidth(loraBandwidth);
    lora.setSpreadingFactor(loraSpreadingFactor);
    lora.setTxPower(loraTxPower, false);
}

void initAccel() {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2D);
    Wire.write(0x08);
    Wire.endTransmission();
}

void initPressure() {
    if (!bmp.begin(0x76)) {
	error("BMP280 not found");
    }
}

void initMagnet() {
    if (!lis3mdl.begin_I2C()) {
	error("LIS3MDL not found");
    }

    lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
}

void log(String msg) {
    File file = LittleFS.open("/data.txt", "w");
    if (file) {
	file.print(msg);
	file.close();
    } else error("Couldn't open log file");

    int len = msg.length();

    if (currBufPos + len > RH_RF95_MAX_MESSAGE_LEN - 1) {
	buf[currBufPos] = '\0';
	lora.send(buf, currBufPos);
	currBufPos = 0;
    }

    msg.toCharArray((char *)&buf[currBufPos], sizeof(buf) - currBufPos);
    currBufPos += len;
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

void getAccel() {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x32);
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345_ADDR, 6, true);

    accelX = (int16_t)(Wire.read() | Wire.read() << 8) / 256.0;
    accelY = (int16_t)(Wire.read() | Wire.read() << 8) / 256.0;
    accelZ = (int16_t)(Wire.read() | Wire.read() << 8) / 256.0;
}

void getPressure(){
    temp = bmp.readTemperature(); 
    pressure = bmp.readPressure();
}

void getMagnet() {
    sensors_event_t event;
    lis3mdl.getEvent(&event);

    magnetX = event.magnetic.x;
    magnetY = event.magnetic.y;
    magnetZ = event.magnetic.z;
}
