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
const uint8_t loraTxPower = 20;      // dBm
const float loraFrequency = 433.0;   // MHz
const unsigned long loraBandwidth = 125;      // kHz
const uint8_t loraSpreadingFactor = 7;
RH_RF95 lora(LORA_CS, LORA_INT);

// logging
File logFile;
const char *logFilePath = "/log.csv";
const unsigned long logInterval = 100;
const unsigned long radioLogInterval = 1000;
unsigned long lastRadioLogTime = 0;

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
    if (!debugMode && digitalRead(DEBUG) == HIGH) {
	digitalWrite(STATUS_LED, HIGH);
	logFile.close();

	delay(1000);

	Serial.begin(9600);
	while (!Serial);

	logFile = LittleFS.open(logFilePath, "r");
	if (!logFile) {
	    Serial.println("ERROR :: Failed to open log file");
	    while (1);
	}

	while (logFile.available()) {
	    Serial.write(logFile.read());
	}

	logFile.close();

	while (1);
    }

    getAccel();
    getPressure();
    //getMagnet();

    unsigned long time = millis();
    String msg = String(time)+";"+String(temp)+";"+String(pressure)+";"+String(accelX)+";"+String(accelY)+";"+String(accelZ)+";"+String(magnetX)+";"+String(magnetY)+";"+String(magnetZ)+"\n";

    log(msg);

    delay(logInterval);
}

void log(String msg) {
    logFile.print(msg);
    logFile.flush();

    if (millis() - lastRadioLogTime >= radioLogInterval) {
	lastRadioLogTime += radioLogInterval;

	if (lora.mode() != RH_RF95::RHModeTx) {
	    char packet[RH_RF95_MAX_MESSAGE_LEN];
	    msg.toCharArray(packet, sizeof(packet));
	    lora.send((uint8_t *)packet, msg.length());
	} else if (debugMode) {
	    Serial.println("WARN :: LoRa package skipped");
	}
    }
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

void initFS() {
    if (!LittleFS.begin()) {
	error("Failed to mount LittleFS");
    }

    if (debugMode)
	if (!LittleFS.remove(logFilePath))
	    Serial.println("INFO :: No previous log file found to delete");

    logFile = LittleFS.open(logFilePath, "a");
    if (!logFile) error("Failed to open log file");
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
