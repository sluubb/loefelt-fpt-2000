#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>

#define DEBUG 26
#define STATUS_LED 13

bool debugMode = false;

// LoRa
#define LORA_CS 1
#define LORA_INT 24
#define LORA_RST 25
const float loraFrequency = 433.0;   // MHz
const int loraBandwidth = 125;       // kHz
const int loraTxPower = 20;          // dBm
RH_RF95 lora(LORA_CS, LORA_INT);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 

// Barometer
Adafruit_BMP280 bmp;
const float pressureBaseline = 1013.25; // hPa
float temp, pressure, alt;

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

    initLoRa();
    initAccel();
    initPressure();
//    initMagnet();
}

void loop() {
    getAccel();
    getPressure();
//    getMagnet();

    String msg = String(temp)+";"+String(pressure)+";"+String(accelX)+";"+String(accelY)+";"+String(accelZ);
    Serial.println(msg);
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
    lora.waitPacketSent();

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
    pressure = bmp.readPressure() / 100.0;
    alt = bmp.readAltitude(pressureBaseline);
}

void getMagnet() {
    sensors_event_t event;
    lis3mdl.getEvent(&event);

    magnetX = event.magnetic.x;
    magnetY = event.magnetic.y;
    magnetZ = event.magnetic.z;
}
