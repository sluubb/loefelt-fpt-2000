#include <SPI.h>
#include <RH_RF95.h>

// LoRa module
#define LORA_CS 1
#define LORA_INT 24
#define LORA_RST 25
const float loraFrequency = 433.0; // MHz
const int loraBandwidth = 125;     // kHz
RH_RF95 lora(LORA_CS, LORA_INT);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 

void setup() {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

    Serial.begin(9600);
    while (!Serial);
    delay(1000);

    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    while (!lora.init()) {
	Serial.println("Failed to init LoRa");
	delay(1000);
    }

    lora.setFrequency(loraFrequency);
    lora.setSignalBandwidth(loraBandwidth);
}

void loop() {
    lora.waitAvailable();
    uint8_t len = sizeof(buf);
    if (lora.recv(buf, &len)) {
	Serial.print((char *)buf);
    } else {
	Serial.println("Receive failed");
    }
}

