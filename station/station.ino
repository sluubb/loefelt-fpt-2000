#include <SPI.h>
#include <RH_RF95.h>

// LoRa module
#define LORA_CS 1
#define LORA_INT 24
#define LORA_RST 25
const float loraFrequency = 900.0; // MHz
const int loraBandwidth = 125;     // kHz
const int loraTxPower = 13;        // dBm
RH_RF95 lora(LORA_CS, LORA_INT);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 

void setup() {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

    Serial.begin(9600);
    while (!Serial);

    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    if (!lora.init()) {
	Serial.println("Failed to init LoRa");
	while (1);
    }

    lora.setFrequency(loraFrequency);
    lora.setSignalBandwidth(loraBandwidth);
    lora.setTxPower(loraTxPower, false);
}

void loop() {
    if (lora.available()) {
	uint8_t len = sizeof(buf);
	if (lora.recv(buf, &len)) {
	    RH_RF95::printBuffer("Received: ", buf, len);
	    Serial.print("Got: ");
	    Serial.println((char*)buf);
	    Serial.print("RSSI: ");
	    Serial.println(lora.lastRssi(), DEC);
	} else {
	    Serial.println("Receive failed");
	}
    }
}

