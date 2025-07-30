#include <SPI.h>
#include <RH_RF95.h>

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

void setup() {
    pinMode(DEBUG, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

    if (digitalRead(DEBUG) == HIGH) {
	debugMode = true;

	Serial.begin(9600);
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
}

void loop() {
    log("Aladab");
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
