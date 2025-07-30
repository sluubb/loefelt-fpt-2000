#include <SPI.h>
#include <RH_RF95.h>

#define SERIAL_DEBUG

#define STATUS_LED 13

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
    pinMode(STATUS_LED, OUTPUT);

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);

#ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while (!Serial);
#endif

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
    lora.waitPacketSent();

    int len = msg.length();
    msg.toCharArray((char *)buf, sizeof(buf));

    lora.send(buf, len);
}

void error(String msg) {
#ifdef SERIAL_DEBUG
    Serial.print("ERROR :: ");
    Serial.println(msg);
#endif

    while (1) {
	digitalWrite(STATUS_LED, HIGH);
	delay(100);
	digitalWrite(STATUS_LED, LOW);
	delay(900);
    }
}
