#include <LittleFS.h>

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (!LittleFS.begin()) {
	Serial.println("LittleFS failed");
	while (1);
    }

    File file = LittleFS.open("/log.csv", "r");
    if (!file) {
	Serial.println("Failed to open log file");
	while (1);
    }

    while (file.available()) {
	Serial.write(file.read());
    }

    file.close();
}

void loop() {}
