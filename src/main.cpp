#include <Arduino.h>
#include <DHT.h>

#define DELTA 10.0
#define DELAY_TIME 10000
#define DHT_PIN 2
#define RELAY_PIN 3
#define BOUNDARY_PIN A0

DHT sensor;
float humidity;
float fixedHumidity;
float temperature;
uint8_t maxBound;
int delayTime = 10000;
unsigned long humCheck;
unsigned long boundaryCheck;

bool cooling = false;
bool relayOpen = false;

void log(float hum, float temp, bool cooling) {
    Serial.print("$");
    Serial.print(humidity, 1);
    Serial.print(" ");
    Serial.print(temperature, 1);
    Serial.println(";");
}

void openRelay() {
    if (!relayOpen) {
        relayOpen = true;
        digitalWrite(RELAY_PIN, HIGH);
    }
}

void closeRelay() {
    if (relayOpen) {
        relayOpen = false;
        digitalWrite(RELAY_PIN, LOW);
    }
}

bool setBoundary() {
    int potentiometrValue = map(analogRead(BOUNDARY_PIN), 0, 1023, 50, 100);

    if (potentiometrValue != maxBound) {
        maxBound = potentiometrValue;

        return true;
    }
    return false;
}

void setup() {
    Serial.begin(9600);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BOUNDARY_PIN, INPUT);
    sensor.setup(DHT_PIN);
}

void loop() {
    if (millis() - boundaryCheck > DELAY_TIME) {
        boundaryCheck = millis();
        if(setBoundary()) {
            cooling = false;

            if (humidity < maxBound) {
                closeRelay();
                delayTime = 10000;
            }
        };
    }

    if (millis() - humCheck > delayTime) {
        humCheck = millis();
        humidity = sensor.getHumidity();
        temperature = sensor.getTemperature();

        if (!cooling && humidity > maxBound) {
            cooling = true;
            delayTime = 1000;
            fixedHumidity = humidity;
        }

        log(humidity, temperature, cooling);

        if (cooling) {
            if (abs(humidity - fixedHumidity) < DELTA) {
                openRelay();
            } else {
                closeRelay();
                cooling = false;
                delayTime = 10000;
            }
        }
    }
}