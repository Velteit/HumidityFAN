#include <Arduino.h>
#include <DHT.h>

#define DELTA 10.0
#define DELAY_TIME 500
#define DHT_PIN 2
#define RELAY_PIN 4
#define BOUNDARY_PIN A0
#define RED_LED_PIN A1
#define GREEN_LED_PIN A2
#define BUTTON_PIN 3

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
    Serial.print(" ");
    Serial.print(cooling);
    Serial.println(";");
}

void openRelay() {
    if (!relayOpen) {
        Serial.println("Open relay");
        relayOpen = true;
        digitalWrite(RELAY_PIN, LOW);
        delay(500);
    }
}

void closeRelay() {
    if (relayOpen) {
        Serial.println("Close relay");
        relayOpen = false;
        digitalWrite(RELAY_PIN, HIGH);
        delay(500);
    }
}

bool setBoundary() {
    int potentiometrValue = map(analogRead(BOUNDARY_PIN), 0, 1023, 40, 100);

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
    relayOpen = true;
    closeRelay();

    sensor.setup(DHT_PIN);

    humidity = sensor.getHumidity();
    temperature = sensor.getTemperature();

    delay(sensor.getMinimumSamplingPeriod());
}

void loop() {
    if (millis() - boundaryCheck > DELAY_TIME) {
        boundaryCheck = millis();

        if(setBoundary()) {
            Serial.print("Boundary level: ");
            Serial.println(maxBound);

            cooling = false;

            if (humidity < maxBound) {
                closeRelay();
                delayTime = 10000;
            }
        }
    }

    if (millis() - humCheck > delayTime) {
        humCheck = millis();
        humidity = sensor.getHumidity();
        temperature = sensor.getTemperature();

        if (!cooling && humidity > maxBound) {
            cooling = true;
            delayTime = 500;
            fixedHumidity = humidity;
        }

        log(humidity, temperature, cooling);
        Serial.println(fixedHumidity);

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
    // openRelay();
    // Serial.println("Open");
    // delay(5000);
    // closeRelay();
    // Serial.println("Close");
    // delay(5000);
}