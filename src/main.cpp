#include <Arduino.h>
#include <DHT.h>

#define DELTA 10.0
#define DELAY_TIME 500
#define BUTTON_DELAY_TIME 100
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
volatile bool powerDown;
unsigned long humCheck;
unsigned long boundaryCheck;
volatile unsigned long buttonCheck;

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
    if (!relayOpen && !powerDown) {
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

void toggle() {
    if(millis() - buttonCheck > BUTTON_DELAY_TIME) {
        buttonCheck = millis();
        powerDown = !powerDown;
        if(powerDown) {
            cooling = false;
            delayTime = 10000;
            closeRelay();

            digitalWrite(RED_LED_PIN, HIGH);
            digitalWrite(GREEN_LED_PIN, LOW);
        } else {
            digitalWrite(RED_LED_PIN, LOW);
            digitalWrite(GREEN_LED_PIN, HIGH);
        }
    }
}

void setup() {
    Serial.begin(9600);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BOUNDARY_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    
    relayOpen = true;
    digitalWrite(GREEN_LED_PIN, HIGH);
    closeRelay();

    attachInterrupt(BUTTON_PIN, toggle, CHANGE);

    sensor.setup(DHT_PIN);

    humidity = sensor.getHumidity();
    temperature = sensor.getTemperature();

    delay(sensor.getMinimumSamplingPeriod());
}

void loop() {
    if (!powerDown) {
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
    }
}