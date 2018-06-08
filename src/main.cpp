#include <Arduino.h>
#include <DHT.h>

DHT sensor;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.println();
    Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)");

    sensor.setup(2);
}

void loop() {
    delay(sensor.getMinimumSamplingPeriod());
    //Read data and store it to variables hum and temp
    float humidity = sensor.getHumidity();
    float temperature = sensor.getTemperature();
    //Print temp and humidity values to serial monitor
    Serial.print(sensor.getStatusString());
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.println();    
}