#include <Arduino.h>

struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
};

UltrasonicSensor sensorFront = {19, 18};
const float SPEED_OF_SOUND = 0.0343;
void setup() {
  // put your setup code here, to run once:

}

void loop() {
    static unsigned long lastSensorRead = 0;
    // Measure Distances Every 100ms
  if (millis() - lastSensorRead >= 100) {
    float distanceFront = measureDistance(sensorFront);
    //float distanceLeft  = measureDistance(sensorLeft);
    //float distanceRight = measureDistance(sensorRight);
    //float distanceBack = measureDistance(sensorBack);

    // Print Sensor Data
    Serial.printf("Front: %.1f cm | Left: %.1f cm | Right: %.1f cm | Back: %.1f cm\n", 
                  distanceFront/*, distanceLeft, distanceRight, distanceBack*/);

    // Update Turning Flags
    //updateFlags(distanceFront, distanceLeft, distanceRight);

    //readCommandFromPi();
    //updateState();
    lastSensorRead = millis();
  }


}
float measureDistance(UltrasonicSensor sensor) {
    pinMode(sensor.trigPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
  
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(5);
  
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigPin, LOW);
  
    long duration = pulseIn(sensor.echoPin, HIGH, 30000);
    return duration == 0 ? 400 : (duration * SPEED_OF_SOUND) / 2.0;
}