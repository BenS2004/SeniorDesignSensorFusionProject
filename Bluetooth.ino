#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("SensorFusionRobot"); // Bluetooth device name
}

void loop() {
  SerialBT.println("Does this work?");

  if (SerialBT.available()) {
    Serial.write(SerialBT.read()); // Send data from Bluetooth to Serial Monitor
  }
}
