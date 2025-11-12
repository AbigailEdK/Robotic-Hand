#include "Header.h"
#include "PinDefinitions.h"
#include "IMUAddresses.h"
#include "MuxAddresses.h"
#include "AHRSAddresses.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Server on ESP32-S3...");
  BluetoothSerialStart();

  I2C_init(I2C_SDA, I2C_SCL); // Initialise I2C protocol
  delay(2); // give sensor time to wake up
  Serial.println("I2C initialised.");
  
  startScreen();
}

void loop() {
  if (deviceConnected) {
    if (commandReady) {
        CommandChecker(receivedCommand); // handle command
        commandReady = false;            // reset flag
    }

    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        CommandChecker(cmd);
    }

    if (CALIB1 || CALIB2 || pollSens) {
      MUXreadSensors();
    }

    } else {
      Serial.println("Advertising...");
      delay(1000);

  }
}
