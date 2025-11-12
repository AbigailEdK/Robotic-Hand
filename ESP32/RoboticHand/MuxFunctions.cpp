#include "Header.h"
#include "PinDefinitions.h"
#include "IMUAddresses.h"
#include "MuxAddresses.h"
#include "AHRSAddresses.h"

// Sensor initialiser through mux
void muxStartup(){
  for (uint8_t ch = 0; ch < 8; ch++){
    selectMuxChannel(ch); // Select mux channel to communicate with
    
    switch(ch){
      case P1:
        AHRSStart();
      break;
      
      case F1:
      case F2:
      case F3:
      case F4:
      case F5:
        IMUStart(LSM_ADR);
      break;
      
      default:
      break;
    }
  }
}

// Select a channel on the mux (0–7)
void selectMuxChannel(uint8_t channel) {
  if(channel > 7) return; // Only 0-7 are valid
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  uint8_t error = Wire.endTransmission();
  delay(2); // small delay to let mux settle

  // Serial.print("Mux channel "); Serial.print(channel); Serial.print(": ");
}

// Sensor finder through mux 
void muxSearcher(){
  for (uint8_t ch = 0; ch < 8; ch++){
    selectMuxChannel(ch); // Select mux channel to communicate with

    Wire.beginTransmission(LSM_ADR); // Talk to 0x6B
    Wire.write(WHO_AM_I);           // Point to this register in device
    Wire.endTransmission(false);    // End transmission, but do not release bus
    Wire.requestFrom(LSM_ADR, 1);   // Request 1 byte of data from IMU

    if (Wire.available()) {         // If data received, display it
      uint8_t whoami = Wire.read();
      Serial.print("Channel "); Serial.print(ch);
      Serial.print(" WHO_AM_I: 0x"); Serial.println(whoami, HEX);
    } 
  }
}

void MUXreadSensors(){
  selectMuxChannel(P1); // Select P1 channel to communicate with
  // delay(10);            // Delay for switching to happen 
  AHRSPoll(AHRS_ADR);   // Poll palm sensor
  // Serial.print(",");
  MUXreadIMUparser();   // Poll all IMUs
  BluetoothTransmit(); // transmit new dataBuffer
}

void MUXreadIMUparser(){
  // Read F1
  selectMuxChannel(F1);
  transmitData(LSM_ADR, 0);
  // Serial.print(",");

  // Read F2
  selectMuxChannel(F2);
  transmitData(LSM_ADR, 1);
  // Serial.print(",");

  // Read F3
  selectMuxChannel(F3);
  transmitData(LSM_ADR, 2);
  // Serial.print(",");

  // Read F4
  selectMuxChannel(F4);
  transmitData(LSM_ADR, 3);
  // Serial.print(",");

  // Read F5
  selectMuxChannel(F5);
  transmitData(LSM_ADR, 4);
  // Serial.print("*");
}

