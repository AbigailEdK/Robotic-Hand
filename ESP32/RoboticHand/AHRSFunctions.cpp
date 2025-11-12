#include "Header.h"
#include "PinDefinitions.h"
#include "IMUAddresses.h"
#include "MuxAddresses.h"
#include "AHRSAddresses.h"


// VARIABLES -------------------------------------------
float ahrsData[9];  
float scale_ACCEL = 8.0f / 32768.0f;
float scale_GYRO  = 0.035f;

float accelBiasX = 0;
float accelBiasY = 0;
float accelBiasZ = 0;

float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

float magBiasX = 0;
float magBiasY = 0;
float magBiasZ = 0;

// FUNCTIONS -------------------------------------------
void AHRSAccelStart(uint8_t sensor_addr){
  SensorTransmit(sensor_addr, REG_BANK_SEL, 0x20); // Select bank 2
  SensorTransmit(sensor_addr, ACCEL_CONFIG, 0x04); // +-8g
  delay(5);
}

void AHRSGyroStart(uint8_t sensor_addr){
  SensorTransmit(sensor_addr, REG_BANK_SEL, 0x20); // Select bank 2
  SensorTransmit(sensor_addr, GYRO_CONFIG_1, 0x04); // +- 1000dps
  delay(5);
}

void AHRSMagStart(uint8_t sensor_addr){
  // Reset
  SensorTransmit(sensor_addr, CNTL3, 0x01);
  delay(50); // longer delay

  // Continuous measurement mode
  SensorTransmit(sensor_addr, CNTL2, 0x08);
  delay(10);

  // Read back
  // uint8_t mode = SensorReceive(sensor_addr, CNTL2);
  // Serial.print("Mag mode after init = 0x");
  // Serial.println(mode, HEX);
}

void AHRSControlStart(uint8_t sensor_addr){
  SensorTransmit(sensor_addr, REG_BANK_SEL, 0x00); // Select bank 0
  SensorTransmit(sensor_addr, PWR_MGMT_1, 0x80); // Reset device registers, wake from sleep
  delay(5);
  SensorTransmit(sensor_addr, PWR_MGMT_1, 0x01); // Clock auto select
  SensorTransmit(sensor_addr, USER_CTRL, 0x04); // Disable DMP, disable I2C master, reset SRAM.
  SensorTransmit(sensor_addr, PWR_MGMT_2, 0x00); // Enable all axes
  SensorTransmit(sensor_addr, INT_PIN_CFG, 0x02); // Enable bypass mode
  SensorTransmit(sensor_addr, INT_ENABLE_1, 0x01); // Enable data ready interrupts
  SensorTransmit(sensor_addr, FIFO_EN_2, 0x00); // Disable FIFO writes
  delay(5);
}

void AHRScalibrateGyro(uint8_t sensor_addr, uint16_t samples, uint16_t delay_ms){
  int32_t sumX = 0, sumY = 0, sumZ = 0;

  for(uint16_t i = 0; i < samples; i++){
    // Read raw gyro registers
    uint8_t xh = SensorReceive(sensor_addr, GYRO_XOUT_H);
    uint8_t xl = SensorReceive(sensor_addr, GYRO_XOUT_L);
    uint8_t yh = SensorReceive(sensor_addr, GYRO_YOUT_H);
    uint8_t yl = SensorReceive(sensor_addr, GYRO_YOUT_L);
    uint8_t zh = SensorReceive(sensor_addr, GYRO_ZOUT_H);
    uint8_t zl = SensorReceive(sensor_addr, GYRO_ZOUT_L);

    int16_t gx = (int16_t)((xh << 8) | xl);
    int16_t gy = (int16_t)((yh << 8) | yl);
    int16_t gz = (int16_t)((zh << 8) | zl);

    sumX += gx;
    sumY += gy;
    sumZ += gz;

    delay(delay_ms);
  }

  // Compute average bias
  gyroBiasX = (float)sumX / samples;
  gyroBiasY = (float)sumY / samples;
  gyroBiasZ = (float)sumZ / samples;
}

void AHRScalibrateAccel(uint8_t sensor_addr, uint16_t samples, uint16_t delay_ms){
  int32_t sumX = 0, sumY = 0, sumZ = 0;

  for(uint16_t i = 0; i < samples; i++){
    uint8_t xh = SensorReceive(sensor_addr, ACCEL_XOUT_H);
    uint8_t xl = SensorReceive(sensor_addr, ACCEL_XOUT_L);
    uint8_t yh = SensorReceive(sensor_addr, ACCEL_YOUT_H);
    uint8_t yl = SensorReceive(sensor_addr, ACCEL_YOUT_L);
    uint8_t zh = SensorReceive(sensor_addr, ACCEL_ZOUT_H);
    uint8_t zl = SensorReceive(sensor_addr, ACCEL_ZOUT_L);

    int16_t ax = (int16_t)((xh << 8) | xl);
    int16_t ay = (int16_t)((yh << 8) | yl);
    int16_t az = (int16_t)((zh << 8) | zl);

    sumX += ax;
    sumY += ay;
    sumZ += az;

    delay(delay_ms);
  }

  float avgX = (float)sumX / samples;
  float avgY = (float)sumY / samples;
  float avgZ = (float)sumZ / samples;

  // Store bias in RAW units
  accelBiasX = avgX;
  accelBiasY = avgY;
  accelBiasZ = avgZ - 4096.0f;  // remove 1 g for ±8 g FS (4096 counts/g)
}

void AHRScalibrateMag(uint8_t sensor_addr, uint16_t samples, uint16_t delay_ms){
  int32_t sumX = 0, sumY = 0, sumZ = 0;

  for(uint16_t i = 0; i < samples; i++){
    uint8_t xl = SensorReceive(sensor_addr, HXL);
    uint8_t xh = SensorReceive(sensor_addr, HXH);
    uint8_t yl = SensorReceive(sensor_addr, HYL);
    uint8_t yh = SensorReceive(sensor_addr, HYH);
    uint8_t zl = SensorReceive(sensor_addr, HZL);
    uint8_t zh = SensorReceive(sensor_addr, HZH);

    int16_t mx = (int16_t)((xh << 8) | xl);
    int16_t my = (int16_t)((yh << 8) | yl);
    int16_t mz = (int16_t)((zh << 8) | zl);

    sumX += mx;
    sumY += my;
    sumZ += mz;

    delay(delay_ms);
  }

  // Compute average bias
  magBiasX = (float)sumX / samples;
  magBiasY = (float)sumY / samples;
  magBiasZ = (float)sumZ / samples;

  // Serial.print("Mag biases: "); 
  // Serial.print(magBiasX); Serial.print(",");
  // Serial.print(magBiasY); Serial.print(",");
  // Serial.println(magBiasZ); 

}


void AHRSStart(){
  AHRSControlStart(AHRS_ADR);
  AHRSAccelStart(AHRS_ADR);
  AHRSGyroStart(AHRS_ADR);
  delay(10);
  AHRSMagStart(MAG_ADR);

  AHRScalibrateAccel(AHRS_ADR);
  AHRScalibrateGyro(AHRS_ADR);
  AHRScalibrateMag(MAG_ADR);
  Serial.println("Palm sensor initialised and calibrated.");
}

void AHRSPoll(uint8_t sensor_addr) {
  uint8_t raw[18]; // 6 bytes accel + 6 bytes gyro
  SensorTransmit(sensor_addr, REG_BANK_SEL, 0x00); // Select bank 0

  // ----- Read accel + gyro in one burst -----
  Wire.beginTransmission(sensor_addr);
  Wire.write(ACCEL_XOUT_H);  // start at accel X high
  Wire.endTransmission(false);
  Wire.requestFrom(sensor_addr, (uint8_t)12);

  for (int i = 0; i < 12; i++) raw[i] = Wire.read();

  int16_t ax_raw = (int16_t)(raw[0] << 8 | raw[1]);
  int16_t ay_raw = (int16_t)(raw[2] << 8 | raw[3]);
  int16_t az_raw = (int16_t)(raw[4] << 8 | raw[5]);

  int16_t gx_raw = (int16_t)(raw[6] << 8 | raw[7]);
  int16_t gy_raw = (int16_t)(raw[8] << 8 | raw[9]);
  int16_t gz_raw = (int16_t)(raw[10] << 8 | raw[11]);

  dataBuffer[0] = (ax_raw - accelBiasX) * scale_ACCEL;
  dataBuffer[1] = (ay_raw - accelBiasY) * scale_ACCEL;
  dataBuffer[2] = (az_raw - accelBiasZ) * scale_ACCEL - 1;

  dataBuffer[3] = gx_raw * scale_GYRO - gyroBiasX;
  dataBuffer[4] = gy_raw * scale_GYRO - gyroBiasY;
  dataBuffer[5] = gz_raw * scale_GYRO - gyroBiasZ;

  uint8_t st1 = SensorReceive(MAG_ADR, 0x10);
  // Serial.print("ST1 = 0x"); Serial.println(st1, HEX);
  // if (!(st1 & 0x01)) return; // no new data yet

  // ----- Read magnetometer in one burst -----
  Wire.beginTransmission(MAG_ADR);
  Wire.write(HXL);  // start at HXL
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADR, (uint8_t)6);

  uint8_t mag_raw[6];
  for (int i = 0; i < 6; i++) {mag_raw[i] = Wire.read();}

  int16_t mx_raw = (int16_t)(mag_raw[1] << 8 | mag_raw[0]);
  int16_t my_raw = (int16_t)(mag_raw[3] << 8 | mag_raw[2]);
  int16_t mz_raw = (int16_t)(mag_raw[5] << 8 | mag_raw[4]);

  // Clear DRDY by reading ST2
  uint8_t st2 = SensorReceive(MAG_ADR, 0x18);
  // Serial.print("ST2 = 0x"); Serial.println(st2, HEX);

  float mx_uT = (mx_raw) * 0.15f;
  float my_uT = (my_raw) * 0.15f;
  float mz_uT = (mz_raw) * 0.15f;

  // Serial.print("Raw mag: "); 
  // Serial.print(mx_raw); Serial.print(",");
  // Serial.print(my_raw); Serial.print(",");
  // Serial.println(mz_raw); 

  dataBuffer[6] = mx_uT;
  dataBuffer[7] = my_uT;
  dataBuffer[8] = mz_uT;

  // dataBuffer[6] = 0;
  // dataBuffer[7] = 0;
  // dataBuffer[8] = 0;

  // Serial.print("RAW: ");
  // Serial.print(ax_raw); Serial.print(",");
  // Serial.print(ay_raw); Serial.print(",");
  // Serial.println(az_raw);

  // ----- Print / transmit CSV -----
  Serial.print(" P: ");
  Serial.print(dataBuffer[0]); Serial.print(",");
  Serial.print(dataBuffer[1]); Serial.print(",");
  Serial.print(dataBuffer[2]); Serial.print(",");
  Serial.print(dataBuffer[3]); Serial.print(",");
  Serial.print(dataBuffer[4]); Serial.print(",");
  Serial.print(dataBuffer[5]); Serial.print(",");
  Serial.print(dataBuffer[6]); Serial.print(",");
  Serial.print(dataBuffer[7]); Serial.print(",");
  Serial.println(dataBuffer[8]);

  // // ----- BLUETOOTH -----
  // Format as CSV string
  
  // delay(2);

}


