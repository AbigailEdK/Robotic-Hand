#include "Header.h"
#include "PinDefinitions.h"
#include "IMUAddresses.h"
#include "BluetoothSerial.h"

// VARIABLES -------------------------------------------
float imuData[6];  
float scale_XL = 0.000244f;
float scale_G  = 0.035f;

float XLBiasX[5];
float XLBiasY[5];
float XLBiasZ[5];

float GBiasX[5];
float GBiasY[5];
float GBiasZ[5];

int IMUCounter = 0;

// FUNCTIONS -------------------------------------------
// Transmission to IMU
void IMUTransmit(uint8_t sensor_addr, uint8_t register_addr, uint8_t message, uint16_t wait_ms) {
  Wire.beginTransmission(sensor_addr);
  Wire.write(register_addr);
  Wire.write(message);
  Wire.endTransmission();
  delay(wait_ms);
}

// Wake accelerometer from sleep
void IMUAccelStart(uint8_t sensor_addr) {
  IMUTransmit(sensor_addr, CTRL1_XL, 0x7C); // 883 Hz, 8g, 400 Hz filter
  IMUTransmit(sensor_addr, CTRL8_XL, 0x01); // LPF settings
  IMUTransmit(sensor_addr, CTRL9_XL, 0x38); // Axis enables; magnetometer correction

  delay(5);
}

void IMUcalibrateAccel(uint8_t sensor_addr, uint16_t samples, uint16_t delay_ms) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t raw[6];

    // Serial.println("Calibrating accelerometer, keep sensor stationary...");

    for (uint16_t i = 0; i < samples; i++) {
        // Read accel
        Wire.beginTransmission(sensor_addr);
        Wire.write(OUTX_L_XL);
        Wire.endTransmission(false);
        Wire.requestFrom(sensor_addr, (uint8_t)6);

        for (int j = 0; j < 6; j++) raw[j] = Wire.read();

        int16_t ax = (int16_t)(raw[1] << 8 | raw[0]);
        int16_t ay = (int16_t)(raw[3] << 8 | raw[2]);
        int16_t az = (int16_t)(raw[5] << 8 | raw[4]);

        sumX += ax;
        sumY += ay;
        sumZ += az;

        delay(delay_ms);
    }

    // Compute average in g
    XLBiasX[IMUCounter] = (float)sumX / samples * scale_XL;
    XLBiasY[IMUCounter] = (float)sumY / samples * scale_XL;
    XLBiasZ[IMUCounter] = (float)sumZ / samples * scale_XL - 1.0f;  // subtract 1g for gravity

    // Serial.print("Accel bias X: "); Serial.println(XLBiasX[IMUCounter]);
    // Serial.print("Accel bias Y: "); Serial.println(XLBiasY[IMUCounter]);
    // Serial.print("Accel bias Z: "); Serial.println(XLBiasZ[IMUCounter]);
}

// Wake gyroscope from sleep
void IMUGyroStart(uint8_t sensor_addr) {
  IMUTransmit(sensor_addr, CTRL2_G, 0x78); // 833 Hz, 1000 dps
  IMUTransmit(sensor_addr, CTRL7_G, 0x00); // High performance mode, no HPF
  IMUTransmit(sensor_addr, CTRL10_C, 0x3C); // Axis enables, accel filter enable

  delay(5);
}

void IMUcalibrateGyro(uint8_t sensor_addr, uint16_t samples, uint16_t delay_ms) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t raw[6];

    // Serial.println("Calibrating gyroscope, keep sensor stationary...");

    for (uint16_t i = 0; i < samples; i++) {
        // Read gyro
        Wire.beginTransmission(sensor_addr);
        Wire.write(OUTX_L_G);
        Wire.endTransmission(false);
        Wire.requestFrom(sensor_addr, (uint8_t)6);

        for (int j = 0; j < 6; j++) raw[j] = Wire.read();

        int16_t gx = (int16_t)(raw[1] << 8 | raw[0]);
        int16_t gy = (int16_t)(raw[3] << 8 | raw[2]);
        int16_t gz = (int16_t)(raw[5] << 8 | raw[4]);

        sumX += gx;
        sumY += gy;
        sumZ += gz;

        delay(delay_ms);
    }

    GBiasX[IMUCounter] = (float)sumX / samples * scale_G;
    GBiasY[IMUCounter] = (float)sumY / samples * scale_G;
    GBiasZ[IMUCounter] = (float)sumZ / samples * scale_G;

    // Serial.print("Gyro bias X: "); Serial.println(GBiasX[IMUCounter]);
    // Serial.print("Gyro bias Y: "); Serial.println(GBiasY[IMUCounter]);
    // Serial.print("Gyro bias Z: "); Serial.println(GBiasZ[IMUCounter]);
}

void IMUMasterConfig(uint8_t sensor_addr) {
  IMUTransmit(sensor_addr, MASTER_CONFIG, 0x00); // Data ready on int 1

  delay(5);
}

void IMUControlStart(uint8_t sensor_addr) {
  IMUTransmit(sensor_addr, CTRL3_C, 0x64); // Output data updates after read, incr register addr
  IMUTransmit(sensor_addr, CTRL4_C, 0x08); // Mask DRDY on wake up
  IMUTransmit(sensor_addr, CTRL5_C, 0x00); // Set rounding for gyro & accel
  IMUTransmit(sensor_addr, CTRL6_C, 0x00); // Gyro triggers, high performance for Accel

  delay(10);
}

// Start up accel, gyro, interrupts, and configure IMU
void IMUStart(uint8_t sensor_addr) {
  IMUAccelStart(sensor_addr);      // Initialize accelerometer
  IMUGyroStart(sensor_addr);       // Initialize gyroscope
  IMUMasterConfig(sensor_addr);    // Configure master settings
  IMUControlStart(sensor_addr);    // General IMU control

  IMUcalibrateAccel(sensor_addr);
  IMUcalibrateGyro(sensor_addr);

  Serial.print("Finger Sensor "); Serial.print(IMUCounter+1); Serial.println(" initialised and calibrated.");

  if (IMUCounter == 4){
    IMUCounter = 0;
  } else {
    IMUCounter++;
  }
}

void transmitData(uint8_t sensor_addr, uint8_t sensor_num) {
    uint8_t raw[6];

    // ----- Read accelerometer -----
    Wire.beginTransmission(sensor_addr);
    Wire.write(OUTX_L_XL);  // start at accel X low
    Wire.endTransmission(false);
    Wire.requestFrom(sensor_addr, (uint8_t)6);

    for (int i = 0; i < 6; i++) raw[i] = Wire.read();
    int16_t ax_raw = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t ay_raw = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t az_raw = (int16_t)(raw[5] << 8 | raw[4]);

    dataBuffer[9 + IMUCounter*6] = ax_raw * scale_XL - XLBiasX[sensor_num];  
    dataBuffer[10 + IMUCounter*6] = ay_raw * scale_XL - XLBiasY[sensor_num];
    dataBuffer[11 + IMUCounter*6] = az_raw * scale_XL - XLBiasZ[sensor_num];

    // ----- Read gyroscope -----
    Wire.beginTransmission(sensor_addr);
    Wire.write(OUTX_L_G);  // start at gyro X low
    Wire.endTransmission(false);
    Wire.requestFrom(sensor_addr, (uint8_t)6);

    for (int i = 0; i < 6; i++) raw[i] = Wire.read();
    int16_t gx_raw = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t gy_raw = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t gz_raw = (int16_t)(raw[5] << 8 | raw[4]);

    dataBuffer[12 + IMUCounter*6] = gx_raw * scale_G - GBiasX[sensor_num];
    dataBuffer[13 + IMUCounter*6] = gy_raw * scale_G - GBiasY[sensor_num];
    dataBuffer[14 + IMUCounter*6] = gz_raw * scale_G - GBiasZ[sensor_num];

    // // ----- Send CSV data over Serial (Python-friendly) -----
    Serial.print("F"); Serial.print(IMUCounter+1); Serial.print(": ");
    Serial.print(dataBuffer[9 + IMUCounter*6]); Serial.print(",");
    Serial.print(dataBuffer[10 + IMUCounter*6]); Serial.print(",");
    Serial.print(dataBuffer[11 + IMUCounter*6]); Serial.print(",");
    Serial.print(dataBuffer[12 + IMUCounter*6]); Serial.print(",");
    Serial.print(dataBuffer[13 + IMUCounter*6]); Serial.print(",");
    Serial.println(dataBuffer[14 + IMUCounter*6]);  // newline signals end of row

    if (IMUCounter == 4){
      IMUCounter = 0;
    } else {
      IMUCounter++;
    }

    // delay(2);
}

