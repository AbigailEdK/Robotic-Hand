#ifndef Header_h   // Put these two lines at the top of your file.
#define Header_h   // (Use a suitable name, usually based on the file name.)

// INCLUDES --------------------------------------------
#include <Wire.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE service and characteristic UUIDs
#define SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define NOTIFY_CHAR_UUID "abcd1234-5678-90ab-cdef-1234567890ab"
#define WRITE_CHAR_UUID  "abcd5678-1234-90ab-cdef-1234567890ab"

extern BLEServer* pServer;
extern BLECharacteristic* pCharacteristic;
extern bool deviceConnected;

extern String receivedCommand;
extern bool commandReady;

// EXTERNAL VARIABLES ----------------------------------
extern volatile bool  Output;
enum Commands {  CMD_ADDR, CMD_FREE, CMD_LOCK, CMD_POLL, CMD_UNKNOWN, CMD_HELP, CMD_CALIB1, CMD_CALIB2 };
extern volatile bool  pollSens;
extern float dataBuffer[39];

extern volatile bool CALIB1;
extern volatile bool CALIB2;

// IMU VARIABLES
extern float imuData[6];  
extern float scale_XL;
extern float scale_G;
extern float XLBiasX[5];
extern float XLBiasY[5];
extern float XLBiasZ[5];
extern float GBiasX[5];
extern float GBiasY[5];
extern float GBiasZ[5];
extern int IMUCounter;

// AHRS VARIABLES
extern float ahrsData[9];  
extern float scale_ACCEL;
extern float scale_GYRO;
extern float accelBiasX;
extern float accelBiasY;
extern float accelBiasZ;
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;
extern float magBiasX;
extern float magBiasY;
extern float magBiasZ;

// Admin Functions -------------------------------------
void startScreen();
Commands parseCommand(const String &cmd);
void CommandChecker(const String &cmd);
int16_t read16(uint8_t sensor_addr, uint8_t reg);
void I2C_init(uint8_t I2C_data, uint8_t I2C_clock);
void I2C_AddressFinder();
void SensorTransmit(uint8_t sensor_addr, uint8_t register_addr, uint8_t message, uint16_t wait_ms = 50);
uint8_t SensorReceive(uint8_t sensor_addr, uint8_t reg_addr);
void BluetoothSerialStart();
void BluetoothTransmit();

// Mux Functions ---------------------------------------
void muxStartup();
void selectMuxChannel(uint8_t channel);
void muxSearcher();
void MUXreadSensors();
void MUXreadIMUparser();

// IMU Functions ---------------------------------------
void IMUTransmit(uint8_t sensor_addr, uint8_t register_addr, uint8_t message, uint16_t wait_ms = 10);
void IMUAccelStart(uint8_t sensor_addr);
void IMUGyroStart(uint8_t sensor_addr);
void IMUcalibrateGyro(uint8_t sensor_addr, uint16_t samples = 100, uint16_t delay_ms = 2);
void IMUcalibrateAccel(uint8_t sensor_addr, uint16_t samples = 100, uint16_t delay_ms = 2);
void IMUMasterConfig(uint8_t sensor_addr);
void IMUControlStart(uint8_t sensor_addr);
void IMUStart(uint8_t sensor_addr);
void transmitData(uint8_t sensor_addr, uint8_t sensor_num);

// AHRS Functions --------------------------------------
void AHRSAccelStart(uint8_t sensor_addr);
void AHRSGyroStart(uint8_t sensor_addr);
void AHRSMagStart(uint8_t sensor_addr);
void AHRSControlStart(uint8_t sensor_addr);
void AHRScalibrateGyro(uint8_t sensor_addr, uint16_t samples = 100, uint16_t delay_ms = 2);
void AHRScalibrateAccel(uint8_t sensor_addr, uint16_t samples = 100, uint16_t delay_ms = 2);
void AHRScalibrateMag(uint8_t sensor_addr, uint16_t samples = 100, uint16_t delay_ms = 2);
void AHRSStart();
void AHRSPoll(uint8_t sensor_addr);
void AHRSTransmit(uint8_t sensor_addr);

#endif // Header.h    // Put this line at the end of your file.
