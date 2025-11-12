#ifndef AHRSADDRESSES_H   // unique macro for this file
#define AHRSADDRESSES_H

// ADDRESSES -------------------------------------------
#define AHRS_ADR 0x68
#define MAG_ADR 0x0C

// CONTROL REGISTERS -----------------------------------
#define USER_CTRL 0x03
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define INT_PIN_CFG 0x0F
#define INT_ENABLE_1 0x11
#define INT_STATUS_1 0x1A
#define FIFO_EN_2 0x67
#define DATA_RDY_STATUS 0x74
#define REG_BANK_SEL 0x7F

// ACCELEROMETER REGISTERS -----------------------------
// CONTROL
#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15

// DATA
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32

// GYROSCOPE REGISTERS ---------------------------------
// CONTROL
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02

// DATA
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

// MAGNETOMETER REGISTERS ------------------------------
// CONTROL
#define ST1 0x10
#define CNTL2 0x31
#define CNTL3 0x32

// DATA
#define HXL 0x11
#define HXH 0x12
#define HYL 0x13
#define HYH 0x14
#define HZL 0x15
#define HZH 0x16

#endif