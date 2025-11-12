#ifndef IMUADDRESSES_H   // unique macro for this file
#define IMUADDRESSES_H

#define LSM_ADR 0x6B

// FIFO REGISTERS --------------------------------------
#define FIFO_CTRL1 0x06
#define FIFO_CTRL2 0x07
#define FIFO_CTRL3 0x08
#define FIFO_CTRL4 0x09
#define FIFO_CTRL5 0x0A

// INTERRUPT REGISTERS ---------------------------------
#define INT1_CTRL 0x0D
#define INT2_CTRL 0x0E

// CONTROL REGISTERS -----------------------------------
#define MASTER_CONFIG 0x1A
#define WAKE_UP_SRC 0x1B
#define STATUS_REG 0x1E
#define WHO_AM_I 0x0F

#define CTRL3_C 0x12
#define CTRL4_C 0x13
#define CTRL5_C 0x14
#define CTRL6_C 0x15
#define CTRL10_C 0x19

// ACCELEROMETER REGISTERS -----------------------------
// CONTROL
#define CTRL1_XL 0x10
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18

// DATA
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

// GYROSCOPE REGISTERS ---------------------------------
// CONTROL
#define CTRL2_G 0x11
#define CTRL7_G 0x16

// DATA
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#endif