/*
 * Configuration Header for AI-Based Vibration Detection System
 * Target Hardware: ESP32 + MPU6050 (I2C)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// -------------------------------------------------------------
// 1. I2C HARDWARE PIN DEFINITIONS (ESP32)
// Default ESP32 I2C pins: SDA = 21, SCL = 22
// Adjust these constants if using customized board pinout.
// -------------------------------------------------------------
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif

#define I2C_FREQ_HZ 400000  // 400 kHz Fast I2C Bus Speed

// -------------------------------------------------------------
// 2. MPU6050 I2C ADDRESSES & REGISTERS
// -------------------------------------------------------------
#define MPU6050_PRIMARY_ADDR   0x68
#define MPU6050_ALT_ADDR       0x69

#define MPU6050_REG_WHO_AM_I   0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_ACCEL_XOUT 0x3B

// Expected WHO_AM_I response bytes (0x68 for MPU6050, 0x70/0x72 for clones)
#define MPU6050_WHO_AM_I_VAL1  0x68
#define MPU6050_WHO_AM_I_VAL2  0x70
#define MPU6050_WHO_AM_I_VAL3  0x72

// -------------------------------------------------------------
// 3. SENSOR SAMPLING & WINDOW CONFIGURATION
// -------------------------------------------------------------
#define SENSOR_WINDOW_SIZE    50     // 50 samples per feature window
#define SAMPLING_RATE_HZ      50     // 50 Hz sampling rate
#define SAMPLE_INTERVAL_MS    (1000 / SAMPLING_RATE_HZ) // 20 ms interval

// Accelerometer scale factor for ±2g (16384 LSB/g -> m/s^2 conversion)
#define ACCEL_SCALE_FACTOR    (9.81f / 16384.0f)

// Gyroscope scale factor for ±250 deg/s (131 LSB / (deg/s))
#define GYRO_SCALE_FACTOR     (1.0f / 131.0f)

// -------------------------------------------------------------
// 4. PREPROCESSING & INFERENCE CONFIGURATION
// -------------------------------------------------------------
#define NUM_INPUT_FEATURES    5      // acc_mean, acc_std, acc_p2p, acc_var, gyro_std
#define CONFIDENCE_THRESHOLD  0.50f  // Probability >= 0.50 triggers VIBRATION class

// Temporal Stabilization (Majority voting over N window predictions)
#define STABILIZATION_WINDOW  3      // Debounce over last 3 consecutive inferences

// -------------------------------------------------------------
// 5. DEBUG & SERIAL MONITOR CONTROLS (STAGE 8 REQUIREMENT)
// -------------------------------------------------------------
#define SERIAL_BAUD_RATE      115200

// DEBUG_MODE:
// 1 = Verbose printing (I2C address, raw values, 5 raw features, 5 scaled features, TFLite tensor info, timing)
// 0 = Clean production mode (Final prediction & probability score only)
#define DEBUG_MODE            1

// Classification Label Enum
enum ClassificationResult {
    CLASS_NORMAL = 0,
    CLASS_VIBRATION = 1,
    CLASS_UNKNOWN = -1
};

#endif // CONFIG_H
