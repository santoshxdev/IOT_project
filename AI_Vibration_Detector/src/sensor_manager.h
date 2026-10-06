/*
 * MPU6050 Sensor Manager Module
 * Handles I2C bus scanning, sensor detection, WHO_AM_I check, hardware initialization,
 * diagnostic troubleshooting prints, and continuous data sampling.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

struct SensorData {
    float ax; // Acceleration X (m/s^2)
    float ay; // Acceleration Y (m/s^2)
    float az; // Acceleration Z (m/s^2)
    float gx; // Gyroscope X (deg/s)
    float gy; // Gyroscope Y (deg/s)
    float gz; // Gyroscope Z (deg/s)
    unsigned long timestamp_ms;
};

class SensorManager {
private:
    uint8_t m_i2c_addr;
    bool m_initialized;
    
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t val);
    
public:
    SensorManager();
    
    // Scans I2C bus, checks for 0x68/0x69 addresses, verifies WHO_AM_I, and initializes sensor
    bool begin(int sda = I2C_SDA_PIN, int scl = I2C_SCL_PIN);
    
    // Performs standalone I2C bus scan and prints detailed output
    uint8_t scanI2CBus();
    
    // Verifies MPU6050 register response
    bool checkWhoAmI();
    
    // Reads 6-axis raw sensor data and converts to engineering units
    bool readSensor(SensorData &data);
    
    // Diagnostics & Error print helper
    void printDiagnosticReport();
    
    uint8_t getI2CAddress() const { return m_i2c_addr; }
    bool isInitialized() const { return m_initialized; }
};

#endif // SENSOR_MANAGER_H
