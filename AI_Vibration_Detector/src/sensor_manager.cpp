/*
 * Implementation of MPU6050 Sensor Manager
 */

#include "sensor_manager.h"

SensorManager::SensorManager() : m_i2c_addr(0), m_initialized(false) {}

uint8_t SensorManager::readRegister8(uint8_t reg) {
    Wire.beginTransmission(m_i2c_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 0x00;
    }
    Wire.requestFrom(m_i2c_addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0x00;
}

void SensorManager::writeRegister8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(m_i2c_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t SensorManager::scanI2CBus() {
    Serial.println("\n--------------------------------------------------");
    Serial.println("[I2C SCANNER] Scanning I2C bus for devices...");
    Serial.print("Target Pins: SDA = GPIO ");
    Serial.print(I2C_SDA_PIN);
    Serial.print(", SCL = GPIO ");
    Serial.println(I2C_SCL_PIN);
    
    uint8_t count = 0;
    uint8_t found_addr = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print(" -> Found device at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            
            if (address == MPU6050_PRIMARY_ADDR || address == MPU6050_ALT_ADDR) {
                Serial.print(" [MPU6050 DETECTED]");
                found_addr = address;
            }
            Serial.println();
            count++;
        }
    }
    
    if (count == 0) {
        Serial.println(" -> NO I2C DEVICES FOUND ON BUS!");
    } else {
        Serial.print(" -> I2C scan complete. Total devices found: ");
        Serial.println(count);
    }
    Serial.println("--------------------------------------------------\n");
    return found_addr;
}

bool SensorManager::checkWhoAmI() {
    if (m_i2c_addr == 0) return false;
    
    uint8_t who = readRegister8(MPU6050_REG_WHO_AM_I);
    Serial.print("[MPU6050 DIAGNOSTIC] Reading WHO_AM_I register (0x75): 0x");
    if (who < 16) Serial.print("0");
    Serial.println(who, HEX);
    
    if (who == MPU6050_WHO_AM_I_VAL1 || who == MPU6050_WHO_AM_I_VAL2 || who == MPU6050_WHO_AM_I_VAL3) {
        Serial.println("[MPU6050 DIAGNOSTIC] SUCCESS: Valid MPU6050 hardware signature verified!");
        return true;
    } else {
        Serial.print("[MPU6050 DIAGNOSTIC] WARNING: Unexpected WHO_AM_I byte (0x");
        Serial.print(who, HEX);
        Serial.println("). Expected 0x68, 0x70, or 0x72.");
        // Non-blocking fallback if device responded at 0x68/0x69
        return true;
    }
}

bool SensorManager::begin(int sda, int scl) {
    Serial.println("==================================================");
    Serial.println("   AI VIBRATION DETECTOR - MPU6050 INIT SYSTEM    ");
    Serial.println("==================================================");
    
    Wire.begin(sda, scl);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);
    
    // Step 1 & 2: Scan I2C bus
    m_i2c_addr = scanI2CBus();
    
    if (m_i2c_addr == 0) {
        // Retry default primary address 0x68
        m_i2c_addr = MPU6050_PRIMARY_ADDR;
    }
    
    // Step 3 & 4: Check WHO_AM_I
    bool who_ok = checkWhoAmI();
    
    // Step 5: Wake up MPU6050 (Clear sleep bit in PWR_MGMT_1)
    writeRegister8(MPU6050_REG_PWR_MGMT_1, 0x00);
    delay(50);
    
    // Step 6: Configure Accelerometer range (±2g) and Gyroscope range (±250 deg/s)
    writeRegister8(MPU6050_REG_ACCEL_CFG, 0x00);
    writeRegister8(MPU6050_REG_GYRO_CFG, 0x00);
    delay(50);
    
    // Step 7: Verify sensor readings
    SensorData test_data;
    bool read_ok = readSensor(test_data);
    
    if (read_ok && (abs(test_data.ax) > 0.001f || abs(test_data.ay) > 0.001f || abs(test_data.az) > 0.001f)) {
        m_initialized = true;
        Serial.println("[MPU6050 STATUS] MPU6050 Successfully Initialized & Verified!");
        Serial.print("[MPU6050 STATUS] Initial Acceleration Vector -> X: ");
        Serial.print(test_data.ax, 2);
        Serial.print(" m/s^2, Y: ");
        Serial.print(test_data.ay, 2);
        Serial.print(" m/s^2, Z: ");
        Serial.print(test_data.az, 2);
        Serial.println(" m/s^2");
        Serial.println("==================================================\n");
        return true;
    } else {
        m_initialized = false;
        Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("  CRITICAL ERROR: MPU6050 IS NOT FOUND OR FAILING ");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        printDiagnosticReport();
        return false;
    }
}

bool SensorManager::readSensor(SensorData &data) {
    if (m_i2c_addr == 0) return false;
    
    Wire.beginTransmission(m_i2c_addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    // Request 14 bytes: Accel (6), Temp (2), Gyro (6)
    uint8_t bytesReceived = Wire.requestFrom(m_i2c_addr, (uint8_t)14);
    if (bytesReceived < 14) {
        return false;
    }
    
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    
    int16_t raw_temp = (Wire.read() << 8) | Wire.read(); (void)raw_temp; // unused
    
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    
    // Convert LSB to physical values
    data.ax = (float)raw_ax * ACCEL_SCALE_FACTOR;
    data.ay = (float)raw_ay * ACCEL_SCALE_FACTOR;
    data.az = (float)raw_az * ACCEL_SCALE_FACTOR;
    
    data.gx = (float)raw_gx * GYRO_SCALE_FACTOR;
    data.gy = (float)raw_gy * GYRO_SCALE_FACTOR;
    data.gz = (float)raw_gz * GYRO_SCALE_FACTOR;
    
    data.timestamp_ms = millis();
    return true;
}

void SensorManager::printDiagnosticReport() {
    Serial.println("\n==================================================");
    Serial.println("     MPU6050 TROUBLESHOOTING & DIAGNOSTIC GUIDE   ");
    Serial.println("==================================================");
    Serial.println("The ESP32 was unable to communicate with MPU6050.");
    Serial.println("Please check the following potential causes:");
    Serial.println(" 1. INCORRECT WIRING:");
    Serial.print  ("    - Verify ESP32 SDA (GPIO ");
    Serial.print  (I2C_SDA_PIN);
    Serial.println(") -> MPU6050 SDA");
    Serial.print  ("    - Verify ESP32 SCL (GPIO ");
    Serial.print  (I2C_SCL_PIN);
    Serial.println(") -> MPU6050 SCL");
    Serial.println(" 2. POWER SUPPLY:");
    Serial.println("    - MPU6050 VCC -> ESP32 3.3V (or 5V if module has 3.3V LDO regulator)");
    Serial.println("    - MPU6050 GND -> ESP32 GND (MUST share common ground!)");
    Serial.println(" 3. I2C ADDRESS MISMATCH:");
    Serial.println("    - If AD0 pin is GND / floating -> Address is 0x68");
    Serial.println("    - If AD0 pin is pulled HIGH (3.3V) -> Address is 0x69");
    Serial.println(" 4. PULL-UP RESISTORS:");
    Serial.println("    - Standard MPU6050 breakout boards have 4.7k pull-up resistors.");
    Serial.println("    - Ensure breadboard jumper wires are securely connected.");
    Serial.println("==================================================\n");
}
