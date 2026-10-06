/*
 * AI-Based Vibration Detection System Using ESP32 and MPU6050
 * Main Arduino Sketch
 *
 * Microcontroller: ESP32 Development Board
 * Sensor: MPU6050 Accelerometer + Gyroscope (I2C)
 * AI Engine: TensorFlow Lite Micro
 * Classification: NORMAL (0) / VIBRATION (1)
 */

#include <Arduino.h>
#include <Wire.h>

#include "src/config.h"
#include "src/sensor_manager.h"
#include "src/feature_extraction.h"
#include "src/model_runner.h"

// System Components
SensorManager g_sensor;
FeatureExtractor g_extractor;
ModelRunner g_model;

unsigned long g_last_sample_time = 0;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    
    Serial.println("\n");
    Serial.println("==================================================");
    Serial.println("       AI VIBRATION DETECTOR (ESP32 + MPU6050)    ");
    Serial.println("==================================================");
    Serial.println("Initializing hardware and TinyML model...\n");
    
    // STAGE 1: Initialize Sensor & Scan I2C
    if (!g_sensor.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("\n[SYSTEM ERROR] MPU6050 initialization failed!");
        Serial.println("System halted. Please check hardware wiring and reset board.");
        while (1) {
            delay(1000);
        }
    }
    
    // STAGE 4: Initialize TFLite Micro Model Engine
    if (!g_model.begin()) {
        Serial.println("\n[SYSTEM ERROR] TensorFlow Lite model loading failed!");
        Serial.println("System halted. Please check src/vibration_model.h.");
        while (1) {
            delay(1000);
        }
    }
    
    g_extractor.reset();
    
    Serial.println("==================================================");
    Serial.println(" MPU6050 detected");
    Serial.println(" AI model loaded");
    Serial.println(" Starting continuous real-time inference loop...");
    Serial.println("==================================================\n");
}

void loop() {
    unsigned long current_time = millis();
    
    // Maintain 50 Hz sampling rate (20 ms interval)
    if (current_time - g_last_sample_time >= SAMPLE_INTERVAL_MS) {
        g_last_sample_time = current_time;
        
        SensorData raw_data;
        if (g_sensor.readSensor(raw_data)) {
            // STAGE 3: Add raw sample to 50-sample window buffer
            bool window_ready = g_extractor.addSample(raw_data);
            
            // Perform feature extraction and inference once window buffer is full (50 samples)
            if (window_ready) {
                FeatureVector features;
                g_extractor.extractFeatures(features);
                
                InferenceOutput output;
                // STAGE 5: Run TFLite inference on scaled features
                if (g_model.predict(features, output)) {
                    // STAGE 6: Apply temporal stabilization majority voting filter
                    ClassificationResult stable_class = g_model.stabilizeResult(output.predicted_class);
                    
                    // STAGE 8: Display Output based on DEBUG_MODE
#if DEBUG_MODE
                    Serial.println("==================================================");
                    Serial.print("I2C Address: 0x"); Serial.println(g_sensor.getI2CAddress(), HEX);
                    
                    Serial.println("Raw Sensor Values:");
                    Serial.print("  Accel -> X: "); Serial.print(raw_data.ax, 2);
                    Serial.print(" m/s^2, Y: "); Serial.print(raw_data.ay, 2);
                    Serial.print(" m/s^2, Z: "); Serial.print(raw_data.az, 2); Serial.println(" m/s^2");
                    Serial.print("  Gyro  -> X: "); Serial.print(raw_data.gx, 2);
                    Serial.print(" deg/s, Y: "); Serial.print(raw_data.gy, 2);
                    Serial.print(" deg/s, Z: "); Serial.print(raw_data.gz, 2); Serial.println(" deg/s");
                    
                    Serial.println("\nExtracted 5 Raw Features:");
                    Serial.print("  1. acc_mean : "); Serial.print(features.raw_features[0], 3); Serial.println(" m/s^2");
                    Serial.print("  2. acc_std  : "); Serial.print(features.raw_features[1], 3); Serial.println(" m/s^2");
                    Serial.print("  3. acc_p2p  : "); Serial.print(features.raw_features[2], 3); Serial.println(" m/s^2");
                    Serial.print("  4. acc_var  : "); Serial.print(features.raw_features[3], 3); Serial.println();
                    Serial.print("  5. gyro_std : "); Serial.print(features.raw_features[4], 3); Serial.println(" deg/s");
                    
                    Serial.println("\nStandardScaler Normalized Features [(x - mean) / std]:");
                    for (int i = 0; i < NUM_INPUT_FEATURES; i++) {
                        Serial.print("  Feature ["); Serial.print(i); Serial.print("]: ");
                        Serial.println(features.scaled_features[i], 4);
                    }
                    
                    Serial.println("\nModel Inference Output:");
                    Serial.print("  Sigmoid Probability P(VIBRATION): "); Serial.println(output.raw_output, 6);
                    Serial.print("  Raw Classification  : ");
                    Serial.println(output.predicted_class == CLASS_VIBRATION ? "VIBRATION" : "NORMAL");
                    Serial.print("  Stabilized Result   : ");
                    Serial.println(stable_class == CLASS_VIBRATION ? "*** VIBRATION ***" : "NORMAL");
                    Serial.print("  Confidence          : "); Serial.print(output.confidence * 100.0f, 2); Serial.println("%");
                    Serial.print("  Inference Latency   : "); Serial.print(output.latency_us); Serial.println(" us");
                    Serial.println("==================================================\n");
#else
                    // Clean output mode (DEBUG_MODE = 0)
                    Serial.print("AI Prediction: ");
                    if (stable_class == CLASS_VIBRATION) {
                        Serial.println("VIBRATION");
                    } else {
                        Serial.println("NORMAL");
                    }
                    Serial.print("Vibration Probability: ");
                    Serial.print(output.raw_output * 100.0f, 2);
                    Serial.println("%\n");
#endif
                }
            }
        } else {
            Serial.println("[ERROR] Sensor read error! Check I2C bus wiring.");
        }
    }
}
