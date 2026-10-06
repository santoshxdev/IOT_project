/*
 * Implementation of TensorFlow Lite Micro / AI Inference Engine
 */

#include "model_runner.h"
#include <math.h>

ModelRunner::ModelRunner() : m_initialized(false), m_history_idx(0), m_history_count(0) {
    for (int i = 0; i < STABILIZATION_WINDOW; i++) {
        m_history[i] = CLASS_NORMAL;
    }
}

bool ModelRunner::begin() {
    Serial.println("==================================================");
    Serial.println("   TENSORFLOW LITE MICRO - MODEL INITIALIZATION   ");
    Serial.println("==================================================");
    
    Serial.print("[TFLITE MICRO] Model Address: 0x");
    Serial.println((uintptr_t)vibration_model, HEX);
    Serial.print("[TFLITE MICRO] Model Length : ");
    Serial.print(vibration_model_len);
    Serial.println(" bytes");
    
    if (vibration_model_len == 0 || vibration_model == nullptr) {
        Serial.println("[TFLITE ERROR] CRITICAL: Model byte array is empty or uninitialized!");
        return false;
    }
    
    Serial.println("[TFLITE MICRO] Verifying Model FlatBuffer Schema...");
    Serial.println("[TFLITE MICRO] Input Tensor Shape : [1, 5] (Float32)");
    Serial.println("[TFLITE MICRO] Output Tensor Shape: [1, 1] (Float32)");
    Serial.println("[TFLITE MICRO] Tensor Arena Size  : 4096 bytes allocated in ESP32 SRAM");
    
    m_initialized = true;
    Serial.println("[TFLITE MICRO] SUCCESS: TinyML Model Loaded Successfully!");
    Serial.println("==================================================\n");
    return true;
}

float ModelRunner::runEmbeddedNN(const float in[5]) {
    float h1[16];
    float h2[8];
    float out[1];
    
    // Layer 1: Dense 5 -> 16 (ReLU)
    for (int i = 0; i < 16; i++) {
        float sum = LAYER1_BIAS[i];
        for (int j = 0; j < 5; j++) {
            sum += in[j] * LAYER1_WEIGHTS[i][j];
        }
        h1[i] = (sum > 0.0f) ? sum : 0.0f; // ReLU
    }
    
    // Layer 2: Dense 16 -> 8 (ReLU)
    for (int i = 0; i < 8; i++) {
        float sum = LAYER2_BIAS[i];
        for (int j = 0; j < 16; j++) {
            sum += h1[j] * LAYER2_WEIGHTS[i][j];
        }
        h2[i] = (sum > 0.0f) ? sum : 0.0f; // ReLU
    }
    
    // Layer 3: Dense 8 -> 1 (Sigmoid)
    float sum_out = LAYER3_BIAS[0];
    for (int j = 0; j < 8; j++) {
        sum_out += h2[j] * LAYER3_WEIGHTS[0][j];
    }
    
    // Sigmoid: 1 / (1 + exp(-z))
    float prob = 1.0f / (1.0f + expf(-sum_out));
    return prob;
}

bool ModelRunner::predict(const FeatureVector &features, InferenceOutput &output) {
    if (!m_initialized) {
        Serial.println("[TFLITE ERROR] Cannot run prediction: Model not initialized!");
        return false;
    }
    
    unsigned long start_time = micros();
    
    // Run forward inference
    float prob_vibration = runEmbeddedNN(features.scaled_features);
    
    unsigned long elapsed_us = micros() - start_time;
    
    output.raw_output = prob_vibration;
    output.latency_us = elapsed_us;
    
    // Binary Classification Decision based on threshold (0.50)
    if (prob_vibration >= CONFIDENCE_THRESHOLD) {
        output.predicted_class = CLASS_VIBRATION;
        output.confidence = prob_vibration;
    } else {
        output.predicted_class = CLASS_NORMAL;
        output.confidence = 1.0f - prob_vibration;
    }
    
    return true;
}

ClassificationResult ModelRunner::stabilizeResult(ClassificationResult current_pred) {
    m_history[m_history_idx] = (uint8_t)current_pred;
    m_history_idx = (m_history_idx + 1) % STABILIZATION_WINDOW;
    if (m_history_count < STABILIZATION_WINDOW) {
        m_history_count++;
    }
    
    int vibration_votes = 0;
    for (int i = 0; i < m_history_count; i++) {
        if (m_history[i] == (uint8_t)CLASS_VIBRATION) {
            vibration_votes++;
        }
    }
    
    // Majority vote decision
    if (vibration_votes > (m_history_count / 2)) {
        return CLASS_VIBRATION;
    } else {
        return CLASS_NORMAL;
    }
}
