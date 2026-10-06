/*
 * Implementation of Feature Extraction & Preprocessing
 */

#include "feature_extraction.h"
#include <math.h>

FeatureExtractor::FeatureExtractor() {
    reset();
}

void FeatureExtractor::reset() {
    m_sample_count = 0;
}

bool FeatureExtractor::addSample(const SensorData &sample) {
    if (m_sample_count < SENSOR_WINDOW_SIZE) {
        m_buffer[m_sample_count] = sample;
        m_sample_count++;
    }
    
    if (m_sample_count >= SENSOR_WINDOW_SIZE) {
        return true; // Window buffer full, ready for feature extraction
    }
    return false;
}

void FeatureExtractor::extractFeatures(FeatureVector &out_features) {
    if (m_sample_count == 0) return;
    
    float acc_mags[SENSOR_WINDOW_SIZE];
    float gyro_mags[SENSOR_WINDOW_SIZE];
    
    float sum_acc_mag = 0.0f;
    float sum_gyro_mag = 0.0f;
    
    float min_acc_mag = 1e9f;
    float max_acc_mag = -1e9f;
    
    // 1. Calculate Magnitudes
    for (uint16_t i = 0; i < SENSOR_WINDOW_SIZE; i++) {
        float ax = m_buffer[i].ax;
        float ay = m_buffer[i].ay;
        float az = m_buffer[i].az;
        float amag = sqrtf(ax * ax + ay * ay + az * az);
        
        float gx = m_buffer[i].gx;
        float gy = m_buffer[i].gy;
        float gmag = sqrtf(gx * gx + gy * gy);
        
        acc_mags[i] = amag;
        gyro_mags[i] = gmag;
        
        sum_acc_mag += amag;
        sum_gyro_mag += gmag;
        
        if (amag < min_acc_mag) min_acc_mag = amag;
        if (amag > max_acc_mag) max_acc_mag = amag;
    }
    
    // Mean Acceleration & Gyroscope
    float acc_mean = sum_acc_mag / (float)SENSOR_WINDOW_SIZE;
    float gyro_mean = sum_gyro_mag / (float)SENSOR_WINDOW_SIZE;
    
    // Peak-to-Peak Acceleration Magnitude
    float acc_p2p = max_acc_mag - min_acc_mag;
    
    // 2. Variance & Standard Deviation
    float sum_sq_diff_acc = 0.0f;
    float sum_sq_diff_gyro = 0.0f;
    
    for (uint16_t i = 0; i < SENSOR_WINDOW_SIZE; i++) {
        float diff_acc = acc_mags[i] - acc_mean;
        sum_sq_diff_acc += diff_acc * diff_acc;
        
        float diff_gyro = gyro_mags[i] - gyro_mean;
        sum_sq_diff_gyro += diff_gyro * diff_gyro;
    }
    
    float acc_var = sum_sq_diff_acc / (float)SENSOR_WINDOW_SIZE;
    float acc_std = sqrtf(acc_var);
    float gyro_std = sqrtf(sum_sq_diff_gyro / (float)SENSOR_WINDOW_SIZE);
    
    // Store Raw Unscaled Features
    out_features.raw_features[0] = acc_mean;
    out_features.raw_features[1] = acc_std;
    out_features.raw_features[2] = acc_p2p;
    out_features.raw_features[3] = acc_var;
    out_features.raw_features[4] = gyro_std;
    
    // 3. Preprocess / Scale using StandardScaler formula: (x - mean) / std
    for (int i = 0; i < NUM_INPUT_FEATURES; i++) {
        float std_val = (FEATURE_STDS[i] > 1e-6f) ? FEATURE_STDS[i] : 1.0f;
        out_features.scaled_features[i] = (out_features.raw_features[i] - FEATURE_MEANS[i]) / std_val;
    }
    
    // Reset sample count for next window shift
    reset();
}
