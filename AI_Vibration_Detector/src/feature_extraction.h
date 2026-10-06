/*
 * Feature Extraction Module
 * Collects 50 sensor samples in a sliding buffer, calculates the 5 statistical features,
 * and performs StandardScaler feature normalization matching the trained Python model.
 */

#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <Arduino.h>
#include "config.h"
#include "sensor_manager.h"
#include "vibration_model.h"

struct FeatureVector {
    float raw_features[NUM_INPUT_FEATURES];    // Unscaled features: acc_mean, acc_std, acc_p2p, acc_var, gyro_std
    float scaled_features[NUM_INPUT_FEATURES]; // StandardScaler normalized features
};

class FeatureExtractor {
private:
    SensorData m_buffer[SENSOR_WINDOW_SIZE];
    uint16_t m_sample_count;
    
public:
    FeatureExtractor();
    
    // Resets buffer sample count
    void reset();
    
    // Adds a new sample to the window buffer. Returns true when window buffer is full (50 samples)
    bool addSample(const SensorData &sample);
    
    // Computes the 5 statistical features and standardizes them using FEATURE_MEANS and FEATURE_STDS
    void extractFeatures(FeatureVector &out_features);
    
    uint16_t getSampleCount() const { return m_sample_count; }
};

#endif // FEATURE_EXTRACTION_H
