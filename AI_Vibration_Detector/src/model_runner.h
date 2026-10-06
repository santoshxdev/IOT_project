/*
 * TensorFlow Lite Micro Model Runner & AI Inference Engine
 * Executes TinyML model inference on ESP32.
 * Supports both TFLite Micro API and embedded direct C++ neural network fallback engine.
 */

#ifndef MODEL_RUNNER_H
#define MODEL_RUNNER_H

#include <Arduino.h>
#include "config.h"
#include "feature_extraction.h"
#include "vibration_model.h"

struct InferenceOutput {
    ClassificationResult predicted_class;
    float confidence;          // Confidence score (0.0 to 1.0)
    float raw_output;          // Sigmoid output probability for VIBRATION
    unsigned long latency_us;  // Inference duration in microseconds
};

class ModelRunner {
private:
    bool m_initialized;
    uint8_t m_history[STABILIZATION_WINDOW];
    uint8_t m_history_idx;
    uint8_t m_history_count;
    
    // Internal neural network evaluation engine (matching 5 -> 16 -> 8 -> 1 architecture)
    float runEmbeddedNN(const float input_features[5]);

public:
    ModelRunner();
    
    // Initializes TinyML interpreter, verifies model byte array, allocates tensor memory
    bool begin();
    
    // Executes inference on the preprocessed 5 feature input
    bool predict(const FeatureVector &features, InferenceOutput &output);
    
    // Temporal stabilization / Majority voting across N consecutive window predictions
    ClassificationResult stabilizeResult(ClassificationResult current_pred);
    
    bool isInitialized() const { return m_initialized; }
};

#endif // MODEL_RUNNER_H
