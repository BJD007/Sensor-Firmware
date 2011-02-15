#ifndef RADAR_77GHZ_H
#define RADAR_77GHZ_H

#include <stdint.h>
#include <complex.h>

// Constants
#define SAMPLES_PER_CHIRP 512
#define NUM_CHIRPS 64
#define NUM_RX_ANTENNAS 4
#define FFT_SIZE 512
#define MAX_TARGETS 10
#define RANGE_RESOLUTION 0.05f  // meters per range bin (might be different for 77GHz)
#define VELOCITY_RESOLUTION 0.05f  // m/s per Doppler bin (might be different for 77GHz)
#define MAX_ASSOCIATION_DISTANCE 2.0f  // meters

// Structures
typedef struct {
    float range;
    float velocity;
    float angle;
} Target;

// Function prototypes
void configure_hardware_77ghz();
void collect_radar_data_77ghz();
void preprocess_data_77ghz();
void range_processing_77ghz();
void doppler_processing_77ghz();
void cfar_detection_77ghz();
void angle_estimation_77ghz();
void target_tracking_77ghz();
void send_output_77ghz();

// Global variables (if needed to be accessed from multiple files)
extern uint16_t adc_buffer[SAMPLES_PER_CHIRP * NUM_CHIRPS * NUM_RX_ANTENNAS];
extern float complex range_doppler_map[NUM_CHIRPS][SAMPLES_PER_CHIRP];
extern float complex angle_fft[NUM_RX_ANTENNAS];
extern Target detected_targets[MAX_TARGETS];
extern int num_detected_targets;

#endif // RADAR_77GHZ_H
