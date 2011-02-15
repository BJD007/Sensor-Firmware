#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define SAMPLES_PER_CHIRP 512
#define NUM_CHIRPS 64
#define NUM_RX_ANTENNAS 4
#define FFT_SIZE 512
#define MAX_TARGETS 10
#define RANGE_RESOLUTION 0.1f  // meters per range bin
#define VELOCITY_RESOLUTION 0.1f  // m/s per Doppler bin
#define MAX_ASSOCIATION_DISTANCE 2.0f  // meters

// ADC buffer
uint16_t adc_buffer[SAMPLES_PER_CHIRP * NUM_CHIRPS * NUM_RX_ANTENNAS];

// Processed data
float complex range_doppler_map[NUM_CHIRPS][SAMPLES_PER_CHIRP];
float complex angle_fft[NUM_RX_ANTENNAS];

// Detected targets
typedef struct {
    float range;
    float velocity;
    float angle;
} Target;

Target detected_targets[MAX_TARGETS];
int num_detected_targets = 0;

// Hardware handles
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

// Function prototypes
void configure_hardware();
void collect_radar_data();
void preprocess_data();
void range_processing();
void doppler_processing();
void cfar_detection();
void angle_estimation();
void target_tracking();
void send_output();

int main() {
    HAL_Init();
    configure_hardware();

    while (1) {
        collect_radar_data();
        preprocess_data();
        range_processing();
        doppler_processing();
        cfar_detection();
        angle_estimation();
        target_tracking();
        send_output();

        HAL_Delay(16);  // Ensure 60Hz operation
    }

    return 0;
}

void configure_hardware() {
    // Enable clocks for peripherals
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    // Configure ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    // Configure DMA for ADC
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    // Configure timer for 60Hz operation
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (SystemCoreClock / 60) - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // Configure UART for output
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart2);

    // Configure radar front-end GPIO pins (example)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void collect_radar_data() {
    // Trigger radar chirp sequence
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    // Start ADC with DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, SAMPLES_PER_CHIRP * NUM_CHIRPS * NUM_RX_ANTENNAS);

    // Wait for data collection to complete
    while (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_BUSY_REG) {
        // Could add a timeout here
    }

    HAL_ADC_Stop_DMA(&hadc1);
}

void preprocess_data() {
    for (int chirp = 0; chirp < NUM_CHIRPS; chirp++) {
        for (int sample = 0; sample < SAMPLES_PER_CHIRP; sample++) {
            float complex sum = 0;
            for (int antenna = 0; antenna < NUM_RX_ANTENNAS; antenna++) {
                int index = chirp * SAMPLES_PER_CHIRP * NUM_RX_ANTENNAS + sample * NUM_RX_ANTENNAS + antenna;
                float value = (float)adc_buffer[index] - 2048.0f;  // Assuming 12-bit ADC centered at 2048
                float window = 0.5f * (1.0f - cosf(2.0f * M_PI * sample / (SAMPLES_PER_CHIRP - 1)));
                sum += value * window;
            }
            range_doppler_map[chirp][sample] = sum / NUM_RX_ANTENNAS;
        }
    }
}

void range_processing() {
    for (int chirp = 0; chirp < NUM_CHIRPS; chirp++) {
        arm_cfft_f32(&arm_cfft_sR_f32_len512, (float32_t*)range_doppler_map[chirp], 0, 1);
    }
}

void doppler_processing() {
    float complex doppler_fft[NUM_CHIRPS];
    for (int range_bin = 0; range_bin < SAMPLES_PER_CHIRP; range_bin++) {
        for (int chirp = 0; chirp < NUM_CHIRPS; chirp++) {
            doppler_fft[chirp] = range_doppler_map[chirp][range_bin];
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len64, (float32_t*)doppler_fft, 0, 1);
        for (int chirp = 0; chirp < NUM_CHIRPS; chirp++) {
            range_doppler_map[chirp][range_bin] = doppler_fft[chirp];
        }
    }
}

void cfar_detection() {
    const int guard_cells = 2;
    const int training_cells = 4;
    const float pfa = 1e-6;  // Probability of false alarm
    const float scale_factor = powf(pfa, -1.0f / ((2 * training_cells + 1) * (2 * training_cells + 1) - 1)) - 1;

    num_detected_targets = 0;

    for (int doppler = guard_cells + training_cells; doppler < NUM_CHIRPS - guard_cells - training_cells; doppler++) {
        for (int range = guard_cells + training_cells; range < SAMPLES_PER_CHIRP - guard_cells - training_cells; range++) {
            float noise_level = 0;
            int cell_count = 0;

            // Calculate average noise level
            for (int i = -training_cells - guard_cells; i <= training_cells + guard_cells; i++) {
                for (int j = -training_cells - guard_cells; j <= training_cells + guard_cells; j++) {
                    if (abs(i) > guard_cells || abs(j) > guard_cells) {
                        noise_level += cabsf(range_doppler_map[doppler + i][range + j]);
                        cell_count++;
                    }
                }
            }
            noise_level /= cell_count;

            // CFAR detection
            if (cabsf(range_doppler_map[doppler][range]) > noise_level * scale_factor) {
                if (num_detected_targets < MAX_TARGETS) {
                    detected_targets[num_detected_targets].range = range * RANGE_RESOLUTION;
                    detected_targets[num_detected_targets].velocity = (doppler - NUM_CHIRPS / 2) * VELOCITY_RESOLUTION;
                    num_detected_targets++;
                }
            }
        }
    }
}

void angle_estimation() {
    for (int i = 0; i < num_detected_targets; i++) {
        int range_bin = (int)(detected_targets[i].range / RANGE_RESOLUTION);
        int doppler_bin = (int)((detected_targets[i].velocity / VELOCITY_RESOLUTION) + NUM_CHIRPS / 2);

        for (int antenna = 0; antenna < NUM_RX_ANTENNAS; antenna++) {
            angle_fft[antenna] = range_doppler_map[doppler_bin][range_bin];
        }
        
        arm_cfft_f32(&arm_cfft_sR_f32_len4, (float32_t*)angle_fft, 0, 1);
        
        int peak_index = 0;
        float peak_value = 0;
        for (int j = 0; j < NUM_RX_ANTENNAS; j++) {
            float magnitude = cabsf(angle_fft[j]);
            if (magnitude > peak_value) {
                peak_value = magnitude;
                peak_index = j;
            }
        }
        
        detected_targets[i].angle = asinf(peak_index / (float)NUM_RX_ANTENNAS) * 180.0f / M_PI;
    }
}

void target_tracking() {
    static Target previous_targets[MAX_TARGETS];
    static int num_previous_targets = 0;

    for (int i = 0; i < num_detected_targets; i++) {
        float min_distance = INFINITY;
        int closest_target = -1;

        for (int j = 0; j < num_previous_targets; j++) {
            float distance = sqrtf(powf(detected_targets[i].range - previous_targets[j].range, 2) +
                                   powf(detected_targets[i].velocity - previous_targets[j].velocity, 2) +
                                   powf(detected_targets[i].angle - previous_targets[j].angle, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest_target = j;
            }
        }

        if (closest_target != -1 && min_distance < MAX_ASSOCIATION_DISTANCE) {
            detected_targets[i].range = 0.7f * detected_targets[i].range + 0.3f * previous_targets[closest_target].range;
            detected_targets[i].velocity = 0.7f * detected_targets[i].velocity + 0.3f * previous_targets[closest_target].velocity;
            detected_targets[i].angle = 0.7f * detected_targets[i].angle + 0.3f * previous_targets[closest_target].angle;
        }
    }

    memcpy(previous_targets, detected_targets, sizeof(Target) * num_detected_targets);
    num_previous_targets = num_detected_targets;
}

void send_output() {
    char buffer[256];
    int buffer_len = 0;

    buffer_len += sprintf(buffer + buffer_len, "Detected Targets: %d\r\n", num_detected_targets);

    for (int i = 0; i < num_detected_targets; i++) {
        buffer_len += sprintf(buffer + buffer_len, "Target %d: Range=%.2f m, Velocity=%.2f m/s, Angle=%.2f deg\r\n",
                              i + 1, detected_targets[i].range, detected_targets[i].velocity, detected_targets[i].angle);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, buffer_len, HAL_MAX_DELAY);
}
