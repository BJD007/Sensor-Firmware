# 60GHz-FMCW-Radar
The code provides a complete implementation of a radar signal processing chain, including:
Hardware configuration
Data collection
Preprocessing
Range processing
Doppler processing
CFAR detection
Angle estimation
Target tracking
Output transmission
Key points to note:

The code is written for an STM32F4 microcontroller. You'll need to adapt it to your specific hardware.
It assumes the use of ARM CMSIS DSP library for FFT operations.
The code is set up for a 60Hz update rate, with 64 chirps per frame and 512 samples per chirp.
It uses a simple nearest-neighbor approach for target tracking.
Output is sent via UART.
To use this code in a real system% Readme
