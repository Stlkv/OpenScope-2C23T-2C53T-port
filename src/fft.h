#ifndef FFT_H
#define FFT_H

#include <stdint.h>

#define FFT_SIZE 256
#define FFT_BITS 8


void compute_fft(const float *input_real, float *output_magnitude, uint8_t window_type);
void compute_fft_bin(const float *input_real, uint8_t window_type, uint16_t bin,
                     float *out_magnitude, float *out_phase_rad);

#endif // FFT_H
