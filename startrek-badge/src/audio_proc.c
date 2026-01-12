#include "audio_proc.h"
#include <string.h>

/*
 * Simple Delay-and-Sum Beamformer
 *
 * Mics are L/R.
 * Broadside array (Speech from front) -> Delay = 0. Sum L+R.
 * Endfire array (Speech from end) -> Delay = d/c.
 *
 * Default to Broadside Summation for noise reduction (uncorrelated noise reduction).
 * To enable "Background Reduction" via differential (Endfire), set delay > 0 and subtract.
 *
 * Since we don't know the exact geometry, we provide a configurable delay.
 * For now, we assume Broadside (Delay=0) and Sum.
 *
 * To implement Time Difference, we use a circular buffer for one channel.
 */

#define MAX_DELAY_SAMPLES 32
#define DEFAULT_DELAY_SAMPLES 0

// Buffer to hold delayed samples for Channel 2 (Right)
static int16_t delay_buffer[MAX_DELAY_SAMPLES];
static int delay_index = 0;
static int current_delay = DEFAULT_DELAY_SAMPLES;

void audio_proc_init(void)
{
    memset(delay_buffer, 0, sizeof(delay_buffer));
    delay_index = 0;
    current_delay = DEFAULT_DELAY_SAMPLES;
}

void audio_proc_process(const int16_t *input_stereo, int16_t *output_mono, size_t sample_count)
{
    for (size_t i = 0; i < sample_count; i++) {
        int16_t left = input_stereo[2 * i];
        int16_t right = input_stereo[2 * i + 1];

        // Store current Right sample in delay buffer
        delay_buffer[delay_index] = right;

        // Retrieve delayed Right sample
        // index - delay (modulo size)
        int read_index = (delay_index - current_delay);
        if (read_index < 0) {
            read_index += MAX_DELAY_SAMPLES;
        }
        int16_t right_delayed = delay_buffer[read_index];

        // Update write index
        delay_index++;
        if (delay_index >= MAX_DELAY_SAMPLES) {
            delay_index = 0;
        }

        // Beamforming Logic
        // Simple Sum (Broadside): (L + R) / 2
        // This improves SNR by 3dB for correlated signal (speech) vs uncorrelated noise.
        int32_t sum = (int32_t)left + (int32_t)right_delayed;

        // If we wanted differential (Endfire nulling rear):
        // int32_t diff = (int32_t)left - (int32_t)right_delayed;
        // output_mono[i] = (int16_t)diff;

        // We use the sum, divided by 2.
        // Since input is 16-bit, sum is max 65534. Divided by 2 fits in 16-bit.
        output_mono[i] = (int16_t)(sum / 2);
    }
}
