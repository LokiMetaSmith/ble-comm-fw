#ifndef AUDIO_PROC_H
#define AUDIO_PROC_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the audio processing module.
 */
void audio_proc_init(void);

/**
 * @brief Process stereo audio data to mono using beamforming.
 *
 * @param input_stereo  Pointer to the stereo input buffer (interleaved L/R).
 * @param output_mono   Pointer to the mono output buffer.
 * @param sample_count  Number of stereo samples (input frames).
 *                      The input buffer must be at least sample_count * 2 samples.
 *                      The output buffer must be at least sample_count samples.
 */
void audio_proc_process(const int16_t *input_stereo, int16_t *output_mono, size_t sample_count);

#endif // AUDIO_PROC_H
