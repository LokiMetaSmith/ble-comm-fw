#ifndef LE_AUDIO_H
#define LE_AUDIO_H

#include <zephyr/bluetooth/audio/audio.h>

void le_audio_init(void);

// Function to send audio data (e.g., from Mic)
// Returns 0 on success, negative errno on failure.
int le_audio_send(const uint8_t *data, size_t len);

// Callback type for receiving audio data (e.g. for Speaker)
typedef void (*le_audio_recv_cb_t)(const uint8_t *data, size_t len);

// Register a callback for received audio
void le_audio_register_recv_cb(le_audio_recv_cb_t cb);

#endif
