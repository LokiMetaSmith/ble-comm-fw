#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <stdio.h>
#include <string.h>

#include "le_audio.h"
#include "battery.h"
#include "led_ctrl.h"
#include "audio_proc.h"

LOG_MODULE_REGISTER(main);

#define DEVICE_NAME_PREFIX "ComBadge-"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME_PREFIX) + 4)

static const struct device *i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_dev));
static const struct device *amp_dev = DEVICE_DT_GET(DT_ALIAS(audio_amp));

/* Audio Buffers */
/* 
 * INMP441 Microphones require 64x fs clock. 
 * For 16kHz, this implies 32-bit slots per channel (Stereo).
 * 160 samples * 2 channels * 4 bytes/sample = 1280 bytes.
 */
#define I2S_BLOCK_SAMPLES 160
#define PCM_BLOCK_SIZE (I2S_BLOCK_SAMPLES * 2 * 4) // 1280 bytes
/* Mono Block Size for sending */
#define MONO_BLOCK_SIZE (I2S_BLOCK_SAMPLES * 2)    // 320 bytes (16-bit Mono)

/* Increased slab count to support sound effects queuing */
K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, PCM_BLOCK_SIZE, 16, 4);

/* I2S Config */
struct i2s_config i2s_cfg;

/* State */
static bool is_connected = false;

static void get_random_name(char *name, size_t len)
{
    uint16_t rand_val;
    sys_rand_get(&rand_val, sizeof(rand_val));
    snprintf(name, len, "%s%04X", DEVICE_NAME_PREFIX, rand_val);
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
        is_connected = true;
        led_ctrl_set_state(LED_STATE_CONNECTED);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    is_connected = false;
    led_ctrl_set_state(LED_STATE_ADVERTISING);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Sound Effect Generation */
static void play_chirp(void)
{
    // Generate 100ms chirp: 50ms High (2kHz), 50ms Low (1kHz)
    // 50ms at 10ms/block = 5 blocks. Total 10 blocks.

    // 16kHz sample rate.
    // 2kHz = 8 samples/cycle. (4 samples high, 4 low)
    // 1kHz = 16 samples/cycle. (8 samples high, 8 low)

    int32_t *pcm;
    void *mem_block;
    int ret;

    LOG_INF("Playing Chirp");

    // Amplitude for 32-bit: 0x2000 (from 16-bit) << 16
    const int32_t amp = ((int32_t)0x2000) << 16; 

    // Part 1: High Tone (2kHz) - 5 Blocks
    for (int b = 0; b < 5; b++) {
        ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0) {
            LOG_WRN("Failed to alloc slab for chirp");
            return;
        }
        pcm = (int32_t *)mem_block;
        // Fill block (160 samples per block, Stereo)
        for (int i = 0; i < I2S_BLOCK_SAMPLES; i++) {
            // Square wave: 0-3 High, 4-7 Low (repeat every 8 samples)
            int32_t val = ((i % 8) < 4) ? amp : -amp;
            pcm[2*i] = val;     // Left
            pcm[2*i+1] = val;   // Right
        }
        i2s_write(i2s_dev, mem_block, PCM_BLOCK_SIZE);
    }

    // Part 2: Low Tone (1kHz) - 5 Blocks
    for (int b = 0; b < 5; b++) {
        ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0) return;
        pcm = (int32_t *)mem_block;
        for (int i = 0; i < I2S_BLOCK_SAMPLES; i++) {
             // Square wave: 0-7 High, 8-15 Low (repeat every 16 samples)
             int32_t val = ((i % 16) < 8) ? amp : -amp;
             pcm[2*i] = val;
             pcm[2*i+1] = val;
        }
        i2s_write(i2s_dev, mem_block, PCM_BLOCK_SIZE);
    }
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    /*
     * Using DK_BTN3 as a proxy for the touch button during Phase 1.
     * The device tree alias `touch-button` maps to &button2, but the DK library
     * uses hardcoded button masks. In the actual board file, &button2 corresponds to DK_BTN3_MSK.
     */
    if (has_changed & DK_BTN3_MSK) {
        if (button_state & DK_BTN3_MSK) {
            LOG_INF("Touch Button Pressed");
            led_ctrl_flash_feedback();
            play_chirp();
        }
    }
}

/* Audio Data Callbacks */
static void audio_recv_cb(const uint8_t *data, size_t len)
{
    void *mem_block;
    int ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
    if (ret < 0) return;

    // Incoming data is 16-bit Mono (LE Audio Standard)
    // Output is 32-bit Stereo (I2S Hardware)
    
    int32_t *dst = (int32_t *)mem_block;
    const int16_t *src = (const int16_t *)data;
    size_t samples = len / sizeof(int16_t);
    
    // Ensure we don't overflow the block
    if (samples > I2S_BLOCK_SAMPLES) {
        samples = I2S_BLOCK_SAMPLES;
    }

    for (size_t i = 0; i < samples; i++) {
        // Expand 16-bit to 32-bit (shift to MSB)
        int32_t val = ((int32_t)src[i]) << 16;
        dst[2*i] = val;     // Left
        dst[2*i+1] = val;   // Right
    }
    
    // Fill remaining if any (shouldn't happen with fixed 10ms frames)
    if (samples < I2S_BLOCK_SAMPLES) {
        memset(&dst[2*samples], 0, (I2S_BLOCK_SAMPLES - samples) * 8);
    }

    ret = i2s_write(i2s_dev, mem_block, PCM_BLOCK_SIZE);
    if (ret < 0) {
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
    }
}

static int i2s_setup(void)
{
    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("I2S device not ready");
        return -ENODEV;
    }

    i2s_cfg.word_size = 32; // 32-bit for INMP441 (64x fs)
    i2s_cfg.channels = 2;   // Stereo
    i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    i2s_cfg.frame_clk_freq = 16000; // Match 16kHz
    i2s_cfg.mem_slab = &i2s_mem_slab;
    i2s_cfg.block_size = PCM_BLOCK_SIZE;
    i2s_cfg.timeout = 2000;

    int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) return ret;

    ret = i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
    if (ret < 0) return ret;

    return 0;
}

void main(void)
{
    int err;
    char device_name[DEVICE_NAME_LEN + 1];
    void *mic_buffer;

    LOG_INF("Starting Star Trek Com Badge Firmware");

    led_ctrl_init();

    err = dk_buttons_init(button_handler);
    if (err) LOG_ERR("Failed to init buttons (err %d)", err);

    err = battery_init();
    if (err) LOG_ERR("Failed to init battery (err %d)", err);

    audio_proc_init();

    err = i2s_setup();
    if (err) LOG_ERR("Failed to init I2S (err %d)", err);

    if (!device_is_ready(amp_dev)) {
        LOG_WRN("Audio amplifier not ready");
    } else {
        LOG_INF("Audio amplifier initialized");
    }

    get_random_name(device_name, sizeof(device_name));
    bt_set_name(device_name);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    le_audio_init();
    le_audio_register_recv_cb(audio_recv_cb);

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    led_ctrl_set_state(LED_STATE_ADVERTISING);
    LOG_INF("Advertising started as %s", device_name);

    // Pre-fill TX to prevent immediate underrun
    void *silence_block;
    if (k_mem_slab_alloc(&i2s_mem_slab, &silence_block, K_NO_WAIT) == 0) {
        memset(silence_block, 0, PCM_BLOCK_SIZE);
        i2s_write(i2s_dev, silence_block, PCM_BLOCK_SIZE);
    }

    i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

    // Buffer for 16-bit Stereo conversion (intermediate)
    int16_t stereo_16bit[I2S_BLOCK_SAMPLES * 2];

    while (1) {
        // Handle Mic Data
        size_t size = PCM_BLOCK_SIZE; // 1280 bytes (32-bit Stereo)
        err = i2s_read(i2s_dev, &mic_buffer, &size);
        if (err == 0) {
            // Processing: 32-bit Stereo (I2S) -> 16-bit Stereo -> Audio Proc -> 16-bit Mono (LE Audio)
            
            int32_t *src = (int32_t *)mic_buffer;
            
            // 1. Convert 32-bit Stereo to 16-bit Stereo
            for(int i=0; i < I2S_BLOCK_SAMPLES; i++) {
                // Get top 16 bits of 24-bit data (in 32-bit slot)
                // Assuming data is MSB aligned in 32-bit word, or standard I2S 24-bit in 32-bit slot
                // INMP441 outputs 24-bit data left-justified in the 32-bit frame (or I2S standard)
                // Typically we just take the MSB part. 
                // Using >> 16 handles the MSB extraction if it's 32-bit signed integer.
                stereo_16bit[2*i]     = (int16_t)(src[2*i] >> 16);
                stereo_16bit[2*i+1]   = (int16_t)(src[2*i+1] >> 16);
            }

            // 2. Audio Processing (Noise Cancelling)
            // Expects interleaved 16-bit stereo. Returns 16-bit mono in the same buffer (or output buffer).
            // audio_proc_process(const int16_t *input, int16_t *output, size_t sample_count)
            // It processes `sample_count` frames.
            
            // We can reuse the start of stereo_16bit as the output buffer (in-place if safe, or if it produces mono)
            // audio_proc_process usually takes stereo input and produces mono output?
            // Let's assume it produces Mono output into the output buffer.
            // Wait, looking at the previous merge, it was `audio_proc_process(samples, samples, sample_count);`
            // which implies it supports in-place or separate.
            
            audio_proc_process(stereo_16bit, stereo_16bit, I2S_BLOCK_SAMPLES);

            // 3. Send Mono Data
            // The result is now in the first I2S_BLOCK_SAMPLES elements of stereo_16bit
            le_audio_send((uint8_t *)stereo_16bit, I2S_BLOCK_SAMPLES * sizeof(int16_t));

            k_mem_slab_free(&i2s_mem_slab, &mic_buffer);
        }

        static int64_t next_batt_check = 0;
        int64_t now = k_uptime_get();

        if (now >= next_batt_check) {
            int level = battery_sample();
            if (level < 20) {
                led_ctrl_set_state(LED_STATE_LOW_BATTERY);
            } else if (is_connected) {
                 if (led_ctrl_get_state() == LED_STATE_LOW_BATTERY) {
                     led_ctrl_set_state(LED_STATE_CONNECTED);
                 }
            }
            next_batt_check = now + 10000;
        }

        led_ctrl_process();
        k_sleep(K_MSEC(10));
    }
}
