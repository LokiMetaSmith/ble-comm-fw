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
/* PCM_BLOCK_SIZE increased to 640 to hold stereo samples (2 channels * 160 samples * 2 bytes) */
#define PCM_BLOCK_SIZE 640
/* Mono Block Size for sending */
#define MONO_BLOCK_SIZE 320

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

    int16_t *pcm;
    void *mem_block;
    int ret;

    LOG_INF("Playing Chirp");

    // Part 1: High Tone (2kHz) - 5 Blocks
    for (int b = 0; b < 5; b++) {
        ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0) {
            LOG_WRN("Failed to alloc slab for chirp");
            return;
        }
        pcm = (int16_t *)mem_block;
        // Fill block (160 stereo samples per block -> 320 values)
        for (int i = 0; i < 160; i++) {
            // Square wave: 0-3 High, 4-7 Low (repeat every 8 samples)
            int16_t val = ((i % 8) < 4) ? 0x2000 : -0x2000; // ~12% amplitude
            // Interleaved Stereo: Left = val, Right = val
            pcm[2 * i] = val;
            pcm[2 * i + 1] = val;
        }
        i2s_write(i2s_dev, mem_block, PCM_BLOCK_SIZE);
    }

    // Part 2: Low Tone (1kHz) - 5 Blocks
    for (int b = 0; b < 5; b++) {
        ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0) return;
        pcm = (int16_t *)mem_block;
        for (int i = 0; i < 160; i++) {
             // Square wave: 0-7 High, 8-15 Low (repeat every 16 samples)
             int16_t val = ((i % 16) < 8) ? 0x2000 : -0x2000;
             pcm[2 * i] = val;
             pcm[2 * i + 1] = val;
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

    int16_t *pcm_out = (int16_t *)mem_block;
    const int16_t *pcm_in = (const int16_t *)data;
    size_t sample_count = len / sizeof(int16_t);

    // Safety check for buffer overrun
    if (sample_count > 160) sample_count = 160;

    // Expand Mono to Stereo
    for (size_t i = 0; i < sample_count; i++) {
        pcm_out[2 * i] = pcm_in[i];
        pcm_out[2 * i + 1] = pcm_in[i];
    }

    // If input was shorter, pad with silence?
    // Usually len matches expected frame.
    // However, slab size is now 640 bytes. We should ensure we write full block or handle it.
    // The previous code just memcpy'd.

    // Fill remaining buffer with 0 if necessary
    if (sample_count < 160) {
         memset(&pcm_out[2 * sample_count], 0, (160 - sample_count) * 2 * sizeof(int16_t));
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

    i2s_cfg.word_size = 16;
    i2s_cfg.channels = 2; // Stereo
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

    while (1) {
        // Handle Mic Data
        size_t size = PCM_BLOCK_SIZE; // 640 bytes (Stereo)
        err = i2s_read(i2s_dev, &mic_buffer, &size);
        if (err == 0) {
            // Process Stereo -> Mono
            // We reuse the first half of the buffer for the mono output to save memory,
            // or we could use a separate buffer.
            // le_audio_send copies data into its own pool, so we can use a temporary buffer
            // or just overwrite the mic_buffer since the input is interleaved.
            // But we need to be careful not to overwrite data we haven't read yet if we do it in-place?
            // In-place Stereo -> Mono:
            // Input: L0 R0 L1 R1 ...
            // Output: M0 M1 ...
            // M0 writes to index 0. Input L0 is index 0. Safe.
            // M1 writes to index 1. Input L1 is index 2. Safe.
            // So in-place processing is safe.

            int16_t *samples = (int16_t *)mic_buffer;
            size_t sample_count = size / (sizeof(int16_t) * 2); // 320 / 2 = 160 samples (frames)

            audio_proc_process(samples, samples, sample_count);

            // Note: Sending raw PCM. For real BAP compliance, this should be LC3 encoded.
            // Send mono data (sample_count * sizeof(int16_t) bytes)
            le_audio_send((uint8_t *)samples, sample_count * sizeof(int16_t));

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
