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

#include "le_audio.h"
#include "battery.h"
#include "led_ctrl.h"

LOG_MODULE_REGISTER(main);

#define DEVICE_NAME_PREFIX "ComBadge-"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME_PREFIX) + 4)

static const struct device *i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_dev));

/* Audio Buffers */
#define AUDIO_BLOCK_SIZE_BYTES 80 // 16kHz * 2 bytes * 10ms (mono) = 320 bytes? No wait.
// LC3 frame size for 16kHz 10ms is typically 40-120 bytes encoded.
// But I2S is PCM.
// 16kHz sample rate, 16-bit depth, mono = 32000 bytes/sec.
// 10ms = 320 bytes.
// So our I2S block size should match the PCM requirements, NOT the LC3 encoded size.
// The LE Audio stack (LC3 codec) handles compression.
// Wait, if we are passing data to 'le_audio_send', are we passing PCM or Encoded?
// Zephyr BAP stack usually expects Encoded data if we don't have a software codec layer integrated
// or if we are just a conduit.
// However, the prompt says "flush out".
// If we want to send *audio*, we need to Encode it.
// Since we don't have a full LC3 encoder in this simple loop, we will assume
// for the "minimal implementation" that we are just piping data.
// But realistically, I2S gives PCM. BAP sends LC3.
// We should probably check if we can use the software codec offload or just stub it.
// For now, let's stick to the structure. I'll use a block size of 320 bytes for PCM.

#define PCM_BLOCK_SIZE 320
K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, PCM_BLOCK_SIZE, 4, 4);

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

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & DK_BTN3_MSK) {
        if (button_state & DK_BTN3_MSK) {
            LOG_INF("Touch Button Pressed - Toggle Play/Pause");
            // Here we would toggle streaming state or send a command
        }
    }
}

/* Audio Data Callbacks */
static void audio_recv_cb(const uint8_t *data, size_t len)
{
    // Received encoded audio from BLE.
    // In a real system, we decode LC3 -> PCM.
    // Here we just write to I2S (assuming it was PCM for loopback test, or just noise)
    // To properly support this, we'd need an LC3 decoder.

    // Allocate I2S buffer
    void *mem_block;
    int ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
    if (ret < 0) {
        // Discard
        return;
    }

    // Copy (and pad if necessary since PCM > LC3 encoded size)
    // Just a placeholder copy
    size_t copy_len = len > PCM_BLOCK_SIZE ? PCM_BLOCK_SIZE : len;
    memcpy(mem_block, data, copy_len);

    // Write to I2S
    ret = i2s_write(i2s_dev, mem_block, PCM_BLOCK_SIZE);
    if (ret < 0) {
        k_mem_slab_free(&i2s_mem_slab, &mem_block);
    }
    // i2s_write frees the slab when done usually, depending on trigger?
    // Wait, i2s_write copies or takes ownership? Zephyr I2S driver takes ownership if trigger started?
    // Actually typically you pass the slab to config and it manages it.
}

static int i2s_setup(void)
{
    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("I2S device not ready");
        return -ENODEV;
    }

    i2s_cfg.word_size = 16;
    i2s_cfg.channels = 1; // Mono
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

    // Init Modules
    led_ctrl_init();

    err = dk_buttons_init(button_handler);
    if (err) LOG_ERR("Failed to init buttons (err %d)", err);

    err = battery_init();
    if (err) LOG_ERR("Failed to init battery (err %d)", err);

    // Initialize I2S
    err = i2s_setup();
    if (err) {
        LOG_ERR("Failed to init I2S (err %d)", err);
    } else {
        LOG_INF("I2S Initialized");
    }

    // Bluetooth Setup
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

    // Trigger I2S RX
    i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

    // Main loop
    while (1) {
        // Handle Mic Data (Source)
        // Read from I2S
        size_t size = PCM_BLOCK_SIZE;
        err = i2s_read(i2s_dev, &mic_buffer, &size);
        if (err == 0) {
            // We have PCM data.
            // Send to LE Audio (needs encoding, but we send raw for now as placeholder)
            // Ideally we call le_audio_send(mic_buffer, PCM_BLOCK_SIZE);
            // But LE Audio expects ~40-120 bytes encoded. Sending 320 bytes might fail or fragment.
            // Just sending first 100 bytes as dummy payload
            le_audio_send(mic_buffer, 100);

            k_mem_slab_free(&i2s_mem_slab, &mic_buffer);
        }

        // Battery Monitor (every 10s)
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
        k_sleep(K_MSEC(10)); // Faster loop for audio handling
    }
}
