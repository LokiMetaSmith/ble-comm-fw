#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main);

#define DEVICE_NAME_PREFIX "ComBadge-"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME_PREFIX) + 4)

static struct k_work_delayable blinky_work;

/*
 * This is a placeholder for the actual I2S and Touch button configuration.
 * In a real application, you would define the device tree nodes and aliases.
 */

static void get_random_name(char *name, size_t len)
{
    uint16_t rand_val;
    sys_rand_get(&rand_val, sizeof(rand_val));
    snprintf(name, len, "%s%04X", DEVICE_NAME_PREFIX, rand_val);
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // Full name will be updated dynamically
};

void main(void)
{
    int err;
    char device_name[DEVICE_NAME_LEN + 1];

    LOG_INF("Starting Star Trek Com Badge Firmware");

    // Initialize Random Bluetooth Name
    get_random_name(device_name, sizeof(device_name));
    LOG_INF("Generated Device Name: %s", device_name);

    // Set the device name
    err = bt_set_name(device_name);
    if (err) {
        LOG_ERR("Failed to set device name (err %d)", err);
        return;
    }

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    // Start Advertising
    // Note: In a real app, we would dynamically update the AD data with the new name
    // For now we just start with basic advertising
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");

    // Placeholder for I2S initialization
    // const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
    // ...

    // Placeholder for Touch Button
    // ...

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
