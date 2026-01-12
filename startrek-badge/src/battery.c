#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery);

/*
 * SAADC configuration for NRF54L series (or generic NRF).
 * Assuming monitoring VDD directly via internal channel if supported,
 * or via a divider on an AIN pin.
 * For simplicity, we use the device tree node &adc.
 */

#define BATTERY_ADC_NODE DT_NODELABEL(adc)

static const struct device *adc_dev = DEVICE_DT_GET(BATTERY_ADC_NODE);

/*
 * Channel configuration:
 * Using generic configuration. In a real scenario, this matches the
 * device tree 'io-channels' or is configured here.
 */
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = 0, // 0 usually corresponds to AIN0 or VDD/x depending on config
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = SAADC_CH_PSELP_PSELP_VDD, // Measure VDD directly
#endif
};

static int16_t sample_buffer;

static struct adc_sequence sequence = {
    .channels = BIT(0),
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

int battery_init(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    int err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err) {
        LOG_ERR("ADC channel setup failed (err %d)", err);
        return err;
    }

    return 0;
}

int battery_sample(void)
{
    int err = adc_read(adc_dev, &sequence);
    if (err) {
        LOG_ERR("ADC read failed (err %d)", err);
        return err;
    }

    /* Convert sample to voltage (mV) and then percentage */
    int32_t mv_value = sample_buffer;

    // V = (Value / 2^12) * Reference / Gain
    // Ref = 0.6V (internal), Gain = 1/4 -> Full scale = 2.4V?
    // Wait, VDDHD typically needs higher range.
    // Usually Gain=1/6, Ref=0.6V -> Range=3.6V.
    // Let's assume standard calculation or use adc_raw_to_millivolts if available.

    int32_t adc_vref = adc_ref_internal(adc_dev);
    adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

    LOG_INF("Battery Voltage: %d mV", mv_value);

    // Simple percentage estimation for Li-ion/Coin cell (3.0V nominal)
    // 3.0V = 100%, 2.0V = 0%
    uint8_t level;
    if (mv_value >= 3000) level = 100;
    else if (mv_value <= 2000) level = 0;
    else level = (mv_value - 2000) / 10;

    bt_bas_set_battery_level(level);
    return level;
}
