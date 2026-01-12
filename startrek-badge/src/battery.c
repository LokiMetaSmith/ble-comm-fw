#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery);

#define BATTERY_ADC_NODE DT_NODELABEL(adc)

static const struct device *adc_dev = DEVICE_DT_GET(BATTERY_ADC_NODE);

#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_4 // VDD/4
#define ADC_REFERENCE ADC_REF_INTERNAL // 0.6V

// Full scale voltage = 0.6V / (1/4) = 2.4V? No.
// If Gain is 1/4, Input = V * 1/4.
// (V * 1/4) < 0.6V => V < 2.4V.
// Wait, coin cell is 3.0V. We need a different gain or divider.
// NRF54L SAADC might support 1/6 gain.
// 0.6 / (1/6) = 3.6V. This is suitable for 3V battery.
#define ADC_GAIN_SAFE ADC_GAIN_1_6

static struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN_SAFE,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = SAADC_CH_PSELP_PSELP_VDD,
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
        return 0;
    }

    int32_t mv_value = sample_buffer;
    int32_t adc_vref = adc_ref_internal(adc_dev);

    // Convert raw to mV
    // Result = (Value / 2^12) * Ref / Gain
    adc_raw_to_millivolts(adc_vref, ADC_GAIN_SAFE, ADC_RESOLUTION, &mv_value);

    LOG_INF("Battery Voltage: %d mV", mv_value);

    /*
     * CR2032 Discharge Curve Approximation (Non-linear)
     * > 3000mV : 100%
     * 2900mV : 80%
     * 2800mV : 60%
     * 2700mV : 40%
     * 2600mV : 30%
     * 2500mV : 20%
     * 2400mV : 10%
     * < 2000mV : 0%
     */
    uint8_t level;
    if (mv_value >= 3000) level = 100;
    else if (mv_value >= 2900) level = 80 + (mv_value - 2900) * 20 / 100;
    else if (mv_value >= 2800) level = 60 + (mv_value - 2800) * 20 / 100;
    else if (mv_value >= 2700) level = 40 + (mv_value - 2700) * 20 / 100;
    else if (mv_value >= 2600) level = 30 + (mv_value - 2600) * 10 / 100;
    else if (mv_value >= 2500) level = 20 + (mv_value - 2500) * 10 / 100;
    else if (mv_value >= 2400) level = 10 + (mv_value - 2400) * 10 / 100;
    else level = 0;

    bt_bas_set_battery_level(level);
    return level;
}
