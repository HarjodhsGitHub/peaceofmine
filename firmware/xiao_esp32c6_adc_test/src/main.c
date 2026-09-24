// XIAO ESP32-C6 sensor ADC test
//
// Captures one ADC1 input continuously with DMA. It intentionally prints only
// one summary per second: printing every 100 kSPS sample would overflow USB
// serial and make the measurement unreliable.

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#ifndef ADC_INPUT_GPIO
#define ADC_INPUT_GPIO 0  // XIAO D0
#endif

#ifndef ADC_SAMPLE_RATE_HZ
#define ADC_SAMPLE_RATE_HZ 80000
#endif

#if ADC_SAMPLE_RATE_HZ > SOC_ADC_SAMPLE_FREQ_THRES_HIGH
#error "ADC_SAMPLE_RATE_HZ exceeds this ESP-IDF driver's supported continuous ADC rate"
#endif

#ifndef WAVEFORM_REPORT_INTERVAL_MS
#define WAVEFORM_REPORT_INTERVAL_MS 5000
#endif

#ifndef WAVEFORM_SAMPLE_COUNT
#define WAVEFORM_SAMPLE_COUNT 256
#endif

#ifndef PRINT_WAVEFORM_CSV
#define PRINT_WAVEFORM_CSV 0
#endif

#define ADC_READ_BUFFER_BYTES 512
#define STATS_WINDOW_US 1000000
#define WAVEFORM_REPORT_INTERVAL_US ((int64_t)WAVEFORM_REPORT_INTERVAL_MS * 1000)
#define ADC_FULL_SCALE_CODE 4095.0
#define ADC_PIN_FULL_SCALE_MV 3300.0
#define CLIP_LOW_CODE 20
#define CLIP_HIGH_CODE 4075

static const char *TAG = "sensor_adc";
static adc_cali_handle_t adc_calibration_handle = NULL;

typedef struct {
    uint64_t count;
    uint64_t sum;
    uint64_t sum_of_squares;
    uint32_t low_clip_count;
    uint32_t high_clip_count;
    uint16_t minimum;
    uint16_t maximum;
} sample_stats_t;

static void reset_stats(sample_stats_t *stats)
{
    *stats = (sample_stats_t){
        .minimum = UINT16_MAX,
        .maximum = 0,
    };
}

static void add_sample(sample_stats_t *stats, uint16_t raw)
{
    stats->count++;
    stats->sum += raw;
    stats->sum_of_squares += (uint32_t)raw * raw;

    if (raw < stats->minimum) {
        stats->minimum = raw;
    }
    if (raw > stats->maximum) {
        stats->maximum = raw;
    }
    if (raw <= CLIP_LOW_CODE) {
        stats->low_clip_count++;
    }
    if (raw >= CLIP_HIGH_CODE) {
        stats->high_clip_count++;
    }
}

static double code_to_pin_millivolts(double raw_code)
{
    const int raw_integer = (int)lround(fmax(0.0, fmin(ADC_FULL_SCALE_CODE, raw_code)));
    int calibrated_millivolts = 0;
    if (adc_calibration_handle &&
        adc_cali_raw_to_voltage(adc_calibration_handle, raw_integer, &calibrated_millivolts) == ESP_OK) {
        return calibrated_millivolts;
    }

    // This is only a fallback if this particular board does not have usable
    // calibration data programmed in its eFuse.
    return raw_code * ADC_PIN_FULL_SCALE_MV / ADC_FULL_SCALE_CODE;
}

static void enable_adc_voltage_calibration(adc_unit_t adc_unit, adc_channel_t adc_channel)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = adc_unit,
        .chan = adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    const esp_err_t calibration_status = adc_cali_create_scheme_curve_fitting(
        &calibration_config, &adc_calibration_handle);
    if (calibration_status == ESP_OK) {
        ESP_LOGI(TAG, "ADC voltage calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s); using approximate millivolts",
                 esp_err_to_name(calibration_status));
    }
#else
    ESP_LOGW(TAG, "This ESP-IDF build has no ADC calibration scheme; using approximate millivolts");
#endif
}

static void print_stats(const sample_stats_t *stats, int64_t elapsed_us)
{
    if (stats->count == 0) {
        ESP_LOGW(TAG, "No samples arrived in this reporting window");
        return;
    }

    const double mean = (double)stats->sum / stats->count;
    const double mean_square = (double)stats->sum_of_squares / stats->count;
    const double variance = fmax(0.0, mean_square - (mean * mean));
    const double ac_rms = sqrt(variance);
    const uint16_t peak_to_peak = stats->maximum - stats->minimum;
    const double amplitude = peak_to_peak / 2.0;
    const double sample_rate = stats->count * 1000000.0 / elapsed_us;

    // One compact, human-readable serial line each second. This avoids
    // flooding the Serial Monitor while the input circuit is still being built.
    printf("ADC | rate %.0f S/s | raw %u to %u | p2p %u | amplitude %.1f | "
           "mean %.1f | ac_rms %.1f | pin %s min %.0f mV max %.0f mV mean %.0f mV | "
           "clips low=%" PRIu32 " high=%" PRIu32 "\n",
           sample_rate,
           stats->minimum,
           stats->maximum,
           peak_to_peak,
           amplitude,
           mean,
           ac_rms,
           adc_calibration_handle ? "calibrated" : "approx",
           code_to_pin_millivolts(stats->minimum),
           code_to_pin_millivolts(stats->maximum),
           code_to_pin_millivolts(mean),
           stats->low_clip_count,
           stats->high_clip_count);
    fflush(stdout);
}

static void print_waveform(const uint16_t *waveform, uint32_t sample_count)
{
    // This is deliberately a short capture. A USB serial monitor cannot carry
    // a continuous 100 kSPS stream, so the ADC is paused while this CSV block
    // is printed and is restarted immediately afterwards.
    printf("# waveform_begin,sample_rate_hz=%d,gpio=%d,samples=%" PRIu32 "\n",
           ADC_SAMPLE_RATE_HZ, ADC_INPUT_GPIO, sample_count);
    printf("waveform,index,raw_code,pin_est_mV\n");
    for (uint32_t i = 0; i < sample_count; i++) {
        printf("waveform,%" PRIu32 ",%u,%.1f\n",
               i, waveform[i], code_to_pin_millivolts(waveform[i]));
    }
    printf("# waveform_end\n");
    fflush(stdout);
}

static bool estimate_frequency_hz(const uint16_t *waveform,
                                  uint32_t sample_count,
                                  double sample_rate_hz,
                                  double *frequency_hz)
{
    // Find the waveform's midpoint, then measure the spacing of successive
    // rising midpoint crossings. This works well for the expected 6.71 kHz
    // near-sine signal and needs no known DC offset.
    if (sample_count < 4 || sample_rate_hz <= 0.0) {
        return false;
    }

    uint16_t minimum = UINT16_MAX;
    uint16_t maximum = 0;
    for (uint32_t i = 0; i < sample_count; i++) {
        if (waveform[i] < minimum) {
            minimum = waveform[i];
        }
        if (waveform[i] > maximum) {
            maximum = waveform[i];
        }
    }

    // A tiny signal is likely noise, so it has no trustworthy frequency.
    if (maximum - minimum < 20) {
        return false;
    }

    const double midpoint = ((double)minimum + maximum) / 2.0;
    uint32_t rising_crossings = 0;
    double first_crossing = 0.0;
    double last_crossing = 0.0;

    for (uint32_t i = 1; i < sample_count; i++) {
        const double previous = waveform[i - 1];
        const double current = waveform[i];
        if (previous < midpoint && current >= midpoint && current > previous) {
            // Fractional position gives a more accurate result than rounding
            // each crossing to a whole 80 kS/s sample.
            const double fraction = (midpoint - previous) / (current - previous);
            const double crossing = (i - 1) + fraction;
            if (rising_crossings == 0) {
                first_crossing = crossing;
            }
            last_crossing = crossing;
            rising_crossings++;
        }
    }

    if (rising_crossings < 2 || last_crossing <= first_crossing) {
        return false;
    }

    *frequency_hz = (rising_crossings - 1) * sample_rate_hz / (last_crossing - first_crossing);
    return true;
}

static void print_frequency(const uint16_t *waveform,
                            uint32_t sample_count,
                            double sample_rate_hz)
{
    double frequency_hz = 0.0;
    if (estimate_frequency_hz(waveform, sample_count, sample_rate_hz, &frequency_hz)) {
        printf("Frequency | %.1f Hz | measured from %" PRIu32 " samples at %.0f S/s\n",
               frequency_hz, sample_count, sample_rate_hz);
    } else {
        printf("Frequency | not available yet (need a stable waveform)\n");
    }
    fflush(stdout);
}

void app_main(void)
{
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;

    ESP_ERROR_CHECK(adc_continuous_io_to_channel(ADC_INPUT_GPIO, &adc_unit, &adc_channel));
    if (adc_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO %d is not an ADC1 pin. Use XIAO D0/GPIO0, D1/GPIO1, or D2/GPIO2.",
                 ADC_INPUT_GPIO);
        return;
    }

    ESP_LOGI(TAG, "Starting ADC1 capture on GPIO %d at %d S/s", ADC_INPUT_GPIO, ADC_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Keep the buffer output between GND and 3.3 V. Turn radios off while testing.");
    enable_adc_voltage_calibration(adc_unit, adc_channel);

    adc_continuous_handle_t adc_handle = NULL;
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = ADC_READ_BUFFER_BYTES,
        .flags = {
            .flush_pool = true,
        },
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = adc_channel,
        .unit = adc_unit,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = ADC_SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint8_t dma_buffer[ADC_READ_BUFFER_BYTES];
    uint16_t waveform[WAVEFORM_SAMPLE_COUNT];
    uint32_t waveform_count = 0;
    sample_stats_t stats;
    reset_stats(&stats);
    int64_t window_started_us = esp_timer_get_time();
    int64_t last_waveform_report_us = window_started_us;

    while (true) {
        uint32_t bytes_read = 0;
        const esp_err_t read_status = adc_continuous_read(
            adc_handle, dma_buffer, sizeof(dma_buffer), &bytes_read, 1000);

        if (read_status == ESP_OK && bytes_read > 0) {
            for (uint32_t offset = 0;
                 offset + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read;
                 offset += SOC_ADC_DIGI_RESULT_BYTES) {
                // ESP-IDF 5.4 exposes each C6 DMA result directly as a
                // type-2 ADC word. Copy it first so this remains safe even
                // if the DMA buffer itself is not naturally aligned.
                adc_digi_output_data_t sample;
                memcpy(&sample, &dma_buffer[offset], sizeof(sample));

                if (sample.type2.channel < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1) &&
                    sample.type2.channel == adc_channel) {
                    const uint16_t raw = sample.type2.data;
                    add_sample(&stats, raw);
                    if (waveform_count < WAVEFORM_SAMPLE_COUNT) {
                        waveform[waveform_count++] = raw;
                    }
                }
            }
        } else if (read_status != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC read: %s", esp_err_to_name(read_status));
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - window_started_us >= STATS_WINDOW_US) {
            const int64_t stats_elapsed_us = now_us - window_started_us;
            const double observed_sample_rate_hz =
                stats.count * 1000000.0 / stats_elapsed_us;
            print_stats(&stats, stats_elapsed_us);

            if (now_us - last_waveform_report_us >= WAVEFORM_REPORT_INTERVAL_US) {
                print_frequency(waveform, waveform_count, observed_sample_rate_hz);

                if (PRINT_WAVEFORM_CSV) {
                    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
                    print_waveform(waveform, waveform_count);
                    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
                }

                waveform_count = 0;
                last_waveform_report_us = now_us;
            }

            reset_stats(&stats);
            window_started_us = esp_timer_get_time();
        }
    }
}
