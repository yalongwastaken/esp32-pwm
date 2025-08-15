// Author: Anthony Yalong
// Description: Reads analog potentiometer value and controls LED brightness via PWM.
//              Updates every 50ms with serial monitoring for real-time feedback.

#include "pwm_control.h"

// Logging
const static char *TAG = "PWM_BASIC (ESP-IDF)";

// System Error Flag
static bool system_error = false;

void app_main() {}

static esp_err_t init_gpio(void) {
    // Pin validity check
    if (!GPIO_IS_VALID_OUTPUT_GPIO(LED_NUM)) {
        ESP_LOGE(TAG, "init_gpio: invalid LED gpio pin");
        return ESP_ERR_INVALID_ARG;
    }

    // LED configuration setup
    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1UL << LED_NUM),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    
    // Set LED configuration 
    esp_err_t ret = gpio_config(&led_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_gpio: failed to configure LED GPIO");
    }
    else {
        ESP_LOGI(TAG, "init_gpio: successfully initialized GPIO");
    }

    return ret;
}
static esp_err_t init_adc(void) {
    esp_err_t ret;

    ret = adc1_config_width(ADC_WIDTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_adc: failed to configure adc1's width");
        return ret;
    }

    ret = adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_adc: failed to configure adc1's channel attenuation");
        return ret;
    }

    ESP_LOGI(TAG, "init_adc: successfully initialized ADC");
    return ESP_OK;
}

static esp_err_t init_pwm(void) {
    esp_err_t ret;

    ledc_timer_config_t timer_config = {

    };
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_pwm: failed to initialize ledc timer");
        return ret;
    }

    ledc_channel_config_t channel_config = {
        
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init_pwm: failed to initialize ledc channel");
        return ret;
    }

    ESP_LOGI(TAG, "init_pwm: successfully initialized LEDC");
    return ESP_OK;
}

static esp_err_t set_duty(uint32_t duty) {
    // Clamp duty
    if (duty > ((1 << PWM_RESOLUTION) - 1)) {
        duty = (1 << PWM_RESOLUTION) - 1;
    }

    esp_err_t ret;

    ret = ledc_set_duty(PWM_SPEED_MODE, PWM_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_duty: failed to set duty");
        return ret;
    }

    ret = ledc_update_duty(PWM_SPEED_MODE, PWM_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_duty: failed to update duty");
        return ret;
    }

    ESP_LOGI(TAG, "set_duty: set & updated duty");
    return ESP_OK;
}

static int read_raw_adc(void) {
    return adc1_get_raw(ADC_CHANNEL);
}

static esp_err_t main_task(void *pvParameters);