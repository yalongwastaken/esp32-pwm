// Author: Anthony Yalong
// Description:

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>

// LED configuration
#define RED_LED GPIO_NUM_25
#define RED_CHANNEL LEDC_CHANNEL_0
#define GREEN_LED GPIO_NUM_26
#define GREEN_CHANNEL LEDC_CHANNEL_1
#define BLUE_LED GPIO_NUM_27
#define BLUE_CHANNEL LEDC_CHANNEL_2

// ADC configuration

// PWM/Timer configuration
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_FREQ 1000
#define PWM_RES LEDC_TIMER_11_BIT
#define PWM_CLK_CFG LEDC_AUTO_CLK

// Logging configuratoin
static const char *TAG = "PWM_MULTI-CHANNEL (ESP-IDF)";

// Error queue configuration
static QueueHandle_t err_queue;
#define ERR_QUEUE_SIZE 10
#define ERR_QUEUE_ITEM esp_err_t

// Task queue configuraiton
static QueueHandle_t task_queue;

// Function prototypes
static void halt_program(void);
static void queue_error(const char *source, esp_err_t err);
static void init_pwm(void);
static void init_adc(void);
static void err_queue_task(void *pvParameters);

void app_main() {
    // Initialize error queue
    err_queue = xQueueCreate(ERR_QUEUE_SIZE, sizeof(ERR_QUEUE_ITEM));
    if (err_queue == NULL) halt_program();
}

static void halt_program(void) {
    ESP_LOGw(TAG, "Halting program.");

    while (1) {
        ESP_LOGW(TAG, "System Error! Please update source code & reflash.");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void queue_error(const char *source, esp_err_t err) {
    if (err == ESP_OK) return;

    BaseType_t queue_send_status = pdFALSE;
    int32_t queue_send_retry_counter = 10;

    while (queue_send_status != pdTRUE && queue_send_retry_counter >= 0) {
        ESP_LOGE(TAG, "Error in %s: %s", source, esp_err_to_name(err));
        queue_send_status = (err_queue, &err, 0);
        if (queue_send_status != pdTRUE) {
            ESP_LOGW(TAG, "Queue send failed. Re-attempting %ld times.", queue_send_retry_counter);
            queue_send_retry_counter--;
        }
    }

    if (queue_send_status != pdTRUE) {
        ESP_LOGW(TAG, "Queue send failed");
    }
}

static void init_pwm(void) {
    // Timer config
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .duty_resolution = PWM_RES,
        .clk_cfg = PWM_CLK_CFG,
    };
    queue_error("ledc_timer_config", ledc_timer_config(&timer_config));

    // Red channel config
    ledc_channel_config_t red_channel_config = {
        .channel = RED_CHANNEL,
        .gpio_num = RED_LED,
        .timer_sel = PWM_TIMER,
        .speed_mode = PWM_SPEED_MODE,
        .duty = 0,
        .hpoint = 0,
    };
    queue_error("ledc_channel_config", ledc_channel_config(&red_channel_config));

    // Green channel config
    ledc_channel_config_t green_channel_config = {
        .channel = GREEN_CHANNEL,
        .gpio_num = GREEN_LED,
        .timer_sel = PWM_TIMER,
        .speed_mode = PWM_SPEED_MODE,
        .duty = 0,
        .hpoint = 0,
    };
    queue_error("ledc_channel_config", ledc_channel_config(&green_channel_config));

    // Blue channel config
    ledc_channel_config_t blue_channel_config = {
        .channel = BLUE_CHANNEL,
        .gpio_num = BLUE_LED,
        .timer_sel = PWM_TIMER,
        .speed_mode = PWM_SPEED_MODE,
        .duty = 0,
        .hpoint = 0,
    };
    queue_error("ledc_channel_config", ledc_channel_config(&blue_channel_config));
}

static void init_adc(void) {
}

static void err_queue_task(void *pvParameters) {
    while (1) {
        esp_err_t err;

        if (xQueueReceive(err_queue, &err, portMAX_DELAY) == pdTRUE) {
            halt_program();
        }
    }
}