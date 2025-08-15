// Author: Anthony Yalong
// Description: Reads analog potentiometer value and controls LED brightness via PWM.
//              Updates every 50ms with serial monitoring for real-time feedback.

#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <esp_log.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// GPIO Configuration
#define LED_NUM GPIO_NUM_2

// PWM Configuration
#define PWM_TIMER LEDC_TIMER_0
#define PWM_TIMER_CONFIG LEDC_AUTO_CLK
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define PWM_FREQ 1000
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0

// ADC Configuration
#define ADC_CHANNEL ADC_CHANNEL_0       // GPIO36
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_0

// Function Prototypes
static esp_err_t init_gpio(void);
static esp_err_t init_adc(void);
static esp_err_t init_pwm(void);
static esp_err_t set_duty(uint32_t duty);
static int read_raw_adc(void);
static esp_err_t map_value(uint32_t value, uint32_t in_min, uint32_t in_max, 
                            uint32_t out_min, uint32_t out_max, uint32_t *out);
static esp_err_t main_task(void *pvParameters);

#endif