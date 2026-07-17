/*
 * F1 — Basic motor spin (no PID yet)
 * Edge AMR / SLAM robot — Week 2, Phase 2 (Firmware)
 *
 * Goal: drive both TT motors forward at a fixed 50% PWM to prove the
 * ESP32 -> TB6612FNG -> motor path works end to end. The encoder and
 * PID come later (F2, F3) — right now we just want movement.
 *
 * Pin map (ESP32 38-pin DevKit  ->  TB6612FNG):
 *   GPIO16 -> PWMA      GPIO17 -> PWMB     (speed  — driven by LEDC)
 *   GPIO18 -> AIN1      GPIO21 -> BIN1     (direction)
 *   GPIO19 -> AIN2      GPIO22 -> BIN2     (direction)
 *   GPIO23 -> STBY                         (enable, HIGH = run)
 *   3V3    -> VCC   (logic power)
 *   5V     -> VM    (motor power — temporary: ESP32 5V passthrough)
 *   GND    -> GND   (must be common with the powerbank ground)
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

/* --- Direction + enable pins: plain digital outputs (HIGH/LOW only) --- */
#define AIN1_PIN  GPIO_NUM_18
#define AIN2_PIN  GPIO_NUM_19
#define BIN1_PIN  GPIO_NUM_21
#define BIN2_PIN  GPIO_NUM_22
#define STBY_PIN  GPIO_NUM_23

/* --- Speed pins: driven by the LEDC PWM peripheral, not set by hand --- */
#define PWMA_PIN  GPIO_NUM_16
#define PWMB_PIN  GPIO_NUM_17

#define PWM_FREQ_HZ  1000   /* 1 kHz PWM carrier */
#define PWM_DUTY_50  128    /* 128 / 255 = 50% duty (8-bit resolution) */

/* Configure the five direction/enable pins as digital outputs. */
static void setup_gpio(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                        (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN) |
                        (1ULL << STBY_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* Configure the LEDC hardware to generate PWM on PWMA and PWMB.
 * LEDC does the fast HIGH/LOW switching in hardware, so app_main never
 * has to toggle the pin itself. */
static void setup_pwm(void) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,   /* duty range 0..255 */
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch_a = {
        .gpio_num   = PWMA_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = PWM_DUTY_50,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_a));

    ledc_channel_config_t ch_b = {
        .gpio_num   = PWMB_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = PWM_DUTY_50,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_b));
}

void app_main(void) {
    setup_gpio();
    setup_pwm();

    gpio_set_level(STBY_PIN, 1);   /* wake the TB6612FNG out of standby */

    /* Motor A forward: AIN1 = HIGH, AIN2 = LOW */
    gpio_set_level(AIN1_PIN, 1);
    gpio_set_level(AIN2_PIN, 0);

    /* Motor B forward: BIN1 = HIGH, BIN2 = LOW */
    gpio_set_level(BIN1_PIN, 1);
    gpio_set_level(BIN2_PIN, 0);

    /* app_main returns here; the GPIO levels and LEDC PWM keep running. */
}
