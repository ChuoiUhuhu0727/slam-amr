/*
 * F1 — Basic motor spin, F2 — encoder RPM read
 * Edge AMR / SLAM robot — Week 2, Phase 2 (Firmware)
 *
 * F1: drive both TT motors forward at a fixed 50% PWM to prove the
 * ESP32 -> TB6612FNG -> motor path works end to end.
 * F2: count LM393 encoder pulses via ISR, compute RPM every 1s.
 * Merged into one project because F3 (PID) needs both together anyway.
 *
 * Pin map (ESP32 38-pin DevKit  ->  TB6612FNG):
 *   GPIO16 -> PWMA      GPIO17 -> PWMB     (speed  — driven by LEDC)
 *   GPIO18 -> AIN1      GPIO21 -> BIN1     (direction)
 *   GPIO19 -> AIN2      GPIO22 -> BIN2     (direction)
 *   GPIO23 -> STBY                         (enable, HIGH = run)
 *   3V3    -> VCC   (logic power)
 *   5V     -> VM    (motor power — temporary: ESP32 5V passthrough)
 *   GND    -> GND   (must be common with the powerbank ground)
 *
 * Pin map (ESP32 -> LM393 encoders):
 *   GPIO34 -> left encoder OUT   (input-only pin, no internal pull-up needed —
 *   GPIO35 -> right encoder OUT   LM393 module has its own onboard pull-up)
 *
 * F2 test procedure (do this BEFORE trusting F3/PID):
 *   Comment out the "wake the TB6612FNG" + direction lines in app_main so
 *   the motors stay off, flash, then spin each wheel by hand and confirm
 *   the RPM printed on the serial monitor looks sane. Only once that's
 *   verified, uncomment and let both F1 (motor) and F2 (encoder) run together.
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

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

/* --- F2: encoder pins + RPM constants --- */
#define ENC_L_PIN GPIO_NUM_34   /* left encoder signal */
#define ENC_R_PIN GPIO_NUM_35   /* right encoder signal */
#define SLOTS_PER_REV 20        /* LM393 disk: 20 slots = 1 full wheel revolution */

/* --- F3: P-only velocity control (Ki/Kd come later, tune Kp first) --- */
#define TARGET_RPM 30.0f   /* hardcoded target for now; /cmd_vel comes in F5 */
#define KP 3.0f            /* start small, increase until oscillation appears, back off */

static const char *TAG = "encoder_task";

/* Shared between ISR (writer) and encoder_task (reader) -> must be volatile
 * so the compiler never caches a stale copy in a register. */
static volatile uint32_t pulse_count_left = 0;
static volatile uint32_t pulse_count_right = 0;

/* Latest computed RPM, exposed for pid_task (F3) to read later. */
static volatile float rpm_left_shared = 0.0f;
static volatile float rpm_right_shared = 0.0f;

/* Spinlock used to make "read + reset" atomic (avoids the lost-pulse race
 * where a pulse arrives between reading the counter and zeroing it). */
static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

/* One ISR handles both pins; `arg` tells it which counter to bump.
 * IRAM_ATTR: ISR code must live in IRAM so it still runs even while
 * flash cache is temporarily disabled (standard ESP-IDF requirement). */
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    volatile uint32_t *counter = (volatile uint32_t *)arg;
    (*counter)++;   /* ONLY this. No math, no logging, no delays in an ISR. */
}

static void encoder_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENC_L_PIN) | (1ULL << ENC_R_PIN),
        .mode = GPIO_MODE_INPUT,
        /* GPIO34/35 are input-only pins anyway (can't drive internal
         * pull-up/down), and the LM393 module already has its own. */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,   /* rising edge only */
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_L_PIN, encoder_isr_handler, (void *)&pulse_count_left);
    gpio_isr_handler_add(ENC_R_PIN, encoder_isr_handler, (void *)&pulse_count_right);
}

static void encoder_task(void *arg) {
    encoder_gpio_init();

    uint32_t snapshot_left, snapshot_right;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    while (1) {
        vTaskDelayUntil(&last_wake, period);   /* fixed absolute cadence, no drift */

        portENTER_CRITICAL(&encoder_mux);
        snapshot_left = pulse_count_left;
        pulse_count_left = 0;
        snapshot_right = pulse_count_right;
        pulse_count_right = 0;
        portEXIT_CRITICAL(&encoder_mux);

        rpm_left_shared  = (snapshot_left  / (float)SLOTS_PER_REV) * 60.0f;
        rpm_right_shared = (snapshot_right / (float)SLOTS_PER_REV) * 60.0f;

        ESP_LOGI(TAG, "RPM L=%.1f  R=%.1f", rpm_left_shared, rpm_right_shared);
    }
}

/* error(t) = target - actual; u(t) = Kp * error(t); PWM = clamp(u(t), 0, 255).
 * P-only for now — Ki/Kd are Week 2 stretch, not required to move to F4. */
static uint32_t pid_step(float target_rpm, float actual_rpm) {
    float error = target_rpm - actual_rpm;
    float u = KP * error;

    if (u < 0.0f)   u = 0.0f;    /* clamp: can't spin backward with AIN/BIN fixed forward */
    if (u > 255.0f) u = 255.0f;  /* clamp: LEDC 8-bit duty tops out at 255 */

    return (uint32_t)u;
}

static void pid_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);   /* 100 Hz = 10 ms period */

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        uint32_t pwm_left  = pid_step(TARGET_RPM, rpm_left_shared);
        uint32_t pwm_right = pid_step(TARGET_RPM, rpm_right_shared);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pwm_left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, pwm_right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

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

    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 5, NULL);
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 5, NULL);

    /* app_main returns here; the GPIO levels, LEDC PWM, encoder_task, and
     * pid_task keep running. */
}
