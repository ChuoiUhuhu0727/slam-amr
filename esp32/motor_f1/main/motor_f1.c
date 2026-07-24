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
 *   VM     <- powerbank (direct, bypasses ESP32 — fixed 2026-07-24)
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

static const char *TAG = "control_task";

/* Shared between ISR (writer) and control_task (reader) -> must be volatile
 * so the compiler never caches a stale copy in a register. */
static volatile uint32_t pulse_count_left = 0;
static volatile uint32_t pulse_count_right = 0;

/* Latest computed RPM, exposed for uros_task (F5) to read later. */
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

/* VM now has its own wire from the powerbank (fixed 2026-07-24, see README
 * "Power Architecture") — the brownout reset loop this cap was guarding
 * against is confirmed gone. Left at 100 for now since Kp hasn't been
 * retuned with valid encoder data yet; raise deliberately, not by default,
 * once real RPM tracking data says the controller needs more headroom. */
#define MAX_SAFE_PWM 100.0f

/* error(t) = target - actual; u(t) = Kp * error(t); PWM = clamp(u(t), 0, MAX_SAFE_PWM).
 * P-only for now — Ki/Kd are Week 2 stretch, not required to move to F4. */
static uint32_t pid_step(float target_rpm, float actual_rpm) {
    float error = target_rpm - actual_rpm;
    float u = KP * error;

    if (u < 0.0f)           u = 0.0f;           /* clamp: can't spin backward with AIN/BIN fixed forward */
    if (u > MAX_SAFE_PWM)   u = MAX_SAFE_PWM;   /* clamp: temporary current-safety ceiling, not the LEDC max */

    return (uint32_t)u;
}

/* Caps how much the applied PWM can change in one cycle, regardless of how
 * big a jump pid_step() wants. This limits di/dt on the shared power rail —
 * a sudden PWM jump stresses a marginal supply harder than the same
 * steady-state current reached gradually. Does not replace the VM rewire;
 * it reduces (not eliminates) brownout risk while that's still pending. */
#define MAX_PWM_STEP_PER_CYCLE 10.0f

static float slew_limit(float applied, float target) {
    if (target > applied + MAX_PWM_STEP_PER_CYCLE) return applied + MAX_PWM_STEP_PER_CYCLE;
    if (target < applied - MAX_PWM_STEP_PER_CYCLE) return applied - MAX_PWM_STEP_PER_CYCLE;
    return target;
}

/* Encoder read + PID + PWM write, all in ONE task/cycle — replaces the old
 * separate encoder_task (1 Hz) + pid_task (100 Hz). That split meant PID
 * recomputed 100x/sec against an RPM value that was up to 1 full second
 * stale. Running both halves in the same 20 Hz loop means PID always acts
 * on the reading from THIS cycle — zero cross-task staleness.
 *
 * Trade-off: 20 Hz (50 ms window) means coarser RPM resolution than the
 * old 1 Hz window (1 pulse = 60 RPM/step now, vs. 3 RPM/step before) —
 * faster feedback costs resolution with only 20 slots/rev. Accepted for now. */
#define CONTROL_PERIOD_MS 50   /* 20 Hz */

static void control_task(void *arg) {
    encoder_gpio_init();

    uint32_t snapshot_left, snapshot_right;
    float applied_pwm_left = 0.0f, applied_pwm_right = 0.0f;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        portENTER_CRITICAL(&encoder_mux);
        snapshot_left = pulse_count_left;
        pulse_count_left = 0;
        snapshot_right = pulse_count_right;
        pulse_count_right = 0;
        portEXIT_CRITICAL(&encoder_mux);

        rpm_left_shared  = (snapshot_left  / (float)SLOTS_PER_REV) * (60000.0f / CONTROL_PERIOD_MS);
        rpm_right_shared = (snapshot_right / (float)SLOTS_PER_REV) * (60000.0f / CONTROL_PERIOD_MS);

        float target_pwm_left  = (float)pid_step(TARGET_RPM, rpm_left_shared);
        float target_pwm_right = (float)pid_step(TARGET_RPM, rpm_right_shared);

        applied_pwm_left  = slew_limit(applied_pwm_left,  target_pwm_left);
        applied_pwm_right = slew_limit(applied_pwm_right, target_pwm_right);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)applied_pwm_left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)applied_pwm_right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        ESP_LOGI(TAG, "RPM L=%.1f R=%.1f  PWM L=%.0f R=%.0f",
                 rpm_left_shared, rpm_right_shared, applied_pwm_left, applied_pwm_right);
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

    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);

    /* app_main returns here; the GPIO levels, LEDC PWM, and control_task
     * keep running. */
}
