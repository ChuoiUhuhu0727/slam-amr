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
#include "esp_timer.h"

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

/* --- F3: PI velocity control (Kd deliberately skipped, see KI comment below) --- */
/* Raised 30 -> 60 (2026-07-27): 30 forced PWM to sit at MIN_SAFE_PWM on the
 * right wheel (its natural RPM at the floor already exceeded target), and
 * even after fixing the earlier RPM-quantization bug, chasing 30 meant
 * probing PWM near the motor's weak/deadzone end - no margin for heavier
 * hardware later. 60 sits inside the PWM range (~20-40) both wheels already
 * hit comfortably today, without floor-clamping either side. Still a test
 * value for validating the control loop, NOT a calibrated real-world speed -
 * that needs wheel diameter + a chosen cm/s target, deferred to F4 odometry. */
#define TARGET_RPM 60.0f
#define KP 3.0f            /* start small, increase until oscillation appears, back off */

/* Added 2026-07-27: P-only data showed the right wheel settling ~15 RPM
 * below TARGET_RPM consistently (30-60 RPM band, target 60) - a textbook
 * P-only steady-state error, not noise (resolution is now 15 RPM/step,
 * fine enough to trust this as real). KI=0.2 is a conservative starting
 * guess, same "start small" approach as KP.
 * Kd deliberately NOT added: RPM is still a coarse step signal (15 RPM/step
 * @ 5Hz) - differentiating a stair-step amplifies jitter into PWM swings
 * ("derivative kick") rather than smoothing anything. PI is the standard
 * choice for velocity/RPM loops anyway; Kd matters more for position
 * control. Revisit only if PI alone proves insufficient. */
#define KI 0.2f

/* Anti-windup: without this, the integral term grows unbounded whenever
 * output is pinned at MIN/MAX_SAFE_PWM - exactly what happened during
 * today's left-wheel stall (PWM=100, RPM=0, for 10+ seconds straight).
 * An unclamped integral would have kept accumulating that whole time, then
 * caused a huge overshoot the moment the wheel freed up. Caps the integral
 * term's own contribution to output, independent of Kp's contribution. */
#define MAX_I_CONTRIBUTION 40.0f
#define MAX_INTEGRAL (MAX_I_CONTRIBUTION / KI)

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

/* Debounce: while the motor is actually running (not hand-spun), PWM
 * switching edges and DC-brush arcing couple electrical noise onto the
 * encoder signal line, which the ISR was counting as real slot transitions
 * (confirmed 2026-07-27: hand-spinning gave clean 0-60 RPM, motor-running
 * gave RPM spiking as high as 6300). Reject edges that arrive faster than
 * any real slot transition physically could - at TARGET_RPM=30 the real
 * interval is ~100ms, so there's large margin to push this threshold up.
 * 3ms wasn't enough (tested 2026-07-27: cut peak noise from RPM=6300 down
 * to RPM=540, but R still noisy vs. L's clean 0/60) - PWM runs at 1kHz
 * (1ms period), so switching noise can recur faster than a 3ms window
 * rejects. Raised to 15ms, still >>3x below the real ~50ms interval. */
#define MIN_PULSE_INTERVAL_US 15000

static volatile int64_t last_edge_us_left = 0;
static volatile int64_t last_edge_us_right = 0;

typedef struct {
    volatile uint32_t *counter;
    volatile int64_t *last_edge_us;
} encoder_isr_arg_t;

static encoder_isr_arg_t enc_left_arg  = { &pulse_count_left,  &last_edge_us_left };
static encoder_isr_arg_t enc_right_arg = { &pulse_count_right, &last_edge_us_right };

/* One ISR handles both pins; `arg` tells it which counter/debounce-clock to use.
 * IRAM_ATTR: ISR code must live in IRAM so it still runs even while
 * flash cache is temporarily disabled (standard ESP-IDF requirement).
 * esp_timer_get_time() is documented safe to call from ISR context. */
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    encoder_isr_arg_t *a = (encoder_isr_arg_t *)arg;
    int64_t now = esp_timer_get_time();

    if (now - *(a->last_edge_us) < MIN_PULSE_INTERVAL_US) {
        return;   /* too soon to be a real slot transition - noise, drop it */
    }
    *(a->last_edge_us) = now;
    (*(a->counter))++;   /* only this otherwise. No math, no logging, no delays in an ISR. */
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
    gpio_isr_handler_add(ENC_L_PIN, encoder_isr_handler, (void *)&enc_left_arg);
    gpio_isr_handler_add(ENC_R_PIN, encoder_isr_handler, (void *)&enc_right_arg);
}

/* VM now has its own wire from the powerbank (fixed 2026-07-24, see README
 * "Power Architecture") — the brownout reset loop this cap was guarding
 * against is confirmed gone. Left at 100 for now since Kp hasn't been
 * retuned with valid encoder data yet; raise deliberately, not by default,
 * once real RPM tracking data says the controller needs more headroom. */
#define MAX_SAFE_PWM 100.0f

/* Floor, not just a "can't go negative" clamp: the powerbank feeding VM
 * auto-shuts-off when it sees low/no current draw (no-load detection meant
 * for phone charging, wrong assumption for a motor). Lowered 40 -> 20
 * (2026-07-27, power confirmed stable since the hardware fix) because 40
 * was clamping the right wheel's PWM even though its actual RPM sat well
 * above TARGET_RPM=30 - PID couldn't slow it down further to reach target.
 * Raise it again if the powerbank cuts out at this lower floor. */
#define MIN_SAFE_PWM 20.0f

/* error(t) = target - actual; u(t) = Kp*error(t) + Ki*integral(t);
 * PWM = clamp(u(t), MIN_SAFE_PWM, MAX_SAFE_PWM). `integral` is per-wheel
 * state owned by the caller (control_task) since left/right run independent
 * loops - it persists across calls, unlike everything else in here. */
static uint32_t pid_step(float target_rpm, float actual_rpm, float *integral) {
    float error = target_rpm - actual_rpm;
    float dt = CONTROL_PERIOD_MS / 1000.0f;

    *integral += error * dt;
    if (*integral > MAX_INTEGRAL)  *integral = MAX_INTEGRAL;   /* anti-windup clamp, see MAX_INTEGRAL comment */
    if (*integral < -MAX_INTEGRAL) *integral = -MAX_INTEGRAL;

    float u = KP * error + KI * (*integral);

    if (u < MIN_SAFE_PWM)   u = MIN_SAFE_PWM;   /* clamp: floor keeps current draw above the powerbank's auto-shutoff threshold */
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
 * Trade-off: shorter window = coarser RPM resolution with only 20 slots/rev.
 * Raised 50ms -> 200ms (2026-07-27): at 50ms, 1 pulse = 60 RPM/step, and
 * TARGET_RPM=30 sits exactly BETWEEN two measurable levels (0, 60) - the
 * controller could never read "at target", only ever +/-30 RPM error, a
 * pure quantization limit cycle no amount of Kp/Ki tuning can remove
 * (confirmed live: RPM alternating 0/60 every cycle while PWM hunted).
 * At 200ms, 1 pulse = 15 RPM/step, so 30 RPM = exactly 2 pulses - target
 * is now representable. Slower loop (5 Hz vs 20 Hz), acceptable at these
 * low target speeds. */
#define CONTROL_PERIOD_MS 200   /* 5 Hz */

/* Silent-failure guard: if PWM is clearly high enough to move the wheel but
 * pulse count stays 0 for this many consecutive cycles, the encoder is
 * almost certainly dead (misaligned, unglued, wiring fault) rather than the
 * wheel legitimately being stopped. 20 cycles = 4s at 5Hz (was 1s at the old
 * 20Hz) — one slot alone only needs ~100ms at TARGET_RPM, so this still has
 * generous margin against false positives from startup transients. */
#define STALL_PWM_THRESHOLD 30.0f
#define STALL_CYCLE_LIMIT   20

static void control_task(void *arg) {
    encoder_gpio_init();

    uint32_t snapshot_left, snapshot_right;
    float applied_pwm_left = 0.0f, applied_pwm_right = 0.0f;
    float integral_left = 0.0f, integral_right = 0.0f;
    uint32_t stall_cycles_left = 0, stall_cycles_right = 0;
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

        float target_pwm_left  = (float)pid_step(TARGET_RPM, rpm_left_shared,  &integral_left);
        float target_pwm_right = (float)pid_step(TARGET_RPM, rpm_right_shared, &integral_right);

        applied_pwm_left  = slew_limit(applied_pwm_left,  target_pwm_left);
        applied_pwm_right = slew_limit(applied_pwm_right, target_pwm_right);

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)applied_pwm_left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)applied_pwm_right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        /* Stall watch, left wheel */
        if (applied_pwm_left > STALL_PWM_THRESHOLD && snapshot_left == 0) {
            stall_cycles_left++;
            if (stall_cycles_left == STALL_CYCLE_LIMIT) {
                ESP_LOGW(TAG, "LEFT encoder stall suspected: PWM=%.0f but 0 pulses for %ds",
                         applied_pwm_left, STALL_CYCLE_LIMIT * CONTROL_PERIOD_MS / 1000);
            }
        } else {
            stall_cycles_left = 0;
        }

        /* Stall watch, right wheel */
        if (applied_pwm_right > STALL_PWM_THRESHOLD && snapshot_right == 0) {
            stall_cycles_right++;
            if (stall_cycles_right == STALL_CYCLE_LIMIT) {
                ESP_LOGW(TAG, "RIGHT encoder stall suspected: PWM=%.0f but 0 pulses for %ds",
                         applied_pwm_right, STALL_CYCLE_LIMIT * CONTROL_PERIOD_MS / 1000);
            }
        } else {
            stall_cycles_right = 0;
        }

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
