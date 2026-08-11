/*
 * F1 — Basic motor spin, F2 — encoder RPM read
 * Edge AMR / SLAM robot — Week 2, Phase 2 (Firmware)
 *
 * F1: drive both TT motors forward at a fixed 50% PWM to prove the
 * ESP32 -> TB6612FNG -> motor path works end to end.
 * F2: count LM393 encoder pulses via ISR, compute RPM every 1s.
 * Merged into one project because F3 (PID) needs both together anyway.
 *
 * Raw hardware wiring (ESP32 38-pin DevKit  ->  TB6612FNG), UNCHANGED since
 * original solder-down — this describes the physical wires, not which
 * software label (AIN1_PIN etc.) currently points at each one (see 2026-08-11
 * note below the #defines for why those two now differ):
 *   GPIO16 -> PWMA      GPIO17 -> PWMB     (speed  — driven by LEDC)
 *   GPIO18 -> AIN1      GPIO21 -> BIN1     (direction)
 *   GPIO19 -> AIN2      GPIO22 -> BIN2     (direction)
 *   GPIO23 -> STBY                         (enable, HIGH = run)
 *   3V3    -> VCC   (logic power)
 *   VM     <- powerbank (direct, bypasses ESP32 — fixed 2026-07-24)
 *   GND    -> GND   (must be common with the powerbank ground)
 *
 * Raw hardware wiring (ESP32 -> LM393 encoders), also unchanged:
 *   GPIO34 -> encoder OUT on the TB6612 "A" side (same physical wheel as AIN1/AIN2/PWMA)
 *   GPIO35 -> encoder OUT on the TB6612 "B" side (same physical wheel as BIN1/BIN2/PWMB)
 *
 * F2 test procedure (do this BEFORE trusting F3/PID):
 *   Comment out the "wake the TB6612FNG" + direction lines in app_main so
 *   the motors stay off, flash, then spin each wheel by hand and confirm
 *   the RPM printed on the serial monitor looks sane. Only once that's
 *   verified, uncomment and let both F1 (motor) and F2 (encoder) run together.
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

/* --- Direction + enable pins: plain digital outputs (HIGH/LOW only) ---
 *
 * SWAPPED 2026-08-11 (was AIN1=18,AIN2=19 / BIN1=21,BIN2=22): the stereo
 * camera is mounted facing the chassis's ORIGINAL reverse direction, and
 * rather than re-route the CSI ribbon cable to spin the camera around
 * (this project's single biggest recurring hardware failure mode — CSI
 * cable, encoder VCC, encoder solder joint, left-motor GPIO have all
 * worked loose from handling before, see project memory), the robot's own
 * notion of "forward" was redefined in software instead so the camera's
 * facing direction becomes the new forward.
 *
 * A 180-degree redefinition of "forward" is a rigid rotation about the
 * vertical axis — it flips BOTH front/back AND left/right together, it is
 * not possible to flip just one (same principle already worked out for the
 * IMU mount orientation, see project memory 2026-07-18). This #define swap
 * is the left/right half of that: software's "channel A" (LEDC_CHANNEL_0,
 * used for target_pwm_left/ENC_L_PIN everywhere else in this file) now
 * points at the physical GPIOs that used to be TB6612 channel B, so the
 * math in control_task() — which was already correct and is NOT touched by
 * this change — now drives the wheel that is physically on the robot's new
 * left side. The front/back half of the flip is the AIN1/AIN2/BIN1/BIN2
 * polarity flip in app_main() below (0,1 instead of the original 1,0) —
 * both halves are required together, neither alone gives the right result
 * (worked through with vịt before making this change, see conversation).
 *
 * Physical wiring itself has NOT changed — only which label points at which
 * already-wired GPIO. Cross-check against the raw wiring table above if
 * this ever needs re-deriving. */
#define AIN1_PIN  GPIO_NUM_21
#define AIN2_PIN  GPIO_NUM_22
#define BIN1_PIN  GPIO_NUM_18
#define BIN2_PIN  GPIO_NUM_19
#define STBY_PIN  GPIO_NUM_23

/* --- Speed pins: driven by the LEDC PWM peripheral, not set by hand ---
 * Swapped together with AIN/BIN above, same reason. */
#define PWMA_PIN  GPIO_NUM_17
#define PWMB_PIN  GPIO_NUM_16

#define PWM_FREQ_HZ  1000   /* 1 kHz PWM carrier */
#define PWM_DUTY_50  128    /* 128 / 255 = 50% duty (8-bit resolution) */

/* --- F2: encoder pins + RPM constants ---
 * Swapped together with AIN/BIN/PWMA/PWMB above, same reason: ENC_L_PIN
 * must read the same physical wheel that LEDC_CHANNEL_0/target_pwm_left
 * now drives, or RPM feedback would be reporting the wrong wheel to its
 * own PID loop. */
#define ENC_L_PIN GPIO_NUM_35   /* left encoder signal (was 34) */
#define ENC_R_PIN GPIO_NUM_34   /* right encoder signal (was 35) */
#define SLOTS_PER_REV 20        /* LM393 disk: 20 slots = 1 full wheel revolution */

/* Moved up from near control_task() (2026-07-27): pid_step() needs this for
 * its dt calculation, but #define order matters in C - the compiler had
 * never actually seen this macro yet at the point pid_step() used it,
 * since the old location was much further down in the file. That was a
 * silent build failure (`'CONTROL_PERIOD_MS' undeclared`), not a subtle
 * bug - every build since Ki was added failed here, meaning any test
 * result since then that "worked" was very likely re-flashing a stale,
 * older binary rather than the code we thought we were testing.
 *
 * Original context for the value itself: shorter window = coarser RPM
 * resolution with only 20 slots/rev. Raised 50ms -> 200ms (2026-07-27):
 * at 50ms, 1 pulse = 60 RPM/step, and a since-changed 30 RPM test target
 * sat exactly BETWEEN two measurable levels - the controller could never
 * read "at target". At 200ms, 1 pulse = 15 RPM/step - finer, though RPM
 * itself no longer depends on this window at all now (see last_interval_us
 * / RPM_STALE_US further down - period measurement replaced pulse
 * counting for the actual RPM value). This constant now only sets how
 * often PID/sync/PWM update, not measurement resolution. */
#define CONTROL_PERIOD_MS 200   /* 5 Hz */

/* --- F3: PI velocity control (Kd deliberately skipped, see KI comment below) --- */
/* TARGET_RPM (hardcoded 30, then 60) retired 2026-07-27 now that F5 wires
 * real /cmd_vel - per-wheel targets are computed live from linear.x/angular.z
 * instead (see the F5 section below). Kept using it through F1-F4 was the
 * right call (needed *some* number to validate the control loop before the
 * real command source existed) - it's just no longer the source of truth. */
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

/* Wheel sync: per-wheel PID keeps each wheel near TARGET_RPM independently,
 * but confirmed live (2026-07-27) that's not enough - robot pulls right,
 * would circle if left running. Small per-cycle differences between the two
 * (different motors, different overshoot/settling) add up over time into
 * real position drift. This trims each wheel's target RPM based on TOTAL
 * distance traveled so far (cumulative pulses, never reset), not just
 * instantaneous RPM - a wheel that's fallen behind gets sped up until it
 * actually catches up, not just "goes the same speed from now on".
 * KSYNC=0.05 is a conservative starting guess, same philosophy as Kp/Ki.
 * Gated 2026-07-27 (F5): only active while /cmd_vel's angular.z is near
 * zero (straight-line driving) - an intentional turn deliberately makes the
 * two wheels travel different distances, and without this gate the sync
 * logic would fight every turn command, trying to "correct" a difference
 * that was never a mistake. See STRAIGHT_ANGULAR_THRESHOLD below. */
#define KSYNC 0.05f
#define MAX_SYNC_TRIM 15.0f   /* clamp: don't let sync fighting dominate over the base target */

/* --- F4: odometry (measured 2026-07-27: wheel diameter 6cm, wheelbase 10cm) ---
 * distance per pulse = wheel circumference / slots per rev - converts a raw
 * pulse count directly into linear distance traveled by that wheel.
 * Differential-drive kinematics: center-of-robot distance is the AVERAGE of
 * the two wheels' distances (if both go the same distance, robot moves
 * straight with no turn); heading change is the DIFFERENCE divided by the
 * wheelbase (if right travels further than left, robot rotates toward the
 * left/slower side - matches the direction convention already confirmed
 * earlier today: faster right wheel -> drifts left). */
#define WHEEL_DIAMETER_M 0.06f
#define WHEELBASE_M 0.10f
#define PI_F 3.14159265f   /* not relying on M_PI - not guaranteed defined by every libc without _USE_MATH_DEFINES */
#define WHEEL_CIRCUMFERENCE_M (PI_F * WHEEL_DIAMETER_M)
#define DISTANCE_PER_PULSE_M (WHEEL_CIRCUMFERENCE_M / SLOTS_PER_REV)

/* --- F5: /cmd_vel in, /odom out (2026-07-27) ---
 * Safety default: if no /cmd_vel received within this long (agent crash,
 * USB unplugged, Jetson down), treat commanded velocity as 0 rather than
 * keep driving on the last command received - a lost connection must mean
 * "stop", never "keep going blind". Applies both before the first command
 * ever arrives AND if a live connection drops mid-drive. */
#define CMD_VEL_TIMEOUT_US 1000000   /* 1s */

/* Below this |angular.z|, treat the commanded motion as "going straight" for
 * wheel-sync purposes (see KSYNC comment above) - not exactly 0 because a
 * genuinely-straight command from ros2/Nav2 may carry tiny floating-point
 * noise rather than a clean 0.0. */
#define STRAIGHT_ANGULAR_THRESHOLD 0.05f   /* rad/s */

#define ODOM_PUBLISH_PERIOD_MS 100   /* 10 Hz */

static const char *TAG = "control_task";

/* Shared between ISR (writer) and control_task (reader) -> must be volatile
 * so the compiler never caches a stale copy in a register. */
static volatile uint32_t pulse_count_left = 0;
static volatile uint32_t pulse_count_right = 0;

/* Latest computed RPM, exposed for uros_task (F5) to read later. */
static volatile float rpm_left_shared = 0.0f;
static volatile float rpm_right_shared = 0.0f;

/* F5: /cmd_vel, written by uros_task's subscription callback, read by
 * control_task. last_cmd_vel_us is the watchdog clock - see
 * CMD_VEL_TIMEOUT_US comment above. */
static volatile float cmd_linear_x = 0.0f;
static volatile float cmd_angular_z = 0.0f;
static volatile int64_t last_cmd_vel_us = 0;

/* F5: pose + velocity, written by control_task (F4 odometry math), read by
 * uros_task to publish /odom. Plain floats (not the mutex-protected pulse
 * counters) - a torn read here would be a stale/slightly-off reading, not a
 * silently-corrupted control decision, so it's not worth the locking. */
static volatile float pose_x_shared = 0.0f;
static volatile float pose_y_shared = 0.0f;
static volatile float pose_theta_shared = 0.0f;
static volatile float linear_vel_shared = 0.0f;
static volatile float angular_vel_shared = 0.0f;

/* Added for hardware debugging (Alex, one-wheel-not-spinning investigation):
 * applied_pwm_left/right were previously local-only to control_task, so
 * uros_task had no way to report them. Mirrors the rpm_*_shared pattern
 * above. */
static volatile float pwm_left_shared = 0.0f;
static volatile float pwm_right_shared = 0.0f;

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

/* 2026-07-27, after fixing a bad encoder solder joint: with clean pulses,
 * RPM was still visibly jumping in fixed steps (0, 15, 30, 45...) with no
 * in-between values. Root cause: counting whole pulses in a fixed 200ms
 * window can only ever measure in units of "1 pulse / window" = 15 RPM -
 * a motor spinning at, say, 22 RPM is invisible to that method. Fixed by
 * also recording the time BETWEEN consecutive pulses (`last_interval_us`),
 * so control_task can compute RPM from timing instead of counting - the
 * standard "period measurement" tachometer technique, far finer resolution
 * at low speed than "frequency measurement" (pulse-counting) gives. */
static volatile int64_t last_interval_us_left = 0;
static volatile int64_t last_interval_us_right = 0;

typedef struct {
    volatile uint32_t *counter;
    volatile int64_t *last_edge_us;
    volatile int64_t *last_interval_us;
} encoder_isr_arg_t;

static encoder_isr_arg_t enc_left_arg  = { &pulse_count_left,  &last_edge_us_left,  &last_interval_us_left };
static encoder_isr_arg_t enc_right_arg = { &pulse_count_right, &last_edge_us_right, &last_interval_us_right };

/* One ISR handles both pins; `arg` tells it which counter/debounce-clock to use.
 * IRAM_ATTR: ISR code must live in IRAM so it still runs even while
 * flash cache is temporarily disabled (standard ESP-IDF requirement).
 * esp_timer_get_time() is documented safe to call from ISR context. */
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    encoder_isr_arg_t *a = (encoder_isr_arg_t *)arg;
    int64_t now = esp_timer_get_time();
    int64_t since_last = now - *(a->last_edge_us);

    if (since_last < MIN_PULSE_INTERVAL_US) {
        return;   /* too soon to be a real slot transition - noise, drop it */
    }
    *(a->last_interval_us) = since_last;
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
 * Lowered again 20 -> 10 (same day, real ground test): right wheel still
 * pinned at PWM=20 with RPM 150-195 (vs. left's ~45) while actually driving
 * - the sync-trim fix couldn't help because PID still couldn't push PWM any
 * lower. This is a diagnostic test as much as a fix: if RPM stays this high
 * even at PWM=10, the cause isn't the floor anymore, it's a real mechanical/
 * motor asymmetry - raise the floor back if the powerbank cuts out. */
#define MIN_SAFE_PWM 10.0f

/* error(t) = target - actual; u(t) = Kp*error(t) + Ki*integral(t);
 * PWM = clamp(u(t), MIN_SAFE_PWM, MAX_SAFE_PWM). `integral` is per-wheel
 * state owned by the caller (control_task) since left/right run independent
 * loops - it persists across calls, unlike everything else in here.
 *
 * BUG FOUND 2026-07-27 (F5 first test): MIN_SAFE_PWM was clamping the
 * output even when target_rpm==0 - the F5 safety watchdog forces target
 * to 0 on a stale/missing /cmd_vel, but the floor was overriding that,
 * so the robot never actually stopped (kept driving on no command at all,
 * confirmed live - x drifted past 2m unattended). Explicit target==0
 * bypass added below: a commanded stop means PWM=0, full stop, no floor -
 * the floor only makes sense while the robot is supposed to be moving.
 * Also resets the integral so a long stop doesn't leave stale windup to
 * bias the next real command. */
static uint32_t pid_step(float target_rpm, float actual_rpm, float *integral) {
    if (target_rpm == 0.0f) {
        *integral = 0.0f;
        return 0;
    }

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
 * (CONTROL_PERIOD_MS itself now lives up near SLOTS_PER_REV - pid_step()
 * needs it too, and #define order matters in C.) */

/* Companion to the period-measurement RPM calc above: period measurement
 * alone can't distinguish "stopped" from "just hasn't ticked yet" - without
 * a timeout it would keep reporting the last (increasingly stale) interval
 * forever after the wheel actually stops. If no new pulse arrives within
 * this long, treat RPM as 0 instead. ~500ms is generous margin (~10x) over
 * the ~50ms/pulse interval expected near TARGET_RPM=60. */
#define RPM_STALE_US 500000

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
    uint32_t total_pulses_left = 0, total_pulses_right = 0;
    uint32_t stall_cycles_left = 0, stall_cycles_right = 0;
    /* F4 pose lives in pose_x_shared/pose_y_shared/pose_theta_shared now
     * (F5 needs uros_task to read it) - origin = wherever the robot is at
     * boot/flash, all initialized 0 at file scope. */
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        int64_t now = esp_timer_get_time();
        int64_t last_edge_left, last_edge_right, interval_left, interval_right;

        portENTER_CRITICAL(&encoder_mux);
        snapshot_left = pulse_count_left;
        pulse_count_left = 0;
        snapshot_right = pulse_count_right;
        pulse_count_right = 0;
        /* int64_t isn't atomic on a 32-bit MCU (two word-writes) - read these
         * inside the same critical section as the pulse counts to avoid a
         * torn read racing the ISR mid-update. */
        last_edge_left   = last_edge_us_left;
        last_edge_right  = last_edge_us_right;
        interval_left    = last_interval_us_left;
        interval_right   = last_interval_us_right;
        portEXIT_CRITICAL(&encoder_mux);

        /* RPM from time between pulses, not pulses-per-window - see
         * last_interval_us / RPM_STALE_US comments above for why. */
        rpm_left_shared = (interval_left == 0 || (now - last_edge_left) > RPM_STALE_US)
                              ? 0.0f : 60000000.0f / (SLOTS_PER_REV * (float)interval_left);
        rpm_right_shared = (interval_right == 0 || (now - last_edge_right) > RPM_STALE_US)
                              ? 0.0f : 60000000.0f / (SLOTS_PER_REV * (float)interval_right);

        /* F4 odometry: how far each wheel moved THIS cycle (not cumulative),
         * then combine into how far the robot's center moved + how much it
         * turned. Midpoint integration (average current and new heading,
         * not just the old one) instead of plain Euler - at 200ms/cycle a
         * turning robot's heading can shift enough mid-cycle that using only
         * the pre-turn heading for the whole cycle's distance would visibly
         * bias x/y, especially while turning sharply. */
        float dist_left  = snapshot_left  * DISTANCE_PER_PULSE_M;
        float dist_right = snapshot_right * DISTANCE_PER_PULSE_M;
        float dist_center = (dist_left + dist_right) / 2.0f;
        float dtheta = (dist_right - dist_left) / WHEELBASE_M;
        float theta_mid = pose_theta_shared + dtheta / 2.0f;
        float dt = CONTROL_PERIOD_MS / 1000.0f;

        pose_x_shared += dist_center * cosf(theta_mid);
        pose_y_shared += dist_center * sinf(theta_mid);
        pose_theta_shared += dtheta;
        linear_vel_shared  = dist_center / dt;
        angular_vel_shared = dtheta / dt;

        /* F5: /cmd_vel -> per-wheel target RPM. Watchdog: stale command (or
         * none ever received) means 0, not "keep driving" - see
         * CMD_VEL_TIMEOUT_US comment above. */
        bool cmd_vel_fresh = (now - last_cmd_vel_us) <= CMD_VEL_TIMEOUT_US;
        float linear_x  = cmd_vel_fresh ? cmd_linear_x  : 0.0f;
        float angular_z = cmd_vel_fresh ? cmd_angular_z : 0.0f;

        float v_left_mps  = linear_x - angular_z * WHEELBASE_M / 2.0f;
        float v_right_mps = linear_x + angular_z * WHEELBASE_M / 2.0f;
        float target_rpm_left  = v_left_mps  / WHEEL_CIRCUMFERENCE_M * 60.0f;
        float target_rpm_right = v_right_mps / WHEEL_CIRCUMFERENCE_M * 60.0f;

        /* Wheel sync, gated to straight-line motion only - see KSYNC and
         * STRAIGHT_ANGULAR_THRESHOLD comments above. Positive diff = left
         * has traveled further than right so far; trim left's target down /
         * right's target up to close that gap. */
        bool going_straight = fabsf(angular_z) < STRAIGHT_ANGULAR_THRESHOLD;
        if (going_straight) {
            total_pulses_left  += snapshot_left;
            total_pulses_right += snapshot_right;
        }
        float sync_diff = (float)total_pulses_left - (float)total_pulses_right;
        float sync_trim = going_straight ? KSYNC * sync_diff : 0.0f;
        if (sync_trim > MAX_SYNC_TRIM)  sync_trim = MAX_SYNC_TRIM;
        if (sync_trim < -MAX_SYNC_TRIM) sync_trim = -MAX_SYNC_TRIM;

        float target_pwm_left  = (float)pid_step(target_rpm_left  - sync_trim, rpm_left_shared,  &integral_left);
        float target_pwm_right = (float)pid_step(target_rpm_right + sync_trim, rpm_right_shared, &integral_right);

        applied_pwm_left  = slew_limit(applied_pwm_left,  target_pwm_left);
        applied_pwm_right = slew_limit(applied_pwm_right, target_pwm_right);
        pwm_left_shared  = applied_pwm_left;
        pwm_right_shared = applied_pwm_right;

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

        /* NOTE (F5): once uros_task's custom transport claims UART0, this
         * won't show up on idf.py monitor anymore - debug via `ros2 topic
         * echo /odom` / `ros2 topic pub /cmd_vel` on the Jetson instead.
         * Left in place since it's harmless and still useful for standalone
         * bring-up before uros_task's transport takes over. */
        ESP_LOGI(TAG, "RPM L=%.1f R=%.1f  PWM L=%.0f R=%.0f  sync_diff=%.0f trim=%.1f  "
                      "pose x=%.3fm y=%.3fm theta=%.1fdeg",
                 rpm_left_shared, rpm_right_shared, applied_pwm_left, applied_pwm_right,
                 sync_diff, sync_trim, pose_x_shared, pose_y_shared, pose_theta_shared * 180.0f / PI_F);
    }
}

/* rclc_executor callback for the /cmd_vel subscription - just latches the
 * values and timestamps them for the watchdog. Runs in uros_task's context
 * (not an ISR), so no special atomicity concerns beyond the usual volatile. */
/* F5 diagnostics (2026-07-27): odometry was observed resetting toward 0
 * repeatedly during a real driving test, and the agent log showed the
 * micro-ROS client_key changing (new session) at the same time - strong
 * evidence the ESP32 itself is rebooting mid-drive, not just losing the
 * micro-ROS connection. Since console logging is muted (shares UART0 with
 * the transport - see esp_log_level_set below), there's no way to see a
 * reboot reason via idf.py monitor while /cmd_vel testing is happening.
 * Instead: read esp_reset_reason() once at boot (before anything else
 * runs) and publish it over the already-working micro-ROS link, so
 * `ros2 topic echo /esp32_diag` answers "was that a brownout?" without
 * needing the serial console at all. */
static esp_reset_reason_t g_reset_reason;

static const char *reset_reason_str(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT_PIN";
        case ESP_RST_SW:        return "SW_RESTART";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    cmd_linear_x    = (float)msg->linear.x;
    cmd_angular_z   = (float)msg->angular.z;
    last_cmd_vel_us = esp_timer_get_time();
}

/* F5: connects to the micro-ROS agent, subscribes /cmd_vel, publishes /odom.
 * Structured the same way as the proven esp32/microros_hello.c pattern
 * (Week 1) - outer loop pings the agent and (re)creates everything on
 * connect, inner loop runs while the agent stays reachable, and everything
 * gets torn down and retried if the agent disappears.
 * No printf()/ESP_LOG calls anywhere in this function or its callers from
 * here on - confirmed live (2026-07-27) that ANY console write to UART0
 * corrupts the micro-ROS binary stream sharing the same wire, including
 * during the ping-agent phase. Silence is required, not just tidiness. */
static void uros_task(void *arg) {
    while (1) {
        while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rcl_subscription_t cmd_vel_sub;
        rcl_publisher_t odom_pub;
        rcl_publisher_t diag_pub;
        rclc_executor_t executor;
        geometry_msgs__msg__Twist cmd_vel_msg;
        nav_msgs__msg__Odometry odom_msg = {0};
        std_msgs__msg__String diag_msg = {0};
        char diag_text[96];

        if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_node_init_default(&node, "esp32_motor_node", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_subscription_init_default(&cmd_vel_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&odom_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) {
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&diag_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32_diag") != RCL_RET_OK) {
            if (rcl_publisher_fini(&odom_pub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        rclc_executor_t *exec = &executor;
        rclc_executor_init(exec, &support.context, 1, &allocator);
        rclc_executor_add_subscription(exec, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

        /* diag_text's CONTENT is now rebuilt every publish cycle (see the
         * publish block below) so /esp32_diag carries live per-wheel
         * RPM/PWM, not just the reset reason from connection time. The
         * buffer pointer/capacity themselves don't change per-message, so
         * only need to be set once here. */
        diag_msg.data.data = diag_text;
        diag_msg.data.capacity = sizeof(diag_text);

        /* Set once - frame_id strings don't change per-message, only the
         * numeric fields below do. */
        odom_msg.header.frame_id.data = "odom";
        odom_msg.header.frame_id.size = strlen("odom");
        odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
        odom_msg.child_frame_id.data = "base_link";
        odom_msg.child_frame_id.size = strlen("base_link");
        odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

        TickType_t last_publish = xTaskGetTickCount();
        const TickType_t publish_period = pdMS_TO_TICKS(ODOM_PUBLISH_PERIOD_MS);
        bool agent_ok = true;

        while (agent_ok) {
            rclc_executor_spin_some(exec, RCL_MS_TO_NS(20));

            if (xTaskGetTickCount() - last_publish >= publish_period) {
                last_publish = xTaskGetTickCount();

                odom_msg.pose.pose.position.x = pose_x_shared;
                odom_msg.pose.pose.position.y = pose_y_shared;
                odom_msg.pose.pose.position.z = 0.0;

                /* Orientation as a quaternion, rotation about Z only (2D
                 * robot) - qz=sin(theta/2), qw=cos(theta/2), qx=qy=0. */
                float half_theta = pose_theta_shared / 2.0f;
                odom_msg.pose.pose.orientation.x = 0.0;
                odom_msg.pose.pose.orientation.y = 0.0;
                odom_msg.pose.pose.orientation.z = sinf(half_theta);
                odom_msg.pose.pose.orientation.w = cosf(half_theta);

                odom_msg.twist.twist.linear.x  = linear_vel_shared;
                odom_msg.twist.twist.angular.z = angular_vel_shared;

                if (rcl_publish(&odom_pub, &odom_msg, NULL) != RCL_RET_OK) {
                    agent_ok = false;
                }

                /* Rebuilt every cycle now (see comment above) - live
                 * per-wheel RPM/PWM for hardware debugging, since
                 * idf.py monitor is unusable once this transport owns
                 * UART0 (see note above cmd_vel_callback). */
                snprintf(diag_text, sizeof(diag_text),
                         "reset=%s RPM L=%.1f R=%.1f PWM L=%.0f R=%.0f",
                         reset_reason_str(g_reset_reason),
                         rpm_left_shared, rpm_right_shared,
                         pwm_left_shared, pwm_right_shared);
                diag_msg.data.size = strlen(diag_text);

                if (rcl_publish(&diag_pub, &diag_msg, NULL) != RCL_RET_OK) {
                    agent_ok = false;
                }
            }

            /* BUG FOUND 2026-07-27 (F5 first connectivity test): timeout=0
             * gives the ping response no time to actually arrive over the
             * UART round-trip, so it reported "failed" almost every cycle
             * even with the agent alive and working - confirmed live as a
             * connect/create-everything/disconnect loop every ~1-2s, right
             * after all the ROS entities were successfully created. 100ms
             * is generous for a serial round-trip without blocking the
             * executor/publish loop for long. */
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                agent_ok = false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (rcl_publisher_fini(&diag_pub, &node) != RCL_RET_OK) {}
        if (rcl_publisher_fini(&odom_pub, &node) != RCL_RET_OK) {}
        if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
        if (rclc_executor_fini(exec) != RCL_RET_OK) {}
        if (rcl_node_fini(&node) != RCL_RET_OK) {}
        rclc_support_fini(&support);
        vTaskDelay(pdMS_TO_TICKS(1000));
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

/* UART0 is the same physical wire idf.py monitor uses for the console - see
 * the F5 note on ESP_LOGI above. Reused from the proven esp32/microros_hello
 * pin/transport setup (Week 1), not a new choice. */
static size_t uart_port = UART_NUM_0;

void app_main(void) {
    /* First thing, before anything else touches power/peripherals - see
     * g_reset_reason comment above for why this matters. */
    g_reset_reason = esp_reset_reason();

    setup_gpio();
    setup_pwm();

    /* Direction pins are wired for forward, but F5 now drives actual speed
     * (including reverse/turn) through /cmd_vel via control_task - these
     * just set the electrical default; PWM=0 from a stale/absent /cmd_vel
     * (the safety default) means the motors don't move regardless.
     *
     * FLIPPED 2026-08-11 (was AIN1=1,AIN2=0 / BIN1=1,BIN2=0): the front/back
     * half of the forward-direction redefinition — see the AIN1_PIN #define
     * comment above for the full reasoning. This reverses each wheel's
     * physical spin direction for the same PWM value, which combined with
     * the pin-routing swap above is what makes the robot's "forward" match
     * the camera's facing direction instead of the original mount
     * direction. */
    gpio_set_level(STBY_PIN, 1);   /* wake the TB6612FNG out of standby */
    gpio_set_level(AIN1_PIN, 0);
    gpio_set_level(AIN2_PIN, 1);
    gpio_set_level(BIN1_PIN, 0);
    gpio_set_level(BIN2_PIN, 1);

    rmw_uros_set_custom_transport(true, (void *)&uart_port,
        esp32_serial_open, esp32_serial_close,
        esp32_serial_write, esp32_serial_read);

    /* BUG FOUND 2026-07-27 (F5 first test): ESP_LOGI/ESP_LOGW in control_task
     * and the micro-ROS transport above both write to UART0 at the same
     * time - confirmed live as a burst of garbled characters in the monitor
     * output, at the same moment the micro-ROS agent never saw a valid
     * session-establish packet. The two streams were corrupting each other.
     * Once the transport is set, all console log output must stop - from
     * here on, debugging is via `ros2 topic echo` on the Jetson, not
     * idf.py monitor (see README F5 section). */
    esp_log_level_set("*", ESP_LOG_NONE);

    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreate(uros_task, "uros_task", 16000, NULL, 5, NULL);

    /* app_main returns here; both tasks keep running. */
}
