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
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
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

/* --- IMU (MPU6050) sanity-check read, added 2026-08-14 ---
 * Soldered pins (2026-07-29), never read from firmware until now - this is
 * the first empirical data point for the still-unverified axis-mapping
 * hypothesis in project memory (2026-07-18/20 entries). Raw int16 values
 * only - deliberately NOT converted to g/deg-per-s on-device (vịt's call):
 * keeps this firmware change tiny/additive, and the axis-mapping math is
 * easier to iterate on in Python on a real machine than to re-flash for.
 * Conversion factors for whoever processes this later: MPU6050 wakes into
 * its power-on-reset default full-scale range (ACCEL_CONFIG/GYRO_CONFIG are
 * never written here, so they stay 0x00) - that's accel +/-2g
 * (16384 LSB/g) and gyro +/-250 deg/s (131 LSB/(deg/s)).
 * AD0 assumed tied low (0x68) - the common default on breakout boards; if
 * i2c_master_probe/the wake write fails, check AD0 first (0x69 otherwise). */
#define IMU_SDA_PIN       GPIO_NUM_26
#define IMU_SCL_PIN       GPIO_NUM_25
#define MPU6050_I2C_ADDR  0x68
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_INT_PIN_CFG  0x37
#define MPU6050_I2C_BYPASS_EN    0x02
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  /* accel(6) + temp(2) + gyro(6) = 14B burst */
#define IMU_READ_LEN 14

/* --- Magnetometer, added 2026-08-28 for the heading-drift fix ---
 * Lives on the MPU6050's AUX I2C bus (see MPU6050_I2C_BYPASS_EN below),
 * exposed onto the main SDA/SCL bus once bypass mode is enabled - this is
 * the compass chip on the new GY-86/GY-87-style board, added specifically
 * to stop the gyro-only heading estimate from drifting forever (see
 * README). Register map assumed HMC5883L-compatible (config A/B + mode +
 * X,Z,Y data burst - NOT X,Y,Z order) since that's the standard chip on
 * this class of board, but NOT independently confirmed - if MAG=ok but the
 * fused heading still looks wrong, this register-map assumption is the
 * first thing to re-check, not the address.
 * ADDRESS: was 0x1E (textbook HMC5883L default) - changed 2026-08-28 to
 * 0x2C after a live post-bypass I2C scan showed a real device newly
 * appearing at 0x2C (not present in the pre-bypass scan), while nothing
 * ever ACKed at 0x1E. Trusting the live scan over the datasheet default -
 * this board's actual compass chip is evidently not a stock HMC5883L, or
 * uses a non-default address strap. */
#define HMC5883L_I2C_ADDR       0x2C
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
#define HMC5883L_READ_LEN 6   /* X(2) + Z(2) + Y(2) */

/* Forward-declared: defined below setup_pwm() (near the other setup_*
 * helpers), but called from uros_task() which appears EARLIER in this file.
 * Without this, the compiler hits the call site first, assumes an
 * implicit int-returning function, then errors on the real bool-returning
 * definition later ("conflicting types") - the exact CONTROL_PERIOD_MS
 * class of bug this file has been bitten by before (declaration-order
 * bugs are silent at a glance, loud at build time - always read past the
 * first error line before assuming a fix is complete). */
static bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz);

/* Same forward-reference reason as imu_read_raw above - defined near
 * setup_magnetometer() further down, called from uros_task() above that. */
static bool mag_read_raw(int16_t *mx, int16_t *my, int16_t *mz);

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
 * resolution with only 20 slots/rev, back when RPM was computed by
 * counting pulses per window. That's no longer how RPM is measured (see
 * last_interval_us / RPM_STALE_US further down - period-based timing
 * replaced pulse counting), so this constant only sets how often PID/PWM
 * update, not measurement resolution - free to lower without reintroducing
 * the old quantization problem. Lowered 200ms -> 50ms (2026-08-27) for
 * tighter speed-loop response; control_task itself has no I2C/blocking
 * calls, plenty of headroom on the ESP32 at this rate. */
#define CONTROL_PERIOD_MS 50   /* 20 Hz */

/* --- F3: PI velocity control (Kd deliberately skipped, see KI comment below) --- */
/* TARGET_RPM (hardcoded 30, then 60) retired 2026-07-27 now that F5 wires
 * real /cmd_vel - per-wheel targets are computed live from linear.x/angular.z
 * instead (see the F5 section below). Kept using it through F1-F4 was the
 * right call (needed *some* number to validate the control loop before the
 * real command source existed) - it's just no longer the source of truth. */
/* Kp/Ki/max-integral-contribution converted from #define to live-tunable
 * globals 2026-08-27: raising the mission's speed caps exposed a real
 * steady-state PID weakness (PWM plateauing ~100-130 under real load,
 * never reaching the wheels' true target RPM) - the fixed MAX_I_CONTRIBUTION
 * below was too small to let the integral term close a persistent error on
 * this heavier-than-bench-tested robot. Rather than guess a new fixed
 * number blind and reflash repeatedly, these are now pushed live from the
 * dashboard over /pid_gains (see pid_gains_callback below and the
 * dashboard's PID box in search_and_rescue.py) so they can be tuned against
 * the real load in real time. Defaults are just the starting point - not
 * meant to be the final tuned values anymore. */
static volatile float g_kp = 3.0f;             /* start small, increase until oscillation appears, back off */
static volatile float g_ki = 0.2f;
/* Anti-windup: without SOME limit, integral grows unbounded whenever output
 * is pinned at MIN/MAX_SAFE_PWM (e.g. a stalled wheel), causing a huge
 * overshoot once it frees up. Was a fixed 40.0f contribution cap - too
 * tight for real load, hence live-tunable now too. */
static volatile float g_max_i_contribution = 40.0f;

/* --- Heading control ---
 * Speed regulation is two INDEPENDENT per-wheel PID loops (see control_task)
 * - each wheel converges to its own target from its own measured RPM, with
 * its own integrator, so a real hardware asymmetry (e.g. more friction on
 * one motor) gets corrected at its actual source instead of being inferred
 * indirectly from rotation.
 *
 * Heading correction comes from the gyro, layered on TOP of that: compare
 * the desired turn rate (angular.z, or the heading-lock setpoint below when
 * not actively told to turn) against the MEASURED turn rate
 * (imu_gz_rad_per_s_shared), PROPORTIONAL only (no integral - matches this
 * codebase's own "start simple" philosophy). This shifts each wheel's
 * TARGET (added to one, subtracted from the other, in RPM units - same
 * domain as target_rpm_left/right, so it composes by plain addition), not
 * its PID gain - Kp/Ki only ever affect how hard a wheel's own loop chases
 * its own error, heading correction only ever affects what that target IS.
 * Falls back to trim=0 whenever the IMU read is invalid - never correct off
 * a signal we don't trust. */
static volatile float g_kheading = 15.0f;   /* RPM of trim per rad/s of heading-rate error - live-tunable, see pid_gains_callback */
/* Ceiling on heading_trim_rpm - was a fixed #define (20.0f), too tight once
 * live testing (2026-08-27) showed it silently capping the correction well
 * below what a real heading error needed, with no way to see that from the
 * dashboard. Now live-tunable same as everything else above - set it high
 * (or very high, effectively "off") from the dashboard if a real, sustained
 * heading error needs more authority than the old fixed ceiling allowed. */
static volatile float g_max_heading_trim_rpm = 20.0f;

/* Heading LOCK (position, not rate): g_kheading/g_max_heading_trim_rpm above
 * only correct a RATE mismatch (commanded turn speed vs measured turn
 * speed) - once the robot stops rotating it stops correcting, even if it's
 * now pointed the wrong way. This adds the missing angle term: whenever the
 * commanded angular.z is ~0 (straight driving, or sitting still after a
 * turn), latch the current heading as a target and keep computing a virtual
 * "turn rate" proportional to how far off that target the robot drifts -
 * this virtual rate feeds into the same g_kheading rate loop above (see
 * control_task), reusing it instead of adding a second output path. */
static volatile float g_kheading_lock = 2.0f;          /* virtual rad/s per rad of heading error */
static volatile float g_max_heading_lock_rate = 0.6f;  /* rad/s ceiling on that virtual rate */
/* volatile: written by control_task, read by uros_task for the diag string
 * (same cross-task pattern as pose_theta_shared etc. above). */
static volatile float g_target_heading_rad = 0.0f;
static volatile bool  g_heading_lock_active = false;
#define HEADING_LOCK_DEADBAND_RAD (0.5f * 3.14159265f / 180.0f)  /* ~0.5deg - don't hunt on sensor noise */
#define ANGZ_TURN_DEADBAND 0.02f  /* rad/s - below this, cmd_vel counts as "not turning" */

/* Manual per-wheel PWM trim (2026-08-28) - a flat PWM offset added on top of
 * everything the PID/heading loops already decided, for compensating a
 * mechanically weaker motor (different friction/torque between the two
 * physical motors - not something a speed PID or heading PID is meant to
 * fix, since it's a per-wheel hardware asymmetry, not an error signal).
 * Live-tunable via the same /pid_gains mechanism as Kp/Ki/etc (see
 * pid_gains_callback) - deliberately NOT touched by pid_step's target==0
 * safety bypass (see the commanded_to_move gate at its application site
 * below): a commanded stop must still mean PWM=0, trim included -
 * otherwise a stale/lost /cmd_vel would leave one wheel creeping instead of
 * actually stopping, defeating the whole point of that safety bypass. */
static volatile float g_trim_left = 0.0f;
static volatile float g_trim_right = 0.0f;

/* Odometry reset request (2026-08-27) - set by pid_gains_callback on the
 * "RESET_ODOM" sentinel command (see below), cleared by control_task once
 * actioned. A plain bool flag between the two tasks, same pattern as the
 * other cross-task shared vars in this file - no lock needed, one writer
 * (uros_task), one reader (control_task), and a torn read of a single bool
 * isn't a real hazard the way the paired pulse-count/timestamp fields are. */
static volatile bool g_reset_odom_requested = false;

/* Motor kill switch (2026-08-28) - live toggle for tuning convenience, set
 * by pid_gains_callback on "MOTOR_OFF"/"MOTOR_ON" (same channel/pattern as
 * RESET_ODOM above). STBY is pulled LOW/HIGH immediately in the callback
 * itself, not just next control_task cycle - the TB6612FNG ignores PWM/DIR
 * entirely while in standby, so this is a real power cut, not a commanded-
 * zero-speed workaround the PID loop could fight. control_task also forces
 * PWM=0 and clears both integrators while this is set (see its use below),
 * so the diag panel's PWM readout stays honest and re-enabling doesn't slam
 * the wheels with windup built up while the loop chased a target it
 * physically couldn't reach. */
static volatile bool g_motor_killed = false;

/* --- F4: odometry (measured 2026-07-27: wheelbase 10cm; wheel diameter
 * re-measured 2026-08-26 as 6.7cm, corrected from the original 6cm) ---
 * distance per pulse = wheel circumference / slots per rev - converts a raw
 * pulse count directly into linear distance traveled by that wheel.
 * Differential-drive kinematics: center-of-robot distance is the AVERAGE of
 * the two wheels' distances (if both go the same distance, robot moves
 * straight with no turn); heading change is the DIFFERENCE divided by the
 * wheelbase (if right travels further than left, robot rotates toward the
 * left/slower side - matches the direction convention already confirmed
 * earlier today: faster right wheel -> drifts left). */
#define WHEEL_DIAMETER_M 0.067f
#define WHEELBASE_M 0.10f
#define PI_F 3.14159265f   /* not relying on M_PI - not guaranteed defined by every libc without _USE_MATH_DEFINES */
#define WHEEL_CIRCUMFERENCE_M (PI_F * WHEEL_DIAMETER_M)
#define DISTANCE_PER_PULSE_M (WHEEL_CIRCUMFERENCE_M / SLOTS_PER_REV)

static inline float wrap_angle_rad(float a) {
    while (a > PI_F)  a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

/* --- F5: /cmd_vel in, /odom out (2026-07-27) ---
 * Safety default: if no /cmd_vel received within this long (agent crash,
 * USB unplugged, Jetson down), treat commanded velocity as 0 rather than
 * keep driving on the last command received - a lost connection must mean
 * "stop", never "keep going blind". Applies both before the first command
 * ever arrives AND if a live connection drops mid-drive. */
#define CMD_VEL_TIMEOUT_US 1000000   /* 1s */

/* Also gates the IMU/magnetometer reads in uros_task (same block, see
 * below) - lowered 100ms -> 50ms (2026-08-27) alongside CONTROL_PERIOD_MS
 * so the gyro rate feeding heading correction refreshes every control
 * cycle instead of every other one. Each I2C read is on the order of ~1-2ms
 * at this bus speed, cheap against a 50ms budget. Not pushed lower than
 * this: uros_task's outer loop already has an unconditional 20ms
 * vTaskDelay per iteration (see the end of the agent_ok while-loop below),
 * so a period much under ~40-50ms wouldn't reliably be hit anyway - the
 * outer loop's own cadence would become the real bottleneck, not this
 * constant. */
#define ODOM_PUBLISH_PERIOD_MS 50   /* 20 Hz */

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

/* Gyro-Z heading fusion (2026-08-22), written by uros_task's existing IMU
 * read, read by control_task - same cross-task pattern as rpm_*_shared
 * above. imu_gz_valid_shared is false whenever the most recent read failed
 * (IMU never woke, or the ~20s-uptime read-degradation bug latched imu_dev
 * dead for this boot - see README "Lessons Learned") so control_task can
 * always fail safe back to the original encoder-differential dtheta
 * instead of blending in a stale/garbage gyro value. */
static volatile float imu_gz_rad_per_s_shared = 0.0f;
static volatile bool imu_gz_valid_shared = false;

/* Gyro zero-rate bias, raw LSB units (same units as the int16 gz field
 * imu_read_raw returns) - measured once at boot by calibrate_gyro_bias()
 * (see setup_imu()), subtracted from every raw gz reading before converting
 * to rad/s. MEMS gyros essentially never read exactly zero at rest - a
 * typical uncorrected zero-rate offset is a degree or more per second,
 * which integrates directly into the heading estimate every single cycle,
 * gyro-only or fused. Declared here (before uros_task, which is the first
 * user) for the same declaration-order reason as the other globals above -
 * this file has been bitten by that mistake enough times this session to
 * not risk it again. Only ever written once, at boot, before either task
 * starts - no cross-task synchronization needed. */
static float g_gyro_z_bias_raw = 0.0f;

/* Magnetometer heading fusion (2026-08-28), same cross-task pattern as
 * imu_gz_rad_per_s_shared above - written by uros_task's mag read, read by
 * control_task. mag_heading_valid_shared false means "don't use this",
 * same fail-safe philosophy as imu_gz_valid_shared. Value is already
 * zero-referenced against wherever the robot was pointed at boot (see
 * g_mag_calibrated in uros_task) - NOT a true compass bearing, doesn't need
 * to be one, since this whole system's heading frame is relative/arbitrary
 * (odom frame) to begin with. */
static volatile float mag_heading_rad_shared = 0.0f;
static volatile bool mag_heading_valid_shared = false;

/* Boot-time zero-reference "calibration" (see uros_task's mag_read_raw call
 * site) - only ever touched from within uros_task itself, no cross-task
 * sharing needed, unlike the two above. Declared here (before uros_task,
 * which is the first user of them) for the same declaration-order reason as
 * CONTROL_PERIOD_MS/GYRO_RAW_TO_RAD_PER_S above - this file has been bitten
 * by using-before-declaring before, this is that same bug caught at
 * compile time this round instead of shipping it. */
static float g_mag_heading_offset_rad = 0.0f;
static bool g_mag_calibrated = false;

/* How strongly each 200ms control cycle nudges pose_theta_shared toward the
 * compass heading. Deliberately small: the gyro should still dominate
 * moment-to-moment (smooth, fast, low-noise), the compass only slowly
 * drags the estimate back when the gyro has drifted - a standard
 * complementary filter. Starting guess, same "start small" philosophy as
 * Kp/Ki above - watch whether long-run drift is actually reduced before
 * tuning this further. */
#define MAG_FUSION_WEIGHT 0.02f

/* MPU6050 power-on-reset default full-scale range: gyro +/-250deg/s at
 * 131 LSB/(deg/s) (same constant jetson/tools/process_imu_raw.py uses
 * off-device). /imu_raw itself stays raw ints by design - this conversion
 * is only for control_task's internal heading math, which needs radians
 * to add directly to pose_theta_shared. Defined here (before uros_task,
 * which is the first user of it) - not down near imu_read_raw() where it
 * conceptually "belongs", to avoid the exact declaration-order bug this
 * file has been bitten by before (CONTROL_PERIOD_MS, 2026-07-27). */
#define GYRO_RAW_TO_RAD_PER_S (PI_F / (180.0f * 131.0f))

/* Number of samples averaged for the boot-time gyro bias calibration (see
 * g_gyro_z_bias_raw / calibration loop in setup_imu()). Bumped 100->400
 * (2026-08-28) for better real accuracy - random sample noise averages down
 * with the square root of sample count, so 4x the samples is roughly 2x
 * less noise in the final number. Deliberately NOT paired with a settle
 * delay or any change to when sampling starts (see the calibration loop's
 * own comment) - that combination is what caused the earlier regression;
 * this only runs the same already-proven averaging loop longer. 400 @
 * ~10ms apart is ~4s total, a one-time boot cost. */
#define GYRO_CAL_SAMPLES 400

/* Number of independent samples averaged for the boot-time compass
 * heading-zero calibration (see g_mag_heading_offset_rad / the calibration
 * loop at the end of setup_imu()). NOT spaced the same as GYRO_CAL_SAMPLES
 * above - the HMC5883L is configured for a 15Hz data output rate (CONFIG_A
 * = 0x70 in setup_magnetometer), so reading faster than ~67ms apart would
 * just re-read the same cached register value repeatedly, averaging
 * multiple copies of one sample instead of actually reducing noise. 30
 * samples @ 70ms apart is ~2.1s - genuinely independent readings, still a
 * small one-time boot cost. */
#define MAG_CAL_SAMPLES 30
#define MAG_CAL_SAMPLE_PERIOD_MS 70

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

/* Raised to the true hardware ceiling 2026-08-24: confirmed live that PID
 * was maxing out at the old 100/200 cap (out of the LEDC's real 0-255
 * range) with RPM still reading 0 - wheels needed more than that to
 * overcome stall/friction, so the cap itself was the blocker, not a real
 * mechanical problem. VM has had its own dedicated powerbank wire since
 * 2026-07-24 (see README "Power Architecture"), so the original brownout
 * concern this cap was guarding against doesn't apply. No ceiling below
 * hardware max anymore - this bound only exists so a PID overshoot can't
 * cast a value above 255 into the uint32_t duty (undefined/wraps), not to
 * limit real power. */
#define MAX_SAFE_PWM 255.0f

/* No floor as of 2026-08-24 - confirmed the powerbank no longer needs the
 * artificial minimum current draw the old 10.0f floor existed for. True 0
 * is now reachable, so the robot can actually sit fully silent/still
 * instead of a constant faint motor whine at idle. That whine's real cause
 * (never fully root-caused, see README "Lessons Learned"): target_rpm
 * reaching pid_step() as a tiny non-zero float instead of bit-exact 0.0f
 * (e.g. from sync_trim) skips the target_rpm==0.0f bypass above and used
 * to fall through to this floor. Setting the floor itself to 0 removes
 * that symptom too, not just this constant's original intended case - the
 * lower bound stays here (not removed outright) purely so a negative `u`
 * can't underflow the uint32_t cast below, same reasoning as the ceiling. */
#define MIN_SAFE_PWM 0.0f

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
    float max_integral = g_max_i_contribution / g_ki;   /* live-tunable now, see g_kp/g_ki/g_max_i_contribution above */

    *integral += error * dt;
    if (*integral > max_integral)  *integral = max_integral;   /* anti-windup clamp */
    if (*integral < -max_integral) *integral = -max_integral;

    float u = g_kp * error + g_ki * (*integral);

    if (u < MIN_SAFE_PWM)   u = MIN_SAFE_PWM;   /* clamp: floor keeps current draw above the powerbank's auto-shutoff threshold */
    if (u > MAX_SAFE_PWM)   u = MAX_SAFE_PWM;   /* clamp: temporary current-safety ceiling, not the LEDC max */

    return (uint32_t)u;
}

/* Slew limiter (capped PWM change per cycle) REMOVED 2026-08-27 at vịt's
 * explicit request ("no annoying motor power or rotation speed cap") - it
 * existed to limit di/dt on a marginal power rail, but VM has had its own
 * dedicated powerbank wire since 2026-07-24 (see MAX_SAFE_PWM comment
 * above), which was the actual fix for that brownout risk; this was only
 * ever a secondary mitigation on top of it. Real trade-off being accepted:
 * PID output can now jump PWM instantly cycle-to-cycle instead of ramping,
 * which is a small step back toward the original brownout risk if VM's
 * wiring is ever disturbed - worth remembering if brownouts/resets reappear
 * in g_reset_reason/esp32_diag.
 *
 * Encoder read + PID + PWM write, all in ONE task/cycle — replaces the old
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

static void control_task(void *arg) {
    encoder_gpio_init();

    uint32_t snapshot_left, snapshot_right;
    float applied_pwm_left = 0.0f, applied_pwm_right = 0.0f;
    /* Independent per-wheel integrators (restored) - see the speed
     * regulation block below for why this and IMU-based heading trim don't
     * fight each other the way a single shared-average loop did. */
    float integral_left = 0.0f, integral_right = 0.0f;
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

        /* Odometry reset, requested from the dashboard's Reset All button
         * (2026-08-27) - previously the only way to zero the ESP32's own
         * (x,y,theta) was a physical reset. Zeroed here, before this cycle's
         * pose math runs, so the reset takes effect immediately rather than
         * one cycle's worth of stale delta being added to a nonzero base
         * first. */
        if (g_reset_odom_requested) {
            pose_x_shared = 0.0f;
            pose_y_shared = 0.0f;
            pose_theta_shared = 0.0f;
            g_heading_lock_active = false;   /* re-latch fresh against the new zero below */
            g_reset_odom_requested = false;
        }

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
        float dt = CONTROL_PERIOD_MS / 1000.0f;

        /* Heading (2026-08-22): the encoder-differential dtheta below is
         * exactly what causes the documented ~-5 to -16deg wobble on a
         * nominally-straight command - wheel slip/quantization shows up
         * directly as false rotation. Prefer the gyro whenever the most
         * recent IMU read succeeded (no wheel-contact dependency at all);
         * snapshot both shared vars once, matching the pulse-count
         * snapshot pattern above, since uros_task writes them
         * concurrently. Falls back to pure encoder dtheta - the original,
         * already-proven formula - whenever the IMU hasn't initialized or
         * has latched dead (see imu_gz_valid_shared comment above): never
         * blend in a stale/failed gyro reading. */
        bool imu_gz_ok = imu_gz_valid_shared;
        float imu_gz_rad_per_s = imu_gz_rad_per_s_shared;
        float dtheta_encoder = (dist_right - dist_left) / WHEELBASE_M;
        float dtheta = imu_gz_ok ? (imu_gz_rad_per_s * dt) : dtheta_encoder;
        float theta_mid = pose_theta_shared + dtheta / 2.0f;

        pose_x_shared += dist_center * cosf(theta_mid);
        pose_y_shared += dist_center * sinf(theta_mid);
        pose_theta_shared += dtheta;

        /* Magnetometer fusion (2026-08-28): the gyro integration above
         * accumulates drift forever with nothing to correct it. Whenever a
         * valid compass heading is available, slowly drag pose_theta_shared
         * back toward it - complementary filter, see MAG_FUSION_WEIGHT
         * comment above. Angle-wrap-safe (compares the shortest way around
         * the circle, not raw subtraction, so it can't get a bogus huge
         * correction crossing the +/-180deg seam). */
        if (mag_heading_valid_shared) {
            float mag_diff = mag_heading_rad_shared - pose_theta_shared;
            if (mag_diff > PI_F)  mag_diff -= 2.0f * PI_F;
            if (mag_diff < -PI_F) mag_diff += 2.0f * PI_F;
            pose_theta_shared += MAG_FUSION_WEIGHT * mag_diff;
        }

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

        /* Heading lock: latch a target heading the instant we're not being
         * told to actively turn, then keep computing a virtual turn-rate
         * setpoint toward it every cycle - this is what makes the robot
         * fight back against drift/inertia slip even after cmd_vel goes to
         * (0,0), not just while a nonzero command is live. */
        bool commanding_turn = fabsf(angular_z) > ANGZ_TURN_DEADBAND;
        if (commanding_turn) {
            g_heading_lock_active = false;
        } else if (!g_heading_lock_active) {
            g_target_heading_rad = pose_theta_shared;
            g_heading_lock_active = true;
        }

        float angular_z_setpoint = angular_z;
        if (g_heading_lock_active) {
            float heading_error = wrap_angle_rad(g_target_heading_rad - pose_theta_shared);
            if (fabsf(heading_error) > HEADING_LOCK_DEADBAND_RAD) {
                angular_z_setpoint = g_kheading_lock * heading_error;
                if (angular_z_setpoint > g_max_heading_lock_rate)  angular_z_setpoint = g_max_heading_lock_rate;
                if (angular_z_setpoint < -g_max_heading_lock_rate) angular_z_setpoint = -g_max_heading_lock_rate;
            } else {
                angular_z_setpoint = 0.0f;
            }
        }

        /* Heading trim, in RPM units - shifts each wheel's OWN target below,
         * same domain as target_rpm_left/right so it composes with them by
         * plain addition. Computed the same way as before (rate loop vs the
         * IMU, fed by the position-lock setpoint above); what changed is
         * only where it lands - onto two independent per-wheel targets
         * instead of a shared averaged one. */
        float heading_trim_rpm = 0.0f;
        bool commanded_to_move = (fabsf(linear_x) > 0.001f) || (fabsf(angular_z_setpoint) > 0.001f);
        if (imu_gz_valid_shared && commanded_to_move) {
            float heading_rate_error = angular_z_setpoint - imu_gz_rad_per_s_shared;
            heading_trim_rpm = g_kheading * heading_rate_error;
            if (heading_trim_rpm > g_max_heading_trim_rpm)  heading_trim_rpm = g_max_heading_trim_rpm;
            if (heading_trim_rpm < -g_max_heading_trim_rpm) heading_trim_rpm = -g_max_heading_trim_rpm;
        }

        float signed_target_left  = target_rpm_left  - heading_trim_rpm;
        float signed_target_right = target_rpm_right + heading_trim_rpm;

        /* Direction pins, set every cycle from this cycle's target sign -
         * PID/PWM below only ever computes a MAGNITUDE (the single-channel
         * encoders can't report direction either, so actual_rpm is always
         * >=0 too), so direction has to be decided separately, from the
         * signed target above. AIN1=0/AIN2=1 (BIN1=0/BIN2=1) = forward, per
         * the flipped convention set in app_main - reverse is the opposite
         * pair. */
        gpio_set_level(AIN1_PIN, signed_target_left  < 0.0f ? 1 : 0);
        gpio_set_level(AIN2_PIN, signed_target_left  < 0.0f ? 0 : 1);
        gpio_set_level(BIN1_PIN, signed_target_right < 0.0f ? 1 : 0);
        gpio_set_level(BIN2_PIN, signed_target_right < 0.0f ? 0 : 1);

        /* Speed regulation: INDEPENDENT per-wheel PID (restored) - each
         * wheel converges to its OWN (heading-trimmed) target from its OWN
         * measured RPM, with its own integrator. This is what actually
         * fixes the Kp/Kheading coupling a shared-average loop had: Kp/Ki
         * only ever affect how hard a wheel's own loop chases ITS OWN
         * error, and heading correction only ever affects what that target
         * IS (via signed_target_left/right above) - there's no shared
         * output magnitude for the two to fight over, and no ratio-split
         * multiplication that lets Kp inadvertently scale heading
         * authority. A genuine hardware asymmetry (e.g. one motor with more
         * internal friction) is corrected at its actual source too: that
         * wheel's own PID sees its own RPM undershoot and pushes its own
         * PWM harder, independent of the other wheel entirely. */
        float target_pwm_left  = (float)pid_step(fabsf(signed_target_left),  rpm_left_shared,  &integral_left);
        float target_pwm_right = (float)pid_step(fabsf(signed_target_right), rpm_right_shared, &integral_right);

        /* Manual trim, added last - only while actually commanded to move
         * (real target or heading-lock correction, not a genuine stop), so
         * a commanded/watchdog stop still means true PWM=0 on both wheels
         * once heading error is also resolved, trim included. Clamped both
         * ends - trim could otherwise push this below 0 before the
         * uint32_t cast a few lines down, wrapping into a huge PWM value
         * instead of a small negative one. */
        if (commanded_to_move) {
            target_pwm_left  += g_trim_left;
            target_pwm_right += g_trim_right;
        }
        if (target_pwm_left  < 0.0f) target_pwm_left  = 0.0f;
        if (target_pwm_right < 0.0f) target_pwm_right = 0.0f;
        if (target_pwm_left  > MAX_SAFE_PWM) target_pwm_left  = MAX_SAFE_PWM;
        if (target_pwm_right > MAX_SAFE_PWM) target_pwm_right = MAX_SAFE_PWM;

        /* Motor kill switch - STBY is already LOW by this point (see
         * pid_gains_callback), so the TB6612FNG is physically ignoring these
         * values regardless; this just keeps the software side consistent
         * with that instead of quietly windup-ing against an unreachable
         * target. */
        if (g_motor_killed) {
            target_pwm_left  = 0.0f;
            target_pwm_right = 0.0f;
            integral_left = 0.0f;
            integral_right = 0.0f;
        }

        applied_pwm_left  = target_pwm_left;
        applied_pwm_right = target_pwm_right;
        pwm_left_shared  = applied_pwm_left;
        pwm_right_shared = applied_pwm_right;

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)applied_pwm_left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)applied_pwm_right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        /* NOTE (F5): once uros_task's custom transport claims UART0, this
         * won't show up on idf.py monitor anymore - debug via `ros2 topic
         * echo /odom` / `ros2 topic pub /cmd_vel` on the Jetson instead.
         * Left in place since it's harmless and still useful for standalone
         * bring-up before uros_task's transport takes over. */
        ESP_LOGI(TAG, "RPM L=%.1f R=%.1f  PWM L=%.0f R=%.0f  head_trim=%.1f  "
                      "pose x=%.3fm y=%.3fm theta=%.1fdeg",
                 rpm_left_shared, rpm_right_shared, applied_pwm_left, applied_pwm_right,
                 heading_trim_rpm, pose_x_shared, pose_y_shared, pose_theta_shared * 180.0f / PI_F);
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

/* Populated once by i2c_scan_bus() (defined near setup_imu() below),
 * surfaced in /esp32_diag - lets us see what (if anything) actually ACKs
 * on the I2C bus without needing idf.py monitor (which can't run at the
 * same time as the micro-ROS agent, since both want exclusive UART0).
 * "unscanned" means setup_imu() never got far enough to run the scan
 * (bus creation itself failed). Declared up here, not next to
 * i2c_scan_bus() itself, because uros_task() (which reads it for the
 * diag string) is defined earlier in this file than setup_imu() - same
 * forward-reference trap as g_reset_reason above. */
static char g_i2c_scan_result[40] = "unscanned";

/* Which retry attempt (1-5) the IMU wake-up write succeeded on, 0 if never
 * attempted (bus/device setup itself failed), -1 if all 5 attempts failed.
 * Surfaced in /esp32_diag alongside g_i2c_scan_result - if this lands on
 * attempt 2-5 rather than always 1 or always -1, that's a strong signal of
 * a marginal/intermittent physical connection rather than a clean pass/fail. */
static int g_imu_wake_attempts = 0;

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

/* Live PID gain updates (2026-08-27) - the dashboard publishes a plain text
 * command instead of a std_msgs/Float32MultiArray on purpose: this project's
 * micro-ROS build only has bounded-string sequences proven working (see
 * diag_msg/imu_msg), and a dynamic-array message type would need new
 * colcon.meta bounds + a full library rebuild to add mid-session - real risk
 * for zero benefit over just parsing three numbers out of a string. Malformed
 * text (wrong field count, e.g. a stale/partial message) is silently ignored
 * - gains only change on an unambiguous full match, never partially. */
static void pid_gains_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    /* Odometry-reset command shares this same already-proven channel rather
     * than adding a whole new subscription/entity for one rare button click
     * - same reasoning as this file's choice of plain text over a dynamic-
     * array message type above. Checked before the gains parse below so it
     * can never be mistaken for a (malformed) gains string. */
    if (strcmp(msg->data.data, "RESET_ODOM") == 0) {
        g_reset_odom_requested = true;
        return;
    }
    /* Motor kill switch - toggle STBY right here, not in control_task, so
     * it takes effect the instant the message arrives rather than waiting
     * up to one control period. */
    if (strcmp(msg->data.data, "MOTOR_OFF") == 0) {
        g_motor_killed = true;
        gpio_set_level(STBY_PIN, 0);
        return;
    }
    if (strcmp(msg->data.data, "MOTOR_ON") == 0) {
        g_motor_killed = false;
        gpio_set_level(STBY_PIN, 1);
        return;
    }
    float kp, ki, max_i, khead, trim_l, trim_r, max_head, klock, max_lock;
    if (sscanf(msg->data.data, "KP=%f,KI=%f,MAXI=%f,KHEAD=%f,TRIML=%f,TRIMR=%f,MAXHEAD=%f,KLOCK=%f,MAXLOCK=%f",
               &kp, &ki, &max_i, &khead, &trim_l, &trim_r, &max_head, &klock, &max_lock) == 9) {
        g_kp = kp;
        g_ki = ki;
        g_max_i_contribution = max_i;
        g_kheading = khead;
        g_trim_left = trim_l;
        g_trim_right = trim_r;
        g_max_heading_trim_rpm = max_head;
        g_kheading_lock = klock;
        g_max_heading_lock_rate = max_lock;
    }
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
        rcl_subscription_t pid_gains_sub;
        rcl_publisher_t odom_pub;
        rcl_publisher_t diag_pub;
        rcl_publisher_t imu_pub;
        rclc_executor_t executor;
        geometry_msgs__msg__Twist cmd_vel_msg;
        std_msgs__msg__String pid_gains_msg = {0};
        nav_msgs__msg__Odometry odom_msg = {0};
        std_msgs__msg__String diag_msg = {0};
        std_msgs__msg__String imu_msg = {0};
        char pid_gains_text[160];  /* was 64, then 128 -- too small once KHEAD/TRIML/TRIMR
                                     * (and now MAXHEAD, plus RESET_ODOM) were added to the
                                     * KP/KI/MAXI format (2026-08-27): the full field string
                                     * is already ~72+ bytes even at small values, so a too-
                                     * small buffer here means every /pid_gains update gets
                                     * silently truncated and fails pid_gains_callback's exact
                                     * sscanf match -- no gain change reaches the firmware,
                                     * with no error anywhere. Sized with real headroom this
                                     * time so adding one more field later doesn't repeat this. */
        char diag_text[400];   /* grown from 96/112/160/200/280/320 -- added TRIML/TRIMR fields plus
                                 * the existing RPM/PWM/KP/KI/MAXI/KHEAD/I2C/WAKE/GZ/MAG/MX/MY/MZ/
                                 * MAGHDG fields plus g_i2c_scan_result's own 40-char worst case can
                                 * exceed 200. MOTOR (2026-08-28, kill switch) fit within this
                                 * existing headroom, no resize needed. */
        char imu_text[80];

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
        if (rclc_subscription_init_default(&pid_gains_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "pid_gains") != RCL_RET_OK) {
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&odom_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) {
            if (rcl_subscription_fini(&pid_gains_sub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&diag_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "esp32_diag") != RCL_RET_OK) {
            if (rcl_publisher_fini(&odom_pub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&pid_gains_sub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        if (rclc_publisher_init_default(&imu_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "imu_raw") != RCL_RET_OK) {
            if (rcl_publisher_fini(&diag_pub, &node) != RCL_RET_OK) {}
            if (rcl_publisher_fini(&odom_pub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&pid_gains_sub, &node) != RCL_RET_OK) {}
            if (rcl_subscription_fini(&cmd_vel_sub, &node) != RCL_RET_OK) {}
            if (rcl_node_fini(&node) != RCL_RET_OK) {}
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        rclc_executor_t *exec = &executor;
        rclc_executor_init(exec, &support.context, 2, &allocator);
        rclc_executor_add_subscription(exec, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(exec, &pid_gains_sub, &pid_gains_msg, &pid_gains_callback, ON_NEW_DATA);

        /* diag_text's CONTENT is now rebuilt every publish cycle (see the
         * publish block below) so /esp32_diag carries live per-wheel
         * RPM/PWM, not just the reset reason from connection time. The
         * buffer pointer/capacity themselves don't change per-message, so
         * only need to be set once here. pid_gains_msg's buffer likewise -
         * micro-ROS writes each incoming message into this same buffer, it's
         * not allocated fresh per-message. */
        diag_msg.data.data = diag_text;
        diag_msg.data.capacity = sizeof(diag_text);
        imu_msg.data.data = imu_text;
        imu_msg.data.capacity = sizeof(imu_text);
        pid_gains_msg.data.data = pid_gains_text;
        pid_gains_msg.data.capacity = sizeof(pid_gains_text);

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

                /* IMU sanity-check read (2026-08-14) - done inline here
                 * rather than in its own task: I2C at 400kHz for 14 bytes
                 * is on the order of tens of microseconds, negligible next
                 * to this loop's 100ms period, and doing it in-line means
                 * no shared volatile state and no ISR/task race to reason
                 * about at all - unlike pulse_count_left/right, which DO
                 * cross a real task boundary and need portENTER_CRITICAL. */
                int16_t imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz;
                if (imu_read_raw(&imu_ax, &imu_ay, &imu_az,
                                  &imu_gx, &imu_gy, &imu_gz)) {
                    snprintf(imu_text, sizeof(imu_text),
                             "ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
                             imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz);
                    /* gz <-> robot Z (yaw), empirically confirmed 2026-08-22:
                     * positive gz = turning left (CCW from above), matching
                     * control_task's existing encoder-differential dtheta
                     * sign exactly - no sign flip needed. Bias-subtracted
                     * (2026-08-28) - see g_gyro_z_bias_raw comment above for
                     * why an uncorrected zero-rate offset matters here. */
                    imu_gz_rad_per_s_shared = ((float)imu_gz - g_gyro_z_bias_raw) * GYRO_RAW_TO_RAD_PER_S;
                    imu_gz_valid_shared = true;
                } else {
                    snprintf(imu_text, sizeof(imu_text), "imu_read_failed");
                    imu_gz_valid_shared = false;
                }
                imu_msg.data.size = strlen(imu_text);
                if (rcl_publish(&imu_pub, &imu_msg, NULL) != RCL_RET_OK) {
                    agent_ok = false;
                }

                /* Magnetometer (2026-08-28): read alongside the existing
                 * IMU read above, same "inline, no separate task" reasoning
                 * (fast I2C transaction, no shared-state/ISR concerns).
                 * "Calibration" here is deliberately the simple version Alex
                 * asked for, not a full hard-iron spin calibration: take
                 * ONE reading right after boot and treat it as heading-zero.
                 * This works for this system specifically because nothing
                 * here needs a TRUE compass bearing - pose_theta_shared is
                 * already a relative/arbitrary reference (origin =
                 * wherever the robot is at boot, same as the gyro-only
                 * estimate always was), so a fixed, uncorrected hard-iron
                 * offset doesn't cause drift - it just means "heading 0"
                 * isn't true north, which nothing here depends on. Revisit
                 * with a real spin calibration ONLY if live testing shows
                 * the raw heading behaves inconsistently at different
                 * physical headings (the actual symptom of un-compensated
                 * hard-iron distortion) - see README.
                 * NOTE: unlike gz's sign (empirically confirmed 2026-08-22),
                 * this atan2f sign convention has NOT been verified on real
                 * hardware yet - check whether a rising heading value here
                 * actually matches "turning left" before trusting the fused
                 * pose_theta_shared, may need a sign flip. */
                int16_t mag_x = 0, mag_y = 0, mag_z = 0;
                if (mag_read_raw(&mag_x, &mag_y, &mag_z)) {
                    float raw_heading = atan2f((float)mag_y, (float)mag_x);
                    if (!g_mag_calibrated) {
                        g_mag_heading_offset_rad = raw_heading;
                        g_mag_calibrated = true;
                    }
                    float corrected = raw_heading - g_mag_heading_offset_rad;
                    if (corrected > PI_F)  corrected -= 2.0f * PI_F;
                    if (corrected < -PI_F) corrected += 2.0f * PI_F;
                    mag_heading_rad_shared = corrected;
                    mag_heading_valid_shared = true;
                } else {
                    mag_heading_valid_shared = false;
                }

                /* Rebuilt every cycle now (see comment above) - live
                 * per-wheel RPM/PWM for hardware debugging, since
                 * idf.py monitor is unusable once this transport owns
                 * UART0 (see note above cmd_vel_callback). Moved to AFTER
                 * both the IMU and magnetometer reads above (2026-08-28,
                 * was built earlier in this same function before either had
                 * run) so it reports THIS cycle's values, not last cycle's -
                 * and now includes raw MX/MY/MZ + the computed pre-fusion
                 * compass heading (MAGHDG), added specifically to debug the
                 * "heading keeps drifting back to ~17deg" report: this
                 * shows directly whether the raw magnetometer values
                 * actually change as the robot physically rotates (a real
                 * sensor reading real world) or stay put regardless of
                 * orientation (nearby magnetic interference - e.g. the
                 * drive motors' permanent magnets - or a wrong axis in the
                 * atan2f call swamping the real signal). */
                snprintf(diag_text, sizeof(diag_text),
                         "reset=%s MOTOR=%s RPM L=%.1f R=%.1f PWM L=%.0f R=%.0f KP=%.2f KI=%.2f MAXI=%.1f KHEAD=%.1f MAXHEAD=%.1f TRIML=%.1f TRIMR=%.1f KLOCK=%.2f MAXLOCK=%.2f LOCK=%s TGT=%.1f HDGERR=%.1f I2C=%s WAKE=%d GZ=%s GZBIAS=%.1f MAG=%s MX=%d MY=%d MZ=%d MAGHDG=%.1f",
                         reset_reason_str(g_reset_reason),
                         g_motor_killed ? "off" : "on",
                         rpm_left_shared, rpm_right_shared,
                         pwm_left_shared, pwm_right_shared,
                         g_kp, g_ki, g_max_i_contribution, g_kheading, g_max_heading_trim_rpm, g_trim_left, g_trim_right,
                         g_kheading_lock, g_max_heading_lock_rate,
                         g_heading_lock_active ? "on" : "off",
                         g_target_heading_rad * 180.0f / PI_F,
                         wrap_angle_rad(g_target_heading_rad - pose_theta_shared) * 180.0f / PI_F,
                         g_i2c_scan_result, g_imu_wake_attempts,
                         imu_gz_valid_shared ? "ok" : "no-read",
                         g_gyro_z_bias_raw / 131.0f,   /* raw LSB -> deg/s, human-readable */
                         mag_heading_valid_shared ? "ok" : "no-read",
                         mag_x, mag_y, mag_z,
                         mag_heading_rad_shared * 180.0f / PI_F);
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

        if (rcl_publisher_fini(&imu_pub, &node) != RCL_RET_OK) {}
        if (rcl_publisher_fini(&diag_pub, &node) != RCL_RET_OK) {}
        if (rcl_publisher_fini(&odom_pub, &node) != RCL_RET_OK) {}
        if (rcl_subscription_fini(&pid_gains_sub, &node) != RCL_RET_OK) {}
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

/* Set once in app_main(), read (not written) by uros_task() afterward - no
 * cross-task synchronization needed for this handle itself (it's immutable
 * after setup), unlike the encoder pulse counters which DO cross an
 * ISR/task boundary and need portENTER_CRITICAL. */
static i2c_master_dev_handle_t imu_dev = NULL;

/* Magnetometer device handle, same lifecycle/threading rules as imu_dev
 * above - set once by setup_magnetometer() (called from setup_imu(), see
 * below), read-only afterward. g_mag_heading_offset_rad/g_mag_calibrated are
 * declared earlier in the file (near mag_heading_rad_shared) - see the
 * comment there for why. */
static i2c_master_dev_handle_t mag_dev = NULL;

/* One-shot diagnostic: probe every valid 7-bit address (1-126) and record
 * which ones ACK. TEMPORARY debugging aid for the 2026-08-20 imu_read_failed
 * investigation (wiring/power/continuity all confirmed OK by hand, so the
 * remaining question is literally "is anything answering, and at what
 * address" - AD0 tied high would put the MPU6050 at 0x69, not the 0x68
 * this firmware hardcodes). Fine to remove once the IMU is confirmed
 * working - not something to keep running on every boot forever. */
static void i2c_scan_bus(i2c_master_bus_handle_t bus_handle) {
    char *p = g_i2c_scan_result;
    char *end = g_i2c_scan_result + sizeof(g_i2c_scan_result);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(bus_handle, addr, 30) == ESP_OK) {
            int n = snprintf(p, end - p, found ? ",%02X" : "%02X", addr);
            if (n > 0 && p + n < end) {
                p += n;
            }
            found++;
        }
    }
    if (!found) {
        snprintf(g_i2c_scan_result, sizeof(g_i2c_scan_result), "none");
    }
}

/* Bring up the HMC5883L magnetometer on the same I2C bus as the MPU6050.
 * Same soft-fail philosophy as setup_imu() below: a bad/unplugged compass
 * should not take down the already-proven motor/encoder/odom path. Config
 * values are the standard, widely-used HMC5883L init sequence: 8-sample
 * averaging + 15Hz output (Config A), default +/-1.3 Ga gain (Config B),
 * continuous-measurement mode (Mode) - the chip otherwise powers up in
 * single-measurement mode, which would only ever return one reading. */
static void setup_magnetometer(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HMC5883L_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &mag_dev) != ESP_OK) {
        mag_dev = NULL;
        return;
    }
    uint8_t cfg_a[2] = { HMC5883L_REG_CONFIG_A, 0x70 };
    uint8_t cfg_b[2] = { HMC5883L_REG_CONFIG_B, 0x20 };
    uint8_t mode[2]  = { HMC5883L_REG_MODE,     0x00 };
    if (i2c_master_transmit(mag_dev, cfg_a, sizeof(cfg_a), pdMS_TO_TICKS(100)) != ESP_OK ||
        i2c_master_transmit(mag_dev, cfg_b, sizeof(cfg_b), pdMS_TO_TICKS(100)) != ESP_OK ||
        i2c_master_transmit(mag_dev, mode,  sizeof(mode),  pdMS_TO_TICKS(100)) != ESP_OK) {
        mag_dev = NULL;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  /* short settle time after the mode-register write */
}

/* Bring up the I2C bus + MPU6050 device and wake it from sleep.
 * Deliberately NOT ESP_ERROR_CHECK'd end-to-end like setup_gpio/setup_pwm:
 * a bad/unplugged IMU should not crash-loop the whole robot and take the
 * already-proven motor/encoder/odom path down with it. imu_dev stays NULL
 * on any failure; imu_read_raw() checks that and no-ops. */
static void setup_imu(void) {
    /* MPU6050 needs some tens of ms after power-up before it reliably
     * answers on I2C (datasheet power-on-reset timing). setup_gpio()/
     * setup_pwm() above already burn a little time, but not reliably
     * enough to depend on - without this, a cold boot could race the
     * sensor and permanently disable IMU reads for that whole session
     * (see imu_dev = NULL below) even though nothing is actually wrong. */
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = IMU_SCL_PIN,
        .sda_io_num = IMU_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    if (i2c_new_master_bus(&bus_cfg, &bus_handle) != ESP_OK) {
        return;
    }

    /* 2026-08-20: was 400000 (Fast Mode). Real writes (the PWR_MGMT_1 wake
     * command below) were failing while i2c_master_probe() - a much lighter
     * transaction - still ACKed fine at the same address. Classic breadboard
     * symptom: internal ESP32 pull-ups + jumper wire capacitance can't
     * reliably hold Fast Mode timing even though the device is genuinely
     * present and wired correctly. Standard Mode (100kHz) is far more
     * tolerant of exactly this. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_dev) != ESP_OK) {
        imu_dev = NULL;
        return;
    }

    /* Retried rather than one-shot: i2c_master_probe() above already
     * confirmed the device ACKs its address, so if this still fails outright
     * every time, address/wiring isn't the question anymore - only whether
     * a real (multi-byte) transaction can get through at all. Retrying
     * turns "always fails" vs. "succeeds on attempt N" into a diagnostic:
     * the latter points at a marginal/intermittent connection rather than
     * a clean broken/not-broken wiring fault. */
    uint8_t wake_cmd[2] = { MPU6050_REG_PWR_MGMT_1, 0x00 };
    bool wake_ok = false;
    for (int attempt = 1; attempt <= 5; attempt++) {
        if (i2c_master_transmit(imu_dev, wake_cmd, sizeof(wake_cmd),
                                 pdMS_TO_TICKS(100)) == ESP_OK) {
            g_imu_wake_attempts = attempt;
            wake_ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!wake_ok) {
        g_imu_wake_attempts = -1;
        imu_dev = NULL;  /* wiring/address wrong - don't let later reads run */
    }

    /* Gyro zero-rate bias calibration (2026-08-28, reverted 2026-08-28) - see
     * g_gyro_z_bias_raw comment above for the why. Runs once here, blocking,
     * before either task starts - the robot MUST be stationary through this
     * (nothing else is driving it yet at this point in boot, so that's
     * already true). Only meaningful if the IMU actually woke - a dead/
     * absent IMU has nothing to calibrate, g_gyro_z_bias_raw stays at its
     * 0.0f default (imu_gz_valid_shared will be false anyway in that case,
     * so this bias is never even applied to anything).
     *
     * A same-day attempt at "improving" this (a 60ms post-wake settle delay
     * before the first sample, plus two-pass outlier rejection) made the
     * real-hardware result WORSE, not better - confirmed live: ~1.8-1.9deg/s
     * with this plain version below, ~3.8deg/s with the settle-delay
     * version, on the same physical setup. Best guess, not confirmed: the
     * MPU6050's zero-rate output isn't actually stable immediately after
     * wake - it can drift for a bit as the internal oscillator/die
     * temperature settles - and shifting WHEN the sampling window starts
     * (even by 60ms) likely moved it into a worse part of that transient
     * rather than past it. Reverted rather than guessing again with no way
     * to verify on real hardware from here - simple immediate averaging is
     * the version with actual evidence behind it. If this needs revisiting,
     * the settle time would need to be verified empirically (try several
     * delay values against a known-good reading), not just picked. */
    if (wake_ok) {
        float bias_sum = 0.0f;
        int good_samples = 0;
        for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
            int16_t cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz;
            if (imu_read_raw(&cal_ax, &cal_ay, &cal_az, &cal_gx, &cal_gy, &cal_gz)) {
                bias_sum += (float)cal_gz;
                good_samples++;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (good_samples > 0) {
            g_gyro_z_bias_raw = bias_sum / (float)good_samples;
        }
    }

    /* GY-86/87-style boards wire the magnetometer through the MPU6050's own
     * AUX I2C bus, not straight onto the main SDA/SCL lines - confirmed
     * 2026-08-28: a bus scan found the MPU6050 (0x68) and the barometer
     * (0x77) directly, but NOTHING at the magnetometer's address, even
     * though it's the same physical board/wiring. Without this, the
     * compass is electrically invisible to the ESP32 no matter what
     * address setup_magnetometer() probes. Setting I2C_BYPASS_EN makes the
     * MPU6050 pass its aux bus straight through onto the main bus, so the
     * magnetometer becomes directly addressable. Only meaningful if the
     * MPU6050 itself is actually awake and responding. */
    if (wake_ok) {
        uint8_t bypass_cmd[2] = { MPU6050_REG_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN };
        i2c_master_transmit(imu_dev, bypass_cmd, sizeof(bypass_cmd), pdMS_TO_TICKS(100));
        /* Not retried/checked - a failure here is already visible as
         * MAG=no-read on the dashboard, same as any other magnetometer
         * fault, no separate error path needed. */
    }

    /* Scan moved here (2026-08-28, was right after i2c_new_master_bus above)
     * - it used to run BEFORE the bypass write, so I2C=... on the dashboard
     * could never confirm whether bypass actually exposed a new address; it
     * was structurally incapable of answering that question. Now it runs
     * after wake + bypass, so I2C=... reflects the TRUE post-bypass bus
     * state - if the magnetometer still doesn't show up here, bypass mode
     * isn't the fix (wrong address / different chip / real hardware fault),
     * not a diagnostic blind spot. */
    i2c_scan_bus(bus_handle);

    /* Magnetometer (2026-08-28): separate I2C device on the same bus, wired
     * on the new GY-86/87-style board alongside this same MPU6050. Tried
     * unconditionally - it's a different chip at a different address, its
     * fate isn't tied to whether the MPU6050 wake above succeeded. */
    setup_magnetometer(bus_handle);

    /* Compass heading-zero calibration (2026-08-28) - previously "heading
     * zero" was just whatever the FIRST successful read happened to be,
     * lazily captured on uros_task's first control cycle (see the
     * g_mag_calibrated check there) - a single noisy sample, same class of
     * problem the gyro bias calibration above already solved for the gyro.
     * Same fix here: average many samples instead of trusting one - but
     * average the raw X/Y VECTOR components first and take atan2f only
     * once at the end, not the angles themselves (averaging angles needs
     * care around the +/-180deg wrap boundary; averaging the underlying
     * vector sidesteps that entirely and is the mathematically correct way
     * to average a direction anyway). Robot MUST be stationary through
     * this, same requirement as the gyro cal - already true this early in
     * boot. On success, g_mag_calibrated is set true here, so uros_task's
     * own first-good-read fallback below never fires; it's left in place
     * purely as a safety net for the case this loop gets zero good samples
     * (e.g. magnetometer not answering yet at boot but recovering later). */
    {
        float mx_sum = 0.0f, my_sum = 0.0f;
        int good_samples = 0;
        for (int i = 0; i < MAG_CAL_SAMPLES; i++) {
            int16_t cal_mx, cal_my, cal_mz;
            if (mag_read_raw(&cal_mx, &cal_my, &cal_mz)) {
                mx_sum += (float)cal_mx;
                my_sum += (float)cal_my;
                good_samples++;
            }
            vTaskDelay(pdMS_TO_TICKS(MAG_CAL_SAMPLE_PERIOD_MS));
        }
        if (good_samples > 0) {
            g_mag_heading_offset_rad = atan2f(my_sum / (float)good_samples,
                                               mx_sum / (float)good_samples);
            g_mag_calibrated = true;
        }
    }
}

/* Burst-reads accel+temp+gyro (14 bytes from ACCEL_XOUT_H) and splits it
 * into the 6 raw int16 values callers actually want (temp bytes are read
 * and discarded - the sensor only supports reading this block contiguously,
 * there's no cheaper way to skip over temp mid-burst). Big-endian per
 * MPU6050's register layout (MSB byte first for every axis). */
#define IMU_MAX_CONSECUTIVE_FAILURES 5

/* Guards uros_task's control loop from a flaky/loose IMU wire: bounds each
 * stall tightly (a healthy MPU6050 answers in well under 1ms at 400kHz -
 * 5ms is already generous) AND gives up retrying after repeated failures
 * instead of eating that stall every single 100ms cycle forever. Matches
 * the same "don't let IMU trouble touch the proven motor/odom path"
 * philosophy as setup_imu()'s soft-fail. */
static bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz) {
    static uint8_t consecutive_failures = 0;

    if (imu_dev == NULL) {
        return false;
    }
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[IMU_READ_LEN];
    /* 2026-08-20: was 5ms. The wake-up write (a simpler transaction, no
     * repeated-start) succeeds on attempt 1 every time, but this combined
     * write+repeated-start+14-byte-read was failing 100% of the time at
     * 5ms - testing whether that's simply too tight for the more complex
     * transaction on this wiring, not a fully broken connection. */
    if (i2c_master_transmit_receive(imu_dev, &reg, 1, data, sizeof(data),
                                     pdMS_TO_TICKS(100)) != ESP_OK) {
        if (++consecutive_failures >= IMU_MAX_CONSECUTIVE_FAILURES) {
            imu_dev = NULL;  /* bus likely wedged - stop paying the stall */
        }
        return false;
    }
    consecutive_failures = 0;
    *ax = (int16_t)((data[0]  << 8) | data[1]);
    *ay = (int16_t)((data[2]  << 8) | data[3]);
    *az = (int16_t)((data[4]  << 8) | data[5]);
    /* data[6]/data[7] = temperature, unused here */
    *gx = (int16_t)((data[8]  << 8) | data[9]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    *gz = (int16_t)((data[12] << 8) | data[13]);
    return true;
}

/* Burst-reads 6 bytes starting at the X MSB register. HMC5883L's register
 * order is X, Z, Y - not X,Y,Z, confirmed against the datasheet - so the
 * parsing below deliberately doesn't follow the byte order line by line.
 * Same fail-soft/give-up-after-N-failures pattern as imu_read_raw() above. */
#define MAG_MAX_CONSECUTIVE_FAILURES 5

static bool mag_read_raw(int16_t *mx, int16_t *my, int16_t *mz) {
    static uint8_t consecutive_failures = 0;

    if (mag_dev == NULL) {
        return false;
    }
    uint8_t reg = HMC5883L_REG_DATA_X_MSB;
    uint8_t data[HMC5883L_READ_LEN];
    if (i2c_master_transmit_receive(mag_dev, &reg, 1, data, sizeof(data),
                                     pdMS_TO_TICKS(100)) != ESP_OK) {
        if (++consecutive_failures >= MAG_MAX_CONSECUTIVE_FAILURES) {
            mag_dev = NULL;  /* bus likely wedged - stop paying the stall */
        }
        return false;
    }
    consecutive_failures = 0;
    *mx = (int16_t)((data[0] << 8) | data[1]);
    *mz = (int16_t)((data[2] << 8) | data[3]);
    *my = (int16_t)((data[4] << 8) | data[5]);
    return true;
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
    setup_imu();

    /* Sets the electrical default (forward) before control_task starts
     * updating these live every cycle from the actual commanded direction
     * (see the signed_target_left/right block in control_task, fixed
     * 2026-08-26). PWM=0 from a stale/absent /cmd_vel (the safety default)
     * means the motors don't move regardless of what these say.
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
