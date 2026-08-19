#!/usr/bin/env python3
"""Convert a raw IMU CSV (from log_imu_raw.py) to physical units and plot it.

Usage:
    python3 process_imu_raw.py <csv_path> [output_prefix]

Outputs:
    <prefix>_processed.csv  -- same rows, plus ax_g/ay_g/az_g/gx_dps/gy_dps/gz_dps
    <prefix>_plot.png       -- accel (g) and gyro (deg/s) vs time, if matplotlib
                                is available (skipped otherwise, CSV still written)

Conversion factors match the MPU6050's power-on-reset default full-scale
range (set once in setup_imu(), never reconfigured -- see motor_f1.c):
accel +/-2g (16384 LSB/g), gyro +/-250 deg/s (131 LSB/(deg/s)).

Reading the output: hold the robot still in a known orientation for a few
seconds, then look at which axis sits at ~-1g (or ~+1g) during that segment
-- that axis is the one gravity is acting along, which is the empirical
answer to "which raw axis is up/down in this orientation". Repeat for 2-3
orientations to build the full mapping instead of trusting the geometric
guess in project notes.
"""
import csv
import sys

ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0


def process(csv_path, prefix):
    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row.get('ax_raw'):
                continue  # imu_read_failed row, no numeric data - skip
            t = float(row['t_sec'])
            ax = int(row['ax_raw']) / ACCEL_LSB_PER_G
            ay = int(row['ay_raw']) / ACCEL_LSB_PER_G
            az = int(row['az_raw']) / ACCEL_LSB_PER_G
            gx = int(row['gx_raw']) / GYRO_LSB_PER_DPS
            gy = int(row['gy_raw']) / GYRO_LSB_PER_DPS
            gz = int(row['gz_raw']) / GYRO_LSB_PER_DPS
            rows.append((t, ax, ay, az, gx, gy, gz))

    if not rows:
        print('No valid rows found (all reads failed?) - nothing to process.')
        return

    out_csv = f'{prefix}_processed.csv'
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_sec', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'])
        writer.writerows(rows)
    print(f'Wrote {len(rows)} rows -> {out_csv}')

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed - skipping plot (pip install matplotlib to enable)')
        return

    t = [r[0] for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    ax1.plot(t, [r[1] for r in rows], label='ax (g)')
    ax1.plot(t, [r[2] for r in rows], label='ay (g)')
    ax1.plot(t, [r[3] for r in rows], label='az (g)')
    ax1.axhline(-1.0, color='gray', linestyle='--', linewidth=0.5)
    ax1.axhline(1.0, color='gray', linestyle='--', linewidth=0.5)
    ax1.set_ylabel('accel (g)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, [r[4] for r in rows], label='gx (deg/s)')
    ax2.plot(t, [r[5] for r in rows], label='gy (deg/s)')
    ax2.plot(t, [r[6] for r in rows], label='gz (deg/s)')
    ax2.set_ylabel('gyro (deg/s)')
    ax2.set_xlabel('time (s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.suptitle('IMU raw sanity check -- which axis sits at ~-1g per orientation segment')
    fig.tight_layout()
    out_png = f'{prefix}_plot.png'
    fig.savefig(out_png, dpi=120)
    print(f'Saved plot -> {out_png}')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 process_imu_raw.py <csv_path> [output_prefix]')
        sys.exit(1)
    csv_path = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else csv_path.rsplit('.', 1)[0]
    process(csv_path, prefix)


if __name__ == '__main__':
    main()
