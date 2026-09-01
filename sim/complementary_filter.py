"""
complementary_filter.py
Reproduces the notebook's complementary filter (Entry 03, alpha = 0.98)
and tests it against synthetic gyro + accelerometer data that includes
the two error sources a real MPU-6050 actually has: gyro bias drift
and accelerometer vibration noise.

Firmware logic being modeled:
    roll_est = alpha * (roll_est + gyro_rate * dt) + (1 - alpha) * accel_roll
"""

import numpy as np

G = 9.81


def true_roll_profile(t):
    """A platform that tilts through a slow S-curve, then holds a steady
    15 degree tilt, representative of a vehicle cornering and then
    holding a bank angle."""
    ramp = 15 * (1 / (1 + np.exp(-2 * (t - 4))))
    return ramp


def synthesize_imu(t, gyro_noise_std=0.5, gyro_bias_dps=0.8,
                    accel_noise_std_g=0.05, vibration_std_g=0.15, seed=None):
    rng = np.random.default_rng(seed)
    true_roll = true_roll_profile(t)
    dt = np.gradient(t)
    true_gyro_rate = np.gradient(true_roll) / dt  # deg/s

    gyro_meas = true_gyro_rate + gyro_bias_dps + rng.normal(0, gyro_noise_std, size=t.shape)

    # Accelerometer measures gravity vector tilt plus noise plus vibration
    # spikes (the real MPU-6050 sees these strongly whenever the motor runs).
    accel_roll_ideal = true_roll
    vibration = rng.normal(0, vibration_std_g, size=t.shape) * 57.3  # rough g->deg coupling
    accel_roll_meas = accel_roll_ideal + rng.normal(0, accel_noise_std_g * 57.3, size=t.shape) + vibration

    return {"t": t, "true_roll": true_roll, "gyro_rate": gyro_meas, "accel_roll": accel_roll_meas}


def complementary_filter(t, gyro_rate, accel_roll, alpha=0.98):
    roll = np.zeros_like(t)
    roll[0] = accel_roll[0]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        gyro_integrated = roll[i - 1] + gyro_rate[i] * dt
        roll[i] = alpha * gyro_integrated + (1 - alpha) * accel_roll[i]
    return roll


def gyro_only(t, gyro_rate, roll0):
    roll = np.zeros_like(t)
    roll[0] = roll0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        roll[i] = roll[i - 1] + gyro_rate[i] * dt
    return roll


if __name__ == "__main__":
    t = np.arange(0, 12, 0.02)  # 50 Hz, matches sensorTask
    data = synthesize_imu(t, seed=3)

    for alpha in [0.90, 0.95, 0.98, 0.995]:
        est = complementary_filter(t, data["gyro_rate"], data["accel_roll"], alpha=alpha)
        rmse = np.sqrt(np.mean((est - data["true_roll"]) ** 2))
        steady_err = np.mean(est[-100:] - data["true_roll"][-100:])
        print(f"alpha={alpha:.3f}  rmse={rmse:5.2f} deg  steady_state_bias={steady_err:+5.2f} deg")

    gyro_drift = gyro_only(t, data["gyro_rate"], data["true_roll"][0])
    print(f"\ngyro-only integration drift at t={t[-1]:.0f}s: "
          f"{gyro_drift[-1] - data['true_roll'][-1]:+.2f} deg (unbounded, grows with time)")
