"""
rpm_calc.py
Reproduces the firmware's pulse-period RPM measurement exactly as written
in the notebook (Entry 04), so it can be tested against synthetic
hall-effect sensor data instead of just trusted from a bench table.

Firmware logic being modeled:
    pulseInterval_us = time between rising edges (measured in the ISR)
    RPM = 60,000,000 / (pulseInterval_us * PULSES_PER_REV)
    stale timeout: report 0 RPM if no pulse for > 1.5 s
"""

import numpy as np

PULSES_PER_REV = 1
STALE_TIMEOUT_US = 1_500_000


def generate_pulse_train(rpm_profile_fn, t_end_s, jitter_std_frac=0.03, seed=None):
    """
    Walk simulated time forward and emit a hall-effect pulse every time
    the wheel completes one revolution, given a (possibly time-varying)
    RPM profile. jitter_std_frac adds per-pulse timing jitter as a
    fraction of the ideal interval, representing magnet placement
    tolerance and ADC/interrupt latency on real hardware.
    """
    rng = np.random.default_rng(seed)
    t_us = 0.0
    pulse_times_us = [0.0]
    while t_us < t_end_s * 1e6:
        rpm = rpm_profile_fn(t_us / 1e6)
        if rpm <= 0:
            t_us += 20_000  # idle, check again in 20 ms
            continue
        ideal_interval_us = 60_000_000.0 / (rpm * PULSES_PER_REV)
        jitter = rng.normal(0, jitter_std_frac * ideal_interval_us)
        interval_us = max(ideal_interval_us + jitter, 1.0)
        t_us += interval_us
        pulse_times_us.append(t_us)
    return np.array(pulse_times_us)


def compute_rpm_firmware_logic(pulse_times_us, sample_times_us, stale_timeout_us=STALE_TIMEOUT_US):
    """
    Exact port of computeRPM() from the notebook: for each requested
    sample time, look at the most recent pulse interval available at
    that moment and convert it to RPM, applying the 1.5 s stale timeout.
    """
    out = np.zeros(len(sample_times_us))
    pi = 0
    last_pulse_us = 0.0
    interval_us = 0.0

    for i, ts in enumerate(sample_times_us):
        while pi < len(pulse_times_us) and pulse_times_us[pi] <= ts:
            if last_pulse_us > 0:
                interval_us = pulse_times_us[pi] - last_pulse_us
            last_pulse_us = pulse_times_us[pi]
            pi += 1

        if last_pulse_us == 0:
            out[i] = 0.0
        elif ts - last_pulse_us > stale_timeout_us:
            out[i] = 0.0
        elif interval_us == 0:
            out[i] = 0.0
        else:
            out[i] = 60_000_000.0 / (interval_us * PULSES_PER_REV)

    return out


def compute_rpm_smoothed(pulse_times_us, sample_times_us, alpha=0.3, stale_timeout_us=STALE_TIMEOUT_US):
    """
    Same firmware logic, but with a light exponential moving average on
    top, the fix explored in Section 9 after the raw single-interval
    estimate turned out to be noisier than the notebook's bench table
    implied at low RPM.
    """
    raw = compute_rpm_firmware_logic(pulse_times_us, sample_times_us, stale_timeout_us)
    smoothed = np.zeros_like(raw)
    smoothed[0] = raw[0]
    for i in range(1, len(raw)):
        smoothed[i] = alpha * raw[i] + (1 - alpha) * smoothed[i - 1]
    return smoothed


if __name__ == "__main__":
    # Reproduce the Entry 04 bench test: steady RPM, check accuracy
    for true_rpm in [18, 31, 54, 79, 97]:
        pulses = generate_pulse_train(lambda t: true_rpm, t_end_s=5.0, seed=1)
        sample_times = np.arange(0, 5.0, 0.02) * 1e6
        est = compute_rpm_firmware_logic(pulses, sample_times)
        steady = est[len(est) // 2:]  # ignore startup transient
        print(f"true={true_rpm:5.1f} rpm  raw_mean={np.mean(steady):6.2f}  "
              f"raw_std={np.std(steady):5.2f}  error={np.mean(steady)-true_rpm:+5.2f}")
