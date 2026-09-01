"""
pi_controller.py
A first-order DC motor plant model fit to the notebook's bench data
(Entry 07), driven by a PI speed controller that replicates the
firmware's pseudocode exactly (Entry 08), including the deadband floor
clamp and conditional anti-windup, so the control logic can be tested
in closed loop instead of only checked against a single bench step
response.
"""

import numpy as np

DEADBAND_DUTY = 0.28
MAX_DUTY = 1.0

# Steady-state duty -> RPM fit from the Entry 07 bench table
# (30%,31) (50%,54) (75%,79) (100%,97)
_DUTY_PTS = np.array([0.30, 0.50, 0.75, 1.00])
_RPM_PTS = np.array([31, 54, 79, 97])


def motor_steady_state_rpm(duty):
    if duty < DEADBAND_DUTY:
        return 0.0
    return float(np.interp(duty, _DUTY_PTS, _RPM_PTS))


class MotorPlant:
    """First-order lag model: real motors don't reach steady-state RPM
    instantly, inertia and back-EMF give it a time constant."""

    def __init__(self, tau_s=0.45):
        self.rpm = 0.0
        self.tau = tau_s

    def step(self, duty, dt):
        target = motor_steady_state_rpm(duty)
        self.rpm += (target - self.rpm) / self.tau * dt
        self.rpm = max(self.rpm, 0.0)
        return self.rpm


class PIController:
    """Direct port of the Entry 08 pseudocode."""

    def __init__(self, kp, ki, fix_zero_setpoint_bug=False):
        self.kp = kp
        self.ki = ki
        self.i_term = 0.0
        self.fix_zero_setpoint_bug = fix_zero_setpoint_bug

    def step(self, setpoint, measured_rpm, dt):
        error = setpoint - measured_rpm

        if self.fix_zero_setpoint_bug and setpoint <= 0:
            # Fixed behavior: a zero setpoint should mean the motor is
            # actually commanded off, not clamped to the deadband floor.
            self.i_term = 0.0
            return 0.0

        p_term = self.kp * error
        unclamped = p_term + self.i_term + self.ki * error * dt

        output = min(max(unclamped, DEADBAND_DUTY), MAX_DUTY)

        # Conditional anti-windup: only integrate if doing so wouldn't
        # push further into the direction that's already saturated.
        saturated_high = output >= MAX_DUTY and error > 0
        saturated_low = output <= DEADBAND_DUTY and error < 0
        if not (saturated_high or saturated_low):
            self.i_term += self.ki * error * dt

        return output


def run_step_response(setpoint, kp, ki, t_end=6.0, dt=0.02, fix_bug=False, seed=None, tau_s=0.45):
    rng = np.random.default_rng(seed)
    plant = MotorPlant(tau_s=tau_s)
    ctrl = PIController(kp, ki, fix_zero_setpoint_bug=fix_bug)

    t = np.arange(0, t_end, dt)
    rpm_log = np.zeros_like(t)
    duty_log = np.zeros_like(t)

    for i, ti in enumerate(t):
        measured = plant.rpm + rng.normal(0, 0.6)  # encoder measurement noise
        duty = ctrl.step(setpoint, measured, dt)
        plant.step(duty, dt)
        rpm_log[i] = plant.rpm
        duty_log[i] = duty

    return {"t": t, "rpm": rpm_log, "duty": duty_log}


def characterize_step(t, rpm, setpoint):
    if setpoint == 0:
        return None
    try:
        t10 = t[np.argmax(rpm >= 0.1 * setpoint)]
        t90 = t[np.argmax(rpm >= 0.9 * setpoint)]
        rise_time = t90 - t10
    except Exception:
        rise_time = None
    overshoot_pct = max(0.0, (np.max(rpm) - setpoint) / setpoint * 100)
    band = 0.05 * setpoint
    settled = np.abs(rpm - setpoint) <= band
    settle_idx = None
    for i in range(len(t)):
        if np.all(settled[i:]):
            settle_idx = i
            break
    settle_time = t[settle_idx] if settle_idx is not None else None
    steady_state_err = np.mean(rpm[-50:]) - setpoint
    return {
        "rise_time_s": rise_time, "overshoot_pct": overshoot_pct,
        "settle_time_s": settle_time, "steady_state_err": steady_state_err,
    }


if __name__ == "__main__":
    print("=" * 60)
    print("Step response, Kp=0.004, Ki=0.003 (notebook's selected gains)")
    print("=" * 60)
    for sp in [30, 50, 70, 100]:
        r = run_step_response(sp, kp=0.004, ki=0.003, seed=7)
        c = characterize_step(r["t"], r["rpm"], sp)
        print(f"setpoint={sp:4d}  rise={c['rise_time_s']:.2f}s  "
              f"overshoot={c['overshoot_pct']:.1f}%  settle={c['settle_time_s']}s  "
              f"ss_err={c['steady_state_err']:+.2f} rpm")

    print("\n" + "=" * 60)
    print("Zero-setpoint test (does the motor actually stop?)")
    print("=" * 60)
    r_bug = run_step_response(0, kp=0.004, ki=0.003, t_end=4.0, seed=7)
    r_fix = run_step_response(0, kp=0.004, ki=0.003, t_end=4.0, fix_bug=True, seed=7)
    print(f"as-written firmware logic: final RPM = {r_bug['rpm'][-1]:.1f}, "
          f"final commanded duty = {r_bug['duty'][-1]:.2f}")
    print(f"with fix applied:         final RPM = {r_fix['rpm'][-1]:.1f}, "
          f"final commanded duty = {r_fix['duty'][-1]:.2f}")
