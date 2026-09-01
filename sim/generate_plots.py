import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rpm_calc import generate_pulse_train, compute_rpm_firmware_logic, compute_rpm_smoothed
from complementary_filter import synthesize_imu, complementary_filter, gyro_only, true_roll_profile
from pi_controller import run_step_response, characterize_step
from connection_fsm import backoff_delay_s

OUT = "/home/claude/wvtcs/docs"
import os
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})

# ---------- Figure 1: RPM stale-timeout bug ----------
rpms = np.arange(20, 100, 2)
pct_zero = []
for rpm in rpms:
    pulses = generate_pulse_train(lambda t, r=rpm: r, t_end_s=8.0, seed=1)
    sample_times = np.arange(0, 8.0, 0.02) * 1e6
    est = compute_rpm_firmware_logic(pulses, sample_times)
    steady = est[len(est)//3:]
    pct_zero.append(100 * np.mean(steady == 0))

fig, ax = plt.subplots(figsize=(9, 4.5))
ax.plot(rpms, pct_zero, color="tab:red", lw=2, marker="o", ms=3)
ax.axvline(40, color="gray", ls="--", lw=1, label="Stale-timeout breakeven (40 RPM)")
ax.axvline(31, color="tab:blue", ls=":", lw=1.5, label="Motor's minimum operating RPM (31, deadband floor)")
ax.set_xlabel("True wheel RPM (steady)")
ax.set_ylabel("% of samples falsely reported as 0 RPM")
ax.set_title("Stale-Timeout Bug: 1.5 s Timeout vs. Single-Magnet Pulse Period")
ax.legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}/fig1_rpm_stale_timeout_bug.png", dpi=150); plt.close(fig)

# ---------- Figure 2: RPM fix validation (raw vs smoothed, fixed timeout) ----------
true_rpm = 31
pulses = generate_pulse_train(lambda t: true_rpm, t_end_s=8.0, seed=1)
sample_times_us = np.arange(0, 8.0, 0.02) * 1e6
sample_times_s = sample_times_us / 1e6
raw_broken = compute_rpm_firmware_logic(pulses, sample_times_us, stale_timeout_us=1_500_000)
raw_fixed = compute_rpm_firmware_logic(pulses, sample_times_us, stale_timeout_us=2_500_000)
smoothed_fixed = compute_rpm_smoothed(pulses, sample_times_us, alpha=0.3, stale_timeout_us=2_500_000)

fig, ax = plt.subplots(figsize=(9, 4.5))
ax.plot(sample_times_s, raw_broken, color="tab:red", lw=1, alpha=0.7, label="Original (1.5 s timeout) -- drops to 0")
ax.plot(sample_times_s, raw_fixed, color="tab:orange", lw=1, alpha=0.8, label="Fixed timeout (2.5 s), raw")
ax.plot(sample_times_s, smoothed_fixed, color="tab:blue", lw=1.8, label="Fixed timeout + EMA smoothing")
ax.axhline(true_rpm, color="black", ls="--", lw=1, label="True RPM (31, steady)")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Reported RPM")
ax.set_title("RPM Estimate at 31 RPM: Before and After the Stale-Timeout Fix")
ax.legend(fontsize=8, loc="lower right")
fig.tight_layout(); fig.savefig(f"{OUT}/fig2_rpm_fix_comparison.png", dpi=150); plt.close(fig)

# ---------- Figure 3: complementary filter tracking ----------
t = np.arange(0, 12, 0.02)
data = synthesize_imu(t, seed=3)
est = complementary_filter(t, data["gyro_rate"], data["accel_roll"], alpha=0.98)
gyro_drift = gyro_only(t, data["gyro_rate"], data["true_roll"][0])

fig, ax = plt.subplots(figsize=(9, 5))
ax.plot(t, data["true_roll"], color="black", lw=1.8, label="True roll angle")
ax.plot(t, data["accel_roll"], ".", ms=2, alpha=0.25, color="tab:orange", label="Raw accelerometer estimate")
ax.plot(t, gyro_drift, color="tab:red", lw=1.2, ls="--", label="Gyro-only integration (drifts)")
ax.plot(t, est, color="tab:blue", lw=1.8, label="Complementary filter (alpha=0.98)")
ax.set_xlabel("Time (s)"); ax.set_ylabel("Roll angle (deg)")
ax.set_title("Complementary Filter vs. Raw Accelerometer vs. Gyro-Only Drift")
ax.legend(fontsize=8, loc="lower right")
fig.tight_layout(); fig.savefig(f"{OUT}/fig3_complementary_filter.png", dpi=150); plt.close(fig)

# ---------- Figure 4: PI controller, original vs retuned gains ----------
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
for sp in [30, 50, 70, 100]:
    r = run_step_response(sp, kp=0.004, ki=0.003, t_end=8.0, seed=7, tau_s=0.2)
    axes[0].plot(r["t"], r["rpm"], lw=1.5, label=f"setpoint {sp}")
axes[0].set_title("Original Gains (Kp=0.004, Ki=0.003)\nAcross the Full Setpoint Range")
axes[0].set_xlabel("Time (s)"); axes[0].set_ylabel("RPM")
axes[0].legend(fontsize=8)

for sp in [30, 50, 70, 100]:
    r = run_step_response(sp, kp=0.004, ki=0.018, t_end=8.0, seed=7, tau_s=0.2)
    axes[1].plot(r["t"], r["rpm"], lw=1.5, label=f"setpoint {sp}")
axes[1].set_title("Retuned Gains (Kp=0.004, Ki=0.018)\nAcross the Full Setpoint Range")
axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("RPM")
axes[1].legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}/fig4_pi_gain_comparison.png", dpi=150); plt.close(fig)

# ---------- Figure 5: zero-setpoint bug ----------
r_bug = run_step_response(0, kp=0.004, ki=0.003, t_end=4.0, seed=7, tau_s=0.2)
r_fix = run_step_response(0, kp=0.004, ki=0.003, t_end=4.0, fix_bug=True, seed=7, tau_s=0.2)
fig, ax = plt.subplots(figsize=(9, 4.5))
ax.plot(r_bug["t"], r_bug["rpm"], color="tab:red", lw=1.8, label="As-written firmware logic (deadband floor always applied)")
ax.plot(r_fix["t"], r_fix["rpm"], color="tab:blue", lw=1.8, label="Fixed (setpoint 0 bypasses the deadband clamp)")
ax.set_xlabel("Time (s)"); ax.set_ylabel("RPM")
ax.set_title("Commanding a 0 RPM Setpoint: Does the Motor Actually Stop?")
ax.legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}/fig5_zero_setpoint_bug.png", dpi=150); plt.close(fig)

# ---------- Figure 6: exponential backoff schedule ----------
fig, ax = plt.subplots(figsize=(8, 4.2))
retries = np.arange(0, 8)
delays = [backoff_delay_s(n) for n in retries]
ax.bar(retries, delays, color="tab:blue")
ax.axhline(5, color="red", ls="--", lw=1.2, label="FR-04 target (5 s)")
ax.set_xlabel("Consecutive failed retry #"); ax.set_ylabel("Backoff delay before next attempt (s)")
ax.set_title("Exponential Backoff Schedule vs. the 5-Second FR-04 Target")
ax.legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{OUT}/fig6_backoff_schedule.png", dpi=150); plt.close(fig)

print("All figures saved to", OUT)
