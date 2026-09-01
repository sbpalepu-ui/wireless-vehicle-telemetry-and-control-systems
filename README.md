# WVTCS Engineering Notebook & Algorithm Validation

Engineering notebook for a wireless vehicle telemetry and control system
built around an ESP32 — sensor integration, FreeRTOS firmware architecture,
WiFi/MQTT telemetry, a PI speed controller, and a browser dashboard, plus
Python code that reimplements and stress-tests four of the notebook's core
algorithms (hall-effect RPM measurement, complementary filter, PI speed
controller, WiFi/MQTT reconnection state machine) against synthetic sensor
data beyond what the original bench tests covered. That stress-testing
found two real bugs — a stale-timeout that falsely reports 0 RPM near the
motor's minimum operating speed, and a controller deadband that prevents
the motor from actually stopping at a 0 RPM setpoint — both fixed in code.

See [`docs/WVTCS_Engineering_Notebook.pdf`](docs/WVTCS_Engineering_Notebook.pdf)
(viewable on GitHub; `.docx` also included) for the full notebook: system
design, wiring, bench data, and requirements verification.

## Structure
- `sim/` — rpm_calc.py, complementary_filter.py, pi_controller.py,
  connection_fsm.py, generate_plots.py
- `docs/` — the engineering notebook (PDF + .docx)

## Run it
```
cd sim
python3 rpm_calc.py              # RPM stale-timeout bug demo
python3 complementary_filter.py  # alpha sweep + drift comparison
python3 pi_controller.py         # gain comparison + zero-setpoint bug
python3 connection_fsm.py        # FR-04 reconnect scenarios
python3 generate_plots.py        # regenerate validation plots
```

Requires: numpy, matplotlib (`pip3 install numpy matplotlib`)
