# WVTCS Algorithm Validation

Python reimplementation and stress-testing of four core algorithms from the
Wireless Vehicle Telemetry & Control System engineering notebook: the
hall-effect RPM measurement, the complementary filter, the PI speed
controller, and the WiFi/MQTT reconnection state machine. Two real bugs
were found and fixed; see `docs/WVTCS_Validation_Report.docx` for the
full writeup.

## Structure
- `sim/` — rpm_calc.py, complementary_filter.py, pi_controller.py,
  connection_fsm.py, generate_plots.py
- `docs/` — the report (.docx) and all 6 figures

## Run it
```
cd sim
python3 rpm_calc.py              # RPM stale-timeout bug demo
python3 complementary_filter.py  # alpha sweep + drift comparison
python3 pi_controller.py         # gain comparison + zero-setpoint bug
python3 connection_fsm.py        # FR-04 reconnect scenarios
python3 generate_plots.py        # regenerate all 6 report figures
```

Requires: numpy, matplotlib (`pip3 install numpy matplotlib`)
