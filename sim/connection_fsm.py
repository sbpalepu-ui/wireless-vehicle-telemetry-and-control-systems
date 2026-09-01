"""
connection_fsm.py
Reproduces the WiFi/MQTT connection state machine from Entry 06,
including the exponential backoff formula, and tests it against both
a single-drop scenario (what the bench "unplug/replug" test actually
exercises) and a repeated-failure scenario, to check whether the
design actually guarantees the notebook's FR-04 requirement
("reconnect within 5 s of drop") in general, or only in the easy case.
"""

import numpy as np

STATES = ["DISCONNECTED", "WIFI_CONNECTING", "WIFI_CONNECTED",
          "MQTT_CONNECTING", "MQTT_CONNECTED"]

WIFI_CONNECT_TIMEOUT_S = 10.0
MQTT_CONNECT_TIMEOUT_S = 5.0


def backoff_delay_s(retry_count):
    return min(30.0, 1.0 * (2 ** retry_count))


def simulate_reconnect(wifi_available_at_s, mqtt_available_at_s, drop_time_s=0.0,
                        dt=0.05, t_end=40.0):
    t = np.arange(0, t_end, dt)
    state = "DISCONNECTED"
    retry_count = 0
    next_action_t = drop_time_s
    connecting_since = None
    reconnected_t = None

    for ti in t:
        if ti < drop_time_s:
            continue

        if state == "DISCONNECTED" and ti >= next_action_t:
            state = "WIFI_CONNECTING"
            connecting_since = ti

        elif state == "WIFI_CONNECTING":
            if ti >= wifi_available_at_s:
                state = "WIFI_CONNECTED"
            elif ti - connecting_since > WIFI_CONNECT_TIMEOUT_S:
                retry_count += 1
                state = "DISCONNECTED"
                next_action_t = ti + backoff_delay_s(retry_count)

        elif state == "WIFI_CONNECTED":
            state = "MQTT_CONNECTING"
            connecting_since = ti

        elif state == "MQTT_CONNECTING":
            if ti >= mqtt_available_at_s:
                state = "MQTT_CONNECTED"
                reconnected_t = ti
                break
            elif ti - connecting_since > MQTT_CONNECT_TIMEOUT_S:
                retry_count += 1
                state = "DISCONNECTED"
                next_action_t = ti + backoff_delay_s(retry_count)

    return {"reconnected_t": reconnected_t, "retry_count": retry_count,
            "recovery_time_s": (reconnected_t - drop_time_s) if reconnected_t else None}


if __name__ == "__main__":
    print("Case 1: bench-style test, network available immediately (the actual unplug/replug test)")
    r = simulate_reconnect(wifi_available_at_s=0.3, mqtt_available_at_s=0.6, drop_time_s=0.0)
    print(f"  recovery time: {r['recovery_time_s']:.2f} s, retries used: {r['retry_count']}  "
          f"(FR-04 target: under 5 s) -> {'PASS' if r['recovery_time_s'] < 5 else 'FAIL'}")

    print("\nCase 2: router takes 12 s to come back up (a real power-cycle scenario)")
    r2 = simulate_reconnect(wifi_available_at_s=12.0, mqtt_available_at_s=12.3, drop_time_s=0.0)
    print(f"  recovery time: {r2['recovery_time_s']:.2f} s, retries used: {r2['retry_count']}  "
          f"(FR-04 target: under 5 s) -> {'PASS' if r2['recovery_time_s'] < 5 else 'FAIL'}")

    print("\nCase 3: broker briefly overloaded, doesn't accept MQTT connections for 20 s")
    r3 = simulate_reconnect(wifi_available_at_s=0.3, mqtt_available_at_s=20.0, drop_time_s=0.0)
    print(f"  recovery time: {r3['recovery_time_s']:.2f} s, retries used: {r3['retry_count']}  "
          f"(FR-04 target: under 5 s) -> {'PASS' if r3['recovery_time_s'] < 5 else 'FAIL'}")

    print("\nBackoff schedule after repeated failures:")
    for n in range(6):
        print(f"  retry {n}: wait {backoff_delay_s(n):.1f} s")
