#!/usr/bin/env python3
"""Generate a mock pulse trace for testing the analyzer."""

import json
from pathlib import Path

# Simulate a trace showing the failure mode: gain stalls at ~0.55, slip ~0.3, yaw rate ratio ~0.15
trace = []

# Turn command
trace.append({
    "v": 1, "type": "turn_command", "ts": 1000.0, "sim_t": 0.0, "robot": "husky_ne",
    "seq": 1, "commanded_rad": 1.5708, "initial_gain": 0.55, "spin_speed": 1.0,
    "turn_gain_typical": 0.17, "timeout_s": 45.0, "source": "external"
})

# 10 pulses, each with similar pattern
for pulse_id in range(1, 11):
    # Pulse start
    trace.append({
        "v": 1, "type": "pulse_start", "ts": 1000.0 + pulse_id * 5, "sim_t": pulse_id * 5.0,
        "robot": "husky_ne", "pulse_id": pulse_id, "pulse_w": 1.0,
        "pulse_dur_sim": 1.57, "pulse_want": 0.86, "turn_gain": 0.55
    })

    # Pulse ticks (10 ticks per pulse at 0.016s)
    for tick in range(10):
        sim_t = pulse_id * 5.0 + tick * 0.016
        # Wheel targets: left=-1.0, right=+1.0 (pure turn)
        # Wheel actuals: only ~30% of target due to slip
        # Chassis yaw rate: only ~15% of commanded due to contact dynamics
        trace.append({
            "v": 1, "type": "pulse_tick", "ts": 1000.0 + sim_t, "sim_t": round(sim_t, 3),
            "robot": "husky_ne", "phase": "pulse", "pulse_id": pulse_id, "turn_gain": 0.55,
            "remaining_rad": 1.57 - pulse_id * 0.16,
            "wheel_targets": {"left_radps": -1.0, "right_radps": 1.0},
            "wheel_actual": {"left_radps": [-0.31], "right_radps": [0.31]},
            "chassis": {"x": 3.0, "y": 3.0, "yaw": 0.0 + pulse_id * 0.016},
            "v_angular": 0.15,  # Only 15% of commanded 1.0
            "pulse_params": {"pulse_w": 1.0, "pulse_end_sim": pulse_id * 5.0 + 1.57,
                           "pulse_y0": 0.0, "pulse_want": 0.86, "settle_t0": None}
        })

    # Gain update - gain barely changes because got/want ratio is wrong
    got = 0.16  # Only ~0.16 rad achieved per pulse
    want = 0.86
    ratio = got / want  # ~0.186
    gain_before = 0.55
    gain_after = max(0.08, min(1.5, gain_before * ratio))  # Clamped to floor quickly
    trace.append({
        "v": 1, "type": "pulse_gain_update", "ts": 1000.0 + pulse_id * 5 + 1.6,
        "sim_t": round(pulse_id * 5.0 + 1.6, 3), "robot": "husky_ne",
        "pulse_id": pulse_id, "got_rad": got, "want_rad": want, "ratio": ratio,
        "gain_before": gain_before, "gain_after": gain_after
    })

    # Settle ticks
    for tick in range(5):
        sim_t = pulse_id * 5.0 + 1.6 + tick * 0.016
        trace.append({
            "v": 1, "type": "pulse_tick", "ts": 1000.0 + sim_t, "sim_t": round(sim_t, 3),
            "robot": "husky_ne", "phase": "settle", "pulse_id": pulse_id, "turn_gain": gain_after,
            "remaining_rad": 1.57 - pulse_id * 0.16,
            "wheel_targets": {"left_radps": 0.0, "right_radps": 0.0},
            "wheel_actual": {"left_radps": [0.0], "right_radps": [0.0]},
            "chassis": {"x": 3.0, "y": 3.0, "yaw": 0.0 + pulse_id * 0.016},
            "v_angular": 0.0,
            "pulse_params": {"pulse_w": 1.0, "pulse_end_sim": pulse_id * 5.0 + 1.57,
                           "pulse_y0": 0.0, "pulse_want": 0.86, "settle_t0": pulse_id * 5.0 + 1.57}
        })

# Completion
trace.append({
    "v": 1, "type": "pulse_completion", "ts": 1050.0, "sim_t": 50.0,
    "robot": "husky_ne", "total_pulses": 10, "commanded_rad": 1.5708,
    "achieved_rad": 0.1627, "error_rad": -1.4081, "final_gain": 0.08,
    "settled": False, "timed_out": True
})

# Write trace
trace_path = Path("/home/jon/Downloads/lear_husky_turn_v83/mock_pulse_trace.jsonl")
with trace_path.open("w") as f:
    for rec in trace:
        f.write(json.dumps(rec) + "\n")

print(f"Mock trace written to {trace_path} ({len(trace)} records)")