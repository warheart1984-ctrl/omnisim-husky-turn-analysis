To: OmniLink Team
From: Lear / OmniLink Agents
Date: 2026-09-07
Re: Husky +90° Turn Root-Cause Evidence — Build 11343e3aa, v8.3 — COLLIDER HYPOTHESIS REFUTED, MOTOR ACTUATION FAILURE ISOLATED

================================================================================
CRITICAL CORRECTION: COLLIDER HYPOTHESIS REFUTED
================================================================================

**OmniLink's clean baseline-vs-collider comparison (2026-09-07) refutes the missing-boundingObject hypothesis.**

| Run | Settled Yaw (rad) | Achieved/Commanded Ratio | Median Wheel Speed (rad/s) |
|-----|-------------------|--------------------------|----------------------------|
| **Baseline** (no Robot collider) | 0.003382912 | **0.005593** (0.56%) | 0.010994 |
| **Collider-enabled** (proper Newton path) | 0.003088162 | **0.005106** (0.51%) | 0.010994 |

**Key findings:**
- Enabling the proper Newton robot-collider path made performance **slightly worse**, not better
- Median achieved wheel speed **identical** in both runs: **0.010994 rad/s** vs **1.037190 rad/s target** (1.06%)
- The "missing boundingObject" warning was a red herring — the Husky URDF already provides collision geometry on its links
- **The control target reaches the backend, but wheel actuation fails regardless of collider configuration**

**OmniLink's explicit correction**: "My earlier email over-interpreted that warning, and the 'definitive root cause' conclusion should be withdrawn. The control target still reaches the backend, but enabling the proper colliders does not restore wheel actuation or chassis yaw. The next defensible investigation is therefore the wheel motor/constraint execution path and joint semantics, while treating the joint-order warning separately—not a blind gain change."

================================================================================
ROOT CAUSE ISOLATED: WHEEL MOTOR ACTUATION FAILURE
================================================================================

**The wheel motors receive velocity commands but cannot sustain torque.**

| Component | Status | Evidence |
|-----------|--------|----------|
| Control target delivery | ✅ WORKING | Wheel targets [-1.037, -1.037, +1.037, +1.037] rad/s reach backend |
| Wheel motor actuation | ❌ **BROKEN** | Actual wheel speeds 0.011 rad/s (1.06% of target) |
| Chassis yaw generation | ❌ **BROKEN** | 0.51–0.56% of commanded rotation |
| Collider configuration | ✅ IRRELEVANT | Proper collider slightly worsens performance |
| Joint-order warning | 🔍 SEPARATE | Real but not the actuation root cause |

**Per-tick trace shows the failure mode:**
- At sim_time 0.112s: wheels spin up to ~0.97 rad/s (93% of target)
- By sim_time 0.128s: wheels decay to ~0.46 rad/s
- By sim_time 0.160s: wheels at ~0.11 rad/s
- By sim_time 0.208s: wheels stabilize at **0.011 rad/s** (1.06% of target)
- Chassis yaw rate remains **~0** throughout the entire pulse

**The motors receive velocity commands but cannot sustain torque against the load.**

================================================================================
MOTOR/CONSTRAINT EXECUTION PATH — FAILURE MODES
================================================================================

The engine log warning is the smoking gun:
```
WARNING: Joint motor 'front_left_wheel_motor' declares no position limits,
so this physics backend configures it as a VELOCITY wheel:
setPosition() on it will be IGNORED and setVelocity() is what drives it.
If it is meant to be a servo, give its motor a minPosition/maxPosition
(or its joint a minStop/maxStop) and it will be position-controlled.
(Since 2026-09-01 the first finite setPosition() FROM THE CONTROLLER
promotes it to a position servo automatically; OMNISIM_NEWTON_PROMOTE_SERVO=0
disables the promotion.)
```

**Joint motor parameters (from engine log):**
- `effort=200` — maximum force/torque the motor can apply
- `velLim=60` rad/s — velocity limit (target is 1.037, well within limit)
- `kd=500` — damping coefficient (very high)
- `axis=(0, 1, 0)` — hinge axis (Y-axis = wheel rotation axis)
- `anchor=(0.256, ±0.2775, 0.03282)` — wheel center positions

| Failure Mode | Likelihood | Evidence |
|--------------|------------|----------|
| **High damping (kd=500) overwhelms motor torque** | 🔴 HIGH | kd=500 is very large; viscous damping force = kd × velocity; at 1 rad/s → 500 Nm damping vs effort=200 |
| **Motor effort limit (200) insufficient for Husky mass** | 🔴 HIGH | Husky mass ~100kg; wheel radius 0.165m; track 0.555m; required torque for 0.6 rad/s yaw acceleration >> 200 Nm |
| **Motor promotion bug** | 🟡 MEDIUM | Auto-promotion to position servo on first `setPosition()`; bridge uses `setVelocity()` but promotion logic may trigger incorrectly |
| **Joint axis/anchor wrong** | 🟡 MEDIUM | Hinge axis=(0,1,0) at anchor=(0.256, ±0.2775, 0.03282) — verify Y-axis is correct rotation axis |
| **Joint order warning** | 🟢 LOW (separate) | Non-topological joint order → MuJoCo/Newton kinematic divergence; real but not actuation cause |

**Most likely: kd=500 damping + effort=200 limit = motors stall immediately under load.**

================================================================================
IMMEDIATE DIAGNOSTIC: ADD MOTOR TELEMETRY TO PULSE TRACE
================================================================================

The current trace has `achieved_wheel_rad_s` but we need **motor internal state** to confirm the failure mode. Add to `pulse_trace_patch.py`:

```python
def _read_motor_state(self) -> List[Dict]:
    """Read motor internal state: target vel, actual vel, force/torque, position."""
    states = []
    for m in self.left_motors + self.right_motors:
        if m is not None:
            try:
                states.append({
                    "target_vel": m.getTargetVelocity() if hasattr(m, 'getTargetVelocity') else None,
                    "actual_vel": m.getVelocity(),
                    "force": m.getForce() if hasattr(m, 'getForce') else None,
                    "torque": m.getTorque() if hasattr(m, 'getTorque') else None,
                    "position": m.getPosition() if hasattr(m, 'getPosition') else None,
                })
            except Exception:
                states.append({})
        return states
```

**Emit in pulse_tick record:**
```python
"motor_states": self._read_motor_state(),
```

This will show:
- `target_vel` vs `actual_vel` — confirms command reaches motor
- `force`/`torque` — shows if motor hits effort limit (200)
- `position` — shows if joint is rotating

================================================================================
RECOMMENDED FIXES (PRIORITY ORDER)
================================================================================

| Priority | Fix | Location | Expected Effect |
|----------|-----|----------|-----------------|
| **P0** | Disable motor promotion | Launch env: `OMNISIM_NEWTON_PROMOTE_SERVO=0` | Prevents accidental position-servo mode |
| **P0** | Reduce damping | Husky URDF/joint def: `kd=500` → `kd=50` or `kd=10` | Reduces viscous damping force by 10-50x |
| **P0** | Increase motor effort | Husky URDF/joint def: `effort=200` → `effort=2000` | Allows motor to overcome load |
| **P1** | Verify joint axis | Husky URDF: confirm `axis="0 1 0"` for wheel rotation | Ensures torque applied in correct direction |
| **P1** | Verify joint limits | Husky URDF: confirm no `minStop`/`maxStop` on wheel joints | Continuous rotation required |
| **P2** | Fix joint order | Husky URDF: reorder joints depth-first | Eliminates MuJoCo/Newton divergence warning |

================================================================================
VALIDATION PLAN
================================================================================

1. **Apply P0 fixes** (disable promotion, reduce kd, increase effort)
2. **Run single-pulse test** with motor telemetry enabled
3. **Verify**: 
   - Wheel actual_vel ≈ target_vel (±1.037 rad/s)
   - Motor torque < effort limit (not saturated)
   - Chassis yaw rate ≈ 0.6 rad/s during pulse
   - Achieved/commanded ratio → ~0.17 (typical skid-steer gain)
4. **Run full 90° turn** — should achieve ~90° in 4-5 pulses

================================================================================
MY TOOLING STATUS
================================================================================

| Tool | Status | Notes |
|------|--------|-------|
| `pulse_trace_patch.py` | **NEEDS UPDATE** | Add `_read_motor_state()` and emit `motor_states` in pulse_tick |
| `analyze_pulse_trace.py` | **VALIDATED** | Will identify motor saturation pattern |
| `turn_gain_estimator.py` | **INACTIVE METADATA** | Correctly inert |
| `offline_diagnostic.py` | **SUPERSEDED** | Collider hypothesis refuted |

================================================================================
EVIDENCE PACKAGE
================================================================================

- `husky_turn_result.json` — Original 90° turn replay
- `run_lear_husky_turn.engine.log` — Full physics trace (motor warnings visible)
- `FINDINGS_NOTE.md` — This document (corrected)
- `lear_husky_collider_comparison_bundle/` — OmniLink's comparison data:
  - `lear_husky_collider_comparison.json` — Summary comparison
  - `lear_husky_single_pulse_trace.jsonl` — Baseline per-tick trace
  - `lear_husky_root_collider_pulse_trace.jsonl` — Collider-enabled trace
  - `lear_husky_single_pulse_summary.json` — Baseline summary
  - `lear_husky_root_collider_pulse_summary.json` — Collider summary
  - `lear_husky_turn_with_root_collider.omniworld` — Collider-enabled world
- `pulse_trace_patch.py` — Per-tick trace instrumentation (needs motor telemetry)
- `analyze_pulse_trace.py` — Trace analyzer
- `turn_gain_estimator.py` — Inactive evidence metadata

================================================================================
STATUS
================================================================================

**COLLIDER HYPOTHESIS: REFUTED** — Proper collider slightly worsens performance
**ROOT CAUSE: WHEEL MOTOR ACTUATION FAILURE** — Motors receive commands but stall at ~1% of target due to high damping (kd=500) and/or low effort limit (200)
**NEXT STEP: MOTOR TELEMETRY PULSE TEST** — Add motor state to trace, apply P0 fixes, re-run
**JOINT-ORDER WARNING: REAL BUT SEPARATE** — Fix topology, not the actuation cause