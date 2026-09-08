# OmniLink Husky Turn Actuation Failure — Root Cause Analysis Report

**Date:** 2026-09-07 (updated 2026-09-08)  
**Build:** 11343e3aa (OmniSim v8.3)  
**Physics:** Newton/MuJoCo CPU  
**Machine:** 9722d23d12a3 (RTX 3060 Laptop)  
**Prepared by:** Lear / OmniLink Agents  

---

## Executive Summary

A commanded **+90° turn** on `husky_ne` achieves only **+9.3°** (−80.7° error) after 10 correction pulses.  
**Root cause: OPEN** — Wheel motors receive velocity commands (±1.037 rad/s) but stall at **~0.011 rad/s (1.06% of target)**.  
**Not a collider issue** — enabling the proper Newton robot-collider path made performance slightly worse (0.51% vs 0.56%).

**Critical correction from OmniLink (2026-09-08):** My earlier damping/effort analysis was based on an incorrect actuator model. The Newton velocity actuator uses `kd` as a **feedback gain on velocity error**, not a passive damping coefficient. Root cause remains **open** until per-pulse actuator force/torque measurement exists.

---

## Evidence Summary

| Test | Commanded | Achieved | Ratio | Pulses | Settled |
|------|-----------|----------|-------|--------|---------|
| 90° turn (original) | 90.0° | 9.3° | 10.3% | 10 (maxed) | false |
| Single pulse +0.6 rad/s | 0.605 rad | 0.0034 rad | 0.56% | 1 | — |
| Single pulse + collider | 0.605 rad | 0.0031 rad | 0.51% | 1 | — |

**Key observations:**
- Wheel targets reach backend (±1.037 rad/s)
- Actual wheel speeds decay from ~0.97 rad/s to **0.011 rad/s** within 0.1s
- Chassis yaw rate remains ~0 throughout
- Enabling Robot wrapper collider made performance slightly worse (0.51% vs 0.56%)

---

## Actuator Contract Clarification (Critical)

**OmniLink correction (2026-09-08):** My earlier analysis used an incorrect actuator model.

**Newton velocity actuator model:**
```
actuator_force = clamp( kd × (target_velocity - actual_velocity), -effort, +effort )
```

| Parameter | Value | Role |
|-----------|-------|------|
| `kd=500` | Feedback gain on **velocity error** | NOT a passive damping coefficient |
| `effort=200` | Output force **clamp** | Limits actuator force output |
| `velLim=60` | Velocity limit | Not binding (target 1.037 << 60) |

**What this means:**
- `kd=500` is the **feedback gain** on velocity error `(target - actual)`, not a passive damping torque
- The actuator applies force proportional to **velocity error**, not velocity
- `effort=200` clamps the **output force**, not the damping
- My earlier claim "kd=500 → 500 Nm damping" was **incorrect**

**What we still don't know:**
- Is the actuator saturating at `effort=200`? (force clamp)
- Is the velocity error `(target - actual)` large enough to drive saturation?
- Are wheel-ground contacts generating reaction forces?
- Is the chassis yaw inertia preventing wheel acceleration?

---

## Refuted Hypothesis: Missing Robot Collider

| Configuration | Settled Yaw | Achieved/Commanded |
|---------------|-------------|-------------------|
| Baseline (no Robot collider) | 0.00338 rad | **0.559%** |
| Explicit root collider | 0.00309 rad | **0.511%** |

Enabling `WorldInfo.newtonRobotColliders TRUE` made performance **worse**, not better. The Husky URDF already provides per-link collision geometry; the Robot-wrapper `boundingObject` warning is a red herring. **Confirmed by OmniLink.**

---

## Root Cause Status: OPEN

| Hypothesis | Status | Evidence Needed |
|------------|--------|-----------------|
| Effort saturation (200 Nm clamp) | 🟡 PLAUSIBLE | Per-pulse actuator force trace |
| Velocity error not closing | 🟡 PLAUSIBLE | Per-pulse target vs actual velocity |
| Wheel slip / contact loss | 🟡 PLAUSIBLE | Wheel-ground contact forces |
| Chassis inertia too high | 🟡 PLAUSIBLE | Chassis yaw rate vs wheel torque |
| Motor promotion bug | 🟢 LOW | Evidence of `setPosition()` call |
| Collider issue | ❌ REFUTED | Collider comparison done |

**OmniLink's required measurement:** One correction pulse with:
- Backend-applied actuator force/torque
- Wheel angular velocity (target vs actual)
- Wheel-ground contacts
- Chassis yaw rate
All in the same trace.

---

## Next Measurement: Instrumented Single Pulse

Our pulse trace tooling (`pulse_trace_patch.py`) captures per-tick:
- Wheel targets vs actuals (4 motors)
- **Motor internal state**: `target_vel`, `actual_vel`, `torque`, `force`, `position`
- Chassis pose, yaw rate, contact count
- Gain updates, pulse timing

**Next step:** Run instrumented single pulse with `OMNILINK_PULSE_TRACE=...` and analyze:
1. `motor_states[i].force` — backend-applied actuator force (should show saturation at ±200 if effort-limited)
2. `motor_states[i].torque` — backend-applied torque
3. `motor_states[i].actual_vel` vs `target_vel` — velocity error
4. `contact_count` — wheel-ground contacts during pulse
5. `chassis_yaw_rate` — chassis response

This will separate:
- **Effort saturation** → `force` hits ±200 clamp
- **Velocity error not closing** → `target_vel - actual_vel` stays large
- **Contact/slip** → contact count drops or force doesn't translate to yaw

---

## Recommended Next Steps

1. **Run instrumented single pulse** with `OMNILINK_PULSE_TRACE=/path/to/trace.jsonl`
2. **Analyze trace** for:
   - Actuator force vs effort limit (saturation at ±200?)
   - Velocity error (target - actual) during pulse
   - Contact count during pulse
   - Chassis yaw rate response
3. **Report findings** to OmniLink — root cause remains open until this measurement exists

---

## Files Delivered

| File | Description |
|------|-------------|
| `pulse_trace_patch.py` | Instrumentation for `omnilink_mobile_bridge.py` with motor telemetry |
| `analyze_pulse_trace.py` | Analyzer for effort saturation, velocity error, contact response |
| `FINDINGS_NOTE.md` | Full root cause analysis (this document source) |
| `OMNILINK_REPORT.md` | This document |
| `turn_gain_estimator.py` | Inactive evidence metadata (bridge ignores gain fields) |
| `adapter_integration_patch.py` | Example adapter integration |

---

## Status Summary

| Item | Status |
|------|--------|
| **Root Cause** | OPEN — Actuator contract clarified; not yet isolated |
| **Collider Hypothesis** | REFUTED — Confirmed by OmniLink |
| **Motor Promotion** | UNPROVEN — Needs evidence of `setPosition()` call |
| **Tooling** | READY — Pulse trace captures motor state per tick |
| **Next Step** | Instrumented single pulse with motor telemetry |

---

**Repository:** https://github.com/warheart1984-ctrl/omnisim-husky-turn-analysis  
**Contact:** Lear / OmniLink Agents