# OmniLink Husky Turn Actuation Failure — Root Cause Analysis Report

**Date:** 2026-09-07  
**Build:** 11343e3aa (OmniSim v8.3)  
**Physics:** Newton/MuJoCo CPU  
**Machine:** 9722d23d12a3 (RTX 3060 Laptop)  
**Prepared by:** Lear / OmniLink Agents  

---

## Executive Summary

A commanded **+90° turn** on `husky_ne` achieves only **+9.3°** (−80.7° error) after 10 correction pulses.  
**Root cause:** Wheel motors receive velocity commands (±1.037 rad/s) but stall at **~0.011 rad/s (1.06% of target)**.  
**Not a collider issue** — enabling the proper Newton robot-collider path made performance slightly worse (0.51% vs 0.56%).

**Failure mode:** Wheel motor actuation path — motors cannot sustain torque against load due to:
1. **Excessive damping (kd=500)** → viscous force ≈ 500 Nm >> effort limit
2. **Insufficient motor effort (200)** → cannot overcome Husky inertia

---

## Evidence Summary

| Test | Commanded | Achieved | Ratio | Pulses |
|------|-----------|----------|-------|--------|
| 90° turn (original) | 90.0° | 9.3° | 10.3% | 10 (maxed) |
| Single pulse +0.6 rad/s | 0.605 rad | 0.0034 rad | 0.56% | 1 |
| Single pulse + collider | 0.605 rad | 0.0031 rad | 0.51% | 1 |

**Key observation:** Wheel targets reach backend (±1.037 rad/s) but actual wheel speeds decay from ~0.97 rad/s to **0.011 rad/s** within 0.1s. Chassis yaw rate remains ~0 throughout.

---

## Root Cause: Wheel Motor Actuation Failure

### Motor Parameters (from engine log)
```
effort=200          # Max torque/force
velLim=60 rad/s     # Velocity limit (not binding)
kd=500              # Damping coefficient (very high)
axis=(0,1,0)        # Hinge axis (Y-axis = wheel rotation)
```

### Failure Mechanism
| Factor | Effect |
|--------|--------|
| **kd=500** | Viscous damping force = kd × velocity ≈ 500 × 1 = **500 Nm** at 1 rad/s |
| **effort=200** | Motor torque limit = **200 Nm** |
| **Result** | Damping force (500 Nm) > effort limit (200 Nm) → **motor stalls immediately** |

The motors spin up briefly (~0.97 rad/s at 0.112s) then decay to 0.011 rad/s as damping overwhelms available torque.

---

## Refuted Hypothesis: Missing Robot Collider

| Configuration | Settled Yaw | Achieved/Commanded |
|---------------|-------------|-------------------|
| Baseline (no Robot collider) | 0.00338 rad | **0.559%** |
| Explicit root collider | 0.00309 rad | **0.511%** |

Enabling `WorldInfo.newtonRobotColliders TRUE` made performance **worse**, not better. The Husky URDF already provides per-link collision geometry; the Robot-wrapper `boundingObject` warning is a red herring.

---

## Recommended Fixes (Priority Order)

| Priority | Fix | Location | Expected Effect |
|----------|-----|----------|-----------------|
| **P0** | `OMNISIM_NEWTON_PROMOTE_SERVO=0` | Launch environment | Prevents accidental position-servo promotion |
| **P0** | `kd=500` → `kd=50` (or 10) | Husky URDF / joint def | Reduces damping force 10-50× |
| **P0** | `effort=200` → `effort=2000` | Husky URDF / joint def | Allows motor to overcome load |
| **P1** | Verify joint axis = `0 1 0` | Husky URDF | Confirms torque applied in correct direction |
| **P1** | Confirm no `minStop`/`maxStop` | Husky URDF | Continuous rotation required |
| **P2** | Fix joint order (depth-first) | Husky URDF | Eliminates MuJoCo/Newton divergence warning |

---

## Validation Plan

1. **Apply P0 fixes** (disable promotion, reduce kd, increase effort)
2. **Run single-pulse test** with motor telemetry enabled (`OMNILINK_PULSE_TRACE=...`)
3. **Verify:**
   - Wheel `actual_vel` ≈ `target_vel` (±1.037 rad/s)
   - Motor `torque` < effort limit (not saturated)
   - Chassis `yaw_rate` ≈ 0.6 rad/s during pulse
   - Achieved/commanded ratio → ~0.17 (typical skid-steer gain)
4. **Run full 90° turn** — should achieve ~90° in 4-5 pulses

---

## Instrumentation Ready

**Pulse trace instrumentation** (`pulse_trace_patch.py`) captures per-tick:
- Wheel targets vs actuals (4 motors)
- Motor internal state: `target_vel`, `actual_vel`, `torque`, `force`, `position`
- Chassis pose, yaw rate, contact count
- Gain updates, pulse timing

**Analyzer** (`analyze_pulse_trace.py`) detects:
- Torque saturation (effort limit)
- High damping force (kd=500)
- Gain convergence, slip, yaw rate tracking

Enable with: `export OMNILINK_PULSE_TRACE=/path/to/trace.jsonl`

---

## Files Delivered

| File | Description |
|------|-------------|
| `pulse_trace_patch.py` | Instrumentation for `omnilink_mobile_bridge.py` with motor telemetry |
| `analyze_pulse_trace.py` | Analyzer for torque saturation & damping force |
| `FINDINGS_NOTE.md` | Full root cause analysis (this document source) |
| `turn_gain_estimator.py` | Inactive evidence metadata (bridge ignores gain fields) |
| `adapter_integration_patch.py` | Integration example for adapter |

---

## Next Steps for OmniLink

1. **Apply P0 fixes** to Husky URDF/joint definitions
2. **Run instrumented single-pulse test** to confirm motor telemetry shows:
   - `torque` < effort limit (no saturation)
   - `force` (damping) reduced to manageable levels
   - `actual_vel` ≈ `target_vel`
3. **Re-run 90° turn** — expect ~90° in 4-5 pulses

---

**Contact:** Lear / OmniLink Agents  
**Repository:** https://github.com/warheart1984-ctrl/omnisim-husky-turn-analysis