To: OmniLink Team
From: Lear / OmniLink Agents
Date: 2026-09-07 (updated 2026-09-08)
Re: Husky +90° Turn Root-Cause Evidence — Build 11343e3aa, v8.3 — ROOT CAUSE OPEN, ACTUATOR CONTRACT CLARIFIED

================================================================================
OMNILINK CORRECTIONS (RECEIVED 2026-09-08 04:35)
================================================================================

**Critical corrections to my earlier analysis:**

1. **Actuator contract clarification**: In Newton's actuator contract, `kd` is the **velocity-actuator feedback gain** in `kd × (target_velocity - actual_velocity)`, subject to the effort clamp. It is **NOT** a separate passive braking torque `kd × actual_velocity`. Therefore `kd=500` does **not** by itself prove a 500 Nm damping load. Lowering `kd` or raising `effort` blindly is not a validated fix.

2. **Motor promotion clarification**: `OMNISIM_NEWTON_PROMOTE_SERVO` applies after a **finite `setPosition` command** on a limit-less motor. The logged wheel path is velocity mode (`setVelocity`), so disabling promotion needs evidence that a finite position target was actually issued.

3. **Root cause status**: **OPEN** — the root-cause label must remain open until we have a per-pulse measurement with backend-applied actuator force/torque, wheel angular velocity, wheel-ground contacts, and chassis yaw rate in the same trace.

4. **Collider hypothesis**: Confirmed refuted — enabling the Robot wrapper collider is not the remedy; the descendant wheel colliders are the intended contact path.

**OmniLink's explicit guidance**: "Your trace tooling is useful; the root-cause label just needs to remain open until that measurement exists."

================================================================================
CONFIRMED REPRODUCTION (OMNILINK 2026-09-08)
================================================================================

OmniLink independently reproduced the severe under-turn:
- Commanded: 1.570796 rad (90°)
- Achieved: 0.162920 rad (9.33°)
- Error: -1.407876 rad (-80.67°)
- Corrections: 10 (maxed)
- Settled: false

This matches our original replay exactly.

================================================================================
CURRENT EVIDENCE SUMMARY
================================================================================

| Test | Commanded | Achieved | Ratio | Pulses | Settled |
|------|-----------|----------|-------|--------|---------|
| 90° turn (original) | 90.0° | 9.3° | 10.3% | 10 (maxed) | false |
| Single pulse +0.6 rad/s | 0.605 rad | 0.0034 rad | 0.56% | 1 | — |
| Single pulse + collider | 0.605 rad | 0.0031 rad | 0.51% | 1 | — |

**Key observations (unchanged):**
- Wheel targets reach backend (±1.037 rad/s)
- Actual wheel speeds decay from ~0.97 rad/s to **0.011 rad/s** within 0.1s
- Chassis yaw rate remains ~0 throughout
- Enabling Robot wrapper collider made performance slightly worse (0.51% vs 0.56%)

================================================================================
ACTUATOR CONTRACT CLARIFICATION (CRITICAL)
================================================================================

**Newton velocity actuator model (per OmniLink):**
```
actuator_force = clamp( kd × (target_velocity - actual_velocity), -effort, +effort )
```

**What this means:**
- `kd=500` is the **feedback gain** on velocity error, not a passive damping coefficient
- The actuator applies force proportional to **velocity error**, not velocity
- `effort=200` clamps the **output force**, not the damping
- My earlier claim "kd=500 → 500 Nm damping" was **incorrect**

**What we still don't know:**
- Is the actuator saturating at `effort=200`? (force clamp)
- Is the velocity error `(target - actual)` large enough to drive saturation?
- Are wheel-ground contacts generating reaction forces?
- Is the chassis yaw inertia preventing wheel acceleration?

================================================================================
ROOT CAUSE STATUS: OPEN
================================================================================

| Hypothesis | Status | Evidence Needed |
|------------|--------|-----------------|
| Effort saturation (200 Nm clamp) | 🟡 PLAUSIBLE | Per-pulse actuator force trace |
| Velocity error not closing | 🟡 PLAUSIBLE | Per-pulse target vs actual velocity |
| Wheel slip / contact loss | 🟡 PLAUSIBLE | Wheel-ground contact forces |
| Chassis inertia too high | 🟡 PLAUSIBLE | Chassis yaw rate vs wheel torque |
| Motor promotion bug | 🟢 LOW | Evidence of `setPosition()` call |
| Collider issue | ❌ REFUTED | Collider comparison done |

**OmniLink's required measurement**: One correction pulse with:
- Backend-applied actuator force/torque
- Wheel angular velocity (target vs actual)
- Wheel-ground contacts
- Chassis yaw rate
All in the same trace.

================================================================================
NEXT MEASUREMENT: INSTRUMENTED SINGLE PULSE
================================================================================

Our pulse trace tooling (`pulse_trace_patch.py`) captures per-tick:
- Wheel targets vs actuals (4 motors)
- **Motor internal state**: `target_vel`, `actual_vel`, `torque`, `force`, `position`
- Chassis pose, yaw rate, contact count
- Gain updates, pulse timing

**But we need to add**: Backend-applied actuator force/torque (if available via Newton API).

**Next step**: Run instrumented single pulse with `OMNILINK_PULSE_TRACE=...` and analyze:
1. `motor_states[i].force` — backend-applied actuator force (should show saturation at ±200 if effort-limited)
2. `motor_states[i].torque` — backend-applied torque
3. `motor_states[i].actual_vel` vs `target_vel` — velocity error
4. `contact_count` — wheel-ground contacts during pulse
5. `chassis_yaw_rate` — chassis response

This will separate:
- **Effort saturation** → `force` hits ±200 clamp
- **Velocity error not closing** → `target_vel - actual_vel` stays large
- **Contact/slip** → contact count drops or force doesn't translate to yaw

================================================================================
UPDATED TOOLING STATUS
================================================================================

| Tool | Status | Notes |
|------|--------|-------|
| `pulse_trace_patch.py` | **READY** | Captures motor state per tick; needs backend force/torque if API exists |
| `analyze_pulse_trace.py` | **READY** | Detects torque saturation, damping force; will add effort saturation detection |
| `turn_gain_estimator.py` | **INACTIVE METADATA** | Bridge ignores gain fields |
| `adapter_integration_patch.py` | **INACTIVE METADATA** | Example only |

================================================================================
IMMEDIATE NEXT STEPS
================================================================================

1. **Run instrumented single pulse** with `OMNILINK_PULSE_TRACE=/path/to/trace.jsonl`
2. **Analyze trace** for:
   - Actuator force vs effort limit (saturation at ±200?)
   - Velocity error (target - actual) during pulse
   - Contact count during pulse
   - Chassis yaw rate response
3. **Report findings** to OmniLink — root cause remains open until this measurement exists

================================================================================
FILES DELIVERED (UNCHANGED)
================================================================================

| File | Description |
|------|-------------|
| `pulse_trace_patch.py` | Instrumentation for `omnilink_mobile_bridge.py` with motor telemetry |
| `analyze_pulse_trace.py` | Analyzer for effort saturation, velocity error, contact response |
| `FINDINGS_NOTE.md` | This document (updated) |
| `OMNILINK_REPORT.md` | Polished report (needs update for actuator contract) |
| `turn_gain_estimator.py` | Inactive evidence metadata |
| `adapter_integration_patch.py` | Example adapter integration |

================================================================================
STATUS
================================================================================

**ROOT CAUSE: OPEN** — Actuator contract clarified; root cause not yet isolated
**NEXT MEASUREMENT: INSTRUMENTED SINGLE PULSE** — With motor telemetry + backend force/torque
**COLLIDER HYPOTHESIS: REFUTED** — Confirmed by OmniLink
**MOTOR PROMOTION: UNPROVEN** — Needs evidence of `setPosition()` call
**TOOLING: READY** — Pulse trace captures motor state per tick