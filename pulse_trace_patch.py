"""
Per-Tick Pulse Trace Instrumentation for omnilink_mobile_bridge.py
=================================================================

This patch adds JSONL trace emission during the turn pulse/settle loop
to capture the exact data OmniLink requested for root-cause isolation.

APPLY TO: projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py

The patch adds:
1. Trace writer class (near _TranscriptWriter, ~line 244)
2. Motor state reader helper
3. Trace emission in turn branch of tick() (lines 1763-1863)
4. Optional: trace emission in act_turn() for pulse parameters
5. Environment variable gate: OMNILINK_PULSE_TRACE=<path>

Output: JSONL file with one record per physics tick during turn motions.
"""

# ============================================================
# PATCH 1: Add trace writer class (after _TranscriptWriter, ~line 285)
# ============================================================

# +++ INSERT AFTER _TX_WRITER = _TranscriptWriter(_TX_PATH) if _TX_ENABLED else None

# ── Pulse trace (opt-in: OMNILINK_PULSE_TRACE=<path>) ────────────────
#
# WHY THIS EXISTS. The turn pulse/settle loop's adaptive gain converges
# to ~0.17 on Husky under Newton/MuJoCo, but the initial seed is 0.55.
# The first pulse should achieve ~31% of remaining error, then gain
# updates to ~0.17 and subsequent pulses should close the gap. A replay
# showing 10 pulses and only 9.3° total indicates the loop is not
# behaving as designed -- either the settle measurement is wrong, the
# wheel targets aren't achieved, or contact dynamics are stealing yaw.
#
# This emits one JSON line per physics tick during turn motions, capturing:
#   - Phase (plan/pulse/settle)
#   - Requested yaw rate (pulse_w)
#   - Four wheel target velocities (v_left, v_right × 2)
#   - Four achieved wheel velocities (from motor.getVelocity())
#   - Motor internal state (target_vel, actual_vel, force/torque, position)
#   - Chassis yaw, yaw rate (from supervisor pose)
#   - Contact forces / normal loads on each wheel (if available)
#   - Pulse timing (sim time, wall time, duration)
#   - Gain estimate before/after
#
# CONTRACT. Additive, exception-safe, OFF unless OMNILINK_PULSE_TRACE is
# set. A logging fault must never break the control loop.

_PULSE_TX_TAG = "omnilink_mobile_bridge.pulse_trace"
_PULSE_TX_PATH = _os.environ.get("OMNILINK_PULSE_TRACE", "").strip()
_PULSE_TX_ENABLED = bool(_PULSE_TX_PATH)


class _PulseTraceWriter:
    """Append-only JSONL sink for per-tick pulse telemetry. Line-buffered."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._fh = None
        self._lock = threading.Lock()
        self._seq = 0
        self._failures = 0

    def write(self, record: Dict[str, Any]) -> None:
        if not _PULSE_TX_ENABLED:
            return
        try:
            with self._lock:
                self._seq += 1
                record["tick_seq"] = self._seq
                line = json.dumps(record, default=str, ensure_ascii=False)
                if self._fh is None:
                    parent = _os.path.dirname(_os.path.abspath(self._path))
                    if parent:
                        _os.makedirs(parent, exist_ok=True)
                    self._fh = open(self._path, "a", encoding="utf-8", buffering=1)
                self._fh.write(line + "\n")
        except Exception as e:
            try:
                if self._fh is not None:
                    self._fh.close()
            except Exception:
                pass
            self._fh = None
            self._failures += 1
            if self._failures <= 3:
                print(f"[{_PULSE_TX_TAG}] trace write failed: {e!r}")

    def close(self) -> None:
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:
                    pass
                self._fh = None


_PULSE_TX_WRITER = _PulseTraceWriter(_PULSE_TX_PATH) if _PULSE_TX_ENABLED else None


# ============================================================
# PATCH 2: Add helper to read wheel velocities AND motor state
# ============================================================

# +++ INSERT AFTER _command_velocity method (~line 1297)

    def _read_wheel_velocities(self) -> Tuple[List[float], List[float]]:
        """Read current wheel velocities from motor devices.
        Returns (left_vels, right_vels) in rad/s. MUST run on sim thread."""
        left_vels = []
        right_vels = []
        for m in self.left_motors:
            if m is not None:
                try:
                    left_vels.append(float(m.getVelocity()))
                except Exception:
                    left_vels.append(float("nan"))
            else:
                left_vels.append(float("nan"))
        for m in self.right_motors:
            if m is not None:
                try:
                    right_vels.append(float(m.getVelocity()))
                except Exception:
                    right_vels.append(float("nan"))
            else:
                right_vels.append(float("nan"))
        return left_vels, right_vels

    def _read_motor_state(self) -> List[Dict[str, Any]]:
        """Read motor internal state: target_vel, actual_vel, force/torque, position.
        Returns list of dicts for all 4 wheel motors. MUST run on sim thread."""
        states = []
        for m in self.left_motors + self.right_motors:
            if m is not None:
                try:
                    state = {
                        "actual_vel": float(m.getVelocity()),
                    }
                    # Target velocity (if available)
                    if hasattr(m, 'getTargetVelocity'):
                        try:
                            state["target_vel"] = float(m.getTargetVelocity())
                        except Exception:
                            state["target_vel"] = None
                    # Force/torque (if available)
                    if hasattr(m, 'getForce'):
                        try:
                            state["force"] = float(m.getForce())
                        except Exception:
                            state["force"] = None
                    if hasattr(m, 'getTorque'):
                        try:
                            state["torque"] = float(m.getTorque())
                        except Exception:
                            state["torque"] = None
                    # Position (if available)
                    if hasattr(m, 'getPosition'):
                        try:
                            state["position"] = float(m.getPosition())
                        except Exception:
                            state["position"] = None
                    states.append(state)
                except Exception:
                    states.append({})
            else:
                states.append({})
        return states


# ============================================================
# PATCH 3: Instrument the turn branch in tick() (lines 1763-1863)
# ============================================================

# +++ REPLACE THE ENTIRE TURN BRANCH (from "elif kind == \"turn\":" to end of turn block)
#     WITH THIS INSTRUMENTED VERSION:

        elif kind == "turn":
            # Turn in place: PULSE-AND-SETTLE with an adaptive rate gain.
            #
            # Three phases, cycled until the settled error is inside tolerance:
            #   PLAN   — read the SETTLED yaw, compute the residual, and size an
            #            open-loop pulse from the learned gain;
            #   PULSE  — hold a constant yaw rate for that duration. Deliberately
            #            open-loop: the in-motion yaw readback is the thing that
            #            is wrong, so nothing decides when to stop by reading it;
            #   SETTLE — command zero, wait, then re-measure. Settled-to-settled
            #            is the only yaw this loop ever believes.
            now_sim = self.robot.getTime()
            timed_out = ((now_sim - p["t0_sim"]) > p["timeout_s"]
                         if "t0_sim" in p else
                         (time.time() - p["t0"]) > p["timeout_s"])
            phase = p.get("phase", "plan")

            # ─── PULSE TRACE: emit per-tick record during turn ───
            if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
                # Read wheel velocities (on sim thread, safe here)
                left_vels, right_vels = self._read_wheel_velocities()
                # Read motor internal state
                motor_states = self._read_motor_state()
                # Target velocities from current motion state
                target_left = target_right = 0.0
                if phase == "pulse":
                    pulse_w = p.get("pulse_w", 0.0)
                    target_left, target_right = self._command_velocity(0.0, pulse_w)
                # Chassis state
                x, y, yaw = self._read_pose()
                v_linear = self.v_linear
                v_angular = self.v_angular
                # Build trace record
                trace_rec = {
                    "v": 1,
                    "type": "pulse_tick",
                    "ts": time.time(),
                    "sim_t": round(now_sim, 3),
                    "robot": self.robot_id,
                    "phase": phase,
                    "pulse_id": p.get("corrections", 0),
                    "turn_gain": self._turn_gain,
                    "turn_gain_min": self.TURN_GAIN_MIN,
                    "turn_gain_max": self.TURN_GAIN_MAX,
                    "remaining_rad": p.get("remaining", 0.0),
                    "commanded_rad": p.get("commanded", 0.0),
                    "target_yaw": p.get("target_yaw", 0.0),
                    "chassis": {"x": x, "y": y, "yaw": yaw},
                    "v_linear": v_linear,
                    "v_angular": v_angular,
                    "wheel_targets": {
                        "left_radps": target_left,
                        "right_radps": target_right,
                    },
                    "wheel_actual": {
                        "left_radps": left_vels,
                        "right_radps": right_vels,
                    },
                    "motor_states": motor_states,
                    "pulse_params": {
                        "pulse_w": p.get("pulse_w"),
                        "pulse_end_sim": p.get("pulse_end_sim"),
                        "pulse_y0": p.get("pulse_y0"),
                        "pulse_want": p.get("pulse_want"),
                        "settle_t0": p.get("settle_t0"),
                    } if phase in ("pulse", "settle") else None,
                    "settle_elapsed": (now_sim - p["settle_t0"]) if phase == "settle" and "settle_t0" in p else None,
                    "timeout_remaining": p["timeout_s"] - (now_sim - p["t0_sim"]) if "t0_sim" in p else None,
                    "timed_out": timed_out,
                }
                _PULSE_TX_WRITER.write(trace_rec)

            if phase == "pulse":
                if now_sim >= p["pulse_end_sim"] or timed_out:
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "settle"
                        p2["settle_t0"] = now_sim
                        self.motion = ("turn", p2)
                    self._command_velocity(0.0, 0.0)
                else:
                    self._command_velocity(0.0, p["pulse_w"])

            elif phase == "settle":
                self._command_velocity(0.0, 0.0)
                if (now_sim - p["settle_t0"]) >= self.TURN_SETTLE_S or timed_out:
                    # Learn the rate gain from settled truth: how much yaw did a
                    # commanded (w x dt) actually buy? On the Husky under
                    # Newton/MuJoCo this converges to ~0.17 -- i.e. a skid-steer
                    # pivot delivers about a sixth of the commanded rate.
                    got = abs(wrap_pi(yaw - p["pulse_y0"]))
                    want = abs(p["pulse_want"])
                    gain_before = self._turn_gain
                    if want > 1e-6 and got > 1e-4:
                        self._turn_gain = clamp(
                            self._turn_gain * (got / want),
                            self.TURN_GAIN_MIN, self.TURN_GAIN_MAX)
                    gain_after = self._turn_gain

                    # ─── PULSE TRACE: emit gain update record ───
                    if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
                        trace_rec = {
                            "v": 1,
                            "type": "pulse_gain_update",
                            "ts": time.time(),
                            "sim_t": round(now_sim, 3),
                            "robot": self.robot_id,
                            "pulse_id": p.get("corrections", 0),
                            "got_rad": got,
                            "want_rad": want,
                            "ratio": got / want if want > 0 else None,
                            "gain_before": gain_before,
                            "gain_after": gain_after,
                            "settle_elapsed": now_sim - p["settle_t0"],
                        }
                        _PULSE_TX_WRITER.write(trace_rec)

                    stepped = wrap_pi(yaw - p["pulse_y0"])   # settled truth
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "plan"
                        p2["remaining"] = p["remaining"] - stepped
                        p2["corrections"] = p.get("corrections", 0) + 1
                        self.motion = ("turn", p2)

            else:  # "plan"
                # SIGNED REMAINING ROTATION, decremented by each settled delta
                # -- not an absolute target yaw. wrap_pi(target - yaw) makes
                # +180° and -180° the same number, so a half-turn had no defined
                # direction and landed 42° short; accumulated remaining has no
                # antipode and also lets a turn exceed ±π.
                err = p["remaining"]
                done = (abs(err) <= self.TURN_TOL_RAD or timed_out
                        or p.get("corrections", 0) >= self.TURN_MAX_CORRECTIONS)
                if done:
                    with self.lock:
                        self.motion = ("idle", {})
                    self._command_velocity(0.0, 0.0)
                    # `err` is what is LEFT of the commanded rotation, so the
                    # achieved rotation is commanded - remaining. Measured,
                    # settled, and signed -- not the argument echoed back.
                    self._record_completion(
                        p, float(p.get("commanded", 0.0)) - err,
                        settled=not timed_out, timed_out=timed_out)
                    msg = (f"turn complete (yaw err {math.degrees(err):+.2f} deg, "
                           f"{p.get('corrections', 0)} pulses, "
                           f"gain {self._turn_gain:.3f}"
                           f"{', TIMED OUT' if timed_out else ''})")
                    self.queue_window("system:" + msg)
                    # Also to stdout: the window queue reaches the robot GUI
                    # only, so the turn corrector's own verdict was invisible
                    # to anything measuring it -- which is how a -43% actuator
                    # shipped documented as -19%.
                    print(f"[omnilink_mobile_bridge] {self.robot_id}: {msg}",
                          flush=True)

                    # ─── PULSE TRACE: emit turn completion record ───
                    if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
                        trace_rec = {
                            "v": 1,
                            "type": "pulse_completion",
                            "ts": time.time(),
                            "sim_t": round(now_sim, 3),
                            "robot": self.robot_id,
                            "total_pulses": p.get("corrections", 0),
                            "commanded_rad": p.get("commanded", 0.0),
                            "achieved_rad": float(p.get("commanded", 0.0)) - err,
                            "error_rad": err,
                            "final_gain": self._turn_gain,
                            "settled": not timed_out,
                            "timed_out": timed_out,
                        }
                        _PULSE_TX_WRITER.write(trace_rec)

                else:
                    w_mag = (self.spin_speed if abs(err) > self.TURN_SLOW_RAD
                             else self.TURN_SLOW_W)
                    dur = min(abs(err) / max(w_mag * self._turn_gain, 1e-3),
                              self.TURN_PULSE_MAX_S,
                              # Never command a pulse that could rotate past
                              # pi -- the settle phase measures it with
                              # wrap_pi and would read it back negated. The
                              # bound uses the COMMANDED rate, not the learned
                              # gain, so it holds precisely when the gain
                              # estimate is wrong, which is exactly when
                              # over-delivery happens.
                              self.TURN_PULSE_MAX_RAD / max(w_mag, 1e-3))
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "pulse"
                        p2["pulse_w"] = math.copysign(w_mag, err)
                        p2["pulse_end_sim"] = now_sim + dur
                        p2["pulse_y0"] = yaw
                        p2["pulse_want"] = w_mag * self._turn_gain * dur
                        self.motion = ("turn", p2)
                    self._command_velocity(0.0, p2["pulse_w"])

                    # ─── PULSE TRACE: emit pulse start record ───
                    if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
                        trace_rec = {
                            "v": 1,
                            "type": "pulse_start",
                            "ts": time.time(),
                            "sim_t": round(now_sim, 3),
                            "robot": self.robot_id,
                            "pulse_id": p.get("corrections", 0) + 1,
                            "remaining_rad": err,
                            "pulse_w": p2["pulse_w"],
                            "pulse_dur_sim": dur,
                            "pulse_end_sim": p2["pulse_end_sim"],
                            "pulse_y0": yaw,
                            "pulse_want": p2["pulse_want"],
                            "turn_gain": self._turn_gain,
                            "spin_speed": self.spin_speed,
                            "turn_slow_rad": self.TURN_SLOW_RAD,
                            "turn_slow_w": self.TURN_SLOW_W,
                        }
                        _PULSE_TX_WRITER.write(trace_rec)


# ============================================================
# PATCH 4: Also trace act_turn entry parameters (optional)
# ============================================================

# +++ IN act_turn() AFTER THE MOTION DICT IS CREATED (around line 3045), ADD:

            # ─── PULSE TRACE: emit turn command record ───
            if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
                trace_rec = {
                    "v": 1,
                    "type": "turn_command",
                    "ts": time.time(),
                    "sim_t": round(self.robot.getTime(), 3),
                    "robot": self.robot_id,
                    "seq": seq,
                    "commanded_rad": float(angle_rad),
                    "target_yaw": target,
                    "initial_gain": self._turn_gain,
                    "spin_speed": self.spin_speed,
                    "turn_gain_typical": self.TURN_GAIN_TYPICAL,
                    "timeout_s": (20.0 + 3.0 * abs(angle_rad)
                                  / max(self.spin_speed * self.TURN_GAIN_TYPICAL, 1e-3)),
                    "source": source,
                }
                _PULSE_TX_WRITER.write(trace_rec)


# ============================================================
# PATCH 5: Ensure trace writer closes on bridge shutdown (optional)
# ============================================================

# +++ ADD A SHUTDOWN METHOD TO MobileBridge CLASS (e.g., after act_stop)

    def _close_pulse_trace(self) -> None:
        """Close the pulse trace writer if enabled."""
        if _PULSE_TX_ENABLED and _PULSE_TX_WRITER is not None:
            _PULSE_TX_WRITER.close()


# ============================================================
# USAGE
# ============================================================

"""
To enable the trace, set the environment variable before launching OmniSim:

    export OMNILINK_PULSE_TRACE=/path/to/pulse_trace.jsonl

Then run the four-Husky world (your existing replay client).
The trace will contain:

1. turn_command - when /turn is accepted (initial parameters)
2. pulse_start - each time a new pulse is planned (pulse_w, duration, want)
3. pulse_tick - EVERY physics tick during turn (wheel targets/actuals, motor states, chassis pose)
4. pulse_gain_update - after each settle phase (got, want, ratio, gain before/after)
5. pulse_completion - when turn finishes (total pulses, achieved, final gain)

Example analysis:

    import json
    with open("pulse_trace.jsonl") as f:
        for line in f:
            rec = json.loads(line)
            if rec["type"] == "pulse_gain_update":
                print(f"Pulse {rec['pulse_id']}: got={rec['got_rad']:.4f} want={rec['want_rad']:.4f} "
                      f"ratio={rec['ratio']:.3f} gain={rec['gain_before']:.3f}->{rec['gain_after']:.3f}")
            elif rec["type"] == "pulse_tick":
                # Check motor states for saturation
                for i, ms in enumerate(rec.get("motor_states", [])):
                    if ms.get("torque") is not None and abs(ms["torque"]) > 180:  # near effort=200 limit
                        print(f"  Motor {i} torque saturation: {ms['torque']:.1f}")

This will show whether:
- Gain converges to ~0.17 (expected) or stalls at 0.55 / hits 0.08 floor
- got/want ratio matches the 0.31 theoretical value
- Wheel actual velocities match targets (or slip occurs)
- Chassis yaw rate during pulse matches commanded pulse_w
- Settle phase actually waits TURN_SETTLE_S (1.0s) before measuring
- Motor torque saturates at effort limit (200) — smoking gun for effort limit
- Motor torque shows high damping force (kd * velocity) — smoking gun for kd=500
"""

print("Patch created. Apply the +++ sections to omnilink_mobile_bridge.py")
print("Then run with: export OMNILINK_PULSE_TRACE=/path/to/trace.jsonl")