#!/usr/bin/env python3
"""
Pulse Trace Analyzer
====================

Parses the JSONL pulse trace from omnilink_mobile_bridge and produces
a root-cause summary for the turn gain convergence failure.

Usage:
    python analyze_pulse_trace.py /path/to/pulse_trace.jsonl
    python analyze_pulse_trace.py /path/to/pulse_trace.jsonl --output report.md
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class PulseTick:
    sim_t: float
    phase: str
    pulse_id: int
    turn_gain: float
    remaining_rad: float
    wheel_targets: dict
    wheel_actual: dict
    chassis: dict
    v_angular: float
    motor_states: list
    pulse_params: Optional[dict] = None


@dataclass
class PulseGainUpdate:
    sim_t: float
    pulse_id: int
    got_rad: float
    want_rad: float
    ratio: float
    gain_before: float
    gain_after: float


@dataclass
class PulseStart:
    sim_t: float
    pulse_id: int
    pulse_w: float
    pulse_dur_sim: float
    pulse_want: float
    turn_gain: float


@dataclass
class TurnCommand:
    sim_t: float
    seq: int
    commanded_rad: float
    initial_gain: float
    spin_speed: float
    turn_gain_typical: float


@dataclass
class PulseCompletion:
    sim_t: float
    total_pulses: int
    commanded_rad: float
    achieved_rad: float
    error_rad: float
    final_gain: float
    settled: bool
    timed_out: bool


@dataclass
class TraceAnalysis:
    turn_command: Optional[TurnCommand] = None
    pulse_starts: list[PulseStart] = field(default_factory=list)
    pulse_ticks: list[PulseTick] = field(default_factory=list)
    gain_updates: list[PulseGainUpdate] = field(default_factory=list)
    completion: Optional[PulseCompletion] = None

    # Derived metrics
    gain_convergence: list[float] = field(default_factory=list)
    slip_ratios: list[float] = field(default_factory=list)
    yaw_rate_ratios: list[float] = field(default_factory=list)
    motor_torque_saturation: list[dict] = field(default_factory=list)
    motor_damping_force: list[dict] = field(default_factory=list)


def parse_trace(trace_path: Path) -> TraceAnalysis:
    """Parse JSONL trace into structured analysis object."""
    analysis = TraceAnalysis()

    with trace_path.open() as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"Warning: line {line_num}: JSON decode error: {e}", file=sys.stderr)
                continue

            rtype = rec.get("type")
            if rtype == "turn_command":
                analysis.turn_command = TurnCommand(
                    sim_t=rec.get("sim_t", 0.0),
                    seq=rec.get("seq", 0),
                    commanded_rad=rec.get("commanded_rad", 0.0),
                    initial_gain=rec.get("initial_gain", 0.0),
                    spin_speed=rec.get("spin_speed", 0.0),
                    turn_gain_typical=rec.get("turn_gain_typical", 0.0),
                )
            elif rtype == "pulse_start":
                analysis.pulse_starts.append(PulseStart(
                    sim_t=rec.get("sim_t", 0.0),
                    pulse_id=rec.get("pulse_id", 0),
                    pulse_w=rec.get("pulse_w", 0.0),
                    pulse_dur_sim=rec.get("pulse_dur_sim", 0.0),
                    pulse_want=rec.get("pulse_want", 0.0),
                    turn_gain=rec.get("turn_gain", 0.0),
                ))
            elif rtype == "pulse_tick":
                analysis.pulse_ticks.append(PulseTick(
                    sim_t=rec.get("sim_t", 0.0),
                    phase=rec.get("phase", ""),
                    pulse_id=rec.get("pulse_id", 0),
                    turn_gain=rec.get("turn_gain", 0.0),
                    remaining_rad=rec.get("remaining_rad", 0.0),
                    wheel_targets=rec.get("wheel_targets", {}),
                    wheel_actual=rec.get("wheel_actual", {}),
                    chassis=rec.get("chassis", {}),
                    v_angular=rec.get("v_angular", 0.0),
                    motor_states=rec.get("motor_states", []),
                    pulse_params=rec.get("pulse_params"),
                ))
            elif rtype == "pulse_gain_update":
                analysis.gain_updates.append(PulseGainUpdate(
                    sim_t=rec.get("sim_t", 0.0),
                    pulse_id=rec.get("pulse_id", 0),
                    got_rad=rec.get("got_rad", 0.0),
                    want_rad=rec.get("want_rad", 0.0),
                    ratio=rec.get("ratio", 0.0),
                    gain_before=rec.get("gain_before", 0.0),
                    gain_after=rec.get("gain_after", 0.0),
                ))
                analysis.gain_convergence.append(rec.get("gain_after", 0.0))
            elif rtype == "pulse_completion":
                analysis.completion = PulseCompletion(
                    sim_t=rec.get("sim_t", 0.0),
                    total_pulses=rec.get("total_pulses", 0),
                    commanded_rad=rec.get("commanded_rad", 0.0),
                    achieved_rad=rec.get("achieved_rad", 0.0),
                    error_rad=rec.get("error_rad", 0.0),
                    final_gain=rec.get("final_gain", 0.0),
                    settled=rec.get("settled", False),
                    timed_out=rec.get("timed_out", False),
                )

    # Compute derived metrics from pulse_ticks
    for tick in analysis.pulse_ticks:
        if tick.phase == "pulse":
            # Slip ratio: actual wheel velocity / target wheel velocity
            targets = tick.wheel_targets
            actuals = tick.wheel_actual
            if targets and actuals:
                t_left = targets.get("left_radps", 0.0)
                t_right = targets.get("right_radps", 0.0)
                a_left = actuals.get("left_radps", [0.0])[0] if actuals.get("left_radps") else 0.0
                a_right = actuals.get("right_radps", [0.0])[0] if actuals.get("right_radps") else 0.0
                if abs(t_left) > 1e-3 or abs(t_right) > 1e-3:
                    slip_left = a_left / t_left if abs(t_left) > 1e-3 else 1.0
                    slip_right = a_right / t_right if abs(t_right) > 1e-3 else 1.0
                    analysis.slip_ratios.append((slip_left + slip_right) / 2)

            # Yaw rate ratio: actual chassis yaw rate / commanded pulse_w
            pulse_w = tick.pulse_params.get("pulse_w") if tick.pulse_params else None
            if pulse_w and abs(pulse_w) > 1e-3:
                analysis.yaw_rate_ratios.append(tick.v_angular / pulse_w)

            # Motor torque saturation check
            for i, ms in enumerate(tick.motor_states):
                torque = ms.get("torque")
                force = ms.get("force")
                if torque is not None and abs(torque) > 180:  # near effort=200 limit
                    analysis.motor_torque_saturation.append({
                        "tick_sim_t": tick.sim_t,
                        "motor_idx": i,
                        "torque": torque,
                        "phase": tick.phase,
                    })
                # Damping force check: kd * velocity
                if force is not None and abs(force) > 100:  # arbitrary threshold
                    analysis.motor_damping_force.append({
                        "tick_sim_t": tick.sim_t,
                        "motor_idx": i,
                        "force": force,
                        "phase": tick.phase,
                    })

    return analysis


def format_report(analysis: TraceAnalysis, trace_path: Path) -> str:
    """Generate a markdown report from the analysis."""
    lines = []
    lines.append("# Pulse Trace Analysis Report")
    lines.append(f"**Trace file:** `{trace_path}`")
    lines.append(f"**Generated:** {__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()}")
    lines.append("")

    # Turn Command
    if analysis.turn_command:
        tc = analysis.turn_command
        lines.append("## Turn Command")
        lines.append(f"- **Commanded:** {math.degrees(tc.commanded_rad):.2f}° ({tc.commanded_rad:.4f} rad)")
        lines.append(f"- **Initial gain:** {tc.initial_gain:.3f} (typical: {tc.turn_gain_typical:.3f})")
        lines.append(f"- **Spin speed:** {tc.spin_speed:.3f} rad/s ({math.degrees(tc.spin_speed):.1f}°/s)")
        lines.append(f"- **Sequence:** {tc.seq}")
        lines.append("")

    # Pulse Starts
    if analysis.pulse_starts:
        lines.append("## Pulse Starts")
        lines.append("| Pulse | pulse_w (rad/s) | Duration (s) | Want (rad) | Gain |")
        lines.append("|-------|----------------|--------------|------------|------|")
        for ps in analysis.pulse_starts:
            lines.append(f"| {ps.pulse_id} | {ps.pulse_w:.4f} | {ps.pulse_dur_sim:.3f} | {ps.pulse_want:.4f} | {ps.turn_gain:.3f} |")
        lines.append("")

    # Gain Convergence
    if analysis.gain_updates:
        lines.append("## Gain Convergence")
        lines.append("| Pulse | Got (rad) | Want (rad) | Ratio | Gain Before | Gain After |")
        lines.append("|-------|-----------|------------|-------|-------------|------------|")
        for gu in analysis.gain_updates:
            lines.append(f"| {gu.pulse_id} | {gu.got_rad:.4f} | {gu.want_rad:.4f} | {gu.ratio:.3f} | {gu.gain_before:.3f} | {gu.gain_after:.3f} |")
        lines.append("")

        # Convergence summary
        if analysis.gain_convergence:
            lines.append("### Gain Convergence Summary")
            lines.append(f"- **Initial:** {analysis.gain_convergence[0]:.3f}")
            lines.append(f"- **Final:** {analysis.gain_convergence[-1]:.3f}")
            lines.append(f"- **Typical (expected):** 0.170")
            lines.append(f"- **Min/Max bounds:** 0.080 / 1.500")
            lines.append("")

    # Slip Analysis
    if analysis.slip_ratios:
        lines.append("## Wheel Slip Analysis (Pulse Phase)")
        avg_slip = sum(analysis.slip_ratios) / len(analysis.slip_ratios)
        min_slip = min(analysis.slip_ratios)
        max_slip = max(analysis.slip_ratios)
        lines.append(f"- **Samples:** {len(analysis.slip_ratios)}")
        lines.append(f"- **Average slip ratio:** {avg_slip:.3f} (1.0 = no slip)")
        lines.append(f"- **Min:** {min_slip:.3f}")
        lines.append(f"- **Max:** {max_slip:.3f}")
        if avg_slip < 0.9:
            lines.append("- ⚠️ **Significant wheel slip detected** — wheels not achieving target velocities")
        elif avg_slip > 1.1:
            lines.append("- ⚠️ **Wheel overspeed** — actual > target (unexpected)")
        else:
            lines.append("- ✅ Wheel velocities tracking targets well")
        lines.append("")

    # Yaw Rate Tracking
    if analysis.yaw_rate_ratios:
        lines.append("## Chassis Yaw Rate Tracking (Pulse Phase)")
        avg_yaw = sum(analysis.yaw_rate_ratios) / len(analysis.yaw_rate_ratios)
        min_yaw = min(analysis.yaw_rate_ratios)
        max_yaw = max(analysis.yaw_rate_ratios)
        lines.append(f"- **Samples:** {len(analysis.yaw_rate_ratios)}")
        lines.append(f"- **Average yaw rate ratio:** {avg_yaw:.3f} (1.0 = perfect tracking)")
        lines.append(f"- **Min:** {min_yaw:.3f}")
        lines.append(f"- **Max:** {max_yaw:.3f}")
        if avg_yaw < 0.5:
            lines.append("- 🔴 **Severe yaw rate deficit** — chassis rotating far slower than commanded")
        elif avg_yaw < 0.8:
            lines.append("- ⚠️ **Yaw rate deficit** — chassis rotating slower than commanded")
        else:
            lines.append("- ✅ Chassis yaw rate tracking commanded rate")
        lines.append("")

    # Motor Torque Saturation
    if analysis.motor_torque_saturation:
        lines.append("## Motor Torque Saturation (Effort Limit)")
        lines.append(f"- **Saturation events:** {len(analysis.motor_torque_saturation)}")
        for sat in analysis.motor_torque_saturation[:5]:
            lines.append(f"  - Tick {sat['tick_sim_t']:.3f}s: Motor {sat['motor_idx']} torque={sat['torque']:.1f} (effort limit ~200)")
        if len(analysis.motor_torque_saturation) > 5:
            lines.append(f"  ... and {len(analysis.motor_torque_saturation) - 5} more")
        lines.append("- 🔴 **Motor torque hitting effort limit (200)** — motors cannot overcome load")
        lines.append("")

    # Motor Damping Force
    if analysis.motor_damping_force:
        lines.append("## Motor Damping Force (High kd)")
        lines.append(f"- **High damping events:** {len(analysis.motor_damping_force)}")
        for damp in analysis.motor_damping_force[:5]:
            lines.append(f"  - Tick {damp['tick_sim_t']:.3f}s: Motor {damp['motor_idx']} force={damp['force']:.1f}")
        if len(analysis.motor_damping_force) > 5:
            lines.append(f"  ... and {len(analysis.motor_damping_force) - 5} more")
        lines.append("- 🔴 **High damping force detected** — kd=500 may be overwhelming motor torque")
        lines.append("")

    # Completion
    if analysis.completion:
        c = analysis.completion
        lines.append("## Turn Completion")
        lines.append(f"- **Total pulses:** {c.total_pulses}")
        lines.append(f"- **Commanded:** {math.degrees(c.commanded_rad):.2f}° ({c.commanded_rad:.4f} rad)")
        lines.append(f"- **Achieved:** {math.degrees(c.achieved_rad):.2f}° ({c.achieved_rad:.4f} rad)")
        lines.append(f"- **Error:** {math.degrees(c.error_rad):.2f}° ({c.error_rad:.4f} rad)")
        lines.append(f"- **Final gain:** {c.final_gain:.3f}")
        lines.append(f"- **Settled:** {c.settled}")
        lines.append(f"- **Timed out:** {c.timed_out}")
        lines.append("")

        # Effectiveness
        if c.commanded_rad != 0:
            effectiveness = c.achieved_rad / c.commanded_rad
            lines.append(f"- **Overall effectiveness:** {effectiveness:.1%}")
            if effectiveness < 0.5:
                lines.append("- 🔴 **Severe under-rotation** — root cause confirmed")
            elif effectiveness < 0.8:
                lines.append("- ⚠️ **Under-rotation** — gain not converging fast enough")
            else:
                lines.append("- ✅ Near-target achievement")

    # Root Cause Assessment
    lines.append("")
    lines.append("## Root Cause Assessment")
    lines.append("")

    assessments = []

    # Check gain convergence
    if analysis.gain_convergence:
        final_gain = analysis.gain_convergence[-1]
        if abs(final_gain - 0.17) < 0.05:
            assessments.append("✅ **Gain converged to expected ~0.17** — adaptive estimator working")
        elif final_gain <= 0.08 + 1e-3:
            assessments.append("🔴 **Gain hit floor (0.08)** — estimator cannot go lower, but actual gain may be even smaller")
        elif final_gain >= 1.5 - 1e-3:
            assessments.append("🔴 **Gain hit ceiling (1.5)** — estimator diverging upward")
        elif final_gain > 0.5:
            assessments.append(f"⚠️ **Gain stalled at {final_gain:.3f}** — not converging to 0.17, initial seed (0.55) not corrected")
        else:
            assessments.append(f"⚠️ **Gain at {final_gain:.3f}** — between expected and seed, partial convergence")

    # Check slip
    if analysis.slip_ratios:
        avg_slip = sum(analysis.slip_ratios) / len(analysis.slip_ratios)
        if avg_slip < 0.7:
            assessments.append("🔴 **Wheel slip is primary cause** — wheels achieving <70% of target velocity")
        elif avg_slip < 0.9:
            assessments.append("⚠️ **Moderate wheel slip** — contributing to under-rotation")

    # Check yaw rate tracking
    if analysis.yaw_rate_ratios:
        avg_yaw = sum(analysis.yaw_rate_ratios) / len(analysis.yaw_rate_ratios)
        if avg_yaw < 0.3:
            assessments.append("🔴 **Chassis yaw rate << commanded** — contact dynamics or inertia preventing rotation")
        elif avg_yaw < 0.6:
            assessments.append("⚠️ **Chassis yaw rate deficit** — partial transmission of wheel motion to chassis")

    # Check motor torque saturation
    if analysis.motor_torque_saturation:
        assessments.append("🔴 **Motor torque saturation (effort limit)** — motors hitting effort=200 limit, cannot overcome load")

    # Check motor damping force
    if analysis.motor_damping_force:
        assessments.append("🔴 **High motor damping force** — kd=500 generating excessive viscous damping, overwhelming motor torque")

    # Check pulse count vs effectiveness
    if analysis.completion and analysis.pulse_starts:
        total_pulses = analysis.completion.total_pulses
        if total_pulses >= 10 and analysis.completion.achieved_rad / analysis.completion.commanded_rad < 0.2:
            assessments.append("🔴 **Many pulses, little progress** — each pulse delivering ~10% of expected (gain ~0.1 vs 0.17)")

    if not assessments:
        assessments.append("❓ Insufficient data for assessment")

    for a in assessments:
        lines.append(f"- {a}")

    lines.append("")
    lines.append("---")
    lines.append("*Generated by pulse_trace_analyzer.py*")

    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_pulse_trace.py <trace.jsonl> [--output report.md]")
        sys.exit(1)

    trace_path = Path(sys.argv[1])
    output_path = None
    if "--output" in sys.argv:
        idx = sys.argv.index("--output")
        if idx + 1 < len(sys.argv):
            output_path = Path(sys.argv[idx + 1])

    if not trace_path.exists():
        print(f"Error: Trace file not found: {trace_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {trace_path}...")
    analysis = parse_trace(trace_path)
    print(f"Parsed: {len(analysis.pulse_ticks)} ticks, {len(analysis.gain_updates)} gain updates, {len(analysis.pulse_starts)} pulses")

    report = format_report(analysis, trace_path)

    if output_path:
        output_path.write_text(report)
        print(f"Report written to {output_path}")
    else:
        print(report)


if __name__ == "__main__":
    main()