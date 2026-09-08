"""Launch OmniSim and capture per-tick EvidenceRecord trace for diagnostic protocol."""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from datetime import datetime


ROOT = Path(__file__).resolve().parents[5]
BINARY = ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
WORLD = (ROOT / "projects" / "samples" / "demos" / "worlds" / "flagship" /
         "omnilink_husky_swarm.omniworld")
OUT = Path(__file__).with_suffix(".engine.log")
TRACE = Path(__file__).with_suffix(".evidence_trace.jsonl")
BASE = "http://127.0.0.1:8865"


def request(path: str, body: dict | None = None, timeout: float = 5.0):
    payload = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        BASE + path,
        data=payload,
        headers={"Content-Type": "application/json"} if payload else {},
        method="POST" if payload else "GET",
    )
    with urllib.request.urlopen(req, timeout=timeout) as response:
        return json.loads(response.read())


def extract_evidence_record(state: dict) -> dict | None:
    """Extract the 9 EvidenceRecord fields from bridge robot state."""
    # The bridge state should contain these fields. Try multiple possible keys.
    rec = {
        "timestamp": time.time(),
        "iso_time": datetime.utcnow().isoformat() + "Z",
    }
    
    # raw_world_root: [x, y, z] or [x, y]
    for key in ("raw_world_root", "world_root", "root_pose", "pose_world"):
        if key in state and state[key] is not None:
            rec["raw_world_root"] = state[key]
            break
    
    # bridge pose
    for key in ("bridge_x", "x", "pos_x"):
        if key in state and state[key] is not None:
            rec["bridge_x"] = float(state[key])
            break
    for key in ("bridge_y", "y", "pos_y"):
        if key in state and state[key] is not None:
            rec["bridge_y"] = float(state[key])
            break
    for key in ("bridge_yaw", "yaw", "heading", "theta"):
        if key in state and state[key] is not None:
            rec["bridge_yaw"] = float(state[key])
            break
    
    # commanded velocities
    for key in ("cmd_vel_linear_x", "cmd_vel_x", "vx_cmd", "linear_velocity_cmd"):
        if key in state and state[key] is not None:
            rec["cmd_vel_linear_x"] = float(state[key])
            break
    for key in ("cmd_vel_angular_z", "cmd_vel_yaw", "omega_cmd", "angular_velocity_cmd"):
        if key in state and state[key] is not None:
            rec["cmd_vel_angular_z"] = float(state[key])
            break
    
    # world velocities (derivatives)
    for key in ("world_dx_dt", "dx_dt", "vx_world", "world_vel_x"):
        if key in state and state[key] is not None:
            rec["world_dx_dt"] = float(state[key])
            break
    for key in ("world_dy_dt", "dy_dt", "vy_world", "world_vel_y"):
        if key in state and state[key] is not None:
            rec["world_dy_dt"] = float(state[key])
            break
    
    # odometry pose [x, y, yaw]
    for key in ("odometry_pose", "odom_pose", "odometry", "pose_odom"):
        if key in state and state[key] is not None:
            rec["odometry_pose"] = state[key]
            break
    
    # Also capture any other useful fields
    for extra in ("last_command", "bridge_instance_id", "seq", "settled", "completion_reason"):
        if extra in state:
            rec[extra] = state[extra]
    
    # Only return if we got at least the core fields
    core_fields = ["bridge_x", "bridge_y", "bridge_yaw", "cmd_vel_linear_x", "world_dx_dt", "world_dy_dt"]
    if all(f in rec for f in core_fields):
        return rec
    return None


def main():
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(ROOT)
    env["WEBOTS_HOME"] = str(ROOT)
    runtime = ROOT / "msys64" / "mingw64" / "bin" / "newton-runtime"
    mingw = ROOT / "msys64" / "mingw64" / "bin"
    env["PATH"] = os.pathsep.join((str(mingw), env.get("PATH", ""), str(runtime)))
    env["PYTHON_HOME"] = str(runtime)
    env["WARP_CACHE_PATH"] = str(
        ROOT / "social" / "launch" / "pilots" / "replies_2026_09_07" /
        "evaluations" / "warp_cache"
    )

    trace_records = []
    
    with OUT.open("wb") as log:
        proc = subprocess.Popen(
            [str(BINARY), "--batch", "--mode=realtime", "--no-rendering",
             "--stdout", "--stderr", "--port=1281", str(WORLD)],
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            # Wait for bridge ready
            for _ in range(120):
                if proc.poll() is not None:
                    raise RuntimeError(f"OmniSim exited early with {proc.returncode}")
                try:
                    request("/protocol", timeout=1.0)
                    break
                except (OSError, urllib.error.URLError):
                    time.sleep(0.5)
            else:
                raise RuntimeError("bridge did not become ready within 60 seconds")

            # Get initial state
            before = request("/get_robot_state")
            print(f"Initial state keys: {list(before.keys())}")
            
            # Send turn command
            accepted = request("/turn", {"angle": math.pi / 2, "wait": False})
            print(f"Turn accepted: {accepted}")
            
            # Poll at high frequency during turn, capturing EvidenceRecord each tick
            deadline = time.monotonic() + 130.0
            turn_complete = None
            poll_interval = 0.05  # 50ms = 20Hz
            
            print("Starting high-frequency telemetry capture...")
            while time.monotonic() < deadline:
                tick_start = time.monotonic()
                state = request("/get_robot_state")
                
                # Extract and store EvidenceRecord
                rec = extract_evidence_record(state)
                if rec:
                    trace_records.append(rec)
                
                # Check for turn completion
                candidate = state.get("last_command")
                if (isinstance(candidate, dict)
                        and candidate.get("bridge_instance_id") == accepted.get("bridge_instance_id")
                        and candidate.get("seq") == accepted.get("seq")):
                    turn_complete = candidate
                    print(f"Turn completed at tick {len(trace_records)}")
                    # Capture a few more ticks after completion
                    for _ in range(5):
                        time.sleep(poll_interval)
                        state = request("/get_robot_state")
                        rec = extract_evidence_record(state)
                        if rec:
                            trace_records.append(rec)
                    break
                
                # Maintain poll rate
                elapsed = time.monotonic() - tick_start
                sleep_time = max(0, poll_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            if turn_complete is None:
                raise RuntimeError("turn produced no matching completion within 130 seconds")
            
            after = request("/get_robot_state")
            
            # Write trace to JSONL
            with TRACE.open("w") as f:
                for rec in trace_records:
                    f.write(json.dumps(rec) + "\n")
            print(f"Captured {len(trace_records)} EvidenceRecord ticks -> {TRACE}")
            
            # Compute diagnostic protocol Step 4: R scalar
            compute_diagnostic_R(trace_records)
            
            # Print summary
            print(json.dumps({
                "engine_pid": proc.pid,
                "before": before,
                "accepted": accepted,
                "turn": turn_complete,
                "after": after,
                "trace_file": str(TRACE),
                "trace_ticks": len(trace_records),
            }, indent=2))
            
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(10)


def compute_diagnostic_R(trace_records: list[dict]):
    """Diagnostic Protocol Step 4: Compute velocity ratio scalar R."""
    print("\n" + "="*60)
    print("DIAGNOSTIC PROTOCOL - Step 4: Bridge Integration Cross-Check")
    print("="*60)
    
    valid_ticks = 0
    R_values = []
    
    for i, rec in enumerate(trace_records):
        v_x = rec.get("cmd_vel_linear_x")
        world_dx_dt = rec.get("world_dx_dt")
        world_dy_dt = rec.get("world_dy_dt")
        bridge_yaw = rec.get("bridge_yaw")
        
        if v_x is None or world_dx_dt is None or world_dy_dt is None or bridge_yaw is None:
            continue
        
        if abs(v_x) < 1e-6:
            continue  # Skip near-zero command ticks
        
        # Expected world velocity from command + bridge_yaw
        v_world_exp_x = v_x * math.cos(bridge_yaw)
        v_world_exp_y = v_x * math.sin(bridge_yaw)
        v_world_exp_mag = math.hypot(v_world_exp_x, v_world_exp_y)
        
        # Measured world velocity
        v_world_meas_mag = math.hypot(world_dx_dt, world_dy_dt)
        
        # Velocity ratio scalar R
        R = v_world_meas_mag / abs(v_x)
        R_values.append(R)
        valid_ticks += 1
        
        # Also compute pre-dispatch drift (Step 2)
        raw_root = rec.get("raw_world_root")
        bridge_x = rec.get("bridge_x")
        bridge_y = rec.get("bridge_y")
        if raw_root and bridge_x is not None and bridge_y is not None:
            rx = raw_root[0] if isinstance(raw_root, (list, tuple)) else raw_root
            ry = raw_root[1] if isinstance(raw_root, (list, tuple)) and len(raw_root) > 1 else 0
            dx_pre = bridge_x - rx
            dy_pre = bridge_y - ry
            if i < 5:  # Show first few
                print(f"  Tick {i}: pre-dispatch Δx={dx_pre:.6f}, Δy={dy_pre:.6f}")
        
        if i < 5:  # Show first few ticks
            print(f"  Tick {i}: v_x={v_x:.4f}, yaw={bridge_yaw:.4f}, "
                  f"exp=({v_world_exp_x:.4f},{v_world_exp_y:.4f}), "
                  f"meas=({world_dx_dt:.4f},{world_dy_dt:.4f}), R={R:.4f}")
    
    if R_values:
        R_mean = sum(R_values) / len(R_values)
        R_min = min(R_values)
        R_max = max(R_values)
        print(f"\nValid ticks: {valid_ticks}")
        print(f"R statistics: mean={R_mean:.4f}, min={R_min:.4f}, max={R_max:.4f}")
        
        # Diagnostic Matrix interpretation
        print("\n--- DIAGNOSTIC INTERPRETATION ---")
        if abs(R_mean - 1.0) < 0.1:
            print("✅ R ≈ 1.0: CLEAN TRANSFORMATION - Command dispatch, bridge integration, and frame composition aligned.")
        elif abs(R_mean - 1.414) < 0.1:
            print("⚠️  R ≈ √2 (1.414): DOUBLE FRAME APPLICATION / MISSING ROTATION")
            print("   v_x applied directly to both X and Y axes without bridge_yaw rotation matrix.")
            print("   Root cause: Bridge layer frame error.")
        elif R_mean > 5.0:
            print(f"🔴 R ≈ {R_mean:.2f}: EXTREME GAIN MISMATCH")
            print("   Likely rad/deg confusion or gain applied multiple times.")
            print("   Check: cmd_vel units, bridge_yaw units, integration gain.")
        else:
            print(f"⚠️  R = {R_mean:.4f}: INTEGRATION ACCUMULATION DRIFT")
            print("   Bridge rate integration (∫v dt) misaligned with odometry sample rates.")
            print("   Check: timestamping, bridge integration tick rate.")
    else:
        print("❌ No valid ticks for R computation - missing required fields in trace")
    
    # Step 5: Odometry vs Bridge cross-check
    print("\n" + "="*60)
    print("DIAGNOSTIC PROTOCOL - Step 5: Odometry vs Bridge Composition")
    print("="*60)
    
    odom_deltas = []
    for rec in trace_records:
        bridge_x = rec.get("bridge_x")
        bridge_y = rec.get("bridge_y")
        bridge_yaw = rec.get("bridge_yaw")
        odom = rec.get("odometry_pose")
        
        if all(v is not None for v in (bridge_x, bridge_y, bridge_yaw)) and odom:
            ox = odom[0] if isinstance(odom, (list, tuple)) else odom
            oy = odom[1] if isinstance(odom, (list, tuple)) and len(odom) > 1 else 0
            oyaw = odom[2] if isinstance(odom, (list, tuple)) and len(odom) > 2 else 0
            
            dx = bridge_x - ox
            dy = bridge_y - oy
            dyaw = bridge_yaw - oyaw
            delta_mag = math.hypot(dx, dy)
            odom_deltas.append((delta_mag, dx, dy, dyaw))
    
    if odom_deltas:
        avg_delta = sum(d[0] for d in odom_deltas) / len(odom_deltas)
        max_delta = max(d[0] for d in odom_deltas)
        print(f"Odometry deltas: avg={avg_delta:.6f}, max={max_delta:.6f} (over {len(odom_deltas)} ticks)")
        if avg_delta < 1e-3:
            print("✅ Bridge ≈ Odometry: Error is in low-level motor controller / odometry frame")
        else:
            print("⚠️  Bridge ≠ Odometry: Error is in Omni-Link bridge translation middleware")
    else:
        print("❌ No odometry_pose in trace - cannot run Step 5")


if __name__ == "__main__":
    main()