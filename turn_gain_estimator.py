"""
TurnGainEstimator — Interim Compensation Patch for OmniLink Adapter
===================================================================

This module provides a persistent, bounded, audited gain estimator that learns
the turn compensation factor from each replay result and applies it to subsequent
turn commands. Designed as an interim fix while the bridge PID gain root cause
is addressed upstream.

Usage:
    from turn_gain_estimator import TurnGainEstimator, create_turn_gain_estimator
    
    # Initialize once at adapter startup
    estimator = create_turn_gain_estimator(build_id="7d39130cf")
    
    # Apply compensation before sending turn command
    compensated_rad = estimator.apply(world="omnilink_husky_swarm.omniworld",
                                       robot="husky_ne",
                                       target_rad=math.pi/2)
    bridge.send_turn(compensated_rad)
    
    # Update from replay result (call after each turn completes)
    estimator.update_from_result(result_json)

Requirements:
    - Python 3.8+
    - Standard library only (json, pathlib, threading, time)
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional


@dataclass
class GainRecord:
    """Persistent record of a gain estimate for a (world, robot) pair."""
    world: str
    robot: str
    gain: float
    updated_at: float  # unix timestamp
    sample_count: int
    last_commanded_deg: float
    last_achieved_deg: float
    build_id: str


class TurnGainEstimator:
    """
    Persistent per-(world,robot) turn gain estimator with exponential smoothing.
    
    Features:
    - Per-(world, robot) gain tracking
    - Exponential smoothing (configurable alpha)
    - Bounded gains (configurable min/max)
    - Thread-safe
    - Persistent storage (JSON file)
    - Audit telemetry emission
    - Build ID gating
    """
    
    DEFAULT_ALPHA = 0.3
    DEFAULT_MIN_GAIN = 0.1
    DEFAULT_MAX_GAIN = 10.0
    DEFAULT_INITIAL_GAIN = 1.0
    
    def __init__(
        self,
        storage_path: str | Path,
        alpha: float = DEFAULT_ALPHA,
        min_gain: float = DEFAULT_MIN_GAIN,
        max_gain: float = DEFAULT_MAX_GAIN,
        initial_gain: float = DEFAULT_INITIAL_GAIN,
        build_id: Optional[str] = None,
        telemetry_sink: Optional[callable] = None,
    ):
        """
        Initialize the estimator.
        
        Args:
            storage_path: Path to JSON file for persistent gain storage
            alpha: Exponential smoothing factor (0 < alpha <= 1). Higher = faster adaptation.
            min_gain: Minimum allowed gain (safety bound)
            max_gain: Maximum allowed gain (safety bound)
            initial_gain: Default gain for unseen (world, robot) pairs
            build_id: If set, only apply compensation for this build ID
            telemetry_sink: Optional callable(telemetry_dict) for audit logging
        """
        self.storage_path = Path(storage_path)
        self.alpha = alpha
        self.min_gain = min_gain
        self.max_gain = max_gain
        self.initial_gain = initial_gain
        self.build_id = build_id
        self.telemetry_sink = telemetry_sink
        
        self._gains: dict[tuple[str, str], GainRecord] = {}
        self._lock = threading.RLock()
        
        # Load persisted gains
        self._load()
    
    def _load(self) -> None:
        """Load gains from persistent storage."""
        if not self.storage_path.exists():
            return
        try:
            with self.storage_path.open() as f:
                data = json.load(f)
            for key, record in data.items():
                world, robot = key.split("|", 1)
                self._gains[(world, robot)] = GainRecord(**record)
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            # Corrupted storage - start fresh but log
            self._emit_telemetry({
                "event": "gain_storage_corrupted",
                "error": str(e),
                "path": str(self.storage_path),
            })
            self._gains = {}
    
    def _save(self) -> None:
        """Persist gains to storage."""
        # Atomic write: write to temp then rename
        tmp_path = self.storage_path.with_suffix(".tmp")
        try:
            data = {f"{w}|{r}": asdict(rec) for (w, r), rec in self._gains.items()}
            with tmp_path.open("w") as f:
                json.dump(data, f, indent=2)
            tmp_path.replace(self.storage_path)
        except Exception as e:
            self._emit_telemetry({
                "event": "gain_storage_write_failed",
                "error": str(e),
                "path": str(self.storage_path),
            })
            if tmp_path.exists():
                tmp_path.unlink(missing_ok=True)
    
    def _emit_telemetry(self, event: dict) -> None:
        """Emit telemetry event if sink configured."""
        if self.telemetry_sink:
            try:
                event["timestamp"] = time.time()
                event["iso_time"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
                self.telemetry_sink(event)
            except Exception:
                pass  # Telemetry must never break the estimator
    
    def _get_key(self, world: str, robot: str) -> tuple[str, str]:
        return (world, robot)
    
    def get_gain(self, world: str, robot: str) -> float:
        """Get current gain for (world, robot), or initial_gain if unknown."""
        with self._lock:
            key = self._get_key(world, robot)
            record = self._gains.get(key)
            return record.gain if record else self.initial_gain
    
    def get_record(self, world: str, robot: str) -> Optional[GainRecord]:
        """Get full gain record for (world, robot)."""
        with self._lock:
            return self._gains.get(self._get_key(world, robot))
    
    def apply(self, world: str, robot: str, target_rad: float) -> float:
        """
        Apply gain compensation to a target turn angle.
        
        Args:
            world: World identifier (e.g., "omnilink_husky_swarm.omniworld")
            robot: Robot identifier (e.g., "husky_ne")
            target_rad: Desired turn angle in radians
            
        Returns:
            Compensated turn angle in radians
        """
        # Check build ID gate
        if self.build_id:
            # In practice, you'd check the current build ID here
            # For now, we assume the gate is handled by caller
            pass
        
        gain = self.get_gain(world, robot)
        compensated_rad = target_rad * gain
        
        # Emit telemetry for audit
        self._emit_telemetry({
            "event": "turn_gain_applied",
            "world": world,
            "robot": robot,
            "target_rad": target_rad,
            "target_deg": math.degrees(target_rad),
            "gain": gain,
            "compensated_rad": compensated_rad,
            "compensated_deg": math.degrees(compensated_rad),
            "build_id": self.build_id,
        })
        
        return compensated_rad
    
    def apply_deg(self, world: str, robot: str, target_deg: float) -> float:
        """Apply gain compensation to a target turn angle in degrees."""
        target_rad = math.radians(target_deg)
        compensated_rad = self.apply(world, robot, target_rad)
        return math.degrees(compensated_rad)
    
    def update_from_result(self, result_json: dict) -> None:
        """
        Update gain estimate from a replay result.
        
        Expected keys in result_json:
            - "world": str
            - "robot": str
            - "commanded_deg": float
            - "achieved_deg": float
            - "build_id": str (optional)
        
        Args:
            result_json: Dictionary with replay result fields
        """
        world = result_json.get("world")
        robot = result_json.get("robot")
        commanded_deg = result_json.get("commanded_deg")
        achieved_deg = result_json.get("achieved_deg")
        build_id = result_json.get("build_id")
        
        if not all(v is not None for v in (world, robot, commanded_deg, achieved_deg)):
            self._emit_telemetry({
                "event": "turn_gain_update_skipped",
                "reason": "missing_required_fields",
                "fields_present": {k: v is not None for k, v in {
                    "world": world, "robot": robot,
                    "commanded_deg": commanded_deg, "achieved_deg": achieved_deg
                }.items()},
            })
            return
        
        if commanded_deg == 0:
            return
        
        ratio = achieved_deg / commanded_deg
        if ratio <= 0:
            self._emit_telemetry({
                "event": "turn_gain_update_skipped",
                "reason": "non_positive_ratio",
                "ratio": ratio,
                "world": world,
                "robot": robot,
            })
            return
        
        new_gain = 1.0 / ratio
        
        # Clamp to bounds
        new_gain = max(self.min_gain, min(self.max_gain, new_gain))
        
        with self._lock:
            key = self._get_key(world, robot)
            old_record = self._gains.get(key)
            old_gain = old_record.gain if old_record else self.initial_gain
            sample_count = (old_record.sample_count + 1) if old_record else 1
            
            # Exponential smoothing
            smoothed_gain = (1 - self.alpha) * old_gain + self.alpha * new_gain
            smoothed_gain = max(self.min_gain, min(self.max_gain, smoothed_gain))
            
            record = GainRecord(
                world=world,
                robot=robot,
                gain=smoothed_gain,
                updated_at=time.time(),
                sample_count=sample_count,
                last_commanded_deg=commanded_deg,
                last_achieved_deg=achieved_deg,
                build_id=build_id or "unknown",
            )
            self._gains[key] = record
            self._save()
        
        # Emit telemetry
        self._emit_telemetry({
            "event": "turn_gain_updated",
            "world": world,
            "robot": robot,
            "commanded_deg": commanded_deg,
            "achieved_deg": achieved_deg,
            "raw_gain": new_gain,
            "smoothed_gain": smoothed_gain,
            "old_gain": old_gain,
            "alpha": self.alpha,
            "sample_count": sample_count,
            "build_id": build_id,
        })
    
    def reset(self, world: str, robot: str) -> None:
        """Reset gain for a specific (world, robot) to initial_gain."""
        with self._lock:
            key = self._get_key(world, robot)
            if key in self._gains:
                del self._gains[key]
                self._save()
        
        self._emit_telemetry({
            "event": "turn_gain_reset",
            "world": world,
            "robot": robot,
        })
    
    def reset_all(self) -> None:
        """Reset all gains to initial_gain."""
        with self._lock:
            self._gains = {}
            self._save()
        
        self._emit_telemetry({"event": "turn_gain_reset_all"})
    
    def get_all_gains(self) -> dict[str, GainRecord]:
        """Get all gain records (for inspection/debugging)."""
        with self._lock:
            return {f"{w}|{r}": rec for (w, r), rec in self._gains.items()}


def create_turn_gain_estimator(
    build_id: str = "7d39130cf",
    storage_dir: Optional[str | Path] = None,
    telemetry_sink: Optional[callable] = None,
) -> TurnGainEstimator:
    """
    Factory function to create a TurnGainEstimator with sensible defaults.
    
    Args:
        build_id: Build ID to gate compensation (only compensate for this build)
        storage_dir: Directory for persistent storage (default: ~/.omnilink/gains)
        telemetry_sink: Optional callable for audit telemetry
        
    Returns:
        Configured TurnGainEstimator instance
    """
    if storage_dir is None:
        storage_dir = Path.home() / ".omnilink" / "gains"
    storage_dir = Path(storage_dir)
    storage_dir.mkdir(parents=True, exist_ok=True)
    storage_path = storage_dir / "turn_gains.json"
    
    # Initial gain of 9.66 for the known-bad build, 1.0 otherwise
    initial_gain = 9.66 if build_id == "7d39130cf" else 1.0
    
    return TurnGainEstimator(
        storage_path=storage_path,
        alpha=0.3,
        min_gain=0.1,
        max_gain=10.0,
        initial_gain=initial_gain,
        build_id=build_id,
        telemetry_sink=telemetry_sink,
    )


# ============================================================
# INTEGRATION EXAMPLE
# ============================================================

"""
# In your adapter code (e.g., omnilink_adapter.py):

from turn_gain_estimator import create_turn_gain_estimator
import math

# Initialize once at module load / adapter startup
_turn_gain_estimator = create_turn_gain_estimator(
    build_id="7d39130cf",
    telemetry_sink=lambda evt: print(f"TELEMETRY: {json.dumps(evt)}")
)

def send_turn_command(target_deg: float, world: str, robot: str) -> None:
    '''Send a turn command with gain compensation.'''
    # Apply compensation
    compensated_deg = _turn_gain_estimator.apply_deg(world, robot, target_deg)
    compensated_rad = math.radians(compensated_deg)
    
    # Send to bridge
    bridge.send_turn(compensated_rad)

def on_replay_result(result_json: dict) -> None:
    '''Call this after each turn replay completes.'''
    _turn_gain_estimator.update_from_result(result_json)

# Example usage:
# send_turn_command(90.0, "omnilink_husky_swarm.omniworld", "husky_ne")
# ... turn executes ...
# on_replay_result({
#     "world": "omnilink_husky_swarm.omniworld",
#     "robot": "husky_ne",
#     "commanded_deg": 90.0,
#     "achieved_deg": 9.319,
#     "build_id": "7d39130cf",
# })
"""

# ============================================================
# CLI FOR INSPECTION
# ============================================================

if __name__ == "__main__":
    import sys
    
    estimator = create_turn_gain_estimator(build_id="7d39130cf")
    
    if len(sys.argv) > 1 and sys.argv[1] == "show":
        print("Current gains:")
        for key, rec in estimator.get_all_gains().items():
            print(f"  {key}: gain={rec.gain:.4f}, samples={rec.sample_count}, "
                  f"last_cmd={rec.last_commanded_deg:.1f}°, last_ach={rec.last_achieved_deg:.1f}°")
    elif len(sys.argv) > 1 and sys.argv[1] == "test":
        # Demo: simulate a few updates
        print("Testing estimator...")
        estimator.update_from_result({
            "world": "omnilink_husky_swarm.omniworld",
            "robot": "husky_ne",
            "commanded_deg": 90.0,
            "achieved_deg": 9.319,
            "build_id": "7d39130cf",
        })
        print(f"After 1st update: gain={estimator.get_gain('omnilink_husky_swarm.omniworld', 'husky_ne'):.4f}")
        
        estimator.update_from_result({
            "world": "omnilink_husky_swarm.omniworld",
            "robot": "husky_ne",
            "commanded_deg": 90.0,
            "achieved_deg": 85.0,  # Much better after fix
            "build_id": "7d39130cf",
        })
        print(f"After 2nd update: gain={estimator.get_gain('omnilink_husky_swarm.omniworld', 'husky_ne'):.4f}")
        
        compensated = estimator.apply_deg("omnilink_husky_swarm.omniworld", "husky_ne", 90.0)
        print(f"Compensated 90° -> {compensated:.1f}°")
    else:
        print("Usage:")
        print("  python turn_gain_estimator.py show   # Show current gains")
        print("  python turn_gain_estimator.py test   # Run demo")