"""
Adapter Integration Patch — Turn Gain Compensation
==================================================

This patch shows how to integrate TurnGainEstimator into your existing
OmniLink adapter. Apply the changes marked with +++ to your adapter file.

File to patch: your_adapter.py (or wherever send_turn_command lives)
"""

# ============================================================
# PATCH: Add imports at top of adapter file
# ============================================================

# +++ ADD THESE IMPORTS
import math
import json
from pathlib import Path
from turn_gain_estimator import create_turn_gain_estimator

# ============================================================
# PATCH: Initialize estimator at module level (singleton)
# ============================================================

# +++ ADD AFTER IMPORTS, BEFORE ANY FUNCTIONS
# Build ID from your environment / config
# In production, read from: os.environ.get("OMNISIM_BUILD_ID") or config file
_CURRENT_BUILD_ID = "7d39130cf"  # TODO: make configurable

# Telemetry sink - replace with your actual telemetry pipeline
def _telemetry_sink(event: dict) -> None:
    """Emit telemetry to your logging/observability system."""
    # Example: structured logging
    import logging
    logger = logging.getLogger("omnilink.adapter.turn_gain")
    logger.info("turn_gain_telemetry", extra=event)
    
    # Or write to file for audit trail:
    # audit_path = Path.home() / ".omnilink" / "turn_gain_audit.jsonl"
    # audit_path.parent.mkdir(parents=True, exist_ok=True)
    # with audit_path.open("a") as f:
    #     f.write(json.dumps(event) + "\n")

# Singleton estimator instance
_turn_gain_estimator = create_turn_gain_estimator(
    build_id=_CURRENT_BUILD_ID,
    telemetry_sink=_telemetry_sink,
)

# ============================================================
# PATCH: Replace your existing send_turn_command function
# ============================================================

# +++ REPLACE YOUR EXISTING send_turn_command WITH THIS:

def send_turn_command(target_deg: float, world: str, robot: str) -> None:
    """
    Send a turn command with automatic gain compensation.
    
    Args:
        target_deg: Desired turn angle in degrees (e.g., 90.0)
        world: World identifier (e.g., "omnilink_husky_swarm.omniworld")
        robot: Robot identifier (e.g., "husky_ne")
    """
    # Apply gain compensation
    compensated_deg = _turn_gain_estimator.apply_deg(world, robot, target_deg)
    compensated_rad = math.radians(compensated_deg)
    
    # Log for debugging
    import logging
    logger = logging.getLogger("omnilink.adapter")
    logger.debug(
        "turn_command",
        extra={
            "target_deg": target_deg,
            "compensated_deg": compensated_deg,
            "gain": _turn_gain_estimator.get_gain(world, robot),
            "world": world,
            "robot": robot,
        }
    )
    
    # Send to bridge (YOUR EXISTING CALL)
    bridge.send_turn(compensated_rad)


# ============================================================
# PATCH: Add result handler (call after each turn completes)
# ============================================================

# +++ ADD THIS FUNCTION AND CALL IT FROM YOUR TURN COMPLETION HANDLER

def on_turn_result(result_json: dict) -> None:
    """
    Update gain estimator from turn replay result.
    
    Call this from your turn completion callback / result handler.
    
    Expected result_json keys:
        - "world": str
        - "robot": str
        - "commanded_deg": float
        - "achieved_deg": float
        - "build_id": str (optional)
    """
    _turn_gain_estimator.update_from_result(result_json)
    
    # Log the update
    import logging
    logger = logging.getLogger("omnilink.adapter")
    logger.info(
        "turn_gain_updated",
        extra={
            "world": result_json.get("world"),
            "robot": result_json.get("robot"),
            "commanded_deg": result_json.get("commanded_deg"),
            "achieved_deg": result_json.get("achieved_deg"),
            "current_gain": _turn_gain_estimator.get_gain(
                result_json.get("world", ""),
                result_json.get("robot", "")
            ),
        }
    )


# ============================================================
# PATCH: If you have a replay client, hook it up like this:
# ============================================================

# +++ IN YOUR REPLAY CLIENT (e.g., run_lear_husky_turn.py), AFTER TURN COMPLETES:

"""
# ... existing code that gets 'turn' result ...

# Build result dict for estimator
result_for_estimator = {
    "world": "omnilink_husky_swarm.omniworld",
    "robot": "husky_ne",
    "commanded_deg": math.degrees(turn.get("angle", math.pi/2)),
    "achieved_deg": math.degrees(turn.get("achieved_rad", 0)),
    "build_id": "7d39130cf",  # or read from build info
}

# Update estimator
on_turn_result(result_for_estimator)

# Also save the full result as before
print(json.dumps({...}, indent=2))
"""

# ============================================================
# PATCH: Optional - Add CLI command to inspect gains
# ============================================================

# +++ ADD TO YOUR CLI / DEBUG COMMANDS

def cmd_turn_gain_status(args) -> None:
    """CLI command: show current turn gains."""
    print("Current Turn Gains:")
    print("-" * 80)
    for key, rec in _turn_gain_estimator.get_all_gains().items():
        print(f"  {key}")
        print(f"    gain:           {rec.gain:.4f}")
        print(f"    samples:        {rec.sample_count}")
        print(f"    last commanded: {rec.last_commanded_deg:.1f}°")
        print(f"    last achieved:  {rec.last_achieved_deg:.1f}°")
        print(f"    build_id:       {rec.build_id}")
        print(f"    updated:        {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(rec.updated_at))}")
        print()

def cmd_turn_gain_reset(args) -> None:
    """CLI command: reset gain for a world/robot."""
    world = args.world
    robot = args.robot
    _turn_gain_estimator.reset(world, robot)
    print(f"Reset gain for {world}|{robot}")

def cmd_turn_gain_reset_all(args) -> None:
    """CLI command: reset all gains."""
    _turn_gain_estimator.reset_all()
    print("Reset all gains")


# ============================================================
# COMPLETE MINIMAL EXAMPLE (standalone test)
# ============================================================

"""
# Save as test_turn_gain.py and run to verify integration:

import math
from turn_gain_estimator import create_turn_gain_estimator

# Mock bridge
class MockBridge:
    def send_turn(self, rad):
        print(f"  BRIDGE: send_turn({math.degrees(rad):.1f}°)")

bridge = MockBridge()

# Create estimator with file storage in temp dir
import tempfile
with tempfile.TemporaryDirectory() as tmpdir:
    estimator = create_turn_gain_estimator(
        build_id="7d39130cf",
        storage_dir=tmpdir,
        telemetry_sink=lambda e: print(f"  TELEMETRY: {e['event']}")
    )
    
    # Simulate first turn (under-rotates)
    print("\\n=== Turn 1: 90° command, 9.3° achieved ===")
    compensated = estimator.apply_deg("test_world", "test_robot", 90.0)
    print(f"  Compensated: {compensated:.1f}°")
    bridge.send_turn(math.radians(compensated))
    
    # Update from result
    estimator.update_from_result({
        "world": "test_world",
        "robot": "test_robot",
        "commanded_deg": 90.0,
        "achieved_deg": 9.319,
        "build_id": "7d39130cf",
    })
    print(f"  New gain: {estimator.get_gain('test_world', 'test_robot'):.4f}")
    
    # Simulate second turn (after learning)
    print("\\n=== Turn 2: 90° command (with learned gain) ===")
    compensated = estimator.apply_deg("test_world", "test_robot", 90.0)
    print(f"  Compensated: {compensated:.1f}°")
    bridge.send_turn(math.radians(compensated))
    
    # Update - now achieves near 90°
    estimator.update_from_result({
        "world": "test_world",
        "robot": "test_robot",
        "commanded_deg": 90.0,
        "achieved_deg": 88.5,
        "build_id": "7d39130cf",
    })
    print(f"  New gain: {estimator.get_gain('test_world', 'test_robot'):.4f}")
    
    # Third turn
    print("\\n=== Turn 3: 90° command (converged) ===")
    compensated = estimator.apply_deg("test_world", "test_robot", 90.0)
    print(f"  Compensated: {compensated:.1f}°")
    bridge.send_turn(math.radians(compensated))
"""

print("Patch file created. Apply the +++ sections to your adapter.")
print("See turn_gain_estimator.py for the full estimator implementation.")