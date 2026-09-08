"""Launch one owned OmniSim process and measure a settled 90-degree turn."""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
BINARY = ROOT / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
WORLD = (ROOT / "projects" / "samples" / "demos" / "worlds" / "flagship" /
         "omnilink_husky_swarm.omniworld")
OUT = Path(__file__).with_suffix(".engine.log")
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


def main():
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(ROOT)
    env["WEBOTS_HOME"] = str(ROOT)
    runtime = ROOT / "msys64" / "mingw64" / "bin" / "newton-runtime"
    mingw = ROOT / "msys64" / "mingw64" / "bin"
    env["PATH"] = os.pathsep.join((str(mingw), env.get("PATH", ""), str(runtime)))
    # This is only a build-helper probe, but setting it avoids traversing a
    # separately ACL-protected conventional Python install in the CLI helper.
    env["PYTHON_HOME"] = str(runtime)
    env["WARP_CACHE_PATH"] = str(
        ROOT / "social" / "launch" / "pilots" / "replies_2026_09_07" /
        "evaluations" / "warp_cache"
    )

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

            before = request("/get_robot_state")
            accepted = request("/turn", {"angle": math.pi / 2, "wait": False})
            turn = None
            deadline = time.monotonic() + 130.0
            after = before
            while time.monotonic() < deadline:
                after = request("/get_robot_state")
                candidate = after.get("last_command")
                if (isinstance(candidate, dict)
                        and candidate.get("bridge_instance_id") == accepted.get("bridge_instance_id")
                        and candidate.get("seq") == accepted.get("seq")):
                    turn = candidate
                    break
                time.sleep(0.25)
            if turn is None:
                raise RuntimeError("turn produced no matching completion within 130 seconds")
            print(json.dumps({
                "engine_pid": proc.pid,
                "before": before,
                "accepted": accepted,
                "turn": turn,
                "after": after,
            }, indent=2))
        finally:
            # This is the exact Popen child created above; no other engine is
            # enumerated or touched.
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(10)


if __name__ == "__main__":
    main()
