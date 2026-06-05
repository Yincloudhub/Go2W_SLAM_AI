#!/usr/bin/env bash
set -euo pipefail

# Read-only resource snapshot for the resident robot-side stack.
# It samples /proc counters only and never starts, stops, or signals processes.

WINDOW_S="${1:-${GO2W_RESOURCE_WINDOW_S:-10}}"

python3 - "${WINDOW_S}" <<'PY'
import os
import sys
import time
from pathlib import Path

window_s = float(sys.argv[1])
if window_s <= 0:
    raise SystemExit("sample window must be positive")

clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


def read_cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().decode("utf-8", "replace").replace("\0", " ").strip()
    except OSError:
        return ""


def read_comm(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/comm").read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def find_pid(label: str):
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        comm = read_comm(pid)
        cmdline = read_cmdline(pid)
        if label == "xt16_driver" and comm == "xt16_driver":
            return pid
        if label == "unitree_slam" and comm == "unitree_slam":
            return pid
        if label == "deepyolo_detector" and "yolo_test_realsense_headless" in cmdline:
            return pid
        if label == "deepyolo_bridge" and "deepyolo_semantic_bridge.py" in cmdline:
            return pid
        if label == "stereo_depth" and "realsense_depth_summary.py" in cmdline:
            return pid
        if label == "xt16_geometry" and "xt16_lidar_geometry_summary.py" in cmdline:
            return pid
        if label == "operator_web" and "go2w_operator_web.py" in cmdline:
            return pid
    return None


def read_process(pid: int) -> dict:
    stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
    status = {}
    for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition(":")
        status[key] = value.strip()
    return {
        "ticks": int(stat[13]) + int(stat[14]),
        "rss_kb": int(status.get("VmRSS", "0 kB").split()[0]),
        "threads": int(status.get("Threads", "0")),
        "cmdline": read_cmdline(pid),
    }


def read_thread_ticks(pid: int):
    rows = {}
    for task in Path(f"/proc/{pid}/task").iterdir():
        try:
            stat = (task / "stat").read_text(encoding="utf-8").split()
            rows[int(task.name)] = (stat[1].strip("()"), int(stat[13]) + int(stat[14]))
        except OSError:
            pass
    return rows


labels = ["xt16_driver", "unitree_slam", "stereo_depth", "xt16_geometry", "deepyolo_detector", "deepyolo_bridge", "operator_web"]
pids = {label: find_pid(label) for label in labels}
before = {label: read_process(pid) for label, pid in pids.items() if pid is not None}
slam_pid = pids.get("unitree_slam")
threads_before = read_thread_ticks(slam_pid) if slam_pid is not None else {}

print(f"timestamp={time.strftime('%Y-%m-%dT%H:%M:%S%z')}")
print(f"sample_window_s={window_s:g}")
print(f"logical_cpus={os.cpu_count() or 1}")
time.sleep(window_s)

print("processes:")
for label in labels:
    pid = pids.get(label)
    if pid is None or label not in before:
        print(f"  {label}=stopped")
        continue
    try:
        after = read_process(pid)
    except OSError:
        print(f"  {label}=exited pid={pid}")
        continue
    cpu_percent = 100.0 * (after["ticks"] - before[label]["ticks"]) / clock_ticks / window_s
    print(
        f"  {label}=running pid={pid} cpu_percent={cpu_percent:.1f} "
        f"rss_mb={after['rss_kb'] / 1024:.1f} threads={after['threads']}"
    )

if slam_pid is not None:
    try:
        threads_after = read_thread_ticks(slam_pid)
    except OSError:
        threads_after = {}
    rows = []
    for tid, (name, ticks) in threads_after.items():
        previous = threads_before.get(tid, (name, ticks))[1]
        rows.append((ticks - previous, tid, name))
    print("unitree_slam_top_threads:")
    for delta, tid, name in sorted(rows, reverse=True)[:8]:
        cpu_percent = 100.0 * delta / clock_ticks / window_s
        print(f"  tid={tid} cpu_percent={cpu_percent:.1f} name={name}")

try:
    load = os.getloadavg()
    print(f"load_average_1m={load[0]:.2f} load_average_5m={load[1]:.2f} load_average_15m={load[2]:.2f}")
except OSError:
    pass
PY
