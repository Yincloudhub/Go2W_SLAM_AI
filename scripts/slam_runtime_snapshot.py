from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from edge_autonomy.runtime_state import build_runtime_snapshot  # noqa: E402


SECTION_BEGIN = "__GO2W_SECTION_BEGIN__"
SECTION_END = "__GO2W_SECTION_END__"


REMOTE_SCRIPT = r'''
set +e
source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true

section() {
  name="$1"
  shift
  echo "__GO2W_SECTION_BEGIN__${name}"
  "$@"
  echo "__GO2W_SECTION_END__${name}"
}

section processes bash -lc "ps -eo pid=,comm=,args= | awk '\$2==\"unitree_slam\" || \$2==\"xt16_driver\" || \$2==\"slam_keyboard_c\" || \$2==\"slam_llm_comman\" || \$2==\"slam_keyboard_client\" || \$2==\"slam_llm_command_client\" {print}' || true"
section lidar_state bash -lc "timeout 6 ros2 topic echo /utlidar/lidar_state --qos-reliability reliable --no-arr 2>/dev/null | sed -n '1,100p' || true"
section live_pointcloud bash -lc "timeout 6 ros2 topic echo /unitree/slam_lidar/points --qos-reliability reliable --no-arr 2>/dev/null | sed -n '1,120p' || true"
section relocation_odom bash -lc "timeout 4 ros2 topic echo /unitree/slam_relocation/odom --qos-reliability reliable --no-arr 2>/dev/null | sed -n '1,120p' || true"
section slam_info bash -lc "timeout 4 ros2 topic echo /slam_info --qos-reliability reliable --full-length --no-arr 2>/dev/null | sed -n '1,120p' || true"
section slam_key_info bash -lc "timeout 4 ros2 topic echo /slam_key_info --qos-reliability reliable --full-length --no-arr 2>/dev/null | sed -n '1,120p' || true"
'''


def parse_sections(output: str) -> dict[str, str]:
    sections: dict[str, list[str]] = {}
    current: str | None = None
    for line in output.splitlines():
        if line.startswith(SECTION_BEGIN):
            current = line[len(SECTION_BEGIN) :].strip()
            sections[current] = []
            continue
        if line.startswith(SECTION_END):
            current = None
            continue
        if current is not None:
            sections[current].append(line)
    return {name: "\n".join(lines).strip() for name, lines in sections.items()}


def run_remote_snapshot(host: str, username: str, password: str, *, timeout_s: int) -> str:
    try:
        import paramiko
    except ImportError as exc:
        raise RuntimeError("paramiko is required for SSH snapshot collection") from exc

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=username, password=password, timeout=timeout_s)
    try:
        _, stdout, stderr = ssh.exec_command(REMOTE_SCRIPT, timeout=timeout_s)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
    finally:
        ssh.close()
    if err.strip():
        print(err, file=sys.stderr)
    return out


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Collect a read-only GO2W SLAM/LiDAR runtime snapshot over SSH.")
    parser.add_argument("--host", default="192.168.123.18")
    parser.add_argument("--username", default="unitree")
    parser.add_argument("--password", default=os.environ.get("GO2W_SSH_PASSWORD", ""))
    parser.add_argument("--map-id", default="unknown")
    parser.add_argument("--map-path", default="")
    parser.add_argument("--timeout-s", type=int, default=30)
    parser.add_argument("--pretty", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    password = args.password or getpass.getpass(f"{args.username}@{args.host} password: ")
    output = run_remote_snapshot(args.host, args.username, password, timeout_s=args.timeout_s)
    snapshot = build_runtime_snapshot(
        parse_sections(output),
        host=args.host,
        expected_map_id=args.map_id,
        expected_map_path=args.map_path,
    )
    if args.pretty:
        print(json.dumps(snapshot.to_dict(), ensure_ascii=False, indent=2))
    else:
        print(json.dumps(snapshot.to_dict(), ensure_ascii=False, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
