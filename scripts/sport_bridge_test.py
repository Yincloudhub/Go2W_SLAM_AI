#!/usr/bin/env python3
"""sport_bridge 验收测试 —— 交互式运动测试

用法:
    cd /home/unitree/Go2W_SLAM_AI/robot/slam_gateway_refactor/build
    python3 /home/unitree/Go2W_SLAM_AI/scripts/sport_bridge_test.py

测试项:
    1. stand_up       — StandUp (空闲站立,锁关节)
    2. balance_stand  — BalanceStand (平衡站立,动态可动)
    3. forward        — 前进 0.15 m/s, 3 秒
    4. backward       — 后退 0.10 m/s, 2 秒
    5. rotate_l       — 左转 0.3 rad/s, 2 秒
    6. rotate_r       — 右转 0.3 rad/s, 2 秒
    7. stop           — Move(0,0,0) 停止 (不趴下)
    8. stop_move      — 官方 StopMove (已验不趴)
    9. stand_down     — StandDown 卧倒
    0. recovery       — RecoveryStand 恢复站立
    q. quit
"""

import subprocess
import time
import sys
import os
import json
import select

BRIDGE_PATH = "./sport_bridge"
IFACE = "eth0"

def run_bridge():
    p = subprocess.Popen(
        [BRIDGE_PATH, IFACE],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    time.sleep(1.0)
    return p

def send_and_read(p, cmd_dict, duration_s=None, hz=10):
    """发送命令并读取 stdout 响应。返回 (results, stderr_lines)"""
    if duration_s is None:
        p.stdin.write(json.dumps(cmd_dict) + "\n")
        p.stdin.flush()
        time.sleep(0.5)
    else:
        steps = int(duration_s * hz)
        for _ in range(steps):
            p.stdin.write(json.dumps(cmd_dict) + "\n")
            p.stdin.flush()
            time.sleep(1.0 / hz)
        time.sleep(0.3)

    # 读 stdout
    results = []
    while True:
        r, _, _ = select.select([p.stdout], [], [], 0.2)
        if not r:
            break
        line = p.stdout.readline()
        if line:
            line = line.strip()
            if line.startswith("{"):
                try:
                    results.append(json.loads(line))
                except:
                    results.append(line)

    # 读 stderr
    err_lines = []
    while True:
        r, _, _ = select.select([p.stderr], [], [], 0.1)
        if not r:
            break
        line = p.stderr.readline()
        if line:
            err_lines.append(line.strip())

    return results, err_lines

def stop_and_show(p):
    r, e = send_and_read(p, {"stop": True})
    time.sleep(1)
    return r, e

MENU = """
╔══════════════════════════════════════════════╗
║      sport_bridge 运动验收测试               ║
╠══════════════════════════════════════════════╣
║  1  stand_up      空闲站立 (锁关节)          ║
║  2  balance_stand 平衡站立 (动态可动)        ║
║  3  forward       前进 0.15m/s × 3s         ║
║  4  backward      后退 0.10m/s × 2s         ║
║  5  rotate_l      左转 0.3rad/s × 2s        ║
║  6  rotate_r      右转 0.3rad/s × 2s        ║
║  7  stop          Move(0,0,0) 停止          ║
║  8  stop_move     StopMove API              ║
║  9  stand_down    卧倒                      ║
║  0  recovery      RecoveryStand 恢复        ║
║  q  退出                                    ║
╚══════════════════════════════════════════════╝
"""

def show_result(label, results, errs):
    """显示命令结果"""
    if results:
        for r in results:
            status = r.get("ok") or r.get("error") or str(r)
            prefix = "✓" if "ok" in r else "✗" if "error" in r else "·"
            print(f"  {prefix} {status}")
    for e in errs:
        if "failed" in e.lower() or "error" in e.lower():
            print(f"  ⚠ {e}")

def main():
    print(MENU)

    p = run_bridge()
    print("[bridge] 已启动\n")

    try:
        while True:
            choice = input("选测试项 [0-9/q] > ").strip()

            if choice == "q":
                break

            if choice == "1":
                print("→ stand_up (空闲站立,锁关节) ...")
                r, e = send_and_read(p, {"stand_up": True})
                stop_and_show(p)
                show_result("stand_up", r, e)

            elif choice == "2":
                print("→ balance_stand (平衡站立,动态) ...")
                r, e = send_and_read(p, {"balance_stand": True})
                stop_and_show(p)
                show_result("balance_stand", r, e)

            elif choice == "3":
                print("→ forward 0.15m/s × 3s ...")
                r, e = send_and_read(p, {"vx": 0.15, "vy": 0, "vyaw": 0}, duration_s=3)
                stop_and_show(p)
                show_result("forward", r, e)

            elif choice == "4":
                print("→ backward 0.10m/s × 2s ...")
                r, e = send_and_read(p, {"vx": -0.10, "vy": 0, "vyaw": 0}, duration_s=2)
                stop_and_show(p)
                show_result("backward", r, e)

            elif choice == "5":
                print("→ rotate_l 0.3rad/s × 2s ...")
                r, e = send_and_read(p, {"vx": 0, "vy": 0, "vyaw": 0.3}, duration_s=2)
                stop_and_show(p)
                show_result("rotate_l", r, e)

            elif choice == "6":
                print("→ rotate_r 0.3rad/s × 2s ...")
                r, e = send_and_read(p, {"vx": 0, "vy": 0, "vyaw": -0.3}, duration_s=2)
                stop_and_show(p)
                show_result("rotate_r", r, e)

            elif choice == "7":
                print("→ stop (Move(0,0,0)) ...")
                r, e = send_and_read(p, {"stop": True})
                time.sleep(1)
                show_result("stop", r, e)

            elif choice == "8":
                print("→ stop_move (官方 StopMove) ...")
                r, e = send_and_read(p, {"stop_move": True})
                time.sleep(1)
                show_result("stop_move", r, e)

            elif choice == "9":
                print("→ stand_down (卧倒) ...")
                r, e = send_and_read(p, {"stand_down": True})
                time.sleep(2)
                show_result("stand_down", r, e)

            elif choice == "0":
                print("→ recovery ...")
                r, e = send_and_read(p, {"recovery": True})
                time.sleep(3)
                stop_and_show(p)
                show_result("recovery", r, e)

            else:
                print("  无效选项")

            print()

    finally:
        print("[bridge] 正在退出...")
        stop_and_show(p)
        p.stdin.close()
        p.wait(timeout=3)
        print("[bridge] 已退出")


if __name__ == "__main__":
    main()
