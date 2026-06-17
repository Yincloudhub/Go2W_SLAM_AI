# GO2W 运动控制 — GPT 审查包

> 2026-06-17 | 仅两个文件：`sport_bridge.cpp` + `nav_core.py`
> 不包含 waypoint_nav 重构、agent_intent、TTS/拍照

---

## 背景（30 秒版）

- GO2W 机器狗，Unitree `slam_operate navigate_to_pose` 在 PCD relocate 后不能驱动机器人移动
- 自研方案：SLAM→定位源，PCD路点图→BFS路由，sport_bridge→底盘速度，nav_core→本地闭环控制
- 上一轮 GPT 反馈要点已全部收下（见下方对照表）

---

## GPT 反馈 → 实现对照

| GPT 要求 | 实现位置 |
|----------|---------|
| `move_to_xy()` 替代 `move_ahead()` | `nav_core.py:382-485` |
| deadman 0.5s（不 5s） | `sport_bridge.cpp:37,85-108` |
| Python 心跳每 100ms | `nav_core.py:46,474-478` |
| 四向净空 + 侧向检查 | `nav_core.py:81-83,355-365,457-465` |
| 数据新鲜度 >1s = stop | `nav_core.py:48,306-310,416-428` |
| 返回明确状态码 | `nav_core.py:13-14,396-402` |
| 持久会话（不每次起进程） | `nav_core.py:62-91,104-147` |
| 不碰 waypoint_nav/agent_intent | ❌ 未动 |

---

## 文件 1/2 — `robot/slam_gateway_refactor/src/sport_bridge.cpp`

```cpp
/** sport_bridge — stdin velocity control with deadman watchdog.
 *
 * Usage: ./sport_bridge eth0
 *
 * Input (stdin, one JSON per line):
 *   {"vx": 0.2, "vy": 0.0, "vyaw": 0.0}    // continuous move at velocity
 *   {"stop": true}                           // stop immediately
 *
 * Deadman: if no command received within 500ms, auto-Stop.
 * Python must send a heartbeat (move or stop) at least every 500ms.
 *
 * Output (stdout): {"ok": "true"} or {"error": "reason"}
 *
 * On stdin EOF or process exit: Stop is guaranteed.
 */

#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>

#include <json.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>

namespace {

constexpr const char* SPORT_SERVICE = "sport";
constexpr const char* SPORT_API_VERSION = "1.0.0.0";
constexpr int32_t SPORT_API_ID_MOVE = 1001;
constexpr int32_t SPORT_API_ID_STOP = 1002;

// ── Deadman watchdog ──
constexpr int64_t DEADMAN_TIMEOUT_MS = 500;   // auto-stop after 500ms of silence
constexpr int64_t WATCHDOG_TICK_MS = 50;      // check every 50ms

std::atomic<int64_t> g_last_command_ms{0};
std::atomic<bool> g_watchdog_running{true};
std::atomic<bool> g_stop_requested{false};

class SportClient : public unitree::robot::Client {
public:
    SportClient() : unitree::robot::Client(SPORT_SERVICE, false) {}

    void Init() override {
        SetApiVersion(SPORT_API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(SPORT_API_ID_MOVE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(SPORT_API_ID_STOP);
    }

    bool move(float vx, float vy, float vyaw) {
        nlohmann::json param;
        param["data"]["vx"] = vx;
        param["data"]["vy"] = vy;
        param["data"]["vyaw"] = vyaw;
        std::string reply;
        int32_t code = Call(SPORT_API_ID_MOVE, param.dump(), reply);
        return code == 0;
    }

    bool stopMove() {
        nlohmann::json param;
        param["data"] = nlohmann::json::object();
        std::string reply;
        int32_t code = Call(SPORT_API_ID_STOP, param.dump(), reply);
        return code == 0;
    }
};

int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

void respond(const std::string& key, const std::string& value) {
    nlohmann::json j;
    j[key] = value;
    std::cout << j.dump() << std::endl;
}

void watchdog_loop(SportClient* client) {
    while (g_watchdog_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(WATCHDOG_TICK_MS));

        int64_t now = now_ms();
        int64_t last = g_last_command_ms.load();

        if (last == 0) {
            // Not initialised yet — skip
            continue;
        }

        if (g_stop_requested.load()) {
            // Already stopping — don't repeat
            continue;
        }

        if (now - last > DEADMAN_TIMEOUT_MS) {
            std::cerr << "[sport_bridge] DEADMAN triggered: "
                      << (now - last) << "ms since last command" << std::endl;
            g_stop_requested = true;
            client->stopMove();
        }
    }
}

}  // namespace

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return 1;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    SportClient client;
    client.Init();
    client.SetTimeout(2.0f);

    // Seed the watchdog timestamp
    g_last_command_ms = now_ms();

    // Launch watchdog thread (detached — will terminate on process exit)
    std::thread watchdog(watchdog_loop, &client);
    watchdog.detach();

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        auto cmd = nlohmann::json::parse(line, nullptr, false);
        if (cmd.is_discarded()) {
            respond("error", "invalid_json");
            continue;
        }

        // ── ANY valid command resets deadman ──
        g_last_command_ms = now_ms();

        // ── Stop command ──
        if (cmd.value("stop", false)) {
            g_stop_requested = true;
            bool ok = client.stopMove();
            respond(ok ? "ok" : "error", ok ? "true" : "stop_failed");
            continue;
        }

        // ── Move command (continuous velocity) ──
        float vx = cmd.value("vx", 0.0f);
        float vy = cmd.value("vy", 0.0f);
        float vyaw = cmd.value("vyaw", 0.0f);

        if (vx == 0.0f && vy == 0.0f && vyaw == 0.0f) {
            // All-zero velocity => stop
            g_stop_requested = true;
            bool ok = client.stopMove();
            respond(ok ? "ok" : "error", ok ? "true" : "stop_failed");
            continue;
        }

        g_stop_requested = false;
        bool ok = client.move(vx, vy, vyaw);
        if (!ok) {
            respond("error", "move_call_failed");
            continue;
        }

        respond("ok", "true");
    }

    // ── Clean exit: guarantee Stop ──
    g_watchdog_running = false;
    g_stop_requested = true;
    client.stopMove();

    std::cerr << "[sport_bridge] exiting, robot stopped" << std::endl;
    return 0;
}
```

---

## 文件 2/2 — `scripts/nav_core.py`

```python
"""nav_core — persistent navigation session with closed-loop motion primitives.

Usage:
    with NavigationSession() as nav:
        # Read state
        x, y, yaw = nav.pose
        front = nav.clearance['front']

        # Closed-loop motion
        nav.rotate_to(1.57)                      # rotate to yaw
        result = nav.move_to_xy(2.0, -1.5)       # track to coordinate

        # Results: 'ARRIVED' | 'BLOCKED' | 'LOST' | 'SENSOR_STALE'
        #          | 'TIMEOUT' | 'BRIDGE_ERROR'

Architecture:
    Python (this file)              C++ (sport_bridge)
    ─────────────                   ──────────────────
    send_move(vx,vy,vyaw) ──────►  Move(vx,vy,vyaw)
    (every 100ms heartbeat)         reset deadman timer
                                    if 500ms no command → Stop()
    send_stop() ───────────────►   Stop()

Gateway world_state is read by a background thread into a cache.
Pose/clearance have timestamps; stale data (>1s) blocks motion.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import threading
import time
from typing import Dict, Optional, Tuple

# ── Paths (overridable at init) ──
DEFAULT_REPO = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_BRIDGE = os.path.join(DEFAULT_REPO, 'robot', 'slam_gateway_refactor', 'build', 'sport_bridge')
DEFAULT_GATEWAY = os.path.join(DEFAULT_REPO, 'robot', 'slam_gateway_refactor', 'build', 'slam_llm_command_client')
DEFAULT_NETWORK = 'eth0'

# ── Constants ──
HEARTBEAT_INTERVAL = 0.10       # seconds between sport_bridge velocity commands
LOOP_SLEEP = 0.05               # seconds between control loop iterations
STALE_THRESHOLD_S = 1.0         # pose/clearance older than this → stale
GOAL_TOLERANCE_M = 0.25         # considered arrived within 25cm
YAW_TOLERANCE_RAD = 0.1         # ~6 degrees
LARGE_YAW_THRESHOLD = 0.7       # >~40 degrees: rotate in place only
MAX_SPEED_MPS = 0.20            # maximum forward speed
MIN_SPEED_MPS = 0.05            # minimum to overcome static friction
REDUCED_SPEED_FACTOR = 0.5      # speed multiplier when front clearance is moderate
CLEARANCE_SAFE_M = 0.50         # above this: full speed
CLEARANCE_DANGER_M = 0.30       # below this: hard stop
ROTATE_SPEED_RADPS = 0.30       # rotation speed
K_YAW = 0.5                     # yaw correction proportional gain
DDS_SETTLE_S = 3.0              # seconds to wait for Gateway DDS subscription


class NavigationSession:
    """Persistent session holding sport_bridge + Gateway processes.

    Provides closed-loop motion primitives with real-time
    SLAM pose feedback and XT16 clearance monitoring.
    """

    def __init__(self,
                 bridge_path: str = DEFAULT_BRIDGE,
                 gateway_path: str = DEFAULT_GATEWAY,
                 network_if: str = DEFAULT_NETWORK):
        self._bridge_path = bridge_path
        self._gateway_path = gateway_path
        self._network_if = network_if

        # ── Cached state (updated by reader thread) ──
        self._lock = threading.Lock()
        self._pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._pose_ts: float = 0.0
        self._clearance: Dict[str, float] = {
            'front': 2.0, 'left': 2.0, 'right': 2.0, 'rear': 2.0
        }
        self._clearance_ts: float = 0.0
        self._loc_status: str = 'unknown'

        # ── Processes ──
        self._bridge: Optional[subprocess.Popen] = None
        self._gateway: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._running: bool = False

    # ═══════════════════════════════════════════════════
    # Lifecycle
    # ═══════════════════════════════════════════════════

    def __enter__(self) -> 'NavigationSession':
        self._start()
        return self

    def __exit__(self, *args):
        self._shutdown()

    def _start(self):
        """Start sport_bridge, Gateway, and reader thread."""
        # 1. Start sport_bridge (persistent stdin pipe)
        self._bridge = subprocess.Popen(
            [self._bridge_path, self._network_if],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        # Check bridge started
        time.sleep(0.3)
        if self._bridge.poll() is not None:
            stderr_out = self._bridge.stderr.read() if self._bridge.stderr else ''
            raise RuntimeError(f'sport_bridge failed to start: {stderr_out}')

        # 2. Start Gateway persistent session
        self._gateway = subprocess.Popen(
            [self._gateway_path, self._network_if, '--persistent-world-state-session'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        # 3. Start reader thread
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # 4. Wait for DDS subscription to settle, then request initial world_state
        time.sleep(DDS_SETTLE_S)
        self._send_gateway({'action': 'get_world_state', 'request_id': 'nav_init'})

        # 5. Wait for first valid pose
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._lock:
                if self._pose_ts > 0:
                    break
            time.sleep(0.3)
        else:
            self._shutdown()
            raise RuntimeError('Gateway did not return world_state within 10s')

    def _shutdown(self):
        """Stop robot, terminate processes, join threads."""
        self._running = False

        # Stop robot first
        self._send_bridge({'stop': True})

        # Close stdin to trigger clean exit
        for proc, name in [(self._bridge, 'sport_bridge'), (self._gateway, 'gateway')]:
            if proc is not None and proc.poll() is None:
                try:
                    proc.stdin.close()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        proc.kill()

    # ═══════════════════════════════════════════════════
    # Gateway reader thread
    # ═══════════════════════════════════════════════════

    def _reader_loop(self):
        """Background thread: read Gateway stdout, update cached state."""
        while self._running:
            if self._gateway is None or self._gateway.poll() is not None:
                time.sleep(0.5)
                continue

            try:
                line = self._gateway.stdout.readline()
                if not line:
                    time.sleep(0.1)
                    continue

                line = line.strip()
                if not line.startswith('{'):
                    continue

                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if 'world_state' not in obj:
                    continue

                ws = obj['world_state']
                now = time.time()

                # Parse pose
                cp = ws.get('current_pose', {})
                pp = cp.get('pose', {})
                x = pp.get('x')
                y = pp.get('y')
                yaw_val = pp.get('yaw')
                if x is not None and y is not None and yaw_val is not None:
                    with self._lock:
                        self._pose = (float(x), float(y), float(yaw_val))
                        self._pose_ts = now

                # Parse localization status
                loc = ws.get('localization', {})
                with self._lock:
                    self._loc_status = loc.get('status', 'unknown')

                # Parse clearance
                obs = ws.get('local_obstacle', {})
                front_m = obs.get('front_clearance_m')
                left_m = obs.get('left_clearance_m')
                right_m = obs.get('right_clearance_m')
                rear_m = obs.get('rear_clearance_m')

                if front_m is not None:
                    with self._lock:
                        self._clearance = {
                            'front': float(front_m),
                            'left': float(left_m) if left_m is not None else 2.0,
                            'right': float(right_m) if right_m is not None else 2.0,
                            'rear': float(rear_m) if rear_m is not None else 2.0,
                        }
                        self._clearance_ts = now

            except Exception:
                time.sleep(0.1)

    # ═══════════════════════════════════════════════════
    # Bridge / Gateway communication
    # ═══════════════════════════════════════════════════

    def _send_bridge(self, cmd: dict):
        """Send a JSON command to sport_bridge (non-blocking)."""
        if self._bridge is None or self._bridge.poll() is not None:
            return
        try:
            self._bridge.stdin.write(json.dumps(cmd) + '\n')
            self._bridge.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def _send_gateway(self, cmd: dict):
        """Send a JSON command to Gateway."""
        if self._gateway is None or self._gateway.poll() is not None:
            return
        try:
            self._gateway.stdin.write(json.dumps(cmd) + '\n')
            self._gateway.stdin.flush()
        except (BrokenPipeError, OSError):
            pass

    def _bridge_alive(self) -> bool:
        """Check if sport_bridge process is still running."""
        return self._bridge is not None and self._bridge.poll() is None

    # ═══════════════════════════════════════════════════
    # State queries (thread-safe)
    # ═══════════════════════════════════════════════════

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Current (x, y, yaw) from SLAM, with timestamp."""
        with self._lock:
            return self._pose

    @property
    def pose_age_s(self) -> float:
        """Seconds since last pose update."""
        with self._lock:
            if self._pose_ts == 0:
                return float('inf')
            return time.time() - self._pose_ts

    @property
    def clearance(self) -> Dict[str, float]:
        """Current {front, left, right, rear} clearance in meters."""
        with self._lock:
            return dict(self._clearance)

    @property
    def clearance_age_s(self) -> float:
        """Seconds since last clearance update."""
        with self._lock:
            if self._clearance_ts == 0:
                return float('inf')
            return time.time() - self._clearance_ts

    @property
    def loc_status(self) -> str:
        """Localization status: 'localized' | 'not_started' | 'lost'."""
        with self._lock:
            return self._loc_status

    def is_pose_fresh(self) -> bool:
        return self.pose_age_s < STALE_THRESHOLD_S

    def is_clearance_fresh(self) -> bool:
        return self.clearance_age_s < STALE_THRESHOLD_S

    # ═══════════════════════════════════════════════════
    # Motion primitives
    # ═══════════════════════════════════════════════════

    def stop(self):
        """Emergency stop. Blocks until stop command is sent."""
        self._send_bridge({'stop': True})

    def rotate_to(self, target_yaw: float,
                  tolerance: float = YAW_TOLERANCE_RAD,
                  timeout: float = 8.0) -> bool:
        """Closed-loop rotation to target_yaw.

        Returns True if aligned within tolerance, False on timeout/failure.
        """
        start_t = time.time()
        last_heartbeat = 0.0

        while time.time() - start_t < timeout:
            now = time.time()

            # Read current yaw
            with self._lock:
                yaw = self._pose[2]

            diff = target_yaw - yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))  # normalize to [-pi, pi]

            if abs(diff) < tolerance:
                self._send_bridge({'stop': True})
                return True

            # Freshness checks
            if not self.is_pose_fresh():
                self._send_bridge({'stop': True})
                print('[rotate_to] pose stale, stopping')
                return False

            if not self._bridge_alive():
                print('[rotate_to] sport_bridge died')
                return False

            # Check side clearance for rotation direction
            clearance = self.clearance
            vyaw_dir = 1.0 if diff > 0 else -1.0

            if diff > 0 and clearance.get('right', 2.0) < CLEARANCE_DANGER_M:
                print(f'[rotate_to] right clearance {clearance["right"]:.2f}m too close, blocking CW rotation')
                self._send_bridge({'stop': True})
                return False
            if diff < 0 and clearance.get('left', 2.0) < CLEARANCE_DANGER_M:
                print(f'[rotate_to] left clearance {clearance["left"]:.2f}m too close, blocking CCW rotation')
                self._send_bridge({'stop': True})
                return False

            # Send rotation heartbeat
            if now - last_heartbeat >= HEARTBEAT_INTERVAL:
                self._send_bridge({
                    'vx': 0.0, 'vy': 0.0,
                    'vyaw': vyaw_dir * ROTATE_SPEED_RADPS,
                })
                last_heartbeat = now

            time.sleep(LOOP_SLEEP)

        # Timeout
        self._send_bridge({'stop': True})
        print(f'[rotate_to] timeout after {timeout:.0f}s')
        return False

    def move_to_xy(self, target_x: float, target_y: float,
                   goal_tol: float = GOAL_TOLERANCE_M,
                   max_speed: float = MAX_SPEED_MPS,
                   max_time: float = 30.0) -> str:
        """Closed-loop coordinate tracking with continuous yaw correction.

        Every control cycle (~100ms):
          1. Read current pose from Gateway cache
          2. Compute distance and bearing to target
          3. Compute yaw error → if large (>40°), rotate in place
          4. Check XT16 clearance → dangerous → stop; moderate → reduce speed
          5. Check data freshness → stale → stop
          6. Send velocity heartbeat to sport_bridge

        Returns:
            'ARRIVED'       — within goal_tol of target
            'BLOCKED'       — front clearance < 0.30m
            'LOST'          — SLAM pose stale >1s
            'SENSOR_STALE'  — XT16 clearance stale >1s
            'TIMEOUT'       — max_time exceeded
            'BRIDGE_ERROR'  — sport_bridge process died
        """
        start_t = time.time()
        last_heartbeat = 0.0

        while time.time() - start_t < max_time:
            now = time.time()

            # ── Read state ──
            with self._lock:
                x, y, yaw = self._pose
            clearance = self.clearance

            # ── Freshness checks ──
            if not self.is_pose_fresh():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] LOST: pose age={self.pose_age_s:.1f}s')
                return 'LOST'

            if not self.is_clearance_fresh():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] SENSOR_STALE: clearance age={self.clearance_age_s:.1f}s')
                return 'SENSOR_STALE'

            if not self._bridge_alive():
                print('[move_to_xy] BRIDGE_ERROR: sport_bridge died')
                return 'BRIDGE_ERROR'

            # ── Compute errors ──
            dx = target_x - x
            dy = target_y - y
            dist = math.hypot(dx, dy)

            # Arrived?
            if dist < goal_tol:
                self._send_bridge({'stop': True})
                return 'ARRIVED'

            bearing = math.atan2(dy, dx)
            yaw_error = bearing - yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            # ── Safety: front clearance ──
            front = clearance.get('front', 2.0)
            if front < CLEARANCE_DANGER_M:
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] BLOCKED: front={front:.2f}m < {CLEARANCE_DANGER_M}m')
                return 'BLOCKED'

            # ── Compute velocities ──
            if abs(yaw_error) > LARGE_YAW_THRESHOLD:
                # Large yaw error: rotate only, no forward motion
                vx = 0.0
                vyaw = ROTATE_SPEED_RADPS * (1.0 if yaw_error > 0 else -1.0)

                # Check side clearance for rotation direction
                if yaw_error > 0 and clearance.get('right', 2.0) < CLEARANCE_DANGER_M:
                    vx = 0.0
                    vyaw = 0.0
                    print(f'[move_to_xy] blocking CW rotation: right clearance {clearance["right"]:.2f}m')
                elif yaw_error < 0 and clearance.get('left', 2.0) < CLEARANCE_DANGER_M:
                    vx = 0.0
                    vyaw = 0.0
                    print(f'[move_to_xy] blocking CCW rotation: left clearance {clearance["left"]:.2f}m')
            else:
                # Small yaw error: move forward with yaw correction
                speed_factor = 1.0 if front >= CLEARANCE_SAFE_M else REDUCED_SPEED_FACTOR
                vx = min(max_speed * speed_factor, dist * 0.5)  # slow down near target
                vx = max(vx, MIN_SPEED_MPS)
                vyaw = K_YAW * yaw_error  # proportional correction

            # ── Heartbeat ──
            if now - last_heartbeat >= HEARTBEAT_INTERVAL:
                self._send_bridge({
                    'vx': vx, 'vy': 0.0, 'vyaw': vyaw,
                })
                last_heartbeat = now

            time.sleep(LOOP_SLEEP)

        # Timeout
        self._send_bridge({'stop': True})
        print(f'[move_to_xy] TIMEOUT after {max_time:.0f}s')
        return 'TIMEOUT'


# ═══════════════════════════════════════════════════
# Standalone test
# ═══════════════════════════════════════════════════

def main():
    """Quick test: read state for 30 seconds, no motion."""
    print('=== nav_core state read test ===')
    print(f'Bridge: {DEFAULT_BRIDGE}')
    print(f'Gateway: {DEFAULT_GATEWAY}')

    try:
        with NavigationSession() as nav:
            print(f'Connected. loc={nav.loc_status}')
            start = time.time()
            while time.time() - start < 30:
                x, y, yaw = nav.pose
                c = nav.clearance
                print(
                    f'[{time.time() - start:5.1f}s] '
                    f'pose=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°) '
                    f'front={c["front"]:.2f}m left={c["left"]:.2f}m '
                    f'age: pose={nav.pose_age_s:.1f}s clr={nav.clearance_age_s:.1f}s'
                )
                time.sleep(0.5)
    except RuntimeError as e:
        print(f'ERROR: {e}')
        sys.exit(1)
    except KeyboardInterrupt:
        print('\nInterrupted.')

    print('Done.')


if __name__ == '__main__':
    main()
```

---

## 关键设计决策

### 1. `move_to_xy` 控制循环

```
每 100ms 一个 tick:
  read (x,y,yaw) from Gateway cache
  dx = target_x - x,  dy = target_y - y
  dist = hypot(dx, dy)
  if dist < 0.25m → stop → ARRIVED

  bearing = atan2(dy, dx)
  yaw_error = normalize(bearing - yaw)

  if front < 0.30m → stop → BLOCKED
  if pose_age > 1s → stop → LOST
  if clearance_age > 1s → stop → SENSOR_STALE
  if bridge dead → BRIDGE_ERROR

  if |yaw_error| > 40°:
      vx = 0,  vyaw = ±0.30 rad/s  (rotate only)
      check side clearance for rotation direction
  else:
      vx = min(0.20 * safety, dist * 0.5)  (slow down near target)
      vyaw = 0.5 * yaw_error  (P-controller)

  send {vx, vy=0, vyaw} to sport_bridge  (heartbeat resets deadman)
```

### 2. Deadman 安全链

```
Python ──100ms心跳──► sport_bridge
                        │
                        ├─ 收到命令 → reset 500ms 计时器
                        ├─ 500ms 无命令 → watchdog Stop()
                        ├─ stdin EOF → main() 末尾 Stop()
                        └─ 进程被 kill → OS 清理, DDS 停止
                                (但 Go2 有自身 safety controller,
                                 停 DDS 会触发坐下)
```

Go2 自身的 safety controller 在 DDS 心跳丢失后会触发保护行为（坐下），这是额外一层安全网。

### 3. 为什么不用 `move_ahead(distance)`

旧方案：对准 → 直走 distance 米 → 不管偏航。地面摩擦/侧滑/SLAM 抖动都会累积误差。

新方案：`move_to_xy(x,y)` 每 100ms 重新读位姿，持续修正偏航。机器人始终追踪目标坐标，不是盲走固定距离。

### 4. 已知限制

- sport_bridge stdout 无 reader 线程（输出量小，64KB 缓冲区够用 ~200s）
- 未做 PCD 可达性校验的起点匹配（GPT 建议的后续改进）
- 安全距离未根据速度动态调整（第一版先用固定 0.30m）
- Python 3.8 兼容已通过 `py_compile` 验证
