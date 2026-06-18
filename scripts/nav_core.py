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
        #          | 'SAFETY_BLOCKED' | 'TIMEOUT' | 'BRIDGE_ERROR' | 'STUCK'

Architecture:
    Python (this file)              C++ (sport_bridge)
    ─────────────                   ──────────────────
    send_move(vx,vy,vyaw) ──────►  Move(vx,vy,vyaw)
    (every 100ms heartbeat)         reset deadman timer
                                    if 500ms no command → Stop()
    send_stop() ───────────────►   Stop()

Gateway world_state is read by a background thread into a cache.
Pose/clearance/safety have timestamps; stale data (>1s) blocks motion.
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
MIN_SPEED_MPS = 0.15            # minimum observed start speed for GO2W on site
REDUCED_SPEED_FACTOR = 0.5      # speed multiplier when front clearance is moderate
CLEARANCE_SAFE_M = 0.50         # above this: full speed (normal tracking)
ROTATE_SPEED_RADPS = 0.30       # rotation speed (also used as vyaw clamp)
K_YAW = 0.5                     # yaw correction proportional gain

# ── Lateral evasion ──
MAX_LATERAL_SPEED_MPS = 0.30    # hardware speed cap for vy (matches bridge MAX_ABS_VY)
LATERAL_EVADE_SPEED_MPS = 0.20  # nominal vy during lateral evade
MIN_LATERAL_SPEED_MPS = 0.15    # minimum vy that produces measurable displacement
DIAGONAL_EVADE_VX_MPS = 0.03   # optional forward creep when front is not dangerous

VY_LEFT_SIGN = +1.0
VY_RIGHT_SIGN = -1.0

LATERAL_CLEARANCE_SAFE_M = 0.50   # side clearance must exceed this for safe lateral move
LATERAL_CLEARANCE_DANGER_M = 0.30 # below this: side blocked, don't evade that way

EVADE_TARGET_SHIFT_M = 0.30      # exit evasion once lateral progress reaches this
EVADE_MIN_PROGRESS_M = 0.08      # minimum lateral progress in EVADE_SWITCH_TIME_S
EVADE_SWITCH_TIME_S = 2.0        # time before considering direction switch
MAX_EVADE_TIME_S = 8.0           # max total time in evasion before giving up

EVADE_YAW_HOLD_K = 0.8           # proportional gain for yaw hold during lateral move
EVADE_YAW_HOLD_MAX_RADPS = 0.18  # max vyaw during lateral evade

FRONT_EVADE_TRIGGER_M = 0.35     # below this: enter lateral evasion
FRONT_HARD_STOP_M = 0.12         # below this AND no side escape → BLOCKED
FRONT_RESUME_M = 0.50            # above this: exit evasion, resume normal tracking
GATEWAY_POLL_INTERVAL = 0.20    # seconds between get_world_state requests (5 Hz)
DDS_SETTLE_S = 3.0              # seconds to wait for Gateway DDS subscription
STUCK_TIME_S = 5.0              # seconds of no progress -> STUCK
STUCK_DIST_THRESHOLD_M = 0.05   # minimum distance change to count as progress
SELF_OCCLUSION_M = 0.15         # clearance below this → treat as open space (self-occlusion)

# Gateway localization.status values that are considered healthy.
# Gateway reports: 'localized' | 'not_started' | 'lost'.
LOCALIZED_VALUES = ('localized',)


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
        self._safety_allows_navigation: bool = False
        self._safety_reason: str = 'not_initialized'
        self._safety_ts: float = 0.0

        # ── Processes ──
        self._bridge: Optional[subprocess.Popen] = None
        self._gateway: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._poller_thread: Optional[threading.Thread] = None
        self._bridge_stdout_thread: Optional[threading.Thread] = None
        self._bridge_stderr_thread: Optional[threading.Thread] = None
        self._running: bool = False

        # Serialize writes to subprocess stdin. This avoids interleaving
        # poller/shutdown or motion/stop commands on the same pipe.
        self._bridge_send_lock = threading.Lock()
        self._gateway_send_lock = threading.Lock()

    # ═══════════════════════════════════════════════════
    # Lifecycle
    # ═══════════════════════════════════════════════════

    def __enter__(self) -> 'NavigationSession':
        self._start()
        return self

    def __exit__(self, *args):
        self._shutdown()

    def _start(self):
        """Start sport_bridge, Gateway, and reader/poller/drain threads."""
        # Mark running before launching drain/reader threads.
        # The previous version started the bridge drain thread while _running=False,
        # causing the drain thread to exit immediately.
        self._running = True

        # 1. Start sport_bridge (persistent stdin pipe)
        self._bridge = subprocess.Popen(
            [self._bridge_path, self._network_if],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        time.sleep(0.3)
        if self._bridge.poll() is not None:
            stderr_out = self._bridge.stderr.read() if self._bridge.stderr else ''
            self._running = False
            raise RuntimeError(f'sport_bridge failed to start: {stderr_out}')

        # Drain bridge stdout/stderr with separate threads.
        # A single thread doing stdout.readline() then stderr.readline() can block
        # on an idle stdout and never drain stderr.
        self._bridge_stdout_thread = threading.Thread(
            target=self._drain_pipe_loop,
            args=(self._bridge.stdout, 'sport_bridge.stdout'),
            daemon=True,
        )
        self._bridge_stderr_thread = threading.Thread(
            target=self._drain_pipe_loop,
            args=(self._bridge.stderr, 'sport_bridge.stderr'),
            daemon=True,
        )
        self._bridge_stdout_thread.start()
        self._bridge_stderr_thread.start()

        # 2. Start Gateway persistent session
        self._gateway = subprocess.Popen(
            [self._gateway_path, self._network_if, '--persistent-world-state-session'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        time.sleep(0.3)
        if self._gateway.poll() is not None:
            self._shutdown()
            raise RuntimeError('Gateway failed to start')

        # 3. Start reader thread (reads Gateway stdout)
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # 4. Start poller thread (keeps Gateway feeding world_state).
        # If Gateway already streams world_state, these requests are harmless;
        # if it is request/response only, this keeps pose/clearance fresh.
        self._poller_thread = threading.Thread(target=self._gateway_poll_loop, daemon=True)
        self._poller_thread.start()

        # 5. Wait for DDS subscription to settle
        time.sleep(DDS_SETTLE_S)

        # 6. Wait for first valid pose, clearance, and safety decision.
        # Motion must not start with default clearance/safety values.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._pose_ts > 0 and self._clearance_ts > 0 and self._safety_ts > 0:
                    break
            time.sleep(0.3)
        else:
            pose_age = self.pose_age_s
            clearance_age = self.clearance_age_s
            safety_age = self.safety_age_s
            self._shutdown()
            raise RuntimeError(
                'Gateway did not return fresh pose+clearance+safety within 10s '
                f'(pose_age={pose_age:.1f}s, clearance_age={clearance_age:.1f}s, '
                f'safety_age={safety_age:.1f}s)'
            )

    def _shutdown(self):
        """Stop robot, terminate processes, join threads."""
        # Stop robot first while the bridge is still alive.
        self._send_bridge({'stop': True})

        # Stop poller/reader/drain loops.
        self._running = False

        # Close stdin to trigger clean exit
        for proc, name in [(self._bridge, 'sport_bridge'), (self._gateway, 'gateway')]:
            if proc is not None and proc.poll() is None:
                try:
                    if proc.stdin:
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
    # Bridge drain thread (prevents pipe blocking)
    # ═══════════════════════════════════════════════════

    def _drain_pipe_loop(self, pipe, name: str):
        """Continuously drain one pipe to prevent subprocess pipe blocking.

        The content is intentionally ignored. For debugging, replace the body
        with a rate-limited logger or write to a file.
        """
        if pipe is None:
            return

        while self._running:
            try:
                line = pipe.readline()
                if not line:
                    time.sleep(0.1)
                    continue
                # Intentionally discard. Keeping this silent avoids flooding logs.
                # Uncomment when debugging:
                # print(f'[{name}] {line.rstrip()}')
            except Exception:
                time.sleep(0.2)

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
                now = time.monotonic()

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

                # Parse Gateway safety decision. This is the authoritative
                # motion gate; clearance values alone are not enough.
                safety = ws.get('safety', {})
                if isinstance(safety, dict) and 'allow_navigation' in safety:
                    with self._lock:
                        self._safety_allows_navigation = bool(safety.get('allow_navigation') is True)
                        self._safety_reason = str(safety.get('reason') or 'unknown')
                        self._safety_ts = now

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
                        # Apply self-occlusion filter: values below threshold
                        # are XT16 seeing the robot's own body → treat as open space.
                        for k in list(self._clearance):
                            if self._clearance[k] < SELF_OCCLUSION_M:
                                self._clearance[k] = 2.0

            except Exception:
                time.sleep(0.1)

    # ═══════════════════════════════════════════════════
    # Gateway poll thread (ensures world_state keeps flowing)
    # ═══════════════════════════════════════════════════

    def _gateway_poll_loop(self):
        """Periodically request world_state from Gateway.

        Polls at 5 Hz (200ms). If Gateway already streams world_state,
        these requests are harmless; if request/response only, this
        keeps pose/clearance fresh without overloading the DDS bus.
        """
        while self._running:
            self._send_gateway({
                'action': 'get_world_state',
                'request_id': f'poll_{int(time.monotonic() * 1000)}',
            })
            time.sleep(GATEWAY_POLL_INTERVAL)

    # ═══════════════════════════════════════════════════
    # Bridge / Gateway communication
    # ═══════════════════════════════════════════════════

    def _send_bridge(self, cmd: dict) -> bool:
        """Send a JSON command to sport_bridge.

        Returns False if the bridge is dead or the pipe write failed.
        """
        if self._bridge is None or self._bridge.poll() is not None or self._bridge.stdin is None:
            return False
        try:
            with self._bridge_send_lock:
                self._bridge.stdin.write(json.dumps(cmd) + '\n')
                self._bridge.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def _send_gateway(self, cmd: dict) -> bool:
        """Send a JSON command to Gateway.

        Returns False if Gateway is dead or the pipe write failed.
        """
        if self._gateway is None or self._gateway.poll() is not None or self._gateway.stdin is None:
            return False
        try:
            with self._gateway_send_lock:
                self._gateway.stdin.write(json.dumps(cmd) + '\n')
                self._gateway.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

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
            return time.monotonic() - self._pose_ts

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
            return time.monotonic() - self._clearance_ts

    @property
    def loc_status(self) -> str:
        """Localization status: 'localized' | 'not_started' | 'lost'."""
        with self._lock:
            return self._loc_status

    @property
    def safety_age_s(self) -> float:
        """Seconds since last Gateway safety decision."""
        with self._lock:
            if self._safety_ts == 0:
                return float('inf')
            return time.monotonic() - self._safety_ts

    @property
    def safety_reason(self) -> str:
        """Gateway safety reason from the latest world_state."""
        with self._lock:
            return self._safety_reason

    def is_pose_fresh(self) -> bool:
        return self.pose_age_s < STALE_THRESHOLD_S

    def is_clearance_fresh(self) -> bool:
        return self.clearance_age_s < STALE_THRESHOLD_S

    def is_safety_fresh(self) -> bool:
        return self.safety_age_s < STALE_THRESHOLD_S

    def gateway_allows_motion(self) -> bool:
        with self._lock:
            return bool(self._safety_allows_navigation)

    def is_localized(self) -> bool:
        """True if SLAM reports a healthy localization status.

        Gateway reports: 'localized' | 'not_started' | 'lost'.
        Only 'localized' permits motion.
        """
        with self._lock:
            return self._loc_status in LOCALIZED_VALUES

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
        start_t = time.monotonic()
        last_heartbeat = 0.0

        while time.monotonic() - start_t < timeout:
            now = time.monotonic()

            # Read current yaw
            with self._lock:
                yaw = self._pose[2]

            diff = target_yaw - yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))

            if abs(diff) < tolerance:
                self._send_bridge({'stop': True})
                return True

            # ── Safety checks ──
            if not self.is_localized():
                self._send_bridge({'stop': True})
                print('[rotate_to] loc_status not localized, stopping')
                return False

            if not self.is_pose_fresh():
                self._send_bridge({'stop': True})
                print('[rotate_to] pose stale, stopping')
                return False

            if not self.is_clearance_fresh():
                self._send_bridge({'stop': True})
                print('[rotate_to] clearance stale, stopping')
                return False

            if not self.is_safety_fresh() or not self.gateway_allows_motion():
                self._send_bridge({'stop': True})
                print(f'[rotate_to] safety blocked: {self.safety_reason}')
                return False

            if not self._bridge_alive():
                print('[rotate_to] sport_bridge died')
                return False

            # ── Side clearance: conservative (min of left/right/rear) ──
            clearance = self.clearance
            min_side = min(
                clearance.get('left', 2.0),
                clearance.get('right', 2.0),
                clearance.get('rear', 2.0),
            )
            if min_side < LATERAL_CLEARANCE_DANGER_M:
                print(f'[rotate_to] min side clearance {min_side:.2f}m < {LATERAL_CLEARANCE_DANGER_M}m, blocking rotation')
                self._send_bridge({'stop': True})
                return False

            # ── Heartbeat ──
            if now - last_heartbeat >= HEARTBEAT_INTERVAL:
                vyaw = ROTATE_SPEED_RADPS if diff > 0 else -ROTATE_SPEED_RADPS
                if not self._send_bridge({
                    'vx': 0.0, 'vy': 0.0, 'vyaw': vyaw,
                }):
                    print('[rotate_to] BRIDGE_ERROR: failed to send heartbeat')
                    return False
                last_heartbeat = now

            time.sleep(LOOP_SLEEP)

        self._send_bridge({'stop': True})
        print(f'[rotate_to] timeout after {timeout:.0f}s')
        return False

    @staticmethod
    def _lateral_progress_from(start_pose, current_pose, evade_dir):
        """Compute lateral displacement projected onto the robot's lateral axis.

        Positive means the robot moved in the desired evade direction.
        Uses the yaw at the start of evasion as the reference frame.
        """
        sx, sy, syaw = start_pose
        x, y, _ = current_pose
        dx = x - sx
        dy = y - sy
        # Project world-frame displacement onto the robot's lateral axis
        lateral = -math.sin(syaw) * dx + math.cos(syaw) * dy
        if evade_dir > 0:
            return lateral
        return -lateral

    def move_to_xy(self, target_x: float, target_y: float,
                   goal_tol: float = GOAL_TOLERANCE_M,
                   max_speed: float = MAX_SPEED_MPS,
                   max_time: float = 30.0) -> str:
        """Closed-loop coordinate tracking with lateral evasion.

        Normal tracking uses forward motion + yaw correction.
        When front clearance drops below FRONT_EVADE_TRIGGER_M, the robot
        switches to lateral (vy) evasion instead of rotating in place.

        Returns:
            'ARRIVED'       — within goal_tol of target
            'BLOCKED'       — obstacle too close AND no side escape
            'LOST'          — SLAM not localized or pose stale >1s
            'SENSOR_STALE'  — XT16 clearance stale >1s
            'SAFETY_BLOCKED' — Gateway safety.allow_navigation is false/stale
            'TIMEOUT'       — max_time exceeded
            'STUCK'         — no progress (forward: dist; lateral: lateral_progress)
            'BRIDGE_ERROR'  — sport_bridge process died
        """
        start_t = time.monotonic()
        last_heartbeat = 0.0

        # ── Lateral evasion state ──
        self._evade_mode = None        # None | 'lateral'
        self._evade_t0 = 0.0           # when current evade episode started
        self._evade_attempt_t0 = 0.0   # when current direction attempt started
        self._evade_start_pose = None  # (x, y, yaw) when evade episode started
        self._evade_start_yaw = 0.0    # yaw to hold during lateral motion
        self._evade_dir = 0            # +1 left, -1 right, 0 none
        self._evade_last_progress = 0.0
        self._evade_switch_count = 0

        # Stuck detection state
        stuck_t0 = time.monotonic()
        stuck_dist_at_t0 = float('inf')

        while time.monotonic() - start_t < max_time:
            now = time.monotonic()

            # ── Read state ──
            with self._lock:
                x, y, yaw = self._pose
            clearance = self.clearance

            # ── Freshness + localization checks ──
            if not self.is_localized():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] LOST: loc_status={self.loc_status}')
                return 'LOST'

            if not self.is_pose_fresh():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] LOST: pose age={self.pose_age_s:.1f}s')
                return 'LOST'

            if not self.is_clearance_fresh():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] SENSOR_STALE: clearance age={self.clearance_age_s:.1f}s')
                return 'SENSOR_STALE'

            if not self.is_safety_fresh() or not self.gateway_allows_motion():
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] SAFETY_BLOCKED: {self.safety_reason}')
                return 'SAFETY_BLOCKED'

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
            left = clearance.get('left', 2.0)
            right = clearance.get('right', 2.0)

            # Hard stop: front critically close AND no side escape
            if front < FRONT_HARD_STOP_M:
                left_safe = left >= LATERAL_CLEARANCE_DANGER_M
                right_safe = right >= LATERAL_CLEARANCE_DANGER_M
                if not left_safe and not right_safe:
                    self._send_bridge({'stop': True})
                    print(f'[move_to_xy] BLOCKED: front={front:.2f}m < {FRONT_HARD_STOP_M}m, no side escape')
                    return 'BLOCKED'
                # Front is critical but at least one side is clear — will evade below
                print(f'[move_to_xy] front={front:.2f}m critical but side escape available (L={left:.2f} R={right:.2f})')

            # ── Lateral evasion ──
            in_evasion = (self._evade_mode == 'lateral')
            if front < FRONT_EVADE_TRIGGER_M and not in_evasion:
                # Enter lateral evasion
                self._evade_mode = 'lateral'
                self._evade_t0 = now
                self._evade_attempt_t0 = now
                self._evade_start_pose = (x, y, yaw)
                self._evade_start_yaw = yaw
                self._evade_last_progress = 0.0
                self._evade_switch_count = 0

                # Direction selection (4-direction fallback)
                left_ok = left >= LATERAL_CLEARANCE_SAFE_M
                right_ok = right >= LATERAL_CLEARANCE_SAFE_M
                if left_ok and (not right_ok or left > right):
                    self._evade_dir = +1
                elif right_ok:
                    self._evade_dir = -1
                else:
                    # Neither side safe enough — but don't BLOCKED yet if above hard stop
                    if front >= FRONT_HARD_STOP_M:
                        self._evade_dir = 0  # creep: no lateral, no forward
                    else:
                        self._send_bridge({'stop': True})
                        print(f'[move_to_xy] BLOCKED: front={front:.2f}m, no safe side (L={left:.2f} R={right:.2f})')
                        return 'BLOCKED'

                side_name = 'left' if self._evade_dir > 0 else ('right' if self._evade_dir < 0 else 'none')
                print(f'[move_to_xy] ENTER EVADE front={front:.2f}m → {side_name} (L={left:.2f} R={right:.2f})')
                in_evasion = True

            if in_evasion:
                # ── Evasion timeout ──
                if now - self._evade_t0 > MAX_EVADE_TIME_S:
                    self._send_bridge({'stop': True})
                    print(f'[move_to_xy] BLOCKED: evasion timeout after {now - self._evade_t0:.0f}s')
                    return 'BLOCKED'

                # ── Lateral progress check ──
                if self._evade_start_pose is not None and self._evade_dir != 0:
                    lateral_progress = self._lateral_progress_from(
                        self._evade_start_pose, (x, y, yaw), self._evade_dir)
                else:
                    lateral_progress = 0.0

                # ── Exit conditions ──
                exit_evasion = False
                if front >= FRONT_RESUME_M:
                    print(f'[move_to_xy] EXIT EVADE: front={front:.2f}m >= {FRONT_RESUME_M}m')
                    exit_evasion = True
                elif lateral_progress >= EVADE_TARGET_SHIFT_M:
                    if front >= FRONT_EVADE_TRIGGER_M:
                        print(f'[move_to_xy] EXIT EVADE: lateral={lateral_progress:.2f}m, front={front:.2f}m ok')
                        exit_evasion = True
                    elif self._evade_switch_count == 0:
                        # Lateral goal reached but front still blocked → try other side
                        self._evade_dir = -self._evade_dir
                        self._evade_switch_count += 1
                        self._evade_attempt_t0 = now
                        self._evade_start_pose = (x, y, yaw)
                        self._evade_start_yaw = yaw
                        new_side = 'left' if self._evade_dir > 0 else 'right'
                        print(f'[move_to_xy] EVADE lateral={lateral_progress:.2f}m but front={front:.2f}m → switch {new_side}')
                    # else: already switched both sides → stay in evasion, MAX_EVADE_TIME_S will fire

                if exit_evasion:
                    self._evade_mode = None
                    self._evade_t0 = 0.0
                    self._evade_start_pose = None
                    self._evade_dir = 0
                    self._evade_switch_count = 0
                    in_evasion = False

            if in_evasion and self._evade_dir != 0:
                # ── Direction switch check ──
                dir_side = 'left' if self._evade_dir > 0 else 'right'
                dir_clearance = left if self._evade_dir > 0 else right
                other_clearance = right if self._evade_dir > 0 else left
                other_ok = other_clearance >= LATERAL_CLEARANCE_SAFE_M

                should_switch = False
                if now - self._evade_attempt_t0 > EVADE_SWITCH_TIME_S:
                    if lateral_progress < EVADE_MIN_PROGRESS_M:
                        should_switch = True
                    elif dir_clearance < LATERAL_CLEARANCE_DANGER_M and other_ok:
                        should_switch = True

                if should_switch and self._evade_switch_count == 0:
                    self._evade_dir = -self._evade_dir
                    self._evade_switch_count += 1
                    self._evade_attempt_t0 = now
                    self._evade_start_pose = (x, y, yaw)
                    self._evade_start_yaw = yaw
                    new_side = 'left' if self._evade_dir > 0 else 'right'
                    print(f'[move_to_xy] EVADE SWITCH → {new_side} (progress={lateral_progress:.2f}m, {dir_side}={dir_clearance:.2f}m)')
                elif should_switch and self._evade_switch_count >= 1:
                    self._send_bridge({'stop': True})
                    print(f'[move_to_xy] BLOCKED: both sides failed after switch')
                    return 'BLOCKED'

                # ── Evade motion: lateral vy + yaw hold ──
                vx = 0.0
                if self._evade_dir > 0:
                    vy = VY_LEFT_SIGN * LATERAL_EVADE_SPEED_MPS
                elif self._evade_dir < 0:
                    vy = VY_RIGHT_SIGN * LATERAL_EVADE_SPEED_MPS
                else:
                    vy = 0.0
                # Optional diagonal: add forward creep if front is not dangerous
                if front > LATERAL_CLEARANCE_DANGER_M:
                    vx = DIAGONAL_EVADE_VX_MPS

                # Yaw hold: keep the heading we had when evasion started
                yaw_err = self._evade_start_yaw - yaw
                yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
                vyaw = max(-EVADE_YAW_HOLD_MAX_RADPS,
                           min(EVADE_YAW_HOLD_MAX_RADPS, EVADE_YAW_HOLD_K * yaw_err))

                # ── Stuck detection (lateral mode) ──
                # Reset if making progress or front improving
                if lateral_progress > self._evade_last_progress + STUCK_DIST_THRESHOLD_M:
                    stuck_t0 = now
                    stuck_dist_at_t0 = dist
                self._evade_last_progress = lateral_progress
                if vx == 0.0:
                    stuck_t0 = now  # pure lateral: don't false-trigger forward stuck

            elif in_evasion and self._evade_dir == 0:
                # No side escape available and no body_corridor → cannot safely creep.
                # Blind forward creep without body corridor validation risks collision.
                # Phase 1: return BLOCKED. Phase 2: restore conditional creep via body_corridor.passable.
                self._send_bridge({'stop': True})
                print(f'[move_to_xy] BLOCKED: front={front:.2f}m, no side escape, no body_corridor (L={left:.2f} R={right:.2f})')
                return 'BLOCKED'

            else:
                # ── Normal tracking: forward motion + yaw correction ──
                vy = 0.0
                if abs(yaw_error) > LARGE_YAW_THRESHOLD:
                    vx = 0.0
                    vyaw = ROTATE_SPEED_RADPS if yaw_error > 0 else -ROTATE_SPEED_RADPS
                    # Side clearance check for rotation
                    min_side = min(left, right, clearance.get('rear', 2.0))
                    if min_side < LATERAL_CLEARANCE_DANGER_M:
                        self._send_bridge({'stop': True})
                        print(f'[move_to_xy] BLOCKED: min side clearance {min_side:.2f}m, cannot rotate')
                        return 'BLOCKED'
                else:
                    speed_factor = 1.0 if front >= CLEARANCE_SAFE_M else REDUCED_SPEED_FACTOR
                    vx = min(max_speed * speed_factor, dist * 0.5)
                    vx = max(vx, MIN_SPEED_MPS)
                    vyaw = max(-ROTATE_SPEED_RADPS, min(ROTATE_SPEED_RADPS, K_YAW * yaw_error))

                # ── Stuck detection (forward mode) ──
                if dist < stuck_dist_at_t0 - STUCK_DIST_THRESHOLD_M:
                    stuck_t0 = now
                    stuck_dist_at_t0 = dist
                elif now - stuck_t0 > STUCK_TIME_S and vx > 0:
                    self._send_bridge({'stop': True})
                    print(f'[move_to_xy] STUCK: dist={dist:.2f}m unchanged for {now - stuck_t0:.1f}s')
                    return 'STUCK'
                elif vx == 0:
                    stuck_t0 = now
                    stuck_dist_at_t0 = dist

            # ── Heartbeat ──
            if now - last_heartbeat >= HEARTBEAT_INTERVAL:
                if not self._send_bridge({
                    'vx': vx, 'vy': vy, 'vyaw': vyaw,
                }):
                    print('[move_to_xy] BRIDGE_ERROR: failed to send heartbeat')
                    return 'BRIDGE_ERROR'
                last_heartbeat = now

            time.sleep(LOOP_SLEEP)

        # Timeout
        self._send_bridge({'stop': True})
        print(f'[move_to_xy] TIMEOUT after {max_time:.0f}s, dist remaining={dist:.2f}m')
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
            start = time.monotonic()
            while time.monotonic() - start < 30:
                x, y, yaw = nav.pose
                c = nav.clearance
                print(
                    f'[{time.monotonic() - start:5.1f}s] '
                    f'pose=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°) '
                    f'loc={nav.loc_status} '
                    f'front={c["front"]:.2f}m left={c["left"]:.2f}m '
                    f'safety={nav.gateway_allows_motion()}:{nav.safety_reason} '
                    f'age: pose={nav.pose_age_s:.1f}s clr={nav.clearance_age_s:.1f}s '
                    f'safety={nav.safety_age_s:.1f}s'
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
