"""GO2W Gateway HTTP Server — lightweight REST API."""
import json, subprocess, sys, os, time, threading
from http.server import HTTPServer, BaseHTTPRequestHandler

REPO = os.environ.get("GO2W_REPO", "/home/unitree/Go2W_SLAM_AI")
GATEWAY_CLIENT = os.environ.get(
    "GO2W_GATEWAY_CLIENT",
    f"{REPO}/robot/slam_gateway_refactor/build/slam_llm_command_client",
)
DEFAULT_MAP_ID = "go2w_real_site"
DEFAULT_REGISTRY = f"{REPO}/configs/maps/go2w_real_site_map_registry.json"
_HARDCODED_DEFAULT_MAP_PATH = "/home/unitree/maps/staging/map_701.pcd"
INTERFACE = "eth0"
TIMEOUT = 10
_nav_lock = threading.Lock()


def _resolve_default_map_path():
    # type: () -> str
    """Resolve the default PCD path from active_pcd.json, falling back to hardcoded value."""
    try:
        _scripts = os.path.join(REPO, "scripts")
        if _scripts not in sys.path:
            sys.path.insert(0, _scripts)
        from nav_relocate import get_active_pcd
        active = get_active_pcd()
        return str(active.get("pcd_path", _HARDCODED_DEFAULT_MAP_PATH))
    except Exception:
        return _HARDCODED_DEFAULT_MAP_PATH


DEFAULT_MAP_PATH = _resolve_default_map_path()


def resolve_map_path(
    registry_path: str = "",
    map_id: str = DEFAULT_MAP_ID,
    explicit_map_path: str = "",
) -> str:
    """Resolve the map PCD without touching Gateway or robot processes."""
    if explicit_map_path:
        return explicit_map_path
    path = registry_path or DEFAULT_REGISTRY
    try:
        with open(path, "r", encoding="utf-8") as fh:
            registry = json.load(fh)
        for item in registry.get("maps", []):
            if item.get("map_id") == map_id and item.get("pcd_path"):
                return str(item["pcd_path"])
    except (OSError, json.JSONDecodeError):
        pass
    return DEFAULT_MAP_PATH


def run_gateway(action: str, **kwargs) -> dict:
    command = {"action": action, **kwargs}
    payload = json.dumps(command, ensure_ascii=False) + "\n"
    proc = subprocess.Popen(
        [GATEWAY_CLIENT, INTERFACE],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True,
    )
    try:
        stdout, stderr = proc.communicate(payload, timeout=TIMEOUT)
    except subprocess.TimeoutExpired:
        proc.kill()
        return {"accepted": False, "reason": "gateway timeout"}
    if proc.returncode != 0:
        return {"accepted": False, "reason": stderr.strip() or f"exit {proc.returncode}"}
    for line in stdout.split("\n"):
        line = line.strip()
        if not line or not line.startswith("{"):
            continue
        try:
            obj = json.loads(line)
            if "accepted" in obj:
                return obj
        except json.JSONDecodeError:
            continue
    return {"accepted": False, "reason": "no valid gateway response"}


def get_status(map_id: str = DEFAULT_MAP_ID,
               map_path: str = DEFAULT_MAP_PATH) -> dict:
    result = run_gateway("get_world_state",
                         map_id=map_id,
                         map_path=map_path)
    ws = result.get("world_state", {})
    summary = {
        "accepted": result.get("accepted"),
        "localized": ws.get("localization", {}).get("status") == "localized",
        "slam_ok": ws.get("slam_health", {}).get("status") == "ok",
    }
    pose = ws.get("current_pose", {}).get("pose", {})
    if pose:
        summary["pose"] = {"x": pose.get("x"), "y": pose.get("y"), "yaw": pose.get("yaw")}
    nav = ws.get("navigation", {})
    if nav:
        summary["navigation"] = {"state": nav.get("state"), "target": nav.get("target_node"),
                                  "distance": nav.get("distance_to_goal_m")}
    safety = ws.get("safety", {})
    if safety:
        summary["safety"] = {"allow": safety.get("allow_navigation"), "reason": safety.get("reason"),
                              "speed_limit": safety.get("speed_limit_mps")}
    obstacle = ws.get("local_obstacle", {})
    if obstacle:
        summary["lidar"] = {"front": obstacle.get("front_clearance_m"),
                            "left": obstacle.get("left_clearance_m"),
                            "right": obstacle.get("right_clearance_m"),
                            "rear": obstacle.get("rear_clearance_m")}
    return summary


def get_coarse_map() -> dict:
    try:
        sys.path.insert(0, f"{REPO}/src")
        from edge_autonomy.path_validator import build_adaptive_coarse_map
        m = build_adaptive_coarse_map(
            pcd_path=DEFAULT_MAP_PATH,
            grid_size=30,
            registry_path=DEFAULT_REGISTRY,
            map_id=DEFAULT_MAP_ID,
        )
        return {"grid": m["grid_string"], "nodes": m["nodes"]}
    except Exception as e:
        return {"error": str(e)}


def get_planner_context(
    user_command: str = "",
    map_id: str = DEFAULT_MAP_ID,
    registry_path: str = "",
    map_path: str = "",
) -> dict:
    """Build cloud LLM planner context: coarse_map + waypoint connectivity + current state.

    Args:
        user_command: Natural language command (e.g. "去赵博那")
        map_id: Map identifier in the registry
        registry_path: Path to map registry JSON. Defaults to REPO default.
    """
    try:
        sys.path.insert(0, f"{REPO}/src")
        from edge_autonomy.cloud_llm_planner import build_waypoint_connectivity
        from edge_autonomy.path_validator import build_adaptive_coarse_map

        if not registry_path:
            registry_path = DEFAULT_REGISTRY
        map_path = resolve_map_path(registry_path, map_id, map_path)

        # Coarse map
        try:
            cm = build_adaptive_coarse_map(
                pcd_path=map_path,
                grid_size=40,
                registry_path=registry_path,
                map_id=map_id,
            )
            coarse_map = cm.get("grid_string", "")
            coarse_map_nodes = cm.get("nodes", {})
            coarse_map_grid_size = cm.get("grid_size", 0)
            map_path = cm.get("pcd_path") or map_path
        except Exception:
            coarse_map = ""
            coarse_map_nodes = {}
            coarse_map_grid_size = 0

        # Waypoint connectivity (cached after first call)
        connectivity = build_waypoint_connectivity(
            registry_path,
            map_id=map_id,
            pcd_path=map_path or None,
        )

        # Current state from Gateway (best-effort: context stays useful if the
        # Gateway binary is missing, rebooting, or unreachable — cloud LLM can
        # still plan from coarse_map + connectivity alone)
        try:
            status = get_status(
                map_id=map_id,
                map_path=map_path,
            )
        except Exception:
            status = {
                "accepted": False,
                "reason": "gateway_unavailable",
                "error": "Gateway status unavailable; planning context is map-only",
            }
        current_pose = status.get("pose") if isinstance(status, dict) else None

        return {
            "coarse_map": coarse_map,
            "coarse_map_nodes": coarse_map_nodes,
            "coarse_map_grid_size": coarse_map_grid_size,
            "waypoint_connectivity": {
                "nodes": connectivity.get("nodes", []),
                "adjacency": connectivity.get("adjacency", {}),
                "grid_positions": connectivity.get("grid_positions", {}),
                "edge_source": connectivity.get("edge_source", "unknown"),
                "pcd_path": connectivity.get("pcd_path"),
            },
            "current_pose": current_pose,
            "user_command": user_command,
            "map_id": map_id,
            "status": status,
            "navigation_hint": (
                "Multi-hop routing: examine the coarse_map. "
                "Walls are shown as █, open space as ·. "
                "Topology nodes are overlaid on the grid. "
                "Use waypoint_connectivity.adjacency to find reachable neighbor pairs. "
                "Plan a sequence of hops from current_node to requested_target through "
                "intermediate nodes. Each hop must be a directly connected pair in the adjacency graph. "
                "Output format: {\"hops\": [\"wp_a\", \"wp_b\", \"wp_c\"], \"reason\": \"short explanation\"}"
            ),
        }
    except Exception as e:
        import traceback

        return {
            "error": str(e),
            "error_kind": type(e).__name__,
            "traceback": traceback.format_exc(limit=8),
        }


def handle_cloud_plan(body: dict) -> dict:
    """Receive cloud LLM hop plan, validate, convert to plan, spawn execution."""
    if _nav_lock.locked():
        return {"accepted": False, "reason": "navigation already in progress"}

    hops = body.get("hops")
    if not isinstance(hops, list) or len(hops) < 2:
        return {"accepted": False, "reason": "hops must be a list of at least 2 node IDs"}

    reason = str(body.get("reason", ""))
    map_id = str(body.get("map_id", DEFAULT_MAP_ID))
    registry_path = str(body.get("registry", "") or DEFAULT_REGISTRY)
    map_path = resolve_map_path(registry_path, map_id, str(body.get("map_path", "")))

    try:
        sys.path.insert(0, f"{REPO}/src")
        from edge_autonomy.cloud_llm_planner import hops_to_plan, build_waypoint_connectivity

        connectivity = build_waypoint_connectivity(
            registry_path,
            map_id=map_id,
            pcd_path=map_path or None,
        )
        plan = hops_to_plan(hops, map_id, reason=reason, connectivity=connectivity)

        if plan.get("mode") in ("human_confirm", "safe_hold") and plan.get("confidence", 1.0) < 0.5:
            return {
                "accepted": False,
                "reason": plan.get("reason", "cloud plan rejected by validator"),
                "plan": plan,
            }

        # Write plan to temp file and spawn execution
        import tempfile
        plan_path = os.path.join(tempfile.gettempdir(), f"go2w_cloud_plan_{int(time.time())}.json")
        with open(plan_path, "w", encoding="utf-8") as fh:
            json.dump(plan, fh, ensure_ascii=False)

        target = hops[-1] if len(hops) > 1 else hops[0]
        speed = body.get("speed", 0.2)
        nav_mode = body.get("nav_mode", 0)

        cmd = [
            "python3", f"{REPO}/scripts/run_robot_closed_loop.py",
            "--command", target,
            "--registry", registry_path,
            "--map-id", map_id,
            "--map-path", map_path,
            "--plan-file", plan_path,
            "--execute",
            "--nav-speed-mps", str(speed),
            "--nav-mode", str(nav_mode),
            "--prompt-mode", "hybrid",
        ]

        def run_cloud_nav():
            try:
                subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            except subprocess.TimeoutExpired:
                pass
            finally:
                try:
                    os.unlink(plan_path)
                except OSError:
                    pass
                _nav_lock.release()

        _nav_lock.acquire()
        threading.Thread(target=run_cloud_nav, daemon=True).start()
        return {
            "accepted": True,
            "reason": f"cloud plan accepted: {' → '.join(hops)}",
            "target": target,
            "hops": hops,
        }
    except Exception as e:
        return {"accepted": False, "reason": f"cloud plan error: {e}"}


class GatewayHandler(BaseHTTPRequestHandler):
    def _json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self):
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        params = {k: v[0] for k, v in parse_qs(parsed.query).items()}
        if parsed.path == "/status":
            self._json(get_status(
                map_id=params.get("map_id", DEFAULT_MAP_ID),
                map_path=params.get("map_path", DEFAULT_MAP_PATH),
            ))
        elif parsed.path == "/map":
            self._json(get_coarse_map())
        elif parsed.path == "/planner/context":
            self._json(get_planner_context(
                user_command=params.get("cmd", ""),
                map_id=params.get("map_id", DEFAULT_MAP_ID),
                registry_path=params.get("registry", ""),
                map_path=params.get("map_path", ""),
            ))
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        body = self._read_body()
        if self.path == "/navigate":
            self._handle_navigate(body)
        elif self.path == "/pause":
            self._json(run_gateway("pause_navigation"))
        elif self.path == "/relocate":
            anchor = body.get("anchor", "mapping_origin")
            map_id = body.get("map_id", DEFAULT_MAP_ID)
            registry = body.get("registry", DEFAULT_REGISTRY)
            result = run_gateway("relocate", map_id=map_id,
                                  map_path=resolve_map_path(registry, map_id, body.get("map_path", "")),
                                  anchor_id=anchor, operator_ack=True)
            self._json(result)
        elif self.path == "/planner/plan":
            self._json(handle_cloud_plan(body))
        else:
            self._json({"error": "not found"}, 404)

    def _handle_navigate(self, body):
        """Run navigation in background to avoid blocking the HTTP server."""
        if _nav_lock.locked():
            self._json({"accepted": False, "reason": "navigation already in progress"})
            return

        target = body.get("target", "")
        if not target:
            self._json({"accepted": False, "reason": "missing target"})
            return

        speed = body.get("speed", 0.2)
        registry = body.get("registry",
                            DEFAULT_REGISTRY)
        map_id = body.get("map_id", DEFAULT_MAP_ID)
        map_path = resolve_map_path(registry, map_id, body.get("map_path", ""))

        cmd = [
            "python3", f"{REPO}/scripts/run_robot_closed_loop.py",
            "--command", target,
            "--registry", registry,
            "--map-id", map_id,
            "--map-path", map_path,
            "--execute",
            "--nav-speed-mps", str(speed),
            "--nav-mode", "0",
            "--prompt-mode", "hybrid",
        ]

        def run_nav():
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            except subprocess.TimeoutExpired:
                result = None
            finally:
                _nav_lock.release()

        _nav_lock.acquire()
        threading.Thread(target=run_nav, daemon=True).start()
        self._json({"accepted": True, "reason": f"navigation to {target} started in background"})

    def log_message(self, format, *args):
        pass


def main():
    global INTERFACE
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--interface", default=INTERFACE)
    args = parser.parse_args()
    INTERFACE = args.interface
    server = HTTPServer(("0.0.0.0", args.port), GatewayHandler)
    print(f"GO2W Gateway HTTP on port {args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    main()
