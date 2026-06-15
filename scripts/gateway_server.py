"""GO2W Gateway HTTP Server — lightweight REST API."""
import json, subprocess, sys, os, threading
from http.server import HTTPServer, BaseHTTPRequestHandler

REPO = "/home/unitree/Go2W_SLAM_AI"
GATEWAY_CLIENT = f"{REPO}/robot/slam_gateway_refactor/build/slam_llm_command_client"
INTERFACE = "eth0"
TIMEOUT = 10
_nav_lock = threading.Lock()


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


def get_status() -> dict:
    result = run_gateway("get_world_state")
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
        from edge_autonomy.path_validator import build_coarse_map
        m = build_coarse_map(grid_size=30)
        return {"grid": m["grid_string"], "nodes": m["nodes"]}
    except Exception as e:
        return {"error": str(e)}


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
        if self.path == "/status":
            self._json(get_status())
        elif self.path == "/map":
            self._json(get_coarse_map())
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
            result = run_gateway("relocate", map_id=body.get("map_id", "go2w_real_site"),
                                  map_path=body.get("map_path", "/home/unitree/test.pcd"),
                                  anchor_id=anchor, operator_ack=True)
            self._json(result)
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
                            f"{REPO}/configs/maps/go2w_real_site_map_registry.json")
        map_id = body.get("map_id", "go2w_real_site")
        map_path = body.get("map_path", "/home/unitree/test.pcd")

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
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--interface", default=INTERFACE)
    args = parser.parse_args()
    global INTERFACE
    INTERFACE = args.interface
    server = HTTPServer(("0.0.0.0", args.port), GatewayHandler)
    print(f"GO2W Gateway HTTP on port {args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


if __name__ == "__main__":
    main()
