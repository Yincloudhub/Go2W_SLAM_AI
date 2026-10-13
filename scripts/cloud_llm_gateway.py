#!/usr/bin/env python3
"""Read-only cloud LLM gateway for GO2W planner context.

This server is safe to expose through a temporary tunnel: it has no navigation,
relocation, pause, or execution endpoint. It only returns planner context and
validates LLM hop proposals into a non-executed plan.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


DEFAULT_REPO = os.environ.get("GO2W_REPO", "/home/unitree/Go2W_SLAM_AI")
DEFAULT_REGISTRY = "configs/maps/go2w_multi_map_registry_v2.json"
_FALLBACK_MAP_ID = "map_701"


def _load_active_pcd(repo_root: str | None = None) -> dict[str, Any] | None:
    """Load active PCD info from artifacts/active_pcd.json. Returns None if unavailable."""
    if repo_root is None:
        repo_root = DEFAULT_REPO
    path = Path(repo_root) / "artifacts" / "active_pcd.json"
    try:
        with open(path, "r", encoding="utf-8") as f:
            data: dict[str, Any] = json.load(f)
            return data if isinstance(data, dict) else None
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def _resolve_default_map_id(repo_root: str | None = None) -> str:
    active = _load_active_pcd(repo_root)
    if active and active.get("active_map_id"):
        return str(active["active_map_id"])
    return _FALLBACK_MAP_ID


DEFAULT_MAP_ID = _resolve_default_map_id()


def _repo_path(path: str, repo: str) -> str:
    if not path:
        return path
    p = Path(path)
    if p.is_absolute():
        return str(p)
    return str(Path(repo) / p)


def _load_gateway_module(repo: str):
    scripts_dir = str(Path(repo) / "scripts")
    src_dir = str(Path(repo) / "src")
    for item in (scripts_dir, src_dir):
        if item not in sys.path:
            sys.path.insert(0, item)
    import gateway_server  # type: ignore

    return gateway_server


class CloudLlmHandler(BaseHTTPRequestHandler):
    server_version = "GO2WCloudLLM/1.0"

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _params(self) -> dict[str, str]:
        parsed = urlparse(self.path)
        return {k: v[0] for k, v in parse_qs(parsed.query).items()}

    def _authorized(self) -> bool:
        token = self.server.auth_token  # type: ignore[attr-defined]
        if not token:
            return True
        header = self.headers.get("Authorization", "")
        if header == f"Bearer {token}":
            return True
        if self.headers.get("X-GO2W-Token") == token:
            return True
        return self._params().get("token") == token

    def _reject_if_unauthorized(self) -> bool:
        if self._authorized():
            return False
        self._send_json({"error": "unauthorized"}, 401)
        return True

    def _read_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if self._reject_if_unauthorized():
            return

        if parsed.path == "/health":
            repo = self.server.repo  # type: ignore[attr-defined]
            cloud_key_file = Path(repo) / "artifacts" / "cloud_llm_api.key"
            cloud_configured = bool(
                os.environ.get("GO2W_CLOUD_LLM_API_KEY")
                or os.environ.get("DEEPSEEK_API_KEY")
                or os.environ.get("OPENAI_API_KEY")
                or cloud_key_file.exists()
            )
            local_ask_script = Path("/home/unitree/llm_runtime/scripts/ask_qwen.sh")
            active_pcd_data: dict[str, Any] = {}
            active = _load_active_pcd(repo)
            if active:
                active_pcd_data = {
                    "active_map_id": active.get("active_map_id"),
                    "pcd_path": active.get("pcd_path"),
                    "switched_at": active.get("switched_at"),
                    "anchor_id": active.get("anchor_id"),
                }
            self._send_json(
                {
                    "ok": True,
                    "service": "go2w_cloud_llm_gateway",
                    "readonly": True,
                    "execution_enabled": False,
                    "providers": {
                        "cloud": {"configured": cloud_configured},
                        "local": {"configured": local_ask_script.exists()},
                    },
                    "endpoints": [
                        "GET /health",
                        "GET /planner/context",
                        "POST /planner/validate_hops",
                        "POST /agent/message",
                    ],
                    "default_map_id": self.server.default_map_id,  # type: ignore[attr-defined]
                    "active_pcd": active_pcd_data,
                }
            )
            return

        if parsed.path == "/planner/context":
            params = self._params()
            repo = self.server.repo  # type: ignore[attr-defined]
            registry = _repo_path(
                params.get("registry", self.server.default_registry),  # type: ignore[attr-defined]
                repo,
            )
            map_id = params.get("map_id", self.server.default_map_id)  # type: ignore[attr-defined]
            gateway_server = _load_gateway_module(repo)
            try:
                context = gateway_server.get_planner_context(
                    user_command=params.get("cmd", ""),
                    map_id=map_id,
                    registry_path=registry,
                    map_path=params.get("map_path", ""),
                )
            except Exception as exc:
                import traceback

                context = {
                    "error": str(exc),
                    "error_kind": type(exc).__name__,
                    "traceback": traceback.format_exc(limit=6),
                }
            # Inject active PCD information
            active_pcd = _load_active_pcd(repo)
            candidate_maps: list[dict[str, Any]] = []
            if active_pcd:
                context["active_pcd"] = {
                    "active_map_id": active_pcd.get("active_map_id", map_id),
                    "pcd_path": active_pcd.get("pcd_path", ""),
                    "switched_at": active_pcd.get("switched_at"),
                    "anchor_id": active_pcd.get("anchor_id"),
                }
                for m in active_pcd.get("available_maps", []) or []:
                    candidate_maps.append(m)
            else:
                context["active_pcd"] = {
                    "active_map_id": map_id,
                    "pcd_path": "",
                    "switched_at": None,
                    "anchor_id": None,
                }
            context["active_pcd"]["candidate_maps"] = candidate_maps
            context["cloud_gateway"] = {
                "readonly": True,
                "execution_enabled": False,
                "validated_endpoint": "/planner/validate_hops",
            }
            self._send_json(context)
            return

        self._send_json({"error": "not_found"}, 404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if self._reject_if_unauthorized():
            return

        if parsed.path == "/agent/message":
            try:
                body = self._read_body()
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                self._send_json({"ok": False, "reason": f"invalid_json: {exc}"}, 400)
                return

            message = str(body.get("message") or "")
            if not message:
                self._send_json({"ok": False, "reason": "message is required"}, 400)
                return

            repo = self.server.repo  # type: ignore[attr-defined]
            registry = _repo_path(
                str(body.get("registry") or self.server.default_registry),  # type: ignore[attr-defined]
                repo,
            )
            map_id = str(body.get("map_id") or self.server.default_map_id)  # type: ignore[attr-defined]
            provider = str(body.get("provider") or "cloud")
            current_node = str(body.get("current_node") or "")
            src_dir = str(Path(repo) / "src")
            if src_dir not in sys.path:
                sys.path.insert(0, src_dir)
            from edge_autonomy.mission_agent import handle_agent_message

            result = handle_agent_message(
                message,
                provider=provider,
                registry_path=registry,
                map_id=map_id,
                current_node=current_node,
                cloud_base_url=str(body.get("cloud_base_url") or ""),
                cloud_model=str(body.get("cloud_model") or ""),
                local_ask_script=str(
                    body.get("local_ask_script")
                    or "/home/unitree/llm_runtime/scripts/ask_qwen.sh"
                ),
                local_model=str(
                    body.get("local_model")
                    or "/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf"
                ),
                timeout_s=int(body.get("timeout_s") or 45),
                max_tokens=int(body.get("max_tokens") or 512),
            )
            result["readonly"] = True
            result["execution_enabled"] = False
            self._send_json(result)
            return

        if parsed.path != "/planner/validate_hops":
            self._send_json({"error": "not_found"}, 404)
            return

        try:
            body = self._read_body()
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._send_json({"accepted": False, "reason": f"invalid_json: {exc}"}, 400)
            return

        hops = body.get("hops")
        if not isinstance(hops, list) or not all(isinstance(v, str) for v in hops):
            self._send_json(
                {"accepted": False, "reason": "hops must be a JSON string list"},
                400,
            )
            return

        repo = self.server.repo  # type: ignore[attr-defined]
        registry = _repo_path(
            str(body.get("registry") or self.server.default_registry),  # type: ignore[attr-defined]
            repo,
        )
        map_id = str(body.get("map_id") or self.server.default_map_id)  # type: ignore[attr-defined]
        reason = str(body.get("reason") or "")

        src_dir = str(Path(repo) / "src")
        if src_dir not in sys.path:
            sys.path.insert(0, src_dir)
        from edge_autonomy.cloud_llm_planner import (
            build_waypoint_connectivity,
            hops_to_plan,
        )

        connectivity = build_waypoint_connectivity(
            registry,
            map_id=map_id,
            force_refresh=True,
        )
        plan = hops_to_plan(hops, map_id, reason=reason, connectivity=connectivity)
        accepted = not (
            plan.get("mode") == "human_confirm" and float(plan.get("confidence", 0.0)) < 0.5
        )
        self._send_json(
            {
                "accepted": accepted,
                "readonly": True,
                "execution_enabled": False,
                "hops": hops,
                "connectivity": connectivity,
                "plan": plan,
            }
        )

    def log_message(self, fmt: str, *args: Any) -> None:
        return


def make_server(args: argparse.Namespace) -> HTTPServer:
    server = HTTPServer((args.host, args.port), CloudLlmHandler)
    server.repo = str(Path(args.repo).resolve())  # type: ignore[attr-defined]
    server.default_registry = args.registry  # type: ignore[attr-defined]
    server.default_map_id = args.map_id  # type: ignore[attr-defined]
    server.auth_token = args.token or os.environ.get("GO2W_CLOUD_LLM_TOKEN", "")  # type: ignore[attr-defined]
    return server


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--repo", default=DEFAULT_REPO)
    parser.add_argument("--registry", default=DEFAULT_REGISTRY)
    parser.add_argument("--map-id", default=_resolve_default_map_id())
    parser.add_argument(
        "--token",
        default="",
        help="Prefer GO2W_CLOUD_LLM_TOKEN to avoid exposing secrets in process args.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    server = make_server(args)
    print(
        f"GO2W Cloud LLM gateway on http://{args.host}:{args.port} "
        f"readonly=true token_required={bool(server.auth_token)}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
