#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thin browser UI for the GO2W C++ operator panel.

This server deliberately delegates robot-facing work to the existing C++
operator panel. It keeps only UI/session state here: weak-link display mode,
execute toggle, current-node hint, and a short command history.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse


DEFAULT_PORT = 8765


def truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on", "y"}


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def trim_line(value: str) -> str:
    return value.strip(" \t\r\n")


def parse_panel_summary(text: str) -> Dict[str, str]:
    """Extract the compact `key=value | key=value` line from panel output."""
    summary: Dict[str, str] = {}
    for raw_line in reversed(text.splitlines()):
        line = raw_line.strip()
        if "phase=" not in line and "loc=" not in line and "SLAM=" not in line:
            continue
        for part in line.split("|"):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            key = key.strip()
            if key.startswith("[") and "]" in key:
                key = key.split("]", 1)[1].strip()
            key = key.strip("[]")
            value = value.strip()
            if key:
                summary[key] = value
        if summary:
            break
    return summary


def strip_ansi(text: str) -> str:
    out: List[str] = []
    i = 0
    while i < len(text):
        if text[i] == "\x1b" and i + 1 < len(text) and text[i + 1] == "[":
            i += 2
            while i < len(text) and not ("@" <= text[i] <= "~"):
                i += 1
            i += 1
            continue
        out.append(text[i])
        i += 1
    return "".join(out)


@dataclass
class WebConfig:
    repo_root: Path
    panel_bin: Path
    gateway_client: str
    start_slam_script: str
    start_rviz2_script: str
    network_interface: str = "eth0"
    current_node: str = "initial_point"
    host: str = "127.0.0.1"
    port: int = DEFAULT_PORT
    gateway_timeout_s: int = 30
    gateway_startup_wait_s: float = 1.0
    panel_timeout_s: int = 90
    llm_http_url: str = ""
    llm_http_model: str = "local"
    stereo_summary_path: Path = Path("artifacts/stereo_depth_summary.json")
    stereo_stale_ms: int = 5000
    status_cache_ms: int = 1500
    ensure_slam_on_start: bool = False

    def panel_argv(self, execute_enabled: bool, weak_link_mode: bool, current_node: str) -> List[str]:
        argv = [
            str(self.panel_bin),
            "--repo-root",
            str(self.repo_root),
            "--gateway-client",
            self.gateway_client,
            "--start-slam-script",
            self.start_slam_script,
            "--start-rviz2-script",
            self.start_rviz2_script,
            "--interface",
            self.network_interface,
            "--gateway-timeout-s",
            str(self.gateway_timeout_s),
            "--gateway-startup-wait-s",
            str(self.gateway_startup_wait_s),
            "--current-node",
            current_node,
        ]
        if self.llm_http_url:
            argv.extend(["--llm-http-url", self.llm_http_url])
            argv.extend(["--llm-http-model", self.llm_http_model])
        if execute_enabled:
            argv.append("--execute")
        if weak_link_mode:
            argv.append("--weak")
        return argv


@dataclass
class WebState:
    execute_enabled: bool = False
    weak_link_mode: bool = False
    current_node: str = "initial_point"
    history: List[Dict[str, Any]] = field(default_factory=list)

    def snapshot(self) -> Dict[str, Any]:
        return {
            "execute_enabled": self.execute_enabled,
            "weak_link_mode": self.weak_link_mode,
            "current_node": self.current_node,
            "history": list(self.history[-30:]),
        }

    def apply_local_setting(self, line: str, confirmed: bool = False) -> Optional[Dict[str, Any]]:
        line = trim_line(line)
        if line == "/execute on":
            if not confirmed:
                return {
                    "handled": True,
                    "accepted": False,
                    "exit_code": 2,
                    "stdout": "execute on requires browser confirmation.\n",
                    "stderr": "",
                }
            self.execute_enabled = True
            return self._local_result("真实执行: on\n")
        if line == "/execute off":
            self.execute_enabled = False
            return self._local_result("真实执行: off\n")
        if line == "/weak on":
            self.weak_link_mode = True
            return self._local_result("弱网摘要显示: on\n")
        if line == "/weak off":
            self.weak_link_mode = False
            return self._local_result("弱网摘要显示: off\n")
        if line.startswith("/current "):
            node = trim_line(line[len("/current "):])
            if not node:
                return {
                    "handled": True,
                    "accepted": False,
                    "exit_code": 2,
                    "stdout": "current node is empty.\n",
                    "stderr": "",
                }
            self.current_node = node
            return self._local_result(f"当前位置锚点: {node}\n")
        return None

    @staticmethod
    def _local_result(stdout: str) -> Dict[str, Any]:
        return {
            "handled": True,
            "accepted": True,
            "exit_code": 0,
            "stdout": stdout,
            "stderr": "",
        }

    def remember(self, line: str, result: Dict[str, Any]) -> None:
        self.history.append({
            "ts": int(time.time() * 1000),
            "line": line,
            "exit_code": result.get("exit_code"),
            "accepted": result.get("accepted", result.get("exit_code") == 0),
            "summary": result.get("summary", {}),
        })
        if len(self.history) > 80:
            del self.history[:-80]


class OperatorWebApp:
    def __init__(self, config: WebConfig):
        self.config = config
        self.state = WebState(current_node=config.current_node)
        self.lock = threading.Lock()
        self.panel_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self._status_cache: Optional[Dict[str, Any]] = None
        self._status_cache_ts_ms = 0

    def run_panel_session(self, lines: Iterable[str]) -> Dict[str, Any]:
        input_text = "\n".join(lines) + "\n"
        with self.lock:
            argv = self.config.panel_argv(
                execute_enabled=self.state.execute_enabled,
                weak_link_mode=self.state.weak_link_mode,
                current_node=self.state.current_node,
            )

        if not self.config.panel_bin.exists():
            return {
                "accepted": False,
                "exit_code": 127,
                "stdout": "",
                "stderr": f"operator panel binary not found: {self.config.panel_bin}\n",
                "summary": {},
            }

        try:
            with self.panel_lock:
                completed = subprocess.run(
                    argv,
                    input=input_text,
                    cwd=str(self.config.repo_root),
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    capture_output=True,
                    timeout=self.config.panel_timeout_s,
                    check=False,
                )
            stdout = strip_ansi(completed.stdout)
            stderr = strip_ansi(completed.stderr)
            return {
                "accepted": completed.returncode == 0,
                "exit_code": completed.returncode,
                "stdout": stdout,
                "stderr": stderr,
                "summary": parse_panel_summary(stdout),
            }
        except subprocess.TimeoutExpired as exc:
            stdout = strip_ansi(exc.stdout or "")
            stderr = strip_ansi(exc.stderr or "")
            return {
                "accepted": False,
                "exit_code": 124,
                "stdout": stdout,
                "stderr": stderr + "operator panel command timed out\n",
                "summary": parse_panel_summary(stdout),
            }

    def status(self, *, force: bool = False) -> Dict[str, Any]:
        now = int(time.time() * 1000)
        with self.status_lock:
            if not force and self._status_cache is not None:
                age_ms = max(0, now - self._status_cache_ts_ms)
                if age_ms <= self.config.status_cache_ms:
                    cached = dict(self._status_cache)
                    cached["cache"] = {
                        "hit": True,
                        "age_ms": age_ms,
                        "ttl_ms": self.config.status_cache_ms,
                    }
                    return cached

            result = self._fresh_status()
            self._status_cache_ts_ms = int(time.time() * 1000)
            result["cache"] = {
                "hit": False,
                "age_ms": 0,
                "ttl_ms": self.config.status_cache_ms,
            }
            self._status_cache = dict(result)
            return result

    def _fresh_status(self) -> Dict[str, Any]:
        result = self.run_panel_session(["/quit"])
        result["stereo_summary"] = self.stereo_summary()
        with self.lock:
            result["state"] = self.state.snapshot()
        return result

    def invalidate_status_cache(self) -> None:
        with self.status_lock:
            self._status_cache = None
            self._status_cache_ts_ms = 0

    def command(self, line: str, confirmed: bool = False) -> Dict[str, Any]:
        line = trim_line(line)
        if not line:
            return self.status()

        local: Optional[Dict[str, Any]] = None
        with self.lock:
            local = self.state.apply_local_setting(line, confirmed=confirmed)
            if local is not None:
                local["summary"] = {}
                local["stereo_summary"] = self.stereo_summary()
                local["state"] = self.state.snapshot()
                self.state.remember(line, local)
        if local is not None:
            self.invalidate_status_cache()
            return local

        result = self.run_panel_session([line, "/quit"])
        result["stereo_summary"] = self.stereo_summary()
        with self.lock:
            result["state"] = self.state.snapshot()
            self.state.remember(line, result)
        self.invalidate_status_cache()
        return result

    def state_snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return self.state.snapshot()

    def stereo_summary(self) -> Dict[str, Any]:
        path = self.config.stereo_summary_path
        if not path.is_absolute():
            path = self.config.repo_root / path
        if not path.exists():
            return {"available": False, "path": str(path)}
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            ts = int(data.get("timestamp_ms") or 0)
            age_ms = max(0, int(time.time() * 1000) - ts) if ts else None
            return {
                "available": True,
                "path": str(path),
                "age_ms": age_ms,
                "stale_ms": self.config.stereo_stale_ms,
                "stale_by_age": bool(age_ms is not None and age_ms > self.config.stereo_stale_ms),
                "data": data,
            }
        except Exception as exc:  # noqa: BLE001
            return {"available": False, "path": str(path), "error": str(exc)}


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>GO2W Operator Panel</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7f8;
      --surface: #ffffff;
      --surface-2: #eef2f4;
      --ink: #172026;
      --muted: #63717a;
      --line: #d8e0e4;
      --accent: #0f766e;
      --accent-2: #2563eb;
      --danger: #b42318;
      --warn: #a15c07;
      --ok: #147a3c;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--ink);
      font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 18px;
      border-bottom: 1px solid var(--line);
      background: var(--surface);
      position: sticky;
      top: 0;
      z-index: 2;
    }
    h1 { margin: 0; font-size: 18px; font-weight: 700; letter-spacing: 0; }
    main {
      display: grid;
      grid-template-columns: minmax(280px, 360px) minmax(420px, 1fr);
      gap: 14px;
      padding: 14px;
      max-width: 1440px;
      margin: 0 auto;
    }
    section {
      background: var(--surface);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 14px;
    }
    h2 { margin: 0 0 10px; font-size: 15px; letter-spacing: 0; }
    .stack { display: grid; gap: 12px; align-content: start; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .metric {
      min-height: 62px;
      background: var(--surface-2);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
    }
    .metric label { display: block; color: var(--muted); font-size: 12px; margin-bottom: 5px; }
    .metric strong { display: block; font-size: 14px; overflow-wrap: anywhere; }
    .toolbar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    button, input, textarea, select {
      font: inherit;
      border-radius: 6px;
      border: 1px solid var(--line);
      background: #fff;
      color: var(--ink);
    }
    button {
      min-height: 34px;
      padding: 7px 10px;
      cursor: pointer;
      font-weight: 600;
    }
    button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
    button.secondary { background: var(--accent-2); border-color: var(--accent-2); color: #fff; }
    button.danger { background: #fff4f2; border-color: #f0b8b0; color: var(--danger); }
    button:disabled { opacity: .55; cursor: wait; }
    input, select { min-height: 34px; padding: 7px 9px; width: 100%; }
    textarea { min-height: 88px; padding: 9px; width: 100%; resize: vertical; }
    .row { display: grid; gap: 8px; grid-template-columns: 1fr auto; align-items: center; }
    .statusbar { color: var(--muted); font-size: 13px; }
    .statusbar b { color: var(--ink); }
    .pill {
      display: inline-flex;
      align-items: center;
      min-height: 26px;
      padding: 3px 8px;
      border: 1px solid var(--line);
      border-radius: 999px;
      background: var(--surface-2);
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }
    .pill.ok { color: var(--ok); border-color: #9bd3ad; background: #effaf2; }
    .pill.warn { color: var(--warn); border-color: #e2c38b; background: #fff8e8; }
    .pill.danger { color: var(--danger); border-color: #f0b8b0; background: #fff4f2; }
    pre {
      margin: 0;
      min-height: 280px;
      max-height: 52vh;
      overflow: auto;
      padding: 12px;
      background: #101820;
      color: #dbe7ef;
      border-radius: 6px;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .history { display: grid; gap: 6px; max-height: 240px; overflow: auto; }
    .history div {
      border: 1px solid var(--line);
      background: var(--surface-2);
      border-radius: 6px;
      padding: 7px 8px;
      overflow-wrap: anywhere;
    }
    .split { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .split { grid-template-columns: 1fr; }
    }
    @media (max-width: 560px) {
      header { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
      .row { grid-template-columns: 1fr; }
      button { width: 100%; }
    }
  </style>
</head>
<body>
  <header>
    <h1>GO2W Operator Panel</h1>
    <div class="toolbar">
      <span id="conn" class="pill warn">未连接</span>
      <span id="exec-pill" class="pill">dry-run</span>
      <span id="weak-pill" class="pill">normal</span>
      <button id="refresh" class="primary">刷新状态</button>
    </div>
  </header>
  <main>
    <div class="stack">
      <section>
        <h2>世界状态</h2>
        <div class="grid">
          <div class="metric"><label>任务阶段</label><strong id="m-phase">unknown</strong></div>
          <div class="metric"><label>当前目标</label><strong id="m-target">none</strong></div>
          <div class="metric"><label>定位</label><strong id="m-loc">unknown</strong></div>
          <div class="metric"><label>地图</label><strong id="m-map">unknown</strong></div>
          <div class="metric"><label>运动许可</label><strong id="m-motion">false</strong></div>
          <div class="metric"><label>障碍状态</label><strong id="m-obstacle">unknown</strong></div>
          <div class="metric"><label>网络模式</label><strong id="m-net">normal</strong></div>
          <div class="metric"><label>安全原因</label><strong id="m-safety">-</strong></div>
          <div class="metric"><label>双目前方/中心</label><strong id="m-stereo-front">unavailable</strong></div>
          <div class="metric"><label>双目置信/年龄</label><strong id="m-stereo-health">unavailable</strong></div>
        </div>
      </section>

      <section>
        <h2>运行开关</h2>
        <div class="toolbar">
          <button id="start-slam" class="secondary">启动/检查 SLAM</button>
          <button id="exec-on" class="danger">允许真实执行</button>
          <button id="exec-off">切回 dry-run</button>
          <button id="weak-on">弱网摘要</button>
          <button id="weak-off">完整显示</button>
        </div>
        <div class="row" style="margin-top:10px">
          <input id="current-node" placeholder="current node，例如 initial_point">
          <button id="set-current">设置锚点</button>
        </div>
      </section>

      <section>
        <h2>建图与拓扑</h2>
        <div class="row">
          <input id="map-path" value="/home/unitree/test.pcd">
          <button id="mapping-end">结束建图</button>
        </div>
        <div class="toolbar" style="margin-top:8px">
          <button id="mapping-start" class="danger">开启建图</button>
          <button id="rviz-start">打开 RViz2</button>
        </div>
        <div class="row" style="margin-top:10px">
          <input id="topology-name" placeholder="拓扑点 ID，例如 wp_station_01">
          <button id="topology-preview">预览</button>
        </div>
        <div class="toolbar" style="margin-top:8px">
          <button id="topology-add" class="danger">写入拓扑点</button>
        </div>
      </section>
    </div>

    <div class="stack">
      <section>
        <h2>LLM 输入口</h2>
        <textarea id="llm-input" placeholder="例如：去赵博办公室门口拍照，然后回尹思园工位"></textarea>
        <div class="toolbar" style="margin-top:8px">
          <button id="send-llm" class="primary">发送指令</button>
          <select id="refresh-interval" style="width:150px">
            <option value="0">暂停自动刷新</option>
            <option value="1000">1 秒刷新</option>
            <option value="2000" selected>2 秒刷新</option>
            <option value="5000">5 秒刷新</option>
          </select>
        </div>
      </section>

      <section>
        <h2>反馈显示屏</h2>
        <pre id="output">等待状态刷新...</pre>
      </section>

      <section>
        <h2>任务历史</h2>
        <div id="history" class="history"></div>
      </section>
    </div>
  </main>

  <script>
    const $ = (id) => document.getElementById(id);
    let appState = { execute_enabled: false, weak_link_mode: false, current_node: "initial_point", history: [] };
    let timer = null;

    function setBusy(busy) {
      document.querySelectorAll("button, input, textarea, select").forEach((el) => { el.disabled = busy; });
    }

    function escapeHtml(value) {
      return String(value ?? "").replace(/[&<>"']/g, (ch) => ({ "&":"&amp;", "<":"&lt;", ">":"&gt;", '"':"&quot;", "'":"&#39;" }[ch]));
    }

    function parseSummaryText(text) {
      const lines = String(text || "").split(/\r?\n/).reverse();
      for (const line of lines) {
        if (!line.includes("phase=") && !line.includes("loc=")) continue;
        const out = {};
        for (const part of line.split("|")) {
          const idx = part.indexOf("=");
          if (idx < 0) continue;
          out[part.slice(0, idx).trim().replace(/^\[[^\]]+\]\s*/, "")] = part.slice(idx + 1).trim();
        }
        return out;
      }
      return {};
    }

    function updateMetrics(summary) {
      const s = summary || {};
      $("m-phase").textContent = s.phase || "unknown";
      $("m-target").textContent = s.target || "";
      $("m-loc").textContent = s.loc || "unknown";
      $("m-map").textContent = s.map || "unknown";
      $("m-motion").textContent = s.motion || "false";
      $("m-obstacle").textContent = s.obstacle || "unknown";
      $("m-net").textContent = s.net || "normal";
      $("m-safety").textContent = s.safety || "-";
    }

    function updateState(state) {
      if (!state) return;
      appState = state;
      $("exec-pill").textContent = state.execute_enabled ? "EXEC enabled" : "dry-run";
      $("exec-pill").className = state.execute_enabled ? "pill danger" : "pill ok";
      $("weak-pill").textContent = state.weak_link_mode ? "weak" : "normal";
      $("weak-pill").className = state.weak_link_mode ? "pill warn" : "pill";
      $("current-node").value = state.current_node || "";
      const items = (state.history || []).slice().reverse().map((item) => {
        const code = item.exit_code === 0 ? "ok" : "err";
        return `<div><b>${escapeHtml(code)}</b> ${escapeHtml(item.line)}<br><small>${escapeHtml(JSON.stringify(item.summary || {}))}</small></div>`;
      });
      $("history").innerHTML = items.join("") || "<div>暂无任务历史</div>";
    }

    function updateStereo(stereo) {
      if (!stereo || !stereo.available || !stereo.data) {
        $("m-stereo-front").textContent = "unavailable";
        $("m-stereo-health").textContent = "unavailable";
        return;
      }
      const data = stereo.data;
      const front = data.front_clearance_m;
      const center = data.center_distance_m ?? data.center_window_m;
      const frontText = (front === null || front === undefined) ? "front ?" : `front ${Number(front).toFixed(2)}m`;
      const centerText = (center === null || center === undefined) ? "center ?" : `center ${Number(center).toFixed(2)}m`;
      $("m-stereo-front").textContent = `${frontText} / ${centerText}`;
      const confidence = data.confidence === null || data.confidence === undefined ? "whole ?" : `whole ${Number(data.confidence).toFixed(3)}`;
      const frontConfidence = data.roi_confidence && data.roi_confidence.front !== undefined ? `front ${Number(data.roi_confidence.front).toFixed(3)}` : "front ?";
      const age = stereo.age_ms === null || stereo.age_ms === undefined ? "unknown" : `${Math.round(stereo.age_ms)}ms`;
      $("m-stereo-health").textContent = `${confidence}, ${frontConfidence} / ${age}${stereo.stale_by_age ? " stale" : ""}`;
    }

    async function api(path, options = {}) {
      const res = await fetch(path, { headers: { "content-type": "application/json" }, ...options });
      if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
      return res.json();
    }

    async function refreshStatus(force = false) {
      setBusy(true);
      try {
        const result = await api(force ? "/api/status?force=1" : "/api/status");
        $("conn").textContent = result.exit_code === 0 ? "已连接" : "状态异常";
        $("conn").className = result.exit_code === 0 ? "pill ok" : "pill danger";
        const summary = result.summary && Object.keys(result.summary).length ? result.summary : parseSummaryText(result.stdout);
        updateMetrics(summary);
        updateState(result.state);
        updateStereo(result.stereo_summary);
        $("output").textContent = (result.stdout || "") + (result.stderr ? "\n[stderr]\n" + result.stderr : "");
      } catch (err) {
        $("conn").textContent = "UI 服务异常";
        $("conn").className = "pill danger";
        $("output").textContent = String(err);
      } finally {
        setBusy(false);
      }
    }

    async function runCommand(line, confirmText = "", confirmed = false) {
      if (confirmText && !window.confirm(confirmText)) return;
      setBusy(true);
      try {
        const result = await api("/api/command", {
          method: "POST",
          body: JSON.stringify({ line, confirm: confirmed || Boolean(confirmText) })
        });
        const summary = result.summary && Object.keys(result.summary).length ? result.summary : parseSummaryText(result.stdout);
        updateMetrics(summary);
        updateState(result.state);
        updateStereo(result.stereo_summary);
        $("output").textContent = `$ ${line}\n` + (result.stdout || "") + (result.stderr ? "\n[stderr]\n" + result.stderr : "");
      } catch (err) {
        $("output").textContent = String(err);
      } finally {
        setBusy(false);
      }
    }

    function resetTimer() {
      if (timer) clearInterval(timer);
      const ms = Number($("refresh-interval").value || 0);
      if (ms > 0) timer = setInterval(refreshStatus, ms);
    }

    function installRelocateControl() {
      const current = $("current-node");
      if (!current || $("relocate-anchor")) return;
      const row = document.createElement("div");
      row.className = "row";
      row.style.marginTop = "10px";
      row.innerHTML = '<input id="relocate-anchor" value="initial_point" placeholder="relocalization anchor id"><button id="relocate-anchor-btn">Relocalize</button>';
      current.parentElement.insertAdjacentElement("afterend", row);
      $("relocate-anchor-btn").onclick = () => {
        const anchor = $("relocate-anchor").value.trim() || "initial_point";
        runCommand(`/relocate ${anchor} confirm`, "Confirm SLAM relocalization against this registry anchor? This does not move the chassis.", true);
      };
    }

    $("refresh").onclick = () => refreshStatus(true);
    $("start-slam").onclick = () => runCommand("/start-slam", "确认启动/检查 SLAM 与雷达 driver？", true);
    $("exec-on").onclick = () => runCommand("/execute on", "确认允许真实执行？机器狗趴着时不要开启。", true);
    $("exec-off").onclick = () => runCommand("/execute off");
    $("weak-on").onclick = () => runCommand("/weak on");
    $("weak-off").onclick = () => runCommand("/weak off");
    $("set-current").onclick = () => runCommand(`/current ${$("current-node").value.trim()}`);
    $("mapping-start").onclick = () => runCommand("/mapping start confirm", "确认开启建图？这会改变 SLAM 状态。", true);
    $("mapping-end").onclick = () => runCommand(`/mapping end confirm ${$("map-path").value.trim()}`, "确认结束建图并保存地图？", true);
    $("rviz-start").onclick = () => runCommand("/rviz2 start confirm", "确认打开 RViz2 可视化？", true);
    $("topology-preview").onclick = () => runCommand(`/topology preview ${$("topology-name").value.trim()}`);
    $("topology-add").onclick = () => runCommand(`/topology add ${$("topology-name").value.trim()} confirm`, "确认写入当前位置为拓扑点？", true);
    $("send-llm").onclick = () => {
      const line = $("llm-input").value.trim();
      if (!line) return;
      const confirmText = appState.execute_enabled ? "当前是真实执行模式，确认发送给机器人？" : "";
      runCommand(line, confirmText, Boolean(confirmText));
    };
    $("refresh-interval").onchange = resetTimer;

    installRelocateControl();
    refreshStatus();
    resetTimer();
  </script>
</body>
</html>
"""


class OperatorRequestHandler(BaseHTTPRequestHandler):
    server_version = "GO2WOperatorWeb/0.1"

    def do_GET(self) -> None:
        parsed_path = urlparse(self.path)
        if parsed_path.path == "/":
            self.send_html(INDEX_HTML)
            return
        if parsed_path.path == "/api/status":
            query = parse_qs(parsed_path.query)
            force = truthy(query.get("force", ["0"])[0]) or truthy(query.get("refresh", ["0"])[0])
            self.send_json(self.app.status(force=force))
            return
        if parsed_path.path == "/api/state":
            self.send_json({"state": self.app.state_snapshot()})
            return
        if parsed_path.path == "/api/stereo-summary":
            self.send_json({"stereo_summary": self.app.stereo_summary()})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path != "/api/command":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            payload = self.read_json()
            line = str(payload.get("line", ""))
            confirmed = bool(payload.get("confirm", False))
        except Exception as exc:  # noqa: BLE001 - report bad request to browser.
            self.send_json({"accepted": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        self.send_json(self.app.command(line, confirmed=confirmed))

    @property
    def app(self) -> OperatorWebApp:
        return self.server.app  # type: ignore[attr-defined]

    def read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("content-length", "0"))
        body = self.rfile.read(length).decode("utf-8", errors="replace")
        return json.loads(body or "{}")

    def send_html(self, text: str) -> None:
        data = text.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("content-type", "text/html; charset=utf-8")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json; charset=utf-8")
        self.send_header("cache-control", "no-store")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


class OperatorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address: Tuple[str, int], handler_cls: type[BaseHTTPRequestHandler], app: OperatorWebApp):
        super().__init__(server_address, handler_cls)
        self.app = app


def make_config(argv: Optional[List[str]] = None) -> WebConfig:
    repo_default = repo_root_from_script()
    env = os.environ
    parser = argparse.ArgumentParser(description="GO2W thin browser operator UI")
    parser.add_argument("--repo-root", default=env.get("GO2W_REPO_ROOT", str(repo_default)))
    parser.add_argument("--panel-bin", default=env.get("GO2W_OPERATOR_PANEL_BIN", ""))
    parser.add_argument("--gateway-client", default=env.get("GO2W_GATEWAY_CLIENT", "/home/unitree/slam_gateway_refactor/build/slam_llm_command_client"))
    parser.add_argument("--start-slam-script", default=env.get("GO2W_START_SLAM_SCRIPT", ""))
    parser.add_argument("--start-rviz2-script", default=env.get("GO2W_START_RVIZ2_SCRIPT", ""))
    parser.add_argument("--interface", default=env.get("GO2W_NETWORK_INTERFACE", "eth0"))
    parser.add_argument("--current-node", default=env.get("GO2W_CURRENT_NODE", "initial_point"))
    parser.add_argument("--host", default=env.get("GO2W_WEB_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(env.get("GO2W_WEB_PORT", str(DEFAULT_PORT))))
    parser.add_argument("--gateway-timeout-s", type=int, default=int(env.get("GO2W_GATEWAY_TIMEOUT_S", "30")))
    parser.add_argument("--gateway-startup-wait-s", type=float, default=float(env.get("GO2W_GATEWAY_STARTUP_WAIT_S", "1.0")))
    parser.add_argument("--panel-timeout-s", type=int, default=int(env.get("GO2W_PANEL_TIMEOUT_S", "90")))
    parser.add_argument("--llm-http-url", default=env.get("GO2W_LLM_HTTP_URL", ""))
    parser.add_argument("--llm-http-model", default=env.get("GO2W_LLM_HTTP_MODEL", "local"))
    parser.add_argument("--stereo-summary-path", default=env.get("GO2W_STEREO_SUMMARY_PATH", "artifacts/stereo_depth_summary.json"))
    parser.add_argument("--stereo-stale-ms", type=int, default=int(env.get("GO2W_STEREO_STALE_MS", "5000")))
    parser.add_argument("--status-cache-ms", type=int, default=int(env.get("GO2W_STATUS_CACHE_MS", "1500")))
    parser.add_argument("--ensure-slam-on-start", action="store_true", default=truthy(env.get("GO2W_WEB_ENSURE_SLAM_ON_START", "0")))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)

    repo_root = Path(args.repo_root).expanduser().resolve()
    panel_bin = Path(args.panel_bin).expanduser() if args.panel_bin else repo_root / "cpp" / "build" / "go2w_operator_panel"
    start_slam_script = args.start_slam_script or str(repo_root / "scripts" / "start_go2w_slam_stack.sh")
    start_rviz2_script = args.start_rviz2_script or str(repo_root / "scripts" / "start_go2w_rviz2.sh")
    return WebConfig(
        repo_root=repo_root,
        panel_bin=panel_bin,
        gateway_client=args.gateway_client,
        start_slam_script=start_slam_script,
        start_rviz2_script=start_rviz2_script,
        network_interface=args.interface,
        current_node=args.current_node,
        host=args.host,
        port=args.port,
        gateway_timeout_s=args.gateway_timeout_s,
        gateway_startup_wait_s=args.gateway_startup_wait_s,
        panel_timeout_s=args.panel_timeout_s,
        llm_http_url=args.llm_http_url,
        llm_http_model=args.llm_http_model,
        stereo_summary_path=Path(args.stereo_summary_path).expanduser(),
        stereo_stale_ms=max(1, args.stereo_stale_ms),
        status_cache_ms=max(0, args.status_cache_ms),
        ensure_slam_on_start=args.ensure_slam_on_start,
    )


def run_startup_script(config: WebConfig) -> None:
    script = Path(config.start_slam_script)
    if not script.exists():
        print(f"warning: startup script not found: {script}", file=sys.stderr)
        return
    completed = subprocess.run(
        ["bash", str(script)],
        cwd=str(config.repo_root),
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=max(30, config.panel_timeout_s),
        check=False,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.stderr:
        print(completed.stderr, end="", file=sys.stderr)
    if completed.returncode != 0:
        print(f"warning: startup script exited with {completed.returncode}", file=sys.stderr)


def self_test(config: WebConfig) -> None:
    assert "GO2W Operator Panel" in INDEX_HTML
    assert "/api/status" in INDEX_HTML
    assert "/relocate" in INDEX_HTML
    parsed = parse_panel_summary("phase=idle | target=none | loc=true | map=true | motion=false")
    assert parsed["phase"] == "idle"
    assert parsed["loc"] == "true"
    state = WebState(current_node=config.current_node)
    assert state.apply_local_setting("/execute on", confirmed=False)["exit_code"] == 2
    assert state.apply_local_setting("/execute on", confirmed=True)["exit_code"] == 0
    assert state.execute_enabled is True
    print("go2w_operator_web_self_test=passed")


def main(argv: Optional[List[str]] = None) -> int:
    config = make_config(argv)
    if argv is not None and "--self-test" in argv:
        self_test(config)
        return 0
    if "--self-test" in sys.argv:
        self_test(config)
        return 0

    if config.ensure_slam_on_start:
        run_startup_script(config)

    app = OperatorWebApp(config)
    server = OperatorHTTPServer((config.host, config.port), OperatorRequestHandler, app)
    print(f"GO2W operator web UI: http://{config.host}:{config.port}", flush=True)
    print("default mode: dry-run; execute requires explicit browser confirmation", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
