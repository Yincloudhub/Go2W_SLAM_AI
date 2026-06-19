#!/usr/bin/env python3
"""nav_relocate — reliable PCD relocation with retry and localization verification.

Usage:
    python3 scripts/nav_relocate.py map_701
    python3 scripts/nav_relocate.py map_701 mapping_origin_701
    python3 scripts/nav_relocate.py --list

Integrates with nav_core:
    from nav_relocate import relocate_to_map
    ok = relocate_to_map('map_701')
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
import os
from typing import Optional, Tuple

# ── Paths ──
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_GATEWAY = os.path.join(_REPO, 'robot', 'slam_gateway_refactor', 'build', 'slam_llm_command_client')
_REGISTRY = os.path.join(_REPO, 'configs', 'maps', 'go2w_multi_map_registry_v2.json')
_ACTIVE_PCD_PATH = os.path.join(_REPO, 'artifacts', 'active_pcd.json')
_NETWORK = 'eth0'

# Hardcoded fallback — used when active_pcd.json is missing
_HARDCODED_DEFAULT_PCD = '/home/unitree/maps/staging/map_701.pcd'
_HARDCODED_DEFAULT_ANCHOR = 'mapping_origin_701'

# Map name → PCD path lookup (from registry)
_MAP_INFO = {
    'map_701': {
        'pcd': '/home/unitree/maps/staging/map_701.pcd',
        'anchor': 'mapping_origin_701',
    },
    'map_701_left': {
        'pcd': '/home/unitree/maps/staging/map_701_left.pcd',
        'anchor': 'mapping_origin_701_left',
    },
    'map_terrace_wc': {
        'pcd': '/home/unitree/maps/staging/map_terrace_wc.pcd',
        'anchor': 'mapping_origin_terrace',
    },
}


def _load_registry() -> dict:
    with open(_REGISTRY) as f:
        return json.load(f)


def _get_anchor_pose(map_id: str, anchor_id: str) -> Optional[dict]:
    """Read anchor pose from registry."""
    reg = _load_registry()
    for m in reg.get('maps', []):
        if m.get('map_id') == map_id:
            for a in m.get('relocalization_anchors', []):
                if a.get('anchor_id') == anchor_id:
                    return a.get('pose', {})
    return None


def get_active_pcd():
    # type: () -> dict
    """Read active_pcd.json. Returns dict with active_map_id, pcd_path, switched_at, switched_from, anchor_id.

    Falls back to hardcoded default if the file is missing or invalid.
    This function is safe to call from any script — no side effects, no Gateway dependency.
    """
    try:
        with open(_ACTIVE_PCD_PATH, 'r') as f:
            return json.load(f)
    except Exception:
        return {
            'active_map_id': 'map_701',
            'pcd_path': _HARDCODED_DEFAULT_PCD,
            'switched_at': None,
            'switched_from': None,
            'anchor_id': _HARDCODED_DEFAULT_ANCHOR,
        }


def _write_active_pcd(map_name, pcd_path, anchor_id, previous_map_id=None):
    # type: (str, str, str, Optional[str]) -> None
    """Write active_pcd.json after a successful PCD switch."""
    active = {
        'active_map_id': map_name,
        'pcd_path': pcd_path,
        'switched_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'switched_from': previous_map_id,
        'anchor_id': anchor_id,
    }
    os.makedirs(os.path.dirname(_ACTIVE_PCD_PATH), exist_ok=True)
    with open(_ACTIVE_PCD_PATH, 'w') as f:
        json.dump(active, f, indent=2, ensure_ascii=False)
    print('[active_pcd] written: {} → {}'.format(previous_map_id or '(none)', pcd_path), flush=True)


def _update_registry_pcd_path(pcd_path):
    # type: (str) -> None
    """Update go2w_real_site.pcd_path in the multi-map registry so Gateway picks up the new PCD."""
    try:
        with open(_REGISTRY, 'r') as f:
            reg = json.load(f)
        updated = False
        for m in reg.get('maps', []):
            if m.get('map_id') == 'go2w_real_site':
                m['pcd_path'] = pcd_path
                updated = True
                break
        if updated:
            with open(_REGISTRY, 'w') as f:
                json.dump(reg, f, indent=2, ensure_ascii=False)
            print('[active_pcd] registry updated: go2w_real_site.pcd_path = {}'.format(pcd_path), flush=True)
    except Exception as exc:
        print('[active_pcd] WARNING: could not update registry: {}'.format(exc), flush=True)


def _send_relocate(map_id: str, pcd_path: str, anchor_id: str,
                   timeout: float = 15.0) -> Tuple[bool, str]:
    """Send relocate command to Gateway C++ client.

    Returns (success, reason).
    """
    pose = _get_anchor_pose(map_id, anchor_id)
    if pose is None:
        return False, f'anchor {anchor_id} not found in registry map {map_id}'

    cmd = {
        'action': 'relocate',
        'request_id': f'reloc_{int(time.monotonic() * 1000)}',
        'map_id': 'go2w_real_site',  # Gateway hardcoded
        'map_path': pcd_path,
        'anchor_id': anchor_id,
        'initial_pose': {
            'x': float(pose.get('x', 0)),
            'y': float(pose.get('y', 0)),
            'z': float(pose.get('z', 0)),
            'q_x': float(pose.get('q_x', 0)),
            'q_y': float(pose.get('q_y', 0)),
            'q_z': float(pose.get('q_z', 0)),
            'q_w': float(pose.get('q_w', 1)),
            'yaw': float(pose.get('yaw', 0)),
            'name': anchor_id,
            'mode': 0,
            'speed': 0.0,
        },
        'operator_ack': True,
    }

    try:
        r = subprocess.run(
            [_GATEWAY, _NETWORK],
            input=json.dumps(cmd) + '\n',
            capture_output=True, text=True,
            timeout=timeout,
        )
        for line in r.stdout.split('\n'):
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if obj.get('action') != 'relocate':
                continue
            if obj.get('accepted') is True:
                return True, 'accepted'
            return False, obj.get('reason', line)
        return False, f'gateway returned: {r.stdout[:200]}'
    except subprocess.TimeoutExpired:
        return False, 'timeout'
    except Exception as e:
        return False, str(e)


def _check_localization(max_wait: float = 15.0) -> Tuple[bool, str, dict]:
    """Check if SLAM is localized via persistent world_state session.

    Returns (is_localized, status, world_state).
    """
    import threading
    import queue

    p = subprocess.Popen(
        [_GATEWAY, _NETWORK, '--persistent-world-state-session'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )

    q: queue.Queue = queue.Queue()

    def reader():
        for line in iter(p.stdout.readline, ''):
            line = line.strip()
            if line.startswith('{'):
                try:
                    obj = json.loads(line)
                    if 'world_state' in obj:
                        q.put(obj)
                except json.JSONDecodeError:
                    pass

    threading.Thread(target=reader, daemon=True).start()
    time.sleep(3)  # DDS settle

    deadline = time.monotonic() + max_wait
    last_status = 'unknown'
    last_ws = {}
    last_status_print = 0.0

    while time.monotonic() < deadline:
        p.stdin.write(json.dumps({
            'action': 'get_world_state',
            'request_id': f'loc_{int(time.monotonic() * 1000)}',
        }) + '\n')
        p.stdin.flush()

        try:
            obj = q.get(timeout=3)
            ws = obj['world_state']
            last_ws = ws
            loc = ws.get('localization', {})
            status = loc.get('status', 'unknown')
            last_status = status

            # "debug_map" is the placeholder — PCD not loaded
            if loc.get('map_id') == 'debug_map':
                p.stdin.close()
                p.terminate()
                return False, 'debug_map (PCD not loaded)', last_ws

            if status == 'localized':
                p.stdin.close()
                p.terminate()
                return True, 'localized', last_ws

            if status == 'lost':
                # Still converging, keep waiting
                now = time.monotonic()
                if now - last_status_print >= 1.5:
                    print(f'  [relocate] loc={status}, waiting...', flush=True)
                    last_status_print = now
        except queue.Empty:
            pass

    p.stdin.close()
    p.terminate()
    return False, f'still {last_status} after {max_wait}s', last_ws


def relocate_to_map(map_name: str, anchor_id: str = None,
                    max_attempts: int = 3, wait_s: float = 12.0) -> bool:
    """Relocate to a named PCD map with retry.

    Args:
        map_name: 'map_701', 'map_701_left', or 'map_terrace_wc'
        anchor_id: Override anchor (defaults to mapping_origin_<map>)
        max_attempts: Number of relocate attempts
        wait_s: Seconds to wait for localization after each attempt

    Returns True if successfully localized.
    """
    info = _MAP_INFO.get(map_name)
    if info is None:
        print('[relocate] unknown map: {}'.format(map_name), flush=True)
        return False

    pcd_path = info['pcd']
    anchor = anchor_id or info['anchor']

    # Remember the previous active PCD for switched_from tracking
    prev_active = get_active_pcd()
    prev_map_id = prev_active.get('active_map_id') if prev_active.get('active_map_id') != map_name else prev_active.get('switched_from')

    print('[relocate] target: {} pcd={} anchor={}'.format(map_name, pcd_path, anchor), flush=True)

    for attempt in range(1, max_attempts + 1):
        print('[relocate] attempt {}/{}...'.format(attempt, max_attempts), flush=True)

        ok, reason = _send_relocate(map_name, pcd_path, anchor)
        if not ok:
            print('[relocate] send failed: {}'.format(reason), flush=True)
            time.sleep(2)
            continue

        print('[relocate] command accepted, waiting {}s for convergence...'.format(wait_s), flush=True)
        localized, status, ws = _check_localization(max_wait=wait_s)

        if localized:
            pose = ws.get('current_pose', {}).get('pose', {})
            px = pose.get('x', 0)
            py = pose.get('y', 0)
            print('[relocate] ✓ localized at ({:.3f}, {:.3f})'.format(px, py), flush=True)

            # Persist active PCD state and sync registry
            _write_active_pcd(map_name, pcd_path, anchor, previous_map_id=prev_map_id)
            _update_registry_pcd_path(pcd_path)
            return True

        print('[relocate] ✗ not localized ({}), retrying...'.format(status), flush=True)

    print('[relocate] ✗ failed after {} attempts'.format(max_attempts), flush=True)
    return False


def list_maps():
    """Print available maps."""
    reg = _load_registry()
    for m in reg.get('maps', []):
        mid = m.get('map_id', '?')
        anchors = [a.get('anchor_id', '?') for a in m.get('relocalization_anchors', [])]
        pcd = m.get('pcd_path', '?')
        print(f'  {mid}: pcd={pcd} anchors={anchors}')


def main():
    if len(sys.argv) < 2 or sys.argv[1] == '--list':
        print('Available maps:')
        list_maps()
        return

    map_name = sys.argv[1]
    anchor = sys.argv[2] if len(sys.argv) > 2 else None
    ok = relocate_to_map(map_name, anchor)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
