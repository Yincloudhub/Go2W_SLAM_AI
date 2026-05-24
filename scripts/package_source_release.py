from __future__ import annotations

import argparse
import fnmatch
import json
import subprocess
import time
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

EXCLUDE_DIRS = {
    ".git",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "artifacts",
    "build",
    "cmake-build-debug",
    "cmake-build-release",
    "logs",
    "models",
    "tmp",
    "venv",
}

EXCLUDE_PATTERNS = {
    "*.bag",
    "*.db3",
    "*.engine",
    "*.gguf",
    "*.log",
    "*.onnx",
    "*.pcd",
    "*.pt",
    "*.pth",
    "*.pyc",
    "*.safetensors",
}


def run_git(args: list[str]) -> str:
    completed = subprocess.run(["git", *args], cwd=REPO_ROOT, text=True, encoding="utf-8", errors="replace", capture_output=True, check=True)
    return completed.stdout


def desktop_dir() -> Path:
    candidates = [Path.home() / "Desktop", Path.home() / "桌面"]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return Path.cwd()


def normalize_path(path: str) -> str:
    return path.replace("\\", "/").strip("/")


def should_include(rel_path: str) -> bool:
    rel = normalize_path(rel_path)
    parts = rel.split("/")
    if any(part in EXCLUDE_DIRS for part in parts):
        return False
    return not any(fnmatch.fnmatch(Path(rel).name, pattern) or fnmatch.fnmatch(rel, pattern) for pattern in EXCLUDE_PATTERNS)


def source_files() -> list[str]:
    raw = run_git(["ls-files", "--cached", "--others", "--exclude-standard", "-z"])
    files = [normalize_path(item) for item in raw.split("\0") if item]
    return sorted(rel for rel in files if should_include(rel) and (REPO_ROOT / rel).is_file())


def git_metadata(files: list[str]) -> dict[str, object]:
    def try_git(args: list[str], fallback: str = "") -> str:
        try:
            return run_git(args).strip()
        except subprocess.CalledProcessError:
            return fallback

    return {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "repo": str(REPO_ROOT),
        "branch": try_git(["branch", "--show-current"]),
        "commit": try_git(["rev-parse", "--short", "HEAD"]),
        "status": try_git(["status", "--short"]),
        "file_count": len(files),
        "excluded_dirs": sorted(EXCLUDE_DIRS),
        "excluded_patterns": sorted(EXCLUDE_PATTERNS),
    }


def package(output: Path) -> dict[str, object]:
    files = source_files()
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = git_metadata(files)
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for rel in files:
            archive.write(REPO_ROOT / rel, arcname=f"GO2W_SLAM_AI/{rel}")
        archive.writestr("GO2W_SLAM_AI/PACKAGE_MANIFEST.json", json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    metadata["output"] = str(output)
    metadata["size_bytes"] = output.stat().st_size
    return metadata


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create a GO2W source-code ZIP without models, PCDs, runtime logs, or build artifacts.")
    parser.add_argument("--output", default="", help="ZIP path. Defaults to the current user's Desktop.")
    parser.add_argument("--pretty", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output = Path(args.output) if args.output else desktop_dir() / f"GO2W_SLAM_AI_source_{timestamp}.zip"
    result = package(output)
    print(json.dumps(result, ensure_ascii=False, indent=2 if args.pretty else None))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
