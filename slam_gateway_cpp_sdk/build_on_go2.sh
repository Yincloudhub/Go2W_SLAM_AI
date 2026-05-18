#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
exec bash scripts/build_on_go2.sh "$@"
