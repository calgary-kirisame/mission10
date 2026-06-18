#!/usr/bin/env bash
# Assemble a colcon workspace from this monorepo + pinned externals.
# Usage: scripts/make_workspace.sh <workspace-dir>
set -euo pipefail

ws="${1:?usage: make_workspace.sh <workspace-dir>}"
repo="$(cd "$(dirname "$0")/.." && pwd)"

command -v vcs >/dev/null || { echo "error: vcstool not found (pip install vcstool)" >&2; exit 1; }

mkdir -p "$ws/src"
vcs import "$ws/src" < "$repo/externals.repos"

for pkg in "$repo"/ros/*/; do
  ln -sfn "${pkg%/}" "$ws/src/$(basename "$pkg")"
done

echo "workspace ready: cd $ws && colcon build --symlink-install"
