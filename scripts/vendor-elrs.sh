#!/usr/bin/env bash
# Pin ExpressLRS submodule to a release tag (default 3.5.6).
# ExpressLRS is GPL-3.0 — see third_party/ExpressLRS/LICENSE.
set -euo pipefail

TAG="${1:-3.5.6}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELRS_DIR="$ROOT/third_party/ExpressLRS"

cd "$ROOT"

if [[ ! -f .gitmodules ]] || ! grep -q 'third_party/ExpressLRS' .gitmodules 2>/dev/null; then
  git submodule add https://github.com/ExpressLRS/ExpressLRS.git third_party/ExpressLRS
fi

git submodule update --init --recursive third_party/ExpressLRS
git -C "$ELRS_DIR" fetch --tags origin
git -C "$ELRS_DIR" checkout "$TAG"

REV="$(git -C "$ELRS_DIR" rev-parse --short HEAD)"
echo "ExpressLRS ready: tag=$TAG commit=$REV"
