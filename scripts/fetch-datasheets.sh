#!/usr/bin/env bash
#
# fetch-datasheets.sh — re-download the manufacturer datasheets for the custom-driver
# ICs in this project into datasheets/.
#
# Why this script exists: the manufacturer CDNs block plain HTTP clients (the kind a
# generic "fetch" tool uses) with HTTP 403, and Semtech wraps its PDFs in a Salesforce
# content-distribution session flow. Both are handled below.
#
# Usage:  ./scripts/fetch-datasheets.sh
#
set -euo pipefail

OUT="$(cd "$(dirname "$0")/.." && pwd)/datasheets"
mkdir -p "$OUT"

# A real browser User-Agent is the whole trick for most vendors (TI/ADI/ST/NXP/...).
UA="Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0 Safari/537.36"

say() { printf '\n==> %s\n' "$*"; }

# ---------------------------------------------------------------------------
# 1) Semtech SX1280 / SX1281 radio  (XLRS PHY)
# ---------------------------------------------------------------------------
# Semtech does NOT host a static PDF. The "SX1280 Datasheet" link is a Salesforce
# content-distribution page: GET sets a session cookie -> a JS auto-POST-back -> the
# real file is served by the renditionDownload servlet (rendition=ORIGINAL_Pdf).
#
# DIST is the public distribution link (from the Semtech SX1280 product page, the
# "SX1280 Datasheet" entry). The versionId/contentId/oid below were read from the
# viewer's own network requests (DevTools > Network, or the renditionDownload calls).
# If Semtech republishes the doc these IDs change — refresh them by opening DIST in a
# browser and copying them out of any /sfc/dist/version/renditionDownload?... request.
DIST="https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo"
COMP="E0000000JelG/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo"
VERSION_ID="0683n00000SSjQI"
CONTENT_ID="05T3n00001xCS1F"
OID="00DE0000000JelG"
REND="https://semtech.my.salesforce.com/sfc/dist/version/renditionDownload?rendition=ORIGINAL_Pdf&versionId=${VERSION_ID}&operationContext=DELIVERY&contentId=${CONTENT_ID}&oid=${OID}&d=/a/3n000000l9OZ/Kw7ZeYZuAZW3Q4A3R_IUjhYCQEJxkuLrUgl_GNNhuUo"

say "Semtech SX1280/SX1281 datasheet"
CJ="$(mktemp)"
trap 'rm -f "$CJ"' EXIT
curl -fsL -A "$UA" -c "$CJ" -o /dev/null "$DIST"                                  # establish session
curl -fsL -A "$UA" -b "$CJ" -c "$CJ" --data-urlencode "compositePageName=$COMP" \
  -e "$DIST" -o /dev/null "$DIST"                                                 # JS POST-back
curl -fL  -A "$UA" -b "$CJ" -e "$DIST" -o "$OUT/sx1280-sx1281-semtech.pdf" "$REND"

# ---------------------------------------------------------------------------
say "Done. Verifying:"
for f in "$OUT"/*.pdf; do
  printf '  %-32s %s\n' "$(basename "$f")" "$(file -b "$f" 2>/dev/null | cut -c1-30)"
done
