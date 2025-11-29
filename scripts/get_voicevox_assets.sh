#!/usr/bin/env bash
set -euo pipefail

ENGINE_DIR="engine"
mkdir -p "${ENGINE_DIR}"
cd "${ENGINE_DIR}"

echo "[*] Download VOICEVOX Core assets (linux x64, CPU, models: 0.vvm)"

VER="0.16.2"
DL="download-linux-x64"
URL="https://github.com/VOICEVOX/voicevox_core/releases/download/${VER}/${DL}"

if [[ -n "${GITHUB_TOKEN:-}" ]]; then
  echo "[*] Using GITHUB_TOKEN for authenticated download"
  curl -fsSL -H "Authorization: token ${GITHUB_TOKEN}" -o download "${URL}"
else
  curl -fsSL -o download "${URL}"
fi

chmod +x download

yes | ./download \
  --devices cpu \
  --os linux \
  --cpu-arch x64 \
  --models-pattern 0.vvm

echo "[*] DONE. Assets are under $(pwd)"

