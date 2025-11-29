#!/usr/bin/env bash
set -euo pipefail

# 保存先
ENGINE_DIR="engine"
mkdir -p "${ENGINE_DIR}"
cd "${ENGINE_DIR}"

echo "[*] Download VOICEVOX Core assets (linux x64, CPU, models: 0.vvm)"

# 公式のダウンロードスクリプトを取得して実行
# ref: https://github.com/VOICEVOX/voicevox_core/releases
VER="0.16.2"
DL="download-linux-x64"
URL="https://github.com/VOICEVOX/voicevox_core/releases/download/${VER}/${DL}"

# 認証ヘッダ（任意）
if [[ -n "${GITHUB_TOKEN:-}" ]]; then
  echo "[*] Using GITHUB_TOKEN for authenticated download"
  curl -fsSL -H "Authorization: token ${GITHUB_TOKEN}" -o download "${URL}"
else
  curl -fsSL -o download "${URL}"
fi

chmod +x download

# 必要な資産を落とす（CPU / linux / x64 / モデル 0.vvm）
yes | ./download \
  --devices cpu \
  --os linux \
  --cpu-arch x64 \
  --models-pattern 0.vvm

# --- ホイール自動DL ---
cd ..
if [ ! -f voicevox_core-0.16.2-cp310-abi3-manylinux_2_34_x86_64.whl ]; then
  echo "[+] Downloading VOICEVOX core wheel..."
  wget -q https://github.com/VOICEVOX/voicevox_core/releases/download/0.16.2/voicevox_core-0.16.2-cp310-abi3-manylinux_2_34_x86_64.whl
fi

echo "[*] DONE. Assets are under $(pwd)"
