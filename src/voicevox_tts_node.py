#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import wave
import traceback
from typing import Optional

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voicevox_core.blocking import Onnxruntime, OpenJtalk, Synthesizer, VoiceModelFile


# =========================
# VOICEVOX assets path detection
# =========================

# このファイルの場所: src/voicevox_tts_node.py
# assets はプロジェクト直下の engine/ 以下にある想定
BASE_ENGINE_DIR = os.path.join(os.path.dirname(__file__), "..", "engine")

# 想定されるレイアウト:
# 1) engine/onnxruntime, engine/dict, engine/models
# 2) engine/voicevox_core/onnxruntime, engine/voicevox_core/dict, engine/voicevox_core/models
CANDIDATES = [
    BASE_ENGINE_DIR,
    os.path.join(BASE_ENGINE_DIR, "voicevox_core"),
]

ENGINE_ROOT = None
for root in CANDIDATES:
    onnx_dir = os.path.join(root, "onnxruntime", "lib")
    dict_dir = os.path.join(root, "dict", "open_jtalk_dic_utf_8-1.11")
    model_file = os.path.join(root, "models", "vvms", "0.vvm")
    if os.path.isdir(onnx_dir) and os.path.isdir(os.path.dirname(dict_dir)) and os.path.isfile(model_file):
        ENGINE_ROOT = root
        break

if ENGINE_ROOT is None:
    # 見つからなかった場合は engine/voicevox_core を仮定（ensure_assetsで実際にチェック）
    ENGINE_ROOT = os.path.join(BASE_ENGINE_DIR, "voicevox_core")

ONNX_LIB = os.path.join(ENGINE_ROOT, "onnxruntime", "lib", Onnxruntime.LIB_VERSIONED_FILENAME)
OPENJTALK_DICT = os.path.join(ENGINE_ROOT, "dict", "open_jtalk_dic_utf_8-1.11")
MODEL_PATH = os.path.join(ENGINE_ROOT, "models", "vvms", "0.vvm")

# VOICEVOX のデフォルトサンプリングレート（WAVから読み取ったらそちらを優先）
DEFAULT_SAMPLE_RATE = 24000


def ensure_assets() -> Optional[str]:
    """必要なファイルが揃っているか確認。欠けていたらエラーメッセージを返す。"""
    missing = []
    if not os.path.exists(ONNX_LIB):
        missing.append(f"ONNXRuntime lib not found: {ONNX_LIB}")
    if not os.path.isdir(OPENJTALK_DICT):
        missing.append(f"OpenJTalk dict not found: {OPENJTALK_DICT}")
    if not os.path.isfile(MODEL_PATH):
        missing.append(f"Voice model not found: {MODEL_PATH}")
    if missing:
        return "\n".join(missing)
    return None


class VoiceVoxNode(Node):
    def __init__(self):
        super().__init__("voicevox_tts")

        err = ensure_assets()
        if err:
            self.get_logger().error("VOICEVOX assets missing:\n" + err)
            self.get_logger().error("Run:  pixi run get_assets")
            raise RuntimeError("VOICEVOX assets missing")

        # Synthesizer 初期化
        self.get_logger().info(f"Using ENGINE_ROOT = {ENGINE_ROOT}")
        self.get_logger().info("Initializing VOICEVOX Synthesizer ...")
        self.synth = Synthesizer(
            Onnxruntime.load_once(filename=ONNX_LIB),
            OpenJtalk(OPENJTALK_DICT),
        )
        with VoiceModelFile.open(MODEL_PATH) as vm:
            self.synth.load_voice_model(vm)

        # 話者/スタイル ID（必要に応じて変更）
        self.declare_parameter("style_id", 0)
        self.style_id = int(self.get_parameter("style_id").value)

        # 購読トピック
        topic = "/voicevox_tts"
        self.create_subscription(String, topic, self.cb_tts, 10)
        self.get_logger().info(f"Subscribed: {topic}")
        self.get_logger().info(f"Use style_id={self.style_id}")

        # デフォルトの再生設定（WAVから上書きされる場合あり）
        sd.default.samplerate = DEFAULT_SAMPLE_RATE
        sd.default.channels = 1

    # ---------- WAVバイナリ再生 ----------
    def _play_wav_bytes(self, wav_bytes: bytes):
        """RIFF WAVE バイナリを読み込んで再生"""
        with wave.open(io.BytesIO(wav_bytes), "rb") as wf:
            sr = wf.getframerate()
            ch = wf.getnchannels()
            frames = wf.readframes(wf.getnframes())

        pcm = np.frombuffer(frames, dtype=np.int16)

        sd.default.samplerate = sr
        sd.default.channels = ch
        sd.play(pcm, samplerate=sr, blocking=True)

    # ---------- コールバック ----------
    def cb_tts(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS: \"{text}\"")
        try:
            # tts は環境によって numpy ndarray[int16] または WAV bytes を返すことがある
            wav = self.synth.tts(text, style_id=self.style_id)

            if isinstance(wav, (bytes, bytearray)):
                # RIFF/WAVE バイナリ
                self._play_wav_bytes(wav)
            else:
                # PCM ndarray[int16] 相当とみなす
                pcm = np.asarray(wav, dtype=np.int16)
                sd.default.samplerate = DEFAULT_SAMPLE_RATE
                sd.default.channels = 1
                sd.play(pcm, samplerate=DEFAULT_SAMPLE_RATE, blocking=True)

        except Exception as e:
            self.get_logger().error(f"Synthesis/playback error: {e}")
            self.get_logger().error(traceback.format_exc())


def main():
    rclpy.init()
    node = None
    try:
        node = VoiceVoxNode()
        rclpy.spin(node)
    except Exception:
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

