#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import traceback
from typing import Optional
import io
import wave

import numpy as np
import sounddevice as sd

# rclpy は ROS の site-packages を PYTHONPATH で通す前提
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# VOICEVOX Core (blocking API)
from voicevox_core.blocking import Onnxruntime, OpenJtalk, Synthesizer, VoiceModelFile


ENGINE_DIR = os.path.join(os.path.dirname(__file__), "..", "engine")
ONNX_LIB = os.path.join(ENGINE_DIR, "onnxruntime", "lib", Onnxruntime.LIB_VERSIONED_FILENAME)
OPENJTALK_DICT = os.path.join(ENGINE_DIR, "dict", "open_jtalk_dic_utf_8-1.11")
MODEL_PATH = os.path.join(ENGINE_DIR, "models", "vvms", "0.vvm")

SAMPLE_RATE = 24000  # デフォルト（WAV から読み取ったらそちらを優先）


def ensure_assets() -> Optional[str]:
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
            raise RuntimeError("assets missing")

        # Synthesizer 初期化
        self.get_logger().info("Initializing VOICEVOX Synthesizer ...")
        self.synth = Synthesizer(
            Onnxruntime.load_once(filename=ONNX_LIB),
            OpenJtalk(OPENJTALK_DICT)
        )
        with VoiceModelFile.open(MODEL_PATH) as vm:
            self.synth.load_voice_model(vm)

        # パラメータ
        self.declare_parameter("style_id", 0)  # 話者/スタイル ID
        self.style_id = int(self.get_parameter("style_id").value)

        # 購読トピック
        topic = "/voicevox_tts"
        self.create_subscription(String, topic, self.cb_tts, 10)
        self.get_logger().info(f"Subscribed: {topic}")
        self.get_logger().info(f"Use style_id={self.style_id}")

        # 再生のデフォルト設定（チャンネル数はWAVから上書きされる）
        sd.default.samplerate = SAMPLE_RATE
        sd.default.channels = 1

    def _play_wav_bytes(self, wav_bytes: bytes):
        """RIFF WAVE バイナリを読み込んで再生"""
        with wave.open(io.BytesIO(wav_bytes), "rb") as wf:
            sr = wf.getframerate()
            ch = wf.getnchannels()
            frames = wf.readframes(wf.getnframes())

        pcm = np.frombuffer(frames, dtype=np.int16)

        # sounddevice にチャンネル数を反映
        sd.default.samplerate = sr
        sd.default.channels = ch

        sd.play(pcm, samplerate=sr, blocking=True)

    def cb_tts(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"TTS: \"{text}\"")
        try:
            # tts は環境によって numpy か WAV bytes を返すことがあるので両対応にする
            wav = self.synth.tts(text, style_id=self.style_id)

            if isinstance(wav, (bytes, bytearray)):
                # RIFF WAVE バイナリとして解釈
               self._play_wav_bytes(wav)
            else:
                # すでに PCM ndarray[int16] 形式の場合
                pcm = np.asarray(wav, dtype=np.int16)
                sd.default.samplerate = SAMPLE_RATE
                sd.default.channels = 1
                sd.play(pcm, samplerate=SAMPLE_RATE, blocking=True)

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

