# ROS2でVOICEVOXを使う環境をpixiで構築する
- Ubuntu24.04
- ros2 jazzy
- pixi 0.59.0
- voicevox 0.16.2

- ROS2 は /opt/ros/jazzy のものをそのまま使う
- Python パッケージ（VOICEVOX, sounddevice, numpyなど）は pixi で管理する

- pixi の中に ROS2 まで全部入れたい場合は　ros2-in-pixi.tomlを参考にしてください

[macOS arm64版はこちら](https://github.com/okadahiroyuki/voicevox_ros2_pixi/tree/osx_arm64)

## 0. pixiのインストール
[公式サイト](https://pixi.sh/dev/installation/)に従ってpixiをインストールする。
```
curl -fsSL https://pixi.sh/install.sh | sh
```

## 1. リポジトリのクローンと環境構築
### 本リポジトリをクローンしてください
```
cd ~/
git clone https://github.com/okadahiroyuki/voicevox_ros2_pixi.git
```
### 環境構築
```
cd ~/voicevox_ros2_pixi
# pixi環境構築
pixi install

# VOICEVOXのダウンロード
pixi run get_assets

# VOICEVOX Core の .whl ファイルのダウンロード
wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.16.2/voicevox_core-0.16.2-cp310-abi3-manylinux_2_34_x86_64.whl

# VOICEVOX コアホイールのインストール
pixi run install_voicevox
```
## 動作確認
### ROSノードの起動
```
cd ~/voicevox_ros2_pixi
pixi run run
```

### 別のターミナルから
```
cd ~/voicevox_ros2_pixi
pixi run say
```
あるいは
```
cd ~/voicevox_ros2_pixi
pixi run ros2 topic pub -1 /voicevox_tts std_msgs/String '{data: "おはようございます。これはピクシー環境からの音声です。"}'

```

