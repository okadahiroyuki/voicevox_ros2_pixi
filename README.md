# ROS2でVOICEVOXを使う環境をpixiで構築する
- Ubuntu24.04
- ros2 jazzy
- pixi 0.59.0
- voicevox 0.16.2

- ROS2 は /opt/ros/jazzy のものをそのまま使う
- Python パッケージ（VOICEVOX, sounddevice, numpyなど）は pixi で管理する

- pixi の中に ROS2 まで全部入れたい場合は　ros2-in-pixi.tomlを参考にしてください

[macOS arm64版はこちら](https://github.com/okadahiroyuki/voicevox_ros2_pixi/tree/osx-arm64)

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
## 仕様
サブスクライブするトピックは `/tts_text (std_msgs/String)`。

メッセージ書式：
- 例1（デフォルト話者）："data: 'サンプル音声です'"
- 例2（style_id=1 でしゃべらせる）："data: '[1] 別の話者でしゃべります'"
- 例3（style_id=8 でしゃべらせる）："data: '[8] おはようございます'"

VOICEVOX Ver.0.25.0 での　stayle_id 一覧

- 四国めたん, ノーマル id: 2
- 四国めたん, あまあま id: 0
- 四国めたん, ツンツン id: 6
- 四国めたん, セクシー id: 4
- 四国めたん, ささやき id: 36
- 四国めたん, ヒソヒソ id: 37
- ずんだもん, ノーマル id: 3
- ずんだもん, あまあま id: 1
- ずんだもん, ツンツン id: 7
- ずんだもん, セクシー id: 5
- ずんだもん, ささやき id: 22
- ずんだもん, ヒソヒソ id: 38
- ずんだもん, ヘロヘロ id: 75
- ずんだもん, なみだめ id: 76
- 春日部つむぎ, ノーマル id: 8
- 雨晴はう, ノーマル id: 10
- 波音リツ, ノーマル id: 9
- 波音リツ, クイーン id: 65
- 玄野武宏, ノーマル id: 11
- 玄野武宏, 喜び id: 39
- 玄野武宏, ツンギレ id: 40
- 玄野武宏, 悲しみ id: 41
- 白上虎太郎, ふつう id: 12
- 白上虎太郎, わーい id: 32
- 白上虎太郎, びくびく id: 33
- 白上虎太郎, おこ id: 34
- 白上虎太郎, びえーん id: 35
- 青山龍星, ノーマル id: 13
- 青山龍星, 熱血 id: 81
- 青山龍星, 不機嫌 id: 82
- 青山龍星, 喜び id: 83
- 青山龍星, しっとり id: 84
- 青山龍星, かなしみ id: 85
- 青山龍星, 囁き id: 86
- 冥鳴ひまり, ノーマル id: 14
- 九州そら, ノーマル id: 16
- 九州そら, あまあま id: 15
- 九州そら, ツンツン id: 18
- 九州そら, セクシー id: 17
- 九州そら, ささやき id: 19
- もち子さん, ノーマル id: 20
- もち子さん, セクシー／あん子 id: 66
- もち子さん, 泣き id: 77
- もち子さん, 怒り id: 78
- もち子さん, 喜び id: 79
- もち子さん, のんびり id: 80
- 剣崎雌雄, ノーマル id: 21
- WhiteCUL, ノーマル id: 23
- WhiteCUL, たのしい id: 24
- WhiteCUL, かなしい id: 25
- WhiteCUL, びえーん id: 26
- 後鬼, 人間ver. id: 27
- 後鬼, ぬいぐるみver. id: 28
- 後鬼, 人間（怒り）ver. id: 87
- 後鬼, 鬼ver. id: 88
- No.7, ノーマル id: 29
- No.7, アナウンス id: 30
- No.7, 読み聞かせ id: 31
- ちび式じい, ノーマル id: 42
- 櫻歌ミコ, ノーマル id: 43
- 櫻歌ミコ, 第二形態 id: 44
- 櫻歌ミコ, ロリ id: 45
- 小夜/SAYO, ノーマル id: 46
- ナースロボ＿タイプＴ, ノーマル id: 47
- ナースロボ＿タイプＴ, 楽々 id: 48
- ナースロボ＿タイプＴ, 恐怖 id: 49
- ナースロボ＿タイプＴ, 内緒話 id: 50
- †聖騎士 紅桜†, ノーマル id: 51
- 雀松朱司, ノーマル id: 52
- 麒ヶ島宗麟, ノーマル id: 53
- 春歌ナナ, ノーマル id: 54
- 猫使アル, ノーマル id: 55
- 猫使アル, おちつき id: 56
- 猫使アル, うきうき id: 57
- 猫使アル, つよつよ id: 110
- 猫使アル, へろへろ id: 111
- 猫使ビィ, ノーマル id: 58
-  猫使ビィ, おちつき id: 59
-  猫使ビィ, 人見知り id: 60
-  猫使ビィ, つよつよ id: 112
-  中国うさぎ, ノーマル id: 61
-  中国うさぎ, おどろき id: 62
-  中国うさぎ, こわがり id: 63
-  中国うさぎ, へろへろ id: 64
-  栗田まろん, ノーマル id: 67
-  あいえるたん, ノーマル id: 68
-  満別花丸, ノーマル id: 69
-  満別花丸, 元気 id: 70
-  満別花丸, ささやき id: 71
-  満別花丸, ぶりっ子 id: 72
-  満別花丸, ボーイ id: 73
-  琴詠ニア, ノーマル id: 74
-  Voidoll, ノーマル id: 89
-  ぞん子, ノーマル id: 90
-  ぞん子, 低血圧 id: 91
-  ぞん子, 覚醒 id: 92
-  ぞん子, 実況風 id: 93
-  中部つるぎ, ノーマル id: 94
-  中部つるぎ, 怒り id: 95
-  中部つるぎ, ヒソヒソ id: 96
-  中部つるぎ, おどおど id: 97
-  中部つるぎ, 絶望と敗北 id: 98
-  離途, ノーマル id: 99
-   離途, シリアス id: 101
-   黒沢冴白, ノーマル id: 100
-   ユーレイちゃん, ノーマル id: 102
-   ユーレイちゃん, 甘々 id: 103
-   ユーレイちゃん, 哀しみ id: 104
-   ユーレイちゃん, ささやき id: 105
-   ユーレイちゃん, ツクモちゃん id: 106
-   東北ずん子, ノーマル id: 107
-   東北きりたん, ノーマル id: 108
-   東北イタコ, ノーマル id: 109
