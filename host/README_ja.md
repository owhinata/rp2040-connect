# imu-host

[English](README.md)

PC 側で IMU の RAW データを受信するツールです。ボードが MQTT で送る RAW バッチを直接受け付け、
次の 3 つを行います。

| コマンド | 内容 |
|---|---|
| `imu-host check` | 受信を診断する（欠落・時刻の整合の集計。保存しない） |
| `imu-host record` | 計測を開始して RAW を記録し、停止する |
| `imu-host calib` | 記録から加速度・角速度の校正値を求める |

ボード側の使い方は [`../README_ja.md`](../README_ja.md) を参照してください。

## 必要なもの

- Python 3.10 以上
- ボードが接続する AP を持つ PC（`script/wifi_ap.sh` で作る AP を想定）
- `calib` を使う場合は Docker。校正の計算に
  [imu_tk](https://bitbucket.org/alberto_pretto/imu_tk/) を使い、そのイメージを
  [owhinata/ubuntu-imu_tk](https://github.com/owhinata/ubuntu-imu_tk) からビルドします。

## 導入

```bash
cd host
python3 -m venv .venv
.venv/bin/pip install -e .
```

`host/.venv` に専用の環境を作り、`.venv/bin/imu-host` を入れます。依存は numpy だけです。
Zephyr 用の `.venv`（リポジトリ直下）とは別です。以下の例は `host/` で実行する前提です。

## ポート

どのコマンドも、ボードの送信先ポート（KV の `imu.port`、既定 1883）で待ち受けます。
`imu.port` を変えている場合は `--port` で合わせてください。`record` は、ボードの送信先と
待ち受けが違うと計測を止めてその旨を表示します。

## 受信の診断（check）

```bash
.venv/bin/imu-host check
```

MQTT クライアント 1 台を受け付け、RAW バッチの形式、連番、サンプル番号、時刻を検査します。
計測の開始と停止はボードのシェルで行います（`imu start` / `imu stop`）。Ctrl-C で終了すると、
集計を `RESULT` に続く JSON で 1 行出力します。主な項目は次のとおりです。

| 項目 | 意味 |
|---|---|
| `packets`、`accel`、`gyro` | 受信したバッチ数とサンプル数 |
| `missing_batches`、`missing_accel`、`missing_gyro`、`gaps` | 欠落の数と位置 |
| `invalid` | 仕様に合わないペイロードの数 |
| `unix_valid`、`unix_invalid`、`unix_regressions` | PC 時刻が有効・無効なバッチ数と、時刻の逆行 |
| `tick_regressions`、`ticks_per_sample` | センサー時刻の逆行と、1 サンプルあたりの tick 数 |
| `epoch_changes` | 個数不明の欠落（FIFO エラーからの再開） |
| `batch_unix`、`time_points` | バッチの時刻の間隔と、status の対応点との比較 |

受信側の停止や輻輳を再現するオプションがあります。

| オプション | 動作 |
|---|---|
| `--drop-at 秒`、`--drop-for 秒` | 最初の RAW から指定秒後に切断し、待ち受けも止めてから再開する |
| `--stall-at 秒`、`--stall-for 秒` | 接続したまま読み出しだけを止める |
| `--rcvbuf バイト` | 受信バッファーを小さくし、読み出しの停止を送信側へ早く伝える |
| `--duration 秒`、`--idle-timeout 秒` | 起動から、または受信が途絶えてからの秒数で終了する |

## 記録（record）

```bash
.venv/bin/imu-host record
```

1. AP に繋がっているボードを探します。
2. Telnet で `imu start` を送ります。
3. 受信した RAW と info・status を記録します。
4. Ctrl-C（または `--duration`）で `imu stop` を送り、残りを受け取って終了します。

ボードは USB のシリアルではなく Telnet で操作するので、モバイルバッテリーで動かしたままで
記録できます。記録先は既定で `recordings/<日時>/` です（`--out` で変更）。

記録中は 1 秒ごとに状況を表示します。

```text
  42 秒  静止  3.2 秒    姿勢  7  バッチ 1478  欠落 0  角速度の最大  412 dps
```

「姿勢」は、3 秒以上静止し、直前に数えた姿勢から重力の向きが 10 度以上違うものを数えた
目安です。校正で使われる静止区間は imu_tk が別に判定します。

| オプション | 意味 |
|---|---|
| `--board IP` | ボードの IP アドレスを指定する |
| `--no-board` | ボードを操作しない（`imu start` / `stop` は手動で行う） |
| `--duration 秒` | 最初の RAW からこの秒数で終了する |
| `--out DIR` | 記録先 |

ボードの IP アドレスは、`10.42.0.1`（NetworkManager の共有接続の既定値）を持つインターフェースの
近隣テーブルから探します。機器が複数繋がっていて決められないときや、AP のアドレスが違うときは
`--board` で指定してください。

### 記録の形式

受信したものを加工せずに保存します。校正はこの記録から何度でもやり直せます。

| ファイル | 内容 |
|---|---|
| `raw.bin` | RAW ペイロードの並び。各ペイロードの前に長さ（u32、little-endian） |
| `messages.jsonl` | info / status。1 行 1 件で、受信時刻 `t`、`topic`、`payload` |
| `meta.json` | 記録の開始・終了時刻と受信の集計 |

ペイロードの形式は [`../design/imu-raw-mqtt-design-ja.md`](../design/imu-raw-mqtt-design-ja.md) の
§6 を参照してください。

## 校正（calib）

加速度と角速度それぞれについて、スケール、軸のずれ（ミスアライメント）、バイアスを求めます。
基準器は要りません。いろいろな姿勢で静止させ、「静止時の加速度の大きさは重力に等しい」
「姿勢の間の回転は角速度の積分に等しい」という条件から求める方法（Tedaldi らの方法、imu_tk）
です。

### 取得の手順

1. AP を起動してから、ボードの電源を入れます。数分置いて温度を落ち着かせます。
2. `imu-host record` を起動します。
3. **最初の 10 秒以上は、机に置いたまま動かしません。**
4. 姿勢を変えて置き、**4 秒以上**待ちます。これを **36〜50 回**繰り返します。
5. Ctrl-C で終了します。

- 静止は、手で持って止めるのではなく、置いてください。手の震えは静止と判定されません。
- 姿勢は偏らないように、いろいろな向きにします。ボードとバッテリーを小さな箱に固定すると、
  多くの向きで安定して置けます。
- 欠落があると校正に使えないので、AP の近くで行ってください。
- 所要時間は 5 分前後です。

### 実行

```bash
.venv/bin/imu-host calib recordings/<日時>
```

1. 記録を検査します。欠落、epoch の変化、飽和、長さの不足があれば中止します。
2. imu_tk の Docker イメージがなければビルドします（初回は 10 分以上かかります）。
3. imu_tk を実行します。5 分の記録で 3〜4 分かかり、大半は静止区間の判定です。
4. 結果を検証して表示し、`recordings/<日時>/calib/calibration.json` に保存します。

| オプション | 既定 | 意味 |
|---|---|---|
| `--gravity` | 9.7975 | 重力 [m/s²]。既定は東京の値 |
| `--init-static` | 10 | 最初の静止時間 [秒] |
| `--interval` | 1.0 | 各静止区間から使う長さと、静止判定の窓 [秒] |
| `--force` | | 欠落や飽和があっても実行する |

`--gravity` などを変えると、その値のイメージを別にビルドします。imu_tk のパラメーターは
ビルド時に決まるためです。

### 結果

`calibration.json` の校正値は、RAW（LSB）から物理量への変換です。

```text
calibrated = misalignment @ scale @ (raw_lsb - bias_lsb)
```

| 項目 | 意味 |
|---|---|
| `accel`、`gyro` | `misalignment`（3×3）、`scale`（3×3 の対角）、`bias_lsb`。単位は m/s² と rad/s |
| `chip_id`、`session` | ボード固有の ID と、元の記録のセッション ID |
| `accel_ug_per_lsb`、`gyro_udps_per_lsb`、`*_odr_hz` | 記録時の感度と ODR。レンジを変えたら校正し直す |
| `imu_tk` | 使ったコミット、イメージ、パラメーター、角速度の最適化が収束したか |
| `recording` | 元の記録、長さ、tick の速さとその根拠 |
| `validation` | 下記の検証結果 |

検証は、角速度のばらつきが小さい 1 秒の窓を静止とみなして行います。

- 加速度の大きさと重力の差（校正前と校正後）
- 静止時の角速度の大きさ（校正前と校正後）
- 公称感度からのスケールのずれ

「校正前」は、公称感度だけで換算した値です。imu_tk の出力（`.calib`、ログ）と入力
（`acc.txt`、`gyro.txt`）も同じディレクトリに残ります。

### 時刻の扱い

角速度の積分にはサンプルの時刻の差が効くので、センサーの tick から時刻を作ります。tick の
速さは公称値（25 µs）から 0.5% ほどずれているため、status に含まれる対応点（センサーの tick と
ボードの時計）から実測します。NTP の同期がなくても校正できます。

## 確認した範囲

- `check` と `record` は実機で確認しています（6667 Hz、欠落なしの受信、切断と再接続の検出、
  Telnet での開始と停止）。
- `calib` は、既知の校正値を埋め込んだ合成データ（約 4 分、40 姿勢、ノイズあり）で、スケールを
  0.03% 以内、バイアスを 0.3 LSB 以内で復元できることを確認しています。
- **実機で姿勢を変えて取った記録での校正は、まだ確認していません。**
- 校正値は校正時の温度のものです。RAW に温度は含まれていません。

## 構成

| ファイル | 内容 |
|---|---|
| `imu_host/packet.py` | IMU1 ペイロードの検査と取り出し |
| `imu_host/mqtt.py` | MQTT の受け口（クライアント 1 台） |
| `imu_host/board.py` | ボードの IP アドレスの特定と Telnet 操作 |
| `imu_host/recording.py` | 記録の読み書きと、tick から時刻への変換 |
| `imu_host/check.py`、`record.py`、`calib.py` | 各コマンド |
