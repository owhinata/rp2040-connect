# rp2040-connect

[English](README.md) | 日本語

Arduino Nano RP2040 Connect（RP2040、NINA-W102、LSM6DSOX）向けの Zephyr ワークスペースです。
主なアプリ `zephyr/app/usb-console` は、IMU の加速度・角速度を 6667 Hz の RAW データのまま取得し、
PC の時刻を付けて Wi-Fi 経由の MQTT で送信します。

- IMU の FIFO を I²C（DMA）で読み出し、欠落を検出できる番号を付けてバッチ化します。
- Wi-Fi が切れても取得は続け、送れなかった分は破棄して数えます。
- PC の NTP サーバーと同期し、各バッチに PC 時刻（Unix 時刻、µs）を付けます。
- USB と Telnet のシェルから操作します。

設計の詳細は [IMU RAW・MQTT 配信の設計](design/imu-raw-mqtt-design-ja.md)、ドライバー
（LSM6DSO、I²C、NINA）は [ローカルドライバーの説明](zephyr/app/usb-console/drivers/README_ja.md) を
参照してください。

現状は実験段階です。計測の開始・停止はシェルからの手動操作で、PC 側には検証用の受信
スクリプトしかありません。確認できた範囲と未確認の範囲は設計書にまとめています。

## 環境構築

Ubuntu 24.04 向けの手順です。まずビルドに必要なパッケージを入れます。

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

AArch64 環境では `gcc-multilib` と `g++-multilib` を除いてください。他の OS は
[Zephyr の公式手順](https://docs.zephyrproject.org/latest/develop/getting_started/)を参照してください。

Zephyr のワークスペースは `zephyr/` の下にあります。以下のコマンドは `zephyr/` で実行します。
まず、セットアップスクリプトを読み込みます。

```bash
cd zephyr
source script/setup.sh
```

スクリプトは必要に応じて `zephyr/` の下に `.venv`、west ワークスペース（Zephyr v4.5.0 とモジュール）、
Python パッケージ、Zephyr SDK を用意し、現在のシェルに `ZEPHYR_SDK_INSTALL_DIR` と `west` の
エイリアスを設定します。新しいターミナルでは再度読み込んでください。

## ビルドと書き込み

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

構成は `zephyr/app/usb-console/prj.conf` の 1 つだけです。

書き込みの前に、ボードを USB に接続してブートローダーモード（`RPI-RP2` という USB
ストレージが現れる状態）にします。RESET ボタンを 2 回押しても入れない場合は、`REC` と `GND` を
ジャンパー線で接続したまま RESET ボタンを押し、ストレージが現れたらジャンパー線を外します。

`zephyr/app/blink` は `led0` を 500 ms ごとに切り替える最小のサンプルです。

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

NINA-W102 側は、Arduino の NINA ファームウェア 3.0.2 に 2 件の修正を当てたものを前提に
しています。ビルドと注意点は [NINA ファームウェアの説明](firmware/nina-fw/README.md) を参照して
ください。ボードに入っているバージョンは `nina version` で確認できます。

## 使い方

### 1. シェルに接続する

USB シリアル（CDC ACM）に端末を接続し、Enter を押すとプロンプトが出ます。

```bash
minicom -D /dev/ttyACM0
```

デバイス名は環境によって変わります。`/dev/serial/by-id/` で確認できます。

### 2. Wi-Fi に接続する

2.4 GHz の WPA2-PSK に対応しています。認証情報を保存しておくと、起動時と予期しない切断の
あとに自動で接続します。

```text
wifi cred add -s "SSID" -k 1 -p "パスフレーズ"
wifi cred auto_connect
wifi status
net iface
```

- パスフレーズは端末に表示されます。`wifi cred list` でも表示されます。
- `wifi connect` は接続だけを行い、設定を保存しません。
- `wifi disconnect` で切断した場合は、次の起動か手動接続まで再接続しません。
- `wifi scan` は接続を一時的に切り、終わると保存済みの設定で接続し直します。

PC 側の Wi-Fi アダプターを一時的な AP にするには `script/wifi_ap.sh` が使えます。

```bash
./script/wifi_ap.sh ap <USB-interface> --ssid rp2040-test --channel 6
./script/wifi_ap.sh station <USB-interface>
```

### 3. PC で NTP サーバーを動かす

ボードは既定で、デフォルトゲートウェイ（上の AP を提供している PC）に 2 秒ごとに NTP で
問い合わせます。PC には chrony を入れ、AP 側のサブネットからの問い合わせを許可します。

```bash
sudo apt install chrony
sudo tee /etc/chrony/conf.d/rp2040-ap.conf >/dev/null <<'CONF'
allow 10.42.0.0/24
local stratum 10
CONF
sudo systemctl restart chrony
```

ファイアウォール（ufw など）が有効なら UDP 123 番を開けてください。ボード側の状態は
`ntp status` で確認できます。

### 4. PC で受信する

PC 側のツールは `host/` にあります。導入方法と、記録（`record`）・校正（`calib`）の使い方は
[`host/README_ja.md`](host/README_ja.md) を参照してください。受信したデータを物理量に変換して
姿勢を表示するデスクトップアプリは [`host/ImuStudio/`](host/ImuStudio/README_ja.md) にあります。

`imu-host check` は検証用の受信器です。MQTT クライアント 1 台を直接受け付け、RAW バッチの
形式、連番、サンプル番号、時刻を検査し、終了時に集計を JSON で出力します。汎用の MQTT
ブローカーではなく、保存もしません。

```bash
host/.venv/bin/imu-host check --port 1883
```

Ctrl-C で終了すると結果を表示します。`--drop-at`、`--stall-at`、`--rcvbuf` で受信側の停止や
輻輳を再現できます。

### 5. 計測を開始する

```text
imu start
imu status
imu stop
```

`imu start` は引数を取りません。センサーを 6667 Hz・FIFO watermark 128・I²C 1 MHz に設定し、
止めるまで送信を続けます。

- **MQTT サーバーが動いていないとき**：取得は始まり、繋がるまでのバッチは破棄して数えます。
  繋がった時点から届きます。
- **NTP の同期が取れていないとき**：送信は行い、バッチの PC 時刻を無効にします。センサーの
  tick は付きます。
- **Wi-Fi に繋がっていないとき**：取得だけ始まり、Wi-Fi と MQTT が繋がり次第送信します。

設定は KV ストレージで変えられます。どれも省略でき、省略時は既定値を使います。

| キー | 意味 | 既定値 |
|---|---|---|
| `imu.host` | MQTT サーバーの IPv4 アドレス | デフォルトゲートウェイ |
| `imu.port` | MQTT サーバーのポート | `1883` |
| `imu.device` | トピック `imu/<device>/raw` に使う名前 | `rp2040` |
| `imu.odr` | 加速度・角速度の ODR（Hz） | `6667` |
| `imu.watermark` | FIFO watermark（ワード数） | `128` |

```text
kv set imu.port 18830
kv delete imu.port
```

セッション ID は開始のたびに自動で割り当てます（チップ固有の ID と、保存した通し番号）。

## シェルコマンド

| コマンド | 内容 |
|---|---|
| `imu start` / `stop` / `status` | 計測の開始・停止、カウンターとエラーの表示 |
| `imu cal` / `imu cal reload` / `imu cal zero` | `sensor get` / `stream` の変換の表示、KV の読み直し、静止時の角速度バイアスの取り直し |
| `ntp status` | PC 時刻との同期の状態 |
| `wifi ...`、`net ...` | Wi-Fi と IP の操作（Zephyr 標準） |
| `nina version` | NINA ファームウェアのバージョン |
| `kv set` / `sethex` / `get` / `delete` | KV ストレージ |
| `sensor ...` | センサーの単発取得・ストリーム・属性（Zephyr 標準を拡張） |
| `i2c_dma mode` / `stats` | I²C 読み出しの DMA 切り替えと統計 |
| `zperf ...` | iperf3 による速度測定（Zephyr 標準、下記） |
| `dmesg` | RAM に保存したログの表示 |
| `kernel ps`、`kernel reboot` | スレッドの CPU・スタック、再起動 |
| `gpio ...`、`i2c ...`、`log ...` | Zephyr 標準 |

### `imu status`

1 項目 1 行で、カテゴリーごとに表示します。`errors` と `fifo` の各項目、`time` の
`unix_unavailable` から `tick_out_of_range` までは、正常なら 0 のままです。

| カテゴリー | 見るところ |
|---|---|
| 先頭 | `state`、セッション、`epoch`（個数不明の欠落ごとに 1 進む）、`cpu_active` |
| `acquired` | 取得したサンプル数とバッチ数（送信できたかどうかに関係なく進む） |
| `errors` | ストリームが止まった原因。0 以外なら計測は終了している |
| `fifo` | FIFO オーバーラン、I²C エラー、自動再開の回数など |
| `time` | PC との同期の有無、時刻を付けられなかった数、タイムスタンプの規則からの逸脱 |
| `mqtt` | 接続先、接続状態、送信数、破棄したバッチ数、キューの最大使用量 |

`state` は `idle`（未計測）、`starting`（開始直後で、まだ最初の集計が出ていない）、`running`、
`stopped` のいずれかです。`starting` の間、以降の項目は前回の計測の値のままです。

### 校正値の適用

`sensor get` と `sensor stream` が出す物理量（m/s²、rad/s）には、KV に保存した校正値を適用
できます。校正値は PC 側の `imu-host calib` で求め、`imu-host write` で書き込みます
（[`host/README_ja.md`](host/README_ja.md)）。**MQTT で送る RAW には適用しません。**

```text
imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.65 -4.25 -0.81 LSB (tracked at rest, 14 updates, 0 rejected)
```

校正値はセンサーと感度ごとのキーで持ち、現在の感度のキーがあるときだけ使います。レンジを
変えるなどして対応するキーがなければ、公称感度での変換（`nominal`）になります。`kv` で
キーを書き換えたあとは `imu cal reload` で読み直します（起動時にも読みます）。

**角速度のバイアス**は電源を入れるたびに少し変わるので、静止している間のデータから取り直します。

- `imu start` の計測中と `sensor stream` の間は、約 1 秒静止するたびに自動で更新します。
  動いている間は更新しません。
- センサーを動かしていないときは `imu cal zero` で取り直せます。ボードを置いて実行すると、
  センサーを 1〜3 秒だけ動かして止めます。計測中は使えません（計測が自分で更新します）。
- 取り直した値は RAM にだけ持ちます。再起動、`imu cal reload`、レンジの変更で、保存してある
  校正値のバイアスに戻ります。
- 保存してある値から 1 dps 以上離れた値は採用せず、`rejected` に数えます。重力と同じ軸まわりに
  ゆっくり回っている場合を、静止と区別できないためです。

### センサーの属性

FIFO の設定は `sensor attr_set` / `attr_get` で扱えます。どちらも次のストリーム開始時に適用され、
ストリーム中は変更できません。`imu start` は watermark を KV の値（既定 128）で上書きします。

```text
sensor attr_set lsm6dsox all fifo_watermark 128
sensor attr_set lsm6dsox all fifo_timestamp_decimation 8
```

`fifo_timestamp_decimation` は FIFO にタイムスタンプを入れる間隔で、0（入れない）、1、8、32 を
指定できます。既定は 8 です。0 にするとバッチに時刻が付きません。

### 速度の測定（zperf）

Zephyr の `zperf` を iperf3 プロトコルで有効にしています。PC 側は `iperf3` を使います。

ボード → PC（送信）。PC で `iperf3 -s -p 5201` を動かしてから実行します。引数は
宛先、ポート、秒数、1 回の送信サイズ、UDP では続けて速度です。

```text
zperf tcp upload <PC の IPv4 アドレス> 5201 30 1024
zperf udp upload <PC の IPv4 アドレス> 5201 30 500 500K
```

PC → ボード（受信）。ボードでサーバーを起動してから、PC で
`iperf3 -c <ボードの IPv4 アドレス> -p 5201 -t 30` を実行します。

```text
zperf tcp download 5201
zperf tcp download stop
```

実機での 30 秒・1 回ずつの測定では、TCP 送信 2.93 Mbit/s、TCP 受信 1.54 Mbit/s、UDP 送信は
500 kbit/s の指定で欠落 0 でした。IMU の送信を止めた状態での値で、無線環境によって変わります。

### Wi-Fi 経由のシェル

Wi-Fi 接続後は TCP ポート 23 で Telnet のシェルを利用できます。同時に使えるクライアントは
1 つです。認証と暗号化がなく、到達できる相手はフルシェルを操作できるので、信頼できる
ネットワークでのみ使ってください。

### KV ストレージ

Zephyr の Settings/NVS に保存します。キーは英数字、`_`、`-`、`.` の 1〜32 文字、値は
1〜128 バイトです。`set` はテキスト、`sethex` は 16 進数のバイト列を保存し、`get` は 16 進数で
表示します。`sys.` で始まるキーは内部用です。データは暗号化されません。Wi-Fi の認証情報も
同じ 8 KiB の領域に保存されます。

### ログ

ログは RAM の 4 KiB のリングバッファに保存し、`dmesg` で表示します。通常のリセットでは
残りますが、電源断では消えます。

例外やスタックの番兵などの致命的なエラーが起きると、エラーの報告をログに出してから自動で再起動します
（`CONFIG_RESET_ON_FATAL_ERROR=y`）。報告は再起動の後に `dmesg` で読めます。

## テスト

```bash
zephyr/script/host_tests.sh
```

FIFO ワードの解析、バッチとペイロード、送信キュー、時刻の対応付け、NTP の推定、ドライバーの
FIFO 設定と停止処理、NINA の SPI フレーミングなどを、PC 上で実行します。ボードは不要です。

実機での確認は、上の「使い方」の手順で受信器を動かし、終了時の集計（欠番、時刻の有効数と
逆行、`imu status` のエラー）を見ます。

## 制限

- TLS には対応していません。MQTT は平文で、認証もありません。
- ビルドは Zephyr のテスト用の非暗号乱数を使います。
- 時刻の誤差は、ボードの推定と PC 時刻の間で数 ms 以内を確認していますが、センサー内部の
  遅れは測定していません。
- Zephyr のカーネル時計（`kernel uptime` など）は、6667 Hz の送信中に約 480 ppm 遅れます。
  時刻の基準には使っていません。

## リポジトリの構成

| 場所 | 内容 |
|---|---|
| `zephyr/app/usb-console/src/` | アプリ本体（計測、MQTT、時刻同期、KV、Wi-Fi 設定） |
| `zephyr/app/usb-console/drivers/` | LSM6DSO、I²C、NINA、sensor shell のローカルドライバー |
| `zephyr/app/usb-console/tests/` | ホストテスト |
| `zephyr/app/blink/` | LED 点滅のサンプル |
| `zephyr/script/` | 環境構築、ホストテスト |
| `nuttx/` | NuttX 版（ボード定義、ドライバー、アプリ、パッチ、スクリプト、ホストテスト） |
| `zephyr/manifest/west.yml` | Zephyr とモジュールのバージョン |
| `design/` | 設計と調査手順 |
| `firmware/nina-fw/` | NINA ファームウェアの修正とビルド |
| `host/` | PC 側のツール（受信の診断、記録、校正）と、表示用のアプリ `ImuStudio` |
| `script/` | PC を一時的な AP にするスクリプト、Zephyr 版から NuttX 版への KV の移し替え（`kv_transfer.py`） |

## NuttX 版

同じファームウェアを NuttX 13.1.0 に移植したもの（NuttX 版）が [`nuttx/`](nuttx/README_ja.md) にあります。
計測、MQTT の送信、時刻同期、KV、校正、Wi-Fi、Telnet のシェルは Zephyr 版と同じ動作で、独自のコマンド
（`imu`、`kv`、`ntp`、`nina`、`dmesg`、`i2c_dma`、`sensor`、`wifi cred`）は構文と出力も同じです。それ以外の
コマンドは NuttX 標準のもの（`wapi`、`ifconfig`、`iperf`、`ps`、`reboot` など）に置き換わります。

- 環境構築、ビルド、書き込み、使い方は [`nuttx/README_ja.md`](nuttx/README_ja.md) を見てください。Zephyr 版の
  コマンドと NuttX 版のコマンドの対応表、エラー番号（errno）の対応表、Zephyr 版との違いもそこにあります。
- PC 側のツール（`host/` の `imu-host`、`ImuStudio`）は、どちらのファームウェアにも使えます
  （[`host/README_ja.md`](host/README_ja.md) の「ボードのファームウェア」）。
- KV の保存形式が違うので、Zephyr 版の設定と校正値は `script/kv_transfer.py` で退避して書き戻します
  （`nuttx/README_ja.md` の「Zephyr 版からの移し替え」）。
- 設計上の違いは [設計書](design/imu-raw-mqtt-design-ja.md) の §11 にあります。

## Zephyr 版と NuttX 版の性能の違い

同じボード、同じ PC、同じ AP で、2 つのファームウェアを書き換えて同じ日（2026-11-14）に測った値です。
条件は 6667 Hz の計測を MQTT で送信、PC 側は `imu-host check`、1 回 60 秒で、どちらの版も同じ手順
（USB のシェルから `imu start`、60 秒後に `imu stop`、`imu status`）で測りました。どの回も欠番、
`imu status` の `errors`・`fifo` は 0 でした。

| 項目 | Zephyr 版 | NuttX 版 |
|---|---|---|
| 送信中の `cpu_active` | 62.6 %、63.3 %（2 回） | 67.7〜67.9 %（3 回） |
| 同、`ntp status` を 2 秒ごとに実行しながら | 65.4 %（1 回） | 68.3 %、68.5 %（2 回） |
| 送信中の NTP の往復時間（`ntp status` の `last_us` の中央値） | 7.6 ms（27 点） | 9.9 ms、10.0 ms（30 点ずつ、2 回） |
| TCP の送信（ボード → PC、10 秒） | 3.00 Mbit/s（`zperf`、1024 バイトずつ、1 回） | 3.6〜3.9 Mbit/s（`iperf`、5 回） |
| TCP の受信（PC → ボード、10 秒） | 1.57 Mbit/s（`zperf`、1 回） | 0.95〜1.03 Mbit/s（`iperf`、5 回） |

- NuttX 版は、送信中の CPU 負荷が Zephyr 版より約 5 ポイント高く、NTP の往復が約 2 ms 長くなります。
  差の原因は特定していません。
- NuttX 版の値は `ps` に CPU の列を入れた版のものです。入れる前の版は、同じ手順で `cpu_active` 66.0〜66.1 %
  （3 回）、`ntp status` を見ながら 66.7 %（1 回）、NTP の往復の中央値 8.6 ms（30 点）でした。
- TCP の速度は測定のコマンドが違うので（Zephyr 版は `zperf`＝iperf 3 の方式、NuttX 版は `iperf`＝iperf 2 の
  方式で、1 回に送る大きさも違います）、目安としての比較です。
- `sensor stream` は、どちらの版でも ODR を上げると `Read failed` で止まります。Zephyr 版は 833 Hz 以上、
  NuttX 版は 3333 Hz 以上で止まりました（watermark 128、USB コンソール、各 1 回）。

### 送信中のスレッドごとの CPU 時間

送信中（`imu start` の 8 秒後から 30 秒間）に、各スレッドの実行時間が全スレッドの合計に占める割合です。
Zephyr 版は `kernel thread list` の `Total execution cycles`、NuttX 版は `/proc/<tid>/critmon` の実行時間の
合計を、区間の前後にシェルから読んだ差です。どちらもスレッドの切り替えのときに積算し、割り込みの処理の
時間は割り込まれたスレッドに入ります（アイドル中の割り込みはアイドルに入るので、アイドルの割合は
100 % − `cpu_active` と一致しません）。Zephyr 版は 1 回、NuttX 版は 2 回です。この計測の `cpu_active`
（`imu start` から `imu stop` まで、区間の前後に値を読むコマンドを含む）は Zephyr 版 63.5 %、NuttX 版
68.8 %・68.3 % でした。

| スレッド | Zephyr 版 | NuttX 版 |
|---|---|---|
| アイドル | 41.85 % | 34.58 %、35.17 % |
| `imu_raw_mqtt_tid` | 20.39 % | 22.83 %、22.28 % |
| `imu_fifo_worker` | 18.53 % | 19.09 %、18.82 % |
| `imu_stream_tid` | 16.18 % | 17.40 %、17.73 % |
| `nina_event`（NuttX 版だけのスレッド。NINA のソケットの状態の問い合わせ、送信の再試行） | – | 3.24 %、3.21 % |
| `sysworkq` | 1.85 % | 2.24 %、2.16 % |
| USB（Zephyr 版の `usbd@50110000` と `usbd`。NuttX 版には USB のスレッドが無い） | 0.63 % | – |
| シェル（Zephyr 版の `shell_uart`、NuttX 版の `nsh`） | 0.38 % | 0.36 %、0.35 % |
| `time_sync_tid` | 0.17 % | 0.24 %、0.27 % |
| `nina_monitor_tid` | 0.02 % | 0.02 %、0.02 % |

### メモリの使い方

2026-11-14 のビルドの ELF から、セクションの大きさとシンボルのアドレスを比べたものです。flash（16 MB）と
RAM（264 KB）の割り当て、boot2（0x10000000 の 256 バイト）、コードの開始（0x10000100）、KV の場所（flash の
末尾 8 KB、0x10ffe000 から）は同じです。

| flash | Zephyr 版 | NuttX 版 |
|---|---|---|
| コード | 266,776 バイト | 274,968 バイト（定数を含む） |
| 定数 | 85,808 バイト | コードに含む |
| `.data` の初期値 | 5,232 バイト | 3,716 バイト |
| イメージの合計 | 362,284 バイト | 約 279,000 バイト |
| UF2 のファイル | 725,504 バイト | 558,080 バイト |

| RAM | Zephyr 版 | NuttX 版 |
|---|---|---|
| `.data` と初期値つきのカーネルオブジェクト | 5,232 バイト | 3,716 バイト（ほかに RAM 上のベクターテーブル 192 バイト） |
| `.bss` | 94,637 バイト | 109,436 バイト |
| `.noinit` | 55,712 バイト（全スレッドのスタックと RAM ログ 4 KiB） | 8,208 バイト（RAM ログ 8 KiB） |
| 静的に確保した分の合計 | 155,584 バイト | 121,564 バイト |
| その後ろ | 114,752 バイト（libc の malloc が使える領域） | アイドルのスタック 1,024 バイトとヒープ約 147.7 KB |
| 割り込み用のスタック | 2,048 バイト | 2,048 バイト |

- Zephyr 版は全スレッドのスタックを静的に確保します。NuttX 版は、Zephyr 版から写したスレッド
  （`imu_stream_tid` など）のスタックは `.bss` に置き、NuttX 側で作るタスク（`nsh`、`telnetd`、`nina_event`
  など）のスタックはヒープから取ります。NuttX 版のヒープは、計測していないときに約 23.8 KB を使っています
  （`free` の表示）。
- アプリの大きな変数は両方で同じ大きさです（送信キュー 29,512 バイト、FIFO 読み出しのブロックプール
  8,192 バイト、NINA の送信バッファ 3,584 バイトなど）。違うのは RAM ログ（NuttX 版は 8 KiB）、シェルの
  コマンド履歴（NuttX 版に 4,108 バイト）などです。
