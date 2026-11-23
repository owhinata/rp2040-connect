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
参照してください。同じファームウェアの NuttX 版（`nuttx/`）と Pico SDK + FreeRTOS 版（`freertos/`）もあります
（下の「3 つの OS のファームウェアと `common/`」）。

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
Python パッケージ、Zephyr SDK を用意し、Zephyr とモジュールにこのリポジトリのローカルのパッチ
（`zephyr/manifest/zephyr/patches.yml`）を当て、現在のシェルに `ZEPHYR_SDK_INSTALL_DIR` と `west` の
エイリアスを設定します。新しいターミナルでは再度読み込んでください。パッチの内容と戻し方は
[ドライバーの説明](zephyr/app/usb-console/drivers/README_ja.md) の「Zephyr へのローカルのパッチ」にあります。

## ビルドと書き込み

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
./script/flash.sh            # build/usb-console/zephyr/zephyr.uf2 を書き込む（--build で先にビルド）
```

構成は `zephyr/app/usb-console/prj.conf` の 1 つだけです。`memcpy`・`memset` はブート ROM のルーチン
（`CONFIG_RP2040_ROM_MEM_OPS`）、整数の割り算はハードウェアの除算器（`CONFIG_RP2040_PICO_DIVIDER`）を使い、
`nina passthrough confirm` は `CONFIG_NINA_PASSTHROUGH` で入ります（どれも既定で有効）。

UF2 はブートローダーモード（BOOTSEL。`RPI-RP2` という USB ストレージが現れる状態）から書きます。
`flash.sh`（NuttX 版・FreeRTOS 版と共通の `script/flash_uf2.sh`）は、このストレージに UF2 をコピーします。
ストレージが無ければ、3 つの OS のファームウェアのどれが動いていても、その USB のシェルに `bootsel` を送り（コンソールを別の
プログラムで開いているときは送りません）、ストレージが現れてマウントされるのを待ち、コピーのあと新しい USB
コンソールを表示します。シェルが応答しないときは、BOOTSEL への入り方を表示して待ちます。RESET ボタンを素早く
2 回押すか、それでも入れない場合は、`REC` と `GND` をジャンパー線で接続したまま RESET ボタンを押し、
ストレージが現れたらジャンパー線を外します。`--dry-run` は何をするかを表示するだけです。

`zephyr/app/blink` は `led0` を 500 ms ごとに切り替える最小のサンプルです。

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

NINA-W102 側は、Arduino の NINA ファームウェア 3.0.2 にこのリポジトリのパッチ（0001〜0006）を当てたものを
前提にしています。0003〜0006 の追加機能（接続中のスキャンで接続を保つ、ソケットの事象の通知など）が無い
ファームウェアでも、ドライバーはその機能を使わずに動きます。ビルドと注意点は [NINA ファームウェアの説明](firmware/nina-fw/README.md)
を参照してください。`nina version` の 1 行目がバージョン、2 行目（`NINA capabilities: 0x… (in use 0x…)`）が
モジュールの追加機能のビットと使っているビットです。

### NINA のファームウェアの読み書き

`nina passthrough confirm` でボードが再起動し、USB のシリアルが NINA の UART に繋がります（3 つの OS のファームウェアとも。
RP2040 のファームウェアと KV はそのまま）。PC の esptool で読み書きします（下の例はリポジトリのルートで実行）。esptool 5.3.0 は `--before no-reset` では
繋がらないので、RTS を有効にしたままにする設定ファイルを使います。

```bash
printf '[esptool]\ncustom_reset_sequence = R0|D1|W0.2|R1|W0.6\n' > esptool.cfg
export ESPTOOL_CFGFILE=$PWD/esptool.cfg
esptool --port <ポート> --baud 921600 --before default-reset --after no-reset \
  read-flash 0 0x200000 firmware/nina-fw/images/backup.bin      # 退避（約 26 秒）
esptool --port <ポート> --baud 921600 --before default-reset --after no-reset \
  write-flash 0 firmware/nina-fw/source/NINA_W102.bin           # 書き込み（約 20 秒）
stty -F <ポート> 300                                             # 中継を抜ける
```

ポートを 300 bps にして閉じるか RESET で、通常の起動に戻ります。退避したファイルは認証情報を含みうるので
公開しないでください（`firmware/nina-fw/images/` は Git の管理外）。Arduino の SerialNINAPassthrough スケッチは
要りません。

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
- `wifi scan` は、NINA のファームウェアにパッチ 0004 があれば接続を保ちます。無ければ接続を一時的に切り、
  終わると保存済みの設定で接続し直します。

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
| `nina version` | NINA ファームウェアのバージョンと追加機能のビット |
| `nina passthrough confirm` | USB のシリアルを NINA の UART に中継するモードで再起動（esptool 用） |
| `bootsel` | BOOTSEL（`RPI-RP2`）で再起動 |
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

UDP 送信は 500 kbit/s の指定で欠落 0 でした（2026-11-12、30 秒、1 回）。TCP の速度は最後の節
「3 つの OS のファームウェアの性能」にあります。どちらも IMU の送信を止めた状態での値で、無線環境によって変わります。

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
NuttX 版・FreeRTOS 版と共有するソースと、Zephyr の API の互換層・シェルのサブコマンドの処理・RAM ログのリング・KV の
OS に依存しない部分のテスト（`common/tests/run.sh`）を先に実行します。

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
| `zephyr/app/usb-console/src/` | Zephyr 版だけのアプリのソース（KV、RAM ログ、起動処理） |
| `zephyr/app/usb-console/drivers/` | Zephyr 版のドライバーの説明（I²C、SPI（PL022）、sensor シェルは Zephyr のものにローカルのパッチ、LSM6DSO と NINA のドライバーは `common/drivers/`） |
| `zephyr/app/usb-console/tests/` | Zephyr 版だけのホストテスト |
| `zephyr/app/blink/` | LED 点滅のサンプル |
| `zephyr/script/` | 環境構築、ホストテスト |
| `zephyr/manifest/west.yml` | Zephyr とモジュールのバージョン |
| `zephyr/manifest/zephyr/` | Zephyr とモジュールへのローカルのパッチ |
| `common/` | 3 つの OS のファームウェアが共有するソースとそのホストテスト（構成は[設計書](design/imu-raw-mqtt-design-ja.md)の §12） |
| `nuttx/` | NuttX 版 |
| `freertos/` | Pico SDK + FreeRTOS 版 |
| `openmv/` | OpenMV 版 MicroPython のビルド環境（[`openmv/README_ja.md`](openmv/README_ja.md)） |
| `design/` | 設計と調査手順 |
| `firmware/nina-fw/` | NINA ファームウェアの修正とビルド |
| `host/` | PC 側のツールと表示用のアプリ `ImuStudio` |
| `script/` | PC を一時的な AP にするスクリプト、KV の退避と書き戻し、3 つの OS のファームウェアが共有する UF2 の書き込み |

## 3 つの OS のファームウェアと `common/`

同じファームウェアに、Zephyr 版（`zephyr/`）、NuttX 版（`nuttx/`、NuttX 13.1.0）、Pico SDK + FreeRTOS 版（`freertos/`、
Pico SDK 2.3.1 と FreeRTOS カーネル V11.3.3）の 3 つがあります。どれも、同じファームウェアをそれぞれの OS の上でビルドしたものです。ソースの大半は `common/` の同じファイルで、NuttX 版と
FreeRTOS 版は Zephyr の API を Zephyr の名前のまま再現する互換層の上でそれをビルドします。計測、MQTT の送信、時刻同期、
KV（同じ場所・同じ形式なので、別の OS のファームウェアに書き換えても残ります）、校正、Wi-Fi、Telnet のシェルの動作と、独自のコマンド（`imu`、
`kv`、`ntp`、`nina`、`dmesg`、`i2c_dma`、`sensor`、`wifi cred`）の構文と出力は 3 つの OS のファームウェアで同じです。

| ファームウェア | 使い方 | 使う側から見た違い |
|---|---|---|
| Zephyr 版 | この README | それ以外のコマンドは Zephyr 標準（`wifi`、`net`、`kernel`、`zperf` など） |
| NuttX 版 | [`nuttx/README_ja.md`](nuttx/README_ja.md) | それ以外のコマンドは NuttX 標準（`wapi`、`ifconfig`、`iperf`、`ps`、`reboot` など） |
| FreeRTOS 版 | [`freertos/README_ja.md`](freertos/README_ja.md) | それ以外のコマンドは短い名前（`version`、`reboot`、`ps`、`ifconfig` など）、`zperf` は TCP だけ |

PC 側のツールはどの OS のファームウェアにも使えます（[`host/README_ja.md`](host/README_ja.md) の「ボードのファームウェア」）。`common/` の構成は
[設計書](design/imu-raw-mqtt-design-ja.md)の §12、NuttX 版と FreeRTOS 版の設計上の違いは §11 と §13 にあります。別の OS へ移植するときに
その OS の側で書くもの（カーネルの部分、コンソール、ログ、KV、起動、各ドライバーのつなぎ）は、
[共有のコードと OS の境界](design/os-boundary-ja.md)にまとめています。

## 3 つの OS のファームウェアの性能

同じボード、同じ PC、同じ AP、同じ受信器（`imu-host check`）で、ファームウェアを書き換えて同じ日（2026-11-18）に
測った値です。送信中の値は、6667 Hz の計測を MQTT で送り、USB のシェルから `imu start`、約 60〜70 秒後に `imu stop` で
測りました。どの回も欠番は 0 でした。TCP の速度は IMU の送信を止めた状態で、約 10 秒、ボードからの送信は 1024 バイトずつです。
NINA は 0001〜0006 のファームウェアです。

| 項目 | Zephyr 版 | NuttX 版 | FreeRTOS 版 |
|---|---|---|---|
| 送信中の `cpu_active`（2026-11-23） | 21.9 %（2 回） | 23.5 %（2 回） | 19.6 %（2 回） |
| 静止時の `cpu_active` | 測っていない | 測っていない | 0.7 %（30 秒、3 回） |
| TCP の送信（ボード → PC） | 3.36、3.47 Mbit/s（`zperf`） | 3.34 Mbit/s（`iperf`、1 回） | 3.47〜3.84 Mbit/s（`zperf`、3 回） |
| TCP の受信（PC → ボード） | 1.89、1.99 Mbit/s（`zperf`） | 1.22 Mbit/s（`iperf`、1 回） | 2.41〜2.52 Mbit/s（`zperf`、3 回） |
| イメージの大きさ（`.bin`、2026-11-23） | 378,392 バイト | 287,224 バイト | 240,968 バイト |

- TCP の速度は測定のコマンドが違うので（Zephyr 版と FreeRTOS 版は `zperf`＝iperf 3 の方式、NuttX 版は `iperf`＝
  iperf 2 の方式）、目安としての比較です。
- 送信中の `cpu_active` とイメージの大きさは 2026-11-23 のイメージの値です（NINA の SPI の 64 バイト以上の転送を DMA にし、
  よく通るコードを RAM に置いたあと）。
- 各 OS のファームウェアは、よく通るコード（センサーの読み出しと送信の経路。Zephyr 版 約 56 kB、NuttX 版 約 46 kB、FreeRTOS 版
  約 49 kB）を RAM に置いています。フラッシュから実行するコードのキャッシュ（16 kB）の外れが減り、`cpu_active` は Zephyr 版で
  35.8 % から 21.4 %、NuttX 版で 36.1 % から 23.8 %、FreeRTOS 版で 33.1 % から 19.7 % になりました（外れは Zephyr 版で約 39 万回/秒
  から約 5 千回/秒、FreeRTOS 版で約 34 万回/秒から約 6 千回/秒）。
- コードを RAM に置く前のイメージで DMA を止めると、Zephyr 版 44.1 %、NuttX 版 47.4 %、FreeRTOS 版 42.4 % でした。2026-11-18 の
  イメージ（すべてポーリング）では 49.4 %、50.8 %、40.3 % でした。
- `cpu_active` は、機能が同じでもイメージの配置が変わるだけで動きます（コードをフラッシュから実行していたときは 1〜2 ポイント、
  いまは 0.5 ポイントほど）。
- FreeRTOS 版の `cpu_active` には、1 ms ごとの tick の割り込みの時間が入ります（Zephyr 版と NuttX 版にはこの割り込みが
  ありません）。
- 2026-11-14 のイメージ（boot2 の分周 4、C ライブラリの `memcpy`・`memset` と割り算、NINA はパッチ 0001・0002 のファームウェア）では、
  送信中の `cpu_active` が Zephyr 版 62.6〜63.3 %、NuttX 版 67.7〜67.9 % でした。
- `sensor stream` は、Zephyr 版と NuttX 版のどちらでも ODR を上げると `Read failed` で止まりました。Zephyr 版は
  833 Hz 以上、NuttX 版は 3333 Hz 以上です（2026-11-14 のイメージ、watermark 128、USB コンソール、各 1 回）。
