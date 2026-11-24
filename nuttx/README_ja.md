# NuttX 版

[English](README.md) | 日本語

`zephyr/app/usb-console` の IMU 測定ファームウェア（Zephyr 版）を NuttX 13.1.0 に移植したもの（NuttX 版）です。
対象は Zephyr 版と同じ Arduino Nano RP2040 Connect（RP2040、NINA-W102、LSM6DSOX）です。LSM6DSOX の
加速度・角速度を 6667 Hz の RAW のまま取得し、PC の時刻を付けて Wi-Fi 経由の MQTT で送ります。

- 独自のコマンド（`imu`、`kv`、`ntp`、`nina`、`dmesg`、`i2c_dma`、`sensor`、`wifi cred`）は Zephyr 版と構文・出力が
  同じです。Zephyr 標準のコマンドは NuttX 標準のコマンドに置き換わります（「Zephyr 版とのコマンドの対応」）。
- エラー番号（errno）は NuttX の値のまま表示します（「エラー番号（errno）」）。
- PC 側のツール（`host/` の `imu-host`、`ImuStudio`）はそのまま使えます（`host/README_ja.md` の
  「ボードのファームウェア」）。

詳しい資料:

- 各機能の動作の詳しい説明: [`design/nuttx-porting-ja.md`](../design/nuttx-porting-ja.md)
- ドライバー（NINA、I2C0、LSM6DSOX）と Zephyr の API の互換層: [`drivers/README_ja.md`](drivers/README_ja.md)
- 設計の要約: 設計書 [§11](../design/imu-raw-mqtt-design-ja.md)（「11. NuttX 版」）
- Zephyr 版との性能の比較: ルートの [`README_ja.md`](../README_ja.md) の「4 つの OS のファームウェアの性能」
- 移植の計画・経緯・確かめた事実（作業ログ）: [`PORTING_PLAN.md`](PORTING_PLAN.md)、
  [`PORTING_STATUS.md`](PORTING_STATUS.md)、[`PORTING_NOTES.md`](PORTING_NOTES.md)

## 必要なもの

- Arduino Nano RP2040 Connect（NINA のファームウェアは Zephyr 版と同じもの。`firmware/nina-fw/README.md`）
- Linux の PC（x86_64）。sudo は要りません。
- `git`、`curl` または `wget`、`tar`、`xz`、`sha256sum`
- `python3`（`venv` モジュールを含む。Ubuntu では `python3-venv`）
- `make`、`gcc`（NuttX がホスト用のツールをビルドするため）

## 環境構築

リポジトリのルートで読み込みます。足りないものだけを `nuttx/` の下に取得するので、何度読み込んでも
かまいません。新しいターミナルでは読み込み直します。

```bash
source nuttx/script/setup.sh
```

| 置き場所 | 内容 | バージョン |
|---|---|---|
| `nuttx/nuttx` | NuttX 本体 | タグ `nuttx-13.1.0`（shallow clone） |
| `nuttx/nuttx-apps` | NuttX のアプリ | タグ `nuttx-13.1.0`（shallow clone） |
| `nuttx/pico-sdk` | Raspberry Pi Pico SDK のうち boot2 と除算器のルーチン（`divider_hardware.S`）のビルドが読む部分だけ（約 4 MB）。全体を取得済みならそのまま使う。以前の部分取得に足りないファイルは足す（そのときネットワークが要る） | タグ `2.2.0`（部分取得と sparse checkout） |
| `nuttx/toolchain/` | Arm GNU Toolchain（arm-none-eabi、x86_64 Linux 用） | 13.2.Rel1（SHA-256 を検証） |
| `nuttx/.venv` | Python の venv（NuttX の `menuconfig`・`olddefconfig` が使う kconfiglib） | kconfiglib 14.1.0 |
| `nuttx/stmemsc/` | ST の LSM6DSO レジスタドライバー（`lsm6dso_reg.c`、`lsm6dso_reg.h`） | lsm6dso-pid v3.2.2（Zephyr の hal_st と同じ内容。SHA-256 を検証） |
| `nuttx/nuttx-apps/system/argtable3/argtable3` | argtable3（`iperf` の引数の解析） | タグ `v3.2.0.7402e6e`（shallow clone） |

あわせて、`nuttx/nuttx/external` → `nuttx/drivers` と `nuttx/nuttx-apps/external` → `nuttx/apps` の symlink を張り、
下のパッチを当て、現在のシェルに `PICO_SDK_PATH` と `PATH`（ツールチェーン、`nuttx/.venv/bin`、`nuttx/script`）を
設定します。バージョンはスクリプトの冒頭で固定しています。取得したものは Git の管理外です。パッチの記録や
クローンに変更があるときの扱いは [`design/nuttx-porting-ja.md`](../design/nuttx-porting-ja.md) の「環境構築（`setup.sh`）の動作」です。

### NuttX へのパッチ

`nuttx/patches/nuttx/` は NuttX 本体、`nuttx/patches/apps/` は nuttx-apps へのパッチです。各ファイルの先頭に
目的と対応する Zephyr の箇所を書いてあります。

| パッチ | 内容 |
|---|---|
| `nuttx/0001-cdcacm-no-at-protocol` | CDC ACM のプロトコルを 0 にする（ModemManager がモデムとして扱わない。Zephyr 版と同じ） |
| `nuttx/0002-cdcacm-drop-output-without-dtr` | PC がポートを開いていない間（DTR なし）の出力を捨てる |
| `nuttx/0003-serial-canonical-ctrl-c-eof` | 標準入力を読むコマンドを Ctrl-C・Ctrl-D で終わらせる |
| `nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom` | システム時計とアイドルループをボード側で持つため、in-tree の `rp2040_timerisr.c`・`rp2040_idle.c` を外す |
| `nuttx/0005-usrsock-dontwait-to-daemon-and-daemon-send-timeout` | usrsock の待たない送受信と送信の期限を Zephyr 版の NINA のソケットと同じにする |
| `nuttx/0006-telnet-crlf-line-end` | Telnet の出力の改行を CR LF にする |
| `nuttx/0007-sched-critmon-cpuload-in-perf-counts` | `ps` の CPU 列の積算を 1 µs 単位で行い、終了したタスクで値が壊れるのを直す |
| `nuttx/0008-sched-critmon-irqcount-only-for-csection-busywait` | `SCHED_CRITMONITOR` が `IRQCOUNT` を選ぶのを、使うときだけにする |
| `nuttx/0009-armv6m-rp2040-save-hw-divider-on-context-switch` | タスクの切り替えで RP2040 の除算器を退避・復元する（`RP2040_HW_DIVIDER`） |
| `nuttx/0010-rp2040-rom-memcpy-memset` | `memcpy`・`memset` をブート ROM のルーチンにする（`RP2040_ROM_MEM_OPS`） |
| `nuttx/0011-rp2040-dmachannel-trywait` | 待たずに DMA のチャネルを取る関数（`rp2040_dmachannel_trywait()`）。I2C0 の読み出し用 |
| `nuttx/0012-rp2040-dmac-ints0-own-bit` | DMA のチャネルの確保と解放で、自分のチャネルの割り込みの印だけを消す（ほかのチャネルの保留中の印も消していた） |
| `nuttx/0013-rp2040-spi-dma-channels-trywait` | SPI のドライバーが DMA のチャネルを待たずに取る。2 本取れなければ、その SPI はポーリングで動く（NINA の SPI1 用） |
| `apps/0001-readline-echo-in-readline` | 行の入力中のエコーを readline が行う |
| `apps/0002-readline-cr-lf-line-end` | CR、LF、CR LF、LF CR のどれでも 1 回の Enter にする |
| `apps/0003-readline-ctrl-c` | Ctrl-C で入力中の行を捨てて新しいプロンプトを出す |
| `apps/0004-readline-full-line` | 行が満杯になったら後の文字を捨てる |
| `apps/0005-iperf-free-argtable` | `iperf` が実行のたびにヒープを失うのを直す |
| `apps/0006-iperf-len-option-udp-eagain` | `iperf` に `-l` を足し、UDP の送信の `EAGAIN` で待って送り直す |

## ビルド

```bash
nuttx/script/build.sh
```

ボード `nuttx/boards/arduino-nano-rp2040-connect/` を唯一の構成 `configs/imu/defconfig` でビルドし、
`nuttx/build/nuttx.uf2` を作ります。`setup.sh` は自分で読み込みます。defconfig・Kconfig・パッチなどが
変わったときは自動で configure し直し、イメージ（boot2、flash の範囲、KV の領域との重なり）を確かめてから
UF2 にします（[`design/nuttx-porting-ja.md`](../design/nuttx-porting-ja.md) の「ビルド（`build.sh`）の動作」）。

| オプション | 動作 |
|---|---|
| `--clean` | `make distclean` から configure し直す |
| `--menuconfig` | 必要なら configure し、`make menuconfig` を実行する（ビルドはしない） |
| `--savedefconfig` | `make savedefconfig` の結果を `configs/imu/defconfig` に書き戻す（ビルドはしない） |
| `-j N` | make の並列数（既定は CPU の数） |
| `-v` | make の出力を詳しくする（`V=1`） |

`--menuconfig` で変えた内容は、`--savedefconfig` で保存しないと次の configure で消えます。

次のオプションは既定で有効です。

| オプション | 内容 |
|---|---|
| `CONFIG_RP2040_ROM_MEM_OPS` | `memcpy`・`memset` をブート ROM のルーチンにする（パッチ 0010。無効にすると C ライブラリのループ）。`build.sh` は `__start()` がそれらを呼ばないことを確かめる |
| `CONFIG_RP2040_PICO_DIVIDER` | 整数の割り算を RP2040 のハードウェアの除算器で行う（Pico SDK のルーチン、パッチ 0009。無効にすると libgcc）。`build.sh` は libgcc の割り算がイメージに残っていないことを確かめる |
| `CONFIG_NINA_PASSTHROUGH` | `nina passthrough confirm`（下の「そのほかのコマンド」） |

## 書き込み

UF2 は RP2040 の USB ブートローダー（BOOTSEL。`RPI-RP2` という USB ストレージが現れる状態）から書きます。

```bash
nuttx/script/flash.sh            # nuttx/build/nuttx.uf2 を書き込む
nuttx/script/flash.sh --build    # ビルドしてから書き込む
```

`flash.sh`（Zephyr 版・FreeRTOS 版と共通の `script/flash_uf2.sh`）は、UF2 が RP2040 用であることを確かめ、
`RPI-RP2` のストレージ（RP2040 のブート ROM のもの）を探します。無ければ、4 つの OS のファームウェアのどれが動いていても、その
USB のシェルに `bootsel` を送り（コンソールを別のプログラムで開いているときは送りません）、ストレージが現れるのを
待ちます。シェルが応答しないときは、BOOTSEL に手で入れる方法（RESET ボタンを素早く 2 回押す。入らない場合は、
`REC` と `GND` をジャンパー線で接続したまま RESET ボタンを押し、ストレージが現れたら線を外す）を表示して
待ちます。ストレージがマウントされていなければ `udisksctl` でマウントし（sudo 不要）、`INFO_UF2.TXT` を
確かめてコピーし、NuttX 版の USB コンソールが現れるのを待ってそのパスを表示します。ストレージが複数あるときは
止まります。

| 引数・オプション | 動作 |
|---|---|
| `UF2` | 書き込む UF2（既定は `nuttx/build/nuttx.uf2`） |
| `--build` | 先に `nuttx/script/build.sh` を実行する |
| `--wait 秒` | ストレージを待つ時間と、そのあとコンソールを待つ時間（既定は 60） |
| `--dry-run` | ストレージとコンソールを探して（`lsblk` と `/dev/serial/by-id/` だけ）、何をするかを表示する |

## Zephyr 版からの移し替え

KV（校正値、`imu.*` の設定、Wi-Fi の資格情報）は、Zephyr 版と同じ場所（フラッシュ末尾 8 KB、`0xffe000`〜
`0xffffff`）に Zephyr 版と同じ形式（Zephyr の NVS と settings）で置きます。Zephyr 版と NuttX 版の間で
書き換えても KV はそのまま残り、どの OS のファームウェアが書いた値もそのまま読めます。移し替えの作業はありません。
FreeRTOS 版（`freertos/`）も同じ場所・同じ形式で、NuttX 版が書いた KV をそのまま読めることを確かめています。

KV の領域に、Zephyr の NVS でも消去済みでもないデータ（NuttX 版の以前の形式、OpenMV 版などの別の
ファームウェアの残り）があると、NuttX 版はそれを消さずに残し、KV を使いません。`dmesg` に次の行が出て、
`kv` のコマンドや資格情報の操作はエラーになります（`kv get` は `KV read failed: -84`）。

```text
<err> flash_kv: Key-value area 0xffe000-0xffffff holds the NuttX build's earlier format
<err> flash_kv: Not used; "kv wipe confirm" erases it (all keys and Wi-Fi credentials)
```

この場合は `kv wipe confirm` で領域を消してから、値を登録し直します（`confirm` を付けないと消しません）。

```bash
# 値を読めるファームウェアが動いている間に退避しておいた場合は（host/recordings/kv-backup.json、Git 管理外）
nsh$ kv wipe confirm
Erased
# 書き戻し、Wi-Fi を登録する（パスワードを端末で入力。SSID の既定は rp2040-test、別の AP なら --ssid）
host/.venv/bin/python script/kv_transfer.py restore --usb
```

`script/kv_transfer.py` は、KV の領域が失われる場面（別のファームウェアを書くとき、`kv wipe confirm` の前）に
シェルの `kv get` で値を退避し（`backup`）、あとで `kv sethex` で書き戻す（`restore`）ためのものです。
`restore` は `kv get` で読み返し、`wifi cred add`・`wifi cred auto_connect` と `imu cal reload` を送ります。
`imu.session` は書き戻しません。資格情報がすでに 2 件あると `wifi cred add` が `err: -105` で失敗するので、
先に `wifi cred delete` で消します。

### 以前の NuttX 版の KV が入っているボード

以前の NuttX 版は、KV を NuttX の NVS の形式で持っていました。その KV が入っているボードに今の NuttX 版を
書くと、上の「別のデータ」として扱われます。次の順に移します。

1. 以前の NuttX 版が動いている間に `script/kv_transfer.py backup --usb` で退避する（すでに退避ファイルが
   あれば不要）。
2. 今の NuttX 版を書き込み、`kv wipe confirm` で領域を消す。
3. `script/kv_transfer.py restore --usb` で書き戻し、Wi-Fi の資格情報を登録する（パスワードの入力が要る）。

## 使い方

### シェルへの接続

USB のポートは `/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_<チップ ID>-if00` です（チップ ID は Zephyr 版の
USB のシリアル番号と同じ）。

```bash
picocom /dev/serial/by-id/usb-NuttX_NuttX_USB_Console_*-if00
```

- シェルは nsh で、プロンプトは `nsh$ ` です。開いた直後は何も表示されないので、Enter（または Ctrl-C）を
  押すとプロンプトが出ます。
- `help` でコマンドの一覧、`help <コマンド>` で使い方が出ます。上下の矢印で直前の 8 行を呼び出せ、Tab は
  コマンド名を補完します。Ctrl-C は入力中の行を捨てます。
- 引数は nsh が解釈するので、`$`・`;`・`#`・`>`・`&`・引用符を含む引数は引用符で囲みます。
- この移植のコマンドは、エラーを標準エラー（同じ端末）に出し、終了状態は成功で 0、失敗で 1 です（`echo $?`）。
- `reboot` で再起動します（Zephyr 版の `kernel reboot`。引数を見ずに再起動します）。
- 異常停止（例外、`PANIC()`、assert の失敗、スタックの番兵）のときは、報告を RAM ログに書いてから自動で
  再起動します（`CONFIG_BOARD_RESET_ON_ASSERT=2`）。再起動後の `dmesg` で読めます。

### Wi-Fi

資格情報を保存すると、起動時と切断時に自動で接続します（Zephyr 版と同じ `wifi cred`。2 件まで）。

```text
nsh$ wifi cred add -s <SSID> -k 1 -p <パスフレーズ>
nsh$ wifi cred auto_connect
nsh$ ifconfig wlan0
wlan0	Link encap:Ethernet HWaddr <MAC> at RUNNING mtu 1500
	inet addr:10.42.0.97 DRaddr:10.42.0.1 Mask:255.255.255.0
```

- `wifi cred add` は成功すると何も出しません。`wifi cred list` は一覧（パスフレーズも表示。Zephyr 版と同じ）、
  `wifi cred delete <SSID>` は削除です。接続できる種類は暗号化なし（`-k 0`）と WPA2-PSK（`-k 1`）だけです。
- 接続すると `ifconfig wlan0` に `RUNNING` が付き、NINA の DHCP のアドレスが出ます。`wapi show wlan0` で ESSID・
  BSSID・信号の強さ（Sense）を見られます。
- 一時的な接続・切断・スキャンは NuttX 標準の `wapi` です（「Zephyr 版とのコマンドの対応」）。`wapi` は失敗しても
  何も表示しないので、`echo $?` で確かめます。`wapi disconnect wlan0` の後は、次に接続するまで自動再接続
  しません（Zephyr 版と同じ）。

### Telnet

Wi-Fi に繋がっている間は、ポート 23 で USB と同じシェルを使えます。同時に 1 接続です。認証と暗号化が
ないので、信頼できるネットワークでだけ使ってください。

```bash
telnet <ボードの IPv4 アドレス>
```

接続した直後にプロンプトが届きます。2 本目の接続は受け付けてすぐ閉じます
（`<err> shell_telnet: Telnet client already connected.`）。

### 計測

設定は Zephyr 版と同じ KV のキー（`imu.host`、`imu.port`、`imu.device`、`imu.odr`、`imu.watermark`）から読みます。
`imu.host` が無ければデフォルトゲートウェイへ送ります。

```text
nsh$ kv get imu.port
3138383330
nsh$ imu start
Streaming 6667 Hz to the default gateway, port 18830, as imu/rp2040/raw
nsh$ imu stop
nsh$ imu status
state: stopped
  ...
errors:
  stream:                 0
  ...
```

- `imu status` の項目と並びは Zephyr 版と同じです（`errors`、`fifo`、`time`、`mqtt`）。PC 側の受信と
  NTP サーバーの用意はルートの README の「使い方」と同じです（受信の確認は
  `host/.venv/bin/imu-host check --port <ポート>`）。
- `kv set <キー> <文字列>`、`kv sethex <キー> <16 進数>`、`kv get <キー>`、`kv delete <キー>` の構文と出力は
  Zephyr 版と同じです（`kv get` は 16 進で表示）。`kv wipe confirm` は KV の領域全体（すべてのキーと Wi-Fi の
  資格情報）を消します（Zephyr 版には無い。上の「Zephyr 版からの移し替え」）。読み込み済みの校正値と資格情報の
  一覧は、次の起動まで RAM に残ります。
- `sensor`（`sensor get lsm6dsox accel_xyz` など）は Zephyr 版の sensor シェルと同じです。計測中は `-16` で断ります。

### 時刻同期

```text
nsh$ ntp status
state: synchronized
  server:                     10.42.0.1:123 (default gateway)
  ...
```

Wi-Fi に繋がると NTP の問い合わせ（既定はデフォルトゲートウェイ、2 秒ごと）を始めます。出力は Zephyr 版と
同じです。実時間時計は nsh の `date` で表示します（UTC）。

### 校正

```text
nsh$ imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.29 -3.24 -0.36 LSB (stored, 0 rejected)
```

`imu cal reload`（KV を読み直す）、`imu cal zero`（静止中に角速度のゼロ点を取り直す。RAM だけ）も Zephyr 版と
同じです。校正値は `sensor` の物理量に掛かり、MQTT で送る RAW には掛かりません。

### そのほかのコマンド

```text
nsh$ nina version
NINA firmware: 3.0.2
NINA capabilities: 0x00000007 (in use 0x00000007)
nsh$ i2c_dma stats
dma_transfers=... dma_bytes=... dma_errors=0 dma_fallbacks=0
nsh$ i2c read_byte i2c@40044000 6a 0f
Output: 0x6c
nsh$ irqstack
0x20002f88 IRQ 00                           (real size 2048):	unused 1684	usage  364 / 2048 (17 %)
```

- `nina version` の 2 行目は、NINA のモジュールの追加機能のビットと使っているビットです（Zephyr 版と同じ）。
- `nina passthrough confirm` で、USB のシリアルを NINA の UART に中継するモードで再起動します（esptool で NINA の
  ファームウェアを読み書きする。手順はルートの README の「NINA のファームウェアの読み書き」）。300 bps にして閉じるか RESET で
  通常の起動に戻ります。
- `bootsel` で BOOTSEL（`RPI-RP2`）に再起動します。
- `dmesg` は RAM ログ（8 KiB、`reboot` をまたいで残る）を表示します。行の形式は Zephyr 版のログと同じです。
- `ps` はスレッドごとの優先度、スタック（STACK・USED・FILLED）と CPU の割合を出します。CPU 列は直近 1〜2 秒の
  重みが大きい移動平均（0.1 % 単位）です（Zephyr 版の `kernel ps` は前回の `kernel ps` からの
  区間の割合）。起動からの実行時間の
  合計は `cat /proc/<tid>/critmon` の 2 つ目の値（秒）です。
- `irqstack` は割り込み用のスタックの起動からの最大使用量です（Zephyr 版の `kernel thread stacks` の
  `IRQ 00` の行と同じ書式）。
- 速度の測定は `iperf`（iperf 2、ポート 5001）です。PC 側は `iperf3` ではなく `iperf` を使います。

```bash
# ボードから PC へ（PC で iperf -s）
iperf -c <PC> -t 30 -l 1024
# PC からボードへ（ボードで。PC で iperf -c <ボード> -t 30）
iperf -s -B 0.0.0.0
```

サーバーには `-B 0.0.0.0` を付けます（NINA は特定のアドレスへの bind を受け付けません）。UDP は
`-u -l 500` のように 512 バイト以下にします。

## Zephyr 版とのコマンドの対応

| Zephyr 版 | NuttX 版 | 違い |
|---|---|---|
| `imu start` / `stop` / `status` | 同じ | |
| `imu cal` / `imu cal reload` / `imu cal zero` | 同じ | |
| `ntp status` | 同じ | |
| `kv set` / `sethex` / `get` / `delete` | 同じ | 保存形式も同じ |
| （なし） | `kv wipe confirm` | KV の領域を消す。NuttX 版だけ |
| `nina version` | 同じ | |
| `nina passthrough confirm` | 同じ | |
| `bootsel` | 同じ | FreeRTOS 版も同じ |
| `sensor get` / `attr_set` / `attr_get` / `stream` / `trig` | 同じ | `sensor info` はどちらにも無い |
| `i2c_dma mode` / `stats` | 同じ | |
| `dmesg` | 同じ | |
| `wifi cred add` / `delete` / `list` / `auto_connect` | 同じ | |
| `wifi connect -s <SSID> -p <パスフレーズ> -k 1` | `wapi psk wlan0 <パスフレーズ> 3` の後に `wapi essid wlan0 <SSID> 1` | |
| `wifi connect -s <SSID> -k 0` | `wapi essid wlan0 <SSID> 1` | |
| `wifi disconnect` | `wapi disconnect wlan0` | |
| `wifi scan` | `wapi scan wlan0` | NINA のファームウェアにパッチ 0004 が無ければ、接続中は切断として扱い、終わった後で再接続を頼む（Zephyr 版と同じ） |
| `wifi status` | `wapi show wlan0` と `ifconfig wlan0` | |
| `net iface` | `ifconfig` | |
| `zperf tcp upload <PC> 5201 30 1024` | `iperf -c <PC> -t 30 -l 1024`（PC は `iperf -s`） | iperf 2、ポート 5001 |
| `zperf udp upload <PC> 5201 30 500 500K` | `iperf -c <PC> -u -t 30 -l 500`（PC は `iperf -s -u`） | 速さは指定できない |
| `zperf tcp download 5201` / `download stop` | `iperf -s -B 0.0.0.0` | 1 回の接続で終わる |
| `kernel reboot` | `reboot` | どちらも引数を見ずに再起動する |
| `kernel ps` | `ps` | CPU 列の意味が違う（「そのほかのコマンド」） |
| `kernel thread list`（`Total execution cycles`） | `cat /proc/<tid>/critmon` | 2 つ目の値が起動からの実行時間の合計（秒） |
| `kernel thread stacks` | `ps` と `irqstack` | |
| `kernel version` | `uname -a` | |
| `kernel uptime` | `uptime` | |
| `date get` | `date` | 例 `Sat, Nov 14 03:23:03 2026` |
| `date set 2026-11-14 03:23:03` | `date -s "Nov 14 03:23:03 2026"` | |
| `i2c ...`（`scan`・`read_byte`・`read`・`direct_read`・`write_byte`・`write`・`speed`・`recover`） | 同じ | `i2c speed i2c@40044000 <1〜3>` はセンサーのバスの速さを変える（1 = 100 kHz、2 = 400 kHz、3 = 1 MHz） |
| `devmem <アドレス>`（読み出し） | `xd <16 進のアドレス> <バイト数>` | 書き込みは無い |
| `retval` | `echo $?` | |
| `rem <文>` | `# <文>` | |
| `help` | `help` | |
| – | `irqstack` | 割り込み用のスタックの使用量 |

Zephyr 版の `gpio`、`log`、`device`、`history`、`clear`、`resize`、`shell`、`wifi`・`net`・`kernel` の
上に無いサブコマンドは、NuttX 版にはありません。PC 側のツール（`imu-host`、`ImuStudio`、`script/kv_transfer.py`）が
送るコマンドは、どれも「同じ」の行のものです。

## エラー番号（errno）

コマンドの出力、ログ、`imu status`・`ntp status`、MQTT の status に出るエラー番号は NuttX の値です（Zephyr 版の
値へ変換しません）。Zephyr 版（picolibc）と値の違うもの:

| 名前 | Zephyr 版 | NuttX 版 | この移植で出る場所の例 |
|---|---|---|---|
| `ENOMSG` | 35 | 42 | |
| `ENOSYS` | 88 | 38 | `sensor trig`（`-38`） |
| `EBADMSG` | 77 | 74 | |
| `EOVERFLOW` | 139 | 75 | FIFO のオーバーラン |
| `ENOTSOCK` | 108 | 88 | |
| `EMSGSIZE` | 122 | 90 | 512 バイトを超える UDP の送信（`iperf` の `err=90`） |
| `ENOPROTOOPT` | 109 | 92 | NINA のソケットが受け付けないオプション |
| `EPROTONOSUPPORT` | 123 | 93 | |
| `ENOTSUP` | 134 | 138 | `sensor` の受け付けない属性、`wapi` の受け付けない暗号 |
| `EAFNOSUPPORT` | 106 | 97 | |
| `EADDRINUSE` | 112 | 98 | |
| `EADDRNOTAVAIL` | 125 | 99 | `iperf -s` に `-B 0.0.0.0` が無いとき（`error code: 99`） |
| `ENETDOWN` | 115 | 100 | |
| `ENETUNREACH` | 114 | 101 | |
| `ECONNABORTED` | 113 | 103 | |
| `EISCONN` | 127 | 106 | |
| `ENOTCONN` | 128 | 107 | |
| `ETIMEDOUT` | 116 | 110 | Wi-Fi の接続（約 30 秒）、TCP の接続（10 秒）、NINA の応答、I2C の転送 |
| `EHOSTUNREACH` | 118 | 113 | |
| `EALREADY` | 120 | 114 | Wi-Fi の接続の要求 |
| `EINPROGRESS` | 119 | 115 | 待たない connect |
| `ECANCELED` | 140 | 125 | FIFO のストリームを止めたときの終わりの値 |
| `EILSEQ` | 138 | 84 | KV の領域に別のデータ（NuttX 版だけが返す） |

値の同じ主なもの: `ENOENT` 2（無いキー）、`EIO` 5（I2C の失敗）、`EAGAIN` 11、`ENOMEM` 12、`EBUSY` 16
（ストリーム中の `sensor`）、`ENODEV` 19、`EINVAL` 22、`EMFILE` 24（NINA のソケット 11 個目）、`ENOSPC` 28
（KV が満杯）、`EPROTO` 71、`ENOBUFS` 105（資格情報の 3 件目）。

## Zephyr 版との違い

- **シェル**: nsh。プロンプトは `nsh$ `、未知のコマンドは `nsh: <名前>: command not found`。Tab 補完はコマンド名だけ、
  履歴は 8 行。`sensor stream` の出力の間にプロンプトを描き直さない。
- **USB コンソール**: 製品文字列は `NuttX USB Console`、デバイスクラスは 02/00/00 で IAD なし。ポートを開いていない
  間の出力は捨て、開いた直後は Enter を送るまでプロンプトが見えない。
- **Telnet**: 接続した直後にプロンプトを送り、入力をエコーする。標準入力を読むコマンドは Ctrl-C で終わらない。
- **コマンド**: Zephyr 標準のコマンドは NuttX 標準のものになる（上の表）。`wapi` は失敗の値を表示しない。
  速度の測定は iperf 2。
- **エラー番号**: NuttX の値（上の表）。
- **KV と資格情報**: 形式は同じ（Zephyr の NVS と settings を写したもの）。KV の領域に別のデータがあるときだけ違う。
  NuttX 版は消さずにエラーを返し、`kv wipe confirm` を待つ。Zephyr 版は Zephyr の NVS のとおりに扱う: 2 つの
  セクターの末尾 8 バイトがどちらも書かれていればマウントに失敗して KV を使えず（消すコマンドは無い）、どちらかが
  消去済みなら中断したガベージコレクションとみなして領域を消し、空の KV で始める（以前の NuttX 版の形式は、書き込み中の
  ブロックの末尾が消去済みなので、ふつうこちら）。
- **ログ**: 呼び出したスレッドの中で整形する（Zephyr 版は deferred）。RAM のリングは 8 KiB（Zephyr 版は 4 KiB）。
  実行時のレベル変更はない。
- **時計**: カーネルの時計、`k_uptime_get()`、`k_cycle_get_*()`、センサーの時計、CPU 負荷の時間の源は RP2040 の
  TIMER（水晶の 1 MHz）。Zephyr 版は CPU クロックの SysTick（送信中は遅れる）。tick はどちらも 100 µs の tickless。
- **スレッド**: 優先度は `118 − Zephyr`。Zephyr の協調スレッド（`sysworkq`、`net_mgmt`）は 119 の通常のスレッド。
  NINA のソケットの状態をイベントにする `nina_event` は NuttX 版だけ。割り込みはすべて同じ優先度。
- **ソケット**: NINA のソケットは NuttX の usrsock。MQTT と NTP のソケットは呼び出したスレッドが NINA に問い合わせ、
  それ以外（Telnet、`iperf`）は `nina_event` が 20 ms ごとに問い合わせる。
- **クロックとピン**: PLL_USB の VCO は 480 MHz（出力はどちらも 48 MHz）、ベクター表は SRAM、GPIO4・6・7 を SPI0 に
  割り当てない、ROSC の設定を変えない、GPIO26〜29 の入力を切る、クロックの初期化で SYSCFG と USBCTRL をリセットする。

## スレッドと優先度

優先度は `NuttX = 118 − Zephyr` で対応させています（`common/compat/include/priorities.h` の `APP_PRIO_*`・`APP_STACK_*`。換算は `common/compat/port/nuttx/priorities_os.h`）。

| スレッド | Zephyr | NuttX | スタック |
|---|---|---|---|
| `wifi_reconnect_tid`、`nina_wifi_tid` | 8 | 110 | 2048、2048 |
| `time_sync_tid`、`imu_stream_tid` | 9 | 109 | 2048、2048 |
| `imu_fifo_worker` | 10 | 108 | 2048 |
| `imu_raw_mqtt_tid` | 11 | 107 | 2048 |
| `nina_event`（NuttX 版だけ） | – | 106 | 1536 |
| `sysworkq` | −1（協調） | 119 | 1024 |
| `net_mgmt` | −1（協調） | 119 | 2048 |
| `RTIO WQ 0`（`ps` では `RTIO_WQ_0`） | 0 | 118 | 1024 |
| `sensor_shell_tid` | 0 | 118 | 3072 |
| `main`（NuttX は `init`。起動処理の後 104 に下げる） | 0 | 118 | 2048 |
| シェル（USB の `nsh`、Telnet の `nsh_telnet`）とコマンド | 14 | 104 | 3072 |
| `telnetd`（Zephyr 版は `net_socket_service`） | 15 → 14 | 104 | 2048 |
| `iperf_traffic`・`iperf_report` | 15 → 14 | 100・101 | 4096・4096 |

割り込み用のスタックは 2048 バイトです（`irqstack`）。`ps` の STACK は NuttX が先頭に置く領域の分だけ小さく
出ます（2048 → 2008 など）。

## テスト

```bash
nuttx/script/host_tests.sh
```

OS に依存しない部分のホストテスト 73 本を PC 上でビルドして実行します。最初の 53 本は 4 つの OS のファームウェアで共有する
ソースと、互換層・シェル・RAM ログのリング・KV の共通部分のテスト（最後の `zephyr_copies` は、写した Zephyr の
ファイルが元と同じことの照合）で、`common/tests/run.sh` が実行します。ボードは不要で、ホストの `cc`（C17）だけを
使います。ST の `lsm6dso_reg.c`（`setup.sh` が取得）が無ければ、それを使う 3 本（`lsm6dso_driver`、`imu_cal`、
`sensor_shell`）は `skip` になります。失敗したところで止まります。各テストの内容と、Zephyr 版から変更せずに
写したファイルの一覧（Zephyr 版を直したときに置き換えるもの）は [`design/nuttx-porting-ja.md`](../design/nuttx-porting-ja.md) の
「ホストテストの詳細」です。

## ディレクトリ構成

| 置き場所 | 内容 |
|---|---|
| `boards/arduino-nano-rp2040-connect/` | ボード定義（`configs/imu/defconfig`、`include/board.h`、リンカースクリプト、`src/` の起動処理・時計・I2C0 と DMA の NuttX の部分・KV の flash など） |
| `drivers/` | カーネル側のコード（`libexternal.a` になる）: NINA（`nina/`: オプションとビルドの規則。ドライバーは `../common/drivers/nina/` と、その NuttX へのつなぎ `port/nuttx/`）、I2C（`i2c_dw/`: オプションとビルドの規則。ドライバーは `../common/drivers/i2c_dw/` と `port/nuttx/`）、LSM6DSOX（`lsm6dso/`）、RAM ログ（`log/`: ビルドの規則。ソースは `../common/compat/log/` と `../common/compat/port/nuttx/`）、Zephyr の API の互換層のビルドの規則（`compat/`。NuttX の実装は `../common/compat/port/nuttx/`） |
| `apps/` | アプリ: `init`（起動処理と Telnet）、`imu`（取得・MQTT・時刻同期・校正・KV）、`kv`、`nina`、`ntp`、`wifi`、`sensor`、`i2c_dma`、`dmesg`、`irqstack`、`bootsel`、`subcmd`（Zephyr のシェルのコマンド処理とサブコマンドの処理。ソースは `../common/shell/`、ここには NuttX のシェルのバックエンド） |
| `patches/` | NuttX と nuttx-apps へのパッチ |
| `script/` | `setup.sh`、`build.sh`、`flash.sh`、`host_tests.sh`、`bin2uf2.py`、`kconfig-tweak` |
| `tests/` | ホストテスト（NuttX 版だけのもの） |
| `../common/` | Zephyr 版・FreeRTOS 版・ThreadX 版と共有するソース（`imu/`、`drivers/lsm6dso/`、`drivers/stmemsc/`、`drivers/nina/`、`drivers/i2c_dw/`、`shell/`。`drivers/nina/` の NINA のコアは 4 つの OS のファームウェアが、`drivers/i2c_dw/` の I2C のコアはNuttX 版と FreeRTOS 版が使う）、互換層の OS に依存しない部分（`compat/`）、シェルのコマンド処理（`shell/`）、`LOG_*` マクロと RAM ログ（`compat/log/`）とそれらのホストテスト。ビルドは `VPATH` でそこを参照し、NINA のコア、I2C のコア、stmemsc、`shell/`、`compat/log/` を除く各ファイルに前置きヘッダー（`../common/compat/port/nuttx/common_prelude.h`）を付けてコンパイルする。オブジェクトは `apps/imu`・`apps/wifi`・`apps/sensor`・`apps/subcmd`・`drivers`・ボードのディレクトリーにできる |
| `nuttx/`、`nuttx-apps/`、`pico-sdk/`、`toolchain/`、`stmemsc/`、`.venv/`、`build/` | `setup.sh` が取得するものとビルドの生成物（Git 管理外） |
