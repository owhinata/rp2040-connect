# rp2040-connect

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けの Zephyr ワークスペースです。
複数のアプリケーションで、`manifest/west.yml` に指定した Zephyr と
モジュールを共有します。モジュールはアプリの機能に応じて追加してください。

設計と実装の方針は [IMU FIFO ドライバー計画](design/imu-fifo-driver-plan-ja.md) と
[IMU RAW・MQTT 配信設計案](design/imu-raw-mqtt-design-ja.md) を参照してください。

## 環境構築

以下は Ubuntu 24.04 向けの手順です。初めにビルドに必要なパッケージを
インストールします。

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

AArch64 環境では `gcc-multilib` と `g++-multilib` を除いてください。
他の OS では [Zephyr の公式手順](https://docs.zephyrproject.org/latest/develop/getting_started/)を参照してください。

このリポジトリのルートでセットアップスクリプトを読み込みます。

```bash
source script/setup.sh
```

スクリプトは必要に応じて `.venv`、west ワークスペース、モジュール、
Python パッケージ、Zephyr SDK と Arm ツールチェーンを用意します。
また、現在のシェルに `ZEPHYR_SDK_INSTALL_DIR` と `west` のエイリアスを
設定します。仮想環境の activate は不要です。新しいターミナルでは再度
`source script/setup.sh` を実行してください。

インストール済みかどうかは、west のプロジェクト、主要な Python パッケージ、
SDK と Arm コンパイラの有無から判定します。追加したアプリでモジュールが
必要になった場合は、`manifest/west.yml` を更新し、次を実行してください。

```bash
west update
west packages pip --install
```

## アプリケーションのビルド

`app/blink` は、ボードの `led0` を 500 ms ごとに切り替えるサンプルです。
ビルドには次のコマンドを実行します。

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

ボードを USB に接続してブートローダーモードにし、UF2 runner を指定して
書き込みます。

RESET ボタンの2回押しでブートローダーモードに入れない場合は、`REC` と
`GND` をジャンパー線で接続した状態で RESET ボタンを押します。USB ストレージが
表示されたらジャンパー線を外してください。

```bash
west flash -d build/blink -r uf2
```

## USB シリアルコンソール

`app/usb-console` は USB CDC ACM 経由で Zephyr シェルを使うアプリです。
初回やマニフェストを更新した後は `west update` を実行し、次のコマンドで
ビルドして書き込みます。

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

書き込み後、USB シリアル端末から接続します。デバイス名は環境によって
異なりますが、例えば `/dev/ttyACM0` の場合は次のように接続できます。

```bash
minicom -D /dev/ttyACM0
```

Enter キーを押すとシェルが表示されます。`help` でコマンド一覧を、`hello` で
挨拶を表示します。ログは RAM のリングバッファに保存され、`dmesg` コマンドで
表示できます。バッファは 4 KiB で、いっぱいになると古いデータから上書きされます。
通常のリセット後もログを参照できるよう初期化されない RAM に保存します。
電源断や、RAM を初期化するリセットでは消えます。リセット直前の未処理のログは
保存されない場合があります。タイムスタンプは起動後の経過時間をマイクロ秒単位で
表示し、リセット後は 0 から始まります。
minicom は `sudo apt install minicom` でインストールでき、
終了するには `Ctrl-A`、続けて `X` を押します。

GPIO、I²C、センサー、再起動、ログのシェルコマンドを有効にしています。
`gpio --help`、`i2c --help`、`sensor --help`、`kernel reboot --help`、
`log --help` で使い方を確認できます。`log` はログレベルの参照や変更に使い、
`dmesg` は RAM に保存されたログの表示に使います。

`kernel ps` を実行すると、スレッドの CPU 使用率とスタック使用量を表示します。

## Wi-Fi 接続（開発中）

NINA-W102 の Arduino ファームウェアを SPI で操作し、Zephyr の Wi-Fi 管理 API と
IPv4 TCP ソケットに接続する実装を開発しています。以下の標準シェルコマンドで
WPA2-PSK のアクセスポイントへの接続と DHCP アドレスの取得を実機で確認しました。
従来の通信確認には NINA ファームウェア `1.4.8` を使用しました。その後、
`socket_poll` の初期化を修正した `3.0.2` に更新し、Wi-Fi 再接続と SNTP の
同期を確認しました。ビルド方法は [NINA ファームウェアの説明](firmware/nina-fw/README.md)
を参照してください。現在のバージョンは `nina version` で確認できます。

```text
wifi connect --ssid "SSID" --key-mgmt 1 --passphrase "パスフレーズ"
wifi scan
wifi status
net iface
ntp status
wifi disconnect
```

2.4 GHz のオープンおよび WPA2-PSK 接続を実装しています。オープン接続は未検証です。パスフレーズは端末に
表示されます。シェルの履歴は無効ですが、端末側の記録には注意してください。

`wifi scan` は周囲の 2.4 GHz アクセスポイントを最大 10 件表示します。NINA の
ファームウェアはスキャン時に Wi-Fi を停止するため、接続中は一時切断され、
保存済み設定で再接続を試みます。SSID やチャンネルの絞り込みなどのスキャン
オプションには対応していません。実機で一覧の表示と、スキャン後の自動再接続・
IP アドレスとゲートウェイの再取得を確認しました。

Wi-Fi 接続後、Zephyr の SNTP クライアントで `time.cloudflare.com` から時刻を
取得し、システム時計を設定します。`ntp status` で同期状態と Unix 時刻を確認
できます。サーバーとポートはビルド設定の `CONFIG_NINA_NTP_SERVER` と
`CONFIG_NINA_NTP_PORT` で変更できます。失敗時は 60 秒以上の間隔で再試行し、
成功後は 6 時間ごとに再同期を試みます。24 時間同期できなければ、アプリ向けの
時刻準備状態は未準備になります。NINA の制約により TCP 通信中は UDP による
時刻同期ができず、次回の試行まで待ちます。

SNTP の応答は暗号認証されません。取得した時刻は送信元の信頼性を保証せず、
TLS の安全性には時刻源の信頼も必要です。実機で公開サーバーからの自動同期と
`ntp status` の表示を確認しました。

KV ストレージは Zephyr Settings/NVS を使用します。Wi-Fi 認証情報は Zephyr 標準の
credentials API を使い、同じ Settings/NVS 領域に保存します。
シェルでは次のように利用できます。

```text
kv set greeting hello
kv get greeting
kv sethex bytes 0001ff
kv get bytes
kv delete greeting
```

`get` の結果はバイナリを含めて 16 進数で表示します。`set` はテキストを終端の
NUL を含めずに保存し、`sethex` は 16 進数のバイト列を保存します。キーは英数字、
`_`、`-`、`.` の 1〜32 文字、値は 1〜128 バイトです。空の値は保存できません。
アプリからは `kv_store.h` の `kv_store_set`、`kv_store_get`、`kv_store_delete` を
使用します。読み出しバッファーが不足した場合は `-ENOBUFS` を返します。
`sys.` で始まるキーは内部機能用に予約し、シェルから操作できません。
保存領域は Wi-Fi 設定と共有する 8 KiB で、書き込み時には容量不足やフラッシュ
書き込みの影響があります。データは暗号化されず、シェルで読み出せます。
ファームウェア全消去などで失われる場合があります。

Wi-Fi 設定は標準の `wifi cred` で管理します。`wifi connect` は接続のみを行い、
設定を自動保存しません。設定を保存する場合は次のように登録します。

```text
wifi cred add -s "SSID" -k 1 -p "パスフレーズ"
wifi cred list
wifi cred auto_connect
wifi cred delete "SSID"
```

`-k 1` は WPA2-PSK です。同じ SSID を登録すると上書きします。
`wifi cred list` はパスフレーズも表示します。端末の表示や記録に注意してください。

### Wi-Fi 経由のシェル

パッチ適用済み NINA 3.0.2 とこのアプリでは、Wi-Fi 接続後に TCP ポート 23 の
Zephyr Telnet シェルを利用できます。`net iface` でボードの IPv4 アドレスを確認し、
PC から `telnet <IPv4アドレス> 23` で接続してください。接続直後にプロンプトが
表示されない場合は Enter を押してください。同時に使える Telnet クライアントは
１つです。USB コンソールも引き続き使えます。

Telnet には認証と暗号化がなく、到達できる相手はフルシェルを操作できます。
Wi-Fi 認証情報を含む設定も表示できるため、信頼できるネットワークで使用して
ください。実機では基本コマンド、接続の繰り返し、Wi-Fi 再接続後の Telnet、
IMU の短時間測定と起動時の SNTP 同期を確認しました。通信中に一度接続タイムアウト
が発生し、再起動で回復しました。その後の再接続試験では再現していません。
長時間の安定性と MQTT との同時利用は未検証です。
起動時と予期しない
切断後は保存済み設定への接続を再試行します。明示的な `wifi disconnect` 後は
次回起動または手動接続まで再接続しません。Zephyr の現行実装では接続要求を
受け付けた最初の登録先を試すため、複数登録先の切り替えは保証されません。
以前の独自形式は、標準 credentials への保存成功後に削除します。移行失敗時は
旧データを残して次回起動時に再試行します。

`wifi status` で接続状態、SSID、BSSID、RSSI を確認できます。セキュリティ方式、
チャネル、リンクモードなど、モジュールから取得していない項目は未対応です。

パスフレーズはフラッシュに暗号化せず保存します。ボードやフラッシュの読み出し権限を
持つ人は取得できます。設定用の領域はフラッシュ末尾の 8 KiB で、このアプリの
プログラム領域から分離しています。別のファームウェアや全消去によっては保存設定が
消える場合があります。保存時のフラッシュ処理は IMU 取得に影響し得るため、
Wi-Fi 接続が完了してから測定を開始してください。

実機から PC 上の簡易 MQTT テスト受信口へ CONNECT を送り、CONNACK と QoS 0 の
送信を確認しました。実際の MQTT ブローカー、長時間通信、エラー時や接続切断の
挙動は未検証です。
DNS 名の解決は Zephyr の `getaddrinfo` を NINA にオフロードし、実機で数値 IP と
`example.com` の名前解決を確認しています。IPv4 の TCP クライアントと UDP に対応し、
TLS には未対応です。従来の TCP クライアントでは最大 4 ソケットの同時接続を
実機で確認しました。ドライバのソケット枠は現在 6 個で、Telnet の待ち受けや
`iperf3` の制御・データ接続も共有します。NINA と Zephyr の eventfd の混在 `poll` は
使用しますが、他のオフロードドライバとの混在や切断時の並行操作は未検証です。
現在のビルドは Zephyr のテスト用非暗号
乱数を使います。

従来の UDP は単一ソケットに限定し、他の従来型 UDP または TCP クライアントと同時に
使用しようとすると `EOPNOTSUPP` を返します。NINA ファームウェア 1.4.8 では他ソケットに未読
データがあると UDP 受信が止まるためです。送信は 1 データグラム 512 バイトまで、
空の受信データグラムは検出できません。接続先への `connect`、送受信、短い受信
バッファーによる切り捨てと `poll` を実機で確認しました。連続受信試験では最初に
1 回タイムアウトし、その後の 4 回は成功しましたが、原因は未特定です。
受信時に NINA の 1,500 バイトのバッファーを超える部分は検出できずに失われます。
`bind` は任意アドレスのみ対応し、`sendmsg`、マルチキャストの設定やソケットの
タイムアウト設定は未対応です。

MQTTS に必要な TLS と ATECC608A Security チップはまだ使っていません。標準設定の
I2C0 は 100 kHz です。以前の 1 MHz スキャンでは ATECC608A を検出できなかったため、
IMU と同じバスでの利用時は起床手順や通信速度を含めた共存検証が必要です。

実機で意図しない切断後の IP・ゲートウェイ消去と再接続、`wifi disconnect` 後に
再接続しないことを確認しました。また PC との 600 バイトの TCP バイナリ往復と、
未使用ポートへの接続が `ECONNREFUSED` になることを確認しました。

## Wi-Fi スループットの測定

Zephyr の `zperf` を `iperf3` プロトコルで有効にしています。ボードの USB または
Telnet シェルで TCP サーバーを起動し、PC から送信速度を測れます。

```text
zperf tcp download 5201
```

PC 側では `iperf3 -c <ボードのIPv4アドレス> -p 5201 -t 10` を実行します。
終了後、ボード側で `zperf tcp download stop` を実行してサーバーを停止できます。
これは PC → ボードの TCP 受信速度で、ボード → PC の MQTT 送信速度ではありません。
現在の NINA ソケットは `iperf3` クライアントが要求するソケットオプションに対応
していないため、送信方向の `zperf tcp upload` は未対応です。

NINA 3.0.2、SPI 設定 8 MHz、2.4 GHz Wi-Fi の実機で、Telnet 接続なしの 10 秒
測定を３回行ったところ、ボード側の受信速度は約 227、266、363 kbit/s でした。
Telnet を接続した状態でも 10 秒間の `iperf3` 測定を完了しました。

送信方向は `prj_iperf2.conf` を追加した一時ビルドの `zperf tcp upload` と PC 上の
TCP 受信プログラムを使い、512 バイト単位で10秒間測定しました。ボード側の送信
速度は３回で約 272、263、262 kbit/s で、PC 側で同じバイト数を受信しました。
通常版では `iperf3` を選択しているため、`iperf2` の測定版は次のようにビルドします。

```sh
west build -b arduino_nano_connect app/usb-console -d build/usb-console-iperf2 -- -DEXTRA_CONF_FILE=prj_iperf2.conf
```

測定版を書き込んだ後、PC で `python3 script/tcp_sink.py --port 5001` を起動し、
ボードのシェルから `zperf tcp upload <PCのIPv4アドレス> 5001 10 512` を実行します。
受信ツールは接続を１回受け付け、受信バイト数と速度を表示して終了します。

SPI のデータ部分をまとめて転送する変更後、512 バイト単位では 296、341、
339 kbit/s でした。実験的に TCP の送信単位を 1460 バイトにすると、SPI DMA
なしで 373、530、474 kbit/s、Zephyr の PL022 SPI DMA ありで 450、297、
429 kbit/s でした。いずれも受信データのパターンに不一致はありませんでした。
短時間の測定には大きな変動があり、DMA が本質的に遅いと結論できるものでは
ありません。1460 バイト送信と SPI DMA は実験用設定です。DMA 版の再現には、
別のビルドディレクトリを指定し、`--` の後に
`'-DEXTRA_CONF_FILE=prj_iperf2.conf;prj_tcp_1460.conf;prj_spi_dma.conf'`
と `'-DDTC_OVERLAY_FILE=app.overlay;spi_dma.overlay'` を追加します。
送信コマンドのブロックサイズは 1460 にします。

追加の送信時間計測では、20 ms の再試行待ちで 516 kbit/s だった１回のうち、
NINA のゼロバイト応答は 310 回、再試行待ちは合計約 6.2 秒でした。送信要求の
SPI 処理区間は合計約 1.25 秒です（割り込みやスケジュール待ちを含む経過時間）。
ゼロ応答は送信バッファが満杯の場合などに発生し、NINA の API では原因を区別
できません。再試行間隔を 2 ms にした実験では 539、646、649 kbit/s でしたが、
ゼロ応答の増加と無線環境による変動があるため通常設定は変更していません。
計測には `prj_nina_profile.conf`、2 ms の実験には `prj_nina_retry2.conf` を
追加し、送信前後に `nina_send_profile reset` と `nina_send_profile` を実行します。

さらに実験的に TCP 送信と zperf のブロックを 2872 バイト、再試行を 2 ms にした
ところ、10 秒の３回は省電力有効要求後に 0.71、1.43、1.15 Mbit/s、無効要求後に
1.13、1.07、1.25 Mbit/s でした。無効要求後の 30 秒測定は 0.91 Mbit/s でした。
データの不一致はいずれもありません。省電力要求は NINA が ESP 側の成否を返さない
ため設定成功を確認できず、短時間の無線測定だけで効果を断定できません。
測定版には `prj_nina_tcp2872.conf` と `prj_nina_retry2.conf` を追加します。
`prj_nina_profile.conf` の `nina_benchmark_power on|off` は省電力を要求します。
加速度と角速度を各 208 Hz で同時取得した１回の TCP 測定は 0.96 Mbit/s で、
IMU の 30 秒測定にエラーやオーバーランはありませんでした。両方 1667 Hz では
TCP 試験開始前に FIFO オーバーランしたため、その条件での同時動作は未検証です。

目的条件を確認するため I²C 1 MHz、両センサー各 6667 Hz、watermark 128 で TCP
送信と並行したところ、１回は約 14 秒で I²C エラー、別の１回は約 32 秒間エラーや
overrun 観測なしで動作しましたが、想定外の FIFO ワードが検出されました。後者の
TCP 送信は約 773 kbit/s でした。これはダミーデータの送信であり、IMU データの
MQTT 配信ではありません。

原因切り分けでは以前の IMU 単独版と現在の Wi-Fi 切断時は、各 10 秒の測定で
想定外のワードが 0 個でした。現在版を Wi-Fi 接続すると、IMU 単独でも想定外の
ワードが現れます。診断版で確認した最初のワードは `ff ff ff ff ff ff ff` で、
ある 10 秒の測定では 8 バッチに計 548 個あり、いずれもバッチ末尾に集中して
いました。I²C DMA の無効時にも再現しました。原因と実際の欠落は未確定です。
この条件での無欠落取得は、以前の結果だけから保証できません。

追加の診断では、NINA の Wi-Fi 省電力を無効にすると、6667 Hz の FIFO 取得時に
見られた異常ワードが検出されなくなりました。診断ファームは NINA の省電力設定の
成否を返すパッチを使用しています。原因となる電気的な仕組みは未確定です。

実験用 MQTT 送信では FIFO の各軸を 16-bit の生バイナリのまままとめ、フレーム番号と
加速度／角速度を識別するビット列を付けています。ボードが 2.4 GHz、PC が同じ AP の
5 GHz に接続し、PC と NINA の省電力を無効にした条件では、6667 Hz の 120 秒測定で
加速度 804,564 件、角速度 804,566 件を PC で受信し、件数とペイロードの CRC32 が
ボード側と一致しました。FIFO エラーと診断送信キューの欠落はありませんでした。
MQTT ペイロードの平均速度は約 665 kbit/s です。この測定は NINA 側の実験的な TCP
変更と専用の QoS 0 受信テストを使用しており、製品用の MQTT ブローカーや長期的な
無欠落を保証するものではありません。同じ接続条件で NINA 側の TCP 実験変更を外した
60 秒の測定でも、各 401,404 件を受信し、件数と CRC32 が一致しました。追加の比較では、
PC が 2.4 GHz の場合、省電力有効時は約 2.0 秒、無効時は約 1.8 秒で送信キューが満杯に
なりました。PC が 5 GHz・省電力有効の場合は 30 秒間、受信件数と CRC32 が一致しました。
各条件は短時間の測定であり、無線環境の変動や他の設定では結果が異なる可能性があります。
同じ接続条件での TCP ダミーデータ送信は、IMU 停止時に 30 秒間で約 4.19 Mbit/s、
両センサーの 6667 Hz 取得と並行した 30 秒間で約 2.02 Mbit/s でした。後者の IMU
取得は約 35 秒間、異常タグ、I²C エラー、FIFO オーバーランがありませんでした。

上記の MQTT 診断版は、NINA 3.0.2 に `firmware/nina-fw/build.sh` が適用する 2 件の
パッチを入れ、次の構成でビルドできます。これは試験用であり、通常版とは別です。

```sh
west build -b arduino_nano_connect app/usb-console -d build/usb-console-imu-mqtt -- \
  '-DEXTRA_CONF_FILE=prj_iperf2.conf;prj_nina_tcp3584.conf;prj_nina_profile.conf;prj_nina_retry1.conf;prj_nina_nops.conf;prj_imu_mqtt_probe.conf;prj_nina_state100.conf;prj_nina_poll100.conf;prj_imu_mqtt_prio8.conf'
```

PC 側で `python3 script/imu_mqtt_probe.py --bind <PCのIPv4アドレス> --port 18830` を
実行し、ボードを Wi-Fi に接続してから、USB シェルで次を実行します。接続アドレスは
環境に合わせて変更します。

```text
imu_mqtt_probe start <PCのIPv4アドレス> 18830
imu_mqtt_probe status
i2c speed i2c0 3
sensor attr_set lsm6dsox all fifo_watermark 128
sensor attr_set lsm6dsox accel_xyz sampling_frequency 6667
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 6667
imu_measure start 60
```

`status` の `connected=1 enabled=1` を確認してから測定を始めます。測定完了後に
`imu_mqtt_probe status` で送信キューと CRC32 を確認し、`imu_mqtt_probe stop` で
キューを送り切って切断します。PC 側に出力される件数と CRC32 をボード側と照合します。
この受信プログラムは診断用の簡易 MQTT 受信器で、データを保存する汎用ブローカーでは
ありません。QoS 0 のため通信断からの再送も実装していません。

これらは無線環境と TCP の挙動による変動を含む参考値であり、MQTT や同時 IMU
取得時の速度を保証するものではありません。

## IMU FIFO の測定

`imu_measure` は Sensor Streaming API で FIFO のバッチを取得し、サンプルを
逐次表示せずに件数と転送時間を集計します。まず使用するセンサーのサンプリング
周波数を設定し、測定秒数を指定します。例えば両方 208 Hz で10秒間測定するには：

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

測定終了後に結果を表示します。途中で停止するには `imu_measure stop`、最後の
結果を再表示するには `imu_measure result` を実行します。測定時間は1〜3600秒です。
単一センサーを測る場合は、もう片方の `sampling_frequency` を `0` に設定します。
`sensor stream` と `imu_measure` は同時に使えません。どちらのストリームでも、現在の FIFO カウンターを `imu_measure stats` で確認できます。

`sensor attr_get lsm6dsox all fifo_watermark` で FIFO watermark を表示し、`sensor attr_set lsm6dsox all fifo_watermark 128` でストリーム開始前に 128 ワードへ変更できます。設定範囲は 1〜256、初期値は 32 で、リセットすると初期値に戻ります。大きくするとバッチ処理の負荷が減る一方、データ到着までの時間が長くなり、FIFO の余裕が減ります。IMU FIFO 用途の DMA 対応は完了とし、通常ビルドでは対象となる I²C0 の読み出しに DMA を使用します。今回の用途では 60 秒の確認を十分とし、他の DMA 利用者との競合は必要になった時点で確認します。`i2c_dma stats` で累積カウンターを確認できます。条件と測定結果は [ドライバーの説明](app/usb-console/drivers/README_ja.md) を参照してください。

センサー時刻と内部温度への対応については、[ドライバーの TODO](app/usb-console/drivers/README_ja.md#todo)
を参照してください。

指定時間の終了または途中停止時には、FIFO への新しいバッチ追加を止め、FIFO に
残ったワードと処理中の RTIO 完了通知を集計してから結果を確定します。
`stop_requested_ms` は停止を要求した時点、`elapsed_ms` は残データの回収と終了処理を
含む時間です。センサーごとのバッチ停止は順番に行うため、停止境界は厳密に同時では
ありません。

結果には加速度・ジャイロの受信サンプル数、未対応タグのワード数、設定 ODR、
FIFO バーストの回数と合計・最大時間、I²C エラー、バッファ不足、FIFO overrun の
観測、RTIO の完了通知の欠落、ストリームエラーを表示します。`cpu_active` はアイドルフックから求めた非アイドル時間の割合で、割り込みと計測のオーバーヘッドを含み、表示分解能は 0.1% です。`cpu_busy` は測定区間中の
スケジューラ上の非アイドル時間で、この集計処理と他スレッドの負荷を含みます。アイドル中の割り込みはアイドル時間に含まれる場合があるため、CPU が実際に動作した時間の合計ではありません。
`observed_hz` は受信したサンプル数を `elapsed_ms` で割った概算値です。開始時の設定と
停止・残データ回収にかかる時間の影響を受けるため、短い測定ほど誤差が大きくなります。
FIFO のハードウェア時刻やセンサーの正確な生成時刻を測るものではありません。

`accounting=match` は、ドライバーが読み出したワード数・転送回数とアプリが集計した
ワード数・バッチ数が一致し、集計を妨げるエラーが報告されなかったことを示します。
`mismatch` は両者の不一致、`incomplete` はエラーや完了通知の欠落などにより照合を
完了できなかったことを示します。センサーが生成した全サンプルの無欠落を保証する
ものではありません。

overrun の観測は、不連続が発生したことを示しますが、失われたサンプル数や独立した
overrun の発生件数は示しません。測定中にバッファ不足・I²C エラー・overrun が発生した
場合はストリームを停止し、`stream_error` にエラーを表示します。`cleanup_timeout=1`
の場合は終了処理が完了せず、次の測定には再起動が必要です。完了通知の欠落や
未知のタグが報告された測定も、取得データの連続性を保証できません。
