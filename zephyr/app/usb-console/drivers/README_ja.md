# ローカルドライバー

[English](README.md) | 日本語

`zephyr/app/usb-console` は、Zephyr のドライバーの一部をこのディレクトリのコピーに置き換えて
ビルドします。IMU の FIFO を高いレートで読み出すための変更と、NINA-W102 用の Wi-Fi
ドライバーを含みます。

| ディレクトリ | 内容 | 元 |
|---|---|---|
| `lsm6dso/` | LSM6DSO センサードライバーに FIFO ストリームを追加 | Zephyr v4.5.0 のコピー |
| `i2c_dw/` | DesignWare I²C ドライバーに RP2040 の DMA 読み出しを追加 | Zephyr v4.5.0 のコピー |
| `sensor_shell/` | sensor シェルの修正と独自属性の追加 | Zephyr v4.5.0 のコピー |
| `nina/` | NINA-W102（SPI）の Wi-Fi・ソケットドライバーの Zephyr 側（コアは `common/drivers/nina/`） | このリポジトリで作成 |

コピー元は Zephyr v4.5.0（コミット `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`）で、各ソースの
著作権と Apache-2.0 の表示を維持しています。アプリの `CMakeLists.txt` が、Zephyr の
ドライバーライブラリー内の対象ソースをコピーに差し替えます。Kconfig、devicetree binding、
ST の HAL は Zephyr のものを使います。Zephyr 側の構成が変わって差し替え対象が見つからない
場合は、ビルドをエラーにします。

このうち NuttX 版・FreeRTOS 版と同じファイルを使う部分（LSM6DSO のドライバー本体 `lsm6dso.[ch]` と `lsm6dso_fifo.[ch]`、`lsm6dso_fifo_packet.h`、`lsm6dso_cal.[ch]`、
`lsm6dso_zero.[ch]`、`lsm6dso_fifo_decoder.[ch]`、`lsm6dso_fifo_hw.[ch]`、`lsm6dso_fifo_irq.[ch]`、`lsm6dso_stream.c`、
`lsm6dso_compat.c`、`lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h`、sensor シェルの `sensor_shell.c`、NINA のドライバーのコアと、
I²C の `i2c_dw_rp2040_commands.[ch]`）は、リポジトリの `common/drivers/lsm6dso/`・`common/drivers/sensor_shell/`・
`common/drivers/nina/`・`common/drivers/i2c_dw/` に置いています。`CMakeLists.txt` がそこをドライバーのライブラリー
（NINA はアプリ）に加え、sensor シェルの `lsm6dso_attributes.h` もそこから読みます。このディレクトリーに残る
`sensor_shell/sensor_shell_os.h` は、共有の `sensor_shell.c` が最初に読む Zephyr 版の部分です（何もしない
マクロ 2 つ。NuttX 版は同名のヘッダーで `sensor get` が結果の表示を待ちます）。`lsm6dso/lsm6dso_instance.h` は、共有の
`lsm6dso.c` が末尾で読む Zephyr 版の部分で、ドライバーのデバイスツリーのインスタンス化のマクロをそのまま置いています
（NuttX 版は同名のヘッダーでボードの値を使うデバイスを 1 つ定義します）。`lsm6dso/lsm6dso_shub.c`・`lsm6dso_trigger.c`
（Zephyr の写し。無効にしてある `CONFIG_LSM6DSO_SENSORHUB`・`CONFIG_LSM6DSO_TRIGGER` のときだけビルドされる）は
このディレクトリーに残り、`lsm6dso.h` を `common/drivers/lsm6dso/` から読みます。NINA のコア（`nina_spi.c`、`nina_net.c`、`nina_sock.[ch]`、
`nina_dns.[ch]`、`nina_shell.c`、宣言の `nina_proto.h`、境界のヘッダー `nano_nina.h`・`nina_os.h` ほか）は
`common/drivers/nina/` にあり、3 版で共有します。`nina/` には Zephyr 版の部分だけが残ります（下の NINA の節）。下の表のファイル名はどちらの場所のものも含みます。

## LSM6DSO の FIFO ストリーム

Zephyr の Sensor Streaming API（`sensor_stream()`、RTIO）で FIFO を読み出せるようにしています。
トリガーは `SENSOR_TRIG_FIFO_WATERMARK`、オプションは `SENSOR_STREAM_DATA_INCLUDE` です。

| ファイル | 役割 |
|---|---|
| `lsm6dso_fifo.c` | FIFO ワードの解析（加速度・角速度・タイムスタンプ・その他、タグカウンター） |
| `lsm6dso_fifo_hw.c` | FIFO の設定、状態の読み取り、I²C のバースト読み出し |
| `lsm6dso_fifo_irq.c` | INT1（FIFO しきい値）の割り込み |
| `lsm6dso_stream.c` | ストリームの開始・読み出し・停止（専用のワークキュー、優先度 10） |
| `lsm6dso_fifo_decoder.c` | FIFO バッファ用の Zephyr decoder（`sensor stream` の表示に使用） |
| `lsm6dso_compat.c` | 通常の単発取得の decoder との切り替え |

### 動作

- 開始時に FIFO をバイパスにしてから、加速度・角速度のバッチレート、タイムスタンプの間隔、
  watermark を設定してストリームモードにします。
- INT1 でしきい値到達を受けると、FIFO の状態を読み、たまっているワードを 1 回のバーストで
  読み出して、1 つの RTIO バッファとして渡します。バッファの先頭には
  ヘッダー（ワード数、周期、感度、物理量への変換係数など）が付きます。
- ヘッダーの形式（`lsm6dso_fifo_packet.h`）はドライバー内だけのものです。バッファを読む側は、
  物理量が欲しければデコーダーを、生のワードが欲しければ `lsm6dso_fifo_buffer_words()` を
  使います。ワードは `lsm6dso_fifo_next_entry()` で 1 つずつ解釈でき、種類、xyz、タグ
  カウンター、タイムスタンプのワードが持つカウンター値が得られます。
- **FIFO オーバーラン**、I²C エラー、バッファ不足を検出すると、FIFO を止めてストリームを
  エラーで終えます。失われた個数は分からないので、連続しているように見えるデータは返しません。
  再開するかどうかは呼び出し側（`common/imu/imu_stream.c`）が決めます。
- **停止**は `lsm6dso_fifo_request_stop()` で要求します。バッチを止め、FIFO に残ったワードを
  渡し切ってから終端の完了通知を出します。RTIO の要求を直接キャンセルすると、読み出し済みの
  バッファが消費前に解放され得るためです。
- FIFO を使っている間、通常の単発取得と属性の変更は `-EBUSY` になります。

### 時刻

FIFO にはセンサーのタイムスタンプ（公称 25 µs 単位）を入れます。間隔は属性で変えられます。
タイムスタンプの有効化は開始時に行い、止めません（ストリームを再開してもカウンターは
進み続けます）。サンプルへの対応付けと PC 時刻への変換はアプリ側で行います
（[設計書](../../../../design/imu-raw-mqtt-design-ja.md) §7）。

`lsm6dso_timestamp_read()` は、センサーのタイムスタンプレジスタを、呼び出し側の時計の読み取りで
挟んで読みます。センサーの時計とボードの時計を対応付けるために使います。

decoder が返すサンプル時刻は、ホストの読み出し完了時刻と公称 ODR からの推定で、I²C の転送と
割り込み処理の遅れを含みます。正確な時刻ではありません。

### 属性

`SENSOR_CHAN_ALL` に対する独自属性です（`lsm6dso_attributes.h`）。どちらも次のストリーム開始時に
適用され、ストリーム中は変更できません。

| 属性 | シェルでの名前 | 値 | 初期値 |
|---|---|---|---|
| `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` | `fifo_watermark` | 1〜256 ワード | 32 |
| `SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION` | `fifo_timestamp_decimation` | 0、1、8、32 | 8 |

watermark を大きくすると読み出しの回数が減って負荷が下がりますが、データが届くまでの時間が
延び、FIFO（512 ワード）の余裕が減ります。

### 物理量への変換と校正

RAW から物理量への変換は `lsm6dso_cal.c` の 1 つの関数にまとめてあり、FIFO のデコーダー
（`sensor stream`）と単発読み出し（`sensor get`）の両方がこれを使います。

```text
out = M · (raw − bias)
```

- すべて整数演算です。`M` はスケールと軸のずれをまとめた 3×3 の行列、出力は Q31（shift 9）で、
  1 単位は 2⁻²² m/s² または rad/s です。
- 校正値がないときは、公称感度を対角に置いた行列とバイアス 0 で同じ関数を通します。
- 校正値は `lsm6dso_cal_set_lookup()` で登録した関数から得ます。ドライバーは登録時、
  `lsm6dso_cal_reload()` の呼び出し時、レンジの設定時に、現在の感度でこの関数を呼びます。
  該当がなければ公称値になります。KV からの読み出しはアプリ側（`common/imu/imu_cal.c`）が行います。
- FIFO のバッファには、読み出した時点の変換係数を入れます。デコーダーはバッファだけで
  変換でき、途中でレンジや校正値が変わっても、読み出し時の係数で変換されます。
- `lsm6dso_cal_status()` で、加速度・角速度それぞれが校正値を使っているかと、使用中の角速度の
  バイアスを取得できます。

角速度のバイアスは、静止している間のデータから取り直します（`lsm6dso_zero.c`）。

- FIFO から読んだサンプルを 1/8 秒ごとに区切り、8 区切り連続で角速度の平均（0.3 dps 以内）と
  加速度の平均（2 mg 以内）が揃っていれば静止とみなして、その窓の角速度の平均を取り込みます。
  最初の 20 回は単純平均、その後は 1/20 の指数平滑です。1 サンプルごとの値では判定しません。
  6667 Hz では 1 サンプルのノイズ（約 1.7 dps）が大きすぎるためです。
- FIFO のストリームが動いている間（`imu start`、`sensor stream`）は常に働きます。ドライバーが
  自分でセンサーを動かすことはありません。
- `lsm6dso_cal_zero()` は、その場で 1 回取り直します。センサーが止まっていれば 104 Hz で最長
  3 秒動かし、終わったら止めます。ストリームが FIFO を使っている間は `-EBUSY` を返します。
- 値は RAM にだけ持ち、変換の選び直し（レンジの設定、読み直し）で破棄します。保存してある
  バイアスから 1 dps（校正値がなければ 0 から 5 dps）以上離れた窓は採用しません。

### 統計

`lsm6dso_fifo_get_stats()` で、読み出したワード数、転送の回数と時間、I²C エラー、バッファ不足、
オーバーランの観測回数、設定 ODR、感度、周波数補正値を取得できます。統計は FIFO を開始する
たびにクリアされ、`generation` が 1 進みます。

### 制限

- 対応しているのは I²C 接続だけです。
- 内部温度の取得には対応していません（FIFO にも入れていません）。
- トリガーとセンサーハブ（`CONFIG_LSM6DSO_TRIGGER`、`CONFIG_LSM6DSO_SENSORHUB`）とは併用できません。

## I²C（`i2c_dw`）

RP2040 向けに次を変えています。他のコントローラーの処理は Zephyr のままです。

- RX・TX FIFO の深さを 16、RX しきい値を 4 バイトとし、短い読み出しと末尾ではしきい値を
  下げます。受信待ちで要求を追加できない間は TX_EMPTY 割り込みを抑えます。
- **DMA による読み出し**（`CONFIG_I2C_DW_RP2040_DMA`、既定で有効）。I²C0 の 7 ビットアドレスで、
  STOP で終わる最後の読み出しが 32〜3584 バイトの場合に使います。書き込み、短い読み出し、
  条件に合わない転送は割り込みで処理します。DMA チャネルの確保や設定に失敗した場合は、転送を
  始める前に割り込み方式へ切り替えます。

```text
i2c_dma mode          現在の方式を表示
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（累積）
```

FIFO の取得だけを行った過去の測定（6667 Hz、1 MHz、各 1 回）では、watermark 128 のとき
`cpu_active` が DMA なしで 45.7%、DMA ありで 22.5% でした。

## sensor シェル（`sensor_shell`）

- フレーム数が 0 のチャンネルと、キャンセル時などのバッファのない完了通知を表示から除きます。
- `fifo_watermark` と `fifo_timestamp_decimation` を `sensor attr_set` / `attr_get` で扱えます。

`sensor stream` と `imu start` は同時に使えません。

## NINA-W102（`nina`）

NINA-W102（ESP32）を、SPI 経由の Wi-Fi・ソケットとして Zephyr から使うためのドライバーです。
NINA 側は Arduino の NINA ファームウェア 3.0.2 に 2 件の修正を当てたものを前提にします
（[ファームウェアの説明](../../../../firmware/nina-fw/README.md)）。

### 役割分担

Wi-Fi と TCP/IP は NINA のファームウェア（lwIP）が処理します。RP2040 側は SPI のコマンドで
Wi-Fi の接続とソケットを操作するだけで、パケットは扱いません。Zephyr から見ると、Wi-Fi 管理
API とソケットのオフロードです。

NINA-W102 のドライバーの中身は、NuttX 版・FreeRTOS 版と共有するコア（`common/drivers/nina/`）です。
`nina/` には、コアの OS 側の境界と、Zephyr のソケット層への接着だけを置いています。

| ファイル | 場所 | 役割 |
|---|---|---|
| `nina_spi.c` | `common/drivers/nina/` | SPI のコマンド・応答のフレーミング、モジュールのリセット、READY の待ち |
| `nina_net.c` | `common/drivers/nina/` | ネットワークインターフェース、Wi-Fi の接続・切断・スキャン・状態、DHCP の結果（アドレス・ゲートウェイ）の反映、接続監視 |
| `nina_sock.[ch]` | `common/drivers/nina/` | ソケットの表と命令（待ちのループの 1 回分ずつ） |
| `nina_dns.[ch]` | `common/drivers/nina/` | 名前解決（モジュールのリゾルバー） |
| `nina_shell.c` | `common/drivers/nina/` | `nina version` コマンド |
| `nano_nina.c` | `nina/` | コアの境界 `nano_nina.h` の Zephyr 版：devicetree の `spi1` と 4 本の線（CS、READY、RESET、IRQ）、Zephyr の GPIO・SPI の呼び出し |
| `nina_os.c` | `nina/` | コアの境界 `nina_os.h` の Zephyr 版：mutex、sleep、時計、READY の割り込み待ち |
| `nina_socket.c` | `nina/` | ソケットのオフロード（ファイル記述子、errno、`O_NONBLOCK`、`SO_RCVTIMEO`、待ちのループと sleep、poll）と、名前解決を `getaddrinfo` のオフロードとして登録 |
| `posix/` | `nina/` | コアが使う POSIX のソケットの名前（`struct sockaddr_in`、`AF_INET`、`POLLIN` など）を Zephyr の名前に対応させるヘッダー（コアと `nina_socket.c` だけが読む） |

Zephyr の標準機能のうち、Telnet シェル、MQTT クライアント、zperf がこのソケットの上で動きます。

### SPI のコマンド

- コマンドは 1 つずつ処理されます。RP2040 側は mutex で直列化します。SPI は 8 MHz です
  （`CONFIG_NINA_SPI_FREQUENCY`）。
- NINA の READY（GPIO10）の変化は、GPIO のエッジ割り込みで待ちます（`nina/nina_os.c`）。待つ前に
  割り込みを有効にしてから READY を読むので、読んだあとの変化を取りこぼしません。転送中は割り込みを
  止めます。1 回の待ちは最長 10 ms で、そのたびに READY を読み直します。期限（コマンドの前 2 秒、
  応答まで 10 秒（`nina_ip_config()` は 2 秒）、CS を有効にしたあと 20 ms）を過ぎると `-ETIMEDOUT` です。
- 1 回の TCP 送信は最大 3584 バイトです（`CONFIG_NINA_TCP_MAX_IO`）。NINA 側のコマンド
  バッファに収まる上限で、RAW バッチ 1 個（最大 2448 バイト）を 1 コマンドで送れます。

### ソケット

すべてのソケットを、ファームウェア 3.0.2 の BSD ソケット相当の命令（0x70〜0x7F：socket、
close、bind、listen、accept、connect、send、recv、sendto、recvfrom、ioctl、poll、getsockopt）で
扱います。各命令は NINA 内の lwIP のソケット関数を 1 回呼ぶだけで、ソケット同士は独立して
います。以前使っていた種類別のクライアント命令は使いません。

- **枠は 10 個**です。NINA の lwIP の上限（`CONFIG_LWIP_MAX_SOCKETS=10`）に合わせています。
  Telnet の待ち受け、MQTT、NTP、zperf が共有します。Zephyr 側のファイル記述子の表は
  `CONFIG_ZVFS_OPEN_MAX=16` で確保しています。既定では `CONFIG_NET_MAX_CONTEXTS` から
  大きさが決まるので、その値だけを減らすと表が足りなくなり、Telnet の受け付けが失敗し続けます。
- Zephyr 自前の TCP/IP は通信を扱いません（`net conn` は常に空です）。そのためのパケット
  バッファと接続の管理表は、`prj.conf` で最小にしています。
- **必ず非ブロッキング**にします。命令は NINA の SPI ハンドラーの中で実行されるので、
  ブロックする呼び出しは Wi-Fi の管理を含む全体を止めます。ソケットは作成直後に
  非ブロッキングにし、接続完了・受信・送信可能の待ちは、ドライバー側で poll 命令を
  20 ms 間隔で繰り返します。
- **失敗と「0 バイト」は応答で区別できません**。受信は先に poll で読めることを確かめます
  （読めるのに 0 バイトなら相手が閉じた）。TCP 送信で 0 が返ったときは、100 ms 間隔で poll の
  エラーフラグを見て切断を判定します。接続の成否は SO_ERROR で確認します。
- **TCP の接続**は最長 10 秒待ちます。送信は、送信バッファが空かない状態が 10 秒続くと
  エラーにします。AP が消えたときの切断検出に約 9 秒かかるのはこのためです。
- **UDP** は 1 データグラム 512 バイトまでです。受信バッファに収まらない分は lwIP が捨てます
  （データグラムの境界は保たれます）。`connect` した UDP ソケットは、他の送信元からの
  データグラムをドライバーで読み捨てます。
- `poll` は、1 つのソケットでモジュールへの問い合わせが失敗しても呼び出し全体を失敗させず、そのソケットに
  `POLLERR` を返します。待ちの途中で別のスレッドが閉じたソケットには `POLLNVAL` を返します。
- `bind` は任意アドレス（0.0.0.0）のみ、IPv4 のみです。マルチキャストには
  対応していません。

### Wi-Fi

- 2.4 GHz のオープンと WPA2-PSK に対応しています。オープン接続は実機で試していません。
- 接続時と再接続時に NINA の省電力を OFF にします。省電力が有効だと、送信が数百 ms 単位で
  止まることがあるためです。設定の成否を確認するには、修正版のファームウェアが必要です。
- 認証情報は Zephyr の `wifi cred`（Settings/NVS）に保存します。起動時と予期しない切断の
  あとは、保存済みの設定で接続を試み続けます。`wifi disconnect` のあとは再接続しません。
  複数の設定を登録した場合、どれに繋がるかは保証しません。
- スキャン中は NINA が Wi-Fi を止めるため、接続は一時的に切れます。結果は最大 10 件です。

### 実機で確認した範囲

各 1 回ずつの確認です。

- NTP（UDP）、Telnet（TCP の待ち受け）、MQTT（TCP クライアント）の動作。
- 6667 Hz の RAW 送信（MQTT）と Telnet を動かしたまま、UDP のデータグラム 360 個を往復させて、
  欠落・破損がなく、MQTT 側にも欠番がないこと。
- AP の停止・再起動のあとの自動再接続と、アドレス・ゲートウェイの再取得。

- zperf（PC 側は iperf3、各 30 秒）：TCP 送信 2.93 Mbit/s、TCP 受信 1.54 Mbit/s、UDP 送信は
  500 kbit/s の指定で欠落 0。

確認していないもの：TCP クライアントを複数同時に開く場合、10 ソケット同時、接続先が応答
しないときのタイムアウトの経路。

ソケットオプション（`setsockopt`）は 2 つだけ受け付けます。

- 受信タイムアウト（`SO_RCVTIMEO`）：ドライバー内の受信待ちに反映します。NINA には渡しません。
- `TCP_NODELAY`：受け付けますが、**効果はありません**。NINA の命令では TCP 層のオプションを
  設定できないため、まとめ送り（Nagle）は有効なままです。iperf3 のクライアントが制御用の接続に
  この設定を要求し、失敗すると中止するので、受け付けだけ行っています。

それ以外のオプションは `ENOPROTOOPT` を返します。

### TLS について

TLS には対応していません。検討の結果、次の点が未解決です。

- **乱数源**：ビルドは Zephyr のテスト用の非暗号乱数を使っています。RP2040 には暗号用の
  乱数源がありません。ボード上の ATECC608A は設定とデータが未ロックで、Random コマンドの
  応答が 2 回同一だったため、この状態では使えません。ESP32 のハードウェア乱数を SPI 経由で
  取得するには、NINA のファームウェアに命令を追加する必要があります。
- **証明書の検証**：NINA 側の TLS を使う方法は、IP アドレス指定の接続ではホスト名を
  検証できず、証明書の日付検証も有効になっていません。
- **時刻**：証明書の有効期限の検証には信頼できる時刻が要りますが、NTP の応答は認証されません。

Zephyr 側の TLS を NINA の TCP ソケットの上で動かし、テスト用の乱数と一時的な証明書で
MQTT の接続と送信ができること、ホスト名の不一致・期限切れ・信頼されない CA で接続が
失敗することは、以前に確認しています。これは安全でない乱数を使った動作確認であり、
本番で使える TLS ではありません。乱数源がない場合に、テスト用の乱数へ黙って切り替える
ような実装にはしません。

### 無線区間の調査

送信の遅れや ACK の挙動を無線フレームの段階で調べる手順は、
[Wi-Fi モニターによる TCP ACK 調査手順](../../../../design/wifi-monitor-debug-ja.md) にあります。

## ブート ROM の `memcpy`・`memset`

`CONFIG_NANO_ROM_MEM_OPS`（アプリの Kconfig、既定で有効）で、`memcpy`・`memset`・`__aeabi_memcpy*`・`__aeabi_memset*`
をリンカーの `--wrap` で RP2040 のブート ROM のルーチンへ回します（Pico SDK の `pico_mem_ops` と同じ使い分け。
NuttX 版と共有の `common/drivers/rp2040_rom/`）。無効にすると picolibc のバイト単位のループです。ROM のルーチンは
初回の呼び出しで引いて `.bss` に置くので、`.bss` のゼロ化と `.data` のコピーを行う `arch_early_memset`・`arch_early_memcpy`
は picolibc のものを使います（`src/rom_mem_early.c`）。ホストテストは `common/tests/` の `rp2040_rom_mem`（偽の関数表）。

## テスト

`zephyr/script/host_tests.sh` が、偽の Zephyr ヘッダーを使ってドライバーの次の部分を PC 上で
テストします。`nina_boundary_*` と `nina_socket` 以外は NuttX 版・FreeRTOS 版と共有するテスト（`common/tests/`、`common/tests/run.sh` が実行）です。

| テスト | 対象 |
|---|---|
| `fifo_words` | FIFO ワードの解析 |
| `fifo_hw` | FIFO の設定手順と失敗時の巻き戻し、状態の読み取り |
| `fifo_decoder` | FIFO バッファの decoder |
| `stream_stop` | ストリームの停止、キャンセル、エラー時の終了 |
| `i2c_dma_commands` | `i2c_dma` コマンドの引数の解釈 |
| `nina_spi`、`nina_spi_wait_ready` | NINA の SPI フレーミング、応答の解析、READY の期限（共有のコア `nina_spi.c`。偽の境界） |
| `nina_sock`、`nina_dns`、`nina_dns_offload` | NINA のソケットの命令、名前解決（`nina_dns_offload` はこの版と同じ `NINA_OS_DNS_OFFLOAD` の形） |
| `nina_boundary_irq`、`nina_boundary_poll` | 共有の `nina_spi.c` をこの版の `nina/nano_nina.c`・`nina_os.c` に載せたもの：フレーミング、線と設定の対応、リセットの手順、READY の割り込み待ち（コールバックの登録、セマフォのリセット、エッジの向き、転送前の無効化、10 ms の上限）とポーリング |
| `nina_socket` | ソケットのオフロード `nina/nina_socket.c` を共有のコアに載せたもの：ファイル記述子、待ちのループと sleep、オプション、accept、sendmsg、データグラム、poll、リゾルバーの登録 |
