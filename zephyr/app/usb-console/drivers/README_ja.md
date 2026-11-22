# ローカルドライバー

[English](README.md) | 日本語

`zephyr/app/usb-console` は、Zephyr のドライバーの一部を Zephyr のツリーへのローカルのパッチで変えて使います。
IMU の FIFO を高いレートで読み出すための変更と、NuttX 版・FreeRTOS 版と共有する LSM6DSOX と NINA-W102 のドライバーがあります。

| ドライバー | 内容 | 元 |
|---|---|---|
| LSM6DSO（`lsm6dso`） | FIFO ストリーム、校正値を使う変換、静止時の角速度のバイアス | `common/drivers/lsm6dso/`（Zephyr の `drivers/sensor/st/lsm6dso` の代わり） |
| sensor シェル（`drivers/sensor/sensor_shell.c`） | 修正と LSM6DSO の FIFO の属性 | Zephyr v4.5.0 に `zephyr/0006-sensor-shell-fifo-attributes.patch` |
| DesignWare I²C（`drivers/i2c/i2c_dw.c`） | RP2040 の FIFO の段数、I2C0 の DMA 読み出し | Zephyr v4.5.0 に `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`（`i2c scan` は `0004`） |
| NINA-W102（`nina`） | SPI 経由の Wi-Fi とソケット | `common/drivers/nina/` |

Zephyr v4.5.0 はコミット `41d10b59730406bbab4ca9f1fbbe9e96aafb4966` です。各パッチは、そのドライバーに要るものを
すべて Zephyr のツリーの中に持ちます（下の「Zephyr へのローカルのパッチ」）。アプリの `CMakeLists.txt` はドライバーの
ライブラリーにソースもインクルードパスも足しません。このディレクトリーにはこの文書だけがあります。

LSM6DSO のドライバーは `common/drivers/lsm6dso/` にあり、3 版で共有します。OS を持たないコアは直下にあります:
`lsm6dso.c`（チップの操作: 起動とリセット、レートとレンジ、単発の読み出しとその変換、FIFO のレジスターの操作・統計・所有、
タイムスタンプの読み取り、使用中の校正とバイアスの追従）、`lsm6dso_stream.c`（FIFO ストリームの状態機械。下）、
`lsm6dso_cal.c`（変換の計算と追従の計算）、`lsm6dso_fifo.c`（FIFO のワードの解析）。ヘッダーは `include/lsm6dso_cal.h`・
`include/lsm6dso_fifo.h`（ドライバーを使う側のもの）と `internal/lsm6dso_core.h`・`internal/lsm6dso_stream.h`・
`internal/lsm6dso_fifo_packet.h`（ストリームのバッファの形式）です。コアはチップへ ST の `lsm6dso_reg.c`（hal_st）を通して、
それ以外へ `port/` の 2 つの境界を通してつながります: OS への `lsm6dso_os.h`（2 種類のロック、待ち、サイクルカウンター、FIFO の
長い読み出し、ログ。ストリームのために、ドライバー専用のスレッドの上のワーク、INT1 の線、バッチの時刻の時計）と、ストリームを
使う側への `lsm6dso_stream_req.h`（受け持つ要求、バッチごとのバッファ、バッチかエラーを返す先）です。Zephyr へのつなぎは
`common/drivers/lsm6dso/port/zephyr/` で、デバイスと sensor API（`lsm6dso_sensor.c`・`lsm6dso_sensor.h`）、デバイスツリーの
デバイス（`lsm6dso_instance.h`）、ストリームの RTIO 側（下）、コアの境界の OS 側（`lsm6dso_os_impl.h` と `lsm6dso_os.c`:
`k_mutex`・`k_spinlock`・`k_busy_wait()`・`k_msleep()`・`k_usleep()`・`k_cycle_get_32()`・`i2c_burst_read_dt()`・ワークキュー
`imu_fifo_worker` の上の遅延つきワーク・システムのワークキューを通す INT1 の GPIO の割り込み・センサーの時計・Zephyr のログ）、
使う側の実装（`lsm6dso_stream_req_impl.h`: RTIO のマルチショットの要求）があります。NuttX 版・
FreeRTOS 版は同じつなぎを Zephyr 互換層の上でビルドし、`lsm6dso_instance.h` だけは自分のもの（`port/nuttx/`・
`port/freertos/`）を使います。つなぎは独立した Zephyr のモジュールです（`port/zephyr/zephyr/module.yml`。アプリの
`CMakeLists.txt` が `ZEPHYR_EXTRA_MODULES` に足す）: その `Kconfig` にドライバーのオプション `CONFIG_LSM6DSO_LOCAL`
（`st,lsm6dso` か `st,lsm6dso32` のノードがあれば既定で有効。ST のレジスターのドライバーと Zephyr の stmemsc のために
`HAS_STMEMSC`・`USE_STDC_LSM6DSO` を選ぶ）があり、`prj.conf` は同じデバイスの Zephyr のドライバーを止め（`CONFIG_LSM6DSO=n`）、
その `CMakeLists.txt` がライブラリー `lsm6dso_local` を作って拡張 API `<zephyr/drivers/sensor/lsm6dso.h>`
（`port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h`。`include/lsm6dso.h` を読む）を公開します。devicetree binding は
Zephyr の `st,lsm6dso` です。アプリが読むのは拡張 API だけです。FIFO のワードとその解析（`lsm6dso_fifo_next()`・
`lsm6dso_fifo_next_entry()`・`lsm6dso_fifo_buffer_words()`）、校正の型と呼び出し（`lsm6dso_cal_set_lookup()`・`_reload()`・
`_status()`・`_zero()`）、FIFO の属性・設定・統計・停止、タイムスタンプの読み取りがあります。

パッチ 0006 を当てた `sensor_shell.c` は、ほかの版と同じファイルです（`common/drivers/sensor_shell/`。ホストテスト
`zephyr_sensor_shell_patched` が確かめる）。それが最初に読む `drivers/sensor/sensor_shell_os.h` はこの版のもの（何もしない
マクロ 2 つ。ほかの版は `sensor get` が結果の表示を待つ）です。

NINA のコアは `common/drivers/nina/`（直下に、ヘッダーは `include/` に、ドライバーが提供する機能 `nina_wifi.[ch]`・`nina_sock.[ch]`・
`nina_dns.[ch]`・`nina_passthrough.[ch]`、`internal/` にプロトコルの `nina_proto.[ch]` と待ちの `nina_wait.[ch]`、`port/` に境界のヘッダー `nina_bus.h`・`nina_os.h` と中継の 2 つ）に、`nina` コマンドの `nina_shell.c` は `common/shell/` にあり、3 版で共有します。コアの Zephyr の API へのつなぎは
`common/drivers/nina/port/zephyr/` にあり（FreeRTOS 版はその Wi-Fi とソケットの部分 `nina_net.c`・`nina_socket.c` もビルドし、`nina_os.c` は自分のもの `common/drivers/nina/port/freertos/` を使う）、このディレクトリーには NINA のドライバーのものはありません（下の NINA の節）。下の表のファイル名はそのすべての場所のものを含みます。

## LSM6DSO の FIFO ストリーム

Zephyr の Sensor Streaming API（`sensor_stream()`、RTIO）で FIFO を読み出せるようにしています。
トリガーは `SENSOR_TRIG_FIFO_WATERMARK`、オプションは `SENSOR_STREAM_DATA_INCLUDE` です。

| ファイル | 役割 |
|---|---|
| `lsm6dso.c`（コア） | FIFO の所有とロック: レジスターの設定・一時停止・停止、状態の読み取りとバースト読み出し（static）、しきい値の INT1 への割り当て、統計、タイムスタンプの間引き、タイムスタンプの読み取り。単発の読み出しとその変換、`lsm6dso_cal_zero()` のバイアスの読み取り |
| `lsm6dso_stream.c`（コア） | ストリームの状態機械: 最初の実行での開始（FIFO、INT1 の有効化と割り当て）、INT1 のエッジごとと 20 ms ごとの確認での watermark 分のバッチの読み出し、穏やかな停止、エラー・オーバーラン・使う側のキャンセルでの終了。watermark |
| `lsm6dso_fifo.c`（コア） | FIFO のワードの解析（加速度・角速度・タイムスタンプ・その他、タグカウンター） |
| `lsm6dso_cal.c`（コア） | 物理量への変換と静止時の角速度のバイアス（下）。`lsm6dso.c` とデコーダーが使う |
| `lsm6dso_sensor.c`（つなぎ） | コアの上の上流のドライバー: デバイス、属性、`sample_fetch` と校正値を使う `channel_get`、拡張 API のうちデバイスを取る呼び出し |
| `lsm6dso_rtio_stream.c`（つなぎ） | デバイスごとのストリーム: デバイスツリーの INT1 の線、RTIO の要求をコアのストリームへ渡す（ほかのトリガーは `-ENOTSUP`、2 つ目のストリームは `-EBUSY`）、拡張 API の停止・watermark・タイムスタンプの間引き |
| `lsm6dso_os.c`（つなぎ） | ストリームのワークキュー（`imu_fifo_worker`、優先度 10、2048 バイト）、INT1 の GPIO の割り込み（システムのワークキューを通してストリームを起こす） |
| `lsm6dso_rtio.c`（つなぎ） | ドライバーの RTIO の submit。ストリームは `lsm6dso_rtio_stream.c` へ、単発取得（`sensor get`）は sensor API のフォールバックの写しで RTIO のワークキューへ |
| `lsm6dso_decoder.c`（つなぎ） | `lsm6dso_fifo_buffer_words()`、FIFO バッファ用のデコーダー（`sensor stream` の表示に使用）と、単発取得のバッファを Zephyr の既定のデコーダーに渡すデコーダー |

つなぎのファイル名は、上流と、FIFO ストリームを持つ同系のドライバー `lsm6dsv16x` に合わせています。INT1 の割り込みはドライバー
自身のもの（`lsm6dso_os.c` の中）で、線はストリームの間だけ有効、システムのワークキューはストリームのワークキューを
起こすだけで、I²C の読み出しはすべてストリームのワークキューが行います。上流のトリガー（`CONFIG_LSM6DSO_TRIGGER`、
`lsm6dso_trigger.c`）、センサーハブ（`CONFIG_LSM6DSO_SENSORHUB`、`lsm6dso_shub.c`）、温度の読み取り（`CONFIG_LSM6DSO_ENABLE_TEMP`）は
このドライバーにはありません。

### コアの境界とその負担

コアは自分のオブジェクト `struct lsm6dso` と `struct lsm6dso_stream`（どちらもつなぎの `struct lsm6dso_data` の中）を受け取ります。
つなぎは単発の読み出しと設定の前後でコアの FIFO のロックを取ります（`lsm6dso_acquire()`・`_release()`、static inline）。コアの関数は
`lsm6dso_*()` と `lsm6dso_stream_*()` です。拡張 API の呼び出し（デバイスを取り、名前はそのまま）や `lsm6dso_cal.h` の関数と
同じ名前になるものは、動詞を先にします: `lsm6dso_set_cal_lookup()`・`lsm6dso_reload_cal()`・`lsm6dso_get_cal_status()`・
`lsm6dso_measure_gyro_bias()`（`lsm6dso_cal_zero()` の本体）・`lsm6dso_track_gyro_bias()`・`lsm6dso_copy_cal()`・
`lsm6dso_get_fifo_stats()`・`lsm6dso_set_timestamp_decimation()`・`lsm6dso_get_timestamp_decimation()`・`lsm6dso_read_timestamp()`。

境界 `port/lsm6dso_os.h`・`port/lsm6dso_stream_req.h` は、OS の `lsm6dso_os_impl.h` と使う側の `lsm6dso_stream_req_impl.h` が
定義する static inline の関数でできていて、ロック・サイクルカウンター・バスの読み出し・ワーク・RTIO は Zephyr の関数を直接呼ぶのと
同じ呼び出しになります。普通の関数は INT1 の有効化と無効化（ストリームごとに 1 回。`lsm6dso_os.c`）だけです。OS はコアを
`lsm6dso_stream_run()`（ワーク）と `lsm6dso_stream_int1()`（エッジ）で呼び返します。バスは ST の `stmdev_ctx_t` です。その
`read_reg` は 255 バイトまでしか読めない（stmemsc の長さが `uint8_t`）ので、FIFO のデータ（最大 3584 バイト）・状態・タイムスタンプは
`lsm6dso_os_bus_read()`（コンテキストの先の I²C のデバイスへの `i2c_burst_read_dt()`）で読みます。ほかのバスではストリームを
始めません（`lsm6dso_os_bus_read_supported()`）。バッチ 1 つ（watermark ごと。6667 Hz・128 ワードで毎秒約 104 回）の流れは次のとおり
です: GPIO の割り込みが INT1 のワークをシステムのワークキューに入れ、それが `lsm6dso_stream_int1()` を呼び、ストリームのワークを
すぐ実行するように入れ直します。ワークキュー `imu_fifo_worker` がハンドラー（呼び出し 1 段）から `lsm6dso_stream_run()` を呼び、
ストリームはスピンロックの下で RTIO の要求を引き、コアの `lsm6dso_fifo_status()`・`lsm6dso_fifo_read()`・
`lsm6dso_track_gyro_bias()`・`lsm6dso_copy_cal()` を呼びます。RTIO のバッファと完了、センサーの時計、次の確認の予約はインラインです。
Zephyr 互換層を持つ新しい OS は、このドライバーのために何も書きません。互換層を持たない OS なら、2 つの `_impl.h` と INT1 の 2 つの
関数を書きます。

### 動作

- 開始時に FIFO をバイパスにしてから、加速度・角速度のバッチレート、タイムスタンプの間隔、
  watermark を設定してストリームモードにします。
- INT1 でしきい値到達を受けると、FIFO の状態を読み、たまっているワードを 1 回のバーストで
  読み出して、1 つの RTIO バッファとして渡します。バッファの先頭には
  ヘッダー（ワード数、周期、感度、物理量への変換係数など）が付きます。
- ヘッダーの形式（`internal/lsm6dso_fifo_packet.h` の `struct lsm6dso_fifo_packet`）はドライバー内だけのものです。バッファを読む側は、
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

`SENSOR_CHAN_ALL` に対する独自属性です（`<zephyr/drivers/sensor/lsm6dso.h>`）。どちらも次のストリーム開始時に
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

角速度のバイアスは、静止している間のデータから取り直します（`lsm6dso_cal.c` の `lsm6dso_zero_*()`）。

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

## I²C（`i2c_dw`）

Zephyr の DesignWare I²C ドライバー（`drivers/i2c/i2c_dw.c`）に、ローカルのパッチ `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`
を当てて使います（下の「Zephyr へのローカルのパッチ」）。ドライバーに要るものはすべてパッチが Zephyr のツリーの中に置き、
アプリはドライバーのライブラリーにソースもインクルードのパスも足しません。他のコントローラーの処理は、どのコントローラーにも
当たる下の 2 点のほかは Zephyr のままです。

**どのコントローラーでも**: 0 バイトのメッセージを含む転送は、どのメッセージも送る前に `-ENOTSUP` で断ります（コントローラーは
0 バイトの転送ができず、上流は代わりに 1 バイトを送ります）。シェルの `i2c scan` はその書き込みで確かめるので、最初のアドレスで
止まり、`I2C: i2c@40044000 does not support this scan (zero-length writes)` を表示して `-ENOTSUP` を返します（ローカルの
パッチ `zephyr/0004-i2c-shell-scan-not-supported.patch`。その書き込みを断るどのコントローラーでも同じ。ほかのエラー、つまり
NACK は従来どおり `--`）。1 つのアドレスは `i2c direct_read i2c@40044000 6a 1` で確かめます。`i2c_recover_bus()`（シェルの
`i2c recover`）は `-ENOTSUP` を返します（このボードにはバスの復旧が無い）。転送の前には、上流と同じく、バスが固まった状態を
消して転送に進みます。

**RP2040 のコントローラーへの変更**: RX・TX FIFO の深さを 16、RX しきい値を 4 バイトとし、短い読み出しと末尾ではしきい値を
下げます。受信待ちで要求を追加できない間は TX_EMPTY 割り込みを抑えます。

**DMA による読み出し**（`CONFIG_I2C_DW_RP2040_DMA`。パッチでは既定で無効、`prj.conf` で有効）:

- I²C0 だけで使います。7 ビットアドレスで、STOP で終わる最後の読み出しが 32〜3584 バイトの場合です。書き込み、
  短い読み出し、条件に合わない転送は割り込みで処理します。
- 1 回の読み出しに、Zephyr の DMA ドライバー（`dma_rpi_pico.c`）のチャネルを 2 つ使います。1 つが読み出しのコマンド語を
  `IC_DATA_CMD` に書き、もう 1 つがそこからバイトを受け取ります。チャネルの確保や設定に失敗した場合は、転送を始める前に
  割り込み方式へ切り替えます。
- DMA の処理は、読み出しのコマンド語を作る関数 `i2c_dw_rp2040_read_commands()` も含めて
  `drivers/i2c/i2c_dw_rp2040_dma_priv.h`（`i2c_dw.c` が 1 回だけ読む）にあります。コマンド語とバッファー（合わせて
  10,752 バイト）、セマフォ、事象、チャネルは I²C0 だけのためのドライバーの静的変数です。インスタンスごとのデータに
  持つのは、DMA の読み出し中かどうかと、コントローラーを隔離したかどうかだけです。

**`i2c_dma` コマンド**は、3 つの版で共有する `common/shell/i2c_dma_shell.c` をアプリに入れたものです。パッチが
`include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` で宣言する 3 つの関数（方式の取得・設定、カウンターの取得）を呼びます。

```text
i2c_dma mode          現在の方式を表示
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（累積）
```

**ほかの版との関係**: NuttX 版・FreeRTOS 版の I²C のコア（`common/drivers/i2c_dw/`）は、このパッチ後のドライバーの写しです。
両者のずれはホストテストの `i2c_dw_sync` が検出します（仕組みは[設計書](../../../../design/imu-raw-mqtt-design-ja.md)の §12、
Zephyr を上げるときの手順は下の「Zephyr へのローカルのパッチ」）。

FIFO の取得だけを行った過去の測定（6667 Hz、1 MHz、各 1 回）では、watermark 128 のとき
`cpu_active` が DMA なしで 45.7%、DMA ありで 22.5% でした。

## sensor シェル（`sensor_shell`）

Zephyr の `drivers/sensor/sensor_shell.c` にローカルのパッチ `zephyr/0006-sensor-shell-fifo-attributes.patch` を当てたものです。

- フレーム数が 0 のチャンネルと、キャンセル時などのバッファのない完了通知を表示から除きます。
- `fifo_watermark` と `fifo_timestamp_decimation` を `sensor attr_set` / `attr_get` で扱えます。

`sensor stream` と `imu start` は同時に使えません。

## NINA-W102（`nina`）

NINA-W102（ESP32）を、SPI 経由の Wi-Fi・ソケットとして Zephyr から使うためのドライバーです。
NINA 側は Arduino の NINA ファームウェア 3.0.2 にパッチ 0001〜0006 を当てたものを前提にします
（[ファームウェアの説明](../../../../firmware/nina-fw/README.md)）。0003〜0006 の無いファームウェアでも、その機能を
使わずに動きます（下の「ファームウェアの追加機能」）。

### 役割分担

Wi-Fi と TCP/IP は NINA のファームウェア（lwIP）が処理します。RP2040 側は SPI のコマンドで
Wi-Fi の接続とソケットを操作するだけで、パケットは扱いません。Zephyr から見ると、Wi-Fi 管理
API とソケットのオフロードです。

NINA-W102 のドライバーの中身は、NuttX 版・FreeRTOS 版と共有するコア（`common/drivers/nina/`。OS の定義を持たない）です。
コアの Zephyr の API へのつなぎは `common/drivers/nina/port/zephyr/`（FreeRTOS 版はその Wi-Fi とソケットの部分 `nina_net.c`・`nina_socket.c` を、Zephyr のソケット層をソケットの実装の一覧のところまで持つ互換層の上でビルドし、カーネルの部分は
FreeRTOS と Pico SDK で書いた自分のつなぎ `common/drivers/nina/port/freertos/nina_os.c` を使う）です。RP2040 に固有のもの（esptool の中継のモジュール側 `nina_passthrough_bus_rp2040.c`）も
つなぎと同じ場所にあります。アプリの `CMakeLists.txt` がそこからビルドし、つなぎの devicetree binding は `DTS_ROOT` で、オプションはアプリの
`Kconfig`（`rsource`）で読み込みます。

| ファイル | 場所 | 役割 |
|---|---|---|
| `nina_proto.c` | `common/drivers/nina/internal/` | SPI のコマンド・応答のフレーミング、モジュールのリセット、READY の待ち |
| `nina_wifi.[ch]` | `common/drivers/nina/`（ヘッダーは `include/`） | Wi-Fi の操作：接続（接続と DHCP の結果の待ちを含む）・切断・スキャン・状態・接続の監視。スレッド `nina_scan_tid`・`nina_connect_tid`・`nina_monitor_tid` の本体。起きたことはつなぎのコールバックに渡す |
| `nina_sock.[ch]` | `common/drivers/nina/`（ヘッダーはつなぎ向けで `internal/`） | ソケットの表と命令（待ちのループの 1 回分ずつ） |
| `nina_dns.[ch]` | `common/drivers/nina/`（ヘッダーは `include/`） | 名前解決（モジュールのリゾルバー） |
| `nina_proto.h` | `common/drivers/nina/internal/` | プロトコルのバイトの名前（フレーミング、ドライバーが送るコマンド、その応答の値）と、`nina_proto.c` の要求の関数・モジュールの機能ビットの宣言 |
| `nina_shell.c` | `common/shell/` | `nina version` コマンド（この版では `nina passthrough` も。下の「esptool の中継」） |
| `nina_passthrough.[ch]` | `common/drivers/nina/`（ヘッダーは `include/`） | esptool の中継（NuttX 版・FreeRTOS 版と共有）：再起動の印、DTR・RTS とモジュールの線の対応、レート、16 KiB のバッファーを通した両方向のデータ、出口、数、LED。2 つの境界 `nina_passthrough_bus.h`（モジュール側）と `nina_passthrough_os.h`（ホスト側）の上 |
| `nina_wait.[ch]` | `common/drivers/nina/internal/` | ソケットの待ちのループの、パスの間の待ち（間隔、または IRQ の線のエッジ） |
| `nina_net.c` | `common/drivers/nina/port/zephyr/` | コアの上の Zephyr の Wi-Fi 管理とネットワークインターフェース：デバイス（`NET_DEVICE_DT_OFFLOAD_DEFINE()`）、Zephyr のパラメーターの確認と受け渡し、結果を Zephyr の構造体とイベントに、アドレス・ネットマスク・ゲートウェイと dormant をインターフェースに、Wi-Fi のプロファイルの再接続をコアのフック、3 つのスレッド（`K_THREAD_DEFINE()`） |
| `nina_os.c` | `common/drivers/nina/port/zephyr/` | コアの境界 `nina_os.h` の Zephyr 版：mutex、Wi-Fi のスレッドへの要求、sleep、時計、READY の割り込み待ち、IRQ の線（GPIO2）のエッジの待ち |
| `nina_socket.c` | `common/drivers/nina/port/zephyr/` | ソケットのオフロード（ファイル記述子、errno、`O_NONBLOCK`、`SO_RCVTIMEO`、待ちのループと sleep、poll）と、名前解決を `getaddrinfo` のオフロードとして登録（FreeRTOS 版のソケットも） |
| `nina_passthrough_os.c` | `common/drivers/nina/port/zephyr/` | 中継の境界 `nina_passthrough_os.h` の Zephyr 版：CDC ACM のポートと USB スタックの通知、1 ms の tick、起動（`SYS_INIT`）と `main()` での出口 |
| `nina_passthrough_bus_rp2040.c` | `common/drivers/nina/port/zephyr/` | 中継のモジュール側（`nina_passthrough_bus.h`）を RP2040 のレジスターで：watchdog の scratch レジスターと再起動、UART1 とその割り込み、GPIO2・GPIO3・GPIO8、LED |
| `posix/` | `common/drivers/nina/port/zephyr/` | コアが使う POSIX のソケットの名前（`struct sockaddr_in`、`AF_INET`、`POLLIN` など）を Zephyr の名前に対応させるヘッダー（コアと `nina_socket.c` だけが読む） |
| `Kconfig`、`dts/bindings/wifi/arduino,nina-spi.yaml` | `common/drivers/nina/port/zephyr/` | ドライバーのオプション（`CONFIG_WIFI_NINA_LOCAL`、`CONFIG_NINA_SPI_FREQUENCY`、`CONFIG_NINA_TCP_MAX_IO`、`CONFIG_NINA_SEND_RETRY_MS`）と `nina_w102` のノードの devicetree binding |
| `nina_bus.c` | `common/drivers/nina/port/zephyr/` | コアの境界 `nina_bus.h` の Zephyr 版：devicetree の `spi1` と 4 本の線（CS、READY、RESET、IRQ）、Zephyr の GPIO・SPI の呼び出し。`CONFIG_NINA_PASSTHROUGH` のとき、`nina_passthrough_active()` が真の間（esptool の中継）はバスなし |

Zephyr の標準機能のうち、Telnet シェル、MQTT クライアント、zperf がこのソケットの上で動きます。

### SPI のコマンド

- コマンドは 1 つずつ処理されます。RP2040 側は mutex で直列化します。SPI は 8 MHz です
  （デバイスツリーの NINA のノードの `spi-max-frequency`。SPI のバスと CS もそのノードの親と `reg` から取ります）。
- NINA の READY（GPIO10）の変化は、GPIO のエッジ割り込みで待ちます（`common/drivers/nina/port/zephyr/nina_os.c`）。待つ前に
  割り込みを有効にしてから READY を読むので、読んだあとの変化を取りこぼしません。転送中は割り込みを
  止めます。1 回の待ちは最長 10 ms で、そのたびに READY を読み直します。期限（コマンドの前 2 秒、
  応答まで 10 秒（`nina_proto_ip_config()` は 2 秒）、CS を有効にしたあと 20 ms）を過ぎると `-ETIMEDOUT` です。
  割り込みを使うかどうかは実行時に決まります。最初のリセットのあと `nina_os_ready_start()` が READY の
  コールバックを登録し、登録に失敗したときは 100 µs ごとに READY を読みます（期限と結果は同じ）。
- 1 回の TCP 送信は最大 3584 バイトです（`CONFIG_NINA_TCP_MAX_IO`）。NINA 側のコマンド
  バッファに収まる上限で、RAW バッチ 1 個（最大 2448 バイト）を 1 コマンドで送れます。

### ソケット

すべてのソケットを、ファームウェア 3.0.2 の BSD ソケット相当の命令（0x70〜0x7F：socket、
close、bind、listen、accept、connect、send、recv、sendto、recvfrom、ioctl、poll、getsockopt）で
扱います。各命令は NINA 内の lwIP のソケット関数を 1 回呼ぶだけで、ソケット同士は独立して
います。

- **枠は 10 個**です。NINA の lwIP の上限（`CONFIG_LWIP_MAX_SOCKETS=10`）に合わせています。
  Telnet の待ち受け、MQTT、NTP、zperf が共有します。Zephyr 側のファイル記述子の表は
  `CONFIG_ZVFS_OPEN_MAX=16` で確保しています。既定では `CONFIG_NET_MAX_CONTEXTS` から
  大きさが決まるので、その値だけを減らすと表が足りなくなり、Telnet の受け付けが失敗し続けます。
- Zephyr 自前の TCP/IP は通信を扱いません（`net conn` は常に空です）。そのためのパケット
  バッファと接続の管理表は、`prj.conf` で最小にしています。
- **必ず非ブロッキング**にします。命令は NINA の SPI ハンドラーの中で実行されるので、
  ブロックする呼び出しは Wi-Fi の管理を含む全体を止めます。ソケットは作成直後に
  非ブロッキングにし、接続完了・受信・送信可能の待ちは、ドライバー側で poll 命令を
  20 ms 間隔で繰り返します（ファームウェアがソケットの事象を通知する場合は、IRQ の線のエッジで
  起きます。下の「ファームウェアの追加機能」）。
- **失敗と「0 バイト」は応答で区別できません**。受信は先に poll で読めることを確かめます
  （読めるのに 0 バイトなら相手が閉じた）。TCP 送信で 0 が返ったときは、1 ms 後に poll 命令で
  書き込み可能を確かめ、可能になってからデータを送り直します（全量を毎回 SPI で送り直すことは
  しません）。poll のエラーフラグで切断を判定します。接続の成否は SO_ERROR で確認します。
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
- ファームウェア 3.0.2 はスキャン中に Wi-Fi を止めるため、接続は一時的に切れます（ドライバーは切断として
  扱い、スキャンのあとで繋ぎ直します）。接続を保つファームウェアでは切れません（下の節）。結果は最大 10 件です。

### ファームウェアの追加機能（パッチ 0003〜0006）

`firmware/nina-fw/` のパッチ 0003〜0006 を当てたファームウェアは、命令 0x6A で追加機能のビットを返します
（仕様は `firmware/nina-fw/README.md` の「Host interface of 0003 to 0006」）。ドライバーはインターフェースの
初期化（MAC アドレスの読み出しのあと）で 1 回だけ尋ね、ビットごとに動作を切り替えます。3.0.2 など命令を持たない
ファームウェアは `0xEF 0x00 0xEE` と答えるので、3 バイトを読んだところで「機能なし」と判定し、追加機能を
使わずに動きます。`nina version` の 2 行目に、モジュールのビットと使っているビットを表示します。

| ビット | 機能 | ドライバーの動作 |
|---|---|---|
| 0 | 接続中のスキャンで接続を保つ | 接続中のスキャンを切断として扱いません（アドレスはそのまま、切断のイベントも繋ぎ直しもありません）。スキャン中に接続が切れたときは、接続監視（2 秒ごと）がほかの切断と同じく検出します |
| 1 | 非同期のスキャン | 0x36 で始め、0x6B を 100 ms ごとに尋ね、終わったら 0x27 で結果を読みます。NINA のロックは命令ごとにしか持たないので、スキャン中も送信は止まりません。10 秒たっても終わらなければ 0x27 を送らずに失敗（`-ETIMEDOUT`）にします。始められない・失敗は `-EIO`、接続や切断で止められたときは `-ECANCELED` です |
| 2 | ソケットの事象を IRQ の線で通知 | ソケットの待ち（connect、受信、`poll`、送信の再試行）が、20 ms（送信は 1 ms）の間隔の代わりに、GPIO2 の立ち上がりのエッジの割り込みで起きます（最長 100 ms で聞き直す） |

ビット 2 の待ち方（`common/drivers/nina/internal/nina_wait.h`）:

- ファームウェアの規則どおり、事象を登録しうる命令（poll、recv、accept、send、connect）の前にエッジの数を控え、
  「まだ」の答えのあとは、数が変わるか 100 ms たつまで待って尋ね直します。命令の前のエッジは 1 回余分に
  尋ねるだけで、取りこぼしたエッジは最長 100 ms の遅れになるだけです（正しさは尋ね直しが保ちます）。
- 線はすべてのソケットで共通なので、エッジのたびに待っているすべてのスレッドが起きます。起きても何も
  見つからないことが続いたスレッドは、次に待つ前に少し休みます（1、2、4、8、16 ms、以後 20 ms）。
- 送信は、データを送って 0 バイトだったパスのあとはビット 2 が無いときと同じく 1 ms 待ち、poll だけをしたパス
  （書き込み可能でなかった）のあとにエッジを待ちます。待ちは呼び出しの期限（受信の `SO_RCVTIMEO`、`poll` の
  timeout、connect と送信の 10 秒）を越えません。
- NINA 以外の記述子を含む `poll` は、最長 20 ms ごとに見直します（その記述子の変化の見方はビット 2 が無いときと同じ）。
- エッジの割り込みを使えなかったとき（`nina_os_event_start()` が IRQ の線のコールバックの登録かエッジの
  有効化に失敗したとき）は、ビット 2 があっても、ビット 2 が無いときの間隔で待ちます。使っているビットからビット 2 が外れます。

### esptool の中継（`nina passthrough confirm`）

`nina passthrough confirm` で、USB のシリアル（CDC ACM）を NINA の UART（ESP32 の UART0、ROM のブートローダーが使う）へ
中継するモードで再起動します。PC の esptool で NINA のフラッシュを読み書きするためのもので、Arduino の
SerialNINAPassthrough スケッチの代わりです（RP2040 のファームウェアは書き換えません）。中継そのものは NuttX 版・FreeRTOS 版と
共有するドライバーのコア（`common/drivers/nina/nina_passthrough.c`）で、この版の部分は Zephyr の USB デバイススタックの上のホスト側
（`common/drivers/nina/port/zephyr/nina_passthrough_os.c`）と RP2040 のレジスターのモジュール側（`common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c`）です。
`CONFIG_NINA_PASSTHROUGH`（既定で有効）。

- 配線：GPIO8 = UART1 TX → ESP32 の UART0 RX、GPIO9 = UART1 RX。SPI の MISO・CS と同じピンなので、中継の間は
  NINA の SPI を使えません。GPIO2 = ESP32 の GPIO0（起動モード）、GPIO3 = RESETN。RTS が有効でリセットを解除し、
  DTR が有効で GPIO0 を LOW にします（スケッチの mbed の分岐と同じ）。ポートを開くと ROM のダウンロードモードで起動し、
  閉じるとリセットに保持します。DTR が無効のままリセットを解除した（NINA のファームウェアで起動した）ときは、750 ms 後に
  GPIO0 を離し、GPIO8 を駆動しません。UART1 のレートはホストの line coding に従います（8N1）。
- 入り方：コマンドが watchdog の scratch 0〜3 に印を書いて watchdog で再起動します。起動時（`POST_KERNEL` の 0）に印を
  読んですぐ消すので、RESET やそのあとの再起動は通常の起動です。フラッシュ（KV）には何も書きません。
- 中継の起動でも Zephyr は通常どおり起動しますが、NINA のコアには SPI のバスが無いように見せます
  （`nina_bus_initialize()` が失敗する）。Wi-Fi のインターフェースの初期化、Telnet の待ち受けのソケット、`nina version` などは
  `-ENODEV` で失敗し、SPI1 と NINA の線に触りません（RAM のログに Telnet のエラーが残ります）。`main()` は設定・校正・
  時刻同期・Wi-Fi の接続を行わずに中継を動かします。CDC ACM の割り込みのコールバックを中継のものに差し替えるので
  （`POST_KERNEL` の 99）、シェルは入力を受けず、出力もポートに出ません。ログは RAM だけに出ます。
- 取りこぼし：モジュール → PC は、UART1 の受信割り込み（優先度 0、FIFO に 4 バイトで発生）が 16 KiB のリングへ移します。
  PC が読まない間を 921600 bps で 178 ms 分保持し、あふれた分は数えます。PC → モジュールは 1 パケット（64 バイト）ずつで、
  UART の FIFO に入りきらない分は送信割り込みが書き、その間は CDC ACM の受信を止める（ホストが待つ）ので失われません。
  DTR・RTS とレートは USB のスタックの通知（システムのワークキュー）と 1 ms ごとの確認で反映します。データは CDC ACM の
  コールバック（専用のワークキュー、下の注意）で動かします。
- 出方：ポートを 300 bps にして閉じる（例 `stty -F /dev/ttyACM0 300`）か、RESET。NINA を自分のファームウェアで起動し直して
  （GPIO0 HIGH、RESETN を 10 ms LOW、750 ms 後に GPIO0 を離す）から、watchdog で再起動します。
- RAM のログ（出たあとの `dmesg`）に、開始・ポートを閉じるたび・終了の行が残ります（送受信のバイト数、USB が無い間に
  捨てた数、リングからあふれた数と最大の滞留、UART のオーバーラン・ブレーク・フレーミングエラー、レート）。
- esptool 5.3.0 は `--before no-reset` では繋がりません。`ESPTOOL_CFGFILE` の `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` と
  `--before default-reset --after no-reset` で RTS を有効にしたまま使います（手順はルートの README の「NINA のファームウェアの
  読み書き」）。
- 実機（2026-11-18）: `read-flash 0 0x200000` を 115200 bps（192 秒）と 921600 bps（26.5 秒）で行い、2 つの読み出しが一致し、
  取りこぼしとオーバーランは 0 でした。300 bps で閉じると通常の起動に戻りました。中継の起動では `dmesg` に Telnet の
  待ち受けの `<err>` が 2 行残ります（NINA を起こさないため）。

### CDC ACM の専用のワークキュー

`prj.conf` の `CONFIG_USBD_CDC_ACM_WORKQUEUE=y` で、CDC ACM（USB のシェルの送信完了）をシステムのワークキューから専用の
ワークキューへ移しています。Zephyr 4.5.0 の zperf の iperf3 サーバーは、`zperf tcp download stop` で `srv_lock` を持ったまま
シェルに印字し、同じシステムのワークキューの `idle_work_handler()` が `srv_lock` を待つので、シェル（`shell_uart`）と
システムのワークキューが互いを待って止まっていました（実機で確認。どちらも優先度 -1 で pending）。この設定を足したイメージでは、
接続中の `zperf tcp download stop` を 20 回繰り返しても、USB と Telnet のシェルは止まりませんでした（2026-11-18）。

### 実機で確認した範囲

次の 4 項目は、NINA のドライバーを共有のコアに載せ替える前のイメージでの確認です（各 1 回）。

- NTP（UDP）、Telnet（TCP の待ち受け）、MQTT（TCP クライアント）の動作。
- 6667 Hz の RAW 送信（MQTT）と Telnet を動かしたまま、UDP のデータグラム 360 個を往復させて、
  欠落・破損がなく、MQTT 側にも欠番がないこと。
- AP の停止・再起動のあとの自動再接続と、アドレス・ゲートウェイの再取得。

- zperf（PC 側は iperf3、各 30 秒）：TCP 送信 2.93 Mbit/s、TCP 受信 1.54 Mbit/s、UDP 送信は
  500 kbit/s の指定で欠落 0。

共有のコアと新しいファームウェア（0001〜0006）のイメージ（2026-11-18）では、書き込み直後の送信、送信中の Telnet と
`wifi scan`、手動の切断と復帰、接続中の `zperf tcp download stop` の繰り返しのあとの USB と Telnet のシェルを確かめました。

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

## Zephyr へのローカルのパッチ

Zephyr 本体とモジュールの一部の変更は、Zephyr の `west patch` のローカルのパッチにしています。パッチの一覧は
`zephyr/manifest/zephyr/patches.yml`、ファイルは `zephyr/manifest/zephyr/patches/<モジュール>/` です。

| パッチ | 対象 | 内容 |
|---|---|---|
| `hal_rpi_pico/0001-boot2-at25sf128a-clkdiv-2.patch` | hal_rpi_pico | ボードの boot2（`boot2_at25sf128a.S`）のフラッシュのクロックを clk_sys の 1/4（31.25 MHz）から 1/2（62.5 MHz）に。hal_rpi_pico には値を渡す口が無いので既定値を変える |
| `zephyr/0001-arm-cortex-m-rp2040-divider-save.patch` | zephyr | `CONFIG_ARM_RP2040_DIVIDER_SAVE`: スレッドの切り替え（PendSV、`swap_helper.S`）で SIO の除算器を退避・復元する |
| `zephyr/0002-soc-rp2040-rom-mem-ops.patch` | zephyr | `CONFIG_RP2040_ROM_MEM_OPS`: `memcpy`・`memset` をブート ROM のルーチンにする |
| `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` | zephyr | DesignWare I²C ドライバー（`drivers/i2c/i2c_dw.c`）: RP2040 の FIFO の深さと RX のしきい値、`CONFIG_I2C_DW_RP2040_DMA` の I²C0 の DMA 読み出し（「I²C」）。定数は新しいファイル `drivers/i2c/i2c_dw_rp2040.h` に、DMA の塊と読み出しのコマンド語は `i2c_dw_rp2040_dma_priv.h` に、`i2c_dma` コマンドの関数の宣言は `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` に置き、上流のファイルに入るのは 16 か所（`i2c_dw.c` 14、`i2c_dw.h` 1、`Kconfig.dw` 1）。どのコントローラーでも、0 バイトのメッセージを断り（`-ENOTSUP`）、`i2c_recover_bus()` は `-ENOTSUP` を返す |
| `zephyr/0004-i2c-shell-scan-not-supported.patch` | zephyr | シェルの `i2c scan`（`drivers/i2c/i2c_shell.c`）: 確かめの書き込み（0 バイト）が `-ENOTSUP` を返したら、どこも `--` で「0 devices found」とする代わりに、止まって、コントローラーがこのスキャンに対応していないことを表示し、`-ENOTSUP` を返す。ほかのエラーは従来どおり `--` |
| `zephyr/0006-sensor-shell-fifo-attributes.patch` | zephyr | sensor シェル（`drivers/sensor/sensor_shell.c`）: LSM6DSO の FIFO の属性と修正（「sensor シェル」）、`drivers/sensor/sensor_shell_os.h`。`common/drivers/lsm6dso/` の LSM6DSO のドライバー（その `<zephyr/drivers/sensor/lsm6dso.h>`）が要る |

- **当て方**: `source script/setup.sh` が `west patch apply` で各モジュールの作業ツリーに当てます（commit は作りません。
  `git -C zephyr/zephyr status` には変更として出ます）。当てたものは飛ばすので、何度読み込んでも同じです。当てた集合の写しを
  `zephyr/.west/local-patches/` に持ち、`patches.yml` かパッチが変わっていれば、写しの集合を先に戻してから今の集合を当てます
  （一覧から消したパッチがツリーに残りません）。当てるのに失敗したときは、当てた分を戻して止まります。
- **パッチを直したとき**: `patches.yml` の `sha256sum` を `sha256sum <ファイル>` の値に直してから、`source script/setup.sh` を
  読み込み直します。
- **戻し方**: `zephyr/` で `west patch reverse` を実行し、`zephyr/.west/local-patches/` を消します。戻せないときは、
  `setup.sh` の表示のとおり各ツリーを `git -C <ツリー> checkout -- .` で戻してから写しを消し、読み込み直します。
  戻したツリーでは、このアプリはビルドできません（`prj.conf` がパッチの `CONFIG_RP2040_ROM_MEM_OPS`・`CONFIG_I2C_DW_RP2040_DMA` を使い、
  `CONFIG_RP2040_PICO_DIVIDER` は `CONFIG_ARM_RP2040_DIVIDER_SAVE` が無いと CMake で止まります）。
- **Zephyr を上げるとき**: 先に上の手順でパッチを戻し、`manifest/west.yml` の版を変えて `west update` します。新しいツリーで
  各パッチを作り直して `patches/` に置き、`sha256sum` を直し、`source script/setup.sh` で当ててからビルドします。
  0003 を作り直したら `script/host_tests.sh` を実行し、`i2c_dw_sync` が示す上流の変更を共有のコア（`common/drivers/i2c_dw/`）に
  入れるか、入れない理由を `i2c_dw.c` の冒頭の一覧に書いてから、`common/tests/i2c_dw_sync.py ... --update` で期待する差分を
  書き直します（「テスト」）。0006 は `common/drivers/sensor_shell/sensor_shell.c` から作ります
  （先に新しいツリーの上流のファイルの変更をそこへ入れる）。そうすると `zephyr_sensor_shell_patched` が通ります。

### boot2 の分周 2

boot2 が使う読み出し（EBh）の最高周波数は、AT25SF128A のデータシートで 108 MHz 以上です。clk_sys 125 MHz では分周は偶数なので、
62.5 MHz が上限です（FreeRTOS 版の Pico SDK の boot2 と同じ分周）。実機では `devmem 0x18000014`（SSI の BAUDR）が 2 になります。

### 除算器（`CONFIG_RP2040_PICO_DIVIDER`）

アプリの `CONFIG_RP2040_PICO_DIVIDER`（既定で有効）が、hal_rpi_pico の Pico SDK の `divider_hardware.S` をビルドし、
`__aeabi_idiv`・`__aeabi_idivmod`・`__aeabi_uidiv`・`__aeabi_uidivmod`・`__aeabi_ldivmod`・`__aeabi_uldivmod` をリンカーの `--wrap` で
そこへ回します（`CMakeLists.txt`）。無効にすると libgcc のシフトと減算のループです。

- ルーチンは割り込みを止めません（`PICO_DIVIDER_DISABLE_INTERRUPTS=0`）。割り込みハンドラーが割り算の途中に入った場合は、
  ルーチン自身が DIRTY を見て退避します。スレッドが割り算の途中で切り替わる場合は、パッチの `CONFIG_ARM_RP2040_DIVIDER_SAVE`
  （`CONFIG_RP2040_PICO_DIVIDER` が select する）が PendSV で退避します。退避は DIRTY のときだけ 4 つのレジスターを
  `struct _thread_arch` の 20 バイトに置き、割り算の途中でなかったスレッドは DIRTY が落ちた状態で戻ります（FreeRTOS 版の
  Pico SDK のポートは、切り替えのたびに 4 語を退避します）。
- 0 での割り算は Pico SDK のルーチンの結果です（3 版で同じ）。32 ビットは飽和した商（0 / 0 は 0）と余り 0、64 ビットは
  libgcc と同じです。
- 試験のイメージ（2026-11-18）: 退避なしでは 81 秒で食い違いが 30 回（最も低い優先度のスレッドだけ）、退避ありでは 361 秒・
  約 4,070 万回の割り算で 0 でした。

### ブート ROM の `memcpy`・`memset`（`CONFIG_RP2040_ROM_MEM_OPS`）

パッチの `CONFIG_RP2040_ROM_MEM_OPS`（`prj.conf` で有効）で、SoC の CMake が `memcpy`・`memset`・`__aeabi_memcpy*`・
`__aeabi_memset*` をリンカーの `--wrap` で RP2040 のブート ROM のルーチンへ回します（Pico SDK の `pico_mem_ops` と同じ使い分け）。
picolibc の中からの呼び出しと、GCC が構造体のコピーやループに出す呼び出しも含みます。無効にすると picolibc のバイト単位の
ループです。ROM のルーチンは初回の呼び出しで ROM の関数表から引いて `.bss` に置くので、`.bss` のゼロ化と `.data` のコピーを
行う `arch_early_memset`・`arch_early_memcpy` は picolibc のものを呼びます。NuttX 版は同じ仕組みを NuttX へのパッチ
（`nuttx/patches/nuttx/0010`）で持ちます。

## テスト

`zephyr/script/host_tests.sh` が、偽の Zephyr ヘッダーを使ってドライバーの次の部分を PC 上で
テストします。`nina_boundary_*`、`passthrough`、`nina_socket*`、`nina_net` 以外は NuttX 版・FreeRTOS 版と共有するテスト（`common/tests/`、`common/tests/run.sh` が実行）です。

| テスト | 対象 |
|---|---|
| `fifo_words` | FIFO ワードの解析 |
| `fifo_decoder` | FIFO バッファの decoder |
| `lsm6dso_stream` | ストリームの状態機械（`lsm6dso_stream.c`）を、境界のホストでの実装とコアの FIFO の呼び出しの偽物の上で: 開始（INT1 の線が無い、FIFO と INT1、開始時にすでに有効なレベル、INT1 の有効化・レベルの読み取り・割り当ての失敗）、エッジでのバッチとそのヘッダー、watermark 未満では渡さないこと、オーバーラン・キャンセル・要求が消えたときの終了、最後の端数のバッチを渡す穏やかな停止とそのエラー、2 つ目の要求、watermark の範囲 |
| `zephyr_sensor_shell_patched` | `common/drivers/sensor_shell/sensor_shell.c` が、Zephyr の HEAD の `drivers/sensor/sensor_shell.c` にパッチ 0006 を当てたものとバイト単位で同じこと。Zephyr のツリーがあるときだけ |
| `i2c_dw_sync` | 共有の I²C のコア（`common/drivers/i2c_dw/`）と、パッチ後のドライバー（Zephyr のツリーの HEAD のファイルにパッチ 0003 を当てて作る）の一致。両方のヘッダーの定義（状態、割り込み、レジスター、IC_DATA_CMD のビット、FIFO の深さ 16・バッチ 4、DMA の上限 3584）、125 MHz での SCL のカウント、DMA の下限 32・しきい値 8/0・割り込みのマスク・IC_CON の bit 9・停止の待ち 100 ms・タイムアウト 1000 ms、スパイク長、対になる関数（Zephyr の DMA API の呼び出しを含めて名前をコアのもの（DMA は境界 `i2c_dw_dma_os.h`）にそろえて文ごとに比べ、差分が `common/tests/i2c_dw_sync.expected` と同じこと）、読み出しのコマンド語を作る関数 `i2c_dw_rp2040_read_commands()`（パッチとコアのそれぞれの `i2c_dw_rp2040_dma_priv.h` の静的関数）が文ごとに同じでコマンドのビットも同じこと、パッチが足す `zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` とコアの `common/drivers/i2c_dw/include/i2c_dw_rp2040_dma.h` がコメントとインクルードガードを除いて同じこと、パッチ後のドライバーが読むヘッダーがすべてツリーの中にあること。ビルドがあれば `.config` と `zephyr.dts` の値も見る。Zephyr のツリーがあるときだけ |
| `i2c_dw_public_headers` | I2C のコアを使う側のヘッダー（`common/drivers/i2c_dw/include/`）が、そのディレクトリーだけでコンパイルできること（コアの `internal/`・`port/` を読まない） |
| `nina_proto` | NINA の SPI フレーミング、応答の解析（ファームウェアのエラーの応答 0xEF を含む）、READY の期限、追加機能の問い合わせ（0x6A）（共有のコア `nina_proto.c`。偽の境界）。どれも READY の割り込み待ちを始められたときと始められなかったとき（100 µs ごとに読む）の両方で |
| `nina_core_independent` | NINA のコアの各ファイル（`common/drivers/nina/` の直下と `include/`・`internal/`、`port/` の境界のヘッダー）が C ライブラリー・POSIX のソケットの型・自分のヘッダーだけでコンパイルできること |
| `lsm6dso_core_independent` | LSM6DSO のコアの各ファイル（`common/drivers/lsm6dso/` の直下の `.c`、`include/lsm6dso_cal.h`・`lsm6dso_fifo.h`、`internal/`、`port/` の境界）が、C ライブラリー・ST の `lsm6dso_reg.h`・自分のヘッダー・ホストでの境界の実装（`common/tests/lsm6dso_os_host/` の 2 つ）だけでコンパイルできること |
| `lsm6dso_public_headers` | `common/drivers/lsm6dso/include/` のヘッダーがそのディレクトリーだけでコンパイルできること（`lsm6dso.h` は sensor API と RTIO の偽物と一緒に） |
| `nina_wifi` | Wi-Fi の操作（`nina_wifi.c`）を偽の境界とつなぎで：接続とその確認、そのスレッド（500 ms ごとの確認、省電力の設定、アドレス、60 回、試行中の切断）、切断、監視（2 回で切れたとみなす）、3.0.2 とパッチを当てたファームウェアのスキャンとその終わり方、状態 |
| `nina_wait` | ソケットの待ち（`nina_wait.c`）：間隔（事象を始める前と、始められなかったあと）、IRQ の線のエッジ、100 ms の時間切れ、期限、休み（偽の時計と線） |
| `nina_sock`、`nina_dns`、`nina_dns_offload` | NINA のソケットの命令、名前解決（`nina_dns_offload` はこの版と同じ `NINA_OS_DNS_OFFLOAD` の形） |
| `nina_boundary` | 共有の `nina_proto.c` を `common/drivers/nina/port/zephyr/` の `nina_bus.c`・`nina_os.c`（Wi-Fi のスレッドのロックと要求も）に載せたもの：フレーミング、線と設定の対応、リセットの手順、READY の割り込み待ち（コールバックの登録、セマフォのリセット、エッジの向き、転送前の無効化、10 ms の上限）と、コールバックを登録できなかったときのポーリング。IRQ の線のエッジの待ち（始められないとき、GPIO2 のコールバックと立ち上がりのエッジ、エッジの数、エッジで終わる待ち、時間切れ、待つ直前のエッジ） |
| `nina_boundary_passthrough` | 上と同じものを `CONFIG_NINA_PASSTHROUGH` で。中継の間はバスが無く、線にも SPI にも触れずに `-ENODEV`、中継でなくなれば通常どおり |
| `nina_passthrough` | 中継のコア（`nina_passthrough.c`）を偽の境界で：再起動の印と異常停止の記録の区別、DTR・RTS とモジュールの線の対応、レートと出口、バッファー（2 つのスレッドでも）、起動、両方向のデータとその数（ブレーク、オーバーラン、バッファーの満杯、ホストが無いとき）、tick（GPIO0 を離す時刻、LED）、出口の手順 |
| `passthrough` | 中継の Zephyr 側 `common/drivers/nina/port/zephyr/nina_passthrough_os.c` と `nina_shell.c` の `nina passthrough` を本物のコアと、偽の Zephyr と偽のモジュール側で：起動時の印の扱い、USB の通知と tick での線とレート、両方向のデータと割り込みの受け渡し、USB が無い間の破棄、出口の手順、コマンド |
| `nina_socket` | ソケットのオフロード `common/drivers/nina/port/zephyr/nina_socket.c` を共有のコアに載せたもの：ファイル記述子、待ちのループと sleep（事象を始められなかったあと）、オプション、accept、sendmsg、データグラム、poll、リゾルバーの登録。事象を始めたあとの IRQ の線のエッジの待ち（パスの間のエッジ、時間切れ、期限、送信の待ち方、NINA 以外の記述子を含む `poll`） |
| `nina_net` | Zephyr へのつなぎ `common/drivers/nina/port/zephyr/nina_net.c` を、偽の Zephyr の宣言と偽のコアで：デバイス、インターフェースの初期化、スキャンのパラメーターと結果、接続・切断・状態、コアのコールバックのインターフェースと Zephyr のイベントへの反映、プロファイルのフック、スレッドのスタックと優先度 |
