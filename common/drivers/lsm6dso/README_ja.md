# LSM6DSO のドライバー（`lsm6dso`）

[English](README.md) | 日本語

このリポジトリーのファームウェアは、Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上でそれぞれビルドでき、
それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。互換層は、Zephyr の API を Zephyr の名前のまま
ほかの OS の上で提供する層です（[`common/compat/README_ja.md`](../../compat/README_ja.md)）。

`common/drivers/lsm6dso/` は、ボードの LSM6DSOX（I2C0 の 0x6a）のドライバーで、4 つの OS のファームウェアで共有します。Zephyr の
`drivers/sensor/st/lsm6dso` の代わりに、FIFO ストリーム、校正値を使う変換、静止時の角速度のバイアスの追従を持ちます。OS を持たない
コアと、Zephyr へのつなぎ（`port/zephyr/`）からなり、つなぎは 4 つの OS とも同じもの（NuttX 版・FreeRTOS 版・ThreadX 版では互換層の上）です。
`sensor` コマンド（`common/shell/sensor_shell.c`）も、この文書で説明します。センサーを読む 3 つの経路（`sensor get`・`sensor stream`・
`imu start`）のスレッドとバッファは[設計書](../../../design/imu-raw-mqtt-design-ja.md)の
[§3.4](../../../design/imu-raw-mqtt-design-ja.md#34-センサーを読む-3-つの経路)、共有ソースの全体の中での位置付けは §9.1 にあります。

## ディレクトリー

| ファイル | 役割 |
|---|---|
| `lsm6dso.c` | コア: チップの操作（起動とリセット、レートとレンジ、単発の読み出しとその変換、FIFO のレジスターの操作・所有・ロック・統計、タイムスタンプの読み取り、使用中の校正とバイアスの追従）を `struct lsm6dso` の上で行う |
| `lsm6dso_stream.c` | コア: FIFO ストリームの状態機械（下）を `struct lsm6dso_stream` の上で行う |
| `lsm6dso_cal.c` | コア: 物理量への変換の計算と、静止時の角速度のバイアスの追従の計算（下） |
| `lsm6dso_fifo.c` | コア: FIFO のワードの解析 |
| `include/lsm6dso.h` | 拡張 API（FIFO のワードとその解析、校正の型と呼び出し、FIFO の属性・設定・統計・停止、タイムスタンプの読み取り）。アプリが読むのはこれだけで、Zephyr の名前 `<zephyr/drivers/sensor/lsm6dso.h>` で読む（下の各 OS の節） |
| `include/lsm6dso_cal.h`、`include/lsm6dso_fifo.h` | ドライバーを使う側のもの: 校正の型と呼び出し、FIFO のワードとその解析・統計 |
| `internal/lsm6dso_core.h` | コアの関数（つなぎが呼ぶ） |
| `internal/lsm6dso_stream.h`、`internal/lsm6dso_fifo_packet.h` | ストリームの関数と、そのバッファーの形式（`struct lsm6dso_fifo_packet`。ドライバー内だけのもの） |
| `port/lsm6dso_os.h` | 境界: OS へ（下の「OS が実装する境界」） |
| `port/lsm6dso_stream_req.h` | 境界: ストリームを使う側へ |
| `port/zephyr/` | Zephyr へのつなぎ（4 つの OS とも。下の「Zephyr」）。独立した Zephyr のモジュール（`zephyr/module.yml`、`Kconfig`、`CMakeLists.txt`） |
| `port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h` | Zephyr 版の拡張 API のヘッダー。`include/lsm6dso.h` を読む |
| `port/nuttx/lsm6dso_instance.h` | NuttX 版のデバイス |
| `port/pico_sdk/` | Pico SDK の上の OS（FreeRTOS 版・ThreadX 版）のデバイス、その開始、ビルドの規則 |

ドライバーの外にある関係するファイル:

| ファイル | 役割 |
|---|---|
| `common/drivers/stmemsc/stmemsc.h`、`stmemsc_i2c.c`、`stmemsc_mdelay.c` | Zephyr の `drivers/sensor/st/stmemsc/` の I2C の部分の写し（ST の `stmdev_ctx_t` と Zephyr の I2C の API のつなぎ。`stmemsc_mdelay()` は互換層の `k_msleep()`）。NuttX 版・FreeRTOS 版・ThreadX 版が使う（前置きヘッダーは付けない）。`lsm6dso_sensor.h`・`lsm6dso_os_impl.h` の `<stmemsc.h>` はインクルードパスでこのファイルになる。Zephyr 版は Zephyr の stmemsc |
| ST の `lsm6dso_reg.c`・`lsm6dso_reg.h` | チップのレジスターの操作（hal_st）。Zephyr 版は Zephyr の hal_st、NuttX 版は `nuttx/stmemsc/`、FreeRTOS 版は `freertos/stmemsc/`、ThreadX 版は `threadx/stmemsc/`（どれも Git 管理外。各 OS の `setup.sh` が取得） |
| `common/compat/include/zephyr/drivers/sensor/lsm6dso.h` | 互換層の `<zephyr/drivers/sensor/lsm6dso.h>`。`include/lsm6dso.h` を読む |
| `common/compat/port/gpio_os.h` と、その実装 `common/compat/port/nuttx/rp2040_gpio_os.c`・`common/compat/port/pico_sdk/gpio_os_rp2040.c` | 互換層の `gpio0` の下の GPIO バンク 0（INT1 の割り込み） |
| `common/shell/sensor_shell.c` | `sensor` コマンド（下の「`sensor` コマンド」） |
| `common/compat/sensor_shell_stream.c`、`common/compat/include/sensor_shell.h` | Zephyr の `sensor_shell_stream.c` と `sensor_shell.h` の写し（NuttX 版・FreeRTOS 版・ThreadX 版） |
| `common/shell/port/nuttx/sensor_shell_os.h`、`sensor_shell_nuttx.[ch]`、`common/shell/port/generic/sensor_shell_os.[ch]` | `sensor` コマンドの OS の部分 |
| `nuttx/drivers/lsm6dso/Kconfig`、`Make.defs`、`st_lsm6dso_reg.c` | NuttX 版のオプションとビルドの規則 |

## コアの動作

### FIFO ストリーム

Zephyr の Sensor Streaming API（`sensor_stream()`、RTIO）で FIFO を読み出せるようにしています。トリガーは
`SENSOR_TRIG_FIFO_WATERMARK`、オプションは `SENSOR_STREAM_DATA_INCLUDE` です。

| ファイル | 役割 |
|---|---|
| `lsm6dso.c`（コア） | FIFO の所有とロック: レジスターの設定・一時停止・停止、状態の読み取りとバースト読み出し（static）、しきい値の INT1 への割り当て、統計、タイムスタンプの間引き、タイムスタンプの読み取り。単発の読み出しとその変換、`lsm6dso_cal_zero()` のバイアスの読み取り |
| `lsm6dso_stream.c`（コア） | ストリームの状態機械: 最初の実行での開始（FIFO、INT1 の有効化と割り当て）、INT1 のエッジごとと 20 ms ごとの確認での watermark 分のバッチの読み出し、穏やかな停止、エラー・オーバーラン・使う側のキャンセルでの終了。watermark |
| `lsm6dso_fifo.c`（コア） | FIFO のワードの解析（加速度・角速度・タイムスタンプ・その他、タグカウンター） |
| `lsm6dso_cal.c`（コア） | 物理量への変換と静止時の角速度のバイアス（下）。`lsm6dso.c` とデコーダーが使う |
| `lsm6dso_sensor.c`（つなぎ） | コアの上の上流のドライバー: デバイス、属性、`sample_fetch` と校正値を使う `channel_get`、拡張 API のうちデバイスを取る呼び出し |
| `lsm6dso_rtio_stream.c`（つなぎ） | デバイスごとのストリーム: デバイスツリーの INT1 の線、RTIO の要求をコアのストリームへ渡す（ほかのトリガーは `-ENOTSUP`、2 つ目のストリームは `-EBUSY`）、拡張 API の停止・watermark・タイムスタンプの間引き |
| `lsm6dso_os.c`（つなぎ） | ストリームのワークキュー（`imu_fifo_worker`、Zephyr の優先度 10、2048 バイト）、INT1 の GPIO の割り込み（システムのワークキューを通してストリームを起こす） |
| `lsm6dso_rtio.c`（つなぎ） | ドライバーの RTIO の submit。ストリームは `lsm6dso_rtio_stream.c` へ、単発取得（`sensor get`）は sensor API のフォールバックの写しで RTIO のワークキューへ |
| `lsm6dso_decoder.c`（つなぎ） | `lsm6dso_fifo_buffer_words()`、FIFO バッファ用のデコーダー（`sensor stream` の表示に使用）と、単発取得のバッファを Zephyr の既定のデコーダーに渡すデコーダー |

INT1 の割り込みはドライバー自身のもの（`lsm6dso_os.c` の中）で、線はストリームの間だけ有効、システムのワークキューはストリームの
ワークキューを起こすだけで、I2C の読み出しはすべてストリームのワークキューが行います。

- `sensor_stream()`（`SENSOR_DT_STREAM_IODEV`）が multishot の読み出しを 1 つ出し、`lsm6dso_sensor_submit()` がそれをコアのストリームに渡し、
  ストリームが呼び出し元の文脈でワーカー（`imu_fifo_worker`）を起こします。
- 開始: ワーカーは最初の実行で FIFO をバイパスにしてから、加速度・角速度のバッチレート（BDR）、タイムスタンプの間隔（既定で 8 回に 1 つ）、
  watermark を設定してストリームモードにし、INT1 の立ち上がりエッジの割り込みを有効にしてから FIFO のしきい値を INT1 に出します。
  有効にした時点で INT1 がすでに高ければ、その場で 1 回処理を予約します。
- バッチ: INT1 のエッジ → GPIO の割り込みが `k_work_submit()` でシステムワークキュー（`sysworkq`）に処理を入れる → そこでコアの
  `lsm6dso_stream_int1()` がワーカーを `k_work_reschedule_for_queue(.., K_NO_WAIT)` で起こす → ワーカーが FIFO の状態を読み、watermark 以上なら
  たまっているワードを 1 回のバーストで読み出して、1 つの RTIO バッファとして渡します。完了した multishot はその場でワーカーに戻り
  （`lsm6dso_sensor_submit()`）、ワーカーはすぐもう一度状態を見ます。何も無ければ 20 ms 後に見直す予約（`k_work_schedule_for_queue(.., K_MSEC(20))`）
  をします。エッジを取りこぼしてもこの見直しで読みます。
- バッファの先頭にはヘッダー（ワード数、周期、感度、物理量への変換係数など）が付きます。ヘッダーの形式（`internal/lsm6dso_fifo_packet.h` の
  `struct lsm6dso_fifo_packet`）はドライバー内だけのものです。バッファを読む側は、物理量が欲しければデコーダーを、生のワードが欲しければ
  `lsm6dso_fifo_buffer_words()` を使います。ワードは `lsm6dso_fifo_next_entry()` で 1 つずつ解釈でき、種類、xyz、タグカウンター、
  タイムスタンプのワードが持つカウンター値が得られます。
- バッファーはコンテキストのメモリープールから取ります（最小 = パケットの頭＋watermark 分、最大 = 頭＋512 ワード分）。
- **FIFO オーバーラン**（`-EOVERFLOW`）、I2C エラー（そのエラー）、バッファ不足（`-ENOMEM`）を検出すると、FIFO を止めてストリームをエラーで
  終えます。失われた個数は分からないので、連続しているように見えるデータは返しません。再開するかどうかは呼び出し側
  （`common/imu/imu_stream.c`）が決めます。終えるときは FIFO をバイパスに戻し、INT1 への出力と割り込みを止めます。
- **停止**は `lsm6dso_fifo_request_stop()` で要求します。ワーカーがバッチ化を止め（BDR を 0）、FIFO に残ったワードを最後の端数のバッチとして
  渡し切ってから、FIFO をバイパスに戻して終端の完了通知（`-ECANCELED`）を出します。RTIO の要求を直接キャンセルすると、読み出し済みの
  バッファが消費前に解放され得るためです。キャンセル（`rtio_sqe_cancel()`）は次にワーカーが動いたとき（INT1 か 20 ms の見直し）に気づき、
  バッチ化を止めずにそのまま停止して `-ECANCELED`（停止の失敗ならそのエラー）で終えます。FIFO に残ったワードは捨てます。
- FIFO を使っている間、通常の単発取得と属性の変更は `-EBUSY` になります。
- ストリームを動かしても ODR は変えません（`imu start` が設定し、停止後もそのまま）。

### 時刻

FIFO にはセンサーのタイムスタンプ（公称 25 µs 単位）を入れます。間隔は属性で変えられます。タイムスタンプの有効化は開始時に行い、
止めません（ストリームを再開してもカウンターは進み続けます）。サンプルへの対応付けと PC 時刻への変換はアプリ側で行います
（[設計書](../../../design/imu-raw-mqtt-design-ja.md) §7）。

`lsm6dso_timestamp_read()` は、センサーのタイムスタンプレジスタを、呼び出し側の時計の読み取りで挟んで読みます。センサーの時計と
ボードの時計を対応付けるために使います。

decoder が返すサンプル時刻は、ホストの読み出し完了時刻と公称 ODR からの推定で、I2C の転送と割り込み処理の遅れを含みます。
正確な時刻ではありません。

### 属性

`SENSOR_CHAN_ALL` に対する独自属性です（`<zephyr/drivers/sensor/lsm6dso.h>`）。どちらも次のストリーム開始時に適用され、ストリーム中は
変更できません。

| 属性 | シェルでの名前 | 値 | 初期値 |
|---|---|---|---|
| `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` | `fifo_watermark` | 1〜256 ワード | 32 |
| `SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION` | `fifo_timestamp_decimation` | 0、1、8、32 | 8 |

watermark を大きくすると読み出しの回数が減って負荷が下がりますが、データが届くまでの時間が延び、FIFO（512 ワード）の余裕が減ります。

### 物理量への変換と校正

RAW から物理量への変換は `lsm6dso_cal.c` の 1 つの関数にまとめてあり、FIFO のデコーダー（`sensor stream`）と単発読み出し（`sensor get`）の
両方がこれを使います。

```text
out = M · (raw − bias)
```

- すべて整数演算です。`M` はスケールと軸のずれをまとめた 3×3 の行列、出力は Q31（shift 9）で、1 単位は 2⁻²² m/s² または rad/s です。
- 校正値がないときは、公称感度を対角に置いた行列とバイアス 0 で同じ関数を通します。
- 校正値は `lsm6dso_cal_set_lookup()` で登録した関数から得ます。ドライバーは登録時、`lsm6dso_cal_reload()` の呼び出し時（`imu cal reload`）、
  レンジの設定時に、現在の感度でこの関数を呼びます。該当がなければ公称値になります。ドライバーは KV を読みません。校正値を引く関数は
  アプリの `common/imu/imu_cal.c` が登録します（NuttX 版は init の `imu_cal_init()`）。
- FIFO のバッファには、読み出した時点の変換係数を入れます。デコーダーはバッファだけで変換でき、途中でレンジや校正値が変わっても、
  読み出し時の係数で変換されます。
- 校正値が掛かるのは単発取得の `sensor_channel_get()` と FIFO のデコーダーの出力で、取得サービスが MQTT で送る RAW には掛かりません。
- `lsm6dso_cal_status()` で、加速度・角速度それぞれが校正値を使っているかと、使用中の角速度のバイアスを取得できます。

角速度のバイアスは、静止している間のデータから取り直します（`lsm6dso.c` が動かす `lsm6dso_cal.c` の `lsm6dso_zero_*()`）。

- FIFO から読んだサンプルを 1/8 秒ごとに区切り、8 区切り連続で角速度の平均（0.3 dps 以内）と加速度の平均（2 mg 以内）が揃っていれば
  静止とみなして、その窓の角速度の平均を取り込みます。最初の 20 回は単純平均、その後は 1/20 の指数平滑です。1 サンプルごとの値では
  判定しません。6667 Hz では 1 サンプルのノイズ（約 1.7 dps）が大きすぎるためです。
- FIFO のストリームが動いている間（`imu start`、`sensor stream`）は常に働きます。ドライバーが自分でセンサーを動かすことはありません。
- `lsm6dso_cal_zero()` は、その場で 1 回取り直します。センサーが止まっていれば 104 Hz で最長 3 秒動かし、終わったら止めます。
  ストリームが FIFO を使っている間は `-EBUSY` を返します。
- 値は RAM にだけ持ち、変換の選び直し（レンジの設定、読み直し）で破棄します。保存してあるバイアスから 1 dps（校正値がなければ 0 から
  5 dps）以上離れた窓は採用しません。

### 統計

`lsm6dso_fifo_get_stats()` で、読み出したワード数、転送の回数と時間、I2C エラー、バッファ不足、オーバーランの観測回数、設定 ODR、感度、
周波数補正値を取得できます。統計は FIFO を開始するたびにクリアされ、`generation` が 1 進みます。

### 制限

- 対応しているのは I2C 接続だけです（SPI のデバイスでは FIFO を受け付けない）。
- 内部温度の取得には対応していません（FIFO にも入れていません）。上流のトリガー（`CONFIG_LSM6DSO_TRIGGER`、`lsm6dso_trigger.c`）、
  センサーハブ（`CONFIG_LSM6DSO_SENSORHUB`、`lsm6dso_shub.c`）、温度の読み取り（`CONFIG_LSM6DSO_ENABLE_TEMP`）はこのドライバーにはありません。

## OS が実装する境界

コアは自分のオブジェクト `struct lsm6dso` と `struct lsm6dso_stream`（どちらもつなぎの `struct lsm6dso_data` の中）を受け取ります。つなぎは
単発の読み出しと設定の前後でコアの FIFO のロックを取ります（`lsm6dso_acquire()`・`_release()`、static inline）。コアの関数は `lsm6dso_*()` と
`lsm6dso_stream_*()` です。拡張 API の呼び出し（デバイスを取り、名前はそのまま）や `lsm6dso_cal.h` の関数と同じ名前になるものは、動詞を
先にします: `lsm6dso_set_cal_lookup()`・`lsm6dso_reload_cal()`・`lsm6dso_get_cal_status()`・`lsm6dso_measure_gyro_bias()`（`lsm6dso_cal_zero()` の本体）・
`lsm6dso_track_gyro_bias()`・`lsm6dso_copy_cal()`・`lsm6dso_get_fifo_stats()`・`lsm6dso_set_timestamp_decimation()`・
`lsm6dso_get_timestamp_decimation()`・`lsm6dso_read_timestamp()`。

コアはチップへ ST の `lsm6dso_reg.c`（hal_st）を通して、それ以外へ `port/` の 2 つの境界を通してつながります:

- `port/lsm6dso_os.h`（OS へ）: 2 種類のロック、待ち、サイクルカウンター、FIFO の長い読み出し、ログ。ストリームのために、ドライバー専用の
  スレッドの上のワーク、INT1 の線、バッチの時刻の時計。
- `port/lsm6dso_stream_req.h`（ストリームを使う側へ）: 受け持つ要求、バッチごとのバッファ、バッチかエラーを返す先。

境界は、OS の `lsm6dso_os_impl.h` と使う側の `lsm6dso_stream_req_impl.h` が定義する static inline の関数でできていて、ロック・
サイクルカウンター・バスの読み出し・ワーク・RTIO は Zephyr の関数を直接呼ぶのと同じ呼び出しになります。普通の関数は INT1 の有効化と
無効化（ストリームごとに 1 回。`lsm6dso_os.c`）だけです。OS はコアを `lsm6dso_stream_run()`（ワーク）と `lsm6dso_stream_int1()`（エッジ）で
呼び返します。

バスは ST の `stmdev_ctx_t` です。その `read_reg` は 255 バイトまでしか読めない（stmemsc の長さが `uint8_t`）ので、FIFO のデータ
（最大 3584 バイト）・状態・タイムスタンプは `lsm6dso_os_bus_read()`（コンテキストの先の I2C のデバイスへの `i2c_burst_read_dt()`）で読みます。
ほかのバスではストリームを始めません（`lsm6dso_os_bus_read_supported()`）。

バッチ 1 つ（watermark ごと。6667 Hz・128 ワードで毎秒約 104 回）の流れは次のとおりです: GPIO の割り込みが INT1 のワークをシステムの
ワークキューに入れ、それが `lsm6dso_stream_int1()` を呼び、ストリームのワークをすぐ実行するように入れ直します。ワークキュー
`imu_fifo_worker` がハンドラー（呼び出し 1 段）から `lsm6dso_stream_run()` を呼び、ストリームはスピンロックの下で RTIO の要求を引き、コアの
`lsm6dso_fifo_status()`・`lsm6dso_fifo_read()`・`lsm6dso_track_gyro_bias()`・`lsm6dso_copy_cal()` を呼びます。RTIO のバッファと完了、
センサーの時計、次の確認の予約はインラインです。

4 つの OS とも、この実装は Zephyr へのつなぎの `port/zephyr/lsm6dso_os_impl.h` と `lsm6dso_os.c`、`port/zephyr/lsm6dso_stream_req_impl.h`（RTIO）で、
NuttX 版・FreeRTOS 版・ThreadX 版はそれを互換層の上でビルドします。OS ごとに持つのは、つなぎの末尾のデバイス `lsm6dso_instance.h` だけです。
Zephyr 互換層を持つ新しい OS は、このドライバーのために何も書きません。互換層を持たない OS なら、2 つの `_impl.h` と INT1 の 2 つの関数を書きます。

## 各 OS のつなぎ

### Zephyr

つなぎ `port/zephyr/` は、デバイスと sensor API（`lsm6dso_sensor.c`・`lsm6dso_sensor.h`）、デバイスツリーのデバイス（`lsm6dso_instance.h`）、
ストリームの RTIO 側（`lsm6dso_rtio_stream.c`・`lsm6dso_rtio.c`・`lsm6dso_decoder.[ch]`）、コアの境界の OS 側（`lsm6dso_os_impl.h` と `lsm6dso_os.c`:
`k_mutex`・`k_spinlock`・`k_busy_wait()`・`k_msleep()`・`k_usleep()`・`k_cycle_get_32()`・`i2c_burst_read_dt()`・ワークキュー `imu_fifo_worker` の上の
遅延つきワーク・システムのワークキューを通す INT1 の GPIO の割り込み・センサーの時計・Zephyr のログ）、使う側の実装
（`lsm6dso_stream_req_impl.h`: RTIO のマルチショットの要求）です。ファイル名は、上流と、FIFO ストリームを持つ同系のドライバー `lsm6dsv16x` に
合わせています。

- つなぎは独立した Zephyr のモジュールです（`port/zephyr/zephyr/module.yml`。アプリの `CMakeLists.txt` が `ZEPHYR_EXTRA_MODULES` に足す）。
- その `Kconfig` にドライバーのオプション `CONFIG_LSM6DSO_LOCAL`（`st,lsm6dso` か `st,lsm6dso32` のノードがあれば既定で有効。ST のレジスターの
  ドライバーと Zephyr の stmemsc のために `HAS_STMEMSC`・`USE_STDC_LSM6DSO` を選ぶ）があり、`prj.conf` は同じデバイスの Zephyr のドライバーを
  止めます（`CONFIG_LSM6DSO=n`）。
- その `CMakeLists.txt` がライブラリー `lsm6dso_local` を作って拡張 API `<zephyr/drivers/sensor/lsm6dso.h>`
  （`port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h`。`include/lsm6dso.h` を読む）を公開します。devicetree binding は Zephyr の `st,lsm6dso` です。
- `fifo_read` の転送時間（統計の `transfer_time_ns`）は CPU のクロック（SysTick、8 ns 単位）で測ります。

### NuttX

`nuttx/drivers/lsm6dso/Make.defs` がドライバーのソース（コア、`port/zephyr/`、`common/drivers/stmemsc/`）を `VPATH` で参照し、オブジェクトは
`nuttx/drivers` にできます。共有のソースには前置きヘッダー（`common/compat/port/nuttx/common_prelude.h`）を付けてビルドします。オプションは
`nuttx/drivers/lsm6dso/Kconfig` の `CONFIG_LSM6DSO_LOCAL`（既定で有効。`I2C_DW_I2C0` が要る）と `CONFIG_LSM6DSO_STREAM`（`LSM6DSO_LOCAL` と一緒に常に有効）です。

| ファイル | 役割 |
|---|---|
| `port/nuttx/lsm6dso_instance.h` | デバイス。Zephyr 版のデバイスツリーのインスタンス化のマクロ（`port/zephyr/lsm6dso_instance.h`）の代わりに、ボードの値を使う `DEVICE_DT_DEFINE()` 1 つ（INT1 の `.fifo_gpio` は `<&gpio0 24 0>`）。`lsm6dso_sensor.c` は末尾で `<lsm6dso_instance.h>` を読み、インクルードパスで `port/nuttx/` が `port/zephyr/` より前にあるので NuttX 版のものになり、`lsm6dso_sensor_init` と `lsm6dso_driver_api` は `static` のまま |
| `nuttx/drivers/lsm6dso/st_lsm6dso_reg.c` | ST の `lsm6dso_reg.c`（`nuttx/stmemsc/`）をそのままビルドするための 1 行の包み |
| ボードの `src/rp2040_lsm6dso.[ch]` | I2C0 を互換層のバスのデバイス（`i2c0`、1 MHz）にし、`lsm6dsox` を初期化する（`rp2040_bringup()` から） |
| `common/compat/port/nuttx/rp2040_gpio_os.c` | 互換層の `gpio0` のための GPIO バンク 0（入力の設定、レベルの読み出し、エッジ・レベルの割り込み）。ボードの `libboard.a` に入る（ボードの `src/Make.defs` が `VPATH` で参照する） |

ドライバーが使う OS の機能の、NuttX 版での中身（コアのものは `port/zephyr/lsm6dso_os_impl.h`・`lsm6dso_stream_req_impl.h` を通し、つなぎのものは直接）:

| Zephyr | NuttX 版 |
|---|---|
| `<zephyr/logging/log.h>`、`CONFIG_SENSOR_LOG_LEVEL`（3） | 互換層の `<zephyr/logging/log.h>`（`app_log.h`。同じ行が RAM ログに入る）、レベル 3 は前置きヘッダー |
| `k_mutex_init/_lock/_unlock` | 互換層の `k_mutex`（`rmutex_t`。`K_FOREVER` の待ちはシグナルで終わってもやり直す） |
| `k_spin_lock/_unlock` | 互換層の `k_spin_lock()`（`sched_lock()` と割り込み禁止） |
| `k_busy_wait()` | 互換層のもの。`up_udelay()` |
| `k_msleep()`、`k_usleep()` | 互換層のもの。`nxsched_msleep()`、`nxsched_usleep()`（tick に切り上げて 1 tick 足す。Zephyr と同じ規則、同じ 100 µs） |
| `k_cycle_get_32()`、`k_cyc_to_ns_floor64()` | 互換層のもの。TIMER の下位 32 ビット（1 MHz）と ×1000 |
| `i2c_burst_read_dt()`（コアの FIFO の読み出し） | 互換層の I2C0 の呼び出し |
| `k_work_queue_start()` のキューの上の `k_work_delayable`、`k_sys_work_q` の上の `k_work`（ストリームのワークと INT1 のワーク） | 互換層のワークキュー（`imu_fifo_worker` は優先度 108、`sysworkq`） |
| `gpio_*_dt()`、`sensor_clock_get_cycles()`、RTIO（ストリーム） | 互換層のもの。ボードの `gpio0` と TIMER の上 |
| `CONFIG_SENSOR_ASYNC_API` | `CONFIG_LSM6DSO_STREAM` があれば前置きヘッダーが定義する（Zephyr 版と同じく、無いと `lsm6dso_sensor.c` はビルドできない） |
| `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` | 互換層の `<zephyr/devicetree.h>`。`st_lsm6dso` の `i2c` だけ 1（ボードの LSM6DSOX は I2C0 の 1 個） |
| デバイスツリーのインスタンス化、`POST_KERNEL`・優先度 90 | `DEVICE_DT_DEFINE(lsm6dsox, …)`（`lsm6dso_instance.h`）、ボードの起動処理での `device_init()` |

- 初期化はボードの `rp2040_bringup()` が、I2C0 の初期化の直後、KV（NVS）のマウントより前に行います（Zephyr 版の初期化の順と同じ）。失敗しても
  デバイスが「準備できていない」だけで起動は続きます（`device_is_ready()` が false。Zephyr の `do_device_init()` と同じ）。
- `fifo_read` の転送時間（統計の `transfer_time_ns`）は TIMER（1 µs 単位）で測ります。

### Pico SDK（FreeRTOS・ThreadX で共有）

| ファイル | 役割 |
|---|---|
| `port/pico_sdk/lsm6dso.cmake` | ビルドの規則（`pico_sdk_lsm6dso()`）。NuttX 版の `CONFIG_LSM6DSO_LOCAL`・`CONFIG_LSM6DSO_STREAM` と同じソース（コア、`port/zephyr/` のつなぎ、`common/drivers/stmemsc/`、ST の `lsm6dso_reg.c`）と、互換層のデバイスとセンサーの API を入れる。`CONFIG_I2C_DW_I2C0` が無いと構成を止める |
| `port/pico_sdk/lsm6dso_instance.h` | デバイス。Zephyr 版のデバイスツリーのノード `lsm6dsox@6a` の値で 1 つだけ作る（NuttX 版のボードの `BOARD_LSM6DSOX_*` と同じ値）。インクルードパスで `port/zephyr/` より前 |
| `port/pico_sdk/lsm6dso_board.c`、`lsm6dso_board.h` | その開始 `lsm6dso_board_initialize()`。I2C コアのデバイス `i2c0`（`i2c_dw_i2c0_initialize()` のあと）の上で、Zephyr が `main()` の前に行うのと同じ所でアプリ（`common/imu/port/pico_sdk/usb_console.c`）が呼ぶ。`i2c0` が準備できていなければセンサーに触れず `-ENODEV` |
| `common/compat/port/pico_sdk/gpio_os_rp2040.c` | 互換層の `gpio0` の下の GPIO バンク 0 |

- ST の `lsm6dso_reg.c` は、各 OS の `setup.sh` が `freertos/stmemsc/`・`threadx/stmemsc/` に取得します（`PICO_SDK_STMEMSC_DIR`）。
- GPIO（`gpio_os_rp2040.c`）: 互換層の `gpio0` の下の GPIO バンク 0 を、Zephyr の `gpio_rpi_pico.c` と同じ Pico SDK の呼び出しで操作します。割り込みは
  `gpio_add_raw_irq_handler_masked()` で登録し、使えるのは LSM6DSOX の INT1（GPIO24）だけです（ほかのピンは `-ENOTSUP`。NINA の READY と IRQ の線は
  NINA のつなぎのハンドラー）。

### FreeRTOS

FreeRTOS だけの部分はありません（上の「Pico SDK」と互換層の FreeRTOS の部分の上）。

### ThreadX

ThreadX だけの部分はありません（上の「Pico SDK」と互換層の ThreadX の部分の上）。

## `sensor` コマンド

`common/shell/sensor_shell.c` は、Zephyr の `drivers/sensor/sensor_shell.c` にローカルのパッチ
`zephyr/manifest/zephyr/patches/zephyr/0006-sensor-shell-fifo-attributes.patch` を当てたもので、4 つの OS とも同じファイルです
（`zephyr_sensor_shell_patched` が確かめる）。パッチの内容:

- フレーム数が 0 のチャンネルと、キャンセル時などのバッファのない完了通知を表示から除きます。
- `fifo_watermark` と `fifo_timestamp_decimation` を `sensor attr_set` / `attr_get` で扱えます。
- パッチは `drivers/sensor/sensor_shell_os.h` も足し、`common/drivers/lsm6dso/` のドライバー（その `<zephyr/drivers/sensor/lsm6dso.h>`）が要ります。

`sensor stream` と `imu start` は同時に使えません（FIFO はチップに 1 つ）。

`sensor_shell.c` が最初に読む `sensor_shell_os.h` だけは OS ごとのものです:

| OS | `sensor_shell_os.h` | ビルド |
|---|---|---|
| Zephyr | パッチを当てたツリーの `drivers/sensor/sensor_shell_os.h`（何もしないマクロ 2 つ） | ツリーの中のもとのファイルと、Zephyr 自身の `sensor_shell_stream.c` |
| NuttX | `common/shell/port/nuttx/sensor_shell_os.h`（`sensor_shell_nuttx.[ch]`） | `nuttx/apps/sensor`（`sensor_shell.c`、`common/compat/sensor_shell_stream.c`、`sensor_shell_nuttx.c`） |
| FreeRTOS・ThreadX | `common/shell/port/generic/sensor_shell_os.h`（`sensor_shell_os.c`） | `common/shell/port/pico_sdk/sensor_shell.cmake`（`sensor_shell.c`、`common/compat/sensor_shell_stream.c`、`sensor_shell_os.c`） |

NuttX 版・FreeRTOS 版・ThreadX 版では、`sensor get` が結果の表示を待ちます（最長 2 秒）。Zephyr の `sensor get` はすぐ戻り、処理のスレッド
（`sensor_shell_tid`）の行がプロンプトのあとに出て、Zephyr のシェルはその下にプロンプトを描き直します。ほかの OS のシェルの行編集はほかの
タスクの出力のあとにプロンプトを描き直さないので、コマンドが処理のスレッドの表示を待ってからプロンプトに戻り、端末には Zephyr と同じく
結果とその下のプロンプトが出ます。

## テスト

共有ソースのテスト（`common/tests/run.sh`。4 つの OS の `host_tests.sh` が最初に実行する）:

| テスト | 対象 |
|---|---|
| `fifo_words` | FIFO ワードの解析 |
| `fifo_decoder` | FIFO バッファの decoder（`port/zephyr/lsm6dso_decoder.c`） |
| `lsm6dso_cal` | 物理量への変換と、保存する校正値の形式（`lsm6dso_cal.c`） |
| `lsm6dso_zero` | 静止時の角速度のバイアスの追従（`lsm6dso_cal.c` の `lsm6dso_zero_*()`） |
| `lsm6dso_stream` | ストリームの状態機械（`lsm6dso_stream.c`）を、境界のホストでの実装（`common/tests/lsm6dso_os_host/`）とコアの FIFO の呼び出しの偽物の上で: 開始（INT1 の線が無い、FIFO と INT1、開始時にすでに有効なレベル、INT1 の有効化・レベルの読み取り・割り当ての失敗）、エッジでのバッチとそのヘッダー、watermark 未満では渡さないこと、オーバーラン・キャンセル・要求が消えたときの終了、最後の端数のバッチを渡す穏やかな停止とそのエラー、2 つ目の要求、watermark の範囲 |
| `lsm6dso_core_independent` | コアの各ファイル（直下の `.c`、`include/lsm6dso_cal.h`・`lsm6dso_fifo.h`、`internal/`、`port/` の境界）が、C ライブラリー・ST の `lsm6dso_reg.h`・自分のヘッダー・ホストでの境界の実装（`common/tests/lsm6dso_os_host/` の 2 つ）だけでコンパイルできること |
| `lsm6dso_public_headers` | `include/` のヘッダーがそのディレクトリーだけでコンパイルできること（`lsm6dso.h` は sensor API と RTIO の偽物と一緒に） |
| `zephyr_sensor_shell_patched` | `common/shell/sensor_shell.c` が、Zephyr の HEAD の `drivers/sensor/sensor_shell.c` にパッチ 0006 を当てたものとバイト単位で同じこと。Zephyr のツリーがあるときだけ |

Zephyr を上げるときは、0006 を `common/shell/sensor_shell.c` から作ります（先に新しいツリーの上流のファイルの変更をそこへ入れる）。そうすると
`zephyr_sensor_shell_patched` が通ります。

各 OS のテスト:

| テスト | 対象 |
|---|---|
| NuttX: `lsm6dso_driver` | ドライバー（コアの `lsm6dso.c` と、`port/nuttx/lsm6dso_instance.h` を付けた `port/zephyr/lsm6dso_sensor.c`）と FIFO のストリーム（`lsm6dso_stream.c`、`lsm6dso_rtio_stream.c`、`lsm6dso_os.c`、`lsm6dso_rtio.c`、デコーダー）を、本物の ST の `lsm6dso_reg.c`、互換層（I2C、デバイス、ワークキュー、RTIO、`gpio0`、既定の sensor の iodev とデコーダー）とボードのデバイスの上で、偽の I2C マスターの先の偽のセンサーと偽の INT1 を相手に（`nuttx/tests/lsm6dso_driver.c`） |
| NuttX: `sensor_shell` | `sensor` コマンド（`nuttx/tests/sensor_shell.c`） |
| FreeRTOS: `gpio` | GPIO バンク 0 の境界、`gpio0`、FIFO の割り込み |
| FreeRTOS: `sensor_shell` | `sensor` コマンド、2 つのセッション、校正 |

ST の `lsm6dso_reg.c`（`setup.sh` が取得）が無ければ、それを使うテスト（`lsm6dso_core_independent`、NuttX の `lsm6dso_driver`・`sensor_shell`、
FreeRTOS の `sensor_shell`）は `skip` になります。
