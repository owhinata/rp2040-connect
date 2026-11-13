# ドライバー（NuttX 版）

[English](README.md) | 日本語

`nuttx/drivers/` は NuttX のカーネル側でビルドするこの移植のコードです（`nuttx/nuttx/external` からの
symlink で、`libexternal.a` になります）。ボード側の部分は `nuttx/boards/arduino-nano-rp2040-connect/src/`
にあります。Zephyr 版のドライバー説明は `zephyr/app/usb-console/drivers/README_ja.md` です。この文書は
移植の途中で、移植した部分だけを書いています。

## NINA-W102（`nina`）

### SPI の転送層

| ファイル | 役割 |
|---|---|
| `nina/nina_spi.c` | コマンドと応答のフレーミング（`nina_request()`、`nina_ip_config()`）。Zephyr 版の `drivers/nina/nina_wifi.c` の同名の関数と、分岐・順序・定数・エラー値を 1 対 1 で対応させた写し |
| `nina/nina_proto.h` | 公開する宣言。Zephyr 版の同名ファイルを変更せずに写したもの |
| `nina/nina_os.[ch]` | ロック（再帰可能なミューテックス、優先度継承あり）、sleep、ms の時計。Zephyr の `K_MUTEX`・`k_sleep()`・`k_uptime_get()` に当たる |
| `include/nano_nina.h`、ボードの `src/nano_nina.c` | SPI1 と 4 本の信号線（CS、READY、RESET、IRQ）。Zephyr のデバイスツリーと SPI・GPIO ドライバーが行っていた設定に当たる |

- 要求は呼び出したスレッドの文脈で同期して処理します。ミューテックスで直列化し、CS を下げてから
  応答を読み終えるまで（READY を待つ間も）CS を保持します。
- NINA のリセットは最初の要求のときに 1 回だけ行います（10 ms、750 ms）。起動時にはボードが SPI1 と
  そのピンを設定し、CS を非アクティブにするだけです。
- SPI は NuttX の `rp2040_spi.c` をポーリングで使います（DMA なし）。転送ごとに `SPI_LOCK` を取り、
  最初の転送でモード 0・8 ビット・`CONFIG_NANO_NINA_SPI_FREQUENCY` を設定します。`SPI_SELECT` は
  使いません。
- 期限、配線、実測は `nuttx/README_ja.md` の「NINA-W102」の節にあります。
- `nina/nina_spi.c` はホストテスト（`nuttx/tests/nina_spi.c`）で、偽のボードと時計を使って確かめます。

## I2C0

| ファイル | 役割 |
|---|---|
| ボードの `src/nano_i2c.c` | I2C0 のコントローラー。Zephyr 版の `drivers/i2c_dw/i2c_dw.c` のコントローラーの部分（`i2c_dw_transfer()`、`i2c_dw_isr()`、`i2c_dw_data_ask()`・`_read()`・`_send()`、`i2c_dw_setup()`、`i2c_dw_configure()`、初期化）と DMA での読み出し（`i2c_dw_rp_dma_read()` など）を、関数・分岐・順序・レジスター値・期限・エラー値を合わせて写したもの。NuttX の `struct i2c_master_s`（`I2C_TRANSFER()`、`I2C_RESET()`）として出す |
| ボードの `src/nano_i2c.h` | `nano_i2c0_initialize()`（起動時に `nano_bringup()` が呼び、`/dev/i2c0` に登録する） |
| ボードの `src/nano_i2c_dw.h` | OS に依存しない部分: Zephyr 版の `i2c_dw.h` の定義、SCL のカウント、周波数から速さの区分、NuttX のフラグ（`I2C_M_*`）から Zephyr のフラグ（`I2C_MSG_*`）への変換、`IC_DATA_CMD` に書く語。ホストテスト `nuttx/tests/nano_i2c_dw.c` |
| ボードの `src/nano_i2c_dw_registers.h` | Zephyr 版の `i2c_dw_registers.h` の写し（アクセス関数だけ NuttX 向けに定義） |
| ボードの `src/nano_dma.c`・`nano_dma.h` | DMA 読み出しが使う RP2040 の DMA。Zephyr の `drivers/dma/dma_rpi_pico.c`（`dma_config()`・`dma_start()`・`dma_stop()`・割り込み）と、DMA API のチャネルの確保・解放（`dma_request_channel()`・`dma_release_channel()`）、`i2c_dw.c` が直接呼ぶ pico-sdk の関数（`dma_channel_is_busy()`・`dma_irqn_acknowledge_channel()`・`dma_hw->abort`）をレジスターで書いたもの |
| ボードの `src/i2c_dw_rp2040_commands.[ch]` | Zephyr 版の同名ファイルを変更せずに写したもの（DMA で送るコマンド語）。ホストテスト `nuttx/tests/i2c_dma_commands.c`（これも写し） |
| `include/nano_i2c_dma.h` | `i2c_dma` コマンド（`nuttx/apps/i2c_dma`）が読み書きする状態: DMA の有効・無効と 4 つのカウンター |

- 転送は呼び出したスレッドが進め、バイトの出し入れは割り込みで行います。呼び出し元はセマフォで完了を
  待ちます（期限 1 秒。過ぎると abort して `-ETIMEDOUT`）。転送の全体をバスのセマフォで直列化します
  （Zephyr 版と同じく優先度継承なし）。
- NuttX のメッセージは、最後のメッセージと、`I2C_M_NOSTOP` がなく次が `I2C_M_NOSTART` でないメッセージの
  後に STOP、`I2C_M_NOSTOP` の後の `I2C_M_NOSTART` でないメッセージの先頭に RESTART を付けて、Zephyr の
  メッセージと同じに扱います。向きが変わると（前の転送の最後のメッセージから見ても）RESTART を付けるのは
  Zephyr 版と同じです。
- 速さはメッセージの周波数（100 kHz、400 kHz、1 MHz）で決め、変わったときに Zephyr の `i2c_configure()` と
  同じ計算をします。認めない速さでは何も変えずに `-EINVAL` を返します（Zephyr 版は最小のカウントを
  残してしまう）。
- 移していないもの: ターゲット（スレーブ）モード、Intel LPSS の DMA、`CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`、
  SDA/SCL の期限（`CONFIG_I2C_DW_EXTENDED_SUPPORT`）、電源管理、PCIe、ほかの SoC 向けの処理、バスのリカバリーと
  確認のコールバック（このボードでは登録されず、何もしない）、i2c1。
- DMA での読み出し（`CONFIG_NANO_I2C0_DMA`、既定で有効。Zephyr 版の `CONFIG_I2C_DW_RP2040_DMA`）は
  `i2c_dw.c` の DMA の部分を同じ分岐・順序・値で写したものです:
  - 使う条件: DMA が有効（`i2c_dma mode`）、DMA の準備ができている、32〜3584 バイト
    （`I2C_DW_RP2040_DMA_MAX_READ`）、STOP で終わる、転送の最後のメッセージ、7 ビットアドレス。外れると
    `-EAGAIN` で割り込みの処理に戻ります。チャネルの確保か設定に失敗したときも、`dma_fallbacks` を
    数えて割り込みの処理に戻ります（転送はまだ始まっていません）。
  - TX のチャネルが読み出しのコマンド語（16 ビット。各語に READ、先頭は RESTART が要るときだけ RESTART、
    最後に STOP）を `IC_DATA_CMD` に書き、RX のチャネルが `IC_DATA_CMD` から 8 ビットずつ受け取ります。
    `IC_DMA_TDLR` は 8、`IC_DMA_RDLR` は 0、割り込みは STOP_DET・TX_ABRT・TX_OVER・RX_OVER・RX_UNDER・
    SCL_STUCK_LOW だけ。RX、TX の順にチャネルを始め、`IC_DMA_CR` に 3 を書きます。
  - TX の完了、RX の完了、STOP の 3 つがそろうか、エラー（I2C の異常の割り込み、DMA のバスエラー）で
    終わります。待ちは転送の期限（1 秒）を全体の期限として、残りの時間で待ちます。エラーは `-EIO`、
    期限切れは `-ETIMEDOUT`。
  - 後始末: `IC_DMA_CR` を 0、割り込みマスクを 0 にし、失敗なら I2C を abort、DMA の 2 チャネルを止めます
    （割り込みを外し、CHAN_ABORT を書いて、BUSY と CHAN_ABORT が落ちるまで 1 ms ごとに最長 100 ms 待ち、
    遅れて来た完了の割り込み（RP2040-E13）を DMA_IRQ_0・1 の両方で確認）。止まらなければコントローラーを
    隔離し（`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`、`-EIO`、チャネルは解放
    しない）、以後の転送は `-EIO` です（再起動で戻ります）。止まれば割り込みを消し、失敗なら abort の
    完了を最長 100 ms 待ちます（終わらなければ隔離して
    `<err> i2c_dw: I2C abort did not finish; controller quarantined`）。
  - DMA は Zephyr の RP2040 用ドライバーと同じく、チャネル 0 を DMA_IRQ_0、ほかを DMA_IRQ_1 に割り当て、
    チャネルの確保は待たず、停止も待ちません（待つのは上の I2C 側）。NuttX の `rp2040_dmac.c`
    （`CONFIG_RP2040_DMAC`）は DMA_IRQ_0 をすべてのチャネルで使い、確保で空きを待ち、停止で期限なしに
    待つので、切ってあります（DMA を使うのはこの読み出しだけ。NINA の SPI は DMA を使いません）。
  - `rp_dma_events` の更新は割り込みを禁止して行い（Zephyr の `atomic_or()`）、4 つのカウンターは
    バスのセマフォを持った転送だけが書きます（32 ビットで一周します）。

## LSM6DSOX（`lsm6dso`）

| ファイル | 役割 |
|---|---|
| `lsm6dso/lsm6dso.c` | ドライバー本体。Zephyr 版の `drivers/lsm6dso/lsm6dso.c` の、OS を呼ぶ部分だけを置き換えた写し（関数・分岐・順序・レジスター操作・換算・エラー値・ログの文は同じ）。末尾のデバイスツリーのインスタンス化のマクロは、ボードの値を使う `DEVICE_DT_DEFINE()` 1 つに置き換え |
| `lsm6dso/lsm6dso.h` | Zephyr 版の同名ファイルの写し。カーネルのオブジェクトの型（`k_mutex`、`k_spinlock`）、バスの条件（`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` → `LSM6DSO_BUS_I2C`）、ストリームの条件（`CONFIG_SENSOR_ASYNC_API` → `CONFIG_NANO_LSM6DSO_STREAM`）だけを置き換え。`timestamp_decimation` はストリームの外に出した（下） |
| `lsm6dso/lsm6dso_os.[ch]` | カーネルの機能: ミューテックス（再帰可能、優先度継承あり）、割り込み禁止（`k_spin_lock()`）、`k_busy_wait()`、`k_msleep()`・`k_usleep()`、`k_cycle_get_32()` |
| `lsm6dso/stmemsc.h`、`stmemsc_i2c.c`、`stmemsc_mdelay.c` | Zephyr の `drivers/sensor/st/stmemsc/` の I2C の部分の写し（ST の `stmdev_ctx_t` と Zephyr の I2C の API のつなぎ） |
| `lsm6dso/st_lsm6dso_reg.c` | ST の `lsm6dso_reg.c`（`nuttx/stmemsc/`、Git 管理外）をそのままビルドするための 1 行の包み |
| `lsm6dso/lsm6dso_fifo_hw.[ch]`、`lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h` | Zephyr 版の同名ファイルを変更せずに写したもの |
| ボードの `src/nano_lsm6dso.[ch]` | I2C0 を互換層のバスのデバイス（`i2c0`、100 kHz）にし、`lsm6dsox` を初期化する（`nano_bringup()` から） |

置き換えの対応（`lsm6dso.c` の先頭にも同じ表があります）:

| Zephyr | NuttX 版 |
|---|---|
| `<zephyr/logging/log.h>`、`CONFIG_SENSOR_LOG_LEVEL`（3） | `app_log.h`（同じ行が RAM ログに入る）、レベル 3 |
| `k_mutex_init/_lock/_unlock` | `lsm6dso_os_mutex_init/_lock/_unlock`（`rmutex_t`。シグナルで終わった待ちはやり直す） |
| `k_spin_lock/_unlock` | `lsm6dso_os_spin_lock/_unlock`（`up_irq_save()`） |
| `k_busy_wait()` | `up_udelay()` |
| `k_msleep()`、`k_usleep()` | `nxsched_msleep()`、`nxsched_usleep()`（tick に切り上げて 1 tick 足す。Zephyr と同じ規則、同じ 100 µs） |
| `k_cycle_get_32()`、`k_cyc_to_ns_floor64()` | TIMER の下位 32 ビット（1 MHz）と ×1000 |
| `sys_get_le32()` | 同じ計算をファイル内に書いた |
| デバイスツリーのインスタンス化、`POST_KERNEL`・優先度 90 | `DEVICE_DT_DEFINE(lsm6dsox, …)`、ボードの起動処理での `device_init()` |

- 初期化はボードの `nano_bringup()` が、I2C0 の初期化の直後、KV（NVS）のマウントより前に行います
  （Zephyr 版の初期化の順と同じ）。失敗してもデバイスが「準備できていない」だけで起動は続きます
  （`device_is_ready()` が false。Zephyr の `do_device_init()` と同じ）。
- FIFO の関数（`lsm6dso_fifo_*_locked_driver()`、`lsm6dso_timestamp_read()`、`lsm6dso_fifo_get_stats()` など）は
  ストリームが無くてもビルドします。そのため、Zephyr 版でストリームの部分にある `timestamp_decimation` を
  構造体のストリームの外に置きました。ストリームが値を設定するまでは 0（タイムスタンプのワードなし）です。
  ストリーム（`lsm6dso_stream.c`、`lsm6dso_fifo_irq.c`、RTIO、デコーダー、`lsm6dso_compat.c`）とその属性
  （`fifo_watermark`、`fifo_timestamp_decimation`）は、`CONFIG_NANO_LSM6DSO_STREAM` を足すときに入ります。
- 移していないもの: SPI 接続、センサーハブ（`lsm6dso_shub.c`）、トリガー（`lsm6dso_trigger.c`）、内部温度。
  どれも Zephyr 版の構成で切られていて（`CONFIG_LSM6DSO_SENSORHUB`・`_TRIGGER`・`_ENABLE_TEMP` が無効）、
  ビルドされていません。該当する `#ifdef` の部分は写しに残してあります。
- `fifo_read` の転送時間（統計の `transfer_time_ns`）は TIMER（1 µs 単位）で測ります。Zephyr 版は CPU の
  クロック（SysTick、8 ns 単位）です。

## Zephyr の API の互換層（`compat`）

`nuttx/drivers/compat/` は、写したドライバーとそれを呼ぶ側（取得サービス、校正、センサーのコマンド）が使う
Zephyr の API を、Zephyr の名前のまま最小限だけ用意したものです。インクルードパスは
`$(TOPDIR)/external/compat` です。Zephyr の sensor サブシステムの全体は移していません。

| ヘッダー | 中身 |
|---|---|
| `zephyr/device.h` | `struct device`（名前、config、API、状態、data、初期化関数）、`device_is_ready()`、`device_init()`（Zephyr の `do_device_init()` と同じ意味）、`DEVICE_DT_GET()`、`DEVICE_DT_DEFINE()`、`DEVICE_API()` |
| `zephyr/devicetree.h` | `DT_NODELABEL()`、`DT_BUS()`。ノードは `i2c0` と `lsm6dsox` だけ |
| `zephyr/drivers/sensor.h` | `struct sensor_value`、チャネル・属性・トリガーの列挙（Zephyr と同じ番号）、`struct sensor_driver_api`、`sensor_attr_set()`・`_get()`、`sensor_sample_fetch()`・`_chan()`、`sensor_channel_get()`、`sensor_trigger_set()`、単位の換算（`sensor_ms2_to_g()`、`sensor_rad_to_degrees()`、`sensor_value_to_micro()` など） |
| `zephyr/drivers/i2c.h` | `struct i2c_dt_spec`、`i2c_burst_read_dt()`、`i2c_write_dt()`、`i2c_write_read_dt()`、`i2c_configure()`、速さの区分（`I2C_SPEED_*`、`I2C_SPEED_SET()`） |
| `zephyr/sys/util.h` | `BIT()`、`ARRAY_SIZE()`、`MIN()`・`MAX()`、`IS_ENABLED()`、`CONTAINER_OF()` など |
| `zephyr/rtio/rtio.h` | 仮置き（`lsm6dso_fifo_stats.h` が使う 2 つの名前だけ）。RTIO の互換はストリームと一緒に入る |

- I2C のバスは、NuttX の I2C マスターと速さを持つデバイスです（`i2c_compat_bind()` でボードが結び付ける）。
  Zephyr ではバスのドライバーが速さを持ち、`i2c_configure()` が以後のすべての転送の速さを変えます。NuttX の
  転送はメッセージごとに周波数を持つので、互換層の転送（`i2c_burst_read_dt()` など）がデバイスの速さを
  メッセージに入れます。`i2c` ツールなど、NuttX のマスターを直接使うものは自分の周波数で動きます。
- `i2c_configure()` は Zephyr の RP2040 向けドライバーが受け付ける区分（100 kHz、400 kHz、1 MHz）だけを受け付け、
  ほかは何も変えずに `-EINVAL` を返します。モードのビットは見ません（Zephyr のドライバーもコントローラーに
  固定）。DMA の不具合で I2C0 が隔離されているとき、Zephyr の `i2c_configure()` は `-EIO` を返しますが、
  互換層は速さを記録して 0 を返し、次の転送が `-EIO` になります。
- NuttX の `<nuttx/i2c/i2c_master.h>` には同じ名前の `i2c_write()`・`i2c_read()` と、Hz の値の `I2C_SPEED_*` が
  あります。互換層では、バスのデバイスに対する Zephyr の `i2c_write()`・`i2c_write_read()` を
  `i2c_compat_write()`・`i2c_compat_write_read()` という名前にしました（`_dt` の付く形は Zephyr の名前のまま）。
  `<zephyr/drivers/i2c.h>` を読むファイルでは `I2C_SPEED_*` は Zephyr の区分です（両方読むなら NuttX のものを先に）。

Wi-Fi の管理とソケット（usrsock）はまだ移植していません。
