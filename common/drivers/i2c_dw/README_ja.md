# DesignWare I2C のコア（`i2c_dw`）

[English](README.md) | 日本語

このリポジトリーのファームウェアは、Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上でそれぞれビルドでき、
それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。互換層は、Zephyr の API を Zephyr の名前のまま
ほかの OS の上で提供する層です（[`common/compat/README_ja.md`](../../compat/README_ja.md)）。

`common/drivers/i2c_dw/` は、RP2040 の I2C0 と I2C1 のドライバーです。Zephyr 版のドライバー（Zephyr の `drivers/i2c/i2c_dw.c` に
ローカルのパッチ `zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` を当てたもの）の写しで、
NuttX 版・FreeRTOS 版・ThreadX 版が使います。Zephyr 版はこのコアに載せず、パッチを当てた上流のドライバーを使います（下の「Zephyr」）。
I2C0 では、条件に合う読み出しを DMA で行います。`i2c` コマンドと `i2c_dma` コマンドも、この文書で説明します。
共有ソースの全体の中での位置付けは[設計書](../../../design/imu-raw-mqtt-design-ja.md)の §9.1、センサーを読む経路の中での位置は
[§3.4](../../../design/imu-raw-mqtt-design-ja.md#34-センサーを読む-3-つの経路)にあります。

## ディレクトリー

NINA のドライバーと同じ 3 層です。直下に `i2c_dw.c`、ドライバーを使う側のヘッダーは `include/`、コアの内側のヘッダーは
`internal/`、2 つの境界は `port/`、各 OS のつなぎは `port/<OS>/` と、Pico SDK の上の OS で共有する `port/pico_sdk/` にあります。

| ファイル | 役割 |
|---|---|
| `i2c_dw.c` | I2C0 と I2C1 のコントローラーのコア。Zephyr のドライバーが有効なノードごとに作るのと同じく、それぞれ 1 つのインスタンス（`i2c_0_runtime`・`i2c_1_runtime`。`CONFIG_I2C_DW_I2C0`・`CONFIG_I2C_DW_I2C1` で作る）を持ち、両者で違う値は `I2C_DW_INST()` で選ぶ。Zephyr 版の `i2c_dw.c`（パッチ 0003 を当てたもの）のコントローラーの部分（転送の状態機械、割り込みハンドラーの本体、`i2c_dw_data_ask()`・`_read()`・`_send()`、`i2c_dw_setup()`、`i2c_dw_configure()`、初期化のレジスター操作）と DMA での読み出し（`i2c_dw_rp2040_dma_read()` など）と `i2c_dma` のカウンターを、関数・分岐・順序・レジスター値・期限・エラー値を合わせて写したもの。意図した違いは冒頭の一覧に限る。先頭で境界 `port/i2c_dw_os.h` を、末尾でつなぎの `i2c_dw_instance.h` を読む |
| `include/i2c_dw_rp2040_dma.h` | `i2c_dma` コマンドが読み書きする状態: DMA の有効・無効と 4 つのカウンター（`i2c_dw_rp2040_dma_get_stats()`・`_enabled()`・`_set_enabled()`）。宣言はパッチ 0003 が Zephyr のツリーに足すヘッダーと同じ（`i2c_dw_sync` が確かめる）。`include/` のヘッダーはこれだけで、そのディレクトリーだけでコンパイルできる（`i2c_dw_public_headers`） |
| `internal/i2c_dw.h` | OS に依存しない定義と純粋な関数: Zephyr 版の `i2c_dw.h` の定義、SCL のカウント、周波数から速さの区分、NuttX のフラグ（`I2C_M_*`）から Zephyr のフラグ（`I2C_MSG_*`）への変換、`IC_DATA_CMD` に書く語 |
| `internal/i2c_dw_registers.h` | Zephyr 版の `i2c_dw_registers.h` の写し（アクセス関数は 32 ビットの volatile のアクセスで定義。番地の型は Zephyr の `mm_reg_t` の代わりに `uintptr_t`） |
| `internal/i2c_dw_rp2040.h` | RP2040 の FIFO の深さ（16）と RX のまとめ数（4）。パッチの `drivers/i2c/i2c_dw_rp2040.h` と同じ。`i2c_dw.c` が 1 回だけ読む |
| `internal/i2c_dw_rp2040_dma_priv.h` | DMA での読み出しとそのコマンド語（`i2c_dw_rp2040_read_commands()`）。パッチの同名のファイルを同じ分岐・順序・値で写したもの。`i2c_dw.c` が 1 回だけ読む |
| `port/i2c_dw_os.h` | 境界: コアが OS に求めるもの（下の「OS が実装する境界」）。先頭でつなぎの `i2c_dw_os_impl.h` を読み、その実装が満たすべき関数を宣言する |
| `port/i2c_dw_dma_os.h` | DMA 読み出しの DMA の境界: OS 自身の DMA ドライバーに求めるもの。パッチを当てたドライバーの Zephyr の DMA API の呼び出しに 1 つずつ対応する。OS の名前を含まず、コアは DMA のレジスターに触れない |
| `port/nuttx/` | NuttX へのつなぎ: `i2c_dw_os_impl.h`、`i2c_dw_instance.h`、`i2c_dw_dma_os_rp2040.c`、`i2c_dw_controllers_rp2040.h`（下の「NuttX」） |
| `port/pico_sdk/` | Pico SDK の上の OS で共有するつなぎ: `i2c_dw.cmake`、`i2c_dw_instance.h`、`i2c_dw_controllers_rp2040.h`、`i2c_dw_dma_os_rp2040.c`（下の「Pico SDK」） |
| `port/freertos/i2c_dw_os_impl.h` | 境界の FreeRTOS の実装（下の「FreeRTOS」） |
| `port/threadx/i2c_dw_os_impl.h` | 境界の ThreadX の実装（下の「ThreadX」） |

ドライバーの外にある関係するファイル:

| ファイル | 役割 |
|---|---|
| `common/shell/i2c_shell.c` | `i2c` コマンド。Zephyr の `drivers/i2c/i2c_shell.c` にローカルのパッチ 0004 を当てたものとバイト単位で同じ写し（下の「`i2c` コマンドと `i2c_dma` コマンド」） |
| `common/shell/i2c_dma_shell.c` | `i2c_dma` コマンド。4 つの OS のファームウェアで共有 |
| `common/compat/include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` | 互換層の `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>`。`include/i2c_dw_rp2040_dma.h` を読む |
| `common/tests/i2c_dw.c`、`i2c_dw_sync.py`、`i2c_dw_sync.expected` | ホストテスト（下の「テスト」） |
| `nuttx/drivers/i2c_dw/Kconfig`、`Make.defs` | NuttX 版のオプションとビルドの規則 |

## コアの動作

### どのコントローラーでも

- 0 バイトのメッセージを含む転送は、どのメッセージも送る前に `-ENOTSUP` で断ります（コントローラーは 0 バイトの転送ができず、
  上流は代わりに 1 バイトを送ります）。`i2c scan` はそのとき 1 バイトの読み出しで確かめます（下の「`i2c` コマンド」）。
- バスの復旧（`i2c_recover_bus()`、シェルの `i2c recover`）は `-ENOTSUP` を返します（このボードにはバスの復旧が無い）。
  転送の前には、上流と同じく、バスが固まった状態を消して転送に進みます。

### RP2040 のコントローラーへの変更

RX・TX FIFO の深さを 16、RX しきい値を 4 バイトとし、短い読み出しと末尾ではしきい値を下げます。受信待ちで要求を追加できない間は
TX_EMPTY 割り込みを抑えます。

### 転送

- 転送は呼び出したスレッドが進め、バイトの出し入れは割り込みで行います。呼び出し元はセマフォで完了を待ちます（期限 1 秒。
  過ぎると abort して `-ETIMEDOUT`）。転送の全体をバスのセマフォで直列化します（Zephyr 版と同じく優先度継承なし）。
- 向きが変わると（前の転送の最後のメッセージから見ても）RESTART を付けます（Zephyr 版と同じ）。
- 速さを変えるときは Zephyr の `i2c_configure()` と同じ計算をします。
- デバイスごとにセマフォを持つので、2 つのバスの転送は並んで進みます。
- 移していないもの: ターゲット（スレーブ）モード、Intel LPSS の DMA、`CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`、
  SDA/SCL の期限（`CONFIG_I2C_DW_EXTENDED_SUPPORT`）、電源管理、PCIe、ほかの SoC 向けの処理、バスのリカバリーと
  確認のコールバック（このボードでは登録されない）。

### I2C0 と I2C1

パッチ後のドライバーと同じく、コアはビルドしたコントローラーごとにインスタンスを持ちます。I2C0 は LSM6DSOX がつながっているので
常に作ります（NuttX 版は `CONFIG_I2C_DW_I2C0`、既定で有効）。I2C1 は、NuttX 版の `CONFIG_I2C_DW_I2C1`、FreeRTOS 版・ThreadX 版の
`-DI2C_DW_I2C1=ON`（どれも既定で無効）のときだけ作ります。

- I2C1（ボード上では何もつながっていない）は GPIO18（SDA、Arduino の D6）と GPIO19（SCL、D7）を内部プルアップ付きで使い、
  常に割り込みで転送します（DMA での読み出しと `i2c_dma` は I2C0 だけ）。I2C0 のあとに起動し、失敗してもログに残すだけで、
  ほかは変わりません。`i2c` コマンドからは使えません（今は使うものがない）。
- I2C1 はビルドとホストテストで確かめただけで、実際のデバイスにつないで使ったことはありません。

起動時の I2C0 の速さは 4 つの OS とも 1 MHz です（Zephyr 版は `zephyr/app/usb-console/app.overlay` の `clock-frequency`、NuttX 版はボードの
`include/board.h` の `BOARD_I2C0_FREQUENCY`、Pico SDK の上の OS は `port/pico_sdk/i2c_dw_instance.h` の `I2C0_SPEED`。`i2c_dw_sync` が
3 つを比べる）。

### DMA での読み出し（`CONFIG_I2C_DW_RP2040_DMA`）

- 使う条件: I2C0、DMA が有効（`i2c_dma mode`）、DMA の準備ができている、32〜3584 バイト（`I2C_DW_RP2040_DMA_MAX_READ`）、
  STOP で終わる、転送の最後のメッセージ、7 ビットアドレス。外れると `-EAGAIN` で割り込みの処理に戻ります。書き込み、短い読み出し、
  条件に合わない転送は割り込みで処理します。チャネルの確保か設定に失敗したときも、`dma_fallbacks` を数えて割り込みの処理に
  戻ります（転送はまだ始まっていません）。
- 読み出し: DMA のチャネルを 2 つ使います。TX のチャネルが読み出しのコマンド語（16 ビット。各語に READ、先頭は RESTART が要るときだけ
  RESTART、最後に STOP）を `IC_DATA_CMD` に書き、RX のチャネルが `IC_DATA_CMD` から 8 ビットずつ受け取ります。`IC_DMA_TDLR` は 8、
  `IC_DMA_RDLR` は 0、割り込みは STOP_DET・TX_ABRT・TX_OVER・RX_OVER・RX_UNDER・SCL_STUCK_LOW だけ。RX、TX の順にチャネルを始め、
  `IC_DMA_CR` に 3 を書きます。
- 終わり: TX の完了、RX の完了、STOP の 3 つがそろうか、エラー（I2C の異常の割り込み、DMA のバスエラー）で終わります。待ちは転送の
  期限（1 秒）を全体の期限として、残りの時間で待ちます。エラーは `-EIO`、期限切れは `-ETIMEDOUT`。
- 後始末: `IC_DMA_CR` を 0、割り込みマスクを 0 にし、失敗なら I2C を abort、DMA の 2 チャネルを止めます（割り込みを外し、
  CHAN_ABORT を書いて、各チャネルの BUSY と CHAN_ABORT のそのビットが落ちるまで 1 ms ごとに最長 100 ms 待ち、遅れて来た完了
  （RP2040-E13）のフラグを消す）。止まらなければコントローラーを隔離し（`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`、
  `-EIO`、チャネルは解放しない）、以後の転送は `-EIO` です（再起動で戻ります）。止まれば割り込みを消し、失敗なら abort の完了を
  最長 100 ms 待ちます（終わらなければ隔離して `<err> i2c_dw: I2C abort did not finish; controller quarantined`）。
- `rp2040_dma_events` の更新は割り込みを禁止して行い（Zephyr の `atomic_or()`）、4 つのカウンターはバスのセマフォを持った転送だけが
  書きます（32 ビットで一周します）。
- パッチが Zephyr の DMA API（と、停止の中の Pico SDK のレジスターの関数）を呼ぶところで、コアは DMA の境界 `port/i2c_dw_dma_os.h` を
  1 つずつ呼びます。違うのは、チャネルの設定（`dma_config()` とその構造体の代わりに TX 側と RX 側の 2 つの呼び出し）と、停止の待ち
  （すべてのチャネルの abort ではなく、各チャネル自身の abort を待つ）だけです。

## OS が実装する境界

### `port/i2c_dw_os.h`

どれも static inline の関数かマクロで、OS の割り込みハンドラーと転送の入口はコアの static inline の関数を呼ぶので、コンパイラーが
それぞれを 1 つの関数にします（転送と割り込みの経路に呼び出しが増えない）。「割り込み」の印のものは割り込みハンドラーから呼ばれます。
各 OS の実装は下の「各 OS のつなぎ」にあります。

| 境界 | 意味（Zephyr） |
|---|---|
| `i2c_dw_os_sem_t` | 数を数えるセマフォ（`k_sem`。`bus_sem` は優先度継承なしの 2 値として使う） |
| `i2c_dw_os_sem_give()`（割り込み） | `k_sem_give()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `k_sem_take(sem, K_MSEC(ms))`: 取れたら 0、期限（tick に切り上げ）が先なら 0 以外。シグナルで終わらない |
| `i2c_dw_os_timepoint_t`、`i2c_dw_os_timepoint_calc(ms)`、`i2c_dw_os_sem_take_until(sem, end)` | `sys_timepoint_calc()` と `k_sem_take(sem, sys_timepoint_timeout(end))`（期限を過ぎていても待たずに 1 回取ってみる） |
| `i2c_dw_os_sem_reset()` | `k_sem_reset()` |
| `i2c_dw_os_bus_lock()`・`_unlock()` | `bus_sem` の `k_sem_take(K_FOREVER)`・`k_sem_give()`。失敗した lock の errno は転送の戻り値になる |
| `i2c_dw_os_msleep(ms)` | `k_msleep()` |
| `i2c_dw_os_uptime_ms()` | `k_uptime_get()`（マクロでもよい） |
| `i2c_dw_os_irq_lock()`・`_unlock()`（割り込み） | `irq_lock()`（`atomic_or()`）。入れ子にできる |
| `i2c_dw_os_msg_t`、`I2C_DW_OS_MSG_BUF()`・`_LEN()`・`_FLAGS()` | 転送の 1 つのメッセージ（Zephyr の `struct i2c_msg` なら `buf`・`len`・`flags`。最後のメッセージには STOP） |
| `I2C_DW_OS_DEV_HEAD` | コントローラーのデータの先頭に置く OS のデバイス（空でもよい） |
| `I2C_DW_CLOCK_SPEED` | `CONFIG_I2C_DW_CLOCK_SPEED`（MHz、125） |
| `i2c_dw_os_prepare(dw)`（`i2c_dw_instance.h`） | `reset_line_toggle_dt()` と `pinctrl_apply_state()`: I2C0 か I2C1 のリセット、そのピンを I2C（4 mA、遅いスルー、シュミットと入力を有効。I2C0 の GPIO12・13 はプルなし、I2C1 のピンは内部プルアップ） |
| `i2c_dw_os_irq_enable(dw)`（同） | `IRQ_CONNECT()`・`irq_enable()`: I2C0_IRQ か I2C1_IRQ で `i2c_dw_isr_handler(dw)` を呼ぶハンドラー |
| `app_log.h` | `LOG_MODULE_REGISTER()`、`LOG_ERR()`、`LOG_ERR_RATELIMIT()`（レベルはコアが定める 3） |
| `CONFIG_I2C_DW_I2C0`・`CONFIG_I2C_DW_I2C1` | 作るインスタンス（Zephyr ではデバイスツリーの有効なノード） |
| `CONFIG_I2C_DW_RP2040_DMA` | I2C0 の DMA 読み出しを作る（Zephyr 版と同じ名前のシンボル） |

### `port/i2c_dw_dma_os.h`

OS の DMA ドライバーが提供するものです。普通の関数です（読み出しごとに呼ばれ、バイトごとではない）。呼ぶのはバスのセマフォを持った
転送のスレッドだけで、コールバックは DMA の割り込みから呼ばれます。どれも DMA のほかの利用者を待ちません。実装は NuttX 版と
Pico SDK の上の OS が、OS 自身の DMA ドライバーの上に持ちます（`port/nuttx/`・`port/pico_sdk/` の `i2c_dw_dma_os_rp2040.c`）。

| 境界 | 意味（Zephyr） |
|---|---|
| `i2c_dw_dma_os_initialize()` | DMA のデバイスの起動（Zephyr では I2C0 より前）。何もリセットしない |
| `i2c_dw_dma_os_is_ready()` | `device_is_ready(dma)` |
| `i2c_dw_dma_os_request_channel()`・`_release_channel()` | `dma_request_channel(dma, NULL)`・`dma_release_channel()`。取得は待たない（`-EBUSY`・`-ENODEV`） |
| `i2c_dw_dma_os_config_tx()`・`_config_rx()` | 各側の `dma_config()`: 16 ビットのコマンド語を DREQ 32 の歩調で `IC_DATA_CMD` へ、そこからバイトを DREQ 33 の歩調で。優先度は通常 |
| `i2c_dw_dma_os_start()` | `dma_start()`: チャネルの古いフラグを消し、割り込みを有効にする |
| コールバック（割り込み） | チャネルの終了（0）またはバスエラー（`-EIO`） |
| `i2c_dw_dma_os_stop()` | `dma_stop()`: 割り込みを切って abort を求め、待たない |
| `i2c_dw_dma_os_is_busy()` | `dma_channel_is_busy()` と、チャネルの CHAN_ABORT のビット |
| `i2c_dw_dma_os_acknowledge()` | 遅れて来た完了のフラグ（RP2040-E13）を消す |

### DMA を使うほかのコード

同じプログラムのほかのコードも、OS の DMA を通常どおり使えます。読み出しが持つ 2 つのチャネルに触れない、DMA のブロックを
リセットしない、チャネルが共有するレジスター（割り込みの有効・状態、CHAN_ABORT）を丸ごと書かない（自分のチャネルのビットだけにする）。
割り込みの線の扱いなど、OS ごとの規則は下の各 OS の節にあります。

## 各 OS のつなぎ

### Zephyr

Zephyr 版はこのコアを使いません。Zephyr の DesignWare I2C ドライバー（`drivers/i2c/i2c_dw.c`）に、ローカルのパッチ
`zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` を当てて使います（パッチの一覧と当て方は [`design/zephyr-port-ja.md`](../../../design/zephyr-port-ja.md)）。
ドライバーに要るものはすべてパッチが Zephyr のツリーの中に置き、アプリはドライバーのライブラリーにソースもインクルードのパスも
足しません。上の「コアの動作」は、このパッチ後のドライバーの動作でもあります。ほかのコントローラーの処理は、「どのコントローラーでも」の
2 点のほかは Zephyr のままです。

- パッチは、定数を新しいファイル `drivers/i2c/i2c_dw_rp2040.h` に、DMA の塊と読み出しのコマンド語を `i2c_dw_rp2040_dma_priv.h`
  （`i2c_dw.c` が 1 回だけ読む）に、`i2c_dma` コマンドの関数の宣言を `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` に置き、上流のファイルに
  入るのは 16 か所（`i2c_dw.c` 14、`i2c_dw.h` 1、`Kconfig.dw` 1）です。
- `CONFIG_I2C_DW_RP2040_DMA` はパッチでは既定で無効、`prj.conf` で有効です。1 回の読み出しに、Zephyr の DMA ドライバー（`dma_rpi_pico.c`）の
  チャネルを 2 つ使います。コマンド語とバッファー（合わせて 10,752 バイト）、セマフォ、イベント、チャネルは I2C0 だけのためのドライバーの
  静的変数です。インスタンスごとのデータに持つのは、DMA の読み出し中かどうかと、コントローラーを隔離したかどうかだけです。
- DMA の停止では、CHAN_ABORT の全体が落ちるまで待ちます（コアは各チャネル自身のビット）。認めない速さでは最小のカウントを残します
  （コアは何も変えずに `-EINVAL`）。
- `i2c_dma` コマンドは、共有の `common/shell/i2c_dma_shell.c` をアプリに入れたもので、パッチが宣言する 3 つの関数を呼びます。
  `i2c` コマンドは、パッチ 0004 を当てたツリーの中のもとのファイル `drivers/i2c/i2c_shell.c` をビルドします。
- Zephyr を上げるときに 0003 を作り直したら、`zephyr/script/host_tests.sh` を実行し、`i2c_dw_sync` が示す上流の変更を共有のコアに入れるか、
  入れない理由を `i2c_dw.c` の冒頭の一覧に書いてから、`common/tests/i2c_dw_sync.py ... --update` で期待する差分を書き直します
  （下の「テスト」）。

### NuttX

NuttX の `rp2040_i2c.c`（`CONFIG_RP2040_I2C`）は使いません。`nuttx/drivers/i2c_dw/Make.defs` が `VPATH` でコアと `port/nuttx/` のソースを
参照し、`include/`・`internal/`・`port/`・`port/nuttx/` を include パスに足します（オブジェクトは `nuttx/drivers` にできる）。オプションは
`nuttx/drivers/i2c_dw/Kconfig` の `CONFIG_I2C_DW_I2C0`（既定で有効）、`CONFIG_I2C_DW_I2C1`（既定で無効）、`CONFIG_I2C_DW_RP2040_DMA`
（既定で有効）です。

| ファイル | 役割 |
|---|---|
| `port/nuttx/i2c_dw_os_impl.h` | 境界の NuttX の実装: セマフォ（`sem_t`。`nxsem_post()`、`nxsem_tickwait_uninterruptible()`、`nxsem_wait_uninterruptible()`、`nxsem_reset()`）、時刻（システムの tick）、`nxsched_msleep()`、`up_irq_save()`、メッセージ（`struct i2c_msg_s` と、`i2c_dw_msg_flags()` での Zephyr のフラグへの変換）、`I2C_DW_CLOCK_SPEED`（`BOARD_PERI_FREQ`） |
| `port/nuttx/i2c_dw_instance.h` | I2C0 と I2C1 の NuttX の部分: `struct i2c_master_s` の操作表（`I2C_TRANSFER()` は要求を確かめて周波数から速さを決め、コアの転送を呼ぶ。`I2C_RESET()`）、`irq_attach()` で登録する割り込みハンドラー、リセットとピン（`i2c_dw_pinctrl()`）は両方で共有し、`i2c_dw_i2c0_initialize()` と `i2c_dw_i2c1_initialize()` |
| `port/nuttx/i2c_dw_dma_os_rp2040.c` | DMA の境界の NuttX の実装。NuttX 自身の DMA ドライバー `arch/arm/src/rp2040/rp2040_dmac.c`（`CONFIG_RP2040_DMAC`）の上に作る |
| `port/nuttx/i2c_dw_controllers_rp2040.h` | `i2c_dw_i2c0_initialize()` と、`CONFIG_I2C_DW_I2C1` のとき `i2c_dw_i2c1_initialize()`（起動時に `rp2040_bringup()` が I2C0、I2C1 の順に呼ぶ。`/dev/i2c*` には登録しない）。ボードのコードは include パスの `port/nuttx` で見つける |

境界の NuttX の実装:

| 境界 | NuttX 版 |
|---|---|
| `i2c_dw_os_sem_t` | `sem_t` |
| `i2c_dw_os_sem_give()` | `nxsem_post()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `nxsem_tickwait_uninterruptible(sem, MSEC2TICK(ms))` |
| `i2c_dw_os_timepoint_t`、`i2c_dw_os_timepoint_calc(ms)`、`i2c_dw_os_sem_take_until(sem, end)` | システムの tick（`clock_systime_ticks()`）と、残りを 0 で止めた `nxsem_tickwait_uninterruptible()` |
| `i2c_dw_os_sem_reset()` | `nxsem_reset(sem, 0)` |
| `i2c_dw_os_bus_lock()`・`_unlock()` | `nxsem_wait_uninterruptible()`・`nxsem_post()` |
| `i2c_dw_os_msleep(ms)` | `nxsched_msleep()` |
| `i2c_dw_os_uptime_ms()` | `TICK2MSEC(clock_systime_ticks())`（マクロ） |
| `i2c_dw_os_irq_lock()`・`_unlock()` | `up_irq_save()`・`up_irq_restore()` |
| `i2c_dw_os_msg_t`、`I2C_DW_OS_MSG_BUF()`・`_LEN()`・`_FLAGS()` | `struct i2c_msg_s`、フラグは前後のメッセージから `i2c_dw_msg_flags()` で作る |
| `I2C_DW_OS_DEV_HEAD` | `struct i2c_master_s dev;` |
| `I2C_DW_CLOCK_SPEED` | `BOARD_PERI_FREQ / 1000000` |
| `i2c_dw_os_prepare(dw)` | RESETS と `rp2040_gpio_*()`（`i2c_dw_pinctrl()`。Zephyr の `pinctrl_rpi_pico.c` の手順） |
| `i2c_dw_os_irq_enable(dw)` | `irq_attach()`・`up_enable_irq()` |
| `app_log.h` | `common/compat/include/app_log.h` と NuttX の部分 `common/compat/port/nuttx/app_log_os.h`（RAM ログ） |
| `CONFIG_I2C_DW_I2C0`・`CONFIG_I2C_DW_I2C1` | `nuttx/drivers/i2c_dw/Kconfig`（既定で `I2C_DW_I2C0` は有効、`I2C_DW_I2C1` は無効） |
| `CONFIG_I2C_DW_RP2040_DMA` | Kconfig |

DMA の境界の NuttX の実装（`port/nuttx/i2c_dw_dma_os_rp2040.c` の普通の関数）:

| 境界 | NuttX 版 |
|---|---|
| `i2c_dw_dma_os_initialize()` | I2C0 のコントローラーが起動したあとに呼ぶ。`rp2040_dmachannel_trywait()` を 2 回（空いている最も小さい番号、待たない）、その割り込みを切る。2 つのチャネルは読み出し用に持ち続ける。2 つとも取れなければ、取れた方を返して `-EBUSY` を返す |
| `i2c_dw_dma_os_is_ready()` | 2 つのチャネルが取れた |
| `i2c_dw_dma_os_request_channel()`・`_release_channel()` | 2 つのうち、実行中の読み出しが取っていないもの。返すと次の読み出しが使える |
| `i2c_dw_dma_os_config_tx()`・`_config_rx()` | `rp2040_txdmasetup()`（`RP2040_DMA_DREQ_I2C0_TX`、`RP2040_DMA_SIZE_HALFWORD`、読み出しアドレスを進める）、`rp2040_rxdmasetup()`（`RP2040_DMA_DREQ_I2C0_RX`、`RP2040_DMA_SIZE_BYTE`、書き込みアドレスを進める） |
| `i2c_dw_dma_os_start()` | チャネルのビットを INTS0 と INTE0 に書き、コールバックを付けて `rp2040_dmastart()` |
| コールバック | `rp2040_dmac.c` の DMA_IRQ_0 のハンドラーから（その状態 `EIO` を `-EIO` にする。エラーならチャネルをすぐ abort する。下を参照） |
| `i2c_dw_dma_os_stop()` | チャネルの INTE0 のビットを落とし、CHAN_ABORT にビットを書く（期限なしに待つ `rp2040_dmastop()` は使わない） |
| `i2c_dw_dma_os_is_busy()` | チャネルの CTRL_TRIG の BUSY（`rp2040_dma_register()`）、CHAN_ABORT |
| `i2c_dw_dma_os_acknowledge()` | チャネルのビットを INTS0 に書く |

INTE0（セットとクリアのエイリアス経由）、INTS0、CHAN_ABORT への書き込みは、`rp2040_dmac.c` と同じくそのチャネルのビットだけで、
ほかのチャネルには触れません。

**デバイスと転送**

- OS の I2C0 のデバイス（`i2c_dw_instance.h`）は、`i2c_0_runtime` の `base`（0x40044000）を設定し、セマフォを初期化し
  （`device_sync_sem` と `rp2040_dma_sem` は 0、`bus_sem` は 1）、起動時に `i2c_dw_turn_on()` を 1 回呼び、成功したら
  `i2c_dw_dma_os_initialize()` を呼んで `ready` を立てます（起動しなかったコントローラーは DMA のチャネルを取りません）。I2C1 のデバイスは
  `i2c_1_runtime`（0x40048000）に同じことを、DMA の部分とそのセマフォを除いて行います。
- 転送の入口（両方で 1 つ）は、Zephyr の API で表せない要求（メッセージの配列がない、256 個以上、メッセージごとに違うアドレスや速さ、
  10 ビットアドレス、バッファーがない、準備ができていない）を断り、メッセージが 0 個なら 0 を返し、ほかは
  `i2c_dw_transfer_msgs(dw, msgs, count, address, speed)` の結果を返します（`speed` は Zephyr の速さの区分）。
- I2C0 と DMA の割り込みは同じ優先度で（`CONFIG_ARCH_IRQPRIO` は無効で、すべての割り込みが既定の優先度）、ハンドラーと転送は 1 つの
  コアで動きます。
- NuttX のメッセージは、最後のメッセージと、`I2C_M_NOSTOP` がなく次が `I2C_M_NOSTART` でないメッセージの後に STOP、`I2C_M_NOSTOP` の後の
  `I2C_M_NOSTART` でないメッセージの先頭に RESTART を付けて、Zephyr のメッセージと同じに扱います。
- 速さはメッセージの周波数（100 kHz、400 kHz、1 MHz）で決め、変わったときに Zephyr の `i2c_configure()` と同じ計算をします。認めない速さでは
  何も変えずに `-EINVAL` を返します。
- バスのリカバリーは登録されないので、転送の前のリカバリーは何もせず、`I2C_RESET()` は `-ENOTSUP` を返します。
- I2C1 のピンはボードの `include/board.h` の `BOARD_I2C1_SDA_GPIO`・`BOARD_I2C1_SCL_GPIO` です。互換層のデバイスが無いので、`i2c` コマンドからは
  使えません。defconfig を変えずに作るには、一度ビルドしたあと `printf 'CONFIG_I2C_DW_I2C1=y\n' >> nuttx/nuttx/.config`、
  `make -C nuttx/nuttx olddefconfig`、`nuttx/script/build.sh`（`--clean` を付けない。付けると defconfig に戻る）。
- `i2c` コマンドが使う互換層のバスのデバイス `i2c0` は、NuttX の I2C マスターの上の Zephyr の I2C の呼び出し
  （`common/compat/port/nuttx/zephyr_i2c.c`）で、ボードの `src/rp2040_lsm6dso.c` が I2C0 につなぎます。

**NuttX の DMA ドライバーの上の実装**

- DMA は NuttX 自身のドライバー `rp2040_dmac.c`（`CONFIG_RP2040_DMAC`、有効。`arm_dma_initialize()` は `up_initialize()` の中、ボードの起動処理より
  前に動く）です。割り込みの線は DMA_IRQ_0 の 1 本ですべてのチャネルを扱います。ローカルのパッチ `nuttx/0011-rp2040-dmachannel-trywait` が、
  どこでも待たない `rp2040_dmachannel_trywait()` を足します。空きが無いとき、またはほかのタスクが割り当てのロックを持っているときは、
  取った数を返してすぐ NULL を返すので、チャネルは失われません（`rp2040_dmachannel()` は 12 チャネルがすべて使われていると待つ）。
- `i2c_dw_dma_os_rp2040.c` は起動時に `rp2040_dmachannel_trywait()` で 2 つのチャネルを 1 度だけ取り、読み出し用に持ち続けます。読み出しが
  チャネルを待つことはなく、1 度に 1 つの読み出しには必ずチャネルがあるので、`dma_fallbacks` は設定を断られたとき（チャネルがまだ動いている）
  しか増えません。起動時に空きが 2 つ無ければ、取れた方を返し、I2C0 は DMA なしで起動して、読み出しは割り込みで行います（`dmesg` に
  `No DMA for the reads: -16`）。
- `rp2040_dmastop()` は CHAN_ABORT を期限なしに待つので、停止は `i2c_dw_dma_os_rp2040.c` が自分で CHAN_ABORT を書き、コアが上限付き（100 ms）で
  待ちます。
- NuttX のハンドラーは失敗したチャネルのエラーのフラグをトリガーのレジスターである CTRL_TRIG に書き戻すので、チャネルが止まったところから
  再び動き出すことがあり、つなぎのコールバックがそのチャネルをすぐ abort します。
- 同じ割り込みで先に処理したチャネルの状態があとのチャネルにも残るので、ほかの利用者のバスエラーが同じ割り込みで処理されると、この読み出しも
  失敗します（`-EIO`、`dma_errors` が 1）。

**DMA を使うほかのコード**

- このビルドのほかのコードも、NuttX の DMA を通常の方法で使えます（`rp2040_dmachannel()` など。残りの 10 チャネル）。
- してはいけないこと: 読み出しの 2 つのチャネルに触れる（`rp2040_dmac.c` の表で確保済みなので、その割り当てが渡すことはない）。DMA ブロックを
  リセットする。チャネルが共有するレジスター（INTE0、INTS0、CHAN_ABORT）を丸ごと書く（`rp2040_dmac.c` と同じく自分のビットだけにする）。
  `irq_attach()` で DMA_IRQ_0 を `rp2040_dmac.c` から取り上げる。自分のハンドラーなしに自分のチャネルの割り込みを DMA_IRQ_1 に回す。
- SPI のドライバー（`CONFIG_RP2040_SPI_DMA`。`imu` の構成で有効）もその 1 つで、NINA の最初の要求のときに SPI1 のためにさらに 2 つのチャネルを取り、
  持ち続けます（[NINA のドライバー](../nina/README_ja.md)）。

### Pico SDK（FreeRTOS・ThreadX で共有）

FreeRTOS 版と ThreadX 版は、コアのつなぎのうち Pico SDK の部分を共有し、OS の部分（`port/freertos/`・`port/threadx/` の
`i2c_dw_os_impl.h`: セマフォ、時計、スリープ、割り込み禁止）だけをそれぞれが持ちます。つなぎのカーネルの機能は互換層ではなく
その OS 自身のもので、Zephyr のものとして残るのは使う側の API（互換層のデバイスと `<zephyr/drivers/i2c.h>`）だけです。

| ファイル | 役割 |
|---|---|
| `port/pico_sdk/i2c_dw.cmake` | ドライバー（I2C0 と、オプションで I2C1）と `i2c_dma` コマンドをビルドに入れる。OS の部分は `port/<OS>/`（`PICO_SDK_OS`） |
| `port/pico_sdk/i2c_dw_instance.h` | 互換層のデバイス `i2c0`・`i2c1` と Zephyr の I2C の API、その割り込み・リセット・ピン。コアの `i2c_dw.c` が末尾で読む |
| `port/pico_sdk/i2c_dw_controllers_rp2040.h` | 起動の関数 `i2c_dw_i2c0_initialize()`・`i2c_dw_i2c1_initialize()` |
| `port/pico_sdk/i2c_dw_dma_os_rp2040.c` | Pico SDK の DMA での I2C0 の DMA 読み出し（DMA の境界の実装） |

- **インスタンス**は NuttX 版の Kconfig と同じ形で選びます。`i2c_dw.cmake` が `CONFIG_I2C_DW_I2C0` を常に定義し（LSM6DSOX がつながっているため。
  無いと `common/drivers/lsm6dso/port/pico_sdk/lsm6dso.cmake` が構成を止めます）、CMake のオプション `I2C_DW_I2C1`（既定で無効）のとき
  `CONFIG_I2C_DW_I2C1` を定義します。インスタンスごとのデバイス、割り込みハンドラー、速さの区分、起動はそれぞれのシンボルの下にあり、
  API、ピンの設定、リセット、割り込みの接続は共有です。
- **I2C1** は互換層のデバイス `i2c1`（`DEVICE_DT_GET(DT_NODELABEL(i2c1))`、名前は `i2c@40048000`）で、`i2c0` と同じ Zephyr の I2C の API を持ち、
  ピンは `i2c_dw_instance.h` の `I2C1_SDA_GPIO`・`I2C1_SCL_GPIO` です。`usb_console_init()`（`common/imu/port/pico_sdk/usb_console.c`）で I2C0 のあとに
  起動し、失敗してもログに残して `i2c1` が準備できないだけです。互換層のデバイスの一覧（`device_get_binding()`）には入らないので、`i2c` コマンドからは
  使えません。
- **断る要求**: Zephyr 版のドライバーと同じく `get_config` は無く、`i2c_get_config()` は `-ENOSYS` を返すので、`i2c speed` は
  `I2C_MODE_CONTROLLER` と新しい速さの区分で `i2c_configure()` を呼びます（以後の転送はその速さ）。
- **I2C0 の DMA 読み出し**は、コアの 2 つ目の境界 `port/i2c_dw_dma_os.h` を通して Pico SDK 自身の DMA（`hardware_dma`、`hardware_irq`）を使います。
  読み出しごとに `dma_claim_unused_channel(false)` でチャネルを 2 つ取り（空きが無ければ割り込みで読み、フォールバックを数えます）、SDK の
  チャネル設定で設定し（TX: 16 ビットのコマンド語を IC_DATA_CMD へ、DREQ 32。RX: そこからバイトを、DREQ 33）、`dma_channel_start()` で始め、
  `dma_channel_unclaim()` で返します。起動時に何もリセットしません。
- **その割り込みと停止**: 割り込みは DMA_IRQ_1 で、`irq_add_shared_handler()`（既定の順位）で加えたハンドラーが、読み出しの持つチャネルだけを
  見ます。停止はチャネルのビットを CHAN_ABORT に書き、コアが上限付きで待ちます（100 ms、過ぎればコントローラーを隔離）。SDK の
  `dma_channel_abort()` は上限なしで待つので使いません。
- **DMA を使うほかのコード**: このビルドのほかのコードも、DMA を通常の方法で使えます。チャネルは SDK の割り当てで取り、割り込みの線は
  `irq_add_shared_handler()` で共有します。してはいけないこと: 読み出しが持っている 2 つのチャネル（読み出しの間、SDK の記録で取得済み）に触れる。
  DMA_IRQ_1 に排他的なハンドラーを入れる（共有のハンドラーがあると `irq_set_exclusive_handler()` が panic します）、共有の線のハンドラーで
  ほかのチャネルのフラグを消す。DMA のブロックをリセットする、INTE0/INTE1・INTS0/INTS1・CHAN_ABORT を丸ごと書く（自分のチャネルのビットだけにする）。
  読み出しに DMA を使わせたいときに 12 チャネルすべてを持ち続ける（そのあいだ読み出しは割り込みに戻ります）。
- **NINA の SPI の DMA**（`common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c`）も、SDK の割り当てで 2 つのチャネルを取って持ち続け、
  DMA_IRQ_1 を `irq_add_shared_handler()` で共有するほかの利用者です（[NINA のドライバー](../nina/README_ja.md#pico-sdkfreertosthreadx-で共有)）。
- **割り込み**: I2C0 の読み出しの DMA_IRQ_1 のハンドラーは、Pico SDK の共有の割り込みハンドラーの枠を 1 つ使います。枠の数と
  割り込みの優先度は [FreeRTOS 版のポートの「割り込み」](../../../design/freertos-port-ja.md#割り込み)にあります。

### FreeRTOS

`port/freertos/i2c_dw_os_impl.h` は、セマフォ、時計、スリープ、割り込み禁止を FreeRTOS と Pico SDK で直接書きます。

- セマフォ（`device_sync_sem`、`bus_sem`、DMA 読み出しの `rp2040_dma_sem`）はどれも、ドライバー自身のメモリーに置く上限付きの数と待っているタスクの
  列で（FreeRTOS のキューもヒープも使わない）、インスタンスの起動（`i2c_dw_i2c0_initialize()`）が用意します。`bus_sem` は上限 1（Zephyr の `k_sem` と
  同じく優先度継承は無く、待つタスクは優先度順）です。
- give は Zephyr と同じく数を先頭の待ち手に渡し、ドライバー専用の番号 3 のタスク通知で起こします（割り込みハンドラーからは
  `vTaskNotifyGiveIndexedFromISR()` とハンドラーの終わりの切り替え、タスクからは `xTaskNotifyGiveIndexed()`。`configTASK_NOTIFICATION_ARRAY_ENTRIES` は 4）。
  待ち手は起こされたことではなく自分の完了の印で結果を決めるので、数を見てから待つまでの間の give は失われず、前から残っていた通知は残りの時間を
  もう一度待たせるだけです。
- 時間付きの待ちは Zephyr の `k_sem_take(K_MSEC())` と同じく、時間を tick に切り上げて 1 tick 足した長さだけ待ちます（転送の 1000 ms は 1 ms の
  tick で 1001、abort の 1 ms は 2）。スリープも同じ長さを `vTaskDelay()` で待ち、時計は tick ではなく TIMER（`time_us_64()`）の ms です。

### ThreadX

`port/threadx/i2c_dw_os_impl.h` は、互換層を通さず ThreadX と Pico SDK で直接書いてあります。待ちのオブジェクトはドライバーが持つ `TX_SEMAPHORE` です。

- `TX_SEMAPHORE` を使い、スレッド自身の起こしや停止と再開を使わない理由は [ThreadX 版のポート](../../../design/threadx-port-ja.md#互換層のカーネルの部分とドライバーの待ち)にあります。
- `device_sync_sem` と `rp2040_dma_sem` は 0 から上限 `UINT_MAX`、バスのロック `bus_sem` は 1 から上限 1（優先度継承なし。解放のときに待ち手が
  2 つ以上なら `tx_semaphore_prioritize()` で最も急ぐものを先頭に）。give は `tx_semaphore_ceiling_put()`、`k_sem_reset()` は割り込みを止めて数を書きます。
- `i2c_dw_os_sem_init()` はカーネルの前に 2 回呼ばれることがあり（コントローラーが最初に始まらなかったとき）、2 回目は作ったオブジェクトを残して
  数と上限を書きます。カーネルの前と ISR の中では ThreadX が待ちを断る（`TX_WAIT_ERROR`）ので、そのときは待たずにもう一度取ります。
- 時間つきの待ちとスリープは、時間を tick に切り上げて 1 tick 足します（Zephyr の規則。ThreadX の n の待ちは今の tick + n で終わる）。時計は tick ではなく
  TIMER の ms です。

## `i2c` コマンドと `i2c_dma` コマンド

**`i2c` コマンド**（`common/shell/i2c_shell.c`）は、Zephyr の `drivers/i2c/i2c_shell.c` にローカルのパッチ
`zephyr/0004-i2c-shell-scan-read-probe.patch` を当てたものとバイト単位で同じ写しです（`zephyr_i2c_shell_patched` が照合する）。Zephyr 版はツリーの中の
もとのファイルを、NuttX 版は `nuttx/apps/i2c`、FreeRTOS 版・ThreadX 版は `common/shell/port/pico_sdk/i2c_shell.cmake` でこのファイルをビルドします。
デバイスは I2C0 の `i2c@40044000` です。

- `i2c scan` は 0 バイトの書き込みで確かめます。最初の書き込みが `-ENOTSUP` を返すと、そのアドレスと残りを 1 バイトの読み出しで確かめて、表のあとに
  `Probed with one-byte reads (no zero-length writes)` を表示します（パッチ 0004。その書き込みを断るどのコントローラーでも同じ）。ACK を返した
  アドレスを表示し、エラー（NACK）は `--` です。1 つのアドレスは `i2c direct_read i2c@40044000 6a 1` で確かめます。
- `i2c recover` は `-ENOTSUP` です。
- `i2c speed i2c@40044000 <1〜3>` はバスの速さを変えるので、センサーの転送の速さも変わります。
- `i2c target` のコマンドはビルドしません（`CONFIG_I2C_TARGET` は Zephyr 版でも無効）。

**`i2c_dma` コマンド**（`common/shell/i2c_dma_shell.c`）は 4 つの OS のファームウェアで共有します（NuttX 版のビルドは `nuttx/apps/i2c_dma`）。パッチ後の
ドライバーとコアが同じ 3 つの関数（方式の取得・設定、カウンターの取得）を定義し、コマンドはその宣言を `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>` で
読みます。Zephyr 版ではパッチが Zephyr のツリーに足すヘッダー、NuttX 版・FreeRTOS 版・ThreadX 版では `common/compat` の同名のヘッダーで、後者は
`include/i2c_dw_rp2040_dma.h` を読みます。

```text
i2c_dma mode          現在の方式を表示
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（累積）
```

## テスト

共有ソースのテスト（`common/tests/run.sh`。4 つの OS の `host_tests.sh` が最初に実行する）:

| テスト | 対象 |
|---|---|
| `i2c_dw` | コアの定義（`internal/i2c_dw.h`）: SCL のカウント、速さ、フラグの変換、`IC_DATA_CMD` の語（`common/tests/i2c_dw.c`） |
| `i2c_dw_public_headers` | コアを使う側のヘッダー（`include/`）が、そのディレクトリーだけでコンパイルできること（コアの `internal/`・`port/` を読まない） |
| `i2c_dw_sync` | 共有のコアと、パッチ後のドライバー（Zephyr のツリーの HEAD のファイルにパッチ 0003 を当てて作る）の一致（下）。Zephyr のツリーがあるときだけ |
| `zephyr_i2c_shell_patched` | `common/shell/i2c_shell.c` が、Zephyr の HEAD の `drivers/i2c/i2c_shell.c` にパッチ 0004 を当てたものとバイト単位で同じこと。Zephyr のツリーがあるときだけ |

**上流とのずれの検査（`i2c_dw_sync`）**: パッチ後のドライバーとコアは次のようにしてそろえて保ちます。

- どちらかを直したら、もう一方にも同じ変更を入れる（入れない場合は理由を `i2c_dw.c` の冒頭の一覧に書く）。
- `common/tests/i2c_dw_sync.py` が、ツリーの HEAD のファイルにパッチを当てたものとコアを比べます（ツリーの作業中のファイルは見ない）。次のどれかが
  食い違うと失敗します。
  - 両方のヘッダーにある定義（状態、割り込み、レジスター、IC_DATA_CMD のビット、FIFO の深さ 16・バッチ 4、DMA の上限 3584）、125 MHz での SCL の
    カウント、DMA の下限 32・しきい値 8/0・割り込みのマスク・IC_CON の bit 9・停止の待ち 100 ms・タイムアウト 1000 ms、スパイク長。ビルドがあれば、
    その `.config` と `zephyr.dts` の値も見る（起動時の I2C0 の速さを、Zephyr のデバイスツリー、`port/pico_sdk/i2c_dw_instance.h` の `I2C0_SPEED`、
    NuttX のボードの `BOARD_I2C0_FREQUENCY` で比べることも含む）。
  - 対になる関数: Zephyr の DMA API の呼び出しを含めて名前をコアのもの（DMA は境界 `i2c_dw_dma_os.h`）にそろえて文ごとに比べ、差分が保存した
    `common/tests/i2c_dw_sync.expected` と同じでなければ失敗する。コメントと改行の違いは見ない。
  - 読み出しのコマンド語を作る関数 `i2c_dw_rp2040_read_commands()`（パッチとコアのそれぞれの `i2c_dw_rp2040_dma_priv.h` の静的関数）が、文ごとに
    同じでコマンドのビットも同じであること。
  - パッチが足す `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` とコアの `include/i2c_dw_rp2040_dma.h` が、コメントとインクルードガードを除いて
    同じであること。
  - パッチ後のドライバーが、Zephyr のツリーの外のヘッダーを読まないこと。
- 片方だけを変えれば失敗するので、意図した変更なら差分を確かめてから `--update` で書き直します。
- 比べていない関数（Zephyr の初期化、PM、target モード、OS の部分）のずれは検出しません。OS ごとの `i2c_dw_dma_os.h` の実装は、OS の DMA の偽物の
  上の NuttX 版・FreeRTOS 版のホストテストで確かめます。

各 OS のテスト:

| テスト | 対象 |
|---|---|
| NuttX: `i2c_dw_instance` | NuttX へのつなぎ（`i2c_dw_os_impl.h`、`i2c_dw_instance.h`）を共有のコアの下で、NuttX のヘッダーの偽物（`nuttx/tests/i2c_dw_instance_fakes`）、ボードの `include/board.h`、コントローラーのレジスターと DMA の境界の偽物の上で。I2C0 だけ、I2C1 付き（`CONFIG_I2C_DW_I2C1`）、DMA の境界の起動に失敗したとき（I2C0 は割り込みで起動する）の 3 回（`nuttx/tests/i2c_dw_instance.c`） |
| NuttX: `i2c_dw_dma_os` | DMA の部分（`port/nuttx/i2c_dw_dma_os_rp2040.c`）を、NuttX の `rp2040_dmac.c` と DMA のレジスターの偽物の上で: チャネル、その設定、開始、割り込み、停止、返却（`nuttx/tests/i2c_dw_dma_os.c`） |
| NuttX: `i2c_shell` | `i2c` コマンドを `zshell.c` と互換層の I2C のバスのデバイス（`common/compat/port/nuttx/zephyr_i2c.c`）の上で、NuttX の I2C マスターの偽物と |
| NuttX: `i2c_dma_app` | `i2c_dma` コマンドを `zshell.c` の上で、ドライバー側の偽物と |
| FreeRTOS: `i2c_dw` | I2C0 の境界とデバイス、Zephyr の I2C の API、その上の `i2c` コマンド（引数のデバイスの候補、`speed` とそのあとの転送の速さ、`read_byte`・`write_byte`・`read` の 16 進の表示、無いデバイス、`scan` が断ること、`recover`）（2 回作る: i2c0 だけと、`CONFIG_I2C_DW_I2C1` 付き。後者は i2c1 の起動、ピン、転送、応答の無いアドレス、DMA を使わない長い読み出し、影響を受けない i2c0、2 つのタスクからの両バスの転送を加える）。つなぎのセマフォ、待ちの tick、スリープ、時計。Pico SDK の DMA の偽物の上の DMA の部分（`i2c_dw_dma_os_rp2040.c`）: チャネルの取得、設定、開始、共有のハンドラー（終了、バスエラー、ほかの利用者のチャネル）、停止、返却。その上のコアの DMA 読み出し: 読み出し、32 バイトの下限、フォールバック、TX と RX のバスエラー、タイムアウト、DMA 読み出しの無効、終わらない停止（隔離）、カウンター。コアの DMA のコマンド語（`freertos/tests/i2c_dw/`） |
| ThreadX: `i2c_dw` | FreeRTOS 版のテストの本体（`freertos/tests/i2c_dw/main.c`）を ThreadX の部分（`threadx/tests/i2c_dw/i2c_dw_test_os.c`）と一緒に。I2C0 と I2C1 の境界とデバイス、DMA 読み出し、Zephyr の I2C の API、`i2c` コマンド。加えて、互換層の起こしや別のセマフォで待ちが終わらないこと、待つ直前の give が失われないこと、tick の一周をまたぐ期限、2 回目の `i2c_dw_os_sem_init()`、カーネルの前と ISR の中の呼び出し。割り込みは、ThreadX のコンテキストの保存と復元に包んだ Linux のスレッド 1 本（`threadx/tests/i2c_dw/tx_test_isr.c`） |
