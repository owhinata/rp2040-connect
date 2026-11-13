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

Wi-Fi の管理とソケット（usrsock）はまだ移植していません。
