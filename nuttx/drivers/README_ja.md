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
| ボードの `src/nano_i2c.c` | I2C0 のコントローラー。Zephyr 版の `drivers/i2c_dw/i2c_dw.c` のコントローラーの部分（`i2c_dw_transfer()`、`i2c_dw_isr()`、`i2c_dw_data_ask()`・`_read()`・`_send()`、`i2c_dw_setup()`、`i2c_dw_configure()`、初期化）を、関数・分岐・順序・レジスター値・期限・エラー値を合わせて写したもの。NuttX の `struct i2c_master_s`（`I2C_TRANSFER()`、`I2C_RESET()`）として出す |
| ボードの `src/nano_i2c.h` | `nano_i2c0_initialize()`（起動時に `nano_bringup()` が呼び、`/dev/i2c0` に登録する） |
| ボードの `src/nano_i2c_dw.h` | OS に依存しない部分: Zephyr 版の `i2c_dw.h` の定義、SCL のカウント、周波数から速さの区分、NuttX のフラグ（`I2C_M_*`）から Zephyr のフラグ（`I2C_MSG_*`）への変換、`IC_DATA_CMD` に書く語。ホストテスト `nuttx/tests/nano_i2c_dw.c` |
| ボードの `src/nano_i2c_dw_registers.h` | Zephyr 版の `i2c_dw_registers.h` の写し（アクセス関数だけ NuttX 向けに定義） |

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
- RP2040 の I2C DMA での読み出しはまだありません。使い方と実測は `nuttx/README_ja.md` の「I2C0」の節にあります。

Wi-Fi の管理とソケット（usrsock）はまだ移植していません。
