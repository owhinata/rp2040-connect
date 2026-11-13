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

Wi-Fi の管理とソケット（usrsock）はまだ移植していません。
