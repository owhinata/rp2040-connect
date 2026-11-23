# ThreadX 版の計画（2026-11-24 にユーザーが承認）

同じファームウェアを 4 つ目の OS、Pico SDK + ThreadX の上でビルドする（以下 ThreadX 版）。進め方は FreeRTOS 版と同じで、
subagent（`model: opus`）が実装し、メインがレビュー・ホストテスト・実機確認をして、タスクごとに `main` へ commit する。
経緯と実機の記録は `threadx/PORTING_STATUS.md` に書く。

## 目的

- いちばんの目的は「対応 OS を簡単に増やすこと」。ThreadX 版は、`design/os-boundary(-ja).md` の「新しい OS で書くもの」が
  実際に足りているかを確かめる最初の機会でもある。足りなかったもの・二重になったものは、境界の側を直す。
- 完了条件は FreeRTOS 版と同じ: 全機能同等。`imu-host` と ImuStudio が、USB の検出を除いて無変更で使える。

## 決定事項

| 項目 | 決定 | 決めた人 |
|---|---|---|
| 全体の形 | Pico SDK 2.3.1 + ThreadX のカーネルだけ（Eclipse ThreadX v6.5.2.202603b_rel、MIT）。起動・クロック・USB・フラッシュ・DMA は Pico SDK。USBX・NetX Duo・FileX は使わない | ユーザー（2026-11-24） |
| Pico SDK の部分 | ThreadX 版を書く前に、FreeRTOS 版の中の「Pico SDK だけで書かれたコード」を共有の場所へ切り出す。写して置き換える形は取らない | ユーザー（2026-11-24） |
| ネットワーク | 互換層のソケット層（`common/compat/net_offload`）。FreeRTOS 版と同じ。NetX Duo の下に NINA をつなぐ形は、この計画に入れない | ユーザー（2026-11-24） |
| カーネルの部分 | 基本の操作 26 個を書いて、汎用の実装（`common/compat/kernel_generic`）を使う | メインの案（FreeRTOS 版と同じ形） |
| シェル | 共有のシェルのエンジン（`common/compat/shell_engine`） | 同上 |
| 同じ優先度の選び方 | ThreadX の標準（時分割なし） | 以前の決定「その OS の標準」に合わせる |
| 取得するツリー | `threadx/` の下に ThreadX・Pico SDK・ツールチェーンを `setup.sh` が取得する（NuttX 版・FreeRTOS 版と同じく、OS のディレクトリーごとに持つ） | メインの案 |
| スタイル | Zephyr。新しいファイルの SPDX は MIT。`threadx/script/format.sh` | 既存の決まり |

## 分かっていること（ThreadX のソースを読んで確かめた。ビルドはまだ）

- Cortex-M0 の GCC 用のポート（`ports/cortex_m0/gnu`）と Linux 用のポート（`ports/linux/gnu`。ホストテスト用）がある。
- スレッド・セマフォ・ミューテックスの管理領域は呼ぶ側が渡す。カーネルはヒープを使わない。
- 優先度は 0 が最優先の 32 段階。ミューテックスは再帰と優先度継承（`TX_INHERIT`）を持つ。
- **ロックの注意**: Cortex-M のポートは、サービスの中でスレッドを切り替えるとき、PendSV を立てたあと割り込みを一瞬許可する
  （`_tx_thread_system_return_inline()`）。互換層の `z_os_task_lock()` は「解除まで切り替わらない。中での起こしはいちばん外の解除で
  切り替わる」ことを求めるので、PRIMASK だけでは足りない。ThreadX 内部の `_tx_thread_preempt_disable` を併用する案を T0 で確かめる。
- アイドルスレッドは無い（スケジューラーが待つ）。ポートに省電力のフック（`tx_low_power_enter`・`_exit`）がある。
- スレッドごとの実行時間は、付属の `utility/execution_profile_kit` で取れる。

## FreeRTOS 版の自分のコードの内訳（2026-11-24 に数えた）

`freertos/app`・`freertos/drivers`・各ドライバーの `port/freertos/` の `.c`・`.h` は約 6,000 行（cmake などを入れて約 7,300 行）。

| 種類 | ファイル | 行 |
|---|---|---|
| FreeRTOS の API が中心 | `zephyr_kernel_os_impl.h`、`i2c_dw_os_impl.h`、`nina_os.c`、`nina_bus_dma_rp2040.c`（待ちの部分）、`os_clock.h`、`tick.c`、`FreeRTOSConfig.h` | 約 1,800 |
| FreeRTOS の呼び出しが数か所 | `main.c`、`commands.c`（`ps`・`free`・`version`）、`fault.c`、`idle.c`、`irq_stack.c`、`gpio_os_rp2040.c`、`rp2040_flash_kv_os.c`、`kernel_shell_os_impl.h`、`zephyr_os_port.h`、`priorities_os.h`、`i2c_dw_instance.h`、`tick_catchup.h` | 約 1,900 |
| FreeRTOS の呼び出しなし（Pico SDK と互換層だけ） | `nina_bus_rp2040.c`、`nina_passthrough_*.c`、`i2c_dw_dma_os_rp2040.c`、`stdio_wrap.c`、`shell_serial_os_impl.h`、`shell_stdio.c`、`zephyr_hwinfo.c`、`zephyr_sys_clock.c`、互換層の `i2c.h`、`lsm6dso_*`、`sensor_shell_os.*`、ログの OS 部分、プレリュード ほか | 約 2,300 |

2 行目と 3 行目が切り出しの対象。2 行目は、FreeRTOS の呼び出し（数行）を小さな境界に出してから共有にする。

## タスク

各タスクの完了条件に「ホストテストが通る」「変わるはずのないイメージが基準と一致する」を含む。イメージが変わったら実機で送信を確かめる。

### T0: ロックと基本の操作の確認（ホストだけ）

ThreadX の Linux ポートの上に基本の操作 26 個を書き、互換層のカーネルのホストテスト（`common/tests/zephyr_kernel`、22 項目）を通す。
`threadx/script/setup.sh`（ThreadX の取得）と `threadx/script/host_tests.sh` の最初の形もここで作る。

- 確かめること: ロックの条件（上の注意）、起こしの印（スレッドごとのセマフォ）、全部 0 のミューテックス、スレッドの終わり、
  スタックのあとの予約領域（管理領域・名前・起こしの印）の大きさ。
- 通らない項目があれば、ThreadX の側の制約か、境界の定義が FreeRTOS に寄っているのかを切り分けて報告する。

### T1: Pico SDK の部分の切り出し（FreeRTOS 版は動作を変えない）

FreeRTOS 版の「Pico SDK だけ」のコードを共有の場所へ移し、ThreadX 版と FreeRTOS 版の両方がそこからビルドする形にする。

- 置き場所の案: ドライバーのものは `common/drivers/<ドライバー>/port/pico_sdk/`、ドライバーでないものは `common/chip/rp2040/pico_sdk/`、
  OS に依存しないビルドの定義（共有のソースの一覧）は共有の cmake。**最初に一覧（どのファイルをどこへ、残る OS の呼び出しをどう境界にするか）を作り、
  メインがユーザーに確認してから移す。**
- FreeRTOS の呼び出しが数か所あるファイルは、その呼び出しを小さな境界（static inline のヘッダー）に出す。新しい境界は `design/os-boundary` に足す。
- 完了条件: FreeRTOS 版のイメージが移動の前とバイト単位で一致する（同じ `SOURCE_DATE_EPOCH`）。一致しない場合は差の理由を示し、実機で送信を確かめる。
  Zephyr 版・NuttX 版のイメージは基準と一致。

### T2: 起動とコンソール

`threadx/app/usb-console`: `tx_user.h`、起動（`main()` → `tx_kernel_enter()` → `tx_application_define()`）、SysTick 1 ms の tick と `zephyr_compat_tick()`、
割り込みベクターの名前合わせ、互換層の OS 部分、優先度、ログ、コンソールとシェルのエンジン、汎用のコマンド、`version`・`free`・`ps`、異常停止の記録、
アイドルと `cpu_active`、ビルド・書き込みのスクリプト（`script/flash_uf2.sh --target threadx`、USB の製品名）。

- 完了条件（実機）: USB のコンソールでシェルが動く。`version`・`ps`・`uptime`・`reboot`・`bootsel`・`devmem`・`fault`・`dmesg`。

### T3: IMU だけの構成（`design/os-boundary` の 7 節）

KV のフラッシュ、GPIO バンク 0、I2C0 と DMA（`port/threadx/` の待ち）、LSM6DSOX、`imu`・`sensor`・`i2c`・`i2c_dma`・`kv` のコマンド。

- 完了条件（実機）: `sensor get`、`sensor stream`（104〜6667 Hz）、`imu cal`、KV の読み書きと再起動、ほかの OS のファームウェアが書いた KV が読める。

### T4: 全機能

NINA（`port/threadx/nina_os.c` と DMA の待ち）、互換層のソケット層、Wi-Fi、MQTT の送信、時刻同期、Telnet、`zperf`、中継モード。

- 完了条件（実機）: 送信（欠番 0、時刻の逆行 0）、送信中の Telnet の接続と切断、切断と再接続、`wifi scan`、`zperf`、中継モードでの NINA のフラッシュの読み出しの一致。

### T5: 仕上げ

よく通るコードの RAM への配置と検査、RAM の残りとスタックの調整、`cpu_active` の計測（ほかの 3 つと同じ条件）、
文書（`threadx/README`・`threadx/drivers/README` の日英、ルートの README、設計書、`design/os-boundary` に ThreadX の列）、`CLAUDE.md`。

- `design/os-boundary` の「新しい OS で書くもの」を、実際に書いたものと照らして直す（この計画のいちばんの成果）。

## この計画に入れないもの

- NetX Duo、USBX、FileX。ThreadX の SMP とモジュール。
- ThreadX 付属の FreeRTOS 互換層で FreeRTOS 版を動かす形（境界の確認にならない）。
- Zephyr 版・NuttX 版の作りの変更（T1 の移動に伴うパスの変更を除く）。
