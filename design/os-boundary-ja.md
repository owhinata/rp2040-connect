# 共有のコードと OS の境界

[English](os-boundary.md) | 日本語

このリポジトリーのファームウェアは、同じ機能を 3 つの OS の上でそれぞれビルドします: Zephyr（`zephyr/`）、NuttX
（`nuttx/`）、Pico SDK + FreeRTOS（`freertos/`）。以下、OS ごとにビルドしたものを「OS のファームウェア」、それぞれを
Zephyr 版・NuttX 版・FreeRTOS 版と呼びます。ソースの大半は 3 つの OS で共有し、`common/` にあります。

この文書は、共有のソースが動くために各 OS のファームウェアが用意するもの（4 つ目の OS を足すときに書くもの）の
索引です。今のコードのとおりに書きます。各行に、宣言しているヘッダーと、OS ごとの実装のファイルを挙げます。
「N」は NuttX 版、「F」は FreeRTOS 版です。F のファイルのうち `common/pico_sdk/`、各ドライバーの `port/pico_sdk/`、
`common/compat/os_generic/` にあるものは FreeRTOS のものではなく、Pico SDK と互換層の上で書いてあり、Pico SDK の上のほかの OS の
ファームウェアと共有します。

共有のソースの多くは Zephyr の API（`<zephyr/kernel.h>` など）で書いてあります。NuttX 版と FreeRTOS 版は、その API を
Zephyr と同じ名前で提供する層（以下「互換層」。OS に依存しない部分は `common/compat`、OS ごとの部分は各 OS の
ディレクトリー）の上でそれをビルドします。Zephyr 版は `common/` を Zephyr そのものに対してビルドし、互換層を
使いません。Zephyr 版は、自分のファイルを持つところにだけ出てきます。

`nuttx/boards/arduino-nano-rp2040-connect/` の下のパスは `N-board/` と略します。

## 新しい OS で書くもの

4 つ目の OS で書く順に並べます。手本は、OS のオブジェクトの上に直接書くなら N、基本の操作だけを持つ OS なら F です。

1. **互換層のカーネルの部分**（2 節）: 次のどちらか。
   - 直接に実装する: `zephyr_os.h`、`zephyr_os_spinlock.h`、`zephyr_os_atomic.h` と 2.2 の 18 関数（手本は `nuttx/drivers/compat/`）。
   - 基本の操作の上に置く（`common/compat/kernel_generic` を使う）: `zephyr_kernel_os_impl.h` の型と定数 10 と関数 26（2.5）、
     1 tick に 1 回 `zephyr_compat_tick()` を呼ぶ tick のフック（手本は `freertos/drivers/compat/zephyr_kernel_os_impl.h` と
     そのポートのヘッダー `rp2040/zephyr_os_port.h`。コンパイルが通る最小の形は `common/tests/kernel_generic_host/`）。
     `zephyr_os_atomic.h` は `kernel_generic` に空の既定のものがあり、ヘッダーが atomic の名前を定義する OS だけが自分のものを書きます。

   どちらでも、`priorities_os.h`（3 節）と OS ごとの `<zephyr/sys/clock.h>`・`<zephyr/sys/__assert.h>`・`<zephyr/drivers/i2c.h>`（4.1）を
   用意します。その仕組みを自分で持たない OS は `common/compat/os_generic/include` のものを使います。
   受け入れ試験は、OS の部分の上で動かす互換層のカーネルのホストテスト（`common/tests/zephyr_kernel`。3 節の 2 行）です。
2. **コンソールの経路とシェル**（3 節、4.4）: 自分のシェルを持たない OS は、共有のシェルのエンジン
   （`common/compat/shell_engine`）に `shell_serial_os_impl.h` と stdio の行き先を渡します（手本は `common/pico_sdk/drivers/shell/`、
   `common/pico_sdk/drivers/libc/`）。自分のシェルを持つ OS は `zshell_os_impl.h` と `zephyr_os_shell.h` を書きます（手本は
   `nuttx/apps/subcmd/`、`nuttx/drivers/compat/zephyr_os_shell.h`）。
3. **ログ**（3 節）: `app_log_os.h`、`log_ram_os_impl.h`（手本は `common/compat/os_generic/include/app_log_os.h`、
   `common/pico_sdk/drivers/include/log_ram_os_impl.h`）。
4. **KV のフラッシュ**（3 節）: RP2040 では `rp2040_flash_kv_os.h` の 4 関数とリンカースクリプトの `__kv_start`・`__kv_end`
   （手本は `common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c`）。
5. **ボードの部分**（3 節、6 節）: GPIO バンク 0（`gpio_os.h`）、CPU 負荷（`cpu_load_get()`）、hwinfo（4.2）、tick、割り込み、
   アイドルループ、致命的なエラー。
6. **起動**（4.6）: アプリの起動より前の段階（RAM ログ、互換層、I2C、LSM6DSOX、ネットワーク、KV のフラッシュの範囲、
   シェル）と、`app_main.h` の 3 つの関数の呼び出し。モジュールのオプション（4.5）を定義します。
7. **ドライバーごと**（5 節）: 表の N・F の列が、各 OS のファームウェアが自分で書くファイルです。ネットワークは、OS の
   ネットワーク層の上につなぐか（N）、互換層のネットワークの部分（`common/compat/net_offload`）を使うか（F）です。

最初の目標には、Wi-Fi なしでセンサーを動かす構成（7 節）が使えます。

**Pico SDK の上の OS。** Pico SDK の上で汎用のカーネルの部分を使う OS は、2〜7 と 1 の大半を F が共有しているもの（`common/pico_sdk/`、
各ドライバーの `port/pico_sdk/`、`common/compat/os_generic/`、`common/compat/kernel_generic/tick_catchup.h`）から取り、書くのは次だけです。

- カーネルの部分: `zephyr_kernel_os_impl.h`、そのポートのヘッダー `zephyr_os_port.h`（`z_os_in_isr()`、`z_os_yield_from_isr()`、
  `z_os_task_lock()`、`z_os_task_unlock()`。`z_os_busy_wait_us()`、`z_os_tick_now_us32()`、`z_os_tick_pend()` を持つ
  `common/pico_sdk/drivers/compat/rp2040/zephyr_os_port_rp2040.h` を読む）、`priorities_os.h`、OS の設定。
- tick: OS の tick のハンドラーかフックで、失われた tick の追いつき（`tick_catchup_step()`）と `zephyr_compat_tick()`
  （F は `freertos/app/usb-console/src/tick.c`）。
- Pico SDK の上のアプリの OS の部分（3 節）: `main_os_impl.h`（起動）、`idle_os.h`（アイドルの 1 回分）、`fault_os_impl.h`
  （異常停止）、`commands_os_impl.h`（`ps`、カーネルのバージョン）、`kernel_shell_os_impl.h`（汎用のコマンド）。
- ドライバーの待ち（5 節）: `common/drivers/i2c_dw/port/<OS>/i2c_dw_os_impl.h`、`common/drivers/nina/port/<OS>/nina_os.c` と
  `nina_bus_dma_os_impl.h`。
- ビルド: `common/pico_sdk/app/usb-console/usb-console.cmake` の変数を設定してソースを並べる `CMakeLists.txt`（F は
  `freertos/app/usb-console/CMakeLists.txt`）と、取得とビルドのスクリプト。

## 1. ファームウェアの構成

| 部分 | 場所 | 中身 |
|---|---|---|
| アプリ | `common/imu` | IMU サービス、MQTT の送信、時刻同期、Wi-Fi のプロファイル、KV ストア、校正。アプリの起動（`app_main.h`: `app_main.c` は 3 つの OS のファームウェアすべて、`app_init.c` は N と F。4.6 節） |
| ドライバー | `common/drivers/<ドライバー>/` | OS に依存しないコア、`port/` の境界のヘッダー、`port/<OS>/` のつなぎ（コアを OS の API につなぐファイル。5 節） |
| シェルのコマンド | `common/shell` | `zshell.c`（Zephyr のシェルのコマンド処理）、`subcmd.c`、`nina_shell.c`、`i2c_dma_shell.c`。F だけ: `dmesg_shell.c`（N は `nuttx/apps/dmesg`）、`kernel_shell.c`（`id`・`uptime`・`date`・`sleep`・`devmem`・`fault`・`reboot`・`bootsel`。`kernel_shell_os.h` の上。3 節）、`ifconfig_shell.c`（ネットワークの部分のインターフェースの上） |
| ログ | `common/compat/log` | RAM ログと `LOG_*` マクロの関数（`log_ram.c`。OS の部分 `log_ram_os.h` の上。3 節）、そのリング（`log_ring.c`）、`LOG_*` マクロ（`app_log.h`）、異常停止の記録の文面とその表示、HardFault の例外のフレーム（Cortex-M0+）、次の起動での記録の選び方（`fault_record.c`） |
| 互換層の共有部分 | `common/compat` | Zephyr のヘッダー（`common/compat/zephyr/`）、`k_heap`（`zephyr_heap.h`・`zephyr_heap.c`。どちらのカーネルの部分でも `<zephyr/kernel.h>` が読む。4.2）、ワークアイテム、ワークキューのスレッドと互換層の開始（`zephyr_init.c`）、RTIO、gpio0 のデバイス、net_mgmt、Wi-Fi の要求、KV のフラッシュのデバイス、Zephyr の NVS・settings・getopt・Wi-Fi の資格情報・zperf の写し（Zephyr のソースをそのまま、または縮めて写したもの） |
| 互換層のネットワークの部分（ネットワーク層を持たない OS のため） | `common/compat/net_offload` | Zephyr のソケット層（`subsys/net/lib/sockets`・`lib/os/zvfs` を縮めた写し）、ネットワークのインターフェース 1 つ（`zephyr_net_if.c`）とその開始（`zephyr_net_init.c`）、それらのヘッダーは専用のインクルードのルート `include/`（`zephyr/net/net_if.h`・`net_ip.h`・`socket.h`・`socket_offload.h`、`zephyr/sys/fdtable.h`、`zephyr_net_init.h`）。使うのは F だけ |
| POSIX のネットワークのヘッダー（それを持たない C ライブラリーのため） | `common/compat/libc_net` | `include/`（`arpa/inet.h`、`netdb.h`、`netinet/in.h`、`netinet/tcp.h`、`poll.h`、`sys/socket.h`、`sys/uio.h`）、`inet_pton()`・`inet_ntop()`（`inet.c`）。使うのは F だけ |
| 基本の操作だけを持つ OS のための互換層のカーネルの部分 | `common/compat/kernel_generic` | `k_sem`、`k_poll()`、タイムアウトの一覧、スリープ、スレッドの起動（`zephyr_kernel.c`）、`k_mutex`（`zephyr_mutex.c`）、`<zephyr/kernel.h>`・`<zephyr/spinlock.h>` の OS の部分（`zephyr_os.h`・`zephyr_os_spinlock.h`）、tick の入口（`zephyr_compat_tick()`、`zephyr_compat_os.h`）。すべて `zephyr_kernel_os.h` で宣言した基本の操作（2 節）の上。OS の周期的な tick のための、失われた tick の追いつき（`tick_catchup.h`）。既定の `zephyr_os_atomic.h`（空）。使うのは F だけ |
| その仕組みを自分で持たない OS のための互換層の部分 | `common/compat/os_generic` | `include/`: `zephyr/drivers/i2c.h`（ドライバーのデバイスの上の Zephyr の I2C の API。4.1）、`zephyr/sys/clock.h`、`zephyr/sys/__assert.h`（`CONFIG_ASSERT` が無いのと同じく何もしない）、`app_log_os.h`、`sensor_shell_os.h`、`wifi_profile_os.h`（3 節）。`sensor_shell_os.c`。Kconfig の無いビルドのための共有のソースのプレリュード（`common_prelude.h`。4.5）。使うのは F だけ |
| Pico SDK の上の OS のファームウェアで共有する部分 | `common/pico_sdk` | `app/usb-console/`: 起動（`src/main.c`）、コマンド `version`・`free` と表（`src/commands.c`）、アイドルの 1 回分と CPU 負荷（`src/idle.c`）、異常停止（`src/fault.c`）、割り込み用スタック（`src/irq_stack.c`）。境界は `main_os.h`・`idle_os.h`・`fault_os.h`・`commands_os.h`（3 節）。ビルド（`usb-console.cmake`、`ram_code.cmake`、`ld/`）。`drivers/`: Pico SDK の上の互換層の部分（`compat/`: `compat.cmake`、実時間の時計 `zephyr_sys_clock.c`、ソケットの登録のためのリンカースクリプトの部分、`rp2040/`: `zephyr_os_port_rp2040.h`、GPIO バンク 0、hwinfo）、コンソールの経路（`shell/`）、newlib の stdout・stderr の置き換え（`libc/`）、RAM ログの部分（`include/log_ram_os_impl.h`）、KV のフラッシュの操作とリンカースクリプトの部分（`kv/`）、各ドライバーのビルドのファイル（`<ドライバー>/<ドライバー>.cmake`）。使うのは F だけ |
| 互換層のカーネルのホストテスト（OS の部分の受け入れ試験） | `common/tests/zephyr_kernel` | `zephyr_kernel_test.c`（22 項目）、境界の宣言 `zephyr_kernel_test.h`（3 節）、汎用のカーネルの部分の基本の操作の上の境界の項目（`kernel_generic/zephyr_kernel_test_os.h`）。動かすのは F のホストテスト（`freertos/tests/compat`）で、ファームウェアには入らない |
| 自分のシェルを持たない OS のためのシェル | `common/compat/shell_engine` | 互換層の API の上: 行編集（`cli.c`）、`Z_SHELL_CMD_ROOT_GLOBAL` のルートのコマンドの表と実行と `help`（`shell_engine.c`。`shell_engine_cmds[]` はアプリが用意する）、Zephyr の単語の分割（`shell_argv.c`）、コンソールとスレッドごとの stdout（`shell_console.c`）、セッションごとの getopt の状態（`getopt_state.c`）、`zshell.c` のバックエンド（`zshell_os_impl.h`・`zephyr_os_shell.h`）、シリアルのセッション（`shell_serial.c`。スレッド "console"。コンソールの経路 `shell_serial_os.h` の上。3 節）、Telnet のシェル（`shell_telnet.c`。スレッド "shell_telnet"・"telnet_service"。`zsock_*()` の上）。使うのは F だけ |
| どの OS も持たない RP2040 のコード | `common/chip/rp2040` | `include/rp2040_timer_us.h`（TIMER のレジスターと 64 ビットの読み出し `rp2040_timer_us()`。必ずインライン。3 つの OS のファームウェアすべて）、`include/rp2040_watchdog_reboot.h`（`rp2040_watchdog_reboot()`、レジスターを直接書く watchdog の再起動。Zephyr 版と N）、KV の領域の flash のドライバーを持たない OS のための、RP2040 での KV のフラッシュの境界（`rp2040_flash_kv.c` が OS の側の flash の操作 `include/rp2040_flash_kv_os.h` の上で実装。3 節。`include/rp2040_flash_kv.h`。N と F。ビルドは `N-board/src/Make.defs` と `common/pico_sdk/drivers/kv/kv.cmake`）。各 OS のファームウェアが `include/` を include パスに入れる（Zephyr はアプリの `CMakeLists.txt`、N は `N-board/scripts/Make.defs` で全部のソースに、F はアプリの `CMakeLists.txt`） |
| ホストテスト | `common/tests` | テストと偽物。`common/tests/zephyr_fakes` はカーネルの OS 部分のホスト用の実装 |
| Zephyr 版 | `zephyr/app/usb-console` | `main()`、`log_ram.c`、`kv_store_os.h`、`wifi_profile_os.h`。上流の Zephyr と、`zephyr/manifest/zephyr/patches` のパッチ |
| N: ボード | `N-board/` | 起動、tickless のタイマー、アイドルループと CPU 負荷、GPIO バンク 0、KV のフラッシュ、USB シリアル、BOOTSEL（`include/rp2040_restart.h`） |
| N: 互換層の OS 部分 | `nuttx/drivers/compat` | `zephyr_os*.h`、カーネル、ミューテックス、`struct i2c_master_s` の上の I2C、net_if、hwinfo、プレリュード（共有のソースの前に読ませるヘッダー。4.5）、OS ごとの `<zephyr/...>` ヘッダー |
| N: 共有の OS ヘッダー | `nuttx/drivers/include` | `app_log_os.h`、`log_ram_os_impl.h`、`priorities_os.h` |
| N: ログ | `nuttx/drivers/log` | `log_ram_syslog.c`（RAM ログを syslog のチャネルにする）、`Make.defs` |
| N: ドライバー | `nuttx/drivers/<ドライバー>/` | 各ドライバーの `Make.defs`、`Kconfig` |
| N: アプリ | `nuttx/apps/*` | init タスク（`init`）、コマンドごとの builtin アプリ、シェルの OS 部分（`subcmd`）、Wi-Fi とセンサーの部分 |
| N: NuttX の変更 | `nuttx/patches` | NuttX と nuttx-apps へのローカルのパッチ |
| F: アプリ | `freertos/app/usb-console` | ビルド（`CMakeLists.txt`: `common/pico_sdk/app/usb-console/usb-console.cmake` に渡す設定とソース）、`FreeRTOSConfig.h`、tick のフック（`src/tick.c`）、`src/priorities_os.h`、Pico SDK の上のアプリの境界の FreeRTOS の部分（`src/main_os_impl.h`・`idle_os.h`・`fault_os_impl.h`・`commands_os_impl.h`: `ps` とカーネルのバージョン）、汎用のコマンドの OS の部分（`src/kernel_shell_os_impl.h`） |
| F: 互換層の OS 部分 | `freertos/drivers/compat` | 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）とそのポートのヘッダー（`rp2040/zephyr_os_port.h`） |
| F: ドライバーのつなぎ | `common/drivers/<ドライバー>/port/freertos` | NINA（`nina_os.c`、`nina_bus_dma_os_impl.h`）と I2C（`i2c_dw_os_impl.h`）の待ち・ロック・時計。つなぎの残りは `port/pico_sdk`（5 節） |

### 1.1 選べる部分が要るもの

カーネルの部分（N のように直接に書くか、`kernel_generic`）、ネットワークの部分（OS のネットワーク層か `net_offload`）、
POSIX のネットワークのヘッダー（C ライブラリーのものか `libc_net`）、シェル（OS のものか `shell_engine`）は、それぞれ
別に選べます。各部分が読むものは次のとおりです（`#include` から）。

| 部分 | 要るもの | 要らないもの |
|---|---|---|
| `common/compat/kernel_generic` | `common/compat` のヘッダー、OS の基本の操作（`zephyr_kernel_os_impl.h`。`z_os_thread_t` はポインターの型で、NULL はスレッドが無いこと）、OS の `priorities_os.h`、`zephyr_os_atomic.h`（OS のものかこのディレクトリーの既定のもの）、`<zephyr/sys/clock.h>`（OS のものか `os_generic` のもの） | `net_offload`、`libc_net`（`common/tests/run.sh` の `kernel_generic_independent`） |
| `common/compat/net_offload` | カーネルの部分（どちらでも）、POSIX のソケットのヘッダー（C ライブラリーのものか `libc_net`）、`common/compat` の `net_mgmt.h`・`wifi_mgmt.h`、`<zephyr/sys/__assert.h>`（OS のものか `os_generic` のもの） | `kernel_generic`（`net_offload_independent`。N の形の偽のカーネルの上で） |
| `common/compat/libc_net` | C ライブラリーの `<sys/types.h>` | ほかのすべて |
| `common/compat/shell_engine` | カーネルの部分の任意の 4 項目（2.1）、`common/shell/subcmd.h`、getopt の写し（`common/compat/lib/utils/getopt`）、ログ（`app_log.h`）、コンソールの経路（3 節）。Telnet のシェル（`shell_telnet.c`）だけ `zsock_*()`（OS のネットワーク層か `net_offload`） | `kernel_generic`（カーネルの部分はどちらでもよい） |
| `common/tests/zephyr_kernel` | カーネルの部分、`zephyr_kernel_test_os.h`（汎用のカーネルの部分では、このディレクトリーの `kernel_generic/` のものと OS の `zephyr_kernel_test_os_impl.h`。直接に実装する OS は自分で書く）、プラットフォームの部分（3 節） | ネットワーク、シェル |
| `common/compat/log` | OS の `app_log_os.h`・`log_ram_os_impl.h` | 互換層のほかの部分 |
| `common/compat/os_generic` | `i2c.h`: `common/compat` の `<zephyr/device.h>`・`<zephyr/sys/util.h>`。`sensor_shell_os.*`: カーネルの部分（`k_sem`）、`priorities.h`、センサーのシェルのヘッダー。`wifi_profile_os.h`: `<zephyr/kernel.h>`。ほかは何も読まない | — |
| `common/pico_sdk` | Pico SDK。汎用のカーネルの部分と OS の基本の操作（`gpio_os_rp2040.c` は `z_os_task_lock()`）。アプリの 3 節の OS の部分（`main_os_impl.h`・`idle_os.h`・`fault_os_impl.h`・`commands_os_impl.h`） | FreeRTOS の名前（FreeRTOS 版はディレクトリーを `PICO_SDK_OS_*` で渡す） |
| `common/chip/rp2040` | `rp2040_flash_kv.c` だけ: `common/compat` の `flash_kv.h`・`flash_kv_os.h`、ログ（`app_log.h`）。`include/` のヘッダーは何も読まない | — |
| `common/shell` | `kernel_shell.c`: ログ（`fault_record.h`）、`<zephyr/sys/clock.h>`、hwinfo。`dmesg_shell.c`: ログ。`ifconfig_shell.c`: `<zephyr/net/net_if.h>`（ネットワークの部分）。`nina_shell.c`: NINA の `include/`。`zshell.c`: シェルのバックエンド（3 節） | — |

## 2. 互換層のカーネルの部分

宣言は `common/compat/zephyr/kernel.h` で、このヘッダーは最初に `"zephyr_os.h"` を読みます。OS は次のどちらかの形で
用意します。

- **直接に実装する**（N。`nuttx/drivers/compat/`）: 自分の `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h` と、
  2.2 の関数を OS 自身のオブジェクト（セマフォ、watchdog、カーネルスレッド）の上に書きます。
- **基本の操作の上に置く**（F）: 汎用の実装（`common/compat/kernel_generic/`: `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_kernel.c`・
  `zephyr_mutex.c`・`zephyr_os_internal.h`・`zephyr_compat_os.h`）が、2.1・2.2 と
  スピンロックを、`common/compat/kernel_generic/zephyr_kernel_os.h` で宣言した基本の操作（2.5）の上に Zephyr の規則で実装します。OS が書くのは、
  その基本の操作（`zephyr_kernel_os_impl.h`。static inline。`zephyr_os_atomic.h` は、ヘッダーが atomic の名前を定義するときだけ。
  `kernel_generic` の既定のものは空）、tick の割り込みから 1 tick に 1 回
  `zephyr_compat_tick()` を呼ぶ tick のフック、互換層のカーネルのホストテストのための `zephyr_kernel_test_os_impl.h`（F は `freertos/tests/compat/`）です。

### 2.1 `zephyr_os.h` が用意するもの（16 項目、任意 3 項目）

「汎用」は汎用の実装の `zephyr_os.h`（F が使う）で、その項目は基本の操作から作ります。

| 項目 | 意味 | N | 汎用（F） |
|---|---|---|---|
| 互換層より先に読む OS のヘッダー | 互換層の `<zephyr/sys/atomic.h>` が atomic の名前を引き取るため | NuttX のヘッダー、`"rp2040_timer_us.h"`（`common/chip/rp2040`） | `zephyr_kernel_os.h`（OS のヘッダー。F は FreeRTOS のヘッダーと `zephyr_os_port.h`）、`priorities.h`、`<zephyr/sys/clock.h>` |
| `Z_USEC_PER_TICK` | tick の長さ（µs）。1000 がその倍数であること | `CONFIG_USEC_PER_TICK`（100） | `1000000 / Z_OS_TICK_HZ`（F は `configTICK_RATE_HZ`、1000） |
| `struct k_mutex`、`Z_MUTEX_INITIALIZER(obj)` | 全部 0 のものがロックされていないミューテックスとして動くこと | `rmutex_t` と `ready` の印 | OS のもの（F は静的な再帰ミューテックス。最初の使用で作る） |
| `struct k_sem`、`Z_SEM_INITIALIZER(obj, initial, limit)` | 上限つきの数、poll の事象の一覧 | `sem_t`、上限、事象 | 互換層自身の数、待ち手、事象 |
| `K_SEM_MAX_LIMIT` | 上限の最大値 | `SEM_VALUE_MAX` | `UINT_MAX` |
| `struct _timeout`、`_timeout_func_t` | 遅延つきワークアイテムのタイマー | `struct wdog_s` | 互換層のタイムアウトの一覧の要素 |
| `k_tid_t` | スレッドの識別 | `pid_t` | `z_os_thread_t`（F は `TaskHandle_t`） |
| `K_THREAD_STACK_RESERVED`（任意。既定は 0） | `K_THREAD_STACK_DEFINE()` が各スタックのあとに OS のために足すバイト数 | ―（0） | `Z_OS_THREAD_RESERVED`（F はタスクの制御ブロック、136） |
| `k_current_get()` | 呼んだスレッド | `gettid()` | `z_os_thread_self()` |
| `k_yield()` | 同じ優先度の次の実行可能なスレッドに譲る。無ければ続ける | `sched_yield()` | `z_os_yield()` |
| `K_PRIO_PREEMPT(x)` | Zephyr の優先度 x の OS の優先度。順序を保つ | `APP_PRIO_FROM_ZEPHYR`（118 − x） | `APP_PRIO_FROM_ZEPHYR`（F は 20 − x） |
| `k_cycle_get_32()`、`k_cycle_get_64()` | 1 MHz のカウンター | `rp2040_timer_us()` | `z_os_clock_us()`（F は `rp2040_timer_us()`） |
| `Z_OS_ASSERT(test)` | OS のアサーション。ワークキューのスレッドが起きなかったとき用 | `DEBUGASSERT()`（imu の構成では何もしない） | OS のもの（F は `configASSERT()`: 異常停止の記録と再起動） |
| `k_is_in_isr()`、`k_is_pre_kernel()`、`k_thread_custom_data_get()`、`k_thread_custom_data_set()`（任意。シェルのエンジン用） | ISR の中か。カーネルがスレッドを動かす前か。呼んだスレッドの 1 つのポインター（設定するまでとカーネルがスレッドを動かす前は NULL） | —（ビルドしない） | `z_os_in_isr()`、`z_os_pre_kernel()`、`z_os_thread_local_get()`・`_set()`（F はスレッドローカルのポインター 0） |
| `Z_SEM_TAKE_FOREVER(sem)`（任意） | `k_sem_take(sem, K_FOREVER)` を OS の短い経路で。ワークキューの待ち | `nxsem_wait_uninterruptible()`（`k_sem_take()` と同じ処理を、呼び出しなしで） | 定義しない: `k_sem_take()`（`kernel.h` の既定） |

`<zephyr/kernel.h>` は最後に `common/compat/zephyr_heap.h` を読みます: `struct k_heap`、`K_HEAP_DEFINE()`、`k_heap_alloc()`、`k_heap_free()`。OS に依存せず（`k_spin_lock()` の上）、どちらのカーネルの部分でも同じです（4.2）。

### 2.2 OS が実装する関数（18 関数）

ファイル: N `zephyr_kernel.c`・`zephyr_mutex.c`、汎用 `common/compat/kernel_generic/zephyr_kernel.c`・`zephyr_mutex.c`。

| 関数 | 求められる意味（kernel.h） |
|---|---|
| `k_mutex_init()` | 初期化 |
| `k_mutex_lock(m, t)` | 持ち主には再帰。K_FOREVER はシグナルで終わらない。K_NO_WAIT は −EBUSY、タイムアウトは −EAGAIN |
| `k_mutex_unlock(m)` | 1 段解放 |
| `k_sem_init(s, initial, limit)` | 0 か −EINVAL |
| `k_sem_take(s, t)` | 0、−EBUSY（K_NO_WAIT で無い）、−EAGAIN（タイムアウト） |
| `k_sem_give(s)` | 上限まで 1 足し、poll している者を起こす。割り込みを含むどの文脈からでも |
| `k_sem_reset(s)` | 数を 0 にする |
| `k_sem_count_get(s)` | 数 |
| `k_poll_event_init(e, type, mode, obj)` | セマフォだけ、`K_POLL_MODE_NOTIFY_ONLY` |
| `k_poll(events, n, t)` | どれかが揃えば 0（取りはしない）、タイムアウトで −EAGAIN |
| `k_thread_name_set(tid, name)` | `ps` に出る名前。0 か −EINVAL |
| `z_thread_static_start(thread)` | `*thread` の名前・静的なスタック（`K_THREAD_STACK_DEFINE()`。OS のための `K_THREAD_STACK_RESERVED` バイトを含む）・優先度の新しいスレッドで `entry(p1, p2, p3)` を動かし（`*thread` はスレッドが動くあいだ有効）、`entry` が戻ったら終える。`*tid` を入れる。0 か負の errno。互換層のスレッドはすべてこれで始める |
| `k_sleep(t)` | K_NO_WAIT は譲るだけ。ほかは今の tick + ticks + 1 で起きる。K_FOREVER は永久。0 を返す |
| `k_msleep(ms)`、`k_usleep(us)` | tick に切り上げてから `k_sleep()` と同じ。0 以下は譲るだけ。0 を返す |
| `k_busy_wait(us)` | 少なくとも `us` µs 回る |
| `z_add_timeout(to, fn, t)` | タイムアウトのあと `fn` を割り込みの文脈で動かす（`k_spin_lock()` を持つあいだは動かない） |
| `z_abort_timeout(to)` | 止める |

これらの上で共有するもの: ワークアイテムの処理（`common/compat/zephyr_work.c`）。`common/compat/zephyr_init.c` の
`k_work_queue_start()`（キューが自分のスレッドの `struct z_thread_static` を持つ。スレッドは `z_work_queue_process()`、
`k_yield()`、`Z_SEM_TAKE_FOREVER()` を繰り返す）、オブジェクト `k_sys_work_q`（"sysworkq"、`APP_PRIO_SYSWORKQ`）、
`zephyr_compat_initialize()`（宣言は `common/compat/zephyr_compat.h`。`k_sys_work_q`、そのスレッドが起きなければ −ENOMEM、
RTIO のワークキュー、`CONFIG_LSM6DSO_STREAM` のとき gpio0）、`k_uptime_get()`（`k_cycle_get_64()` の `Z_UPTIME_MS()`）。
"net_mgmt" のスレッド（`common/compat/zephyr_net_mgmt.c`）。

| 関数 | N | 汎用（F） |
|---|---|---|
| ミューテックス | `nxrmutex_*()` | `z_os_mutex_init()`・`_lock()`・`_unlock()` のまわりのタイムアウトと Zephyr の戻り値（F は `xSemaphore*Recursive()` と FreeRTOS の優先度継承） |
| `k_sem_*` | `nxsem_*()` | 互換層の数。待ち手は自分の起こしの印で眠る（`z_os_wait()`。F はタスク通知の番号 1） |
| `z_thread_static_start()` | `kthread_create_with_stack()`。起動のデータは変数で渡し、新しいスレッドが受け取るまで呼んだ側が待つ | `z_os_thread_start()`。データはスレッドの引数（F は `xTaskCreateStatic()`。作るあいだスケジューラーを止める） |
| スリープ | `nxsched_ticksleep()`、`nxsched_msleep()`、`nxsched_usleep()` | `z_os_sleep(ticks + 1)`（F は `vTaskDelay()`） |
| `k_busy_wait()` | `up_udelay()` | `z_os_busy_wait_us()`（F は `busy_wait_us_32()`） |
| `z_add_timeout()` | `wd_start()` | 並べた一覧を、OS の tick のフックから `zephyr_compat_tick()` が動かす |

### 2.3 N と F で動作が違うところ

| 点 | N | F（Zephyr の規則） |
|---|---|---|
| `k_sem_take()` で待つスレッドがいるときの `k_sem_give()` | 数を post し、登録された poll の事象にもすべて知らせる | 待ち手が数を受け取り、poll の事象には知らせない |
| poll している者がいるときの `k_sem_give()` | 登録された事象すべてに知らせ、全員を起こす | 最初に登録された事象だけに知らせる |
| `k_sem_reset()` と待ち手 | 数を空にし、待ち手は待ち続ける | 待ち手は −EAGAIN で戻る |
| 登録された poll の事象の順 | 新しいものが先頭（前に足す） | スレッドの優先度の順、同じ優先度では先着順 |
| 起きたあとの `k_poll()` | すべてのセマフォの数を見直す | 知らされた事象だけを数える |
| 割り込みの中でタイムアウトつきの `k_sem_take()` | 確かめない | −EBUSY |
| `k_mutex_unlock()` のエラー | `nxrmutex_unlock()` の結果 | −EINVAL（誰も持たない）、−EPERM（ほかの持ち主） |
| `k_thread_name_set()` | `CONFIG_TASK_NAME_SIZE` が無いと −ENOSYS | NULL は呼んだスレッド |
| `z_thread_static_start()` のスレッドの終わり | カーネルスレッドが戻る | `z_os_thread_exit()`（F は `vTaskDelete(NULL)`） |
| スレッドの起動 | 新しいスレッドが起動のデータを受け取るまで呼んだ側が待つ | 呼んだ側は続ける。タスクはスケジューラーが選んだときに走る（優先度が高ければすぐ） |
| ワークキューのスレッドが起きなかったとき（`Z_OS_ASSERT()`） | `DEBUGASSERT()`: imu の構成では何もせず、キューのスレッドは 0 のまま | `configASSERT()`: 異常停止の記録と再起動 |
| tick | 100 µs | 1 ms |
| `zephyr_init.c`（したがって gpio0 の開始）をビルドするか | `CONFIG_LSM6DSO_STREAM` のときだけ（カーネルの部分全体） | 常に。gpio0 は `CONFIG_LSM6DSO_STREAM` のときだけ |
| `zephyr_compat_initialize()` を呼ぶところ | `N-board/src/rp2040_bringup.c` | スケジューラーの前の `main()`（`common/pico_sdk/app/usb-console/src/main.c`） |
| 同じ優先度の選び方 | NuttX のもの | FreeRTOS の、切り替えのたびの順番回し |

F の列は、tick、同じ優先度の選び方、起動、アサーション（これらは OS のもの）を除き、汎用の実装の上のどの OS にも当てはまります。

### 2.4 スピンロックと atomic

| ヘッダー（共有） | 読むもの | OS が用意するもの | N | 汎用（F） |
|---|---|---|---|---|
| `common/compat/zephyr/spinlock.h` | `"zephyr_os_spinlock.h"` | `struct k_spinlock`、`k_spinlock_key_t`、`k_spin_lock()`、`k_spin_unlock()`: 割り込みを止め、解除まで切り替えない。スレッドと割り込みで。入れ子にできる | `sched_lock()` + `up_irq_save()` | スレッドでは `z_os_task_lock()`（F は `taskENTER_CRITICAL()`）、割り込みでは `z_os_isr_lock()`（F は `taskENTER_CRITICAL_FROM_ISR()`） |
| `common/compat/zephyr/sys/atomic.h` | `"zephyr_os_atomic.h"` | atomic の名前を定義する OS のヘッダーを先に読む | `<nuttx/atomic.h>` を読み、その `atomic_*` マクロを消す | OS のもの、または汎用の部分の空の既定のもの（F は既定のもの。`common/compat/kernel_generic/zephyr_os_atomic.h`） |

atomic の操作そのものは共有です（`k_spin_lock()` の上）。

### 2.5 汎用の実装の基本の操作（`common/compat/kernel_generic/zephyr_kernel_os.h`）

`zephyr_kernel_os.h` は OS の `"zephyr_kernel_os_impl.h"` を読み、OS が定義すべきものを宣言します（足りないものや違うものは
コンパイルが通らない）。すべて static inline です。`k_sem_give()`・`k_sem_take()`・`k_poll()`・`k_spin_lock()` と tick の
経路にあるからです（関数ポインターを通さない）。F のものは `freertos/drivers/compat/zephyr_kernel_os_impl.h` で、ポートに
よるものは `zephyr_os_port.h`（ボードは `freertos/drivers/compat/rp2040/`、ホストは `freertos/tests/compat/`）にあります。
ボードのものは、Pico SDK の上の OS で共通の `z_os_busy_wait_us()` を持つ
`common/pico_sdk/drivers/compat/rp2040/zephyr_os_port_rp2040.h` を読みます。

型と定数（10 個）: `z_os_thread_t`（`TaskHandle_t`）、`z_os_prio_t`（大きいほど急ぐ。`UBaseType_t`）、`z_os_ticks_t`（符号なし。
`TickType_t`）、`Z_OS_TICKS_FOREVER`（その最大の値で、終わりのない待ち。`portMAX_DELAY`）、`Z_OS_TICK_HZ`（`configTICK_RATE_HZ`）、
`z_os_woken_t`（最初は 0。`BaseType_t`）、`Z_OS_THREAD_NAME_SIZE`（`configMAX_TASK_NAME_LEN`）、`Z_OS_THREAD_RESERVED`（OS が各スレッドの
スタックのあとに制御ブロックのために取るバイト数。8 の倍数で、要らなければ 0。`K_THREAD_STACK_DEFINE()` がこれを足し（Zephyr の
`K_THREAD_STACK_RESERVED` と同じ）、`K_THREAD_STACK_SIZEOF()` はこれを除く。F は `sizeof(StaticTask_t)` を 8 に切り上げた 136）、
`struct k_mutex` と `Z_MUTEX_INITIALIZER()`、`Z_OS_ASSERT()`。

関数（26 個）。「ロック中」は `k_spin_lock()` の中、「ISR」は tick のフックを含みます。

| 基本の操作 | 意味 | F |
|---|---|---|
| `z_os_in_isr()` | ISR の中なら真。どの文脈からでも | IPSR ≠ 0（ホストは tick のフックの深さ） |
| `z_os_pre_kernel()` | OS がスレッドを動かす前なら真。どの文脈からでも | `xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED` |
| `z_os_task_lock()`、`z_os_task_unlock()` | スレッドのロック: 解除まで ISR もほかのスレッドも走らない。入れ子にできる。中で行う起こしはロックを保ち、その切り替えはいちばん外の解除で起きる | `taskENTER_CRITICAL()`、`taskEXIT_CRITICAL()`（ホストはスケジューラーも止める） |
| `z_os_isr_lock()`、`z_os_isr_unlock(state)` | ISR のロック: マスクを保存して戻す。状態は `UINT32_MAX` にならない | `taskENTER_CRITICAL_FROM_ISR()`、`taskEXIT_CRITICAL_FROM_ISR()` |
| `z_os_thread_self()` | 呼んだスレッド。スレッドだけ | `xTaskGetCurrentTaskHandle()` |
| `z_os_thread_prio_self()` | その今の優先度（継承を含む）。スレッドだけ | `uxTaskPriorityGet(NULL)` |
| `z_os_thread_start(fn, name, stack, size, prio, arg, tid)` | 静的なスタックの新しいスレッド。スタックは `K_THREAD_STACK_DEFINE()` の領域とその `K_THREAD_STACK_SIZEOF()`（8 の倍数）で、そのあとの `Z_OS_THREAD_RESERVED` バイトは OS が制御ブロックに使う（何も確保しない）。走れるようになる前に `*tid` を入れる。スレッドかスケジューラーの開始前から。0、−EINVAL、または OS のほかの負の errno | `xTaskCreateStatic()`。制御ブロックはスタックのあと。スケジューラーが動いていればまわりを `vTaskSuspendAll()`。0 か −EINVAL |
| `z_os_thread_exit()` | 呼んだスレッドを終える | `vTaskDelete(NULL)` |
| `z_os_thread_name(tid)` | スレッドの書き込める名前の配列（`Z_OS_THREAD_NAME_SIZE` バイト）。OS が表示するもの | `pcTaskGetName()` |
| `z_os_yield()` | 同じ優先度の次のスレッドに譲る。無ければ続ける。スレッド、ロックの外 | `taskYIELD()` |
| `z_os_thread_local_get()`、`z_os_thread_local_set(p)` | 呼んだスレッドの 1 つのポインター（Zephyr のカスタムデータ）。設定するまで NULL。get は OS がスレッドを動かす前は NULL | `pvTaskGetThreadLocalStoragePointer()`、`vTaskSetThreadLocalStoragePointer()`（番号 0） |
| `z_os_wait_prepare()` | 呼んだスレッドの起こしの印を消す。スレッド、ロック中 | `xTaskNotifyStateClearIndexed()`、`ulTaskNotifyValueClearIndexed()`（番号 1） |
| `z_os_wait(ticks)` | 印が立つか、今の tick + ticks まで待ち（`Z_OS_TICKS_FOREVER` は終わりなし）、印を消す。有限の待ちはこのどちらかでだけ戻る。スレッド、ロックの外 | `ulTaskNotifyTakeIndexed(1, pdTRUE, ticks)` |
| `z_os_wake(tid)` | tid の印を立てる。相手がまだ待っていなくてもよい（印は残る）。スレッド、ロック中。切り替えは解除まで待つ | `xTaskNotifyGiveIndexed()` |
| `z_os_wake_from_isr(tid, &woken)` | 同じことを ISR から、ロック中。切り替えが要るとき `woken` を立てる | `vTaskNotifyGiveIndexedFromISR()` |
| `z_os_yield_from_isr(woken)` | `woken` なら ISR の終わりで切り替える。ロックの外 | `portYIELD_FROM_ISR()`（ホストは何もしない） |
| `z_os_sleep(ticks)` | 今の tick + ticks まで眠る。スレッド、ロックの外 | `vTaskDelay()` |
| `z_os_suspend()` | 呼んだスレッドを終わりなく止める（戻ってもよい。呼び直す） | `vTaskSuspend(NULL)` |
| `z_os_busy_wait_us(us)` | 少なくとも `us` µs 回る。どの文脈からでも | `busy_wait_us_32()`（ホストは時計） |
| `z_os_clock_us()` | 起動からの µs、64 ビット、単調。どの文脈からでも | `rp2040_timer_us()`（TIMER。ホストは `CLOCK_MONOTONIC`） |
| `z_os_mutex_init(m)`、`z_os_mutex_lock(m, ticks)`、`z_os_mutex_unlock(m)` | OS 自身の再帰ミューテックスと、その優先度継承。全部 0 は未ロック。lock は取れたら真。unlock は 0、−EINVAL（誰も持たない）、−EPERM（ほかの持ち主） | `xSemaphoreCreateRecursiveMutexStatic()`（最初の使用でも）、`xSemaphoreTakeRecursive()`、`xSemaphoreGetMutexHolder()` + `xSemaphoreGiveRecursive()` |

RP2040 のポートのヘッダーには、`zephyr_os_port_rp2040.h` から、失われた tick の追いつき（`Z_OS_TICK_HZ` の上の
`common/compat/kernel_generic/tick_catchup.h`。F のフックは `freertos/app/usb-console/src/tick.c`）のための tick のフックの
`z_os_tick_now_us32()`・`z_os_tick_pend()` もあります。これらは汎用の部分の基本の操作ではありません。

## 3. そのほかの宣言された境界

| 境界 | 宣言 | 項目 | 呼ぶ側 | N | F |
|---|---|---|---|---|---|
| 汎用のシェルのコマンド | `common/shell/kernel_shell_os.h`（`"kernel_shell_os_impl.h"` を読む。static inline） | `kernel_shell_os_tick_count()`（OS の tick の数。`uptime`）、`kernel_shell_os_reboot(delay_ms)`・`kernel_shell_os_bootsel(delay_ms)`（`delay_ms` 後に再起動。後者は USB のローダーへ。戻らない）、`kernel_shell_os_fault_last()`（前回の `struct fault_record` か NULL）。ほかは互換層のもの（`k_cycle_get_64()`、`k_sleep()`、`Z_USEC_PER_TICK`、`sys_clock_gettime()`、`hwinfo_get_device_id()`） | `common/shell/kernel_shell.c` | —（ビルドしない。nsh のコマンドと `nuttx/apps/bootsel`） | `freertos/app/usb-console/src/kernel_shell_os_impl.h`: `xTaskGetTickCount()`、Pico SDK の `watchdog_reboot()`・`reset_usb_boot()`、`fault_last()`（`common/pico_sdk/app/usb-console/src/fault.c`）。ホストは `freertos/tests/shell/kernel_shell_os_impl.h` |
| シェルのバックエンド | `common/shell/zshell_os.h` | `zshell_os_backend_get()`、`zshell_os_cmd_enter()`、`zshell_os_cmd_exit()`、`z_shell_backend_vfprintf()`。任意で `ZSHELL_OS_ACTIVE_CMD` と `zshell_os_active_cmd_set()`・`_get()` | `common/shell/zshell.c` | `nuttx/apps/subcmd/zshell_os_impl.h`（`ZSHELL_OS_ACTIVE_CMD` なし） | シェルのエンジンの `common/compat/shell_engine/zshell_os_impl.h`（定義あり。コンソールの上） |
| コンソールの経路 | `common/compat/shell_engine/shell_serial_os.h`（`"shell_serial_os_impl.h"` を読む。static inline） | `shell_serial_os_stdio_init()`（全スレッドの stdout・stderr を `shell_console_write(shell_console_current(), ...)` へ。バッファーなし）、`shell_serial_os_rx_init()`（入力が届いたら `shell_serial_rx_ready()` を呼ぶ。どの文脈からでも）、`shell_serial_os_connected()`、`shell_serial_os_read()`（待たない）、`shell_serial_os_write()`（端末が受け取れない分は捨てる） | `common/compat/shell_engine/shell_serial.c` | —（ビルドしない） | `common/pico_sdk/drivers/shell/shell_serial_os_impl.h`: `stdio_usb`、`shell_stdio_init()`（`shell_stdio.c`）と `common/pico_sdk/drivers/libc/stdio_wrap.c`。ホストは `freertos/tests/shell/shell_serial_os_impl.h` |
| `struct shell` | `common/compat/zephyr/shell/shell.h` が読む | バックエンド 1 つ。コマンドより長く生きる | シェルのコマンド | `nuttx/drivers/compat/zephyr_os_shell.h`（端末のパス、コマンドの pid） | シェルのエンジンの `common/compat/shell_engine/zephyr_os_shell.h`（コンソール、実行中のコマンド） |
| ログの OS 部分 | `common/compat/log/app_log.h` | `app_log_os.h`（OS のヘッダーを先に、任意の `CONFIG_LOG_DEFAULT_LEVEL`・`CONFIG_LOG_MAX_LEVEL`、RAM ログのリングのバイト数 `LOG_RAM_SIZE`。`log_ring.h`・`log_ram.h` が読み、無ければコンパイルできない。N は 8192、F は 4096）、`app_log_printf()`、`app_log_vprintf()`、`app_log_ratelimit_pass()` | `LOG_*()` マクロ | `nuttx/drivers/include/app_log_os.h`。関数は共有の `common/compat/log/log_ram.c` | `common/compat/os_generic/include/app_log_os.h`。関数は共有（同じ） |
| RAM ログ | `common/compat/log/log_ram.h` | `log_ram_initialize()`、`log_ram_store()`（OS 自身のログ。そのままのバイト）、`log_ram_piece()` | 起動、OS のログ、`dmesg` | 共有の `common/compat/log/log_ram.c`。呼ぶのは `N-board/src/rp2040_boardinitialize.c`、`nuttx/drivers/log/log_ram_syslog.c`、`nuttx/apps/dmesg/dmesg_main.c` | 共有（同じ）。呼ぶのは `main()`、`common/shell/dmesg_shell.c`（`log_ram_store()` は使わない） |
| RAM ログの OS 部分 | `common/compat/log/log_ram_os.h`（`log_ram.c` が最初に `"log_ram_os_impl.h"` を読む） | `LOG_RAM_OS_RETAINED(name)`（起動のコードが触らない RAM にリングを置く）、型 `log_ram_os_irqstate_t`。inline の `log_ram_os_irq_save()`・`_irq_restore()`（ロック。割り込みを止める。どの文脈からでも）、`_uptime_us()`（時刻の印と頻度制限）、`_attach()`（OS 自身のログにする） | `common/compat/log/log_ram.c` | `nuttx/drivers/include/log_ram_os_impl.h`: `.noinit`、`up_irq_save()`、`rp2040_timer_us()`、syslog のチャネルを登録（`nuttx/drivers/log/log_ram_syslog.c`） | `common/pico_sdk/drivers/include/log_ram_os_impl.h`: `__uninitialized_ram`、`save_and_disable_interrupts()`、`rp2040_timer_us()`、登録するものなし。ホストは `common/tests/log_ram_os_host/` |
| KV のフラッシュ | `common/compat/flash_kv_os.h`（領域の定数と検査は `flash_kv.h`） | `flash_kv_os_initialize()`（起動の処理が 1 回）、`flash_kv_os_ready()`、`_read()`、`_program()`（256 バイトのページ）、`_erase()`（4 KiB のセクター）。範囲は OS がもう一度確かめる | `common/compat/zephyr_flash_kv.c`。起動の処理 | RP2040 で共有: `common/chip/rp2040/rp2040_flash_kv.c`（次の行）。`rp2040_bringup.c` が呼ぶ | 同じ。`main()` が呼ぶ |
| RP2040 での KV のフラッシュの操作 | `common/chip/rp2040/include/rp2040_flash_kv_os.h`。リンカースクリプトの `__kv_start`・`__kv_end` | `rp2040_flash_kv_os_init()`（リンカースクリプトの確認のあと 1 回）、`_image_end()`（イメージが終わる XIP のアドレス）、`_program(offset, page, lo, hi)`（RAM 上の 256 バイトのページ）、`_erase(offset, lo, hi)`（4 KiB のセクター）: XIP を止め割り込みを止めて行い、flash に触れる直前に要求が `[lo, hi)` に収まることをもう一度確かめる | `common/chip/rp2040/rp2040_flash_kv.c` | `N-board/src/rp2040_flash_kv_os.c`（ブート ROM、RAM 上の関数。NuttX の `rp2040_flash_mtd.c` は使わない） | `common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c`（Pico SDK の `flash_range_*()` を `flash_safe_execute()` で） |
| KV の消去のコマンド | `common/compat/kv_store_os.h` | `kv_store_wipe()`、`kv_store_cmd_wipe()`、`KV_STORE_OS_COMMANDS` | `common/imu/kv_store.c` | 共有: `common/compat/kv_store_wipe.c` | 共有: 同じ |
| GPIO バンク 0 | `common/compat/gpio_os.h` | OS: `gpio_os_configure_input()`、`gpio_os_get_all()`、`gpio_os_irq_configure()`。互換層: `gpio_os_irq_handler()`（バンクの割り込みから呼ぶ） | `common/compat/zephyr_gpio.c`。N では `common/drivers/nina/port/nuttx/nina_os.c` も | `N-board/src/rp2040_gpio_os.c` | `common/pico_sdk/drivers/compat/rp2040/gpio_os_rp2040.c` |
| CPU 負荷 | `common/compat/cpu_load.h`（計算も） | `cpu_load_get(reset)`。0.1% 単位、TIMER で測ったアイドルの区間から | `common/imu/imu_stream.c` | `N-board/src/rp2040_idle_cpuload.c` | アイドルタスクのフック（`idle_os.h`）と共有の 1 回分 `common/pico_sdk/app/usb-console/src/idle.c` |
| 優先度とスタック | `common/compat/priorities.h` | OS: `APP_PRIO_FROM_ZEPHYR(p)` と OS 自身のスレッド。共有: `APP_PRIO_*`・`APP_STACK_*` の 15 組 | 互換層、つなぎ、起動 | `nuttx/drivers/include/priorities_os.h`（118 − p。`NINA_EVENT`、`NINA_PT`） | `freertos/app/usb-console/src/priorities_os.h`（20 − p。`TIMER_SVC`、`IDLE`。互換層の `SYSWORKQ`・`NET_MGMT`・`SENSOR_SHELL`・`TELNET_SERVICE` のスタックを測った大きさに） |
| net_mgmt のスレッド | `common/compat/zephyr/net/net_mgmt.h` | `net_mgmt_event_init()` が "net_mgmt" を `z_thread_static_start()` で起こす。OS のファイルは無い | N `common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`、F `common/compat/net_offload/zephyr_net_init.c` | 共有の `common/compat/zephyr_net_mgmt.c` | 同じ |
| RTIO のワークキュー | `common/compat/zephyr/rtio/work.h` | `rtio_workq_initialize()` が "RTIO WQ 0" を `z_thread_static_start()` で起こす。OS のファイルは無い | `zephyr_compat_initialize()` | 共有の `common/compat/zephyr_rtio_workq.c` | 同じ |
| Wi-Fi のプロファイル | `common/imu/wifi_profile.c` が読む | `extern k_tid_t wifi_reconnect_tid;` | `common/imu/wifi_profile.c` | `nuttx/apps/wifi/wifi_profile_os.h` | `common/compat/os_generic/include/wifi_profile_os.h` |
| センサーのシェル | `common/shell/sensor_shell.c` と `common/compat/sensor_shell_stream.c` が最初に読む | `CONFIG_SENSOR_SHELL_STREAM`、`_THREAD_STACK_SIZE`、`_TRIG_PRINT_TIMEOUT_MS`、`_MAX_TRIGGER_DEVICES`。`sensor_shell_os_get_arm()`、`sensor_shell_os_get_wait()`、`sensor_shell_os_processing_callback` | その 2 つのファイル | `nuttx/apps/sensor/sensor_shell_os.h`、`sensor_shell_nuttx.h`、`sensor_shell_nuttx.c` | `common/compat/os_generic/include/sensor_shell_os.h`、`sensor_shell_os.c` |
| 互換層の開始 | `common/compat/zephyr_compat.h` | `zephyr_compat_initialize()`、`K_THREAD_DEFINE_START(name)` | 起動。`K_THREAD_DEFINE_START()` は `common/imu/app_init.c` | 共有の `common/compat/zephyr_init.c`（2 節） | 同じ |
| 互換層のカーネルのホストテスト、OS の部分 | `common/tests/zephyr_kernel/zephyr_kernel_test.h`（`zephyr_kernel_test.c` が `<zephyr/kernel.h>` のあとに `"zephyr_kernel_test_os.h"` を読む） | 型 `zephyr_kernel_test_os_prio_t`、`ZEPHYR_KERNEL_TEST_OS_NAME_MAX`、`zephyr_kernel_test_os_tick()`、`_tick_sync()`、`_thread_create()`、`_thread_exit()`、`_prio_get()`、`_prio_self()`、`_name_get()`、`_current()`、`_in_isr()`、`_sched_lock()`、`_sched_unlock()` | `common/tests/zephyr_kernel/zephyr_kernel_test.c` | — （動かさない） | 基本の操作の上の `common/tests/zephyr_kernel/kernel_generic/zephyr_kernel_test_os.h`（`_tick_sync()`・`_thread_exit()`・`_prio_self()`・`_name_get()`・`_in_isr()`、型と名前の長さ）。残りは `freertos/tests/compat/zephyr_kernel_test_os_impl.h`（FreeRTOS のタスク） |
| 互換層のカーネルのホストテスト、プラットフォームの部分 | 同じ | `zephyr_kernel_test_port_alarm()`、`_quiet()`、`_realtime()`、`_mask_irqs()`（起こせる割り込み、実時間に従う tick、止められる割り込みが無いプラットフォームは -ENOTSUP か false を返し、その確認は飛ばす） | 同じ | — | `freertos/tests/compat/main.c`（FreeRTOS の POSIX ポート。4 つとも飛ばす） |
| Pico SDK の上の起動 | `common/pico_sdk/app/usb-console/src/main_os.h`（`"main_os_impl.h"` を読む。static inline、always inline） | `MAIN_OS_PROMPT`（シェルのプロンプト）。`main_os_kernel_init()`（中継モードの確認のあと、カーネルと互換層の最初のオブジェクトより前: スケジューラーの前にオブジェクトとスレッドを作るために OS が要ること）。`main_os_thread_start(fn)`（起動のスレッド "main" を `APP_PRIO_MAIN`、`APP_STACK_MAIN` バイトで作る。静的、スケジューラーの前）。`main_os_thread_exit()`（その終わり。戻らない）。`main_os_kernel_start()`（スケジューラー。戻らない）。起動に属する OS のフックも置いてよい | `common/pico_sdk/app/usb-console/src/main.c` | — | `freertos/app/usb-console/src/main_os_impl.h`: `xTaskCreateStatic()`（スタックは SCRATCH_X）、`vTaskDelete(NULL)`、tick の確認と `vTaskStartScheduler()`、`app_run_time_us()` |
| Pico SDK の上のアイドルの 1 回分 | `common/pico_sdk/app/usb-console/src/idle.c`（`"idle_os.h"` を読む） | `IDLE_OS_HOOK`: `idle.c` が定義する `void IDLE_OS_HOOK(void)`（CPU 負荷の窓、LED の点滅、割り込みを止めての `WFI`）の名前。OS は、ほかに動けるスレッドが無いとき、いちばん低い優先度で、スレッドの文脈から割り込みを許可したまま、くり返し呼ぶ | OS のアイドルループ | — | `freertos/app/usb-console/src/idle_os.h`: `vApplicationIdleHook` |
| Pico SDK の上の異常停止 | `common/pico_sdk/app/usb-console/src/fault_os.h`（`fault.c` が入口の並びの中で `"fault_os_impl.h"` を読む） | `fault_os_thread_name(name, size)`（動いているスレッドの名前。RAM の中だけを読み、スケジューラーの前は何もしない。割り込みを止めた状態で、どの文脈からでも。static inline、always inline）。`fault_begin()`・`fault_finish()` の上の OS 自身の異常停止の入口 | `common/pico_sdk/app/usb-console/src/fault.c` | — | `freertos/app/usb-console/src/fault_os_impl.h`: 制御ブロックからタスクの名前、`vApplicationStackOverflowHook()` |
| Pico SDK の上のコマンド | `common/pico_sdk/app/usb-console/src/commands_os.h`（`"commands_os_impl.h"` を読む） | `COMMANDS_OS_KERNEL_NAME`、`COMMANDS_OS_KERNEL_VERSION`（`version`: 「<名前> <バージョン>, Pico SDK <x>, GCC <y>」）。`ps` の処理 `static int cmd_ps()`（スレッドごとの行、`(interrupts)`、前回の `ps` からの `cpu_active`。登録は `commands.c`） | `common/pico_sdk/app/usb-console/src/commands.c` | — | `freertos/app/usb-console/src/commands_os_impl.h`: `tskKERNEL_VERSION_NUMBER`、`uxTaskGetSystemState()` |
| Pico SDK の上の NINA の SPI の DMA の待ち | `common/drivers/nina/port/pico_sdk/nina_bus_dma_os.h`（`"nina_bus_dma_os_impl.h"` を読む。static inline） | 型 `nina_bus_dma_os_thread_t`（NULL は無し）、`nina_bus_dma_os_woken_t`（`NINA_BUS_DMA_OS_NOT_WOKEN`）、`nina_bus_dma_os_ticks_t`。`nina_os_lock()` を持つスレッドから: `nina_bus_dma_os_running()`、`_self()`、`_wake_clear()`、`_ticks_from_ms(ms)`（切り上げて 1 tick 足す）、`_tick_count()`、`_wake_wait(ticks)`。DMA の割り込みから: `_wake_from_isr(thread, &woken)`、`_isr_end(woken)`。起こしは `nina_os.h` の待ちと同じスレッド自身のもの | `common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c` | —（N のつなぎは DMA を自分で持つ） | `common/drivers/nina/port/freertos/nina_bus_dma_os_impl.h`: タスク通知のインデックス 2（always inline） |
| 周期的な tick の追いつき | `common/compat/kernel_generic/tick_catchup.h`（`Z_OS_TICK_HZ` の上） | `struct tick_catchup`、`tick_catchup_step(s, now)`（数えた tick ごとに 1 回、TIMER の下位 32 ビットで。真なら tick をもう 1 つ足す: `z_os_tick_pend()`） | OS の tick のフック（`z_os_tick_now_us32()` で） | —（tickless） | `freertos/app/usb-console/src/tick.c` |
| 汎用のカーネルの部分の tick | `common/compat/kernel_generic/zephyr_compat_os.h` | `zephyr_compat_tick()`: 1 tick に 1 回、tick の割り込みから | F の tick のフック | — | `common/compat/kernel_generic/zephyr_kernel.c` |
| ネットワーク層を持たない OS のネットワークの部分の開始 | `common/compat/net_offload/include/zephyr_net_init.h` | `zephyr_net_initialize()` | F の `main_main()` | —（`nina_net_initialize()`） | 共有の `common/compat/net_offload/zephyr_net_init.c` |
| アプリの起動 | `common/imu/app_main.h` | `app_init_post_kernel()`、`app_threads_start()`、`app_main()` をこの順に呼ぶ（4.6 節）。モジュールのオプション（`CONFIG_KV_STORE`、`CONFIG_APP_IMU`、`CONFIG_IMU_RAW_MQTT`、`CONFIG_TIME_SYNC`、`CONFIG_WIFI_CREDENTIALS`、`CONFIG_WIFI_PROFILE`、`CONFIG_APP_SENSOR`、`CONFIG_WIFI_NINA_LOCAL`、`CONFIG_NET_ZPERF`）は、そのモジュールをビルドするファームウェアが定義する。`CONFIG_APP_IMU` は MQTT の送信と時刻同期の API を常に使うので、`CONFIG_IMU_RAW_MQTT`・`CONFIG_TIME_SYNC` が無ければその代わりに `common/imu/imu_raw_mqtt_null.c`・`time_sync_null.c` をビルドする（7 節） | 各 OS の起動のファイル（Zephyr は `app_main()` だけ） | 共有の `common/imu/app_main.c`、`app_init.c`。オプションは Kconfig。呼ぶのは `nuttx/apps/init/rp2040_init.c` | 共有: 同じ。オプションは各モジュールの `.cmake`。呼ぶのは `main_main()` |

Zephyr 版は自分の `zephyr/app/usb-console/src/kv_store_os.h` と `wifi_profile_os.h` を持ちます。

## 4. 共有の宣言なしに各 OS のファームウェアが用意するもの

### 4.1 OS ごとに用意する `<zephyr/...>` ヘッダー

インクルードパスでは OS のディレクトリーが先で、ほかは `common/compat/zephyr/` にあります。F のネットワークのヘッダーは
`common/compat/net_offload/include` にあり、F はそれを OS のディレクトリーのあと、`common/compat` の前に置きます。N のインクルードパスには
ありません。

| ヘッダー | N | F |
|---|---|---|
| `zephyr/drivers/i2c.h` | `nuttx/drivers/compat/zephyr/drivers/i2c.h`。`struct i2c_master_s` の上（`zephyr_i2c.c`） | `common/compat/os_generic/include/zephyr/drivers/i2c.h`（Zephyr の API の表） |
| `zephyr/net/net_if.h` | `struct net_driver_s` の上（`zephyr_net_if.c`） | `common/compat/net_offload`: NINA のインターフェース 1 つ（`zephyr_net_if.c`） |
| `zephyr/net/net_ip.h` | —（`common/compat` のものを使う） | `common/compat/net_offload`。`common/compat` のものに代わる。POSIX の型に `net_*` の名前を付ける |
| `zephyr/net/socket.h` | `zsock_*()` は NuttX の `socket()`・`poll()` など（usrsock） | `common/compat/net_offload`: Zephyr のソケット層、`subsys/net/lib/sockets/sockets.c`、`socket_offload.c`、`getaddrinfo.c` |
| `zephyr/net/socket_offload.h` | — | `common/compat/net_offload`: オフロードした名前解決 |
| `zephyr/sys/fdtable.h` | — | `common/compat/net_offload`: 記述子の表、`lib/os/zvfs/zvfs_fdtable.c`、`zvfs_poll.c` |
| `zephyr/sys/__assert.h` | `DEBUGASSERT()` | `common/compat/os_generic/include/zephyr/sys/__assert.h`: 何もしない（`CONFIG_ASSERT` が無いのと同じ） |
| `zephyr/sys/clock.h` | `clock_gettime()`・`clock_settime()` の上のインライン | `common/compat/os_generic/include/zephyr/sys/clock.h`。実装は `common/pico_sdk/drivers/compat/zephyr_sys_clock.c`（TIMER と基準） |

### 4.2 関数とオブジェクト

| 項目 | 宣言 | N | F |
|---|---|---|---|
| `rp2040_timer_us()`: 起動からの µs、TIMER、どの文脈からでも | `common/chip/rp2040/include/rp2040_timer_us.h`（共有。Zephyr 版の `common/imu/time_sync.c` も） | `nuttx/drivers/compat/zephyr_os.h`、`nuttx/drivers/include/log_ram_os_impl.h`、ボードが読む | ポートのヘッダー `freertos/drivers/compat/rp2040/zephyr_os_port.h`、`common/pico_sdk/drivers/include/log_ram_os_impl.h`、`zephyr_sys_clock.c`、`idle.c`、`fault.c` が読む |
| `rp2040_watchdog_reboot()`: watchdog での再起動。scratch 0..3 は残る | `common/chip/rp2040/include/rp2040_watchdog_reboot.h`（共有。Zephyr 版の `common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c` も） | `N-board/src/rp2040_restart.c`（`rp2040_reset_usb_boot()` の代わりの道）、`common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c` | 使わない。Pico SDK の `watchdog_reboot()` |
| `k_heap` | `common/compat/zephyr_heap.h`（`<zephyr/kernel.h>` が読む。どちらのカーネルの部分でも） | ビルドしない（使うものが無い） | `common/compat/zephyr_heap.c`。使うのは `common/drivers/nina/port/zephyr/nina_socket.c` |
| `zephyr_compat_tick()` | `common/compat/kernel_generic/zephyr_compat_os.h`（汎用のカーネルの部分） | —（wdog） | `vApplicationTickHook()`（`freertos/app/usb-console/src/tick.c`）が呼ぶ |
| `zephyr_net_initialize()` | `common/compat/net_offload/include/zephyr_net_init.h` | —（`rp2040_bringup.c` の `nina_net_initialize()`、`common/drivers/nina/port/nuttx/nina_net.h`） | `common/compat/net_offload/zephyr_net_init.c` |
| `hwinfo_get_device_id()` | `common/compat/zephyr/drivers/hwinfo.h` | `nuttx/drivers/compat/zephyr_hwinfo.c` | `common/pico_sdk/drivers/compat/rp2040/zephyr_hwinfo.c` |

### 4.3 `DEVICE_DT_GET()` の向こうのデバイス

`DEVICE_DT_GET(DT_NODELABEL(x))` は `&__device_x` です（`common/compat/zephyr/device.h`）。作るのは `DEVICE_DT_DEFINE()` です。
シェルに見せるデバイスの一覧は `common/compat/zephyr_device.c` に手で書いてあります。

| ノード | N | F |
|---|---|---|
| `gpio0` | `common/compat/zephyr_gpio.c` | 同じ |
| `i2c0` | `N-board/src/rp2040_lsm6dso.c`。NuttX の I2C マスターに結び付ける（`zephyr_i2c.c`） | `common/drivers/i2c_dw/port/pico_sdk/i2c_dw_instance.h` |
| `i2c1` | Zephyr のデバイスは無い（`struct i2c_master_s` だけ） | `common/drivers/i2c_dw/port/pico_sdk/i2c_dw_instance.h` |
| `lsm6dsox` | `common/drivers/lsm6dso/port/nuttx/lsm6dso_instance.h`（`port/zephyr/lsm6dso_sensor.c` が最後に読む） | `common/drivers/lsm6dso/port/pico_sdk/lsm6dso_instance.h`（同じ） |
| フラッシュのデバイス | `common/compat/zephyr_flash_kv.c`（`z_flash_kv_device`） | 同じ |

### 4.4 シェルのコマンド: 登録、getopt、stdio

| 点 | N | F |
|---|---|---|
| 登録 | コマンドごとの builtin アプリ。`SHELL_CMD_REGISTER()` が `main()`（`Z_SHELL_CMD_NAMED_MAIN` なら `z_shell_main_<名前>()`）を作る | `Z_SHELL_CMD_ROOT_GLOBAL`: 大域のルートの要素を `shell_engine_cmds[]`（`common/pico_sdk/app/usb-console/src/commands.c`）に並べ、`common/compat/shell_engine/shell_engine.c` が動かす |
| `SHELL_SUBCMD_ADD()` | 大域の要素。親のソースが手で並べる | 同じ |
| getopt | Zephyr の getopt はビルドしない。`nuttx/apps/wifi/wifi_credentials_shell.c` が `sys_getopt_*` を `getopt_long()` に読み替える | `common/compat/lib/utils/getopt` の写し。状態はセッションごと（`common/compat/shell_engine/getopt_state.c` の `sys_getopt_state_get()`、`z_getopt_global_state_update()`）。コマンドの前に `sys_getopt_init()` で戻す |
| `zshell.c` の stdio: `printf()`、`fprintf(stderr)`、`fflush(stdout)` がコマンドの端末に届くこと。1 行あたり約 1.2 KB の `calloc()` | コマンドごとのタスクが自分の stdio を持つ | コンソールの経路の stdio の行き先（3 節）: `common/pico_sdk/drivers/shell/shell_stdio.c`（呼んだタスクのコンソール）、`common/pico_sdk/drivers/libc/stdio_wrap.c`（stdout・stderr の呼び出しの `--wrap`） |

### 4.5 `CONFIG_*` の値

| 出どころ | N | F |
|---|---|---|
| プレリュード（共有のソースに `-include`） | `nuttx/drivers/compat/common_prelude.h`: NuttX のヘッダーを先に、`CONFIG_SENSOR_LOG_LEVEL` 3、`CONFIG_LSM6DSO_STREAM` のとき `CONFIG_SENSOR_ASYNC_API` | `common/compat/os_generic/common_prelude.h`: `CONFIG_SENSOR_LOG_LEVEL` 3、`CONFIG_NINA_NTP_SERVER` ""・`_PORT` 123・`_INTERVAL_MS` 2000、`CONFIG_NINA_TCP_MAX_IO` 3584、`CONFIG_NINA_SEND_RETRY_MS` 1、`CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` 40、`CONFIG_NINA_PASSTHROUGH` 1、ストリームのとき `CONFIG_SENSOR_ASYNC_API` |
| NINA と NTP のオプション | Kconfig: `nuttx/drivers/nina/Kconfig`、`nuttx/apps/imu/Kconfig` | プレリュード（上） |
| NVS と settings の写し | `common/compat/zephyr_kv_config.h`。`nuttx/apps/imu/Makefile` から `-include` | 同じ。`common/pico_sdk/drivers/kv/kv.cmake` から |
| ビルドの定義 | Kconfig（`nuttx/drivers/*/Kconfig`、`nuttx/apps/*/Kconfig`、`N-board/Kconfig`）、`N-board/configs/imu/defconfig` | `compat.cmake`: `Z_SHELL_CMD_ROOT_GLOBAL`、`CONFIG_GETOPT_LONG=1`、`PICO_STDOUT_MUTEX=0`。`i2c_dw.cmake`: `CONFIG_I2C_DW_I2C0`、`_I2C1`。`lsm6dso.cmake`: `CONFIG_LSM6DSO_STREAM`。`wifi.cmake`: `CONFIG_WIFI_CREDENTIALS`、`CONFIG_WIFI_PROFILE`。`nina.cmake`: 名前解決の `NINA_OS_DNS_OFFLOAD`、`CONFIG_WIFI_NINA_LOCAL`。`i2c_shell.cmake`: `CONFIG_I2C_SHELL_BUFFER_SIZE`、`CONFIG_LOG_DEFAULT_LEVEL`（`i2c_shell.c` だけ。N は Kconfig）。アプリの起動が読むモジュールのオプション（4.6 節）: `kv.cmake` の `CONFIG_KV_STORE`、`imu.cmake` の `CONFIG_APP_IMU`・`CONFIG_IMU_RAW_MQTT`・`CONFIG_TIME_SYNC`、`sensor_shell.cmake` の `CONFIG_APP_SENSOR`、`zperf.cmake` の `CONFIG_NET_ZPERF` |
| 共有のヘッダーの中 | `kernel.h`: `CONFIG_THREAD_NAME` 1。`zephyr/rtio/work.h`: `CONFIG_RTIO_WORKQ_POOL_ITEMS` 4 | 同じ |
| センサーのシェル | `sensor_shell_nuttx.h` | `sensor_shell_os.h`（同じ 4 つの値） |

### 4.6 スレッドと起動の順序

アプリの起動は共有の 1 つの手順 `common/imu/app_main.h` で、Zephyr 版のカーネルと `main()` の順です。

1. `app_init_post_kernel()`（`app_init.c`）: アプリの `SYS_INIT(POST_KERNEL)` の部分。`wifi_credentials_init()`
   （`CONFIG_WIFI_CREDENTIALS`。NVS をマウントし、保存した SSID を読む）
2. OS のシェルの `SYS_INIT(APPLICATION)` の部分（F: `wifi_shell_init()`、`shell_telnet_init()`）
3. `app_threads_start()`（`app_init.c`）: `z_init_static_threads()`。下のスレッドの 1 つの表で、1 本ずつ
   `K_THREAD_DEFINE_START()` で起こし、失敗はログに書く（`Could not start <名前>: <errno>`）
4. `app_main()`（`app_main.c`）: `main()`。`USB console started`、`kv_store_init()`、`imu_cal_init()`、
   `time_sync_init()`、`wifi_profile_init()`

Zephyr 版は `main()` から `app_main()` だけを呼びます。1〜3 はカーネルが行います（`SYS_INIT()`、`K_THREAD_DEFINE()`）。
N と F はデバイスのあとにスレッドから 1・3・4 を呼びます。Zephyr 版は `app_init.c` をビルドしません。表の各項目は
そのモジュールのオプションの下にあります。`common/tests/app_threads.py`（`common/tests/run.sh` が実行）が、表が
ソースの `K_THREAD_DEFINE()` を 1 回ずつ挙げていることと、各 OS のファームウェアが定義のファイルをビルドするときに
だけオプションを定義していることを確かめます。

| スレッド（定義のファイル）、表の順 | オプション | N | F |
|---|---|---|---|
| `nina_wifi_tid`（`common/drivers/nina/port/zephyr/nina_net.c`） | `CONFIG_WIFI_NINA_LOCAL` | ビルドしない。代わりに同じ位置で `nina_net_start()`（`CONFIG_NINA_NET`）がコアのスレッドを起こす | 起こす |
| `imu_stream_tid`（`common/imu/imu_stream.c`） | `CONFIG_APP_IMU` | 起こす | 同じ |
| `imu_raw_mqtt_tid`（`common/imu/imu_raw_mqtt.c`） | `CONFIG_IMU_RAW_MQTT` | 同じ | 同じ |
| `time_sync_tid`（`common/imu/time_sync.c`） | `CONFIG_TIME_SYNC` | 同じ | 同じ |
| `wifi_reconnect_tid`（`common/imu/wifi_profile.c`） | `CONFIG_WIFI_PROFILE` | 同じ | 同じ |
| `sensor_shell_tid`（`common/compat/sensor_shell_stream.c`） | `CONFIG_APP_SENSOR` | 同じ | 同じ |
| `zperf_server_tid`（`common/compat/zperf_iperf3.c`） | `CONFIG_NET_ZPERF` | ビルドしない | 同じ |

起動のファイル: Zephyr `zephyr/app/usb-console/src/main.c`（中継モード（NINA のファームウェアを PC から書き換えるための経路）の分岐、`app_main()`）。N
`N-board/src/rp2040_boardinitialize.c`、`N-board/src/rp2040_bringup.c`、`nuttx/apps/init/rp2040_init.c`。
F `common/pico_sdk/app/usb-console/src/main.c`（`main()`、そのあとタスク `main_main()`。Pico SDK の上の OS のファームウェアで共有し、
境界は `main_os.h`）。OS が書くのは、アプリの起動より前の段階（下の表）と、その位置での 1〜4 の呼び出しです。Pico SDK の上では
`main_os_impl.h` だけです。

| 段階 | N | F |
|---|---|---|
| RAM ログ | `rp2040_boardinitialize()` の `log_ram_initialize()` | `main()` の最初 |
| NINA の中継モードの確認 | init タスクの `nina_passthrough_active()` | `main()` の `nina_passthrough_check()` |
| 互換層 | `rp2040_bringup()` の `zephyr_compat_initialize()` | `main()` の `zephyr_compat_initialize()` |
| I2C0、I2C1 | `rp2040_bringup()` の `i2c_dw_i2c0_initialize()`、`i2c_register()` | `main()` の `i2c_dw_i2c0_initialize()`、`_i2c1_` |
| LSM6DSOX | `rp2040_bringup()` の I2C0 のあと | `main_main()` の最初 |
| ネットワーク | `rp2040_bringup()` の `nina_net_initialize()` | `main_main()` の `zephyr_net_initialize()` |
| KV のフラッシュの範囲 | `rp2040_bringup()` の `flash_kv_os_initialize()` | `main()` の `flash_kv_os_initialize()` |
| シェル | init タスクの `nsh_initialize()`（1 の前） | `main()` のコンソールのタスク（`shell_serial_start()`） |
| アプリの起動 | init タスクで 1、3、Telnet（`rp2040_telnet_start()`、スレッドのあと）、4 | `main_main()` で 1、2（`wifi_shell_init()`、`shell_telnet_init()`）、3、4 |

### 4.7 POSIX のソケットのヘッダー

| N | F |
|---|---|
| NuttX の libc（`<sys/socket.h>`、`<netinet/in.h>`、`<arpa/inet.h>`、`<netdb.h>`、`<poll.h>`） | `common/compat/libc_net/include`: `arpa/inet.h`、`netdb.h`、`netinet/in.h`、`netinet/tcp.h`、`poll.h`、`sys/socket.h`、`sys/uio.h`。`common/compat/libc_net/inet.c` |

## 5. ドライバー

| ドライバー | 境界のヘッダー（`port/`） | N のファイル | F のファイル | Zephyr |
|---|---|---|---|---|
| `common/drivers/i2c_dw` | `i2c_dw_os.h`: 型 4、マクロ 5、インライン関数 10、`i2c_dw_os_uptime_ms()`。最後に読む `i2c_dw_instance.h`（`i2c_dw_os_prepare()`、`i2c_dw_os_irq_enable()`、デバイス）。`i2c_dw_dma_os.h`: 関数 10、コールバックの型 1 | `port/nuttx/i2c_dw_os_impl.h`、`i2c_dw_instance.h`（`struct i2c_master_s`）、`i2c_dw_dma_os_rp2040.c`、`i2c_dw_controllers_rp2040.h` | `port/freertos/i2c_dw_os_impl.h`。Pico SDK の上（共有）の `port/pico_sdk/i2c_dw_instance.h`（Zephyr のデバイス）、`i2c_dw_dma_os_rp2040.c`、`i2c_dw_controllers_rp2040.h` | 上流のドライバー + パッチ。`port/zephyr` は無い |
| `common/drivers/nina` | `nina_bus.h`（関数 7）、`nina_os.h`（関数 18、`NINA_OS_DNS_OFFLOAD` でさらに 3）、`nina_passthrough_bus.h`（17）、`nina_passthrough_os.h`（5、構造体、ログのマクロ）。つなぎは `nina_passthrough_run()` も実装する | `port/nuttx/nina_bus_rp2040.c`、`nina_os.c`、`nina_passthrough_bus_rp2040.c`、`nina_passthrough_os.c`。ネットワーク: `nina_netdev.c`、`nina_usrsock.c`、`nina_poller.c`、`nina_wifi_mgmt.c` | `port/freertos/nina_os.c`、`nina_bus_dma_os_impl.h`。Pico SDK の上（共有）の `port/pico_sdk/nina_bus_rp2040.c`（長い転送の DMA は `nina_bus_dma_rp2040.c`。OS の待ちは `nina_bus_dma_os.h`、3 節）、`nina_passthrough_bus_rp2040.c`、`nina_passthrough_os.c`、`nina_board_rp2040.h`。ネットワーク: 互換層の上の `port/zephyr/nina_net.c`、`nina_socket.c` | `port/zephyr/` |
| `common/drivers/lsm6dso` | `lsm6dso_os.h`: 型 5、ログのマクロ、関数 20（INT1 の開始と停止の 2 つは普通の関数、ほかは static inline）。`lsm6dso_stream_req.h`（FIFO のストリームの使い手）: 型 2、inline 関数 6。どちらも実装は Zephyr の API の上の `port/zephyr/`（`lsm6dso_os_impl.h`・`lsm6dso_os.c`、RTIO の `lsm6dso_stream_req_impl.h`）で、3 つの OS とも同じもの | `port/nuttx/lsm6dso_instance.h`（デバイス 1 つ、`board.h` の値）。ほかは互換層の上の `port/zephyr/`（`lsm6dso_sensor.c`、`lsm6dso_rtio.c`、`lsm6dso_rtio_stream.c`、`lsm6dso_decoder.c`、`lsm6dso_os.c`）。センサーの開始 `N-board/src/rp2040_lsm6dso.c`。ビルド `nuttx/drivers/lsm6dso/Make.defs`・`Kconfig` | Pico SDK の上（共有）の `port/pico_sdk/lsm6dso_instance.h`。ほかは同じ `port/zephyr/`。センサーの開始 `common/drivers/lsm6dso/port/pico_sdk/lsm6dso_board.c`。ビルド `common/pico_sdk/drivers/lsm6dso/lsm6dso.cmake` | Zephyr のモジュール `port/zephyr`（`zephyr/module.yml`。アプリの `ZEPHYR_EXTRA_MODULES`）、`CONFIG_LSM6DSO_LOCAL`（`port/zephyr/Kconfig`。Zephyr の `CONFIG_LSM6DSO` の代わり）、`port/zephyr/lsm6dso_instance.h`（デバイスツリーのインスタンス） |

`i2c_dw_i2c0_initialize()` の戻り値は、N が `struct i2c_master_s *`、F が `int` です（`i2c_dw_controllers_rp2040.h`）。

LSM6DSO のコア（`common/drivers/lsm6dso` の直下の `.c`、`include/`、`internal/`、`port/` の境界のヘッダー）は OS の定義を
読みません（`common/tests/run.sh` の `lsm6dso_core_independent`）。各 OS のファームウェアが自分で書くのは、デバイスを
定義する `port/<OS>/lsm6dso_instance.h`（include パスで `port/<OS>` を `port/zephyr` の前に置いて選ぶ。Pico SDK の上では `port/pico_sdk`）、ボードでの
センサーの開始、ビルドの規則です。ST のレジスターのドライバー `lsm6dso_reg.c` は各 OS の `setup.sh` が取得し
（N `nuttx/stmemsc`、F `freertos/stmemsc`。F の `CMakeLists.txt` が共有の cmake のファイルに `PICO_SDK_STMEMSC_DIR` で渡す）、そのつなぎは共有の `common/drivers/stmemsc` です。

## 6. どの場合も OS ごとのもの

| 項目 | N | F |
|---|---|---|
| スケジューラーまでの起動 | NuttX の起動、`N-board/src/rp2040_boardinitialize.c`、`rp2040_bringup.c`、init タスク `nuttx/apps/init/rp2040_init.c` | 共有の `common/pico_sdk/app/usb-console/src/main.c` と `main_os_impl.h`（`xTaskCreateStatic()`、`vTaskStartScheduler()`） |
| カーネルの tick とタイマー | tickless のタイマー `N-board/src/rp2040_oneshot.c`、`rp2040_timerisr_alarm.c`（tick 100 µs） | SysTick 1 ms、tick のフック `tick.c` と共有の追いつき `common/compat/kernel_generic/tick_catchup.h`。`FreeRTOSConfig.h` |
| 割り込みの登録 | `irq_attach()`、`rp2040_gpio_irq_attach()` | Pico SDK の `irq_set_exclusive_handler()`、`irq_add_shared_handler()`、`gpio_add_raw_irq_handler_masked()` |
| アイドルループ（区間の計算は共有の `common/compat/cpu_load.h`） | `N-board/src/rp2040_idle_cpuload.c` | アイドルタスクのフック（`idle_os.h`）と共有の 1 回分 `common/pico_sdk/app/usb-console/src/idle.c` |
| BOOTSEL に入る（ROM の `reset_usb_boot()`） | `N-board/src/rp2040_restart.c`。NuttX の `ROM_LOOKUP()` の上 | Pico SDK の `reset_usb_boot()`（Zephyr 版は hal_rpi_pico のもの） |
| チップの ID（フラッシュの固有の ID。XIP を止めて 1 回読む） | `N-board/src/rp2040_chipid.c`。NuttX の `rp2040_flash_cmd()` の上 | Pico SDK の `pico_unique_id`（Zephyr 版は hwinfo のドライバー） |
| USB デバイスのスタック、コンソールの経路 | `cdcacm`、`N-board/src/rp2040_usbserial.c`。Telnet は `nuttx/apps/init/rp2040_telnet.c` | Pico SDK の `stdio_usb`（TinyUSB）。コンソールの経路 `common/pico_sdk/drivers/shell/shell_serial_os_impl.h`、`shell_stdio.c`、`common/pico_sdk/drivers/libc/stdio_wrap.c`。Telnet はシェルのエンジンのもの |
| ヒープ | NuttX のもの | カーネルには無い（`configSUPPORT_DYNAMIC_ALLOCATION` 0。カーネルのオブジェクトはすべて静的）。newlib の `malloc()` は `.bss` の終わりから |
| OS 自身のシェル | nsh（セッションは `rp2040_init.c`） | 無い。共有のエンジン `common/compat/shell_engine`（コンソール、シリアルのセッション、Telnet）を `common/pico_sdk/drivers/shell` のコンソールの経路の上で使う |
| RAM ログに入る OS 自身のログ | `syslog()`: syslog のチャネル、`nuttx/drivers/log/log_ram_syslog.c` | なし（FreeRTOS のカーネルと Pico SDK はログを持たない） |
| 致命的なエラー | `CONFIG_BOARD_RESET_ON_ASSERT`、報告は RAM ログ | `common/pico_sdk/app/usb-console/src/fault.c` と `fault_os_impl.h`（入口: `panic()`、`isr_hardfault`、`configASSERT()` の `app_assert_failed()`、FreeRTOS のスタックのあふれのフック、`_exit()`。タスクの名前、`__uninitialized_ram` の記録、watchdog の scratch レジスター、SDK の `watchdog_reboot()`）と `common/compat/log/fault_record.c`（記録、`fault_record_set_frame()`、`fault_record_take()`） |
| ビルドの仕組みと構成 | Make、Kconfig、`defconfig`、`nuttx/patches` | CMake（共有の `common/pico_sdk/app/usb-console/usb-console.cmake`・`common/pico_sdk/drivers/*/*.cmake` の上の `freertos/app/usb-console/CMakeLists.txt`）、`FreeRTOSConfig.h` |

## 7. 例: IMU だけの構成

Wi-Fi・MQTT・時刻同期なしで、センサーのデータをシェル（`imu`、`sensor`）で見る最小の構成です。NuttX 版を
`CONFIG_NINA` なし（NuttX の `CONFIG_NET_USRSOCK` も外す）でビルドしたときにコンパイルされるファイルから挙げます。

| 項目 | 中身 |
|---|---|
| オプション | `CONFIG_KV_STORE`、`CONFIG_APP_IMU`、`CONFIG_LSM6DSO_STREAM`（N は `CONFIG_LSM6DSO_LOCAL` も）、任意で `CONFIG_APP_SENSOR`（`sensor` コマンド）。定義しない: `CONFIG_IMU_RAW_MQTT`、`CONFIG_TIME_SYNC`、`CONFIG_WIFI_CREDENTIALS`、`CONFIG_WIFI_PROFILE`、`CONFIG_WIFI_NINA_LOCAL`、`CONFIG_NET_ZPERF` |
| `common/imu` | `app_main.c`、`app_init.c`、`imu_stream.c`、`imu_raw_batch.c`、`imu_raw_packet.c`、`imu_raw_queue.c`、`imu_raw_time.c`、`imu_tick_clock.c`、`imu_cal.c`、`imu_cal_codec.c`、`kv_store.c`。送信と時刻同期の代わりに `imu_raw_mqtt_null.c`（バッチはすべて、繋がっていないときと同じく捨てて数える）と `time_sync_null.c`（単調な時計だけ。サーバーの時刻なし） |
| `common/compat` | `zephyr_init.c`、`zephyr_work.c`、`zephyr_rtio.c`、`zephyr_rtio_workq.c`、`zephyr_mem_blocks.c`、`zephyr_device.c`、`zephyr_gpio.c`、`zephyr_sensor_clock.c`、`sensor_default_rtio.c`、`sensor_shell_stream.c`（`CONFIG_APP_SENSOR`）、`zephyr_flash_kv.c`、`kv_store_wipe.c`、`lib/utils/hex.c`、settings と NVS の写し（`subsys/settings/src/settings.c`・`settings_store.c`・`settings_init.c`・`settings_nvs.c`、`subsys/kvss/nvs/nvs.c`、`subsys/crc/crc8_sw.c`）。ログ `log/log_ram.c`・`log_ring.c` |
| ドライバー | `common/drivers/i2c_dw/i2c_dw.c` と `port/<OS>/` の DMA、`common/drivers/lsm6dso` のコア 4 つと `port/zephyr/` の 5 つ（5 節）、`common/drivers/stmemsc/stmemsc_i2c.c`・`stmemsc_mdelay.c`、ST の `lsm6dso_reg.c`、`common/shell/sensor_shell.c`（`CONFIG_APP_SENSOR`）、任意で `common/shell/i2c_shell.c`（`i2c` コマンド。Zephyr 版のツリーにパッチ 0004 を当てたファイルの写しで、OS の部分は無く、互換層の `<zephyr/drivers/i2c.h>`（`i2c_get_config()`、`i2c_recover_bus()` などを含む）と `shell_hexdump()` の上で動く）、`common/chip/rp2040/rp2040_flash_kv.c` |
| シェル | `common/shell/zshell.c`、`subcmd.c`（任意で `i2c_dma_shell.c`）。自分のシェルを持たない OS は `common/compat/shell_engine` のうち Telnet 以外 |
| OS の側 | カーネルの部分（2 節）、ログ、KV のフラッシュ、GPIO バンク 0（INT1）、CPU 負荷、優先度、hwinfo（セッションの ID）、I2C の `port/<OS>/`、`lsm6dso_instance.h` とセンサーの開始、コンソール、起動（4.6 のうちネットワークの段階を除く）。Pico SDK の上では、カーネルの部分・優先度・tick・I2C の `i2c_dw_os_impl.h`・アプリの OS の部分のほかは共有（冒頭の「Pico SDK の上の OS」） |
| 要らないもの | `common/compat/net_offload`、`common/compat/libc_net`（`imu_raw_mqtt_null.c` の `<arpa/inet.h>` を C ライブラリーが持っていれば。newlib の F は持たない）、`common/compat/zephyr_heap.c`、NINA のドライバー（中継モードの `CONFIG_NINA_PASSTHROUGH` は別に選べる）、`common/imu/mqtt_min*.c`・`imu_raw_status.c`・`time_sync.c`・`time_offset.c`・`wifi_profile.c` |
