# 共有のコードと OS の境界

[English](os-boundary.md) | 日本語

版が共有のソース（`common/`）に渡すものの索引です。今のコードのとおりに書きます。各行に、宣言しているヘッダーと、
各版の実装のファイルを挙げます。「N」は NuttX 版、「F」は Pico SDK + FreeRTOS 版です。Zephyr 版は `common/` を
Zephyr そのものに対してビルドし、`common/compat` を使いません。Zephyr 版は、自分のファイルを持つところにだけ出てきます。

`nuttx/boards/arduino-nano-rp2040-connect/` の下のパスは `N-board/` と略します。

## 1. 版の構成

| 部分 | 場所 | 中身 |
|---|---|---|
| アプリ | `common/imu` | IMU サービス、MQTT の送信、時刻同期、Wi-Fi のプロファイル、KV ストア、校正 |
| ドライバー | `common/drivers/<ドライバー>/` | OS に依存しないコア、`port/` の境界のヘッダー、`port/<OS>/` のつなぎ（5 節） |
| シェルのコマンド | `common/shell` | `zshell.c`（Zephyr のシェルのコマンド処理）、`subcmd.c`、`nina_shell.c`、`i2c_dma_shell.c`、`dmesg_shell.c`（F だけ。N は `nuttx/apps/dmesg`） |
| ログ | `common/log` | RAM ログと `LOG_*` マクロの関数（`log_ram.c`。OS の部分 `log_ram_os.h` の上。3 節）、そのリング（`log_ring.c`）、`LOG_*` マクロ（`app_log.h`）、異常停止の記録の文面（`fault_record.c`） |
| 互換層の共有部分 | `common/compat` | Zephyr のヘッダー（`common/compat/zephyr/`）、ワークアイテム、ワークキューのスレッドと互換層の開始（`zephyr_init.c`）、RTIO、gpio0 のデバイス、net_mgmt、Wi-Fi の要求、KV のフラッシュのデバイス、Zephyr の NVS・settings・getopt・Wi-Fi の資格情報・zperf の写し |
| 互換層のネットワークの部分（ネットワーク層を持たない OS のため） | `common/compat/net_offload` | Zephyr のソケット層（`subsys/net/lib/sockets`・`lib/os/zvfs` を縮めた写し）、ネットワークのインターフェース 1 つ（`zephyr_net_if.c`）とその開始（`zephyr_net_init.c`）、`k_heap`（`zephyr_heap.c`）、それらのヘッダーは専用のインクルードのルート `include/`（`zephyr/net/net_if.h`・`net_ip.h`・`socket.h`・`socket_offload.h`、`zephyr/sys/fdtable.h`、`zephyr_net_init.h`、`zephyr_heap.h`）。使うのは F だけ |
| POSIX のネットワークのヘッダー（それを持たない C ライブラリーのため） | `common/compat/libc_net` | `include/`（`arpa/inet.h`、`netdb.h`、`netinet/in.h`、`netinet/tcp.h`、`poll.h`、`sys/socket.h`、`sys/uio.h`）、`inet_pton()`・`inet_ntop()`（`inet.c`）。使うのは F だけ |
| 基本の操作だけを持つ OS のための互換層のカーネルの部分 | `common/compat/kernel_generic` | `k_sem`、`k_poll()`、タイムアウトの一覧、スリープ、スレッドの起動（`zephyr_kernel.c`）、`k_mutex`（`zephyr_mutex.c`）、`<zephyr/kernel.h>`・`<zephyr/spinlock.h>` の OS の部分（`zephyr_os.h`・`zephyr_os_spinlock.h`）、tick の入口（`zephyr_compat_tick()`、`zephyr_compat_os.h`）、基本の操作の上の自己診断の項目（`zephyr_selftest_os.h`）。すべて `zephyr_kernel_os.h` で宣言した基本の操作（2 節）の上。使うのは F だけ |
| 互換層の自己診断（OS の部分の受け入れ試験） | `common/compat/selftest` | `zephyr_selftest.c`（22 項目）、境界の宣言 `zephyr_selftest.h`（3 節）。使うのは F の `-DSELFTEST=ON` のビルドと F のホストテストだけ |
| 自分のシェルを持たない OS のためのシェル | `common/compat/shell_engine` | 行編集（`cli.c`）、`Z_SHELL_CMD_ROOT_GLOBAL` のルートのコマンドの表と実行（`nano_shell.c`。`nano_shell_cmds[]` はアプリが用意する）、Zephyr の単語の分割（`shell_argv.c`）。使うのは F だけ |
| どの OS も持たない RP2040 のコード | `common/chip/rp2040` | `include/rp2040_timer_us.h`（TIMER のレジスターと 64 ビットの読み出し `rp2040_timer_us()`。必ずインライン。3 版とも）、`include/rp2040_watchdog_reboot.h`（`rp2040_watchdog_reboot()`、レジスターを直接書く watchdog の再起動。Zephyr 版と N）。ヘッダーだけで、各版が `include/` を include パスに入れる（Zephyr はアプリの `CMakeLists.txt`、N は `N-board/scripts/Make.defs` で全部のソースに、F はアプリの `CMakeLists.txt`） |
| ホストテスト | `common/tests` | テストと偽物。`common/tests/zephyr_fakes` はカーネルの OS 部分のホスト用の実装 |
| Zephyr 版 | `zephyr/app/usb-console` | `main()`、`log_ram.c`、`kv_store_os.h`、`wifi_profile_os.h`。上流の Zephyr と、`zephyr/manifest/zephyr/patches` のパッチ |
| N: ボード | `N-board/` | 起動、tickless のタイマー、アイドルループと CPU 負荷、GPIO バンク 0、KV のフラッシュ、USB シリアル、BOOTSEL（`include/rp2040_restart.h`） |
| N: 互換層の OS 部分 | `nuttx/drivers/compat` | `zephyr_os*.h`、カーネル、ミューテックス、`struct i2c_master_s` の上の I2C、net_if、hwinfo、プレリュード、OS ごとの `<zephyr/...>` ヘッダー |
| N: 共有の OS ヘッダー | `nuttx/drivers/include` | `app_log_os.h`、`log_ram_os_impl.h`、`priorities_os.h` |
| N: ログ | `nuttx/drivers/log` | `log_ram_syslog.c`（RAM ログを syslog のチャネルにする）、`Make.defs` |
| N: ドライバー | `nuttx/drivers/<ドライバー>/` | 各ドライバーの `Make.defs`、`Kconfig` |
| N: アプリ | `nuttx/apps/*` | init タスク（`init`）、コマンドごとの builtin アプリ、シェルの OS 部分（`subcmd`）、Wi-Fi とセンサーの部分 |
| N: NuttX の変更 | `nuttx/patches` | NuttX と nuttx-apps へのローカルのパッチ |
| F: アプリ | `freertos/app/usb-console` | `main()`、`FreeRTOSConfig.h`、tick のフック、アイドルと CPU 負荷、`os_clock.h`、`priorities_os.h`、コマンドの表、異常停止の処理 |
| F: 互換層の OS 部分 | `freertos/drivers/compat` | 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）、`zephyr_os_atomic.h`、実時間の時計、ソケットの登録のためのリンカースクリプトの部分、`rp2040/`（ポートのヘッダー、GPIO バンク 0、hwinfo、自己診断のボードの部分）、自己診断の FreeRTOS の部分（`zephyr_selftest_os_impl.h`）、プレリュード |
| F: シェル | `freertos/drivers/shell` | コンソール、コンソールごとの stdio、getopt の状態、シェルの OS 部分 |
| F: C ライブラリーの追加 | `freertos/drivers/libc` | stdout・stderr の置き換え |
| F: そのほかの OS 部分 | `freertos/drivers/{include,kv,telnet,wifi,sensor_shell,imu,zperf}` | `app_log_os.h`・`log_ram_os_impl.h`、KV のフラッシュ、Telnet、Wi-Fi の OS ヘッダー、センサーのシェルの OS 部分、ソースごとのプレリュード、ビルドのファイル |
| F: ドライバー | `freertos/drivers/<ドライバー>/*.cmake` | 各ドライバーのソースとオプション |

## 2. 互換層のカーネルの部分

宣言は `common/compat/zephyr/kernel.h` で、このヘッダーは最初に `"zephyr_os.h"` を読みます。OS は次のどちらかの形で
用意します。

- **直接に実装する**（N。`nuttx/drivers/compat/`）: 自分の `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h` と、
  2.2 の関数を OS 自身のオブジェクト（セマフォ、watchdog、カーネルスレッド）の上に書きます。
- **基本の操作の上に置く**（F）: 汎用の実装（`common/compat/kernel_generic/`: `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_kernel.c`・
  `zephyr_mutex.c`・`zephyr_os_internal.h`・`zephyr_compat_os.h` と、自己診断の `zephyr_selftest_os.h`）が、2.1・2.2 と
  スピンロックを、`common/compat/kernel_generic/zephyr_kernel_os.h` で宣言した基本の操作（2.5）の上に Zephyr の規則で実装します。OS が書くのは、
  その基本の操作（`zephyr_kernel_os_impl.h`。static inline）、`zephyr_os_atomic.h`、tick の割り込みから 1 tick に 1 回
  `zephyr_compat_tick()` を呼ぶ tick のフック、自己診断のための `zephyr_selftest_os_impl.h` です。

### 2.1 `zephyr_os.h` が用意するもの（15 項目、任意 1 項目）

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
| `k_current_get()` | 呼んだスレッド | `gettid()` | `z_os_thread_self()` |
| `k_yield()` | 同じ優先度の次の実行可能なスレッドに譲る。無ければ続ける | `sched_yield()` | `z_os_yield()` |
| `K_PRIO_PREEMPT(x)` | Zephyr の優先度 x の OS の優先度。順序を保つ | `APP_PRIO_FROM_ZEPHYR`（118 − x） | `APP_PRIO_FROM_ZEPHYR`（F は 20 − x） |
| `k_cycle_get_32()`、`k_cycle_get_64()` | 1 MHz のカウンター | `rp2040_timer_us()` | `z_os_clock_us()`（F は `rp2040_timer_us()`） |
| `Z_OS_ASSERT(test)` | OS のアサーション。ワークキューのスレッドが起きなかったとき用 | `DEBUGASSERT()`（imu の構成では何もしない） | OS のもの（F は `configASSERT()`: 異常停止の記録と再起動） |
| `Z_SEM_TAKE_FOREVER(sem)`（任意） | `k_sem_take(sem, K_FOREVER)` を OS の短い経路で。ワークキューの待ち | `nxsem_wait_uninterruptible()`（`k_sem_take()` と同じ処理を、呼び出しなしで） | 定義しない: `k_sem_take()`（`kernel.h` の既定） |

汎用の `zephyr_os.h` は `common/compat/net_offload/include/zephyr_heap.h` も読みます: `struct k_heap`、`K_HEAP_DEFINE()`、`k_heap_alloc()`、`k_heap_free()`（4 節）。

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
| `z_thread_static_start(thread)` | `*thread` の名前・静的なスタック・優先度の新しいスレッドで `entry(p1, p2, p3)` を動かし（`*thread` はスレッドが動くあいだ有効）、`entry` が戻ったら終える。`*tid` を入れる。0 か負の errno。互換層のスレッドはすべてこれで始める |
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
| `zephyr_compat_initialize()` を呼ぶところ | `N-board/src/rp2040_bringup.c` | スケジューラーの前の `main()`（`freertos/app/usb-console/src/main.c`） |
| 同じ優先度の選び方 | NuttX のもの | FreeRTOS の、切り替えのたびの順番回し |

F の列は、tick、同じ優先度の選び方、起動、アサーション（これらは OS のもの）を除き、汎用の実装の上のどの OS にも当てはまります。

### 2.4 スピンロックと atomic

| ヘッダー（共有） | 読むもの | OS が用意するもの | N | 汎用（F） |
|---|---|---|---|---|
| `common/compat/zephyr/spinlock.h` | `"zephyr_os_spinlock.h"` | `struct k_spinlock`、`k_spinlock_key_t`、`k_spin_lock()`、`k_spin_unlock()`: 割り込みを止め、解除まで切り替えない。スレッドと割り込みで。入れ子にできる | `sched_lock()` + `up_irq_save()` | スレッドでは `z_os_task_lock()`（F は `taskENTER_CRITICAL()`）、割り込みでは `z_os_isr_lock()`（F は `taskENTER_CRITICAL_FROM_ISR()`） |
| `common/compat/zephyr/sys/atomic.h` | `"zephyr_os_atomic.h"` | atomic の名前を定義する OS のヘッダーを先に読む | `<nuttx/atomic.h>` を読み、その `atomic_*` マクロを消す | OS のもの（F は空。`freertos/drivers/compat/zephyr_os_atomic.h`） |

atomic の操作そのものは共有です（`k_spin_lock()` の上）。

### 2.5 汎用の実装の基本の操作（`common/compat/kernel_generic/zephyr_kernel_os.h`）

`zephyr_kernel_os.h` は OS の `"zephyr_kernel_os_impl.h"` を読み、OS が定義すべきものを宣言します（足りないものや違うものは
コンパイルが通らない）。すべて static inline です。`k_sem_give()`・`k_sem_take()`・`k_poll()`・`k_spin_lock()` と tick の
経路にあるからです（関数ポインターを通さない）。F のものは `freertos/drivers/compat/zephyr_kernel_os_impl.h` で、ポートに
よるものは `zephyr_os_port.h`（ボードは `freertos/drivers/compat/rp2040/`、ホストは `freertos/tests/compat/`）にあります。

型と定数（9 個）: `z_os_thread_t`（`TaskHandle_t`）、`z_os_prio_t`（大きいほど急ぐ。`UBaseType_t`）、`z_os_ticks_t`（符号なし。
`TickType_t`）、`Z_OS_TICKS_FOREVER`（その最大の値で、終わりのない待ち。`portMAX_DELAY`）、`Z_OS_TICK_HZ`（`configTICK_RATE_HZ`）、
`z_os_woken_t`（最初は 0。`BaseType_t`）、`Z_OS_THREAD_NAME_SIZE`（`configMAX_TASK_NAME_LEN`）、`struct k_mutex` と
`Z_MUTEX_INITIALIZER()`、`Z_OS_ASSERT()`。

関数（23 個）。「ロック中」は `k_spin_lock()` の中、「ISR」は tick のフックを含みます。

| 基本の操作 | 意味 | F |
|---|---|---|
| `z_os_in_isr()` | ISR の中なら真。どの文脈からでも | IPSR ≠ 0（ホストは tick のフックの深さ） |
| `z_os_task_lock()`、`z_os_task_unlock()` | スレッドのロック: 解除まで ISR もほかのスレッドも走らない。入れ子にできる。中で行う起こしはロックを保ち、その切り替えはいちばん外の解除で起きる | `taskENTER_CRITICAL()`、`taskEXIT_CRITICAL()`（ホストはスケジューラーも止める） |
| `z_os_isr_lock()`、`z_os_isr_unlock(state)` | ISR のロック: マスクを保存して戻す。状態は `UINT32_MAX` にならない | `taskENTER_CRITICAL_FROM_ISR()`、`taskEXIT_CRITICAL_FROM_ISR()` |
| `z_os_thread_self()` | 呼んだスレッド。スレッドだけ | `xTaskGetCurrentTaskHandle()` |
| `z_os_thread_prio_self()` | その今の優先度（継承を含む）。スレッドだけ | `uxTaskPriorityGet(NULL)` |
| `z_os_thread_start(fn, name, stack, size, prio, arg, tid)` | 静的なスタックの新しいスレッド。走れるようになる前に `*tid` を入れる。スレッドかスケジューラーの開始前から。0、−EINVAL、−ENOMEM | `xTaskCreateStatic()`。制御ブロックは FreeRTOS のヒープから。スケジューラーが動いていればまわりを `vTaskSuspendAll()` |
| `z_os_thread_exit()` | 呼んだスレッドを終える | `vTaskDelete(NULL)` |
| `z_os_thread_name(tid)` | スレッドの書き込める名前の配列（`Z_OS_THREAD_NAME_SIZE` バイト）。OS が表示するもの | `pcTaskGetName()` |
| `z_os_yield()` | 同じ優先度の次のスレッドに譲る。無ければ続ける。スレッド、ロックの外 | `taskYIELD()` |
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

F のポートのヘッダーには、tick のフックの `z_os_tick_now_us32()`・`z_os_tick_pend()`（失われた tick の追いつき、
`freertos/app/usb-console/src/tick.c`）もあります。これらは汎用の部分の基本の操作ではありません。

## 3. そのほかの宣言された境界

| 境界 | 宣言 | 項目 | 呼ぶ側 | N | F |
|---|---|---|---|---|---|
| シェルのバックエンド | `common/shell/zshell_os.h` | `zshell_os_backend_get()`、`zshell_os_cmd_enter()`、`zshell_os_cmd_exit()`、`z_shell_backend_vfprintf()`。任意で `ZSHELL_OS_ACTIVE_CMD` と `zshell_os_active_cmd_set()`・`_get()` | `common/shell/zshell.c` | `nuttx/apps/subcmd/zshell_os_impl.h`（`ZSHELL_OS_ACTIVE_CMD` なし） | `freertos/drivers/shell/zshell_os_impl.h`（定義あり） |
| `struct shell` | `common/compat/zephyr/shell/shell.h` が読む | バックエンド 1 つ。コマンドより長く生きる | シェルのコマンド | `nuttx/drivers/compat/zephyr_os_shell.h`（端末のパス、コマンドの pid） | `freertos/drivers/shell/zephyr_os_shell.h`（コンソール、実行中のコマンド） |
| ログの OS 部分 | `common/log/app_log.h` | `app_log_os.h`（OS のヘッダーを先に、任意の `CONFIG_LOG_DEFAULT_LEVEL`・`CONFIG_LOG_MAX_LEVEL`）、`app_log_printf()`、`app_log_vprintf()`、`app_log_ratelimit_pass()` | `LOG_*()` マクロ | `nuttx/drivers/include/app_log_os.h`。関数は共有の `common/log/log_ram.c` | `freertos/drivers/include/app_log_os.h`。関数は共有（同じ） |
| RAM ログ | `common/log/log_ram.h` | `log_ram_initialize()`、`log_ram_store()`（OS 自身のログ。そのままのバイト）、`log_ram_piece()` | 起動、OS のログ、`dmesg` | 共有の `common/log/log_ram.c`。呼ぶのは `N-board/src/rp2040_boardinitialize.c`、`nuttx/drivers/log/log_ram_syslog.c`、`nuttx/apps/dmesg/dmesg_main.c` | 共有（同じ）。呼ぶのは `main()`、`common/shell/dmesg_shell.c`（`log_ram_store()` は使わない） |
| RAM ログの OS 部分 | `common/log/log_ram_os.h`（`log_ram.c` が最初に `"log_ram_os_impl.h"` を読む） | `LOG_RAM_OS_RETAINED(name)`（起動のコードが触らない RAM にリングを置く）、型 `log_ram_os_irqstate_t`。inline の `log_ram_os_irq_save()`・`_irq_restore()`（ロック。割り込みを止める。どの文脈からでも）、`_uptime_us()`（時刻の印と頻度制限）、`_attach()`（OS 自身のログにする） | `common/log/log_ram.c` | `nuttx/drivers/include/log_ram_os_impl.h`: `.noinit`、`up_irq_save()`、`rp2040_timer_us()`、syslog のチャネルを登録（`nuttx/drivers/log/log_ram_syslog.c`） | `freertos/drivers/include/log_ram_os_impl.h`: `__uninitialized_ram`、`save_and_disable_interrupts()`、`os_clock.h` の `rp2040_timer_us()`、登録するものなし。ホストは `common/tests/log_ram_os_host/` |
| KV のフラッシュ | `common/compat/nano_flash_kv.h` | `nano_flash_kv_os_ready()`、`_read()`、`_program()`（256 バイトのページ）、`_erase()`（4 KiB のセクター）。範囲は OS がもう一度確かめる | `common/compat/zephyr_flash_kv.c` | `N-board/src/nano_flash_kv.c` | `freertos/drivers/kv/nano_flash_kv.c` |
| KV の消去のコマンド | `common/compat/kv_store_os.h` | `kv_store_wipe()`、`kv_store_cmd_wipe()`、`KV_STORE_OS_COMMANDS` | `common/imu/kv_store.c` | 共有: `common/compat/kv_store_wipe.c` | 共有: 同じ |
| GPIO バンク 0 | `common/compat/gpio_os.h` | OS: `gpio_os_configure_input()`、`gpio_os_get_all()`、`gpio_os_irq_configure()`。互換層: `gpio_os_irq_handler()`（バンクの割り込みから呼ぶ） | `common/compat/zephyr_gpio.c`。N では `common/drivers/nina/port/nuttx/nina_os.c` も | `N-board/src/rp2040_gpio_os.c` | `freertos/drivers/compat/rp2040/gpio_os_rp2040.c` |
| CPU 負荷 | `common/compat/cpu_load.h`（計算も） | `cpu_load_get(reset)`。0.1% 単位、TIMER で測ったアイドルの区間から | `common/imu/imu_stream.c` | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| 優先度とスタック | `common/compat/priorities.h` | OS: `APP_PRIO_FROM_ZEPHYR(p)` と OS 自身のスレッド。共有: `APP_PRIO_*`・`APP_STACK_*` の 15 組 | 互換層、つなぎ、起動 | `nuttx/drivers/include/priorities_os.h`（118 − p。`NINA_EVENT`、`NINA_PT`） | `freertos/app/usb-console/src/priorities_os.h`（20 − p。`TIMER_SVC`、`IDLE`、`HEARTBEAT`） |
| net_mgmt のスレッド | `common/compat/zephyr/net/net_mgmt.h` | `net_mgmt_event_init()` が "net_mgmt" を `z_thread_static_start()` で起こす。OS のファイルは無い | N `common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`、F `common/compat/net_offload/zephyr_net_init.c` | 共有の `common/compat/zephyr_net_mgmt.c` | 同じ |
| RTIO のワークキュー | `common/compat/zephyr/rtio/work.h` | `rtio_workq_initialize()` が "RTIO WQ 0" を `z_thread_static_start()` で起こす。OS のファイルは無い | `zephyr_compat_initialize()` | 共有の `common/compat/zephyr_rtio_workq.c` | 同じ |
| Wi-Fi のプロファイル | `common/imu/wifi_profile.c` が読む | `extern k_tid_t wifi_reconnect_tid;` | `common/imu/wifi_profile.c` | `nuttx/apps/wifi/wifi_profile_os.h` | `freertos/drivers/wifi/wifi_profile_os.h` |
| センサーのシェル | `common/drivers/sensor_shell/sensor_shell.c` と `common/compat/sensor_shell_stream.c` が最初に読む | `CONFIG_SENSOR_SHELL_STREAM`、`_THREAD_STACK_SIZE`、`_TRIG_PRINT_TIMEOUT_MS`、`_MAX_TRIGGER_DEVICES`。`sensor_shell_os_get_arm()`、`sensor_shell_os_get_wait()`、`sensor_shell_os_processing_callback` | その 2 つのファイル | `nuttx/apps/sensor/sensor_shell_os.h`、`sensor_shell_nuttx.h`、`sensor_shell_nuttx.c` | `freertos/drivers/sensor_shell/sensor_shell_os.h`、`sensor_shell_os.c` |
| 互換層の開始 | `common/compat/zephyr_compat.h` | `zephyr_compat_initialize()`、`K_THREAD_DEFINE_START(name)` | 起動 | 共有の `common/compat/zephyr_init.c`（2 節） | 同じ |
| 互換層の自己診断、OS の部分 | `common/compat/selftest/zephyr_selftest.h`（`zephyr_selftest.c` が `<zephyr/kernel.h>` のあとに `"zephyr_selftest_os.h"` を読む） | 型 `zephyr_selftest_os_prio_t`、`ZEPHYR_SELFTEST_OS_NAME_MAX`、`zephyr_selftest_os_tick()`、`_tick_sync()`、`_thread_create()`、`_thread_exit()`、`_prio_get()`、`_prio_self()`、`_name_get()`、`_current()`、`_in_isr()`、`_sched_lock()`、`_sched_unlock()` | `common/compat/selftest/zephyr_selftest.c` | — （ビルドしない） | 基本の操作の上の `common/compat/kernel_generic/zephyr_selftest_os.h`（`_tick_sync()`・`_thread_exit()`・`_prio_self()`・`_name_get()`・`_in_isr()`、型と名前の長さ）。残りは `freertos/drivers/compat/zephyr_selftest_os_impl.h`（FreeRTOS のタスク、ボードとホストで同じ） |
| 互換層の自己診断、ボードの部分 | 同じ | `zephyr_selftest_port_alarm()`、`_quiet()`、`_realtime()`、`_mask_irqs()` | 同じ | — | ボード `freertos/drivers/compat/rp2040/zephyr_selftest_port.c`、ホスト `freertos/tests/compat/main.c` |
| 汎用のカーネルの部分の tick | `common/compat/kernel_generic/zephyr_compat_os.h` | `zephyr_compat_tick()`: 1 tick に 1 回、tick の割り込みから | F の tick のフック | — | `common/compat/kernel_generic/zephyr_kernel.c` |
| ネットワーク層を持たない OS のネットワークの部分の開始 | `common/compat/net_offload/include/zephyr_net_init.h` | `zephyr_net_initialize()` | F の `main_main()` | —（`nina_net_initialize()`） | 共有の `common/compat/net_offload/zephyr_net_init.c` |

Zephyr 版は自分の `zephyr/app/usb-console/src/kv_store_os.h` と `wifi_profile_os.h` を持ちます。

## 4. 共有の宣言なしに各版が用意するもの

### 4.1 OS ごとに用意する `<zephyr/...>` ヘッダー

インクルードパスでは OS のディレクトリーが先で、ほかは `common/compat/zephyr/` にあります。F のネットワークのヘッダーは
`common/compat/net_offload/include` にあり、F はそれを OS のディレクトリーのあと、`common/compat` の前に置きます。N のインクルードパスには
ありません。

| ヘッダー | N | F |
|---|---|---|
| `zephyr/drivers/i2c.h` | `nuttx/drivers/compat/zephyr/drivers/i2c.h`。`struct i2c_master_s` の上（`zephyr_i2c.c`） | `freertos/drivers/compat/zephyr/drivers/i2c.h`（Zephyr の API の表） |
| `zephyr/net/net_if.h` | `struct net_driver_s` の上（`zephyr_net_if.c`） | `common/compat/net_offload`: NINA のインターフェース 1 つ（`zephyr_net_if.c`） |
| `zephyr/net/net_ip.h` | —（`common/compat` のものを使う） | `common/compat/net_offload`。`common/compat` のものに代わる。POSIX の型に `net_*` の名前を付ける |
| `zephyr/net/socket.h` | `zsock_*()` は NuttX の `socket()`・`poll()` など（usrsock） | `common/compat/net_offload`: Zephyr のソケット層、`subsys/net/lib/sockets/sockets.c`、`socket_offload.c`、`getaddrinfo.c` |
| `zephyr/net/socket_offload.h` | — | `common/compat/net_offload`: オフロードした名前解決 |
| `zephyr/sys/fdtable.h` | — | `common/compat/net_offload`: 記述子の表、`lib/os/zvfs/zvfs_fdtable.c`、`zvfs_poll.c` |
| `zephyr/sys/__assert.h` | `DEBUGASSERT()` | 何もしない（`CONFIG_ASSERT` が無いのと同じ） |
| `zephyr/sys/clock.h` | `clock_gettime()`・`clock_settime()` の上のインライン | `zephyr_sys_clock.c`（TIMER と基準） |

### 4.2 関数とオブジェクト

| 項目 | 宣言 | N | F |
|---|---|---|---|
| `rp2040_timer_us()`: 起動からの µs、TIMER、どの文脈からでも | `common/chip/rp2040/include/rp2040_timer_us.h`（共有。Zephyr 版の `common/imu/time_sync.c` も） | `nuttx/drivers/compat/zephyr_os.h`、`nuttx/drivers/include/log_ram_os_impl.h`、ボードが読む | `freertos/app/usb-console/src/os_clock.h` が読む |
| `rp2040_watchdog_reboot()`: watchdog での再起動。scratch 0..3 は残る | `common/chip/rp2040/include/rp2040_watchdog_reboot.h`（共有。Zephyr 版の `common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c` も） | `N-board/src/rp2040_restart.c`（`rp2040_reset_usb_boot()` の代わりの道）、`common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c` | 使わない。Pico SDK の `watchdog_reboot()` |
| `k_heap` | `common/compat/net_offload/include/zephyr_heap.h`（F の `zephyr_os.h` が読む） | ビルドしない | `common/compat/net_offload/zephyr_heap.c`。使うのは `common/drivers/nina/port/zephyr/nina_socket.c` |
| `zephyr_compat_tick()` | `common/compat/kernel_generic/zephyr_compat_os.h`（汎用のカーネルの部分） | —（wdog） | `vApplicationTickHook()`（`freertos/app/usb-console/src/tick.c`）が呼ぶ |
| `zephyr_net_initialize()` | `common/compat/net_offload/include/zephyr_net_init.h` | —（`rp2040_bringup.c` の `nina_net_initialize()`、`common/drivers/nina/port/nuttx/nina_net.h`） | `common/compat/net_offload/zephyr_net_init.c` |
| `hwinfo_get_device_id()` | `common/compat/zephyr/drivers/hwinfo.h` | `nuttx/drivers/compat/zephyr_hwinfo.c` | `freertos/drivers/compat/rp2040/zephyr_hwinfo.c` |
| `nano_flash_kv_initialize()` | 各版 | `N-board/src/rp2040_pico.h`、`nano_flash_kv.c`。`rp2040_bringup.c` が呼ぶ | `freertos/drivers/kv/nano_flash_kv_os.h`、`nano_flash_kv.c`。`main()` が呼ぶ |

### 4.3 `DEVICE_DT_GET()` の向こうのデバイス

`DEVICE_DT_GET(DT_NODELABEL(x))` は `&nano_device_x` です（`common/compat/zephyr/device.h`）。作るのは `DEVICE_DT_DEFINE()` です。
シェルに見せるデバイスの一覧は `common/compat/zephyr_device.c` に手で書いてあります。

| ノード | N | F |
|---|---|---|
| `gpio0` | `common/compat/zephyr_gpio.c` | 同じ |
| `i2c0` | `N-board/src/rp2040_lsm6dso.c`。NuttX の I2C マスターに結び付ける（`zephyr_i2c.c`） | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `i2c1` | Zephyr のデバイスは無い（`struct i2c_master_s` だけ） | `common/drivers/i2c_dw/port/freertos/i2c_dw_instance.h` |
| `lsm6dsox` | ドライバーのつなぎ、`common/drivers/lsm6dso/port/` | 同じ |
| フラッシュのデバイス | `common/compat/zephyr_flash_kv.c`（`z_flash_kv_device`） | 同じ |

### 4.4 シェルのコマンド: 登録、getopt、stdio

| 点 | N | F |
|---|---|---|
| 登録 | コマンドごとの builtin アプリ。`SHELL_CMD_REGISTER()` が `main()`（`Z_SHELL_CMD_NAMED_MAIN` なら `z_shell_main_<名前>()`）を作る | `Z_SHELL_CMD_ROOT_GLOBAL`: 大域のルートの要素を `nano_shell_cmds[]`（`freertos/app/usb-console/src/commands.c`）に並べ、`common/compat/shell_engine/nano_shell.c` が動かす |
| `SHELL_SUBCMD_ADD()` | 大域の要素。親のソースが手で並べる | 同じ |
| getopt | Zephyr の getopt はビルドしない。`nuttx/apps/wifi/wifi_credentials_shell.c` が `sys_getopt_*` を `getopt_long()` に読み替える | `common/compat/lib/utils/getopt` の写し。状態はセッションごと（`freertos/drivers/shell/getopt_state.c` の `sys_getopt_state_get()`、`z_getopt_global_state_update()`）。コマンドの前に `sys_getopt_init()` で戻す |
| `zshell.c` の stdio: `printf()`、`fprintf(stderr)`、`fflush(stdout)` がコマンドの端末に届くこと。1 行あたり約 1.2 KB の `calloc()` | コマンドごとのタスクが自分の stdio を持つ | `freertos/drivers/shell/nano_stdio.c`（呼んだタスクのコンソール）、`freertos/drivers/libc/stdio_wrap.c`（stdout・stderr の呼び出しの `--wrap`） |

### 4.5 `CONFIG_*` の値

| 出どころ | N | F |
|---|---|---|
| プレリュード（共有のソースに `-include`） | `nuttx/drivers/compat/common_prelude.h`: NuttX のヘッダーを先に、`CONFIG_SENSOR_LOG_LEVEL` 3、`CONFIG_LSM6DSO_STREAM` のとき `CONFIG_SENSOR_ASYNC_API` | `freertos/drivers/compat/common_prelude.h`: `CONFIG_SENSOR_LOG_LEVEL` 3、`CONFIG_NINA_NTP_SERVER` ""・`_PORT` 123・`_INTERVAL_MS` 2000、`CONFIG_NINA_TCP_MAX_IO` 3584、`CONFIG_NINA_SEND_RETRY_MS` 1、`CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` 40、`CONFIG_NINA_PASSTHROUGH` 1、ストリームのとき `CONFIG_SENSOR_ASYNC_API` |
| NINA と NTP のオプション | Kconfig: `nuttx/drivers/nina/Kconfig`、`nuttx/apps/imu/Kconfig` | プレリュード（上） |
| NVS と settings の写し | `common/compat/zephyr_kv_config.h`。`nuttx/apps/imu/Makefile` から `-include` | 同じ。`freertos/drivers/kv/kv.cmake` から |
| ビルドの定義 | Kconfig（`nuttx/drivers/*/Kconfig`、`nuttx/apps/*/Kconfig`、`N-board/Kconfig`）、`N-board/configs/imu/defconfig` | `compat.cmake`: `Z_SHELL_CMD_ROOT_GLOBAL`、`CONFIG_GETOPT_LONG=1`、`PICO_STDOUT_MUTEX=0`。`i2c_dw.cmake`: `CONFIG_I2C_DW_I2C0`、`_I2C1`。`lsm6dso.cmake`: `CONFIG_LSM6DSO_STREAM`。`wifi.cmake`: `CONFIG_WIFI_CREDENTIALS`。`nina.cmake`: 名前解決の `NINA_OS_DNS_OFFLOAD` |
| 共有のヘッダーの中 | `kernel.h`: `CONFIG_THREAD_NAME` 1。`zephyr/rtio/work.h`: `CONFIG_RTIO_WORKQ_POOL_ITEMS` 4 | 同じ |
| センサーのシェル | `sensor_shell_nuttx.h` | `sensor_shell_os.h`（同じ 4 つの値） |

### 4.6 スレッドと起動の順序

`K_THREAD_DEFINE()` のスレッドは自動では起きません。起動のコードが並べて起こします。

| スレッド（定義のファイル） | N | F |
|---|---|---|
| `nina_scan_tid`、`nina_connect_tid`、`nina_monitor_tid`（`common/drivers/nina/port/zephyr/nina_net.c`） | ビルドしない。`nina_net_start()` がコアのスレッドを起こす | `K_THREAD_DEFINE_START()` |
| `imu_stream_tid`（`common/imu/imu_stream.c`） | `K_THREAD_DEFINE_START()` | 同じ |
| `imu_raw_mqtt_tid`（`common/imu/imu_raw_mqtt.c`） | 同じ | 同じ |
| `time_sync_tid`（`common/imu/time_sync.c`） | 同じ | 同じ |
| `wifi_reconnect_tid`（`common/imu/wifi_profile.c`） | 同じ | 同じ |
| `sensor_shell_processing_tid`（`common/compat/sensor_shell_stream.c`） | 同じ | 同じ |
| `zperf_server_tid`（`common/compat/zperf_iperf3.c`） | ビルドしない | 同じ |

起動のファイル: Zephyr `zephyr/app/usb-console/src/main.c`。N `N-board/src/rp2040_boardinitialize.c`、
`N-board/src/rp2040_bringup.c`、`nuttx/apps/init/rp2040_init.c`。F `freertos/app/usb-console/src/main.c`
（`main()`、そのあとタスク `main_main()`）。

| 段階 | N | F |
|---|---|---|
| RAM ログ | `rp2040_boardinitialize()` の `log_ram_initialize()` | `main()` の最初 |
| NINA の中継モードの確認 | init タスクの `nina_passthrough_active()` | `main()` の `nina_passthrough_check()` |
| 互換層 | `rp2040_bringup()` の `zephyr_compat_initialize()` | `main()` の `zephyr_compat_initialize()` |
| I2C0、I2C1 | `rp2040_bringup()` の `i2c_dw_i2c0_initialize()`、`i2c_register()` | `main()` の `i2c_dw_i2c0_initialize()`、`_i2c1_` |
| LSM6DSOX | `rp2040_bringup()` の I2C0 のあと | `main_main()` の最初 |
| ネットワーク | `rp2040_bringup()` の `nina_net_initialize()` | `main_main()` の `zephyr_net_initialize()` |
| KV のフラッシュの範囲 | `rp2040_bringup()` の `nano_flash_kv_initialize()` | `main()` の `nano_flash_kv_initialize()` |
| Wi-Fi の資格情報 | init タスクの `wifi_credentials_init()` | `main_main()` の `wifi_credentials_init()`、`wifi_shell_init()` |
| スレッド | 上の表 | 上の表 |
| Telnet | スレッドのあとに `rp2040_telnet_start()` | スレッドの前に `nano_telnet_init()` |
| アプリ（3 版とも同じ順） | `kv_store_init()`、`imu_cal_init()`、`time_sync_init()`、`wifi_profile_init()` | 同じ |

### 4.7 POSIX のソケットのヘッダー

| N | F |
|---|---|
| NuttX の libc（`<sys/socket.h>`、`<netinet/in.h>`、`<arpa/inet.h>`、`<netdb.h>`、`<poll.h>`） | `common/compat/libc_net/include`: `arpa/inet.h`、`netdb.h`、`netinet/in.h`、`netinet/tcp.h`、`poll.h`、`sys/socket.h`、`sys/uio.h`。`common/compat/libc_net/inet.c` |

## 5. ドライバー

| ドライバー | 境界のヘッダー（`port/`） | N のファイル | F のファイル | Zephyr |
|---|---|---|---|---|
| `common/drivers/i2c_dw` | `i2c_dw_os.h`: 型 4、マクロ 5、インライン関数 10、`i2c_dw_os_uptime_ms()`。最後に読む `i2c_dw_instance.h`（`i2c_dw_os_prepare()`、`i2c_dw_os_irq_enable()`、デバイス）。`i2c_dw_dma_os.h`: 関数 10、コールバックの型 1 | `port/nuttx/i2c_dw_os_impl.h`、`i2c_dw_instance.h`（`struct i2c_master_s`）、`i2c_dw_dma_os_rp2040.c`、`i2c_dw_controllers_rp2040.h` | `port/freertos/i2c_dw_os_impl.h`、`i2c_dw_instance.h`（Zephyr のデバイス）、`i2c_dw_dma_os_rp2040.c`、`i2c_dw_controllers_rp2040.h` | 上流のドライバー + パッチ。`port/zephyr` は無い |
| `common/drivers/nina` | `nina_bus.h`（関数 7）、`nina_os.h`（関数 17、`NINA_OS_DNS_OFFLOAD` でさらに 3）、`nina_passthrough_bus.h`（17）、`nina_passthrough_os.h`（5、構造体、ログのマクロ）。つなぎは `nina_passthrough_run()` も実装する | `port/nuttx/nina_bus_rp2040.c`、`nina_os.c`、`nina_passthrough_bus_rp2040.c`、`nina_passthrough_os.c`。ネットワーク: `nina_netdev.c`、`nina_usrsock.c`、`nina_poller.c`、`nina_wifi_mgmt.c` | `port/freertos/nina_bus_rp2040.c`、`nina_os.c`、`nina_passthrough_bus_rp2040.c`、`nina_passthrough_os.c`、`nina_board_rp2040.h`。ネットワーク: 互換層の上の `port/zephyr/nina_net.c`、`nina_socket.c` | `port/zephyr/` |
| `common/drivers/lsm6dso` | `common/drivers/lsm6dso/port/` を参照 | | | |

`i2c_dw_i2c0_initialize()` の戻り値は、N が `struct i2c_master_s *`、F が `int` です（`i2c_dw_controllers_rp2040.h`）。

## 6. どの場合も OS ごとのもの

| 項目 | N | F |
|---|---|---|
| スケジューラーまでの起動 | NuttX の起動、`N-board/src/rp2040_boardinitialize.c`、`rp2040_bringup.c`、init タスク `nuttx/apps/init/rp2040_init.c` | `freertos/app/usb-console/src/main.c`、`vTaskStartScheduler()` |
| カーネルの tick とタイマー | tickless のタイマー `N-board/src/rp2040_oneshot.c`、`rp2040_timerisr_alarm.c`（tick 100 µs） | SysTick 1 ms、tick のフックと追い付き `tick.c`、`tick_catchup.h`。`FreeRTOSConfig.h` |
| 割り込みの登録 | `irq_attach()`、`rp2040_gpio_irq_attach()` | Pico SDK の `irq_set_exclusive_handler()`、`irq_add_shared_handler()`、`gpio_add_raw_irq_handler_masked()` |
| アイドルループ（区間の計算は共有の `common/compat/cpu_load.h`） | `N-board/src/rp2040_idle_cpuload.c` | `freertos/app/usb-console/src/idle.c` |
| BOOTSEL に入る（ROM の `reset_usb_boot()`） | `N-board/src/rp2040_restart.c`。NuttX の `ROM_LOOKUP()` の上 | Pico SDK の `reset_usb_boot()`（Zephyr 版は hal_rpi_pico のもの） |
| チップの ID（フラッシュの固有の ID。XIP を止めて 1 回読む） | `N-board/src/rp2040_chipid.c`。NuttX の `rp2040_flash_cmd()` の上 | Pico SDK の `pico_unique_id`（Zephyr 版は hwinfo のドライバー） |
| USB デバイスのスタック、コンソールの経路 | `cdcacm`、`N-board/src/rp2040_usbserial.c`。Telnet は `nuttx/apps/init/rp2040_telnet.c` | Pico SDK の `stdio_usb`（TinyUSB）。`freertos/drivers/shell/nano_console.c`、`nano_stdio.c`。Telnet は `freertos/drivers/telnet/nano_telnet.c` |
| ヒープ | NuttX のもの | FreeRTOS の `heap_4`、32 KiB（`configTOTAL_HEAP_SIZE`） |
| OS 自身のシェル | nsh（セッションは `rp2040_init.c`） | 無い。共有のエンジン `common/compat/shell_engine`（`cli.c`・`nano_shell.c`）を `freertos/drivers/shell` のコンソールの上で使う |
| RAM ログに入る OS 自身のログ | `syslog()`: syslog のチャネル、`nuttx/drivers/log/log_ram_syslog.c` | なし（FreeRTOS のカーネルと Pico SDK はログを持たない） |
| 致命的なエラー | `CONFIG_BOARD_RESET_ON_ASSERT`、報告は RAM ログ | `freertos/app/usb-console/src/fault.c` と `common/log/fault_record.c` |
| ビルドの仕組みと構成 | Make、Kconfig、`defconfig`、`nuttx/patches` | CMake、`FreeRTOSConfig.h`、`freertos/drivers/*/*.cmake` |
