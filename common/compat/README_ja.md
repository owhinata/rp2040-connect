# Zephyr の API の互換層（`compat`）

[English](README.md) | 日本語

このリポジトリーのファームウェアは、同じファームウェアを Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上で
それぞれビルドしたもので、それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。共有のソース（`common/`）は Zephyr の API で
書いてあります。互換層は、そのソース（写したドライバーとそれを呼ぶ側: 取得サービス、校正、センサーのコマンドなど）が使う Zephyr の API を、
Zephyr の名前のまま最小限だけ用意したもので、NuttX 版・FreeRTOS 版・ThreadX 版がこの上で共有のソースをビルドします。Zephyr 版は使いません
（Zephyr 本体を使う）。Zephyr の sensor・RTIO・カーネルの全体は移していません。

カーネルの部分の作りは 2 通りです。NuttX 版は Zephyr の関数（`k_sem_*()`・`k_mutex_*()`・`k_poll()` など）を NuttX の上に直接実装します。
FreeRTOS 版と ThreadX 版は、汎用の実装（`kernel_generic/`）が使う基本の操作（`port/zephyr_kernel_os.h`。型と定数 10 と関数 26）だけを実装します。
新しい OS が書くものの一覧は [`design/os-boundary-ja.md`](../../design/os-boundary-ja.md) にあります（ここには写しません）。
細部は各ファイルの先頭のコメントにあります。

## ディレクトリー

互換層は、ほかの `common/` の部品と同じ 3 層（`include/`・`internal/`・`port/`）を 1 組だけ持ちます。カーネルの部分・ログ・シェルのエンジン・
ネットワークの部分・POSIX のネットワークのヘッダーは、ソースを分けるサブディレクトリーで、部品ごとの 3 層は持ちません。

| 場所 | 内容 |
|---|---|
| 直下 | OS に依存しないソース: ワーク（`zephyr_work.c`）、ワークキューのスレッドと起動（`zephyr_init.c`）、RTIO（`zephyr_rtio.c`・`zephyr_rtio_workq.c`）、ブロックのプール（`zephyr_mem_blocks.c`）、`k_heap`（`zephyr_heap.c`）、デバイスの一覧（`zephyr_device.c`）、`gpio0`（`zephyr_gpio.c`）、センサーの時計（`zephyr_sensor_clock.c`）、sensor の既定の iodev（`sensor_default_rtio.c`）と sensor シェルの処理スレッド（`sensor_shell_stream.c`）、`net_mgmt`・Wi-Fi の要求（`zephyr_net_mgmt.c`・`zephyr_wifi_mgmt.c`・`zephyr_wifi_utils.c`）、Wi-Fi の資格情報（`wifi_credentials.c`・`wifi_credentials_backend_settings.c`）、KV の flash（`zephyr_flash_kv.c`・`kv_store_wipe.c`）、Pico SDK の上の OS だけがビルドするもの（`zephyr_wifi_shell.c`・`zephyr_zperf_shell.c`・`zperf_iperf3.c`） |
| `include/` | ほかの部品が読むヘッダー。Zephyr の API のヘッダー（`include/zephyr/...`）と、互換層のヘッダー（`zephyr_compat.h`、`priorities.h`、`app_log.h`、`flash_kv.h` など） |
| `internal/` | 互換層の内側のヘッダー（`zephyr_os_internal.h`、`log_ring.h`、`cli.h`、`shell_argv.h`、`wifi_credentials_internal.h`、`zperf_iperf3.h`） |
| `port/` | OS が実装する境界のヘッダー: 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os.h`）、`cpu_load.h`、`gpio_os.h`、`flash_kv_os.h`、`log_ram_os.h`、`shell_serial_os.h` |
| `port/nuttx/` | NuttX の実装（下の「NuttX」） |
| `port/freertos/`、`port/threadx/` | FreeRTOS と ThreadX の基本の操作（`zephyr_kernel_os_impl.h`）、優先度の対応（`priorities_os.h`）、ポートのヘッダー（`rp2040/zephyr_os_port.h`）。ThreadX はロックの変数（`zephyr_kernel_os.c`）も |
| `port/pico_sdk/` | Pico SDK の上の OS（FreeRTOS 版と ThreadX 版）で共有する部分（下の「Pico SDK」） |
| `port/generic/` | その仕組みを自分で持たない OS のための部分（FreeRTOS 版と ThreadX 版。下の「Pico SDK」） |
| `kernel_generic/` | 基本の操作だけを持つ OS のためのカーネルの部分。OS に依存しない: `k_sem`・`k_poll()`・タイムアウト・スリープ・スレッドの起動（`zephyr_kernel.c`）、`k_mutex`（`zephyr_mutex.c`） |
| `log/` | RAM ログ（`log_ram.c`、リングは `log_ring.c`）と異常停止の記録の形式（`fault_record.c`）。NuttX 版・FreeRTOS 版・ThreadX 版が RAM ログを共有し、異常停止の記録は FreeRTOS 版と ThreadX 版が使う |
| `shell_engine/` | 自分のシェルを持たない OS のためのシェル（下の「シェルのエンジン」） |
| `net_offload/` | ネットワーク層を持たない OS のためのネットワークの部分（下の「ソケット層（`net_offload`）」）。ヘッダーは自分の `include/` に |
| `libc_net/` | newlib に無いソケット系のヘッダー（`include/`。型と定数だけ）、`inet_pton`・`inet_ntop`（`inet.c`） |
| `subsys/`、`lib/` | Zephyr のファイルの写し: NVS、settings、CRC-8、16 進の変換（下の「KV」）、`fnmatch()`、getopt、`wifi cred` のコマンド（`wifi_credentials_shell.c`） |

`include/`・`internal/`・`port/`・`port/generic/` のヘッダーはどれも、互換層の全体か、部品（`kernel_generic`、`port/generic` の
その仕組みを持たない OS のための部分、`log`、`shell_engine`）のどれか 1 つに属します。属する先は `common/tests/run.sh` の表にあり、どれにも
入っていないヘッダーや 2 つに入っているヘッダーがあると失敗します。部品のソースが、要らないはずの部品のヘッダーを読んでいないことも
同じ所で確かめます（下の「テスト」）。

## 提供するもの

### ヘッダー

| ヘッダー（場所の無いものは `common/compat/include/`、（NuttX）は `common/compat/port/nuttx/`） | 中身 |
|---|---|
| `zephyr/device.h` | `struct device`（名前、config、API、状態、data、初期化関数）、`device_is_ready()`、`device_init()`（Zephyr の `do_device_init()` と同じ意味）、`DEVICE_DT_GET()`、`DEVICE_DT_DEFINE()`、`DEVICE_API()`、`device_get_binding()`（ノード名）、`device_get_by_dt_nodelabel()`（ノードのラベル）、`DEVICE_API_IS(sensor, dev)`（下） |
| `zephyr/devicetree.h` | `DT_NODELABEL()`、`DT_BUS()`。ノードは `gpio0`、`i2c0`、`lsm6dsox` だけ。`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()`（Zephyr と同じ定義）は `st_lsm6dso` の `i2c` で 1、ほかの互換文字列・バスでは 0（`lsm6dso.h`）。`DT_HAS_CHOSEN()` はいつも 0（Zephyr 版も `zephyr,settings-partition` を選んでいない） |
| `zephyr/kernel.h` | `k_timeout_t`（`K_NO_WAIT`、`K_FOREVER`、`K_MSEC()`、`K_SECONDS()`。単位は OS の tick）、`k_mutex`、`k_sem`（`K_SEM_DEFINE`）、セマフォに対する `k_poll()`（`K_POLL_TYPE_SEM_AVAILABLE`・`K_POLL_MODE_NOTIFY_ONLY`）、ワーク（`k_work`、`k_work_delayable`、`k_work_q`、`k_work_submit()`、`k_work_cancel_sync()`、`k_work_schedule_for_queue()`、`k_work_reschedule_for_queue()`、`k_work_queue_start()`）、`K_THREAD_STACK_DEFINE()`、`K_PRIO_PREEMPT()`、`k_thread_name_set()`、システムワークキュー `k_sys_work_q`、`K_MUTEX_DEFINE()`、`K_THREAD_DEFINE()`（下）、`k_current_get()`、`k_uptime_get()`、`k_sleep()`・`k_msleep()`・`k_usleep()`・`k_busy_wait()`、`k_cycle_get_32()`・`k_cycle_get_64()`・`k_cyc_to_us_floor64()`・`k_cyc_to_us_ceil64()`・`k_cyc_to_ns_floor64()`・`sys_clock_hw_cycles_per_sec()`、`k_heap`（`zephyr_heap.h`。どちらのカーネルの部分でも使える）。全体の一覧はこのヘッダーの先頭のコメント |
| `zephyr/spinlock.h`、`zephyr/sys/atomic.h` | `k_spin_lock()`、`atomic_t` と `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`、`zephyr/rtio/work.h` | RTIO（下）と RTIO のワークキュー（`rtio_work_req_alloc()`・`_submit()`） |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`、`gpio_is_ready_dt()`、`gpio_pin_configure_dt()`（入力だけ）、`gpio_pin_interrupt_configure_dt()`、`gpio_pin_get_dt()`、コールバック（`gpio_init_callback()`・`gpio_add_callback()`・`gpio_remove_callback()`） |
| `zephyr/drivers/sensor.h` | `struct sensor_value`、チャネル・属性・トリガーの列挙（Zephyr と同じ番号）、`struct sensor_driver_api`、`sensor_attr_set()`・`_get()`、`sensor_sample_fetch()`・`_chan()`、`sensor_channel_get()`、`sensor_trigger_set()`、単位の換算。非同期の API: `struct sensor_chan_spec`、デコーダー（`struct sensor_decoder_api`、`sensor_three_axis_data` など）、`SENSOR_DT_READ_IODEV()`、`SENSOR_DT_STREAM_IODEV()`、`sensor_get_decoder()`、`sensor_stream()`、`sensor_read()`、`sensor_read_async_mempool()`、`sensor_processing_with_callback()`。復号したデータの表示の書式 `PRIsensor_*_data`（Zephyr の `sensor_data_types.h`。Q 値の書式は `dsp/print_format.h`） |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`、`sensor_clock_cycles_to_ns()`（下の「センサーの時計」） |
| `zephyr/drivers/i2c.h`（NuttX と `port/generic/`） | `struct i2c_dt_spec`、`i2c_burst_read_dt()`、`i2c_write_dt()`、`i2c_write_read_dt()`、`i2c_configure()`、`i2c_get_config()`、速さの区分（`I2C_SPEED_*`、`I2C_SPEED_SET()`）。`i2c` コマンドのための `struct i2c_msg` とメッセージのフラグ（`I2C_MSG_*`）、`i2c_transfer()`、`i2c_write()`・`i2c_read()`・`i2c_write_read()`、`i2c_recover_bus()`、`i2c_target_driver_register()`・`_unregister()`（`-ENOSYS`）。NuttX の部分は下の「NuttX」、`port/generic/` のものはドライバーのデバイスの上の Zephyr の I2C の API（[I2C のドライバー](../drivers/i2c_dw/README_ja.md)） |
| `zephyr/sys/util.h`、`sys/mpsc_lockfree.h`、`sys/mem_blocks.h`、`dsp/types.h` | `BIT()`、`ARRAY_SIZE()`、`MIN()`・`MAX()`、`IS_ENABLED()`、`CONTAINER_OF()`、`GENMASK()`・`GENMASK64()`・`FIELD_GET()`、`STRINGIFY()`、`KB()`、`ZTESTABLE_STATIC`、`hex2bin()`・`bin2hex()`（実装は `lib/utils/hex.c` の写し）など（Zephyr と同じく `<string.h>` も読む）、RTIO の待ち行列とブロックのプール、`q31_t` |
| `zephyr/sys/__assert.h`（NuttX と `port/generic/`） | `__ASSERT()`・`__ASSERT_NO_MSG()`（NuttX 版は NuttX の `DEBUGASSERT()`。ホストテストは `assert()`） |
| `zephyr/dsp/print_format.h` | Zephyr の同名のヘッダーの写し（先頭の説明と `<zephyr/sys/util.h>` のインクルードだけを足した）。Q 値の表示の書式 `PRIq()`・`PRIq_arg()` |
| `zephyr/types.h` | `<stddef.h>` と `<stdint.h>`（`lsm6dso_sensor.h` が読む目的のもの） |
| `zephyr/init.h` | 中身の無いヘッダー（`lsm6dso_sensor.c` が読む。`SYS_INIT()` と初期化レベルを使うのは Zephyr 版のインスタンス化のマクロだけ） |
| `zephyr/sys/iterable_sections.h` | `STRUCT_SECTION_FOREACH()` だけ（いつも空の区間。Zephyr のリンカーの区間は無い）。sensor シェルが読み（使うのは Zephyr 版の構成に無い `CONFIG_SENSOR_INFO` のときだけ）、settings が静的なハンドラーの区間を回す（この移植には無い） |
| `zephyr/sys/printk.h`、`zephyr/sys/byteorder.h`、`zephyr/sys/cpu_load.h` | `snprintk()`（Zephyr 版の構成 `CONFIG_PICOLIBC=y` と同じく `snprintf()`）、`sys_put_be16/32()`・`sys_get_be16/32()`、`cpu_load_get()`（`port/cpu_load.h`。実装はボード） |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()`（NuttX 版は下の「NuttX」。Pico SDK の上の OS は `port/pico_sdk/zephyr_hwinfo.c`） |
| `zephyr/shell/shell.h` | `shell_print()`・`shell_info()`・`shell_warn()`・`shell_error()`・`shell_fprintf()`、`struct shell`（シェルのバックエンド。中身は OS の部分 `zephyr_os_shell.h`）、コマンドの表のマクロ（`SHELL_CMD_REGISTER()`、`SHELL_STATIC_SUBCMD_SET_CREATE()`、`SHELL_DYNAMIC_CMD_CREATE()`、`SHELL_CMD_ARG()`、`SHELL_CMD()`、`SHELL_COND_CMD()`、`SHELL_EXPR_CMD_ARG()`、`SHELL_HELP()`、`SHELL_SUBCMD_SET_END`）、`shell_device_get_binding()`・`shell_device_filter()`（下の「シェルのコマンドの処理」） |
| `zephyr/net/socket.h`（NuttX と `net_offload/include/`） | ソケットの API（`zsock_*()`）。NuttX 版は NuttX の関数（下の「NuttX」）、FreeRTOS 版と ThreadX 版は Zephyr のソケット層の写し（下の「ソケット層（`net_offload`）」） |
| `zephyr/sys/clock.h`（NuttX と `port/generic/`） | `sys_clock_gettime()`・`sys_clock_settime()`（`SYS_CLOCK_REALTIME`。0 か `-EINVAL`）、`NSEC_PER_USEC`・`USEC_PER_SEC`。NuttX 版は NuttX の `clock_gettime()`・`clock_settime()` の `CLOCK_REALTIME`、Pico SDK の上の OS の壁時計は `port/pico_sdk/zephyr_sys_clock.c` |
| `zephyr/net/net_if.h`（NuttX と `net_offload/include/`） | `struct net_if`、`net_if_get_default()`・`net_if_get_wifi_sta()`、アプリが読む IPv4 のアドレスとゲートウェイ（`net_if_ipv4_get_global_addr()`・`net_if_ipv4_get_gw()`）、`net_if_is_admin_up()` |
| `zephyr/net/net_mgmt.h` | `net_mgmt()`（要求）、`struct net_mgmt_event_callback`、`net_mgmt_init_event_callback()`・`net_mgmt_add_event_callback()`・`_del_event_callback()`、`net_mgmt_event_notify_with_info()`（下） |
| `zephyr/net/wifi_mgmt.h` | Wi-Fi の型（セキュリティ、帯域、MFP、状態、モード。Zephyr と同じ値）、`struct wifi_scan_params`・`wifi_scan_result`・`wifi_connect_req_params`・`wifi_iface_status`・`wifi_status`、`struct wifi_mgmt_ops`、オフロードのインターフェースの API と `NET_DEVICE_DT_OFFLOAD_DEFINE()`、要求 `NET_REQUEST_WIFI_SCAN`・`_CONNECT`・`_DISCONNECT`・`_IFACE_STATUS`・`_CONNECT_STORED`（`CONFIG_WIFI_CREDENTIALS` のとき）、イベント `NET_EVENT_WIFI_SCAN_RESULT`・`_SCAN_DONE`・`_CONNECT_RESULT`・`_DISCONNECT_RESULT`、`wifi_mgmt_raise_connect_result_event()`・`_disconnect_result_event()`（下）、`wifi_security_txt()`・`wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | Zephyr の同名のヘッダーの写し: `struct wifi_credentials_header`・`_personal`、`WIFI_CREDENTIALS_FLAG_*`、`wifi_credentials_set_personal_struct()`・`_get_by_ssid_personal_struct()`・`_delete_by_ssid()`・`_for_each_ssid()`・`_is_empty()` など。設定の値（`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` など。Zephyr の名前）は OS の構成にあればそれ（NuttX 版は `nuttx/apps/wifi/Kconfig`）、無ければ Zephyr 版の値。Zephyr に無い `wifi_credentials_init()`（init が呼ぶ。Zephyr の `SYS_INIT()`）。関数は `wifi_credentials.c`（Zephyr のライブラリーの写し）と、設定に保存する `wifi_credentials_backend_settings.c` |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()`（`_2g`・`_5g`・`_6g`）、`net_bytes_from_str()`（Zephyr では `<zephyr/net/net_ip.h>`） |
| `zephyr/settings/settings.h`、`zephyr/kvss/nvs.h`、`zephyr/sys/slist.h`、`zephyr/sys/list_gen.h` | Zephyr の同名のヘッダーの写し（無改変。下の「KV」）。settings の API（`settings_subsys_init()`、`settings_save_one()`、`settings_load_one()`、`settings_delete()`、`settings_load_subtree_direct()` など）、NVS の API と `struct nvs_fs`、単方向リスト |
| `zephyr/drivers/flash.h`、`zephyr/storage/flash_map.h` | NVS と settings が使う flash の API（`flash_read()`・`flash_write()`・`flash_erase()`・`flash_flatten()`・`flash_get_parameters()`・`flash_get_write_block_size()`・`flash_get_page_info_by_offs()`、`struct flash_parameters`）と flash map（`flash_area_open()`・`flash_area_close()`・`flash_area_get_sectors()`、`PARTITION_ID(storage_partition)`）。デバイスは KV の領域の flash だけ（`zephyr_flash_kv.c`。下） |
| `zephyr/sys/crc.h`、`zephyr/toolchain.h` | `crc8_ccitt()` などの宣言（実装は `subsys/crc/crc8_sw.c` の写し）、`__packed`・`__weak` |
| `flash_kv.h`、`port/flash_kv_os.h` | KV の領域の定数（`0xffe000` から 8 KiB、256 バイトのページ、4 KiB のセクター）、OS の境界（`flash_kv_os.h`。下の「KV」）、範囲の検査・ページへの分割・領域の中身の判定（Zephyr の NVS、以前の NuttX 版の形式、消去済み、それ以外） |
| `zephyr_kv_config.h` | 写した NVS と settings の前に読む Zephyr 版の Kconfig の値（`CONFIG_SETTINGS_NVS_SECTOR_COUNT` など。下の「KV」） |
| `kv_store_os.h` | `common/imu/kv_store.c` の OS の部分: `kv wipe` のサブコマンド（Zephyr 版の `zephyr/app/usb-console/src/kv_store_os.h` は空） |
| `zephyr/logging/log.h` | `LOG_MODULE_REGISTER()`、`LOG_ERR()` などのログのマクロ。`app_log.h`（`dmesg` の RAM ログ）を読むだけ |
| `zephyr/net/net_ip.h` | IPv4 のアドレスの型 `struct in_addr`（`imu_raw_mqtt.h`）。NuttX の `<netinet/in.h>` を読むだけ（同じメンバー `s_addr`、ネットワークバイト順）。FreeRTOS 版と ThreadX 版では `net_offload/include/` の同名のヘッダーがこれに代わる（下の「ソケット層（`net_offload`）」） |
| `zephyr_compat.h` | `zephyr_compat_initialize()`（起動の処理が呼ぶ。下の「起動」）、`K_THREAD_DEFINE_START()`（アプリのスレッドの表、`common/imu/app_init.c` が呼ぶ） |
| `sensor_shell.h` | Zephyr の `drivers/sensor/sensor_shell.h` の写し（sensor シェルの共通の型。Zephyr 版は Zephyr のものを読む） |
| `zephyr_os.h`、`zephyr_os_spinlock.h`、`zephyr_os_atomic.h`、`zephyr_os_shell.h`（NuttX と `port/generic/`） | OS ごとのヘッダー（下の「OS が実装する境界」） |
| `common_prelude.h`（NuttX と `port/generic/`） | 前置きヘッダー（NuttX 版は下の「NuttX」） |

### ソース

| ソース（場所の無いものは `common/compat/`） | 中身 |
|---|---|
| `zephyr_work.c` | ワークとワークキュー。Zephyr の `kernel/work.c` の使っている部分の写し（OS に依存しない） |
| `zephyr_init.c` | ワークキューのスレッド（`k_work_queue_start()`、`kernel/work.c` の `work_queue_main()` のループ）、システムワークキュー `k_sys_work_q`、`zephyr_compat_initialize()`、`k_uptime_get()`（TIMER の ms）。`z_thread_static_start()` の上（OS に依存しない） |
| `zephyr_rtio.c`、`zephyr_rtio_workq.c`、`zephyr_mem_blocks.c` | RTIO の executor（`subsys/rtio/rtio_executor.c`）、RTIO のワークキュー（`rtio_workq.c`。スレッド `RTIO WQ 0` は `zephyr_compat_initialize()` が `z_thread_static_start()` で起動する）、ブロックのプール（`lib/mem_blocks`） |
| `zephyr_heap.c` | `k_heap`（`<zephyr/kernel.h>` が読む `zephyr_heap.h`）。どちらのカーネルの部分でも使える |
| `zephyr_gpio.c` | `gpio0` のデバイス（Zephyr の `gpio.h` の処理と `gpio_rpi_pico.c`）。OS の `port/gpio_os.h` の上 |
| `zephyr_sensor_clock.c` | センサーの時計。互換層のサイクルカウンター `k_cycle_get_64()`（TIMER、`rp2040_timer_us()`、1 MHz） |
| `sensor_default_rtio.c` | Zephyr の `drivers/sensor/default_rtio_sensor.c` の写し（ログだけ置き換え）。センサーの iodev の API、単発取得のフォールバック、既定のデコーダー |
| `zephyr_device.c` | デバイスの初期化と一覧（`gpio0`、`i2c0`、`lsm6dsox` を Zephyr 版の初期化の順に、ノードのラベルと sensor の API かどうかと共に。`gpio0` は OS の構成の `CONFIG_LSM6DSO_STREAM` のとき） |
| `zephyr_net_mgmt.c`、`zephyr_wifi_mgmt.c` | `net_mgmt` のイベントの待ち行列とスレッド（スレッドは `z_thread_static_start()` で起動。優先度とスタックの大きさは `priorities.h` の `APP_PRIO_NET_MGMT`・`APP_STACK_NET_MGMT`）、Wi-Fi の要求の処理（Zephyr の `subsys/net/l2/wifi/wifi_mgmt.c` の検査の写し）とイベント、`NET_REQUEST_WIFI_CONNECT_STORED`（同じファイルの `connect_stored_command()` の写し。`CONFIG_WIFI_CREDENTIALS` のとき）。NuttX 版は `CONFIG_NINA_NET` のときだけ |
| `zephyr_wifi_utils.c` | `wifi_security_txt()`・`wifi_band_txt()`（Zephyr の `wifi_mgmt.c`）、チャネルの検査（`wifi_utils.c`）、`net_bytes_from_str()`（`subsys/net/ip/utils.c`）の写し。`wifi cred` が使う。OS に依存しない。NuttX 版は `CONFIG_NINA_NET` のときだけ |
| `sensor_shell_stream.c` | Zephyr の `drivers/sensor/sensor_shell_stream.c` の写し（sensor シェルの処理スレッド）。OS の `sensor_shell_os.h` を最初に読み、処理スレッドのコールバックを `sensor_shell_os_processing_callback()` にした（NuttX 版は `common/shell/port/nuttx/sensor_shell_nuttx.c`、Pico SDK の上の OS は `common/shell/port/generic/sensor_shell_os.c`） |
| `wifi_credentials.c`、`wifi_credentials_backend_settings.c` | Zephyr の `subsys/net/lib/wifi_credentials/` の写し（保存した Wi-Fi の資格情報）。バックエンドの変更は 2 つ: 読み込みのバッファーを 8 バイトに揃えた（構造体として読むため。Cortex-M0+ は揃っていない読み出しで止まる）、番号の書式を `%zu` にした |
| `subsys/kvss/nvs/`、`subsys/settings/`、`subsys/crc/crc8_sw.c`、`lib/utils/hex.c` | Zephyr v4.5.0 の NVS、settings、CRC-8、16 進の変換の写し（無改変。下の「KV」） |
| `zephyr_flash_kv.c` | KV の領域の flash デバイスと flash map（下の「KV」） |
| `kv_store_wipe.c` | `kv wipe confirm`（領域を消し、settings を初期化し直す） |
| `subsys/portability/posix/c_lib_ext/fnmatch.c`、`lib/utils/getopt/` | Zephyr の `fnmatch()` と getopt の写し（無改変。newlib に `fnmatch()` が無いため。getopt の状態は、シェルのエンジンがセッションごとに持つ） |
| `subsys/net/lib/wifi_credentials/wifi_credentials_shell.c` | `wifi cred`。Zephyr の `wifi_credentials_shell.c` の無改変の写し |
| `zephyr_wifi_shell.c` | `wifi` コマンド。Zephyr の `wifi_shell.c` から `connect`・`disconnect`・`scan`・`status` を取り出し、このボードで使えるオプションだけを残したもの（Pico SDK の上の OS だけ） |
| `zephyr_zperf_shell.c`、`zperf_iperf3.c`、`internal/zperf_iperf3.h` | `zperf`（Pico SDK の上の OS だけ。組み込みは `port/pico_sdk/zperf.cmake`） |

### ワーク

フラグ（RUNNING・CANCELING・QUEUED・DELAYED）と戻り値は `kernel/work.c` と同じです。`submit` はキュー済みなら何もしない（0）、
実行中なら同じキューに入れ直す（2）。`schedule` は遅延中・キュー済みなら何もしない（実行中なら予約できる）、`reschedule` は遅延を
止めてから予約し直す（`K_NO_WAIT` ならその場でキューに入れる）。`k_work_cancel_sync()` はキューから外し、実行中なら終わるまで
待ちます（待てるのは 1 スレッドだけ。Zephyr は複数）。キューのスレッドは処理の後、ほかに待ちがあれば `k_yield()` します。
遅延の時間は Zephyr と同じく「今の tick + tick 数 + 1」で切れます。移していないもの: flush、drain、plug、キューの停止、ワークの
タイムアウト、`k_work_cancel()`、`k_work_cancel_delayable()`。

### セマフォと `k_poll()`

`k_sem_give()` は上限で止め、そのセマフォを `k_poll()` で待っているスレッドを起こします（取らずに通知するだけ。Zephyr の
`K_POLL_MODE_NOTIFY_ONLY`）。`k_sem_take()` は 0、`-EBUSY`（`K_NO_WAIT`）、`-EAGAIN`（時間切れ）。`k_poll()` はセマフォだけを扱います。

### RTIO

- Zephyr の構成（`CONFIG_RTIO_SUBMIT_SEM`・`_CONSUME_SEM`・`_SYS_MEM_BLOCKS`、コールバックとユーザーモードなし）の写しです。
  寸法は呼ぶ側の `RTIO_DEFINE_WITH_MEMPOOL(name, 送信数, 完了数, ブロック数, ブロックの大きさ, 境界)` で決まります
  （`imu_stream.c` は 4、16、128、64、4）。iodev の submit は `rtio_submit()` の呼び出し元の文脈で呼ばれます。multishot は成功すると、
  完了を出す前に iodev に戻り（プールのバッファーの結び付けを外して）、失敗かキャンセルでバッファーと送信を解放して終わります。
  完了の空きが無いと完了は捨てられて `xcqcnt` に数えられます（そのバッファーは誰も解放できず残ります。Zephyr と同じ）。
  `rtio_cqe_consume()` は完了を待たず、`consume_sem` を取ります。ハンドルは送信のプールの番号と世代で、解放のたびに世代が進み、
  古いハンドルは解決もキャンセルもされません。`rtio_sqe_cancel()` は印を付けて iodev のキャンセルの関数（あれば）を呼びます。
  プールの割り当ては `max` から `min` まで 1 ブロックずつ減らして最初に取れた連続ブロック（最初の適合）。Zephyr との違いは、
  待ち行列（`struct mpsc`）がロックフリーでなく割り込み禁止を使うことと、プールの空きリストを起動時（`rtio_init()`）ではなく最初の
  割り当てで作ることです（渡す順は同じ）。移していないもの: チェーンとトランザクション、executor 自身の操作（コールバック、待ち、
  遅延）、書き込みなどの操作、`rtio_sqe_copy_in()`・`rtio_cqe_copy_out()`、`rtio_pool`、完了のコールバック、ユーザーモード。
- **RTIO のワークキュー**: Zephyr の `rtio_workq.c` と同じく、要求 4 つのプールと 1 スレッド（`RTIO WQ 0`、優先度 0、スタック 1024）で、
  投入順に実行します。単発取得のフォールバック（`common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio.c`）が使います。

### GPIO

ポートは `gpio0`（GPIO0〜29）だけで、入力だけです（出力は `-ENOTSUP`）。ピンの設定、割り込み（エッジ・レベル、論理・物理）、
レベルの読み出し、コールバックは Zephyr の `gpio.h` と `gpio_rpi_pico.c` の写しで（`zephyr_gpio.c`）、レジスターの操作は OS の
`port/gpio_os.h` の実装です（NuttX 版は `port/nuttx/rp2040_gpio_os.c`、Pico SDK の上の OS は `port/pico_sdk/gpio_os_rp2040.c`）。

### センサーの時計とサイクルカウンター

Zephyr 版の `k_cycle_get_*()` は 125 MHz の SysTick の数（負荷で遅れる）で、互換層ではボードの TIMER（`rp2040_timer_us()`、水晶の
1 MHz）です。センサーの時計も同じで、ストリームのバッチの時刻と単発取得の時刻は 1 µs 単位になります。1 サイクルが 1 µs なので、
µs への換算（`k_cyc_to_us_floor64()`・`_ceil64()`）はそのままの値です。`k_cycle_get_32()` は下位 32 ビットで約 71.6 分で一周します
（Zephyr 版は約 34 秒）。`k_cyc_to_ns_floor64()` は 1000 倍です。`imu_raw_mqtt.c` がキューのロックの待ちと送信の時間を、LSM6DSO の
ドライバーが FIFO の読み出しの時間を計るのに使います。

### 起動と静的なスレッド

- **起動**: `zephyr_compat_initialize()` を、起動の処理が I2C0（その DMA のチャネルを含む）・LSM6DSOX より前に呼び、システムワークキュー
  （`sysworkq`）、RTIO のワークキュー、`gpio0` を用意します（Zephyr がそれらを `POST_KERNEL` でセンサーより先に用意するのと同じ順）。
  `imu_fifo_worker` は LSM6DSOX の初期化（`lsm6dso_stream_init()`）が作ります。
- **静的なスレッド**（`K_THREAD_DEFINE()`）: Zephyr と同じ引数で、静的なスタック、`K_PRIO_PREEMPT(prio)` の優先度、名前はトークンそのもの
  （`imu_stream_tid`）のスレッドを定義します。Zephyr はこれらを `main()` の前に自動で始めますが、互換層にはその一覧が無いので、
  NuttX 版・FreeRTOS 版・ThreadX 版で共有のアプリの表（`common/imu/app_init.c` の `app_threads_start()`、`common/imu/app_main.h`）から
  `K_THREAD_DEFINE_START(name)`（`zephyr_compat.h`）で 1 本ずつ、`main()` 相当の処理（`app_main()`）の前に始めます。互換層のスレッド
  （ワークキューのもの、`RTIO WQ 0`、`net_mgmt`）も同じ `z_thread_static_start()` で始めます。入口の関数が戻ると終わります。
  `options` と `delay` は 0 だけ受け付けます（ほかはビルドエラー）。

### シェルのコマンドの処理

- `shell_print()`・`shell_info()`・`shell_warn()`・`shell_error()` は改行を付けて書きます（Zephyr は情報を緑、警告を黄色、エラーを赤で出す。
  ここでは色を付けない）。書き先は `sh` で決まります（NULL なら呼んだタスクの標準出力・標準エラー、バックエンドならそのセッション。
  OS ごとの扱いは下の「NuttX」と「シェルのエンジン」）。
- コマンドの表のマクロは Zephyr の `shell.h` と同じ形です。照合・ヘルプ・ワイルドカード・引数の数の検査は共有の `common/shell/zshell.c` が
  行います。`z_shell_cmd_main()` が、Zephyr の `shell.c` の `execute()`・`exec_cmd()`・`cmd_precheck()`、`shell_wildcard.c`、
  `shell_help.c` と同じ手順で、分けた語を照合し（動的なサブコマンドの関数を Zephyr と同じ順に呼ぶ）、ワイルドカードを展開し、引数の数を
  確かめ、ヘルプ（`SHELL_HELP()` の説明と `Usage:` の行、80 桁での折り返し）を出し、ハンドラーを呼びます。ハンドラーの `argv` は Zephyr と
  同じく最後の語の後が NULL です。Zephyr との違い: サブコマンドの集合が動的かどうかを `struct shell_cmd_entry` が持つ（Zephyr は
  `union shell_cmd_entry` を置いたリンカーのセクションで見分ける）、選択モード・別名・リモートのコマンド・生の引数が無い、カーソルを
  VT100 のシーケンスで動かす代わりに空白を出す。
- `zshell.c` は OS に依存しません。OS の部分（`common/shell/port/zshell_os.h` が一覧を持つ）は、`struct shell`、バックエンドへの出力
  `z_shell_backend_vfprintf()`、呼んだタスクのセッションのバックエンドを返す関数、ハンドラーの呼び出しの前後の印の 2 つです
  （NuttX 版は `common/shell/port/nuttx/zshell_os_impl.h`、Pico SDK の上の OS はシェルのエンジンの上の `common/shell/port/generic/zshell_os_impl.h`）。
  ヘルプとエラーは `printf()`・`fprintf(stderr)` で出すので、コマンドを実行するタスクの標準出力・標準エラーがそのセッションの端末で
  あることが前提です。
- `shell_device_get_binding()`・`shell_device_filter()` は Zephyr の `shell_utils.c` と同じで、デバイスの一覧（`zephyr_device.c`）を見ます
  （`DEVICE_API_IS(sensor, dev)` は一覧の印。Zephyr は API の表がそのクラスのリンカーのセクションにあるかで見る）。

### `net_mgmt` と Wi-Fi の要求

- **`net_mgmt`**: 要求 `net_mgmt(NET_REQUEST_X, iface, data, len)` は、Zephyr のマクロと同じく `net_mgmt_NET_REQUEST_X()` を呼び出した
  スレッドで呼びます。イベントは Zephyr 版の構成（`CONFIG_NET_MGMT_EVENT_THREAD`・`_QUEUE`・`_INFO`、待ち行列 5 個、空きを 10 ms 待つ）と
  同じく、情報を写して待ち行列に入れ、スレッド `net_mgmt`（優先度 −1、スタック 2048）が順に取り出して、マスクの合うコールバックを
  呼びます（`cb->info`・`info_length` を付け、コールバックの一覧のロックの下で）。10 ms で空きができなければ捨てます（Zephyr 版の警告は
  ログのレベル 0 で消えている）。イベントの番号はこの移植のビット（Zephyr の 64 ビットの層と命令の値とは違う）で、同じ名前で比べます。
  スレッドはインターフェースより先に始めます（Zephyr の `net_init()` と同じ順）。
- **Wi-Fi の要求**（`zephyr_wifi_mgmt.c`）: Zephyr の `wifi_mgmt.c` と同じく、インターフェースの API にドライバーの操作が無ければ
  `-ENOTSUP`、`IFF_UP` でなければ `-ENETDOWN`、接続の引数は同じ検査（チャネル、SSID の長さ 1〜32、パスフレーズの長さ 8〜64（WEP は
  5/10/13/26）、セキュリティの種類ごとの必須項目）をしてからドライバーを呼びます（Zephyr 版の構成には wpa_supplicant が無い）。スキャンの
  結果は 1 件ずつ `NET_EVENT_WIFI_SCAN_RESULT`、終わりは `NET_EVENT_WIFI_SCAN_DONE`（`struct wifi_status`）のイベントにします。
  `NET_REQUEST_WIFI_CONNECT_STORED` は、保存した資格情報を番号の順に読み、資格情報から接続の引数を作って（チャネル 0 は「どれでも」、
  期限 0 は 30 秒、種類が範囲外なら暗号化なし、帯域と MFP はフラグから）`NET_REQUEST_WIFI_CONNECT` を出し、最初に受け付けられたところで
  止めます。戻り値は常に 0 です（Zephyr と同じ。Zephyr 版の構成ではこのファイルのログはレベル 0 で消えているので、ログも出しません）。
- インターフェースと Wi-Fi のドライバーは NINA のつなぎが提供します（[NINA-W102 のドライバー](../drivers/nina/README_ja.md)）。

### KV: Zephyr の NVS と settings の写し

NuttX 版・FreeRTOS 版・ThreadX 版の KV は、Zephyr 版と同じバイト列を flash に書くため、Zephyr v4.5.0 の NVS と settings のうち
Zephyr 版が使う部分を改変せずに写し、Zephyr 版と同じ設定で動かします。写したファイルが元と同じことは、Zephyr のツリー
（`zephyr/zephyr`）があるときにホストテスト（`common/tests/run.sh` の `zephyr_copies`）が `cmp` で確かめます。

| 写し（`common/compat/`） | 元（Zephyr v4.5.0） | 変更 |
|---|---|---|
| `subsys/kvss/nvs/nvs.c`、`nvs_priv.h` | `subsys/kvss/nvs/` | なし |
| `subsys/settings/src/settings.c`、`settings_store.c`、`settings_init.c`、`settings_nvs.c`、`settings_priv.h` | `subsys/settings/src/` | なし |
| `subsys/settings/include/settings/settings_nvs.h`、`settings_file.h` | `subsys/settings/include/settings/` | なし |
| `subsys/crc/crc8_sw.c` | `subsys/crc/crc8_sw.c` | なし |
| `lib/utils/hex.c` | `lib/utils/hex.c` | なし |
| `zephyr/kvss/nvs.h`、`zephyr/settings/settings.h`、`zephyr/sys/slist.h`、`zephyr/sys/list_gen.h` | `include/zephyr/...` | なし |
| `wifi_credentials_backend_settings.c` | `subsys/net/lib/wifi_credentials/` | 上の表（2 つ） |

- 設定: 写したソースは `zephyr_kv_config.h`（Zephyr 版の `.config` の値。`CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT` 1、
  `CONFIG_SETTINGS_NVS_SECTOR_COUNT` 8、`CONFIG_SETTINGS_DYNAMIC_HANDLERS`、`CONFIG_MULTITHREADING`、ログのレベル 3。
  `CONFIG_NVS_LOOKUP_CACHE`・`CONFIG_NVS_DATA_CRC`・`CONFIG_NVS_INIT_BAD_MEMORY_REGION`・`CONFIG_SETTINGS_NVS_NAME_CACHE` は Zephyr 版と
  同じく無効）を前置きしてビルドします。無効の値を `#if` で見るので `-Wundef` を外し、`nvs.c` は `uint32_t` を `%d` で出す（Zephyr の
  `uint32_t` は `unsigned int`、newlib と NuttX は `unsigned long`。どちらも 32 ビット）ので `-Wformat` を外します。
- flash の上の形と、それを決める値（Zephyr 版の根拠）:

  | 値 | ここ | Zephyr 版 |
  |---|---|---|
  | 領域 | `0xffe000` から `0x2000`（`flash_kv.h`、`<zephyr/storage/flash_map.h>`） | `app.overlay` の `storage_partition` |
  | セクター | 4096 バイト × 2 | `rp2040.dtsi` の `erase-block-size` × `SECTOR_SIZE_MULT` 1、8 KiB に入る数（`settings_nvs.c`） |
  | 書き込み単位・消去値 | 1、0xFF | `flash_rpi_pico.c` の `flash_rpi_parameters`、DT の `write-block-size` |
  | 索引（ATE） | 8 バイト、`crc8_ccitt(0xff, ...)` | `nvs_priv.h`（無改変） |
  | 名前と値の ID | 0x8000、0x8001〜、+0x4000 | `settings_nvs.h`（無改変） |
  | 値 | 校正値 56 バイト、文字列、`imu.session` 4 バイト、資格情報 224 バイト | `imu_cal_codec.c`・`imu_stream.c`（共通）、`struct wifi_credentials_personal`（並びを `BUILD_ASSERT` で Zephyr 版の ELF と同じに固定。arm-none-eabi の 1 バイトの enum） |

- 前提: settings のロック（`K_MUTEX_DEFINE(settings_lock)`、全部 0 から始まるミューテックス）と NVS のロックは互換層の `k_mutex`
  （再帰。`kv wipe` は settings のロックを持ったまま settings を初期化し直す）。初期化は Zephyr と同じく最初の `settings_subsys_init()`
  （失敗したら次の呼び出しでやり直す）。
- **flash の境界**（`port/flash_kv_os.h`。OS ごとに実装する。RP2040 では NuttX 版・FreeRTOS 版・ThreadX 版が
  `common/chip/rp2040/rp2040_flash_kv.c` を共有する。下）:

  ```c
  int flash_kv_os_initialize(void);
  bool flash_kv_os_ready(void);
  int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);
  int flash_kv_os_program(uint32_t page_offset, const uint8_t page[256]);
  int flash_kv_os_erase(uint32_t sector_offset);
  ```

  `initialize` は起動の処理が 1 回呼ぶ（リンカースクリプトの範囲を確かめる。何も消さない）。`offset` は flash の先頭（XIP の 0x10000000）
  からの位置。`ready` が偽のあいだ flash map はデバイスが無い扱い（`-ENODEV`）。`read` は今の flash の中身を返す（書き込み・消去の直後も）。
  `program` は 256 の倍数の位置の 1 ページを書く（書かない所は 0xFF。書き込みはビットを落とすだけ）。`erase` は 4096 の倍数の位置の
  1 セクターを消す。どれも 0 か負の errno を返し、スレッドからだけ、同時に 1 つだけ呼ばれます（互換層がロックを持つ）。呼ぶ側
  （`zephyr_flash_kv.c`）は領域の中の揃った要求しか渡しませんが、OS の実装も範囲を確かめ直し、外なら flash に触れずに `-EINVAL` を
  返します。書き込みと消去の間は割り込みを止め、そのあいだ flash 上のコードを動かしません（NuttX 版はページ 1 回・セクター 1 回ごと）。
- **RP2040 の部分**（`common/chip/rp2040/rp2040_flash_kv.c`。NuttX 版・FreeRTOS 版・ThreadX 版が共有）: `initialize` はリンカースクリプトの
  `__kv_start`・`__kv_end` とイメージの終わりを定数と比べ、そのあと OS の側の flash の操作を準備する。`read` はキャッシュを通らない XIP の
  別名（0x13000000）から写す。`program` はページを RAM に写す。`program` と `erase` は要求を定数と範囲で確かめ、OS の側の flash の操作
  （`common/chip/rp2040/port/rp2040_flash_kv_os.h`）に範囲と一緒に渡す:

  ```c
  int rp2040_flash_kv_os_init(void);
  uintptr_t rp2040_flash_kv_os_image_end(void);
  int rp2040_flash_kv_os_program(uint32_t page_offset, const uint8_t *page, uint32_t lo, uint32_t hi);
  int rp2040_flash_kv_os_erase(uint32_t sector_offset, uint32_t lo, uint32_t hi);
  ```

  OS の側は、flash に触れる直前に割り込みを止めた中で、要求が `[lo, hi)` に収まることをもう一度確かめる。NuttX 版
  （`common/chip/rp2040/port/nuttx/rp2040_flash_kv_os.c`。ボードの `src/Make.defs` が `VPATH` で `libboard.a` に入れる）: `init` はブート ROM の
  flash の関数を探し、boot2 を RAM に写す。`program` と `erase` は、割り込みを止めて RAM 上の関数（`.ram_code.kv_flash_op`。ブート ROM と
  boot2 の写しだけを呼ぶ）を動かす。NuttX 自身のこのチップの flash ドライバー（`rp2040_flash_mtd.c`）は使わない: イメージの後ろの SmartFS の
  領域の MTD デバイスで、最初の起動のときにそこから flash の終わりまでを消す。FreeRTOS 版・ThreadX 版
  （`common/chip/rp2040/port/pico_sdk/rp2040_flash_kv_os.c`）: Pico SDK の `flash_range_program()`・`flash_range_erase()` を `flash_safe_execute()` で
  （リンカースクリプトと割り込みを止める間の扱いは [`design/freertos-port-ja.md`](../../design/freertos-port-ja.md)）。
- `zephyr_flash_kv.c`: flash のデバイス（`z_flash_kv_device`）と flash map。要求は領域の中だけを受け付け（Zephyr 版のドライバーは flash
  全体）、書き込みを 256 バイトのページに分け、消去をセクターごとに境界へ渡します。`flash_area_open()` は起動後に 1 回、領域の中身を判定し、
  Zephyr の NVS でも消去済みでもなければログに出して `-EILSEQ` を返します（settings は初期化されず、何も書かない）。
  `kv wipe confirm` の後にもう一度判定します。

### ソケット層（`net_offload`）

ネットワーク層を持たない OS（FreeRTOS 版と ThreadX 版）のための、互換層のネットワークの部分です。OS に依存しません。

- 互換層は、Zephyr のソケット層をソケットの実装の一覧のところまで持ちます（Zephyr の `sockets.c`・`getaddrinfo.c`・`socket_offload.c`・
  `zvfs_fdtable.c`・`zvfs_poll.c` を縮めた写しで、`net_offload/subsys/net/lib/sockets/` と `net_offload/lib/os/zvfs/` にあります。省いた
  ものはそれぞれの先頭に書いてあります）。ネットワークのインターフェース（`zephyr_net_if.c`）とネットワークの部分の開始
  （`zephyr_net_init.c`、`zephyr_net_initialize()`。インターフェースを Zephyr の `net_init()` と同じく起動時に登録する）もここにあり、
  ヘッダーは `net_offload/include/` の下（`zephyr/net/`、`zephyr/sys/fdtable.h`、`zephyr_net_init.h`）です。
- 実装は `NET_SOCKET_OFFLOAD_REGISTER()` か `NET_SOCKET_REGISTER()` で自分を登録し、リンカースクリプトが登録を優先度の順に並べます
  （`port/pico_sdk/ld/section_extra_post_text.incl`）。`zsock_socket()` は、ファミリーが合い `is_supported()` が受け付ける最初の実装を
  使います。ほかの呼び出しは fd の表から実装の `struct socket_op_vtable` へ fd ごとのミューテックスの下で渡り、`poll` は
  `ZFD_IOCTL_POLL_PREPARE` と `ZFD_IOCTL_POLL_OFFLOAD` を通り、`zsock_getaddrinfo()` は登録された名前解決へ行きます。NINA のソケットは
  Zephyr 版のつなぎを変えずにビルドしたもの（`common/drivers/nina/port/zephyr/nina_socket.c`、優先度 40）で、Zephyr 版と同じく、呼んだ
  タスクで NINA に問い合わせて待ちます（NuttX 版の usrsock と `nina_event` のような、イベントのスレッドはありません）。
- fd の表は 16 個で、空いている最も小さい番号から 0〜15（Zephyr 版と同じく、標準入出力のための 0〜2 は取っておきません）。NINA の
  ソケットは 10 個なので、11 個目は `EMFILE` です。fd の番号は newlib の fd とは別で、`zsock_*()` だけが使います。newlib に無い型と
  定数は `libc_net/include/` にあり、値は Linux と同じです（`EAI_*` は Zephyr の負の値）。`net_offload/include/` の
  `<zephyr/net/net_ip.h>` がそれらに Zephyr の `net_*` の名前を付けます（インクルードパスで `include/` より先にあり、NuttX 版が使う
  `include/zephyr/net/net_ip.h` に代わる）。
- ソケットの動作は Zephyr 版と同じです: モジュールへの poll が失敗したソケットは `POLLERR` になります（呼び出しは失敗しません）。
  `poll` は fd ごとのミューテックスを取るので、ほかのタスクがそのソケットで進めている呼び出し（待っている受信、接続）を待ちます。
  `sendmsg` は 1 つずつ（sleep を含めて）動きます。名前解決の結果は 2048 バイトのヒープから取ります（解放されていない結果が
  約 36 個になると `EAI_MEMORY`）。Telnet のサーバーは、待ち受けのソケットのエラーのあと作り直し、それが失敗する間は 1 秒ごとに
  作り直します。
- Wi-Fi: NINA のコアの Wi-Fi の操作と Zephyr 版と同じそのつなぎ（`common/drivers/nina/port/zephyr/nina_net.c`: Zephyr の Wi-Fi 管理と
  インターフェース、スレッド `nina_wifi_tid`）、資格情報、自動再接続（`common/imu/wifi_profile.c`）を、互換層の `net_if`・`net_mgmt`・
  `wifi_mgmt`（`net_offload/zephyr_net_if.c`）の上で動かします。NINA の側は [NINA-W102 のドライバー](../drivers/nina/README_ja.md)にあります。
- ネットワークの部分は、汎用のカーネルの部分（`kernel_generic/`）にも POSIX のヘッダー（`libc_net/`）にも頼りません（下の「テスト」の
  `net_offload_independent`）。

### シェルのエンジン（`shell_engine`）

自分のシェルを持たない OS（FreeRTOS 版と ThreadX 版）のためのシェルで、互換層の API の上で書いています。

| ファイル | 中身 |
|---|---|
| `cli.c` | 行編集 |
| `shell_engine.c` | コマンドの表と実行と `help` |
| `shell_argv.c` | Zephyr の引数の分割の写し |
| `shell_console.c` | コンソールとスレッドごとの出力先 |
| `getopt_state.c` | セッションごとの getopt |
| `shell_serial.c` | シリアルのセッション（OS が用意するコンソールの経路は `port/shell_serial_os.h` が宣言する） |
| `shell_telnet.c` | Telnet のシェル |

ヘッダーは `include/`（`shell_console.h`・`shell_engine.h`・`shell_serial.h`・`shell_telnet.h`）と `internal/`（`cli.h`・`shell_argv.h`）、
`zshell.c` に対するバックエンドは `common/shell/port/generic/` と `port/generic/zephyr_os_shell.h` です。

- コマンドは Zephyr のシェルの API（`SHELL_CMD_REGISTER()` など）で書き、照合・ヘルプ・ワイルドカード・引数の数の検査は共有の
  `common/shell/zshell.c` が行います（上の「シェルのコマンドの処理」）。コマンドの表と行編集はシェルのエンジン（`shell_engine.c`・`cli.c`。
  `help` も `shell_engine.c`）です。
- コンソール、セッション、Telnet はシェルのエンジンのもので、互換層の `k_mutex`・`k_sem`・`z_thread_static_start()` とスレッドのカスタム
  データ（`k_thread_custom_data_get()`）の上で書いています。コンソール（`shell_console.c`）はセッションの端末（USB と Telnet）で、各タスクを
  1 つのコンソールに結び付けます。タスクの `printf()`・`fprintf(stderr)` はそのコンソールに出ます。結び付いていないタスクの出力は USB に
  出ます。
- Telnet（`shell_telnet.c`。`port/pico_sdk/telnet.cmake` が組み込む）: ポート 23、同時 1 接続。Zephyr の `shell_telnet.c` と同じく、接続しても
  何も送らず、Telnet のコマンドは取り除いて応答せず、エコーしません。Zephyr 版と違えた点はファイルの先頭のコメントにあります
  （2 本目を断る前に 1 本目が閉じていないか確かめ直す、新しいクライアントは空の行から、など）。
- getopt の状態はセッションごとに持ちます（`getopt_state.c`）。USB と Telnet で同時にコマンドを実行できます。
- コンソールの経路（USB）は OS が用意します（下の「Pico SDK」）。

## OS が実装する境界

一覧と、新しい OS が書く順は [`design/os-boundary-ja.md`](../../design/os-boundary-ja.md) にあります（カーネルの部分は 2 節、ほかの境界は
3 節、OS ごとに用意する `<zephyr/...>` ヘッダーは 4.1）。ここでは、互換層の側から見た形を示します。

### カーネルの部分（2 通り）

共通のヘッダーは OS の型を持たず、OS ごとのディレクトリー（NuttX 版は `port/nuttx/`、FreeRTOS 版・ThreadX 版は `port/generic/`、
ホストテストは `common/tests/zephyr_fakes/`）の次のヘッダーを読みます。満たすべきものの一覧は、それを読む共通のヘッダーの先頭の
コメントにあります。

- `zephyr_os.h`（`<zephyr/kernel.h>` が最初に読む）: 互換層より先に要る OS のヘッダー、`Z_USEC_PER_TICK`、`struct k_mutex` と
  `Z_MUTEX_INITIALIZER()`（全部 0 のものが未ロックとして働くこと）、`struct k_sem`・`Z_SEM_INITIALIZER()`・`K_SEM_MAX_LIMIT`、
  `struct _timeout`・`_timeout_func_t`、`k_tid_t`・`k_current_get()`、`k_yield()`、`K_PRIO_PREEMPT()`（Zephyr の優先度を順を保って OS の
  優先度に）、`k_cycle_get_32()`・`k_cycle_get_64()`（1 MHz）、`Z_OS_ASSERT()`、任意で `Z_SEM_TAKE_FOREVER()`（ワークキューの待ち）。関数
  （`k_sem_*()`、`k_mutex_*()`、`k_poll()`、`k_sleep()` など、`z_add_timeout()`・`z_abort_timeout()`、互換層のすべてのスレッドを起こす
  `z_thread_static_start()`）は OS の `.c`。ワークキューのスレッド、`k_sys_work_q`、`k_uptime_get()`、`zephyr_compat_initialize()` は共有
  （`zephyr_init.c`）。
- `zephyr_os_spinlock.h`（`<zephyr/spinlock.h>` が読む）: `struct k_spinlock`、`k_spinlock_key_t`、`k_spin_lock()`・`k_spin_unlock()`。
- `zephyr_os_atomic.h`（`<zephyr/sys/atomic.h>` が名前を定義する前に読む）: 互換層が引き取る名前を持つ OS のヘッダーと、そのマクロの
  `#undef`。ホストテストでは空。
- `zephyr_os_shell.h`（`<zephyr/shell/shell.h>` が読む）: `struct shell`。

これらの書き方が 2 通りあります。

| | 直接に実装する（NuttX 版） | 基本の操作の上に置く（FreeRTOS 版・ThreadX 版） |
|---|---|---|
| 書くもの | `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h` と、上の関数（`port/nuttx/zephyr_kernel.c`・`zephyr_mutex.c`） | `port/zephyr_kernel_os.h` の基本の操作（型と定数 10、関数 26）を `port/<OS>/zephyr_kernel_os_impl.h` に、1 tick に 1 回 `zephyr_compat_tick()` を呼ぶ tick のフック |
| `zephyr_os.h` など | OS のもの（`port/nuttx/`） | 共有の `port/generic/zephyr_os.h`・`zephyr_os_spinlock.h`・既定（空）の `zephyr_os_atomic.h` |
| 関数の実装 | OS のもの | 共有の `kernel_generic/zephyr_kernel.c`・`zephyr_mutex.c` |

汎用の実装は、Zephyr の `k_sem`（Zephyr の起こし方の規則）、セマフォに対する `k_poll()`、遅延ワークのタイムアウトの整列した一覧、スリープ、
スレッドの起動、`k_spinlock`、`k_mutex` を基本の操作の上に作ります。基本の操作は `k_sem_give()`・`k_sem_take()`・`k_poll()`・
`k_spin_lock()` と tick の経路にあるので、OS の部分は `static inline` の関数として定義します（ポインターを通した呼び出しや余分な呼び出しを
しない）。OS の部分は互換層のヘッダーを読みません。一覧とそれぞれに求めることは `port/zephyr_kernel_os.h` の先頭のコメントと
[`design/os-boundary-ja.md`](../../design/os-boundary-ja.md) の 2.5・2.6 にあります。tick のフックの入口は `include/zephyr_compat_os.h`、
周期の tick が割り込みを長く止められて失う tick の追いつきは `include/tick_catchup.h` です。コンパイルが通る最小の形は
`common/tests/kernel_generic_host/` にあります。

### Zephyr で成り立つ前提

互換層は「Zephyr で成り立つ前提」を再現します。前提が抜けると実機でだけ出る不具合になります。例: Zephyr の `k_mutex` は全部 0 のとき
未ロック（`lock_count` 0）で、Zephyr の MQTT ライブラリーは `mqtt_client_init()` の前のクライアントのミューテックスを `mqtt_abort()` で
lock し、`mqtt_client_init()` は接続のたびにクライアントごと 0 で埋めます。settings のロック（`K_MUTEX_DEFINE(settings_lock)`）も全部 0 から
始まります。そのため、どの OS の部分でも、全部 0 の `k_mutex` は未ロックとして働き、OS は未ロックのミューテックスへの参照を持ちません
（各 OS のやり方は下の「各 OS のつなぎ」）。

### ほかの境界

共通の `.c` とヘッダーが読む境界のヘッダー:

| ヘッダー | 中身 | 実装 |
|---|---|---|
| `include/priorities.h` | スレッドの優先度とスタックの表（`APP_STACK_NET_MGMT`・`APP_PRIO_RTIO_WQ`・`APP_STACK_RTIO_WQ` など） | OS の部分 `priorities_os.h` の `APP_PRIO_FROM_ZEPHYR()`（`port/<OS>/`） |
| `port/gpio_os.h` | `zephyr_gpio.c` の下の GPIO バンク 0 | `port/nuttx/rp2040_gpio_os.c`、`port/pico_sdk/gpio_os_rp2040.c` |
| `include/app_log.h` | `<zephyr/logging/log.h>`、`sensor_default_rtio.c`、資格情報のログ | OS の部分 `app_log_os.h`（`port/nuttx/`、`port/generic/`） |
| `port/log_ram_os.h` | RAM ログ（`log/log_ram.c`）の OS の部分 | `log_ram_os_impl.h`（`port/nuttx/`、`port/pico_sdk/`） |
| `port/cpu_load.h` | `<zephyr/sys/cpu_load.h>` の `cpu_load_get()` とその計算 | ボード（アイドル） |
| `port/flash_kv_os.h` | KV の領域の flash（上の「KV」） | RP2040 の共有の部分 `common/chip/rp2040/rp2040_flash_kv.c` |
| `port/shell_serial_os.h` | シェルのエンジンのコンソールの経路 | `port/pico_sdk/shell_serial_os_impl.h` |
| `sensor_shell_os.h` | `sensor_shell_stream.c` の OS の部分 | `common/shell/port/nuttx/`、`common/shell/port/generic/` |

- 全体が OS ごとのヘッダー: `zephyr/sys/clock.h`、`zephyr/sys/__assert.h`、`zephyr/drivers/i2c.h`（NuttX 版は `port/nuttx/`、FreeRTOS 版・
  ThreadX 版は `port/generic/`）、`zephyr/net/socket.h`、`zephyr/net/net_if.h`（NuttX 版は `port/nuttx/`、FreeRTOS 版・ThreadX 版は
  `net_offload/include/`）。
- 構成の値 `CONFIG_LSM6DSO_STREAM`（`zephyr_device.c`）と `CONFIG_WIFI_CREDENTIALS`・`_MAX_ENTRIES`・`_SAE_PASSWORD_LENGTH`・
  `_CONNECT_STORED_CONNECTION_TIMEOUT`（Wi-Fi のヘッダーと `zephyr_wifi_mgmt.c`）は、互換層のヘッダーより先に定義されている必要があります
  （NuttX 版は `<nuttx/config.h>`、共通の `.c` には前置きヘッダー `common_prelude.h` が読ませる）。
- **OS ごとのヘッダー**（共有するソースの）: 共有するソースのうち OS のファームウェアごとに違う数行は、同じ名前の小さなヘッダーに出し、
  各 OS がそれぞれのものを読みます（LSM6DSO のデバイスの `lsm6dso_instance.h` のほかに。[LSM6DSO のドライバー](../drivers/lsm6dso/README_ja.md)）。
  `sensor_shell_os.h`（`sensor_shell.c` と互換層の `sensor_shell_stream.c` が最初に読む）と `wifi_profile_os.h`（再接続のスレッドの
  `wifi_reconnect_tid` の宣言。Zephyr 版 `zephyr/app/usb-console/src/` は `K_THREAD_DEFINE()` が作るとおり `const k_tid_t`、NuttX 版
  `nuttx/apps/wifi/` と `port/generic/` は互換層の `K_THREAD_DEFINE()` がスレッドを始めるときに値を書くので `const` なし）です。

## 各 OS のつなぎ

### Zephyr

互換層を使いません（共有のソースを Zephyr 本体に対してビルドする）。Zephyr 版が持つのは、共有のソースの OS ごとのヘッダーの Zephyr の
ものだけです: `zephyr/app/usb-console/src/kv_store_os.h`（空）、`zephyr/app/usb-console/src/wifi_profile_os.h`、パッチを当てたツリーの
`drivers/sensor/sensor_shell_os.h`（何もしない）。

### NuttX

**ファイル**（`common/compat/port/nuttx/`）: カーネルの部分（`zephyr_kernel.c`・`zephyr_mutex.c`）、OS ごとのヘッダー（`zephyr_os.h`・
`zephyr_os_spinlock.h`・`zephyr_os_atomic.h`・`zephyr_os_shell.h`）、全体が NuttX に依存するヘッダー（`zephyr/net/socket.h`・`net_if.h`、
`zephyr/sys/clock.h`・`__assert.h`、`zephyr/drivers/i2c.h`）と、I2C・インターフェース・チップ ID の `.c`（`zephyr_i2c.c`・`zephyr_net_if.c`・
`zephyr_hwinfo.c`）、前置きヘッダー（`common_prelude.h`）、共通のヘッダー `app_log.h`・`priorities.h`・RAM ログの NuttX の部分（`app_log_os.h`・
`priorities_os.h`・`log_ram_os_impl.h` と syslog のチャンネル `log_ram_syslog.c`）、`gpio0` の下の GPIO バンク 0（`rp2040_gpio_os.c`）。
`.c` は `nuttx/drivers/compat/Make.defs` が `libexternal.a` にビルドし（`log_ram_syslog.c` は `nuttx/drivers/log/Make.defs`。どちらも `VPATH` で、
共通の部分のディレクトリーより先に参照する）、`rp2040_gpio_os.c` はボードの `src/Make.defs` が `libboard.a` にビルドします（チップの内部の
ヘッダーを読むため）。どれも前置きヘッダーを付けません。

**インクルードパス**は NuttX の実装（`common/compat/port/nuttx`。カーネル側・ボード・アプリ）、共通の部分（`common/compat/include`・`port`・
`internal`）の順です（その仕組みを自分で持たない OS のための実装で、Pico SDK の上の OS が使う `common/compat/port/generic` は入れません）。
シェルのコマンドを持つアプリは、シェルのヘッダーも足します（`apps/subcmd` は `common/shell/include`・`port`・`port/nuttx`、`apps/sensor` は
`common/shell/include`・`port/nuttx`、`apps/nina` と `apps/wifi` は `common/shell/include`）。共通の `.c` はカーネル側（`compat/Make.defs`）と
アプリ（`apps/imu`・`apps/sensor`・`apps/wifi`）が `VPATH` で参照し、前置きヘッダーを付けてコンパイルします（`sensor_shell_stream.c` は
`apps/sensor`、資格情報は `apps/wifi`、NVS・settings・CRC-8・16 進の変換・`zephyr_flash_kv.c`・`kv_store_wipe.c` は `apps/imu` がビルドする）。
ホストテストは `common/tests/zephyr_fakes` に偽のカーネル（OS ごとのヘッダーとその関数）を置き、NuttX の実装より先にインクルードパスに
入れます。

| ソース（`common/compat/port/nuttx/`） | 中身 |
|---|---|
| `zephyr_mutex.c` | NuttX の上の `k_mutex`（`rmutex_t`）。Zephyr の `k_mutex` は全部 0 のとき未ロック（`lock_count` 0）で、Zephyr の MQTT ライブラリは `mqtt_client_init()` の前のクライアントのミューテックスを `mqtt_abort()` で lock する。NuttX の全部 0 のミューテックスはスレッド 0 が持っている扱いで、lock が戻らない。そこで `K_MUTEX_DEFINE()` と `k_mutex_init()` が印（`ready`）を立て、印のないもの（全部 0）は最初の lock・unlock で初期化する（互換層のスピンロックの中で 1 回だけ） |
| `zephyr_kernel.c` | NuttX の上の OS 部分: `k_sem`（`sem_t`、上限は `k_sem_give()` で見る）、`k_poll()`、遅延ワークのタイマー（wdog）、スレッド名、スレッドの起動（`z_thread_static_start()`。静的なスタックのカーネルスレッド。起動のデータを新しいスレッドに渡し、受け取るまで呼んだ側が待つ）、`k_sleep()`・`k_msleep()`・`k_usleep()`・`k_busy_wait()` |
| `zephyr_i2c.c` | I2C（NuttX の I2C マスターの上。下） |
| `zephyr_hwinfo.c` | `hwinfo_get_device_id()`。ボードのチップ ID（`rp2040_chip_id()`） |
| `zephyr_net_if.c` | インターフェース（NuttX のネットワークデバイスの値。下） |

- **OS ごとのヘッダーの中身**: NuttX の型（`rmutex_t`・`sem_t`・wdog・`pid_t`）、tick、優先度、TIMER、`k_yield()`（`sched_yield()`）、
  アサーション（`DEBUGASSERT()`）、ワークキューの待ち（`nxsem_wait_uninterruptible()`）、`sched_lock()`・`up_irq_save()`、`<nuttx/atomic.h>` の
  名前の引き取り、`struct shell`（端末のパスとコマンドのタスク）。`k_current_get()` は呼び出したスレッドの NuttX のスレッド ID で、
  `K_THREAD_DEFINE()` の `k_tid_t` と比べられます。`k_timeout_t` の単位は 100 µs の tick です。
- **スピンロック**: Zephyr の 1 CPU の `k_spin_lock()` は割り込みを禁止し、その間はほかのスレッドに切り替わりません（`k_sem_give()` などで
  起きたスレッドは解放のときに動く）。NuttX は割り込み禁止中でも `nxsem_post()` で起きたスレッドにすぐ切り替えるので、NuttX 版の
  `k_spin_lock()`（`zephyr_os_spinlock.h`）は `sched_lock()` と `up_irq_save()` にしています（割り込みハンドラーの中では `sched_lock()` は何も
  しない）。`atomic_*()` もこれで守ります（Cortex-M0+ に排他ロード・ストアは無く、Zephyr の ARMv6-M も割り込み禁止で実装）。
- **NuttX の名前との衝突**: NuttX の `<nuttx/atomic.h>`（多くのカーネルのヘッダーが読む）にも `atomic_t` と `atomic_set()` などがあり、
  意味が違います。互換層の `<zephyr/sys/atomic.h>` はそれを先に読んでから（`zephyr_os_atomic.h`）、そのファイルの中で名前を引き取ります
  （`atomic_t` は `zephyr_atomic_t`、NuttX のマクロは `#undef`）。両方を使うファイルは NuttX のヘッダーを先に読みます（`common/` のファイルは
  前置きヘッダーが読む）。ボードの起動処理は Zephyr のヘッダーを読まず、`zephyr_compat.h` だけを読みます。
- **前置きヘッダー**（`common_prelude.h`）: Zephyr 版と共有するソースと互換層の共通の `.c`（リポジトリの `common/`）を NuttX 版で
  コンパイルするとき、ビルド定義（`apps/imu`・`apps/wifi`・`apps/sensor` の `Makefile`、`compat/Make.defs`、`lsm6dso/Make.defs`、
  `nina/Make.defs`、ボードの `src/Make.defs`）が各ファイルの前に `-include` で読ませます（Zephyr 版がすべてのファイルに
  `-imacros autoconf.h` で構成を読ませるのと同じ形）。中身は `<nuttx/config.h>`、互換層より先に要る NuttX のヘッダー（`<zephyr/kernel.h>` が
  先に読むもの: `<sys/types.h>`、`<nuttx/irq.h>`、`<nuttx/mutex.h>`、`<nuttx/semaphore.h>`、`<nuttx/wdog.h>`、`<stdio.h>`）と、共有する
  ソースが使う Zephyr の Kconfig の値（`CONFIG_SENSOR_LOG_LEVEL` 3。Zephyr 版の構成と同じ値。`lsm6dso_sensor.c`・`lsm6dso_sensor.h` の
  `CONFIG_SENSOR_ASYNC_API` を、`CONFIG_LSM6DSO_STREAM` があれば定義する）です。そのため共有するソースには NuttX のヘッダーの行が
  ありません。ソケットのヘッダーは入れていません（`accept()` などの名前がソースの関数と重なる。必要なファイルは互換層の
  `<zephyr/net/socket.h>` が読む）。ホストテストも同じヘッダーを付けます（`__NuttX__` が無いと Kconfig の値だけ。`time_sync` と LSM6DSO の
  ドライバーのテストは偽の `<nuttx/config.h>` を先に付ける）。
- **OS ごとのヘッダー**（共有するソースの）: NuttX 版の `sensor_shell_os.h`（`common/shell/port/nuttx/`。ビルドは `nuttx/apps/sensor`）は
  Zephyr 版の構成の値とログを読み、`sensor get` が結果の表示を待つ 2 行と処理スレッドのコールバックを関数にします。
- **ワーク**: キューのスレッドの `k_yield()` は `sched_yield()`、遅延のタイマーは wdog です。
- **RTIO のワークキュー**: 優先度 0 は NuttX の 118 です。NuttX はタスク名の空白を `_` にするので、`ps` では `RTIO_WQ_0` と出ます。
- **起動**: `zephyr_compat_initialize()` はボードの起動処理が呼びます。システムワークキューは優先度 119、スタック 1024 です。
- **静的なスレッド**: init が `K_THREAD_DEFINE_START(name)` で始めます。スレッドは NuttX のカーネルスレッド（`kthread_create_with_stack()`）です。
- **GPIO**: `rp2040_gpio_os.c` が pico-sdk の `gpio_set_pulls()`・`gpio_set_function()`・`gpio_set_dir()`・`gpio_set_irq_enabled()` と同じ
  レジスター操作をします（イベントを有効・無効にする前に `INTR` のそのピンのイベントを消す）。割り込みは NuttX の `rp2040_gpio` のバンクの
  割り込みから来ます（有効なイベントを消してからピンのハンドラーを呼ぶ）。
- **時間**: `k_uptime_get()` は Zephyr の `k_ticks_to_ms_floor64(k_uptime_ticks())`（tick 数を ms に切り捨て）と同じ値です。NuttX の tick は
  ボードの TIMER を 100 µs で割ったもの（tickless のタイマー）なので、TIMER を 1 回読んで 1000 で割ります（1000 が tick の倍数なので同じ値。
  ホストテストで確認）。64 ビットで、32 ビットの `clock_systime_ticks()` のような約 5 日での一周はありません。`k_sleep()` は `K_NO_WAIT` で
  `sched_yield()`、それ以外は `nxsched_ticksleep()` で「今の tick + tick 数 + 1」まで眠ります（Zephyr の `z_add_timeout()` と同じ切り上げ）。
  `k_wakeup()` が無いので戻り値は常に 0 です。`k_msleep()`・`k_usleep()` は NuttX の `nxsched_msleep()`・`nxsched_usleep()` で、時間を tick に
  切り上げ、`k_sleep()` と同じ時点に起きます（Zephyr と同じ規則）。0 以下は Zephyr と同じく `sched_yield()`、シグナルでは終わらず、戻り値は
  0 です。`k_busy_wait()` は TIMER を見て待つ `up_udelay()`（ボードの `rp2040_oneshot.c`）です。この 3 つは LSM6DSO のドライバーが使います。
- **`hwinfo_get_device_id()`**: Zephyr 版の RP2040 のドライバー（`hwinfo_rpi_pico.c`、`CONFIG_HWINFO_RPI_PICO_CHIP_ID` なし）と同じく、
  外付け flash の 64 ビットの unique ID（コマンド 0x4B）を flash が送る順のまま返します（最大 8 バイト）。読み出しはボードが起動時に 1 回
  行い（`rp2040_chipid.c`）、ここではその写しを返します。ボードがまだ読んでいなければ `-EAGAIN` です（Zephyr のドライバーは失敗しない）。
- **シェル**: 書き先 `sh` の扱い:
  - `sh` が NULL: 呼んだタスクの標準出力（`shell_print()`・`shell_info()`）と標準エラー（`shell_warn()`・`shell_error()`）。
    `common/shell/include/subcmd.h` で表を書いたコマンド（`kv`、`nina` など）は NULL を渡します。
  - `sh` がバックエンド: `SHELL_CMD_REGISTER()` で登録したコマンド（`sensor`、`imu`、`ntp`）が受け取るもので、NuttX 版のバックエンド
    （`common/shell/port/nuttx/zshell_os_impl.h`。`struct shell` は `zephyr_os_shell.h`）がコマンドの標準出力の端末ごとに 1 つ持ちます（USB は
    `/dev/ttyACM0`、Telnet は `/dev/telnet0`。`F_GETPATH`、Telnet の装置はそれをソケットに渡してしまうので `/dev` の中から `fstat()` の i-node
    番号で探す）。Zephyr のシェルのインスタンスと同じく、起動中ずっと残ります。そのコマンド自身が書くとき（実行中のタスク）は標準出力・
    標準エラーに、ほかのタスク・スレッドが書くとき（sensor シェルの処理スレッドが、コマンドが戻った後に書くとき）は、1 回ごとに端末を
    開いて 1 回の `write()` で書き、閉じます。Telnet のクライアントがいない間は開けないので捨て、次のクライアントに続きます（Zephyr の
    Telnet のバックエンドと同じ）。Telnet のセッションの装置を書く間しか開いていないので、セッションの終わり（最後の記述子が閉じたとき）と
    次の接続を妨げません。
  - `SHELL_CMD_REGISTER()` が組み込みコマンドの入口を作ります。入口は既定では `main()` で、アプリの main のソース（`MAINSRC`）がそのまま
    組み込みコマンドになります（`sensor_shell.c`、`imu_stream.c`）。main のソースでないファイルは、ビルド定義がそのファイルにだけ
    `Z_SHELL_CMD_NAMED_MAIN` を付けてコンパイルし、入口は `int z_shell_main_<名前>(int argc, char *argv[])` になります。アプリの `main()` が
    それを呼びます（`time_sync.c` の `ntp`: `apps/imu/Makefile` が `time_sync.c` に付け、`apps/ntp/ntp_main.c` の `main()` が
    `z_shell_main_ntp()` を呼ぶ）。語を分けるのは nsh で、Tab 補完も nsh のものです。コマンドごとのタスクが、そのセッションの端末を標準出力・
    標準エラーに持ちます。
- **ネットワーク**: `zsock_inet_pton()`・`zsock_inet_ntop()` は C ライブラリの `inet_pton()`・`inet_ntop()` です。ネットワークが無効な間は、
  `imu` が `CONFIG_LIBC_IPv4_ADDRCONV` を選んで IPv4 の変換を入れます。`zsock_getaddrinfo()`・`zsock_freeaddrinfo()` は NINA の名前解決
  （`nina_dns_getaddrinfo()`。エラーは NuttX の `EAI_*`）です。ソケットは NINA の usrsock で、`zsock_socket()`・`zsock_connect()`・`zsock_send()`・
  `zsock_sendmsg()`・`zsock_sendto()`・`zsock_recv()`・`zsock_poll()`・`zsock_close()` は NuttX の関数（`socket()` … `poll()`・`close()`）、
  `ZSOCK_MSG_DONTWAIT`・`struct zsock_pollfd`・`ZSOCK_POLLIN` などは `MSG_DONTWAIT`・`struct pollfd`・`POLLIN` です。ブロッキングの接続と送信の
  待ちと期限は NINA のドライバーのもので、受信と `poll()` は `nina_event` のイベントを待ちます（[NINA-W102 のドライバー](../drivers/nina/README_ja.md)の
  usrsock の説明）。`poll()` の結果は、Zephyr 版の NINA の `poll()`（`nina_socket.c` の `do_poll()`）と 2 つの場合に違います: 接続していない
  TCP のソケットは `ZSOCK_POLLIN`・`ZSOCK_POLLOUT`・`ZSOCK_POLLHUP`（Zephyr 版は `ZSOCK_POLLHUP` だけ）、接続済みの TCP で NINA のエラーフラグか
  NINA の poll の失敗は `ZSOCK_POLLHUP`（usrsock の `REMOTE_CLOSED`。Zephyr 版は `ZSOCK_POLLERR`）です。`zsock_poll()` を受信の確認（100 ms
  ごと、待ち時間 0）と CONNACK の待ち（5000 ms）に使う `imu_raw_mqtt.c` は、どちらでも接続を終えます。
- **インターフェース**（`net_if.h`）: `struct net_if` は NuttX のネットワークデバイス 1 つです。インターフェースは NINA の `wlan0` 1 つで、
  `net_if_get_default()` と `net_if_get_wifi_sta()` はどちらもそれを返します（`nina_wifi_mgmt.c` が登録する前は NULL）。アドレスは 1 つで、
  NINA のつなぎ（`common/drivers/nina/port/nuttx/nina_netdev.c`）が書きます。常に「優先」（IPv4 の重複検出は Zephyr 版の構成でも無い）なので、
  `net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)` はアドレスがある間それを返します。値の読み出しはネットワークデバイスのロック
  （`netdev_lock()`。`ifconfig` の ioctl と同じ）の下で行います。`net_in_addr` は NuttX の `in_addr` です。インターフェースのイベント
  （`NET_EVENT_IF_UP` など、アドレスの追加・削除）は出しません（移植の中で使うものが無い）。
- **`net_mgmt` と Wi-Fi の要求**: `CONFIG_NINA_NET` のときだけビルドし、NINA のつなぎ（`common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`）が
  コアの上でアプリに提供します。`net_mgmt` のスレッドの優先度 −1 は NuttX の 119 です。`nina_net_initialize()` がインターフェースより先に
  スレッドを始めます（`nina_wifi_mgmt_initialize()`）。NuttX 版のインターフェースは接続だけを持ちます（スキャン・切断・状態は wapi がコアを
  直接呼ぶ）。
- **I2C**（`zephyr_i2c.c`、`zephyr/drivers/i2c.h`）:
  - I2C のバスは、NuttX の I2C マスターと速さを持つデバイスです（`i2c_compat_bind()` でボードが結び付ける）。Zephyr ではバスのドライバーが
    速さを持ち、`i2c_configure()` が以後のすべての転送の速さを変えます。NuttX の転送はメッセージごとに周波数を持つので、互換層の転送
    （`i2c_burst_read_dt()` など）がデバイスの速さをメッセージに入れます。`i2c speed` もこの速さを変えるので、LSM6DSO のドライバーの転送の
    速さが変わります。`i2c_get_config()` は `I2C_MODE_CONTROLLER` とこの速さの区分を返します（Zephyr 版・FreeRTOS 版・ThreadX 版の
    ドライバーには `get_config` が無く `-ENOSYS`。`i2c speed` の結果は同じ）。NuttX のマスターを直接使うものがあれば、自分の周波数で動きます。
  - `i2c_transfer()` は、Zephyr と同じく最後のメッセージに STOP を足し、各メッセージを NuttX のフラグにします（ボードのドライバーの
    `i2c_dw_msg_flags()` の逆: `I2C_MSG_READ` は `I2C_M_READ`、STOP の無いメッセージは `I2C_M_NOSTOP`、STOP の無いメッセージの次で RESTART の
    無いものは続き `I2C_M_NOSTART`）。1 回で 4 個まで（それより多いと `-EINVAL`）。`i2c_recover_bus()` はマスターの `I2C_RESET()` で、ボードの
    ドライバーは `-ENOTSUP` を返します（Zephyr 版のドライバーと同じ。`i2c recover` は失敗を表示する。エラー番号は NuttX の値）。
  - `i2c_configure()` は Zephyr の RP2040 向けドライバーが受け付ける区分（100 kHz、400 kHz、1 MHz）だけを受け付け、ほかは何も変えずに
    `-EINVAL` を返します。モードのビットは見ません（Zephyr のドライバーもコントローラーに固定）。DMA の不具合で I2C0 が隔離されているとき、
    Zephyr の `i2c_configure()` は `-EIO` を返しますが、互換層は速さを記録して 0 を返し、次の転送が `-EIO` になります。
  - NuttX の `<nuttx/i2c/i2c_master.h>` には同じ名前の `i2c_write()`・`i2c_read()` と、Hz の値の `I2C_SPEED_*` があります。互換層では、
    バスのデバイスに対する Zephyr の `i2c_write()`・`i2c_read()`・`i2c_write_read()` を `i2c_compat_write()`・`i2c_compat_read()`・
    `i2c_compat_write_read()` という名前にしました（`i2c_write_read()` と `_dt` の付く形は Zephyr の名前のまま）。NuttX のヘッダーを読まない
    ファイル（`i2c` コマンド）では、`i2c_write()`・`i2c_read()` も Zephyr の名前で使えます。`<zephyr/drivers/i2c.h>` を読むファイルでは
    `I2C_SPEED_*` は Zephyr の区分です（両方読むなら NuttX のものを先に）。
  - ドライバーそのものは [I2C のドライバー](../drivers/i2c_dw/README_ja.md)にあります。

NuttX 版のポートのほかの部分は [`design/nuttx-porting-ja.md`](../../design/nuttx-porting-ja.md) にあります。

### Pico SDK（FreeRTOS・ThreadX で共有）

FreeRTOS 版と ThreadX 版は、カーネルの部分に汎用の実装（`kernel_generic/`）、ネットワークにソケット層（`net_offload/`）と
`libc_net/`、シェルにシェルのエンジン（`shell_engine/`）を使い、次の共有の部分の上で動きます。

| 場所 | 内容 |
|---|---|
| `port/generic/` | その仕組みを自分で持たない OS のための部分: `zephyr/drivers/i2c.h`（ドライバーのデバイスの上の Zephyr の I2C の API）、`zephyr/sys/clock.h`、`zephyr/sys/__assert.h`、前置き（`common_prelude.h`）、`app_log_os.h`（RAM ログの大きさ `LOG_RAM_SIZE` も）、`wifi_profile_os.h`、汎用のカーネルの部分の `zephyr_os.h`・`zephyr_os_spinlock.h`・既定の `zephyr_os_atomic.h`、シェルのエンジンの `struct shell`（`zephyr_os_shell.h`） |
| `port/pico_sdk/` | Pico SDK の上の部分: 互換層・シェル・RAM ログのビルド（`compat.cmake`）、KV のビルド（`kv.cmake`。フラッシュの操作は `common/chip/rp2040/port/pico_sdk/`）、Wi-Fi の組み込み（`wifi.cmake`: NINA のコアの `common/drivers/nina/nina_wifi.c` と、その Zephyr へのつなぎ `common/drivers/nina/port/zephyr/nina_net.c`、資格情報、自動再接続、`ifconfig` の `common/shell/ifconfig_shell.c`）、Telnet のシェルの組み込み（`telnet.cmake`）、`zperf` の組み込み（`zperf.cmake`）、ソケットの実装の登録のためのリンカースクリプトの部分（`ld/section_extra_post_text.incl`）、壁時計（`zephyr_sys_clock.c`）、OS で共通のポートのヘッダーの部分（`zephyr_os_port_rp2040.h`: `z_os_busy_wait_us()`、`z_os_tick_now_us32()`、`z_os_tick_pend()`）、GPIO バンク 0（`gpio_os_rp2040.c`）、チップ ID（`zephyr_hwinfo.c`）、シェルのエンジンに対するコンソールの経路（`shell_serial_os_impl.h`: Pico SDK の USB CDC ACM のドライバー）と Pico SDK の stdio の差し替え（`shell_stdio.c`・`shell_stdio.h`）、newlib の FILE 関数の差し替え（`stdio_wrap.c`）、RAM ログの部分（`log_ram_os_impl.h`: リングのセクション、割り込みの禁止、TIMER） |

- **時計**: `k_uptime_*()`・`k_cycle_*()`・`time_sync_mono_us()` は RP2040 の TIMER（1 MHz、`rp2040_timer_us()`）から作り、tick から作りません。
- **コンソール**: Pico SDK の stdio のドライバーの差し替え（`shell_stdio.c`）と newlib の FILE 関数の `--wrap`（`stdio_wrap.c`）で、出力の経路を
  すべてシェルのエンジンのコンソールに通します。USB は、エンジンのシリアルのセッション（`shell_serial.c`、タスク `console`）を、コンソールの
  経路（`shell_serial_os_impl.h`。宣言は `shell_serial_os.h`: stdio の行き先、入力の到着、回線が開いているか、読み、書き）の上で動かします。
  端末がポートを開くと `common/imu/port/pico_sdk/usb_console.c` のバナーのあとにプロンプトを出します。端末が読まないときは Pico SDK の
  ドライバーが 64 バイトのバッファーの空きを最長 500 ms 待ち、残りを捨てます。
- **Pico SDK の上の OS だけがビルドする共有ファイル**: `zephyr_wifi_shell.c`（`wifi`）、`zephyr_zperf_shell.c`・`zperf_iperf3.c`・
  `internal/zperf_iperf3.h`（`zperf`）、`log/fault_record.c`・`include/fault_record.h`（異常停止の記録の形式）、`common/shell/dmesg_shell.c`
  （`dmesg`）、`common/shell/kernel_shell.c`（`id`・`uptime`・`date`・`sleep`・`devmem`・`fault`・`reboot`・`bootsel`。境界は
  `common/shell/port/kernel_shell_os.h`、実装は `common/shell/port/<OS>/kernel_shell_os_impl.h`）、`common/shell/ifconfig_shell.c`（`ifconfig`）。
  RAM ログ `log/log_ram.c`（リングは `log_ring.c`）は NuttX 版と共有します。
- RAM ログと異常停止、起動の段階、アイドル、RAM に置くコードは [`design/freertos-port-ja.md`](../../design/freertos-port-ja.md) にあります。

### FreeRTOS

**ファイル**（`common/compat/port/freertos/`）: 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）、優先度の対応
（`priorities_os.h`）、ポートのヘッダー（`rp2040/zephyr_os_port.h`: `z_os_in_isr()`、`z_os_yield_from_isr()`、`z_os_task_lock()`・`_unlock()`）。
シェルの汎用のコマンドの OS の部分は `common/shell/port/freertos/kernel_shell_os_impl.h`（`uptime` の tick の数も）です。

カーネルの部分は、FreeRTOS が用意する基本の操作の上の汎用の実装（`kernel_generic/zephyr_kernel.c`・`zephyr_mutex.c`。作りはそのコメント）です。
Zephyr の前提のうち、次のものを再現しています。

- **ミューテックス**（FreeRTOS の再帰ミューテックスの上の `zephyr_mutex.c`）: 再帰可能、FreeRTOS の優先度継承あり。全部 0 の `k_mutex` は
  未ロックとして扱い、最初の lock で作ります（Zephyr の MQTT ライブラリーと settings がこれに頼る）。戻り値は Zephyr と同じ。
- **セマフォと `k_poll`**: Zephyr の規則（give は最も優先度の高い待ち手に渡す、`k_sem_reset()` は待ちを `-EAGAIN` で終える）を互換層の数で
  作り、待ちはタスク通知（番号 1。0 は Pico SDK とアプリ用）で行います。割り込みからの give は、ハンドラーの終わりで切り替えを要求し、
  起こしたタスクがすぐ走ります。
- **静的なスレッド**: `K_THREAD_DEFINE()` のタスクは、起動のタスク（`main`）が共有のアプリの表から始めます。互換層のスレッドも同じ方法で
  始めます（`z_thread_static_start()`: 静的なスタックのタスク。関数が戻ると削除）。
- **スレッドのカスタムデータ**（`k_thread_custom_data_get()`）は、FreeRTOS のスレッドローカルのポインターです。

近似のまま残しているもの:

| 項目 | Zephyr | FreeRTOS 版 |
|---|---|---|
| 優先度 | 小さいほど先、協調 -16〜-1、横取り 0〜14 | `20 − Zephyr`（横取り 0〜14 が 20〜6、協調 -1 が 21）。`Tmr Svc` は 31、アイドルは 0 |
| 協調スレッド | 横取りされない | 高い優先度で近似（より高いタスクには横取りされる。NuttX 版と同じ） |
| 優先度継承 | 持ち主の連鎖をたどり、unlock ごとに戻す | 1 段だけ。持っているミューテックスをすべて離したときに戻す |
| 同じ優先度の選び方 | 横取りされたスレッドが続きを走る。時分割 20 ms（そのスレッドに切り替わってから数える） | FreeRTOS 標準: 切り替えのたびに同じ優先度の次のタスクを順に選ぶので、横取りされたタスクは同じ優先度のほかの準備できたタスクの後になる。時分割は tick フック（`freertos/app/usb-console/src/tick.c`）で、20 tick ごとの決まった刻み（切り替えから数えない） |
| tick と sleep | 100 µs。相対の待ちは「tick に切り上げ + 1 tick」 | 1 ms（`configTICK_RATE_HZ` 1000、32 ビット）、同じ丸め（1 ms より短い待ちは 1〜2 ms になる）。周期の SysTick（tickless ではない） |

tick の一周、割り込みを止めたあとの tick の追いつき（`tick.c`、判定は `include/tick_catchup.h`）、Pico SDK の `sleep_ms()` を使わないことは
[FreeRTOS 版のポートの「tick」](../../design/freertos-port-ja.md#tick)にあります。

タスクの一覧、tick フック、起動、FreeRTOS 版のほかの部分は [`design/freertos-port-ja.md`](../../design/freertos-port-ja.md) にあります。

### ThreadX

**ファイル**（`common/compat/port/threadx/`）: 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）、そのロックの変数
（`zephyr_kernel_os.c`）、優先度と時分割（`priorities_os.h`）、ボードのポートのヘッダー（`rp2040/zephyr_os_port.h`）。シェルの汎用のコマンドの
OS の部分は `common/shell/port/threadx/kernel_shell_os_impl.h` です。

基本の操作の一覧とそれぞれに求めることは `port/zephyr_kernel_os.h` と [`design/os-boundary-ja.md`](../../design/os-boundary-ja.md) の 2.5・2.6 に
あります。ThreadX 版の作りは次のとおりです。

- **スレッドのロック**（`z_os_task_lock()`）: 割り込みを止め（PRIMASK）、いちばん外のロックで ThreadX の `_tx_thread_preempt_disable` を
  上げます。解除では割り込みを止めたまま下げ、マスクを戻してから `_tx_thread_system_preempt_check()` で、ロックの中のサービスが残した
  切り替えを行います。割り込みを止めるだけでは足りません。ThreadX の Cortex-M のポートは、サービスがもっと急ぐスレッドを実行可能に
  したとき、PendSV を立てて割り込みを一瞬許して切り替えるので、ロックの途中で切り替わってしまいます。どのサービスも切り替えの前に
  `_tx_thread_preempt_disable` を見るので（ThreadX 自身も自分の手順のまわりで使う）、これで「解除まで切り替えない」が成り立ちます。
  ロックの中で待つことは無いので、深さと保存したマスクは全スレッドで 1 組です。ISR のロックは PRIMASK だけです。
- **起こしの印とスタックのあとの領域**: `K_THREAD_STACK_DEFINE()` は各スタックのあとに 272 バイト（`Z_OS_THREAD_RESERVED`）を取り、
  そこに `struct z_tx_thread` を置きます: `TX_THREAD`（実行時間のキットの分を含めて 200 バイト）、起こしの印の `TX_SEMAPHORE`（28）、
  スレッドの関数と引数（8）、名前の配列（32）。起こしの印は上限 1 の 2 値セマフォで（`tx_semaphore_ceiling_put(…, 1)`）、待つ前の起こしは
  残ります。印はこの領域にあるので、互換層を通して待つスレッドはすべて `z_os_thread_start()` で始めます（起動のスレッド "main" も）。
  ThreadX の API で直接作ったスレッドは `k_sem_take()` や `k_poll()` を呼べません。
- **スレッドの開始**: 起こしの印を作り、`tx_thread_create()` を止めたまま（`TX_DONT_START`）呼び、`*tid` を入れてから `tx_thread_resume()` します。
  したがってスレッドは `*tid` が入るまで走りません（呼んだ側より急げば resume のときにすぐ走る）。スケジューラーの前からも呼べます。
- **スレッドの終わりと再利用**: `z_os_thread_exit()` は自分を `tx_thread_terminate()` します。ThreadX は終わったスレッドを作成済みの一覧に
  `TX_TERMINATED`（関数が戻ったときは `TX_COMPLETED`）のまま残し、持っていたミューテックスを解放します。同じスタックで次にスレッドを始めると、
  `z_os_thread_start()` がその記録と起こしの印を消してから作ります。まだ終わっていないスレッドのスタックでは `-EBUSY` です。
- **ミューテックス**（`k_mutex`）: `TX_MUTEX` 1 つ（52 バイト）で、優先度継承あり（`TX_INHERIT`）、持ち主には再帰。全部 0 は未ロックで、最初の
  lock で作ります（ロックの中で作るので、2 つのスレッドが同時に作ることはない）。作った直後に ThreadX の作成済みのミューテックスの一覧
  （`_tx_mutex_created_ptr`、`_tx_mutex_created_count`）から外し、削除はしません。アプリは使ったあとのミューテックスを 0 で埋めることがあり
  （`mqtt_client_init()` が接続のたびにクライアントごと埋める）、一覧につながったままだと一覧が壊れるためです。外したミューテックスを ThreadX が
  参照するのは、持ち主がいるあいだ（持ち主の持っているミューテックスの一覧）と待ち手がいるあいだだけで、Zephyr と同じです。
  ThreadX は待ち手がタイムアウトしても持ち主の優先度を解放まで下げないので、タイムアウトのとき `z_os_mutex_lock()` が ThreadX の解放と同じ計算で
  持ち主の優先度を求め、`_tx_mutex_priority_change()` で戻します（Zephyr の `kernel/mutex.c` の規則。受け入れ試験の `mutex_inherit` が確かめる）。
  継承は 1 段で、持ち主は解放のたびに、まだ持っているミューテックスの待ち手の最高（無ければ自分の優先度）に戻ります。
- **優先度**（`priorities_os.h`）: 互換層の値は 15 − Zephyr の優先度（大きいほど急ぐ）で、ThreadX の優先度はその 31 − 値、つまり Zephyr の
  優先度 + 16 です。Zephyr の 32 段（−16〜15）が ThreadX の 32 段（0〜31）に 1 対 1 で対応します。Zephyr のアイドルの 15（ThreadX の 31）は
  互換層のスレッドに使いません。カーネル自身のスレッドはありません（アイドルスレッドもタイマーのスレッドも無い）。
- **時分割**: Zephyr の優先度 0 以上（互換層の値 15 以下）のスレッドには、`z_os_thread_start()` が ThreadX の `time_slice` 20 tick を付けます
  （Zephyr 版の `CONFIG_TIMESLICE_SIZE=20`、`CONFIG_TIMESLICE_PRIORITY=0` と同じ）。協調の優先度には付けません。ThreadX は横取りされたスレッドの
  残りの時間を保ち、時間を使い切ったときに新しい時分割を始めます（Zephyr は切り替えのたびに始め直す）。
- **時計**: `k_cycle_*()`・`k_uptime_*()` は RP2040 の TIMER（`rp2040_timer_us()`）で、tick から作りません。
- **アサーション**: `Z_OS_ASSERT()` は `app_assert_failed()`（異常停止の記録と再起動）です。ThreadX の API のエラー検査は有効のままで、
  間違った呼び出しはエラーの値を返すだけで、異常停止にはなりません。

ThreadX の内部（`_tx_thread_preempt_disable` など）を使うところの一覧、tick、アイドル、異常停止、スレッドの一覧は
[`design/threadx-port-ja.md`](../../design/threadx-port-ja.md) にあります。

## テスト

ホストテストは `common/tests/run.sh` にあり、各 OS の `script/host_tests.sh` が自分のテストの前に動かします。実機は要りません。

- **偽のカーネルの上の互換層**（`common/tests/zephyr_fakes`: OS の部分 `zephyr_os*.h` とその関数）: RTIO（プール、完了、multishot、キャンセル、
  ワークキュー）の `zephyr_rtio`、ワーク（submit、schedule、reschedule、cancel）の `zephyr_work`、最小の MQTT クライアント（`common/imu/mqtt_min.c`）の
  `mqtt_min`。`nuttx/script/host_tests.sh` は、Zephyr のツリーがあれば MQTT のテストを Zephyr 自身のライブラリーでも動かします。
- **tick の追いつき**（`tick_catchup`）: `include/tick_catchup.h` の 1 歩と、SysTick とその保留のビットと割り込みの禁止のモデルの上での動き。
- **ヘッダーの帰属**: `include/`・`internal/`・`port/`・`port/generic/` の各ヘッダーが、互換層の全体か 1 つの部品に属すること（上の
  「ディレクトリー」）。続けて `kernel_generic_independent` が、汎用のカーネルの部分（`<zephyr/kernel.h>`・`<zephyr/spinlock.h>`・
  `zephyr_compat_os.h`、`kernel_generic/` のソース、`zephyr_heap.c`・`zephyr_work.c`・`zephyr_init.c`）が、OS の基本の操作の最小の形
  （`common/tests/kernel_generic_host`）と `port/generic` だけでコンパイルでき、RAM ログ・シェルのエンジン・`net_offload`・`libc_net` のヘッダーを
  読まないことを、`net_offload_independent` が、ネットワークの部分が直接に書いたカーネルの部分（偽のカーネル）の上で、`kernel_generic`・
  `port/generic`・RAM ログ・シェルのエンジン・`libc_net` なしでコンパイルできることを確かめます（`cc -M`）。
- **そのほか**: Zephyr の `fnmatch()` の写しでのサブコマンドの照合（`subcmd_fnmatch`）、getopt の写し（`sys_getopt`。2 つの状態で 2 つの
  コマンド行を交互に解析する）、RAM ログのリングと RAM ログ（`log_ring_*`・`log_ram_*`。FreeRTOS 版の 4096 と NuttX 版の 8192 の大きさで）、
  CPU 負荷の計算（`cpu_load`）、異常停止の記録（`fault_record`）、16 進の変換（`hex`）、KV の領域の補助（`flash_kv`）、KV のストア（`kv_settings`:
  `common/imu/kv_store.c` を NVS と settings の写し、`zephyr_flash_kv.c`、`kv_store_wipe.c` と偽の flash の上で。flash の上のバイト列、読み書き、
  上限、ガベージコレクション、別のデータ、デバイスの拒否、一連の操作のどの flash の操作で電源が切れても）。
- **Zephyr のファイルの写し**（`zephyr_copies`）: Zephyr のツリー（`zephyr/zephyr`、v4.5.0）があるとき、NVS・settings・CRC-8・16 進の変換・
  `fnmatch()`・getopt・`wifi_credentials_shell.c` とそのヘッダーの 23 ファイルが元と同じことを `cmp` で確かめます。`wifi_credentials.c` と
  `wifi_credentials_backend_settings.c` は変更のある写しです（それぞれの先頭に書いてある）。
- **互換層のカーネルの受け入れ試験**（`common/tests/zephyr_kernel/`）: OS に依存しない `zephyr_kernel_test.c`。OS の部分とプラットフォームの
  部分が用意するものは `zephyr_kernel_test.h`、基本の操作の上の項目は `kernel_generic/zephyr_kernel_test_os.h` にあります。OS の部分の上で
  動かし、FreeRTOS 版は `freertos/tests/compat/`（FreeRTOS の POSIX ポートの上）、ThreadX 版は `threadx/tests/compat/`（ThreadX の Linux ポートの上）
  です。
