# Zephyr 版のポートの内部

[English](zephyr-port.md) | 日本語

このリポジトリーのファームウェアは、同じ機能を Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上で
それぞれビルドしたもので、それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。この文書は Zephyr 版
（`zephyr/app/usb-console`）のポートの内部、つまり 4 つの OS で共有するソース（`common/`）の外で Zephyr 版だけが持つものの説明です。
共有のドライバーの中身はそれぞれのドライバーの説明にあり、ここには書きません。使い方はルートの [README](../README_ja.md)、
4 つの OS のファームウェアが共有するソースと OS の境界は[設計書](imu-raw-mqtt-design-ja.md)の §9 と
[OS の境界](os-boundary-ja.md)にあります。

## ドライバー

`zephyr/app/usb-console` は、Zephyr のドライバーの一部を Zephyr のツリーへのローカルのパッチで変えて使います。IMU の FIFO を
高いレートで読み出すための変更と、NuttX 版・FreeRTOS 版・ThreadX 版と共有する LSM6DSOX と NINA-W102 のドライバーがあります。

| ドライバー | 内容 | 元 | 説明 |
|---|---|---|---|
| LSM6DSO（`lsm6dso`） | FIFO ストリーム、校正値を使う変換、静止時の角速度のバイアス | `common/drivers/lsm6dso/`（Zephyr の `drivers/sensor/st/lsm6dso` の代わり） | [LSM6DSO のドライバー](../common/drivers/lsm6dso/README_ja.md) |
| sensor シェル（`drivers/sensor/sensor_shell.c`） | 修正と LSM6DSO の FIFO の属性 | Zephyr v4.5.0 に `zephyr/0006-sensor-shell-fifo-attributes.patch` | [LSM6DSO のドライバー](../common/drivers/lsm6dso/README_ja.md) |
| DesignWare I²C（`drivers/i2c/i2c_dw.c`） | RP2040 の FIFO の段数、I2C0 の DMA 読み出し | Zephyr v4.5.0 に `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`（`i2c scan` は `0004`） | [DesignWare I2C のコア](../common/drivers/i2c_dw/README_ja.md) |
| NINA-W102（`nina`） | SPI 経由の Wi-Fi とソケット | `common/drivers/nina/` | [NINA-W102 のドライバー](../common/drivers/nina/README_ja.md) |
| PL022 SPI（`drivers/spi/spi_pl022.c`） | DMA のモードのしきい値（NINA の SPI1 のため） | Zephyr v4.5.0 に `zephyr/0007-spi-pl022-dma-min-len.patch` | 下の「PL022 の SPI の DMA のモード」 |

Zephyr v4.5.0 はコミット `41d10b59730406bbab4ca9f1fbbe9e96aafb4966` です。各パッチは、そのドライバーに要るものを
すべて Zephyr のツリーの中に持ちます（下の「Zephyr へのローカルのパッチ」）。アプリの `CMakeLists.txt` はパッチを当てた
ドライバーのライブラリーにソースもインクルードパスも足しません。

Zephyr の API で書いたもの（シェル、ソケット、カーネル）は Zephyr のものをそのまま使うので、Zephyr 版は互換層
（ほかの OS の上で Zephyr の API を提供する層。`common/compat/`）を使いません。

## アプリのディレクトリー

| ファイル | 内容 |
|---|---|
| `CMakeLists.txt` | `common/imu/` のアプリのソース、NINA のコアと Zephyr へのつなぎ（`common/drivers/nina/port/zephyr/`）、`common/shell/` の `nina_shell.c`・`i2c_dma_shell.c` をビルドする。NINA のつなぎの devicetree binding を `DTS_ROOT` に、LSM6DSO のつなぎ（`common/drivers/lsm6dso/port/zephyr/`、Zephyr のモジュール）を `ZEPHYR_EXTRA_MODULES` に足す。除算器（下の「除算器」）と `ram_code.cmake` を読む |
| `Kconfig` | `CONFIG_RP2040_PICO_DIVIDER`、NINA のつなぎの `Kconfig` の読み込み（`rsource`）、`CONFIG_NINA_PASSTHROUGH`、NTP の設定、アプリのモジュールの記号（`CONFIG_KV_STORE`・`CONFIG_APP_IMU`・`CONFIG_WIFI_PROFILE`） |
| `prj.conf` | 構成（1 つだけ）。パッチの記号（`CONFIG_RP2040_ROM_MEM_OPS`・`CONFIG_I2C_DW_RP2040_DMA`・`CONFIG_SPI_PL022_DMA_MIN_LEN`）、`CONFIG_CODE_DATA_RELOCATION`、`CONFIG_LSM6DSO=n` と `CONFIG_LSM6DSO_LOCAL=y`、`CONFIG_USBD_CDC_ACM_WORKQUEUE`（下の「CDC ACM の専用のワークキュー」） |
| `app.overlay` | devicetree の追加（下の「devicetree」） |
| `ram_code.cmake`・`ram_code_check.py` | RAM に置くコードの一覧と、リンクのあとの検査（下の「RAM に置くコード」） |
| `src/main.c` | 入口の `main()`（共有の起動 `common/imu/app_main.c` の `app_main()` を呼ぶ）、`kernel ps`、`bootsel` |
| `src/log_ram.c` | RAM のログ（4096 バイト。`dmesg`） |
| `src/kv_store_os.h`・`src/wifi_profile_os.h` | 共有のソース（`common/imu/kv_store.c`・`wifi_profile.c`）が読む Zephyr 版のヘッダー |
| `tests/` | Zephyr 版だけのホストテストの偽物とテスト（下の「テスト」） |

### devicetree

`app.overlay` は次を足します。

- コンソールとシェルは USB の CDC ACM（`cdc_acm_uart0`、受信の FIFO 2048 バイト）です。
- `dma` を有効にします。
- I2C0 を起動時から 1 MHz にします（ボードのファイルは 100 kHz）。
- `spi1` の `dmas` に DMA のチャネル 10（TX）と 11（RX）（下の「PL022 の SPI の DMA のモード」）を入れます。
- LSM6DSOX のレンジ（加速度 8 g、角速度 2000 dps）を設定します。
- NINA-W102 のノードを `arduino,nina-spi`（NINA のつなぎの binding）にします。
- フラッシュの最後の 2 つの消去単位（0xffe000 から 8 KiB）を KV の領域 `storage_partition` にし、イメージの領域から外します。

## PL022 の SPI の DMA のモード

NINA の SPI1 の PL022 のドライバーは DMA のモードで動かします（`prj.conf` の `CONFIG_SPI_PL022_DMA`、`app.overlay` の `spi1` の `dmas` で
チャネル 10 と 11）。ローカルのパッチ `zephyr/0007` の `CONFIG_SPI_PL022_DMA_MIN_LEN`（`prj.conf` で 64）より短い転送はポーリングの
経路を通ります（NuttX 版・FreeRTOS 版・ThreadX 版も同じ境目）。転送の流れ、バスのロック、DMA のエラーと時間切れの扱いは
[NINA-W102 のドライバー](../common/drivers/nina/README_ja.md)の「Zephyr」にあります。

## CDC ACM の専用のワークキュー

`prj.conf` の `CONFIG_USBD_CDC_ACM_WORKQUEUE=y` で、CDC ACM（USB のシェルの送信完了）をシステムのワークキューから専用の
ワークキューへ移しています。Zephyr 4.5.0 の zperf の iperf3 サーバーは、`zperf tcp download stop` で `srv_lock` を持ったまま
シェルに印字し、同じシステムのワークキューの `idle_work_handler()` が `srv_lock` を待つので、この設定が無いとシェル
（`shell_uart`）とシステムのワークキューが互いを待って止まります（どちらも優先度 -1 で pending）。

## RAM に置くコード

フラッシュは RP2040 の 16 KiB の XIP キャッシュを通して実行され、センサーの 6667 Hz の経路と送信の経路のコードはそこに
収まりません。`app/usb-console/ram_code.cmake` に並べたソースのコード（`.text.*`。定数はフラッシュに残す）は RAM で動きます。

- ドライバーとアプリが待ち・通知に使うカーネルのオブジェクト（`kernel/sem.c`・`mutex.c`・`poll.c`・`work.c`・`timeout.c`・
  `queue.c`・`mem_slab.c`）、アトミック操作（`lib/os/atomic_c.c`。Cortex-M0+ では関数）、FIFO のバッファのメモリーブロックと
  そのビット配列、RTIO の実行器とワークキュー、センサーの時計、`gpio0` のドライバーと hal_rpi_pico の `hardware_gpio`
- I2C0（パッチ 0003 の DMA の部分を含む `drivers/i2c/i2c_dw.c`）と DMA のドライバー（`dma_rpi_pico.c`）
- LSM6DSOX のドライバー（コアの `lsm6dso.c`・`lsm6dso_stream.c`・`lsm6dso_fifo.c`・`lsm6dso_cal.c` と、つなぎの
  `lsm6dso_sensor.c`・`lsm6dso_rtio.c`・`lsm6dso_rtio_stream.c`・`lsm6dso_os.c`）と stmemsc の I²C のつなぎ。設定にだけ使う
  ST の `lsm6dso_reg.c` とデコーダーは除く
- IMU のサービスのストリーム、バッチ、キュー、時刻付け（NTP のやり取りと校正は除く）
- 送信: `imu_raw_mqtt.c`・`imu_raw_packet.c` と Zephyr の MQTT のライブラリー、ソケットの振り分け（`sockets.c`・
  `socket_offload.c`、`zvfs` のファイル記述子の表と `poll()`）、NINA のソケット・プロトコル・待ちと Zephyr へのつなぎ
  （`nina_socket.c`・`nina_os.c`・`nina_bus.c`）、PL022 の SPI のドライバー

スケジューラー（`kernel/sched.c`）、スレッドの切り替え（`swap_helper.S`）、割り込みの入口と出口、SysTick のドライバー、
アイドルのループはフラッシュに残ります。ほかの OS のファームウェアの一覧も同じ役割を並べています
（`common/imu/port/pico_sdk/ram_code.cmake`。この一覧が Zephyr のカーネルのオブジェクトを移すところで、互換層のカーネルの
部分を移します。[FreeRTOS 版のポート](freertos-port-ja.md)の「RAM に置くコード」）。配置を確かめるときは、送信中にキャッシュのカウンターをクリアし
（`devmem 0x1400000c 32 0`、`devmem 0x14000010 32 0`。飽和するので必ずクリアする）、15 秒ほどあとに読みます（当たりとアクセスの数）。

- **仕組み**: Zephyr のコードの再配置です。`CONFIG_CODE_DATA_RELOCATION`（`prj.conf`）と
  `zephyr_code_relocate(FILES ... LOCATION RAM_TEXT NOKEEP)`。その生成器（`scripts/build/gen_relocate_app.py`）が、
  オブジェクトの `.text.*` のセクションを出力セクション `.ram_text_reloc` に並べ、Cortex-M のリンカースクリプトはそれを
  フラッシュの `.text` より前に置きます（RAM の先頭。中身はフラッシュから読む）。`z_prep_c()` の `arch_data_copy()` が、
  割り込みの設定とカーネルの起動の前に `.data` と一緒に写します。その前に走るもの（`z_arm_reset`・`relocate_vector_table()`・
  `arch_bss_zero()`、`arch_early_memcpy()` による写し）は一覧にありません。`NOKEEP` なので、どこからも呼ばれない関数は
  これまでどおり捨てられます。パッチ 0008 は、生成器が LSM6DSO のモジュールのライブラリー（ディレクトリーの名前を含まない
  `lsm6dso_local`）のオブジェクトを見つけられるようにします。
- **費用**: RAM 57,320 バイト（`.ram_text_reloc`）。コード 55,546 バイトと、RAM からフラッシュへの長い分岐の中継 106 個
  （1 個 16 バイト）です。フラッシュの `.text` には RAM への中継が 130 個（うち 4 個はフラッシュの書き込みのルーチンへのもの）
  あります。RAM はイメージの後ろ（`_image_ram_end` から）の残りから取ります。そこは picolibc の `malloc()` の領域
  （`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=-1`）で、使うのは NINA の中継モードの 16 KiB のバッファだけです（中継モードの
  ときだけ確保。スタックとほかのバッファは静的）。残りは 63,960 バイトです。
- **検査**: 並べたソースが無ければ `ram_code.cmake` が構成の段階で止めます。リンクのあと、`ram_code_check.py`（ターゲット
  `ram_code_check`。`compile_commands.json`、オブジェクト、マップを読む）が、並べたオブジェクトのコードが RAM に無いか一部が
  フラッシュにあるとき（ビルドされなくなったソースや、生成器が見つけられなかったソース）、並べていないオブジェクトのコードが
  RAM にあるとき、RAM の残りが 18,432 バイト（中継モードのバッファ 16 KiB + `malloc()` の分 2 KiB）より少ないときにビルドを止めます。
  通ったときは RAM にあるものの大きさを表示します。
- **フラッシュに残るもの**: ベクターテーブルと割り込みの入口 `_isr_wrapper`（ソフトウェアの割り込みの表を通して RAM の
  ハンドラー `i2c_dw_isr()`・`dma_rpi_pico_isr()`・`gpio_rpi_isr()` を呼ぶ）、起動のコード、KV のフラッシュの部分（フラッシュの
  書き込みと消去は hal_rpi_pico の `.time_critical` の関数で、`.data` にある。XIP が止まっている間は割り込みを止めて
  それだけが動く）、NINA の中継の UART の割り込み、異常停止の経路（`fault.c`、カーネルの `fatal.c`、ログのコア）です。
  異常停止の経路は移したコードを呼びます: アトミック操作はいつも、`k_sem_give()`・`k_sem_take()`・タイムアウト・`k_work` は
  ログのコアが要るときに。

## Zephyr へのローカルのパッチ

Zephyr 本体とモジュールの一部の変更は、Zephyr の `west patch` のローカルのパッチにしています。パッチの一覧は
`zephyr/manifest/zephyr/patches.yml`、ファイルは `zephyr/manifest/zephyr/patches/<モジュール>/` です。

| パッチ | 対象 | 内容 |
|---|---|---|
| `hal_rpi_pico/0001-boot2-at25sf128a-clkdiv-2.patch` | hal_rpi_pico | ボードの boot2（`boot2_at25sf128a.S`）のフラッシュのクロックを clk_sys の 1/4（31.25 MHz）から 1/2（62.5 MHz）に。hal_rpi_pico には値を渡す口が無いので既定値を変える |
| `zephyr/0001-arm-cortex-m-rp2040-divider-save.patch` | zephyr | `CONFIG_ARM_RP2040_DIVIDER_SAVE`: スレッドの切り替え（PendSV、`swap_helper.S`）で SIO の除算器を退避・復元する |
| `zephyr/0002-soc-rp2040-rom-mem-ops.patch` | zephyr | `CONFIG_RP2040_ROM_MEM_OPS`: `memcpy`・`memset` をブート ROM のルーチンにする |
| `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` | zephyr | DesignWare I²C ドライバー（`drivers/i2c/i2c_dw.c`）: RP2040 の FIFO の深さと RX のしきい値、`CONFIG_I2C_DW_RP2040_DMA` の I²C0 の DMA 読み出し（[DesignWare I2C のコア](../common/drivers/i2c_dw/README_ja.md)）。定数は新しいファイル `drivers/i2c/i2c_dw_rp2040.h` に、DMA の塊と読み出しのコマンド語は `i2c_dw_rp2040_dma_priv.h` に、`i2c_dma` コマンドの関数の宣言は `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` に置き、上流のファイルに入るのは 16 か所（`i2c_dw.c` 14、`i2c_dw.h` 1、`Kconfig.dw` 1）。どのコントローラーでも、0 バイトのメッセージを断り（`-ENOTSUP`）、`i2c_recover_bus()` は `-ENOTSUP` を返す |
| `zephyr/0004-i2c-shell-scan-read-probe.patch` | zephyr | シェルの `i2c scan`（`drivers/i2c/i2c_shell.c`）: 確かめの書き込み（0 バイト）が `-ENOTSUP` を返したら、どこも `--` で「0 devices found」とする代わりに、そのアドレスと残りを 1 バイトの読み出しで確かめ、表のあとにそのことを 1 行表示する。確かめのエラーはどれも `--` |
| `zephyr/0006-sensor-shell-fifo-attributes.patch` | zephyr | sensor シェル（`drivers/sensor/sensor_shell.c`）: LSM6DSO の FIFO の属性と修正（[LSM6DSO のドライバー](../common/drivers/lsm6dso/README_ja.md)）、`drivers/sensor/sensor_shell_os.h`。`common/drivers/lsm6dso/` の LSM6DSO のドライバー（その `<zephyr/drivers/sensor/lsm6dso.h>`）が要る |
| `zephyr/0007-spi-pl022-dma-min-len.patch` | zephyr | PL022 の SPI のドライバー（`drivers/spi/spi_pl022.c`・`Kconfig.pl022`）: `CONFIG_SPI_PL022_DMA_MIN_LEN`（既定 0 で上流と同じ）。DMA のモードで、そのバイト数より短い転送はポーリングの経路を通る（0 バイトの転送はコンテキストのロックを取って何も送らない）。DMA の転送のあとは SSPDMACR の要求を戻し、失敗か時間切れの転送のあとはそのチャネルを止める（「PL022 の SPI の DMA のモード」） |
| `zephyr/0008-code-relocation-module-objects.patch` | zephyr | コードの再配置の生成器（`scripts/build/gen_relocate_app.py`）: ソースのオブジェクトを探す 2 つの方法で見つからないとき、その名前のオブジェクトがほかに無ければそれを使う。名前がディレクトリーの名前を含まないライブラリー（LSM6DSO のモジュールの `lsm6dso_local`）は、そのディレクトリーのソースのオブジェクトを `<ライブラリー>.dir/<名前>.obj` とするので、このパッチが無いと見つからず、そのソースは何も表示されずにフラッシュに残る（「RAM に置くコード」） |

- **当て方**: `source script/setup.sh` が `west patch apply` で各モジュールの作業ツリーに当てます（commit は作りません。
  `git -C zephyr/zephyr status` には変更として出ます）。当てたものは飛ばすので、何度読み込んでも同じです。当てた集合の写しを
  `zephyr/.west/local-patches/` に持ち、`patches.yml` かパッチが変わっていれば、写しの集合を先に戻してから今の集合を当てます
  （一覧から消したパッチがツリーに残らない）。当てるのに失敗したときは、当てた分を戻して止まります。
- **パッチを直したとき**: `patches.yml` の `sha256sum` を `sha256sum <ファイル>` の値に直してから、`source script/setup.sh` を
  読み込み直します。
- **戻し方**: `zephyr/` で `west patch reverse` を実行し、`zephyr/.west/local-patches/` を消します。戻せないときは、
  `setup.sh` の表示のとおり各ツリーを `git -C <ツリー> checkout -- .` で戻してから写しを消し、読み込み直します。
  戻したツリーでは、このアプリはビルドできません（`prj.conf` がパッチの `CONFIG_RP2040_ROM_MEM_OPS`・`CONFIG_I2C_DW_RP2040_DMA`・`CONFIG_SPI_PL022_DMA_MIN_LEN` を使い、
  `CONFIG_RP2040_PICO_DIVIDER` は `CONFIG_ARM_RP2040_DIVIDER_SAVE` が無いと CMake で止まり、0008 が無いと RAM に置くコードの検査がビルドを止める）。
- **Zephyr を上げるとき**: 先に上の手順でパッチを戻し、`manifest/west.yml` の Zephyr のバージョンを変えて `west update` します。新しいツリーで
  各パッチを作り直して `patches/` に置き、`sha256sum` を直し、`source script/setup.sh` で当ててからビルドします。
  0003 を作り直したら `script/host_tests.sh` を実行し、`i2c_dw_sync` が示す上流の変更を共有のコア（`common/drivers/i2c_dw/`）に
  入れるか、入れない理由を `i2c_dw.c` の冒頭の一覧に書いてから、`common/tests/i2c_dw_sync.py ... --update` で期待する差分を
  書き直します（[DesignWare I2C のコア](../common/drivers/i2c_dw/README_ja.md)のテスト）。0006 は `common/shell/sensor_shell.c` から作ります
  （先に新しいツリーの上流のファイルの変更をそこへ入れる）。そうすると `zephyr_sensor_shell_patched` が通ります。

### boot2 の分周 2

boot2 が使う読み出し（EBh）の最高周波数は、AT25SF128A のデータシートで 108 MHz 以上です。clk_sys 125 MHz では分周は偶数なので、
62.5 MHz が上限です（FreeRTOS 版・ThreadX 版の Pico SDK の boot2 と同じ分周）。実機では `devmem 0x18000014`（SSI の BAUDR）が 2 になります。

### 除算器（`CONFIG_RP2040_PICO_DIVIDER`）

アプリの `CONFIG_RP2040_PICO_DIVIDER`（既定で有効）が、hal_rpi_pico の Pico SDK の `divider_hardware.S` をビルドし、
`__aeabi_idiv`・`__aeabi_idivmod`・`__aeabi_uidiv`・`__aeabi_uidivmod`・`__aeabi_ldivmod`・`__aeabi_uldivmod` をリンカーの `--wrap` で
そこへ回します（`CMakeLists.txt`）。無効にすると libgcc のシフトと減算のループです。

- ルーチンは割り込みを止めません（`PICO_DIVIDER_DISABLE_INTERRUPTS=0`）。割り込みハンドラーが割り算の途中に入った場合は、
  ルーチン自身が DIRTY を見て退避します。スレッドが割り算の途中で切り替わる場合は、パッチの `CONFIG_ARM_RP2040_DIVIDER_SAVE`
  （`CONFIG_RP2040_PICO_DIVIDER` が select する）が PendSV で退避します。退避は DIRTY のときだけ 4 つのレジスターを
  `struct _thread_arch` の 20 バイトに置き、割り算の途中でなかったスレッドは DIRTY が落ちた状態で戻ります（FreeRTOS 版の
  Pico SDK のポートは、切り替えのたびに 4 語を退避する。ThreadX 版の ThreadX の Cortex-M0 のポートは、切り替えで退避しない）。
- 0 での割り算は Pico SDK のルーチンの結果です（4 つの OS のファームウェアで同じ）。32 ビットは飽和した商（0 / 0 は 0）と余り 0、64 ビットは
  libgcc と同じです。

### ブート ROM の `memcpy`・`memset`（`CONFIG_RP2040_ROM_MEM_OPS`）

パッチの `CONFIG_RP2040_ROM_MEM_OPS`（`prj.conf` で有効）で、SoC の CMake が `memcpy`・`memset`・`__aeabi_memcpy*`・
`__aeabi_memset*` をリンカーの `--wrap` で RP2040 のブート ROM のルーチンへ回します（Pico SDK の `pico_mem_ops` と同じ使い分け）。
picolibc の中からの呼び出しと、GCC が構造体のコピーやループに出す呼び出しも含みます。無効にすると picolibc のバイト単位の
ループです。ROM のルーチンは初回の呼び出しで ROM の関数表から引いて `.bss` に置くので、`.bss` のゼロ化と `.data` のコピーを
行う `arch_early_memset`・`arch_early_memcpy` は picolibc のものを呼びます。NuttX 版は同じ仕組みを NuttX へのパッチ
（`nuttx/patches/nuttx/0010`。[NuttX 版の設計](nuttx-porting-ja.md)の「C ライブラリの `memcpy`・`memset`」）で持ちます。

## テスト

`zephyr/script/host_tests.sh` は、まず NuttX 版・FreeRTOS 版・ThreadX 版と共有するテスト（`common/tests/`、`common/tests/run.sh` が実行）を
走らせます。共有のテストの中身は、それぞれのドライバーの説明（[NINA-W102](../common/drivers/nina/README_ja.md)、
[DesignWare I2C](../common/drivers/i2c_dw/README_ja.md)、[LSM6DSO](../common/drivers/lsm6dso/README_ja.md)、
[互換層](../common/compat/README_ja.md)）のテストの節にあります。そのあと、偽の Zephyr ヘッダーを使って NINA の Zephyr へのつなぎ（`nina_boundary`・
`nina_boundary_passthrough`・`nina_boundary_spi_dma`・`nina_bus_spi_dma_threshold`・`passthrough`・`nina_socket`・`nina_net`。
偽物は `zephyr/app/usb-console/tests/`）を PC 上でテストします。その中身は [NINA-W102 のドライバー](../common/drivers/nina/README_ja.md)の
テストの節にあります。最後に次を確かめます。

| テスト | 対象 |
|---|---|
| local patches | `patches.yml` がパッチのファイルをちょうど 1 回ずつ並べ、各項目の `sha256sum` がそのファイルの値であること（違うと `west patch` がパッチを断る） |
| `host/imu_host`（syntax） | PC 側の `host/imu_host/*.py` が Python として読めること |
