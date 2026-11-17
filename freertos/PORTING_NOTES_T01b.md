# T01b 事前調査（FreeRTOS 側の未確認事項）

2026-11-17。調査だけで、追跡ファイルは変えていない。実機には触れていない。実験は
`/tmp/claude-1000/-home-ouwa-work-rp2040-connect/ad5eefab-694c-41dd-9cad-b9e1036f99c2/scratchpad/t01b/` と
`freertos/build/t01b-*`（Git 管理外）で行った。

表記: **［確］** 読んだ・動かして確かめた、**［推］** 推測、**［未］** 確かめられなかった。
パスは断りがなければリポジトリのルートから。`port.c` は
`freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c`、`portmacro.h` は同じディレクトリーの
`include/portmacro.h`、`pico-sdk/` は `freertos/pico-sdk/`。

---

## 1. tick を 100 µs にする

### 1.1 64 ビットの tick は RP2040 ポートが受け付けない ［確］

- `portmacro.h:66-78`: `TickType_t` は 16 ビットと 32 ビットだけ。それ以外は
  `#error configTICK_TYPE_WIDTH_IN_BITS set to unsupported tick type width.`
- 実験: `freertos/app/usb-console` を写し、`configTICK_RATE_HZ 10000` と
  `configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_64_BITS` にしてビルド
  （`cmake -S <写し> -B freertos/build/t01b-t64 …; make`）→ **失敗**（`portmacro.h:77:6: error: #error …`、
  続いて `unknown type name 'TickType_t'` が多数）。
- 同じ写しで 32 ビットのまま `configTICK_RATE_HZ 10000` にすると、警告なしでビルドが通った
  （`freertos/build/t01b-t32`）。
- ポートの写し（カーネル全体を scratchpad に複製）の `portmacro.h` に 64 ビットの分岐
  （`typedef uint64_t TickType_t`、`portMAX_DELAY` 0xffff…、`portTICK_TYPE_IS_ATOMIC` は定義しない）を足すと
  ビルドは通るが、`port.c` で警告が 3 つ出る（`freertos/build/t01b-t64p`）:
  - `port.c:952-964` `prvGetEventGroupBit()`: 16/32 ビットの分岐しかなく、64 ビットでは `ulBit` が未初期化のまま返る
    （`-Wuninitialized`）。
  - `port.c:967-974` `prvGetAllEventGroupBits()`: 戻り値なし（`-Wreturn-type`）。
  - これは `configSUPPORT_PICO_SYNC_INTEROP` の要（Pico SDK の mutex・sem・`sleep_ms` が待つイベントグループの
    ビット）で、**そのまま動かすと SDK のロックの待ち合わせが壊れる**。直すなら 64 ビットの分岐を足すパッチが要る
    （`event_groups.h:52-56` は 64 ビットに対応済み: 制御ビットが上位 8 ビット、使えるビットは 56）。
- 64 ビットにした場合の読み出しのアトミック性: ポートが `portTICK_TYPE_IS_ATOMIC` を定義しないので
  `FreeRTOS.h:2925-2934` が `xTaskGetTickCount()` を `portENTER_CRITICAL()`（PRIMASK）で囲む
  （`tasks.c:4220`、ISR 版は `tasks.c:4255`）。正しいが、読むたびに割り込み禁止が入る。

**提案［推］**: tick は **32 ビットのまま 10 kHz** にする。
- 32 ビットの tick は 10 kHz で 2^32 / 10000 s ≈ 4.97 日で一周する。FreeRTOS の遅延・タイムアウトは
  一周をまたいでも正しく扱う（オーバーフロー用の遅延リスト）。1 回の待ちの上限は `portMAX_DELAY - 1` tick
  ≈ 4.97 日で、Zephyr の `K_FOREVER` 以外の長い待ちは互換層で切り詰めるか分割する。
- 64 ビットが要るのは `k_uptime_get()`・`k_uptime_ticks()` などの「起動からの時間」だけで、これは tick を数えずに
  RP2040 の TIMER（`time_us_64()`、64 ビット・1 µs）から作る（下の 1.4 の理由もある）。
- これで計画の「tick は 64 ビット」は不要になる。ポートにパッチを当てない。

### 1.2 tick の割り込み元と変換 ［確］

- tick は **Cortex-M0+ の SysTick**（`port.c:722-740` `vPortSetupTimerInterrupt()`、weak）。
  `LOAD = clock_get_hz(clk_sys) / configTICK_RATE_HZ - 1` = 125 MHz / 10000 − 1 = 12499 で割り切れる
  （誤差なし）。クロック源はプロセッサークロック（`portNVIC_SYSTICK_CLK_BIT`）。
- SysTick と PendSV は最低優先度（`portMIN_INTERRUPT_PRIORITY` 255、`port.c:60-62,310-311,383-385`）。
  Pico SDK は周辺の割り込みを既定で 0x80 にする（`pico-sdk/src/rp2_common/hardware_irq/irq.c:705-715`、
  `irq.h:174`）ので、周辺の割り込みは tick より優先される。
- 割り込みハンドラーは `xPortSysTickHandler()`（`port.c:695-715`）: `xTaskIncrementTick()` と PendSV の要求だけ。
- `pdMS_TO_TICKS()`（`FreeRTOS-Kernel/include/projdefs.h:41-43`）は `uint64_t` で掛けてから `TickType_t` へ
  キャストする（10 kHz でも中間の桁あふれはない。結果が 32 ビットを超えると黙って切れる）。
- **`portTICK_PERIOD_MS` は 10 kHz で 0 になる**（`portmacro.h:83`、`1000 / 10000`）。カーネル・Pico SDK・TinyUSB・
  今のアプリのソースで使っている箇所はコメントだけだった（`grep`）。互換層とアプリで使わないこと。
- SDK 連携の変換 `prvGetTicksToWaitBefore()`（`port.c:912-926`）: 残り時間から 1 tick（100 µs）引いて切り捨てた
  tick 数だけ待ち、残りは呼び出し側（SDK）が詰める。10 kHz でも問題なく動く。
- **Pico SDK の `sleep_ms()`・`sleep_us()` は tick を使わない**［確］。既定の alarm pool があるので
  `sleep_until()`（`pico-sdk/src/common/pico_time/time.c:462-497`）はハードウェアアラーム（TIMER の alarm 3、
  `time.h:342-349`）を `t − PICO_TIME_SLEEP_OVERHEAD_ADJUST_US` に掛けてイベントグループで待ち、
  **最後の `PICO_TIME_SLEEP_OVERHEAD_ADJUST_US` を `busy_wait_until()` で空回りする**。FreeRTOS の連携ヘッダーが
  この値を 150 µs にしている（`include/freertos_sdk_config.h:39-43`）。`configSUPPORT_PICO_TIME_INTEROP` の
  `xPortSyncInternalYieldUntilBefore()`（`port.c:1150-1159`）が使われるのは alarm pool を無効にしたときだけ
  （`time.c:490-494`）。→ 互換層の `k_sleep` は `sleep_ms()` を使わず `vTaskDelay()` で作る（1 回あたり最大
  150 µs の空回りと、`cpu_active` への上乗せを避ける）。
- SDK のイベントグループを割り込みから立てる経路（`port.c:1051` `xEventGroupSetBitsFromISR`）は、タイマータスク（`Tmr Svc`）に
  処理を回す（`FreeRTOS-Kernel/event_groups.c:826` `xTimerPendFunctionCallFromISR`）。今の設定ではタイマータスクの優先度が最高
  （`configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)`、`freertos/app/usb-console/FreeRTOSConfig.h`）。
  優先度の表（計画の `20 − Zephyr`）を作るときに、Zephyr に無いこのタスクの位置を決める必要がある。

### 1.3 sleep の丸め ［確］

| | 規則 | 実際に眠る時間（他に邪魔がないとき） |
|---|---|---|
| Zephyr（v4.5.0、tickless、10 kHz） | `K_MSEC`・`K_USEC` は tick へ切り上げ（`zephyr/zephyr/include/zephyr/sys/clock.h:127-128`、`k_ms_to_ticks_ceil64`）。相対のタイムアウトは「現在の tick 境界 + 切り上げた数 + 1」（`zephyr/zephyr/kernel/timeout.c:216-218`。タイマー割り込みの処理中だけ +1 しない） | (⌈t⌉, ⌈t⌉+1] tick |
| NuttX 版 | 同じ規則に揃えてある（`nuttx/drivers/lsm6dso/lsm6dso_os.h:53-56`、`nuttx/drivers/nina/nina_os.h:30-31`） | 同上 |
| FreeRTOS `vTaskDelay(n)` | 起きる時刻 = 現在の tick 数 + n（`FreeRTOS-Kernel/tasks.c:8729,8767`）。tick の途中から数える | (n−1, n] tick |

- FreeRTOS で Zephyr と同じ時間にするには `vTaskDelay(⌈t / 100 µs⌉ + 1)`。セマフォ・キュー・通知の
  タイムアウトも同じ（tick 数 + 1）。
- **例外**: `K_NO_WAIT`（0 tick）は Zephyr でも `k_yield()` 相当で、+1 しない（`k_sleep(K_NO_WAIT)`）。
  `vTaskDelay(0)` も yield なので、0 のときだけ +1 しない。`K_FOREVER` は `portMAX_DELAY`
  （`INCLUDE_vTaskSuspend` 1 なので無期限）。
- ［推］10 kHz の周期 tick では、割り込みが 100 µs を超えて止まると tick が失われる（SysTick の保留は 1 回分だけ）。
  フラッシュの消去（3.4、NuttX 版の実測で 33.8〜40.2 ms）のたびに約 340〜400 tick 遅れる。tick を時刻の基準に
  しないこと（1.1 の提案どおり、時刻は TIMER から）。

### 1.4 tickless（`configUSE_TICKLESS_IDLE`）の実装 ［確］

- ポートには `vPortSuppressTicksAndSleep()`（`port.c:743-910`、weak）がある。SysTick（24 ビット）を止めて
  再設定する ARM の汎用版の写しで:
  - 抑えられる上限は `portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick`（`port.c:727-728`）= 16777215 / 12500 =
    **1342 tick（134 ms）**。
  - 止めている間のずれを `portMISSED_COUNTS_FACTOR`（45 カウント、`port.c:70-75`）で補正するだけで、tick と実時間の
    ずれが積もる。
  - 眠りは `configPRE_SLEEP_PROCESSING` → `dsb; wfi; isb`（`port.c:808-821`）。
  - 呼ばれるのは予想アイドル時間が `configEXPECTED_IDLE_TIME_BEFORE_SLEEP`（既定 2 tick、
    `FreeRTOS-Kernel/include/FreeRTOS.h:2731-2737`）以上のとき（`tasks.c:5977-6009`）。
- Zephyr 版（`CONFIG_TICKLESS_KERNEL=y`、`zephyr/build/usb-console/zephyr/.config:90`）と NuttX 版
  （`CONFIG_SCHED_TICKLESS=y`、`CONFIG_USEC_PER_TICK=100`、`nuttx/nuttx/.config:403-406`）は **どちらも tickless**。
  計画の「Zephyr 版・NuttX 版と sleep の丸めを揃える」は FreeRTOS の周期 tick で実現できるが、
  **周期割り込み 10 kHz は FreeRTOS 版だけの負荷**（計画どおり実測して報告する）。
- ［推］周期 tick の負荷の見積もり: SysTick の入口・出口と `xTaskIncrementTick()`（起こすタスクが無いとき）で
  およそ 150〜300 サイクル → 125 MHz で 1.2〜2.4 µs、10 kHz で `cpu_active` に **1.2〜2.4 ポイント**。実測が要る。
- 自作の tickless にするなら、`vPortSuppressTicksAndSleep()` を上書きし、TIMER のアラーム（64 ビット・1 µs）で
  起こして `vTaskStepTick()` で進める形になる（weak なので上書きできる）。134 ms の上限とずれの補正の問題は
  無くなる。計画のリスクの「tickless の自作」はこれ。

### 1.5 同じ優先度の時分割 ［確］

- `configUSE_TIME_SLICING 1` だと、同じ優先度の実行可能なタスクは **tick ごと（100 µs）** に切り替わる。
  Zephyr 版は `CONFIG_TIMESLICE_SIZE=20`（20 ms）、`CONFIG_TIMESLICE_PRIORITY=0`
  （`zephyr/build/usb-console/zephyr/.config:113,523-524`）。
- Zephyr 版で同じ優先度を共有するスレッド: 8（`wifi_reconnect`、`nina_scan`、`nina_connect`、
  `CONFIG_UDC_RPI_PICO_THREAD_PRIORITY=8`）、9（`imu_stream`、`time_sync`、`nina_monitor`）など
  （`zephyr/app/usb-console/src/*.c`、`drivers/nina/nina_net.c` の `K_THREAD_DEFINE`）。
  ［推］どれも普段は待っているので影響は小さいが、意味は違う。合わせるなら `configUSE_TIME_SLICING 0` にして
  tick フックで 20 ms ごとに `taskYIELD` 相当を起こす（項目 11 も参照）。

---

## 2. CPU 負荷（`cpu_active`）の計測

### 2.1 Zephyr 版・NuttX 版の方式 ［確］

- NuttX 版 `nuttx/boards/arduino-nano-rp2040-connect/src/nano_idle.c:23-46,85-100`（Zephyr の
  `kernel/idle.c`、`arch/arm/core/cortex_m/cpu_idle.c`、`lib/os/cpu_load/cpu_load_backend_idle.c` を写した説明つき）:
  割り込みを禁止（PRIMASK）→ 窓を開く（TIMER を読む）→ `dsb; wfi` → 窓を閉じる → 割り込みを許可。
  WFI は PRIMASK で止めた割り込みでも起きる（ARMv6-M）ので、起こした割り込みの処理は窓の外で走り、負荷に数えられる。
- 計算は `nano_cpuload.h`（Zephyr の `cpu_load_get_cpu()` と同じ手順、千分率、切り捨て）。

### 2.2 FreeRTOS の RP2040 ポートのアイドル ［確］

- アイドルタスク（`tasks.c` `prvIdleTask`）は、`configUSE_IDLE_HOOK` が 1 なら毎周 `vApplicationIdleHook()` を呼ぶ
  （`tasks.c:5969-5974`）。tickless でなければ **ポートは WFI しない**（空回り）。WFI するのは
  `vPortSuppressTicksAndSleep()` の中だけ（`port.c:819-821`）。
- `configGENERATE_RUN_TIME_STATS` の実行時間（今の `ps`）はタスクの切り替えでしか数えないので、アイドル中の
  割り込みの処理時間は `IDLE` の時間に入る。Zephyr の意味の `cpu_active` には使えない。

### 2.3 提案 ［推］

- `configUSE_IDLE_HOOK 1` にし、`vApplicationIdleHook()` を `nano_idle.c` の `up_idle()` と同じ形にする:
  `cpsid i` → `nano_cpuload_enter(time_us_64())` → `dsb; wfi` → `nano_cpuload_exit(time_us_64())` → `cpsie i`。
  `cpu_load_get()` と `nano_cpuload.h`（純粋な計算）はそのまま使える（P2 の T23 で `common/` へ出す前提と合う）。
  - `time_us_64()` は TIMERAWH・TIMERAWL を読み直す方式で、割り込みの中から呼んでも壊れない
    （`pico-sdk/src/rp2_common/hardware_timer/timer.c:57-73`）［確］。
  - アイドルフックはブロックしてはいけないが、WFI はかまわない。起こした割り込みがタスクを起こせば、`cpsie i` の
    直後に PendSV でアイドルから抜ける。
  - アイドル優先度（0）にアプリのタスクを置かないこと（`configIDLE_SHOULD_YIELD` で窓の外の時間が増える）。
- tickless を使わない（周期 tick のまま）なら、tick の割り込みも 100 µs ごとに WFI を起こし、その処理時間が
  負荷に入る。これが計画の「周期割り込みの分だけ `cpu_active` が高く出る」に当たる。
- 自作の tickless（1.4）にするなら、同じ窓を `vPortSuppressTicksAndSleep()` の WFI の前後に置く
  （`configPRE_SLEEP_PROCESSING`・`configPOST_SLEEP_PROCESSING` の位置。ポートの WFI の前後は割り込み禁止、
  `port.c:808-833`）。

---

## 3. フラッシュの書き込み

### 3.1 Pico SDK の API ［確］

- `hardware_flash`（`pico-sdk/src/rp2_common/hardware_flash/flash.c`）:
  - `flash_range_erase(offs, count)`（214-238 行）: 4096 の倍数。boot2 を RAM へ写し、ROM の
    `connect_internal_flash` → `flash_exit_xip` → `flash_range_erase`（64 KiB 単位にできるところは 0xd8、それ以外は
    4 KiB の 0x20）→ `flash_flush_cache` → boot2 で XIP を戻す。
  - `flash_range_program(offs, data, count)`（245 行〜）: 256 の倍数。同じ手順。
  - どちらも RAM に置かれる（`__no_inline_not_in_flash_func`）が、**割り込みは自分では止めない**。XIP が止まる間に
    フラッシュ上のコード（割り込みハンドラーを含む）を実行してはいけないので、呼び出し側が止める。
  - バイト単位の書き込みは無い。Zephyr の HAL にある `flash_write_partial()` は上流の Pico SDK 2.3.1 に無い
    （項目 4 の調べ）。ページの残りを 0xff で埋めて書く（NuttX 版 `nano_flash_kv.c:30-37` と同じ）。
- `pico_flash` の `flash_safe_execute(func, param, timeout_ms)`（`pico-sdk/src/rp2_common/pico_flash/flash.c`）:
  もう一方のコアを止めてから自コアの割り込みを止め、`func` を呼ぶ。

### 3.2 FreeRTOS と組み合わせたときの `flash_safe_execute()` ［確］

- FreeRTOS のライブラリーは 1 コアの設定でも `pico_multicore` をリンクし、`FREE_RTOS_KERNEL_SMP=1` を定義する
  （`freertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/library.cmake:33-45`。今のビルドの
  `flags.make` にも `LIB_PICO_MULTICORE=1`、`LIB_FREERTOS_KERNEL=1`、`FREE_RTOS_KERNEL_SMP=1`）。
- そのため `PICO_FLASH_SAFE_EXECUTE_SUPPORT_MULTICORE_LOCKOUT` が 1 になり（`pico/flash.h:113-118`）、
  `configNUMBER_OF_CORES` が 1 なので FreeRTOS SMP の経路も使われず、`LIB_FREERTOS_KERNEL && PICO_RP2040` の分岐で
  **`PICO_ERROR_NOT_PERMITTED` を返す**（`pico_flash/flash.c:174-181`）。
- 実験（`freertos/build/t01b-flash-a`）: `flash_safe_execute()` を呼ぶ試験関数をリンクして逆アセンブルすると、
  `default_enter_safe_zone_timeout_ms` は `movs r0, #4; negs r0, r0; bx lr`（常に −4 = NOT_PERMITTED）だった。
- `PICO_FLASH_ASSUME_CORE1_SAFE=1` を定義すると（`freertos/build/t01b-flash-b`）、コア 0 では
  `mrs PRIMASK; cpsid i` して 0 を返すだけになる（`use_irq_only()`、`pico_flash/flash.c:90-99`）。
- **提案**: コア 1 は使わないので `PICO_FLASH_ASSUME_CORE1_SAFE=1` を定義して `flash_safe_execute()` を使うか、
  NuttX 版と同じく自分で `save_and_disable_interrupts()` → `flash_range_erase/program` → `restore_interrupts()`
  を呼ぶ。どちらも「割り込みを止めるだけ」で同じ。`taskENTER_CRITICAL()` も 1 コアでは PRIMASK なので同じ効果
  （`portmacro.h:175-189`）。

### 3.3 リンカースクリプトで末尾を除外する ［確］

- Pico SDK 2.3.1 のリンカースクリプトは部品（`.incl`）の組み合わせで、FLASH の長さは
  `pico_flash_region.ld`（`pico-sdk/src/rp2_common/pico_standard_link/pico_flash_region.template.ld`、
  `PICO_FLASH_SIZE_BYTES` = ボードの 16 MiB、`boards/arduino_nano_rp2040_connect.h:77-80`）から作られる。
- `pico_add_linker_script_override_path(TARGET DIR FILES …)`（`pico_standard_link/CMakeLists.txt:96-124`）で
  同名の部品を差し替えられる。実験で、
  - `pico_flash_region.ld` を `FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 0xffe000` にし、
  - 空の部品 `section_extra_post_end.incl` に `ASSERT(__flash_binary_end <= 0x10ffe000, …)` と
    `__kv_start = 0x10ffe000; __kv_end = 0x11000000;` を書いたところ、
  マップファイルで `FLASH 0x10000000 0x00ffe000`、ASSERT と `__kv_start` が入ったことを確かめた
  （`freertos/build/t01b-flash-a/usb-console.elf.map:2969,4860-4861`）。ASSERT が実際に失敗する場合は試していない。
- `pico_override_flash_size()` も使えるが、ターゲットの `PICO_FLASH_SIZE_BYTES` まで変える（同 CMakeLists.txt:
  130-138）ので、フラッシュの大きさを使うコード（binary info など）に影響する。部品の差し替えのほうが狭い。
- NuttX 版はリンカースクリプト（`scripts/nano-flash.ld`）とビルドスクリプトでイメージの終わりを確かめ、実行時にも
  `__kv_start`・`__kv_end` と定数を比べてから操作を許す（`nano_flash_kv.c:38-46`）。同じ二重の検査を写せる。

### 3.4 割り込みを止める時間と影響

- NuttX 版の実測（`nuttx/PORTING_STATUS.md:354`）: 4 KiB の消去 1 回で割り込み禁止 33.8〜40.2 ms、256 B の書き込み
  1 回で 0.37〜0.51 ms、1 項目の書き換えでページ書き込み 4 回。Zephyr のドライバーも 1 回の消去・書き込みの間
  `irq_lock()` する（`zephyr/zephyr/drivers/flash/flash_rpi_pico.c:82,132`、項目 4 の調べ）。
- ［推］FreeRTOS 版でも同じ時間止まる。違いは、10 kHz の周期 tick がその間に約 340〜400 回失われること
  （1.3）。TIMER は止まらないので、時刻を TIMER から作れば影響は tick（タイムアウトの進み）だけ。
- ［推］消去中は USB の割り込みも止まる。RP2040 の USB はハードウェアが NAK を返すので、ホストの再送で済むはず
  （Zephyr 版・NuttX 版と同じ条件）。DMA がフラッシュ（XIP）を読む転送があると壊れるが、今の I2C の DMA は RAM の
  バッファーだけ［未: 全転送の元を確かめていない］。

---

## 4. KV の形式

（subagent の調べと実験。メインは `nvs.c`・`settings_nvs.c` の場所と行数、`.config` の値、トピックの作り方を照合した。）

### 4.1 Zephyr 版の設定と実際の値 ［確］

- `zephyr/app/usb-console/prj.conf:54-55,90-94,97`: `WIFI_CREDENTIALS`（`_BACKEND_SETTINGS`）、`FLASH`、`FLASH_MAP`、
  `NVS`、`SETTINGS`、`SETTINGS_NVS`、`HWINFO`。
- `app.overlay:42-48`: `storage_partition` = 0xffe000 から 0x2000。`zephyr,settings-partition` の指定が無いので
  `settings_nvs.c:20-24` は `storage_partition` を使う。

| 項目 | 値 | 根拠 |
|---|---|---|
| `SETTINGS_NVS_SECTOR_SIZE_MULT` / `_SECTOR_COUNT` | 1 / 8（上限） | `zephyr/build/usb-console/zephyr/.config:1856-1857` |
| `NVS_LOOKUP_CACHE`、`NVS_DATA_CRC`、`NVS_INIT_BAD_MEMORY_REGION`、`SETTINGS_NVS_NAME_CACHE` | すべて無効 | 同 `.config`（1287 行ほか） |
| セクターの大きさ | 4096（消去単位） | `settings_nvs.c:412-419`、`rp2040.dtsi:208` |
| セクターの数 | **2**（8 KiB に収まる数。上限 8 には届かない） | `settings_nvs.c:425-435` |
| 書き込み単位 / 消去値 | 1 バイト / 0xff | `zephyr/zephyr/drivers/flash/flash_rpi_pico.c:34-37` |
| `WIFI_CREDENTIALS_MAX_ENTRIES` | 2 | `.config` |

### 4.2 v4.5.0 の実装 ［確］

NVS は **`subsys/kvss/nvs/`**（`include/zephyr/fs/nvs.h` は非推奨で `zephyr/kvss/nvs.h` を読むだけ）。

| ファイル | 行数 |
|---|---|
| `zephyr/zephyr/subsys/kvss/nvs/nvs.c` / `nvs_priv.h` / `include/zephyr/kvss/nvs.h` | 1662 / 119 / 206 |
| `subsys/settings/src/settings_nvs.c` / `include/settings/settings_nvs.h` | 460 / 67 |
| `subsys/settings/src/settings.c` / `settings_store.c` / `settings_init.c` / `settings_priv.h` | 336 / 378 / 45 / 124 |
| `include/zephyr/settings/settings.h` | 735 |
| `subsys/crc/crc8_sw.c`（`crc8_ccitt`） | 約 50 |
| `wifi_credentials_backend_settings.c`（資格情報） | 166 |

`settings_line.c` は NVS のバックエンドでは要らない。

### 4.3 フラッシュ上の形式 ［確］

- NVS の項目（ATE）は 8 バイト: id（u16）、offset（u16）、len（u16）、part（u8）、crc8（u8、`crc8_ccitt(0xff, …)`）。
  項目はセクターの末尾から下へ、データは先頭から上へ。
- settings の ID（`settings_nvs.h:25-34`、`settings_nvs.c:136-215,236-350`）: 0x8000（`NVS_NAMECNT_ID`）に最大の名前 ID、
  名前は 0x8001 から（NUL なし）、値は「名前の ID + 0x4000」。保存のたびに名前を全部後ろから読んで探す（キャッシュなし）。
- キー: `app_kv/<key>`（`zephyr/app/usb-console/src/kv_store.c:9`。`imu.port`、`imu.session`、`imu.cal.*` など）、
  `wifi_cred/<番号>`（0〜1、`wifi_credentials_backend_settings.c:18-21`）、`wifi_profile.c:51,73` の旧キー（読んで消すだけ）。
- アプリが使う API は `settings_subsys_init`・`settings_save_one`・`settings_load_one`・`settings_delete`、資格情報だけ
  `settings_load_subtree_direct`（`wifi_credentials_backend_settings.c:107,154`）。

### 4.4 依存している Zephyr の API ［確］

- `nvs.c`: `flash_read`・`flash_write`・`flash_flatten`（消去）・`flash_get_parameters`・`flash_get_write_block_size`・
  `flash_get_page_info_by_offs`、`k_mutex_init/lock(K_FOREVER)/unlock`、`crc8_ccitt`、`LOG_*` と小さなマクロ。
- settings: `flash_area_open`・`flash_area_get_sectors`、`sys_slist_*`、`STRUCT_SECTION_FOREACH`（空でよい）、
  `K_MUTEX_DEFINE(settings_lock)`（`settings.c:27`。**ゼロ初期化のミューテックスが未ロックとして動く**必要があり、F02 の要件と同じ）。

### 4.5 実験 ［確］

1. 上の 6 本（`nvs.c`、`settings_nvs.c`、`settings.c`、`settings_store.c`、`settings_init.c`、`crc8_sw.c`）を **改変せずに**写し、
   最小のスタブヘッダー（約 120 行）と `.config` 相当の設定で `arm-none-eabi-gcc` 15.2（`-mcpu=cortex-m0plus -Os`）でコンパイル
   → 全部通った。`.text` 合計約 5.5 KB。未解決はフラッシュの関数、`flash_area_*`、`k_mutex_*`、libc だけ。
2. 同じソースをホストの gcc で、RAM 上の 8 KiB の「フラッシュ」（書き込み単位 1、書き込みはビットの AND、4096 単位で消去）で動かし、
   保存・2000 回の上書き（GC が 2 セクターを何周もする）・読み出し・削除・再マウント後の読み出しが期待どおり。
   イメージの項目は 4.3 の形式どおり（例 `0380 0000 1200 ff49` = ID 0x8003、offset 0、長さ 18 = `app_kv/imu.session`）。

### 4.6 Zephyr 版の KV をそのまま読めるか

- ［推］ソースを改変せず、同じ幾何（0xffe000、4096 × 2、書き込み単位 1、消去値 0xff）と同じ設定で動かせば同じバイト列になるので、
  **読める見込みが高い**。
- ［未］Zephyr 版が実機で書いたイメージとの照合（生のダンプがリポジトリに無い。`host/recordings/kv-backup.json` は `kv get` の値）。

### 4.7 FreeRTOS 側で新しく書く部分（見積もり ［推］）

1. フラッシュの関数（約 100〜150 行）: 読み出しは XIP からの memcpy（Zephyr も同じ、`flash_rpi_pico.c:58`）、消去は
   `flash_range_erase` を 4096 単位、書き込みは 256 B のページの残りを 0xff で埋めて `flash_range_program`
   （Zephyr の HAL の `flash_write_partial()`、`zephyr/modules/hal/rpi_pico/src/rp2_common/hardware_flash/flash.c:274-295,628-640` は
   上流の SDK に無い。結果のバイト列は同じ）。範囲外の拒否と割り込みの禁止（項目 3）。
2. `flash_area_open`・`flash_area_get_sectors`（固定の 1 領域、約 30 行）。
3. ヘッダー（`zephyr/kvss/nvs.h` は写し、`drivers/flash.h`、`storage/flash_map.h`、`sys/crc.h`、`sys/slist.h`（約 40 行）、空の
   `settings/settings_file.h`）。NuttX の互換層には slist・crc・flash・flash_map が無い（`nuttx/drivers/compat/zephyr/sys/`）。
4. settings の .c は無改変で写す（約 3000 行、Apache-2.0 の表記のまま）。資格情報の `wifi_credentials_backend_settings.c` も
   そのまま使える（`settings_load_subtree_direct` が動くので、NuttX 版のような書き換えが要らない）。

合計: 写し約 3000 行、新規約 250〜300 行。

### 4.8 NuttX 版がしたこと ［確］

- NuttX の NVS（`drivers/mtd/mtd_config_nvs.c`、名前つき項目、`MTD_CONFIG_NAMED=y`、`MTD_CONFIG_NAME_LEN=64`）を `/dev/config` として
  使い、フラッシュとの間は `nuttx/boards/arduino-nano-rp2040-connect/src/nano_flash_kv.c`（618 行）。
- `nuttx/apps/imu/kv_store.c`（136 行）と `nuttx/apps/wifi/settings_nvs.c`（125 行）が ioctl で読み書き。形式は Zephyr 版と違う
  （`kv_store.c:8`。項目 16 バイトで名前を項目の中に持つ）。
- 互換層の `settings.h`（48 行）には `settings_load_subtree_direct` が無く、資格情報のバックエンドを書き換えている。
- 起動時に NuttX の形式でも全面消去でもなければ領域を消去する（`nano_flash_kv.c:57-61`）。

### 4.9 注意と代案

- ［推］**初回の起動**: 今の実機は OpenMV 版で、フラッシュの後ろが FAT（`openmv/README_ja.md:92-93`）。0xffe000 に Zephyr の KV
  ではないデータがあると、`NVS_INIT_BAD_MEMORY_REGION` が無効なので `nvs_mount` が `-EDEADLK` を返すか（`nvs.c:999-1013`）、
  意味の無いデータを読む。マウント失敗時に消去するか（Zephyr 版には無い動作）、`kv` コマンドで初期化するかを決める必要がある。
- 代案（写せない場合）: (1) NuttX 版と同じく独自の形式にして `script/kv_transfer.py` で移す（書き換えのたびに Wi-Fi のパスワード入力）。
  (2) Zephyr の ZMS（これも Zephyr 版と別の形式で利点は小さい）。
- ［推］この写し方が成り立つなら、NuttX 版も同じ形式にでき、「版を書き換えるたびに KV が消える」問題が無くなる（計画の範囲外）。

---

## 5. 再起動で消えない RAM と異常停止の記録

（subagent の調べ。ソースを読んだだけでコンパイルの実験はしていない。メインは MSP の位置を照合した。）

### 5.1 Zephyr 版・NuttX 版 ［確］

- Zephyr 版の RAM ログ（`zephyr/app/usb-console/src/log_ram.c`）: `static __noinit struct retained_log retained`（20 行）に
  magic・その反転・start・length と 4096 バイトのバッファー（9-18 行）。`ram_init()` で検査して合わなければ空にする（66-76 行）。
  `dropped` は .bss。排他は `k_spin_lock`。`__noinit` は `zephyr/zephyr/include/zephyr/linker/section_tags.h:18`。
- Zephyr 版の異常停止: `CONFIG_RESET_ON_FATAL_ERROR=y`、`CONFIG_STACK_SENTINEL=y`、`CONFIG_EXCEPTION_DUMP=y`、`CONFIG_ASSERT` は無効
  （`__ASSERT` は空）。`k_sys_fatal_error_handler()` が `LOG_PANIC()` → `EXCEPTION_DUMP("Resetting system")` →
  `sys_reboot(SYS_REBOOT_WARM)`（`kernel/fatal.c:37-50`）。報告は `***** HARD FAULT *****`（`arch/arm/core/cortex_m/fault.c:730`）、
  r0〜r3・r12・lr・xpsr と `Faulting instruction address (r15/pc)`（`arch/arm/core/fatal.c:26-30,90`）、
  `>>> ZEPHYR FATAL ERROR n: <理由>`、`Current thread: %p (%s)`。panic モードではログが同期で RAM ログに入る。
- NuttX 版: RAM ログは `nuttx/drivers/log/log_ram.c:52` で `.noinit`（`nano-flash.ld:117-122`、.data と .bss の間、NOLOAD）。
  8192 バイトのリング、magic は大きさと XOR（`design/nuttx-porting-ja.md:119-124`）。異常停止は `CONFIG_BOARD_RESET_ON_ASSERT=2`、
  `CONFIG_ARCH_STACKDUMP=y` で報告を RAM ログに書いて `up_systemreset()`（AIRCR、
  `nuttx/boards/arm/rp2040/common/src/rp2040_reset.c:57-59`）。スタックの番兵は自前の `nano_stackcheck.c`。

### 5.2 Pico SDK と今の FreeRTOS 版 ［確］

- `__uninitialized_ram(group)` は `.uninitialized_data.<group>` に置く（`pico-sdk/src/rp2_common/pico_platform_sections/include/pico/platform/sections.h:167-184`）。
  リンカーは `section_uninitialized_data.incl` で `(NOLOAD) > RAM`。crt0 が触るのは .data のコピーと .bss の 0 埋めだけ
  （`pico_crt0/crt0.S:513-545`）。今のビルドでは 0x200000c0（RAM のベクター表の直後）。`.persistent_data` は RP2350 の POWMAN 用で使わない。
- `isr_hardfault` は crt0 の弱いシンボルで既定は `bkpt`（`crt0.S:349`）。デバッガーが無いとそこで止まり、再起動しない。
  強いシンボルの定義か `exception_set_exclusive_handler()` で置き換えられる。
- **今の FreeRTOS 版は異常停止で止まったまま再起動しない**: アサート・スタックのあふれ・ヒープ不足は `panic()`
  （`freertos/app/usb-console/src/main.c:43-56`）、SDK の既定の `panic()` は stdio に出して `_exit(1)`
  （`pico_platform_panic/panic.c`）。`PICO_PANIC_FUNCTION` で置き換えられる（`panic.h:14`）。
- `configCHECK_FOR_STACK_OVERFLOW` 2 の検査は `vTaskSwitchContext()` の中だけ（`tasks.c:5259`、`stack_macros.h:77-116`）。
  フックは PendSV（割り込みの文脈）から呼ばれる。割り込みの出口では見ない（Zephyr の STACK_SENTINEL は見る）。
- MSP（割り込み用スタック）は項目 8.5。`PICO_USE_STACK_GUARDS=1` で MPU で底の 32 バイトを守れる
  （`runtime_init_stack_guard.c:10-33`）。
- watchdog の scratch: `watchdog_reboot()` が使うのは scratch[4]〜[7] だけで、[0]〜[3] はアプリが使える
  （`hardware_watchdog/watchdog.c:92-113`）。今の `reboot` は `watchdog_reboot(0, 0, …)`（`commands.c:116`）。Zephyr 版・NuttX 版は
  AIRCR の SYSRESETREQ。

### 5.3 提案 ［推］

- RAM ログ: `common/log` へ移す予定の `log_ring.[ch]` をそのまま使い、本体を `__uninitialized_ram(log_ram)` に置く。magic は NuttX 版と
  同じ形、排他は割り込み禁止。
- 異常停止の共通の出口 `fatal_report(reason, esf)` を 1 つ作る: 割り込みを禁止し、RAM ログに直接書く（USB の stdio は使わない）。
  内容は理由、r0〜r3・r12・lr・pc・xpsr、`pcTaskGetName(NULL)`、各タスクのスタックの空き。そのあと AIRCR
  （`scb_hw->aircr = 0x05FA0004`）で再起動し、他の 2 つの版とリセットの方式をそろえる。入口は
  (1) 強いシンボルの `isr_hardfault`（EXC_RETURN の bit 2 で MSP/PSP を選ぶ短い naked のアセンブリ）、
  (2) `vApplicationStackOverflowHook`、(3) `configASSERT`（`app_assert_failed`）と `vApplicationMallocFailedHook`、
  (4) `-DPICO_PANIC_FUNCTION=app_panic`。
- RAM ログの先頭が押し出されても分かるように、PC・LR・理由を scratch[0]〜[3] にも残す（`CLAUDE.md` の OpenMV の節の方法）。
- `__ASSERT` は空（他の 2 版と同じ）。`configASSERT` は残す。
- 試験は `CLAUDE.md` の `devmem 0x30000000`（存在しないアドレス）で HardFault を起こす方法が使える（項目 10 の `devmem` が要る）。

---

## 6. newlib のヘッダー

（subagent の実験。メインは `sys/socket.h`・`poll.h`・`netdb.h` が無いこと、`sys/dirent.h:10` の `#error` を照合した。）

### 6.1 実験の方法 ［確］

`freertos/app/usb-console` を `…/scratchpad/t01b/fork-b/app` に写して `freertos/build/t01b-b` で設定し、`compile_commands.json` の
`src/main.c` のフラグ（`-mcpu=cortex-m0plus -std=gnu11` と SDK のインクルード一式）でヘッダーごとの試験ファイルをコンパイルした。
リンクは写した CMakeLists に試験用の実行ファイル（`pico_stdlib` をリンク）を足して確かめた。

### 6.2 結果 ［確］

| ヘッダー／機能 | 結果 | 根拠 |
|---|---|---|
| `<sys/socket.h>` `<netinet/in.h>` `<netinet/tcp.h>` `<arpa/inet.h>` `<poll.h>` `<netdb.h>` `<sys/uio.h>` `<sys/ioctl.h>` `<semaphore.h>` | **無い**（`No such file`） | ツールチェーンの `arm-none-eabi/include` に無い。Pico SDK の `lib/lwip` はサブモジュール未取得で空 |
| `<dirent.h>` | **`#error "<dirent.h> not supported"`** | `arm-none-eabi/include/sys/dirent.h:10` |
| `<pthread.h>` | 通るが**中身が空**（`PTHREAD_MUTEX_INITIALIZER` 未定義、libc に実装なし） | `pthread.h:27`（`_POSIX_THREADS`）。`sys/features.h:370-400` は RTEMS などでしか定義しない |
| `<time.h>` の `clock_gettime`・`CLOCK_MONOTONIC` | **宣言なし、libc にもなし**（`nm`） | `time.h:161,272` |
| `gettimeofday`・`time()` | リンクできる | `_gettimeofday` は SDK の弱いシンボル（`pico_clib_interface/newlib_interface.c:82-97`） |
| `<fnmatch.h>` | ヘッダーはあるが**実装が libc に無い**（`undefined reference to 'fnmatch'`） | `lib/thumb/v6-m/nofp/*.a` に無い |
| `getopt`・`getopt_long` | リンクできる。再入可能版 `__getopt_r`・`__getopt_long_r` もある | `getopt.h:135-169` |
| `<sys/select.h>`、`strerror` | 使える | |
| `htons` 系 | 無い（`__htons` だけ） | `machine/endian.h:58` |

### 6.3 必要なのに無いもの ［確］

対象: `nina_sock.c/.h`・`nina_dns.c/.h`・`nina_sockopt.h`、互換層の `net/socket.h`、それを使う `apps/imu/*`・`nina_net.c`
（grep と、空のヘッダーを置いた `-fsyntax-only`）。

- 型: `struct sockaddr`・`sockaddr_in`・`in_addr`・`iovec`・`msghdr`・`addrinfo`・`pollfd`、`socklen_t`。
- 定数: `AF_INET`・`AF_UNSPEC`・`SOCK_STREAM`・`SOCK_DGRAM`・`IPPROTO_TCP`・`IPPROTO_UDP`・`INET_ADDRSTRLEN`、`MSG_DONTWAIT`・`SOL_SOCKET`・
  `SO_ERROR`・`SO_RCVTIMEO`・`SO_BINDTODEVICE`・`TCP_NODELAY`・`FIONREAD`・`FIONBIO`、`POLLIN/OUT/ERR/HUP/NVAL`、`EAI_*`（8 個）、
  `AI_NUMERICHOST`・`AI_NUMERICSERV`・`AI_ADDRCONFIG`。
- 関数: `htons`・`ntohs`、`inet_pton`・`inet_ntop`。
- ソケットの呼び出し: NuttX では OS（usrsock）が持っていた。FreeRTOS では `zsock_*` から `nina_sock_*`（`nina_sock.h:163-297`）を呼ぶ層と、
  fd の表（fd → NINA のソケット、種類、非ブロッキング）が要る。`getsockname` は `nina_sock.c` に無く、NuttX では usrsock が持っていた
  （`nuttx/drivers/nina/nina_usrsock.c:639`）。
- `nuttx/apps/subcmd/zshell.c`（41-54、97-285 行）: `opendir`・`readdir`・`stat`・`fstat`・`fcntl(F_GETPATH)`・`open` で端末を `/dev` から
  探す NuttX だけの処理と、`pthread_mutex` が 1 つ。FreeRTOS では書き直す。
- `fnmatch`: `subcmd.c:216`、`zshell.c:601` が使う。元は Zephyr の `zephyr/zephyr/subsys/portability/posix/c_lib_ext/fnmatch.c`
  （382 行、BSD-3）か NuttX の `libs/libc/misc/lib_fnmatch.c`（207 行、BSD-2）。
- `getopt`: newlib の `getopt` はグローバルな状態なので、USB と Telnet のシェルが同時に使うと競合する。使うのは `wifi cred add`・
  `wifi connect`（`wifi_shell.c:924-927`）。newlib の `__getopt_long_r` か Zephyr の `lib/utils/getopt/` を使う。
- `clock_gettime`・`clock_settime`: 互換層の `sys/clock.h:41-53`（`time_sync.c` が使う）を TIMER と基準時刻で作る。
- `<semaphore.h>`・`pthread_once`: `apps/sensor/sensor_shell_nuttx.c` だけ（NuttX の接着なので持ってこない［推］）。

### 6.4 errno ［確］

- newlib（GCC 15.2）と Zephyr 版の picolibc（`CONFIG_PICOLIBC=y`、`zephyr-sdk-1.0.1/…/include/sys/errno.h`）の errno 85 個を比べ、
  **違いは 0**。NuttX 版との違い（`nuttx/README_ja.md` の表）のような表は要らない。NuttX との違いは表のほかに `ETIMEDOUT` 116/110、
  `EHOSTUNREACH` 118/113、`EINPROGRESS` 119/115、`EALREADY` 120/114、`ECANCELED` 140/125、`EILSEQ` 138/84。
- errno の数値が出る箇所（Zephyr 版）: `kv` の失敗（`kv_store.c:118,148,187`）、`imu start` の失敗（`imu_stream.c:685,690,695`）、
  `imu status` の `last_recovery_error`（`imu_stream.c:782`）、`ntp status` の `last_error`（`time_sync.c:367`）、`nina version`（`main.c:23`）、
  `imu cal`（`imu_cal.c:95`）、`sensor`（`sensor_shell.c:637`）、ログと MQTT の status。
- 注意: Pico SDK の API は `PICO_ERROR_*` を返す（`pico/error.h:24-35`。`PICO_ERROR_TIMEOUT` は −2 で `-ENOENT` と同じ数値）。errno に直して
  から出す。`EAI_*` は Zephyr が負の値（`DNS_EAI_NONAME` = −2 など、`dns_resolve.h:426-430`）、NuttX が正の値。アプリは
  `time_sync.c:121` が 0 かどうかを見るだけ。
- 注意: Zephyr 版は `CONFIG_NET_NAMESPACE_COMPAT_MODE=y`（`net_compat.h:51`）。Zephyr のファイル（Telnet、zperf、`wifi_shell.c`）を写すと、
  `net_sockaddr`・`NET_AF_INET`・`net_htons`・`net_sad()` など Zephyr 4.5 の名前も要る。

---

## 7. USB コンソール（`pico_stdio_usb`）

### 7.1 TinyUSB を回している文脈 ［確］

`pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c`:

- `stdio_usb_init()`（195-255 行）: `tusb_init()` のあと、未使用の **ユーザー割り込み**（ソフトウェアで起こす
  IRQ 26〜31 のどれか、`user_irq_claim_unused`）を取り、`low_priority_worker_irq` を登録する。
  TinyUSB の RP2040 ドライバーが `USBCTRL_IRQ` を共有ハンドラーで登録する
  （`pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c:367`）ので、`irq_has_shared_handler()` が真になり、
  `usb_irq()`（96-98 行）が USB の割り込みのたびにユーザー割り込みを保留にする。**1 ms の周期タイマーは使わない**
  （周期タイマー 232 行は共有ハンドラーが無いときだけ）。
- `low_priority_worker_irq()`（62-94 行）: `stdio_usb_mutex`（Pico SDK の `mutex_t`）を `mutex_try_enter` で
  取れたら **割り込みの中で `tud_task()`** を回し、受信があれば `chars_available` コールバックを呼ぶ
  （今のアプリは `vTaskNotifyGiveFromISR`、`freertos/app/usb-console/src/main.c:67-74`）。取れなければ
  1 ms 後のワンショットのアラームで再試行する。
- ユーザー割り込みの優先度は既定の 0x80（`irq.c:705-715`）で、FreeRTOS の SysTick・PendSV（0xff）より高い。
  つまり USB の処理は **すべてのタスクより優先され、割り込みとして負荷に数えられる**。
- SOF の割り込みは普段は無効（`dcd_rp2040.c:221-253`。RP2040 のエラッタ E15 の回避
  `TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX` は既定で無効）。計画のリスクの「Pico SDK の USB の処理（約 1 ms ごと）」は、
  このビルドでは「USB の転送があるときだけ」が正しい。
- 出力 `stdio_usb_out_chars()`（100-132 行）: タスクの文脈で `stdio_usb_mutex` を最大 1 s 待って取り
  （`PICO_STDIO_DEADLOCK_TIMEOUT_MS`、`pico_stdio/include/pico/stdio.h:41`）、自分で `tud_task()` と
  `tud_cdc_write_flush()` を呼ぶ。送信バッファーは **64 バイト**（`pico_stdio_usb/include/tusb_config.h:41-42`）。
  空きが無いと **ミューテックスを持ったまま `tud_task()` を空回りして待ち**、500 ms（`PICO_STDIO_USB_STDOUT_TIMEOUT_US`、
  `stdio_usb.h:33`）空かなければ残りを捨てる。端末が開いていない（DTR が無い）ときは捨てる（`stdio_usb_connected()`）。
- 入力 `stdio_usb_in_chars()`（144-169 行）: 受信バッファーは 64 バイト。TinyUSB はバッファーが一杯なら NAK するので
  入力は落ちない（ホストが待つ）。
- `mutex_t` の待ちは `configSUPPORT_PICO_SYNC_INTEROP` でイベントグループの待ちになる（`port.c:976-1075`）。
  所有者の ID は割り込みの中では「コア番号 + 例外番号 × 2」（`port.c:930-947`）なので、割り込みとタスクで別の所有者に
  なる。

### 7.2 高負荷時（6667 Hz の取得中）に詰まる・落ちる要因 ［推］

1. **大量の出力の間の空回り**: `kernel thread stacks`、`dmesg`、`help` などの長い出力は、64 バイトずつホストに
   読まれるのを空回りで待つ（7.1）。そのタスクの優先度で CPU を使い続けるので、コンソールより低い優先度の
   タスクが止まり、`cpu_active` が上がる。Zephyr 版の CDC ACM（`rx-fifo-size = <2048>`、
   `zephyr/app/usb-console/app.overlay:10-15`）と比べて、バッファーが小さい。
2. **ホストが読まないと 500 ms ごとに出力が捨てられる**（端末を開いたまま止めた場合）。Zephyr 版のシェルの振る舞いとは違う
   ［未: Zephyr 版の CDC ACM の満杯時の動作は読んでいない］。
3. **ミューテックスの取り合い**: タスクが出力中（ミューテックス保持）だと、割り込み側の `tud_task()` は 1 ms 後へ
   延びる。受信と制御転送の応答が最大 1 ms 程度遅れる。
4. **割り込みの中の `printf` は使えない**: 競合すると `xPortLockInternalSpinUnlockWithBestEffortWaitOrTimeout()` の
   `configASSERT(!portCHECK_IF_IN_ISR())`（`port.c:1081`。待つ版も `port.c:979`）に当たる。ログは RAM のリングへ書き、出力はタスクで行う
   （Zephyr 版の遅延ログと同じ形）。
5. **改行の変換**: Pico SDK の stdio は既定で `\n` を `\r\n` にする（`PICO_STDIO_DEFAULT_CRLF` 1、`pico_stdio/include/pico/stdio.h:31`）。シェル側でも
   `\r\n` を出すと二重になる［未: 今の `cli.c` の出力と組み合わせて確かめていない］。
6. 割り込みを長く止める処理（フラッシュの消去 34〜40 ms）の間は USB の処理も止まる（3.4）。

### 7.3 TinyUSB を専用タスクで回す構成との比較 ［推］

| | `pico_stdio_usb`（今） | TinyUSB を専用タスクで回す |
|---|---|---|
| `tud_task()` の文脈 | ユーザー割り込み（0x80）と、出力するタスク | 1 本のタスク（優先度を自由に決める） |
| 6667 Hz の取得との関係 | USB の処理が常に取得のタスクより先に走る | 取得のタスクより低くできる |
| 作る量 | 無し（今動いている） | `CFG_TUSB_OS=OPT_OS_FREERTOS` の設定、記述子、CDC のバッファー、BOOTSEL（1200 bps とリセットのインターフェース）を自分で用意。`PICO_STDIO_USB_ENABLE_TINYUSB_INIT` 0 と `LIB_TINYUSB_DEVICE`（`stdio_usb.h:46-56`） |
| 送信バッファー | 64 B（`CFG_TUD_CDC_TX_BUFSIZE` は上書きできる） | 自由 |

- `CFG_TUD_CDC_TX_BUFSIZE`・`RX_BUFSIZE` は `#ifndef` なので（`tusb_config.h:38-43`）、今の構成のままでも大きくできる。
  まずは今の構成でバッファーを大きくして、CP-F2 で取得中に長い出力を流して確かめるのが安い。

### 7.4 USB が列挙されなくなる危険 ［推］

- `CLAUDE.md` の例は、NuttX の計測版でスケジューラーのフックを自作したときのもの（原因未調査）。今の構成では
  USB は割り込みだけで回り（7.1）、スケジューラーが動かなくても、タスクが空回りしても列挙は進む。
  FreeRTOS のトレースマクロ（`traceTASK_SWITCHED_IN` など）で重い処理をしない限り、同じ形の危険は小さい。
- 危険が残るのは: 割り込みを長く止める処理を起動直後（列挙中）に行う場合（KV の形式違いで全面消去するときに
  消去 2 回で約 80 ms）、`irq_set_exclusive_handler` で `USBCTRL_IRQ` やユーザー割り込みを上書きする場合、
  割り込みを 0x80 より高い優先度で長く走らせる場合。
- 復旧は `bootsel` コマンドが使えなくなるので手動の BOOTSEL になる。CP の前に、起動直後の処理を短くしておく。

---

## 8. 割り込みと周辺回路

### 8.1 割り込みハンドラーの登録 ［確］

- Pico SDK は RAM のベクター表を使い、`irq_set_exclusive_handler(num, handler)`（`hardware_irq/include/hardware/irq.h:346`）
  で 1 つのハンドラーを、`irq_add_shared_handler()` で共有ハンドラーを登録する（`hardware_irq/irq.c:267` 以下。排他と共有を同じ番号に混ぜたときの振る舞いは読んでいない）。`isr_i2c0` のような名前の関数を定義して静的に置き換えることもできるが、SDK は勧めていない
  （同 62-64 行）。
- 自前の I2C0 のドライバー（NuttX 版 `nano_i2c.c:1140-1199`、`nano_dma.c:501-512`）を載せるときの取り合い:
  - `I2C0_IRQ`: SDK の `hardware_i2c` は割り込みを使わない。`irq_set_exclusive_handler(I2C0_IRQ, …)` で問題ない。
  - `DMA_IRQ_0`・`DMA_IRQ_1`: NuttX 版は両方に自分のハンドラーを付ける。今のアプリでほかに DMA を使うライブラリーは
    無い（`pico_stdio_usb`、`hardware_flash`、`pico_flash` は DMA を使わない）［確: ソースを grep］。
    SDK の `dma_channel_claim()` の表は使わないので、将来 SDK の DMA 利用者（`pico_cyw43` など）を足すと衝突する。
    使うチャネルを起動時に `dma_channel_claim()` で押さえておくと安全［推］。
  - **NuttX 版の `nano_dma.c:501-503` は DMA ブロック全体をリセットする**。FreeRTOS 版でも SDK の利用者が無い今は
    問題ないが、上と同じ理由で、リセットしたことを文書に書く。
  - リセットとクロック: I2C0 と DMA は `RESETS` を直接操作しており（`nano_i2c.c:1140-1142`）、SDK の
    `reset_block`/`unreset_block_wait` と同じ。I2C は clk_sys で動く。SDK の起動処理（`pico_runtime_init`）は
    クロックを 125 MHz にする（Zephyr 版・NuttX 版と同じ）［確: SPI の計算で clk_peri = 125 MHz を前提に一致］。
  - ピン: `gpio_set_function(12/13, GPIO_FUNC_I2C)` と pad の設定（Zephyr の pinctrl:
    `zephyr/zephyr/boards/arduino/nano_connect/arduino_nano_connect-pinctrl.dtsi:20-26`、入力有効・シュミット有効）。
- 優先度: Pico SDK の既定では周辺はすべて 0x80、SysTick・PendSV は 0xff。Zephyr 版は周辺がすべて最低の 3
  （NuttX 版 `nano_gpio.c` 冒頭のコメント）で、相互の関係（周辺どうしは同じ優先度）は同じ。

### 8.2 INT1 の GPIO 割り込み ［確］

- LSM6DSOX の INT1 は GPIO24（`zephyr/zephyr/boards/arduino/nano_connect/arduino_nano_connect.dts:124-128`）。
  GPIO の割り込みを使うのはこれだけ（NINA は READY をポーリング、`nuttx/drivers/nina/nina_poller.c`）。
- Pico SDK の `IO_IRQ_BANK0`（`hardware_gpio/gpio.c`）:
  - `gpio_set_irq_callback()`（206 行）: コアに 1 つのコールバック。既定のハンドラー（153-171 行）が各ピンの
    事象を確認（`gpio_acknowledge_irq`）してからコールバックを呼ぶ。
  - `gpio_add_raw_irq_handler_with_order_priority_masked()`（219 行）: ピンのマスクつきの共有ハンドラー。
    事象の確認はハンドラーの責任。
  - `gpio_set_irq_enabled()`（173-194 行）は、有効・無効を切り替える前にその事象を確認（消去）する。
- Zephyr の `gpio_rpi_pico.c` はもともと Pico SDK の `hardware_gpio` の上に書かれている（NuttX 版
  `nano_gpio.c:8-27` の説明）。FreeRTOS 版は互換層の `gpio_pin_interrupt_configure`・`gpio_add_callback` を
  `gpio_set_irq_enabled()` と `gpio_add_raw_irq_handler_masked(1u << 24, …)` で作れば、Zephyr と同じ順序
  （事象を消してから INTE を変える、割り込みでは事象を消してからコールバック）になる［推］。

### 8.3 SPI1（NINA、ポーリング） ［確］

- ピン: SCK GPIO14、TX GPIO11、RX GPIO8（入力有効）、CS GPIO9（GPIO で駆動）、READY GPIO10（プルアップ）、
  RESET GPIO3、IRQ GPIO2（プルダウン）（`arduino_nano_connect-pinctrl.dtsi:40-48`、`arduino_nano_connect.dts:149-158`）。
- Pico SDK の `spi_set_baudrate()`（`hardware_spi/spi.c`）で 8 MHz を指定すると、clk_peri 125 MHz から
  CPSDVSR 2、SCR 7（7.8125 MHz）になる。Zephyr 版と同じ設定（NuttX 版 `nano_nina.c:21-24` の説明）。
- `spi_write_read_blocking()` はポーリングで、割り込みも DMA も使わない。

### 8.4 割り込みから呼べる API と優先度の制約 ［確］

- Cortex-M0+ には BASEPRI が無く、RP2040 ポートは **`configMAX_SYSCALL_INTERRUPT_PRIORITY` を使わない**。
  クリティカルセクションは PRIMASK（`portmacro.h:175-189`）で、割り込みの中の `…FromISR` も
  `ulSetInterruptMaskFromISR()`（PRIMASK）。→ **どの優先度の割り込みからでも `…FromISR` を呼べる**。
  その代わり、クリティカルセクションの間はすべての割り込みが止まる（Zephyr の M0+ の `irq_lock()` と同じ）。
- 割り込みの中では `…FromISR` 版だけを使い、`portYIELD_FROM_ISR()`（`portmacro.h:115`）で切り替える。
  互換層の `k_sem_give()` などは `portCHECK_IF_IN_ISR()`（IPSR、`portmacro.h:147-153`）で分ける。
- Pico SDK の `mutex_t`・`sem_t` を割り込みから「待つ」と `configASSERT` で止まる（7.2 の 4）。

### 8.5 割り込み用スタック（MSP） ［確］

- 1 コアの `vPortStartFirstTask()`（`port.c:215-236`）は **MSP を戻さない**（戻すのは 2 コアの分岐で、
  `configRESET_STACK_POINTER` のとき、238-244 行）。MSP は `main()` から呼んだときの位置のまま、割り込み用に使われる。
- MSP の領域は SCRATCH_Y（`0x20041000`〜`0x20042000`、4 KiB）の上端から。予約の大きさは `PICO_STACK_SIZE`
  （既定 0x800 = 2 KiB、`pico-sdk/src/rp2040/pico_platform/include/pico/platform.h:30-33`、`crt0.S:663`）。
  今のビルドで `__StackTop = 0x20042000`、`__StackBottom = 0x20041800`
  （`freertos/build/usb-console/usb-console.elf.map:4837-4841`）。Zephyr 版の `CONFIG_ISR_STACK_SIZE=2048` と同じ大きさ。
- 2 KiB を超えても SCRATCH_Y の残りと SCRATCH_X（コア 1 のスタック、未使用）に伸びるだけで、ガードは無い。
  `configCHECK_FOR_STACK_OVERFLOW` はタスクのスタックしか見ない。
- ［推］使用量を見るには、`main()` の最初で `__StackBottom` から現在の SP までを模様で埋め、`kernel thread stacks`
  相当のコマンドで残りを数える（Zephyr の `CONFIG_INIT_STACKS` と同じ考え方）。`PICO_USE_STACK_GUARDS`
  （MPU のガード、`pico_runtime_init/include/pico/runtime_init.h:477-491`）もあるが RP2040 での動作は確かめていない。

---

## 9. チップ ID

（subagent の調べ。メインはトピックの作り方を照合した。）

- ［確］Zephyr 版の `hwinfo_get_device_id()`（`zephyr/zephyr/drivers/hwinfo/hwinfo_rpi_pico.c:31-65`）は `irq_lock()` の中で
  `flash_get_unique_id()` を呼び、8 バイトをそのまま返す。`CONFIG_HWINFO_RPI_PICO_CHIP_ID` は RP2350 専用
  （`Kconfig.rpi_pico:13-17`）。
- ［確］0x4B の読み方は両方の SDK で同じ（コマンド 0x4b、ダミー 4 バイトのあとの 8 バイトを `id[i] = rxbuf[1+4+i]`）:
  `freertos/pico-sdk/src/rp2_common/hardware_flash/flash.c:24-27,359-371`、
  `zephyr/modules/hal/rpi_pico/src/rp2_common/hardware_flash/flash.c:24-27`。
- ［確］Pico SDK の `pico_unique_id` は main の前に `flash_get_unique_id()` を 1 回呼んで保存し（`unique_id.c`）、
  `pico_get_unique_board_id()` はその写し（8 バイト、`unique_id.h:42`）を返す。文字列版は先頭バイトの上位ニブルから大文字の 16 進。
  **バイトの並びは Zephyr の `hwinfo_get_device_id()` と同じ**。
- ［確（間接）］`host/tests/test_board.py:16-17` に Zephyr 版・NuttX 版の USB シリアル `503533748A8C9218` があり、FreeRTOS 版の `id` も
  同じ値を表示した。NuttX のテスト（`nuttx/tests/zephyr_compat_misc.c:147-155`）が期待するセッション ID は
  `503533748a8c92180000004f00000000`。
- ［確］**MQTT のトピックにはチップ ID を使っていない**。`imu/<device_id>/raw`・`/status`・`/info` とクライアント ID `imu-<device_id>` の
  `device_id` は KV の `imu.device`（既定 `"rp2040"`）から作る（`zephyr/app/usb-console/src/imu_raw_mqtt.c:375-379`、
  `imu_stream.c:37,42,668`）。チップ ID はセッション ID（16 バイト）の先頭 8 バイトだけ（続く 4 バイトは `imu.session` を 1 増やした値、
  ビッグエンディアン。残りは 0。`imu_stream.c:603-626`）。
- ［確］NuttX 版は `nano_chipid.c` が起動時に割り込みを止めて 0x4B を 1 回読み、`zephyr_hwinfo.c:21` がその写しを返す
  （NuttX の `board_uniqueid()` は使わない）。
- 提案: `hwinfo_get_device_id()` を `pico_get_unique_board_id()` からの memcpy（8 バイトまで）で作る。起動前に読み終えているので、
  実行中に XIP を止めない。

---

## 10. Telnet と速度測定、Zephyr 標準のシェルコマンド

（subagent の調べ。）

### 10.1 Zephyr 版 ［確］

- Telnet: `zephyr/zephyr/subsys/shell/backends/shell_telnet.c`（810 行、Apache-2.0）。ポート 23、行バッファー 80、送信タイムアウト 100 ms
  （`.config:1890-1911`）。`SUPPORT_COMMAND` が無いので IAC は読み捨てて応答しない（249-266 行）。2 本目は accept してすぐ close し、
  `Telnet client already connected.` をログに出して 1 本目を保つ（411-463 行）。依存はシェル本体（`SHELL_DEFINE`、
  `shell_transport_api`）、`net_socket_service`（339 行。スレッド、`zsock_poll`、`zvfs_eventfd`、`k_condvar`）、`k_work_delayable`。
- zperf: ビルドに入るのは `api`・`common`・`udp_pacing`・`iperf3`・`iperf3_client`・`iperf3_server`・`shell` で計約 5,380 行（Apache-2.0、
  `subsys/net/lib/zperf/CMakeLists.txt:6-25`）。ほかに `lib/utils/json.c`（2,352 行）、`net_socket_service`、`k_work_q`、`sys_rand_get`。
  ソケットは `zsock_accept/bind/listen/getsockname/fcntl/setsockopt/recvfrom/sendto`。

### 10.2 NuttX 版 ［確］

- Telnet: `nuttx/apps/init/nano_telnet.c`（347 行、Apache-2.0）。NuttX の Telnet ドライバー（`drivers/net/telnet.c`、1,279 行）が土台で、
  accept したソケットを `/dev/telnetN` にして nsh を動かす。FreeRTOS には土台が無いのでそのままは使えない。
- iperf: `nuttx-apps/netutils/iperf`（`iperf.c` 1,006 行、`iperf_main.c` 342 行、Apache-2.0）と argtable3（約 13,700 行）、パッチ 0005・0006。
  依存は pthread、`clock_gettime`、`usleep`、`inet_addr`・`inet_ntoa`、`netlib`、`prctl`。iperf2（ポート 5001）。

### 10.3 FreeRTOS 版で作る量（見積もり ［推］）

| 案 | 中身 | 量 | 利点・欠点 |
|---|---|---|---|
| Telnet A | `shell_telnet.c` の受け付け・2 本目を断る処理・IAC の読み捨て・行バッファー・送信タイムアウトを、FreeRTOS のシェルのバックエンドとして写す。Telnet タスクが `zsock_poll` で待つ | 400〜600 行 + 他タスクのログを Telnet へ出す経路 | Zephyr と同じ挙動 |
| Telnet B | `nano_telnet.c` の方針で自作 | 同程度 | 写せる部分が少ない |
| 速度 A | zperf（iperf3）を写す | 約 5,400 行 + JSON 2,350 行。ソケット層に `bind/listen/accept/getsockname/fcntl/SO_RCVTIMEO` と `k_work_q` | Zephyr と同じ構文、PC 側は `iperf3`。量が最大 |
| 速度 B | iperf2 を写す | 約 1,350 行 + argtable3 約 13,700 行 + pthread の代わり | NuttX と同じ。構文は Zephyr と違う |
| 速度 C | iperf3 互換を最小限に自作 | 800〜1,500 行 | 少ない。UDP と JSON の交換をどこまで作るかで変わる |

### 10.4 文書が使い方として挙げる Zephyr 標準のシェルコマンド ［確］

出典: `CLAUDE.md`、`README_ja.md`、`nuttx/README_ja.md`、`design/imu-raw-mqtt-design-ja.md`、`zephyr/app/usb-console/drivers/README_ja.md`。

| コマンド | Zephyr の元（行数） | FreeRTOS 版で要るか |
|---|---|---|
| `wifi cred add/delete/list/auto_connect` | `wifi_credentials_shell.c`（446）。NuttX 版の写し（567） | 要る（`script/kv_transfer.py` が送る） |
| `wifi connect`・`disconnect`・`scan`・`status` | `wifi_shell.c`（6,289 のうち使う部分約 1,000 行: 281-460、838-1645 行） | 要る |
| `net iface` | `iface.c`（1,008、`net_if` の内部に依存） | 自作（100〜200 行［推］） |
| `kernel reboot` | `reboot.c`（42） | 要る（引数を見ずに即再起動する挙動も合わせる） |
| `kernel thread stacks`・`thread list` | `stacks.c`（107）・`list.c`（141） | 要る（`uxTaskGetSystemState` で作る。MSP の行も要る） |
| `kernel ps` | アプリ側（Zephyr 版 `main.c:36-60`） | 要る（今の `ps` を作り直す） |
| `kernel uptime`・`kernel version` | `uptime.c`（52）・`version.c`（22） | `kernel version` は `kv_transfer.py:95-105` の `detect_firmware` が送る |
| `date` | `date_service.c`（205） | あると対応がそろう |
| `devmem` | `devmem_service.c`（380） | 要る（`CLAUDE.md` の異常停止の試験）。読み出しだけなら小さい |
| `zperf` | 10.3 | 要る（案を選ぶ） |
| `i2c`・`gpio`・`log`・`device`・`retval`・`rem`・`history` | Zephyr 標準 | README は「Zephyr 標準」とだけ書く。PC 側のツールが送るのは `imu start/stop/status`、`imu cal reload`、`kv get/sethex` だけ |

- ［確］`script/kv_transfer.py` の `detect_firmware` は `kernel version` → `uname -a` で版を見分けるので、今のままでは FreeRTOS 版は
  `unknown` になる。

---

## 11. 互換層のカーネル部分を FreeRTOS で書くときの対応

（subagent の調べ。読んだだけで、コンパイルも実機確認もしていない。メインは `vPortExitCritical()` の振る舞い
（`port.c:463-473`）と `CONFIG_THREAD_MAX_NAME_LEN=32` を照合した。）

### 11.1 今の互換層が提供している API ［確］

`nuttx/drivers/compat/zephyr/kernel.h:7-41` ほか:

- タイムアウト: `k_timeout_t`（int64 の tick）、`K_NO_WAIT`・`K_FOREVER`・`K_TICKS`・`K_MSEC`（切り上げ）・`K_SECONDS`・`K_TIMEOUT_EQ`。
- 同期: `k_mutex`（`K_MUTEX_DEFINE`）、`k_sem`（`K_SEM_DEFINE`、`K_SEM_MAX_LIMIT`、init・take・give・reset・count_get）、セマフォだけの
  `k_poll` と `k_poll_event_init`。
- ワーク: 論理は OS に依存しない `zephyr_work.c`、OS の部分は `z_add_timeout()`・`z_abort_timeout()`。
- スレッド: `K_THREAD_STACK_DEFINE`・`K_THREAD_STACK_SIZEOF`・`K_PRIO_PREEMPT`・`k_thread_name_set`・`K_THREAD_DEFINE`（`K_THREAD_DEFINE_START`
  で起動）・`k_current_get`。
- 時間: `k_uptime_get`・`k_sleep`・`k_cycle_get_32/_64`・`k_cyc_to_us_floor64/ceil64`・`sys_clock_hw_cycles_per_sec`。
  `lsm6dso_os.h`・`nina_os.h` は別に `k_busy_wait`・`k_msleep`・`k_usleep`・`k_cyc_to_ns_floor64`。
- `spinlock.h` は `sched_lock` と `up_irq_save`、`atomic.h` は get・set・clear・cas・add・inc・or を `k_spin_lock` で作る。
  `zephyr_rtio_workq.c`・`zephyr_mem_blocks.c` は `k_spin_lock` しか使わない。

### 11.2 FreeRTOS（V11.3.3、RP2040 ポート）の性質 ［確］

- 1 コアの `taskENTER_CRITICAL` は `cpsid i` と全体で 1 つの入れ子の数（`port.c:453-473`、数は 0 から、`port.c:118`）。
  `taskENTER_CRITICAL_FROM_ISR` は PRIMASK を保存して戻すだけ（`task.h:243,262`）。
  **タスクの文脈で PRIMASK だけのロックを取り、その中で `xSemaphoreGive` などを呼ぶと、中の `taskEXIT_CRITICAL` で数が 0 に戻って
  割り込みが許可される**（`port.c:467-471`）。`k_spin_lock` はタスクの文脈では `taskENTER_CRITICAL` を使う必要がある。
- yield は PendSV を立てるだけ（`port.c:439-447`）。割り込みを禁止している間は切り替わらず、解いたときに切り替わる（Zephyr と同じ）。
- タスクの関数が戻ると `prvTaskExitError` → `panic_unsupported()`（`port.c:199-204`）。
- tick フックは tick 割り込みの中で呼ばれる（スケジューラーを止めている間も。溜まった tick を巻き戻すときは呼ばれない、
  `tasks.c:4999-5012,5066-5071`）。
- 優先度の継承は 1 段だけで、持っているミューテックスを全部返すまで元に戻らない（`tasks.c:6869-6870`）。タイムアウト後の戻しは
  `vTaskPriorityDisinheritAfterTimeout`（`tasks.c:6942`）。Zephyr v4.5 は持ち主の鎖を最大 16 段たどる（`kernel/mutex.c:143-157`）。
- `pcTaskGetName()` は TCB の中の書き換えられる配列を返す（`tasks.c:4279-4292`）。今の `configMAX_TASK_NAME_LEN` は 16 で、
  `imu_raw_mqtt_tid`（16 文字）や `sensor_shell_processing_tid`（27 文字）が切れる。Zephyr 版は `CONFIG_THREAD_MAX_NAME_LEN=32`。
- 時分割は 1.5。Zephyr は `K_THREAD_DEFINE` のスレッドをスケジューラーをロックしたまま全部起動する（`kernel/init.c:93-105`）。
- Pico SDK は `main()` の前にコンストラクターを回す（`newlib_interface.c:203-207`）。SDK がタスク通知を使うのは
  `pico_async_context_freertos` の index 0 だけ。

### 11.3 対応表（提案 ［推］）

| Zephyr の API | FreeRTOS での実現 | 注意・再現しにくい点 |
|---|---|---|
| `k_spin_lock`・`k_spin_unlock` | タスクでは `taskENTER_CRITICAL`、割り込みでは `taskENTER_CRITICAL_FROM_ISR`。鍵に「割り込みの中だったか」と PRIMASK を入れる | PRIMASK だけにすると中の API が割り込みを許可してしまう（11.2） |
| `atomic_*` | NuttX 版と同じく `k_spin_lock` の上に作る | なし |
| `k_mutex`・`K_MUTEX_DEFINE` | `xSemaphoreCreateRecursiveMutexStatic` + ready の印。全部 0 のものは最初の lock/unlock でタスクの文脈で作る（`taskENTER_CRITICAL` の中。待ち 0 の送信なので入れ子でも問題ない、`queue.c:634`）。`K_MUTEX_DEFINE` はコンストラクターで作る | 戻り値を Zephyr に合わせる（`kernel/mutex.c:419-430`: K_NO_WAIT で −EBUSY、タイムアウトで −EAGAIN、持ち主でない unlock で −EPERM、ロックされていない unlock で −EINVAL）。継承が 1 段だけなのは再現できない。1 個約 80 バイト |
| `k_sem`・`K_SEM_DEFINE` | 案 A: 数・上限・待ち手の列を自前で持ち、タスク通知の index 1 で起こす（静的な初期化子が書ける）。案 B: `xSemaphoreCreateCountingStatic` をコンストラクターで作る | 案 B は「最初の使用で作る」ができない（最初が割り込みからの give だと、作るときの `taskENTER_CRITICAL` がロックを崩す）。上限での give は errQUEUE_FULL で「何もしない」と合う。`k_sem_reset` は `xQueueReset` |
| 割り込みからの `k_sem_give` | `xSemaphoreGiveFromISR` か `vTaskNotifyGiveIndexedFromISR` → `portYIELD_FROM_ISR` | M0+ には BASEPRI が無く、優先度の制約は無い（8.4） |
| `k_poll`（セマフォ、NOTIFY_ONLY） | NuttX 版と同じ構造で、待ち手は通知 index 1。待つ前に `xTaskNotifyStateClearIndexed` で古い通知を消し、`ulTaskNotifyTakeIndexed(1, pdTRUE, N+1)` | キューセットは使わない（受け取らずに知らせる動きと上限の扱いが合わない）。`configTASK_NOTIFICATION_ARRAY_ENTRIES` を 2 に |
| タイムアウトの換算 | 有限の N は `min(N+1, portMAX_DELAY-1)`、`K_NO_WAIT` は 0、`K_FOREVER` は `portMAX_DELAY` | 32 ビットの tick で 4.97 日を超える待ちは切り詰める |
| `k_sleep`・`k_msleep`・`k_usleep` | `vTaskDelay(N+1)`、0 は `taskYIELD()` | 1.3 |
| `k_busy_wait` | `busy_wait_us_32` | なし |
| `z_add_timeout`・`z_abort_timeout` | 期限の早い順の自前の列を tick フック（割り込みの中）で先頭だけ見る | ソフトウェアタイマーは使わない（stop と change が非同期で、`zephyr_work.c:408-423` の「割り込み禁止の間は時計が進まない」前提が崩れる）。100 µs ごとの負荷は未測定 |
| `K_THREAD_DEFINE`・`_START` | `StaticTask_t` とスタックを静的に定義し、起動のタスクが `xTaskCreateStatic(…, size/4 語, 20−Zephyr)`。入口が戻ったら `vTaskDelete(NULL)` | まとめての起動は `vTaskSuspendAll`／`xTaskResumeAll` で囲めば再現できる。協調優先度（負の値）は高い優先度で近似する（協調の意味は再現しない） |
| `k_work_queue_start` | `xTaskCreateStatic`。`struct k_work_q` の OS ごとの部分に `StaticTask_t` | T20 の「型を OS ごとのヘッダーに出す」と関係 |
| `k_thread_name_set` | `pcTaskGetName(t)` の配列に strlcpy | `configMAX_TASK_NAME_LEN` を 32 に |
| `k_current_get`・`k_tid_t` | `xTaskGetCurrentTaskHandle()`・`TaskHandle_t` | なし |
| `k_uptime_get` | `time_us_64() / 1000`（tick を使わない） | なし（1.1） |
| `k_cycle_get_*` と変換 | `time_us_32()`・`time_us_64()`（1 MHz）。変換は恒等、ns は ×1000 | NuttX 版と同じ。Zephyr は 125 MHz の SysTick |
| 同じ優先度の時分割 | `configUSE_TIME_SLICING 0` + tick フックで 200 tick（20 ms）ごとに yield | 切り替えのたびに残り時間を戻す処理は再現しない |
| `__ASSERT` | 空 | なし |

---

## 計画（`freertos/PORTING_PLAN.md`）と食い違う点・直すべき点・相談すべき点

### 食い違う点・直すべき点

1. **決定事項「FreeRTOS の tick: 64 ビット」はこのポートのままでは作れない**（1.1。`portmacro.h:77` の `#error`、ポートを直しても
   `port.c:952-974` の SDK 連携が壊れる）。**提案**: tick は 32 ビット・10 kHz、起動からの時間（`k_uptime_*`）は TIMER（`time_us_64()`）から作る。
   ポートにパッチを当てる案はとらない。→ ユーザーに相談（決定事項の変更）。
2. **計画のリスク「Pico SDK の USB の処理（約 1 ms ごと）」**: このビルドでは USB の割り込みがあるときだけ `tud_task()` が走り、1 ms の周期
   タイマーは使わない（7.1）。リスクの書き方を直す。代わりに「USB の処理は優先度 0x80 の割り込みで、すべてのタスクより先に走る」
   「長い出力は 64 バイトの送信バッファーで空回りする」を書く。
3. **Zephyr 版・NuttX 版は tickless**（1.4）。FreeRTOS 版だけ 10 kHz の周期割り込みがあり、見積もりで `cpu_active` に 1.2〜2.4 ポイント
   （未測定）。F01 の確認に「周期 tick の負荷の実測」と「足りなければ自作 tickless（`vPortSuppressTicksAndSleep` の上書き、TIMER のアラーム）」
   の判断を入れる。
4. **F01 に足すもの**: 時分割を 20 ms にする（`configUSE_TIME_SLICING 0` + tick フック、1.5・11.3）、`configMAX_TASK_NAME_LEN` 32、
   `configTASK_NOTIFICATION_ARRAY_ENTRIES` 2、`configUSE_IDLE_HOOK` 1（2.3）、タイマータスク（`Tmr Svc`）の優先度を優先度の表に入れる
   （SDK のロックの起床がこのタスクを通る、1.2）、MSP の大きさ（`PICO_STACK_SIZE`）と使用量の見方（8.5）。
5. **F01 の「異常停止の報告」**: 今の FreeRTOS 版は異常停止で止まったまま再起動しない（5.2）。共通の出口を作り、AIRCR で再起動すると明記する。
6. **F02**: `k_sleep` は `sleep_ms()`（150 µs の空回り）を使わず `vTaskDelay(N+1)`（1.2・1.3）。delayable work の時計は FreeRTOS の
   ソフトウェアタイマーにせず tick フックの自前の列（11.3）。`k_spin_lock` を割り込みの中かどうかで分ける（11.2）。`k_sem` の作り方（案 A／B）を決める。
7. **F04（KV）**: Zephyr の NVS は v4.5.0 では `subsys/kvss/nvs`（4.2）。無改変で写せてホストでは動いた（4.5）。Zephyr の HAL の
   `flash_write_partial()` は上流の SDK に無いので 0xff 埋めのページ書き込みで代える（4.7）。`flash_safe_execute()` は
   `PICO_FLASH_ASSUME_CORE1_SAFE=1` が無いと必ず失敗する（3.2）。リンカーの除外は `pico_add_linker_script_override_path` で部品を差し替える（3.3）。
   **CP-F1 に「Zephyr 版が書いた KV を FreeRTOS 版で読めること」の実機照合を入れる**。
8. **F03（シェル）**: 決定事項「`nuttx/apps/subcmd` を共通化して使う」のうち、`zshell.c` の端末を探す処理（`opendir`・`stat`・`F_GETPATH`・
   pthread）は NuttX 専用で、FreeRTOS では書き直す（6.3）。T23 の共通化の範囲から外すか、OS ごとのファイルにする。`fnmatch` は libc に無いので
   Zephyr の `c_lib_ext/fnmatch.c`（BSD-3）を写す。`getopt` は再入可能版にする。
9. **F09（ソケット）**: newlib にソケット系のヘッダーが一つも無い（6.2）。POSIX の型・定数のヘッダー、`htons`・`inet_pton/ntop`、fd の表、
   非ブロッキング、`SO_RCVTIMEO`、`getsockname` を互換層に作る。Telnet・zperf を写すなら `bind/listen/accept/fcntl/recvfrom/sendto` と
   Zephyr 4.5 の `net_*` の名前も要る。量の見積もりを計画に書く。
10. **errno**: newlib の値は Zephyr 版と全部同じ（6.4）。守るのは `PICO_ERROR_*` を errno に直すことだけ。
11. **チップ ID**: 計画（と今回の依頼文）の「MQTT のトピック `imu/<id>/raw` にチップ ID」は事実と違う。トピックは KV の `imu.device`
    （既定 `rp2040`）から作り、チップ ID はセッション ID の先頭 8 バイトだけ（9）。`pico_get_unique_board_id()` は同じバイト列。
12. **F15**: `script/kv_transfer.py` の `detect_firmware` が FreeRTOS 版を見分けられない（10.4）。`kernel version` の出力かスクリプトを直す作業を入れる。
13. **計画の「BOOTSEL は `bootsel` コマンドで入れるので操作は要らない」**: 正しいが、異常停止や USB が列挙されない不具合のときは手動の BOOTSEL が
    要る（7.4）。

### ユーザーに相談すべき点

- 64 ビットの tick をやめて 32 ビット（10 kHz、約 4.97 日で一周）にし、時刻は TIMER から作ること（上の 1）。
- 速度測定（F11）を zperf（iperf3、約 7,700 行の写し）、iperf2（約 15,000 行）、iperf3 互換の最小の自作（800〜1,500 行）のどれにするか（10.3）。
- 優先度の継承が 1 段だけになる差、協調優先度（負の値）を高い優先度で近似する差を受け入れるか（11.3）。
- 初回の起動で、0xffe000 に Zephyr の NVS でも消去済みでもないデータ（今は OpenMV の FAT の残りの可能性）があるときの扱い（消去するか、
  `kv` コマンドで初期化するか。Zephyr 版には自動消去が無い）（4.9）。
- 周期 tick の負荷が大きすぎたときに、自作の tickless に進むか（1.4）。
- （計画の範囲外）Zephyr の NVS をそのまま写せるなら、NuttX 版も同じ形式にして「版を書き換えるたびに KV が消える」問題を無くすか（4.9）。
- `PICO_USE_STACK_GUARDS` で MSP のあふれを検出するか（5.3・8.5）。

## 確かめられなかったこと

- 10 kHz の周期 tick の実際の負荷（`cpu_active`）と、tick フックで時計の列・時分割を回す負荷（見積もりだけ）。
- 64 ビットの tick をポートに足したときに、警告の 2 関数を直せば SDK 連携が正しく動くか（直していない）。
- Zephyr 版が実機で書いた NVS のイメージを、写した NVS で読めるか。フラッシュ末尾 8 KB の今の中身。
- フラッシュの消去中に USB・DMA・センサーの FIFO がどうなるか（FreeRTOS 版では未測定。NuttX 版の割り込み禁止時間の実測を使っただけ）。
  リンカーの ASSERT が実際に失敗する場合。
- 大量の出力（`kernel thread stacks` など）を 6667 Hz の取得中に流したときの USB コンソールの詰まり・取りこぼし。Zephyr 版の CDC ACM の
  満杯時の振る舞い。`\r\n` の二重化。
- 排他ハンドラーと共有ハンドラーを同じ IRQ に混ぜたときの Pico SDK の振る舞い。DMA の全転送の元がフラッシュでないこと。
- `PICO_USE_STACK_GUARDS` の RP2040 での動作。watchdog のリセットで SRAM と scratch が残るか（AIRCR で残ることは NuttX 版で確認済み）。
  HardFault ハンドラーの実機での動作。
- コンストラクターで FreeRTOS のオブジェクトを作れるか（読んだ範囲で問題は見当たらない）。項目 11 の設計のどれもコンパイルしていない。
- newlib の `getopt_long` の並べ替えと未知のオプションでの出力（Zephyr の `sys_getopt_long` との違い）。
- Telnet・zperf を写したときの実際の行数とスタックの量。`wifi_shell.c` の使う部分が互換層の `net_mgmt`・`wifi_mgmt` だけで閉じるか。
- フラッシュのデータシート上の消去・書き込み時間（Web は見ていない）。
