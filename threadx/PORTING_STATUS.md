# ThreadX 版の作業ログ

計画は `threadx/PORTING_PLAN.md`。タスクごとに、決めたこと・試して入れなかったこと・イメージの大きさ・実機の結果を追記する。

## 設計の検討と計画（2026-11-24）

- ユーザーの依頼「ThreadX 版を追加しましょう。まずは設計検討から」。ThreadX のソース（v6.5.2.202603b_rel）を読み、3 つの案
  （Pico SDK + ThreadX のカーネル、ThreadX の一式、付属の FreeRTOS 互換層）を比べた。
- ユーザーの決定: Pico SDK + ThreadX のカーネル、Pico SDK の部分は先に共有の場所へ切り出す、ネットワークは互換層のソケット層、計画を書いてから進める。
- FreeRTOS 版の自分のコード約 7,300 行のうち、FreeRTOS の API が中心なのは約 1,800 行（計画の表）。
- 未確認: Pico SDK の上でのビルド、ロックの条件を `_tx_thread_preempt_disable` で満たせるか（T0）、大きさと `cpu_active`。

## T0: 基本の操作と受け入れ試験（2026-11-24、ホストだけ）

- `threadx/script/setup.sh`（ThreadX v6.5.2.202603b_rel = `b2868c3` を `threadx/threadx` へ）、`threadx/script/host_tests.sh`、
  `threadx/drivers/compat/zephyr_kernel_os_impl.h`（基本の操作 26 個、509 行）・`zephyr_kernel_os.c`（ロックの深さと保存したマスクの 2 変数）・`zephyr_os_atomic.h`、
  ボード用の `rp2040/zephyr_os_port.h`（まだビルドに入っていない。arm-none-eabi-gcc で単体のコンパイルだけ確認）、`threadx/tests/compat/`、`priorities_os.h` の下書き。
- 結果: ThreadX の Linux ポートの上で、互換層のカーネルのホストテスト 22 項目が通る（FreeRTOS の POSIX ポートと同じ 3 項目は飛ばす）。約 2.2 秒。`common/` の変更なし。
- ロック: 割り込みを止める + `_tx_thread_preempt_disable++`、いちばん外の解除で戻して `_tx_thread_system_preempt_check()`。マスクだけの対照は spinlock と timeslice が落ちた。
  Cortex-M0 での動きはソースを読んだ範囲（サービスの切り替えは `TX_THREAD_SYSTEM_RETURN_CHECK` で守られている、tick の割り込みも見ている）で、実機では未確認。
- 起こしの印: スレッドごとの `TX_SEMAPHORE` と `tx_semaphore_ceiling_put(…, 1)`。互換層が起こしたスレッドだけが持つ（ほかの方法で作った ThreadX のスレッドは `k_sem_take()` を呼べない）。
- スタックのあとの予約領域: 248 B（TX_THREAD 180 + セマフォ 28 + 入口と引数 8 + 名前 32。FreeRTOS は 136）。`k_mutex` は 52 B。
- 優先度: 互換層の値 = 15 − p（大きいほど急ぐ）、ThreadX の値 = p + 16。Zephyr の −16〜15 が ThreadX の 0〜31 に 1 対 1。
- ミューテックスで ThreadX の内部を使った 2 点:
  1. アプリは作ったあとのミューテックスを 0 で埋める（`mqtt_client_init()` が接続のたびにクライアントごと memset する）。ThreadX は作ったミューテックスを一覧につなぐので、
     そのままだと一覧が壊れる（対照は segfault）。作った直後に一覧から外し、削除しない。
  2. ThreadX は、待ち手がタイムアウトしても持ち主の優先度を下げない（put のときに下げる）。受け入れ試験（Zephyr・FreeRTOS の動き）に合わせ、タイムアウトのときに
     `_tx_mutex_priority_change()` で戻す処理を足した。
- 時分割: 計画は「時分割なし」だったが、試験の timeslice（Zephyr の 20 ms）が通らない。ThreadX 自身の `time_slice` 20 を、Zephyr の優先度 0 以上のスレッドに付けた（計画を直した）。
- タイマー: `TX_TIMER_TICKS_PER_SECOND 1000`、`TX_TIMER_PROCESS_IN_ISR`（タイマーのスレッドなし）。
- 境界について分かったこと（T5 で `design/os-boundary` に反映する）: 起こしの印は全スレッドが持つ前提、全部 0 の `k_mutex` は「使ったあとの memset」にも耐える必要、
  優先度の数は「大きいほど急ぐ」、`<zephyr/sys/clock.h>`・`__assert.h` は OS が書くものの一覧に無い、スレッドの終わりのあと同じスタックで起こし直すには削除が要る。
- 未確認: 実機と Cortex-M0 での動き全部、`TX_NOT_INTERRUPTABLE`・`TX_DISABLE_ERROR_CHECKING` のビルド、長時間、実行可能な持ち主への優先度の変更。

## T1 の下調べ（2026-11-24、読むだけ）

- FreeRTOS 版の自分のコードの約 3 分の 2 は FreeRTOS を呼ばない。切り出しのあと ThreadX 版が書くのは 1,100〜1,250 行の見込み（予測）。
- 新しい境界の案 5 つ（NINA の DMA の待ち、アイドルの入口、スタック検査の入口、起動、`ps` とバージョン）、7 段階の移動の順、置き場所の案
  （`common/pico_sdk/`、`common/drivers/<ドライバー>/port/pico_sdk/`、`common/compat/os_generic/`）をユーザーに提示し、返事待ち。
- 注意: `configASSERT()` は `__FILE__`・`__LINE__` を含むので、`common/compat/zephyr_init.c:83` と `nina_os.c:373` より上の行数を変えるとイメージが変わる。
  `common/tests/i2c_dw_sync.py` は `port/freertos/i2c_dw_instance.h` をパスで読む。`os_clock.h` の `os_ticks_from_us_ceil()`・`os_sleep_us()` は呼び手が無い。

## T2 の前半: 実機の CPU の上の ThreadX と、受け入れ試験の実機での実行（2026-11-24）

- T1 と並行して進めた（ユーザーの提案: 分担できないか）。`threadx/` の新しいファイルだけ。
- `threadx/script/setup.sh` が Pico SDK 2.3.1・Arm GNU 15.2・picotool も取得、`threadx/script/build.sh [--clean] [APP]`（`threadx/app/` か `threadx/tests/` の APP）。
  `threadx/app/usb-console/`: `tx_user.h`、`src/tx_initialize_low_level.c`、`src/tick.c`（`isr_systick`）、`threadx_kernel.cmake`（ThreadX の `common/src` と `ports/cortex_m0/gnu/src` を無変更で）。
  `threadx/drivers/compat/zephyr/sys/{clock.h,__assert.h}`（FreeRTOS のものと同じ内容。T1 のあと共有のものに置き換える）。
- 起動: `main()` → `_tx_initialize_kernel_setup()` → `zephyr_compat_initialize()` と `z_os_thread_start()` → `tx_kernel_enter()`。FreeRTOS 版の `main()` と同じ形で、
  スケジューラーの前にスレッドを作れる。
- 割り込みベクター: Pico SDK の crt0 の弱い `isr_pendsv`・`isr_svcall` を、リンクの `--defsym` で ThreadX のハンドラーに向ける。`isr_systick` は自前の C。
- **優先度: PendSV・SVCall は 0xC0（最低）、SysTick は 0x80（SDK の既定のデバイスの割り込みと同じ）。** ThreadX の Cortex-M0 のスケジューラーは、実行できるスレッドが無いあいだ
  PendSV のハンドラーの中で待つので、SysTick を PendSV と同じ優先度にすると tick が入らず止まる。最低の優先度（0xC0）にした割り込みは、アイドルのあいだ動かない（いまは無い）。
- 試験用のファームウェア `threadx/tests/board_kernel/`（製品には入らない。USB の製品名 "ThreadX kernel test"、LED の点滅で状態を表示）。
- **実機の結果**: USB が列挙され、tick は TIMER の 1,000,008 µs で 1000 回。ロックの直接の試験 3 つ（ロック中の起こし、入れ子、ロック中のアラームの割り込み）が通る
  （アラームは解除の 18 µs 後、起こされたスレッドはその 20 µs 後）。互換層のカーネルの試験は **22 項目すべて通り、飛ばした項目なし**（ホストで飛ばした割り込みからの起こし・
  tick の追いつき・実時間も）。T0 でソースを読んだだけだった Cortex-M0 でのロックの動きを確認できた。
- 大きさ（試験用）: text 59,712、bss 37,588。ThreadX のカーネルとポートは約 6.4 kB（エラー検査あり）。`malloc` はリンクされない。
- 製品のファームウェアで決めること: アイドルと `cpu_active`（最低優先度のスレッドを自前で置く案）、Pico SDK のロックや `sleep_ms` が ThreadX と連携しないこと
  （FreeRTOS には SDK の連携の設定がある。ThreadX では WFE で回る）、割り込みスタックの検査、ThreadX のスケジューラーの待ちに WFI が無いこと。
- `script/flash_uf2.sh` は ThreadX を知らないので、書き込みのあと「3 つのどのコンソールも現れない」と表示する（書き込み自体はできる）。T2 の後半で `--target threadx` を足す。
- 実機: ThreadX の試験用のファームウェア。

## T1: Pico SDK の部分の切り出し（2026-11-24）

- ユーザーの決定（勧めの案 4 つ）: 置き場所 `common/pico_sdk/`、互換層の新しい部分 `common/compat/os_generic/`、起動の順は共有、T1 ではイメージの一致を厳密に守る。
- 7 段階で移した。各段階のあと `SOURCE_DATE_EPOCH=1763337600 freertos/script/build.sh --clean` のイメージが移動の前（`freertos/build/baseline/freertos/t1-before/`）と
  バイト単位で一致することを確かめた（メインも最後の状態で確認）。Zephyr は `snapshot-1124a` と一致、NuttX は同じ大きさでビルド時刻の文字列だけが違う。
  ホストテスト: FreeRTOS 68、Zephyr 61、NuttX 73（tick とアイドルの点滅のテストがカーネルの要らない `common/tests` に移り、`run.sh` の件数が 2 増えた）。
- いまの配置:
  - `common/pico_sdk/app/usb-console/`: `main.c`・`commands.c`・`fault.c`・`idle.c`・`irq_stack.c`、`usb-console.cmake`（`pico_sdk_usb_console()`）、`ram_code.cmake`、リンカースクリプトの部分。
  - `common/pico_sdk/drivers/`: 互換層の Pico SDK の部分（GPIO、hwinfo、時計、`zephyr_os_port_rp2040.h`）、コンソールと stdio、KV のフラッシュ、各ドライバーの cmake（`pico_sdk_<x>()`）。
  - `common/compat/os_generic/`: 互換層の `i2c.h`・`sys/clock.h`・`sys/__assert.h`、プレリュード、センサーのシェルの OS 部分、`app_log_os.h`、`wifi_profile_os.h`。
  - `common/compat/kernel_generic/`: `tick_catchup.h`（`Z_OS_TICK_HZ` の上）、空の `zephyr_os_atomic.h` の既定。
  - 各ドライバーの `port/pico_sdk/`: I2C のデバイス・コントローラー・DMA、センサーのデバイスと開始、NINA の SPI・DMA・ピン・中継モード。
  - FreeRTOS に残ったもの: `FreeRTOSConfig.h`、`tick.c`、`priorities_os.h`、`kernel_shell_os_impl.h`、新しい境界の実装 4 つ（`main_os_impl.h` 79 行、`idle_os.h` 17、`fault_os_impl.h` 50、
    `commands_os_impl.h` 95）、`zephyr_kernel_os_impl.h`、`rp2040/zephyr_os_port.h`（59 行に）、`port/freertos/` の `nina_os.c`・`nina_bus_dma_os_impl.h`・`i2c_dw_os_impl.h`。
    `freertos/app/usb-console/CMakeLists.txt` は 65 行。`freertos/drivers/` は `compat/` と README だけ。
- 新しい境界: `main_os.h`（プロンプト、カーネルの準備、メインのスレッドの開始と終了、スケジューラーの開始）、`idle_os.h`（アイドルの 1 回分の名前）、`fault_os.h`（実行中のスレッドの名前、
  OS 自身の致命的なエラーの入口）、`commands_os.h`（カーネルの名前とバージョン、`cmd_ps()`）、`nina_bus_dma_os.h`（DMA の完了の待ち）、`zephyr_os_port_rp2040.h`。
- 下調べから変えた点: `kernel_shell_os_impl.h` は FreeRTOS に残した（共有にしてイメージを保つには、ファイルの行数より多い入口が要る）。`zephyr_sys_clock.c` は `os_generic` でなく
  `common/pico_sdk`（`k_cycle_get_64()` に替えるとコードが変わる）。`nina_bus_dma_os.h` は `always_inline`（そうしないと ISR の関数の順が変わる）で、`port/` でなく `port/pico_sdk/` に置いた
  （`nina_core_independent` が `port/*.h` を単独でコンパイルするため）。`os_clock.h` は削除（使われていたのは `os_tick_count()` だけ）。
- 同時に入れたもの（作業 B、コンパイルだけ確認）: ThreadX 用の `common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h`、`common/drivers/nina/port/threadx/nina_os.c`・`nina_bus_dma_os_impl.h`。
  待ちはドライバーが自分で持つ `TX_SEMAPHORE`（ThreadX のセマフォは軽い作りで、FreeRTOS のキューのセマフォとは事情が違う。スレッドの停止と再開は起こしを取りこぼすので取らない）。
  動作は未確認。ホストテストは未作成。
- 境界について分かったこと（T5 で反映）: `nina_bus_dma_os.h` の「スレッド自身の起こしを共用する」は FreeRTOS の作りのまま。`i2c_dw_os_sem_init()` は境界の外なのに共有のコードが呼ぶ。
- `threadx/` の実機用のファイル（`tick.c`、`threadx_kernel.cmake` など）への T1 の追従は、作業 A（アイドル・実行時間・SDK との連携）の commit に入る。この commit の時点では
  `threadx/script/build.sh` は通らない（ホストテストは通る）。
- 実機: 変更なし（ThreadX の試験用のファームウェアのまま）。FreeRTOS 版はイメージが同じなので書いていない。
