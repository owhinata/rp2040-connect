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
