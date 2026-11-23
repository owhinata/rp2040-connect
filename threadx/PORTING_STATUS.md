# ThreadX 版の作業ログ

計画は `threadx/PORTING_PLAN.md`。タスクごとに、決めたこと・試して入れなかったこと・イメージの大きさ・実機の結果を追記する。

## 設計の検討と計画（2026-11-24）

- ユーザーの依頼「ThreadX 版を追加しましょう。まずは設計検討から」。ThreadX のソース（v6.5.2.202603b_rel）を読み、3 つの案
  （Pico SDK + ThreadX のカーネル、ThreadX の一式、付属の FreeRTOS 互換層）を比べた。
- ユーザーの決定: Pico SDK + ThreadX のカーネル、Pico SDK の部分は先に共有の場所へ切り出す、ネットワークは互換層のソケット層、計画を書いてから進める。
- FreeRTOS 版の自分のコード約 7,300 行のうち、FreeRTOS の API が中心なのは約 1,800 行（計画の表）。
- 未確認: Pico SDK の上でのビルド、ロックの条件を `_tx_thread_preempt_disable` で満たせるか（T0）、大きさと `cpu_active`。
