# ローカルドライバーの基準版

[English](README.md) | 日本語

`lsm6dso/` は Zephyr v4.5.0、コミット
`41d10b59730406bbab4ca9f1fbbe9e96aafb4966` の各ドライバーからコピーした。
各ソースの著作権・Apache-2.0 ライセンス表示を維持している。コピーした
ソースには FIFO の内部処理が追加されている。新規の `lsm6dso_fifo.c` は FIFO
ワードの解析処理、`lsm6dso_fifo_hw.c` は FIFO の設定・状態確認・I²C バースト
読み出しの内部処理である。ドライバー内の開始・停止・読み出し関数を
Streaming API に接続している。
`lsm6dso_fifo_decoder.c` は FIFO 用の Zephyr decoder の実装で、通常の単発取得の
decoder と切り替える `get_decoder` に接続している。時刻はホストの読み出し完了時刻と
公称 ODR からの推定であり、I²C 転送と割り込み処理の遅延を含むため、センサーが
生成した正確な時刻ではない。

アプリの CMake は Zephyr のドライバーライブラリー内の対象ソースをコピーに
置き換える。Kconfig、devicetree binding、ST の共通バス層と HAL は Zephyr の
ものを使用する。I²C は Zephyr 標準の `i2c_dw` を使用する。
同一 compatible を持つセンサーにはローカル版が適用される。センサーの標準版と
ローカル版は同時にコンパイルしない。Zephyr のライブラリー構成が変わった場合、
置換対象が見つからなければ構成エラーにする。

`sensor_shell/sensor_shell.c` は同じ Zephyr コミットからのコピーで、フレーム数が
0 のチャンネルと、キャンセル時などのバッファのない完了通知を表示時に除外する。
FIFO 処理は専用の workqueue で動かし、sensor shell の処理スタックを 3 KiB に
設定している。

## TODO

- [ ] **センサー時刻に対応する。** 現在はホストの読み出し完了時刻と公称 ODR から
  サンプル時刻を推定している。LSM6DSOX の FIFO タイムスタンプとタグを用いた時刻の
  再構成を検討し、センサー時刻と RP2040 時刻の対応、カウンターの周回・リセット、
  FIFO overrun 後の不連続を扱う。公称分解能は 25 µs で、実際の時刻精度や ODR には
  センサーの周波数誤差がある。[ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
- [ ] **内部温度の取得に対応する。** 現在は温度チャネルを有効にしておらず、FIFO
  にも温度を保存していない。通常の Sensor Fetch/Get と FIFO ストリーミングでの
  取得方法を整備し、温度タグの解析・単位変換・時刻との対応を検証する。温度は
  センサー内部の値であり、更新レートは最大 52 Hz。
- [ ] **追加データを含めて再評価する。** 時刻・温度を FIFO に追加すると I²C の
  転送量が増えるため、両センサー 6,667 Hz・1 MHz で件数、エラー、転送時間、
  CPU 使用率を再測定する。センサー時刻の FIFO 保存頻度は精度とバス帯域を考慮する。

この構成はビルド成功と生成されたビルドコマンドによるソースの置換を確認済み。
USB shell、I²C scan/speed、sensor get/attr_set、NACK と無効引数を含む実機での
同等動作のうち、100 kHz と 1 MHz での scan、sensor get、および 100 kHz での
accel/gyro の sampling_frequency 設定は実機で確認された。NACK・無効引数などの
確認は残っている。停止後の通常取得と、停止時の無関係なチャンネル表示が出ないことも
確認した。エラー時の復旧、より長時間の動作、他の I²C 利用者との共存は未検証で、
標準 I²C ドライバーに戻した構成は 6667 Hz で 10 秒間の実機測定を行い、FIFO 集計の一致と停止後の sensor get を確認した。

## 実機検証結果

### shell での連続取得

| センサー | 設定 ODR | FIFO ワード | 転送回数 | 転送時間合計 | I²C エラー / バッファ不足 / overrun 観測 |
|---|---:|---:|---:|---:|---:|
| 加速度・ジャイロ | 各 26 Hz | 608 | 19 | 約 0.369 秒 | 0 / 0 / 0 |
| 加速度・ジャイロ | 各 104 Hz | 2,464 | 77 | 約 1.495 秒 | 0 / 0 / 0 |
| 加速度のみ | 104 Hz | 960 | 30 | 約 0.582 秒 | 0 / 0 / 0 |
| ジャイロのみ | 104 Hz | 1,024 | 32 | 約 0.621 秒 | 0 / 0 / 0 |
| 加速度・ジャイロ | 各 208 Hz | 3,904 | 122 | 約 2.366 秒 | 0 / 0 / 0 |

208 Hz の停止後の `ps` では sensor shell のスタック最大使用量は 1,208 / 3,072
バイトだった。停止後の CPU 使用率から取得中の CPU 負荷は分からない。

### `imu_measure` による測定

以下は終了時の残データ回収を追加する前の測定結果である。すべて加速度・ジャイロの
両方を同じ ODR に設定し、サンプルの逐次表示を行わずに測定した。I²C 速度の
「未記録」は実行時の設定が記録されていないことを示す。

| 設定 ODR（各） | I²C | 時間 | 受信サンプル（各） | 観測レート（各） | バッチ / FIFO 転送回数 | FIFO ワード | 転送時間 合計 / 最大 | CPU 使用率 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 208 Hz | 未記録 | 10 秒 | 2,080 | 208.0 Hz | 130 / 130 | 4,160 | 2.520 秒 / 19.4 ms | 1.04% |
| 417 Hz | 未記録 | 10 秒 | 4,176 | 417.6 Hz | 261 / 261 | 8,352 | 5.061 秒 / 19.4 ms | 1.71% |
| 833 Hz | 1 MHz | 10 秒 | 8,368 | 836.8 Hz | 523 / 523 | 16,736 | 1.042 秒 / 2.008 ms | 3.33% |
| 1,667 Hz | 1 MHz | 10 秒 | 16,752 | 1,675.2 Hz | 1,047 / 1,047 | 33,504 | 2.086 秒 / 2.004 ms | 6.60% |
| 3,333 Hz | 1 MHz | 10 秒 | 33,508 | 3,350.8 Hz | 2,094 / 2,095 | 67,048 | 4.186 秒 / 2.864 ms | 13.30% |
| 6,667 Hz | 1 MHz | 10 秒 | 67,015 | 6,701.5 Hz | 2,732 / 2,733 | 134,076 | 8.251 秒 / 4.660 ms | 15.13% |
| 6,667 Hz | 1 MHz | 60 秒 | 402,332 | 6,705.533 Hz | 16,393 / 16,394 | 804,710 | 49.528 秒 / 5.008 ms | 15.05% |

各測定で、未知タグ、I²C エラー、バッファ不足、overrun 観測、RTIO 完了通知欠落、
ストリームエラー、終了処理 timeout はいずれも 0。CPU 使用率は測定区間のシステム
全体であり、集計や他スレッドの負荷を含む。観測レートは測定区間中の受信数からの
概算で、ODR の精度と区間両端のバッチ単位の端数を含む。

3,333 Hz の FIFO ワード数は受信集計より 32 ワード多く、6,667 Hz の10秒・60秒の
測定ではそれぞれ 46 ワード多い。測定期限付近の非同期転送と統計取得の時点が一致
しないため、この差だけから欠落とは判断できない。overrun の観測が 0 であっても、
無欠落が証明されたわけではない。

終了時の残データ回収と件数照合を追加した後の実機結果:

| 設定 ODR（各） | I²C | 停止要求 / 終了 | 受信サンプル（各） | 受信 / FIFO ワード | バッチ / FIFO 転送 | 転送時間 合計 / 最大 | CPU 使用率 | 照合 |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 104 Hz | 100 kHz | 2.000 / 2.032 秒 | 210 | 420 / 420 | 14 / 14 | 0.255 秒 / 19.457 ms | 1.22% | match |
| 6,667 Hz | 1 MHz | 10.001 / 10.013 秒 | 67,065 | 134,130 / 134,130 | 2,021 / 2,021 | 8.272 秒 / 5.401 ms | 15.44% | match |
| 6,667 Hz | 1 MHz | 60.000 / 60.010 秒 | 加速度 402,348 / ジャイロ 402,350 | 804,698 / 804,698 | 12,106 / 12,106 | 49.625 秒 / 5.399 ms | 15.34% | match |

いずれも未知タグ、I²C エラー、バッファ不足、overrun 観測、RTIO 完了通知欠落、
バッファなし完了、不正パケット、ストリーム・停止エラー、終了処理 timeout は 0。
`accounting=match` はホスト側の読み出しと集計が一致したことを示し、センサーが
生成した全サンプルの無欠落を証明するものではない。
104 Hz では途中停止（停止要求 1.747 秒後）と、その直後の再開も実機で確認した。
途中停止では 364 / 364 ワード、再開後は 420 / 420 ワードで、いずれも `match`、
エラーと終了処理 timeout は 0 だった。両センサーの停止は順番に行うため、最終的な
サンプル数が異なる場合がある。

FIFO バーストは [ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf)
の FIFO 出力レジスターの循環読み出しに基づく。標準 STMEMSC の I²C 読み出しは
長さが `uint8_t` なので、FIFO 本体は Zephyr I²C API で直接読み出す。ドライバー
内部の開始・停止・読み出し関数は通常の fetch/get および属性変更と排他する。
FIFO 開始時は設定済み ODR を batch rate に使用し、エラー後は停止成功まで通常
取得に戻さない。既存の trigger と sensor hub はまだこの排他に参加しないため、
それらが有効な構成では FIFO 開始を拒否する。INT1 の watermark を GPIO 経由で
workqueue に渡し、FIFO を読み出す。
内部統計では読み出したワード数、転送時間、I²C エラー、バッファ不足、
overrun フラグの観測回数を記録する。観測回数は失われたサンプル数でも、異なる
overrun の発生回数でもない。

現状のストリームは watermark 32 ワード、`fifo_wm incl` のみをサポートする。
使用するセンサーの ODR をあらかじめ非ゼロに設定する。FIFO overrun は未知の欠落が
生じた不連続として `-EOVERFLOW` を返し停止する。I²C エラーや受信バッファ不足時も
停止する。単発取得と属性変更はストリーム中 `-EBUSY` を返す。

## `imu_measure` の使い方

`imu_measure` は Sensor Streaming API から受信した FIFO バッチを、サンプルごとに
表示せず集計する。測定の前に各センサーの ODR を設定する。例えば両センサーを
208 Hz に設定し、10秒間測定するには次を実行する。

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

指定時間の終了後、自動停止して結果を表示する。秒数は 1〜3600 の範囲で指定する。
途中で終了するには `imu_measure stop`、最後の結果を再表示するには
`imu_measure result` を実行する。片方のセンサーのみを測定する場合は、もう片方の
`sampling_frequency` を `0` に設定する。`sensor stream` と同時には実行できない。
停止時は FIFO へのバッチ追加を止め、残ったワードと処理中の完了通知を受信してから
統計を確定する。両センサーのバッチ停止は順番に行うため、停止境界は厳密に同時ではない。

最大 ODR で両センサーを測定する例:

```text
i2c speed i2c0 3
sensor attr_set lsm6dsox accel_xyz sampling_frequency 6667
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 6667
imu_measure start 60
```

測定結果の `cleanup_timeout=0` を確認してから、`i2c speed i2c0 1` で 100 kHz に
戻せる。1 MHz では同じバス上の `0x60` がスキャンで検出されなかったため、
そのデバイスとの共存は未確認。

| 出力項目 | 意味 |
|---|---|
| `elapsed_ms` / `stop_requested_ms` | 残データ回収・終了処理を含む経過時間 / 停止要求までの時間 |
| `stopped_early` | 途中停止を要求した場合は 1 |
| `batches` | 測定区間に集計した FIFO バッチ数 |
| `accel_samples` / `gyro_samples` | 測定区間に受信した加速度・ジャイロのサンプル数 |
| `other_words` | 対応していないタグの FIFO ワード数 |
| `observed_hz` | 受信サンプル数を測定時間で割った概算レート |
| `configured_hz` | ストリーム開始時に設定されていた ODR |
| `fifo_words` / `transfers` | ドライバーが読み出した FIFO ワード数と転送回数 |
| `transfer_ns` / `max_transfer_ns` | I²C バースト転送時間の合計と最大値 |
| `i2c_errors` / `buffer_too_small` / `overrun_observations` | I²C エラー、バッファ不足、overrun フラグの観測回数 |
| `cpu_busy` | 測定区間におけるシステム全体の CPU 使用率 |
| `accounting` | エラーなく受信・FIFO のワード数とバッチ数が一致すれば `match`、不一致なら `mismatch`、エラーなどで照合できなければ `incomplete` |
| `stream_error` / `stop_error` | ストリーム処理と停止要求・待機のエラー |
| `driver_stop_error` | 割り込み解除や FIFO 停止時にドライバーが検出した最初のエラー |
| `dropped_completions` / `bufferless_completions` / `invalid_packets` | 完了通知の欠落、受信バッファなしの成功通知、不正な受信パケット |
| `terminal_seen` / `cleanup_timeout` | 終了通知の受信有無 / 終了処理 timeout |

CPU 使用率には測定処理や他のスレッドの負荷も含まれる。観測レートは開始時の設定と
停止・残データ回収の時間を含む概算。overrun の観測回数は、失われたサンプル数や
独立した overrun の発生件数を示すものではない。`accounting=match` はホストが読み出し
集計した分の一致を示すが、センサーが生成した全サンプルの無欠落を証明しない。
`cleanup_timeout=1` の場合は終了処理が完了しておらず、次の測定には再起動が必要。

### shell で値を表示する場合

次のコマンドはサンプルを逐次表示するため、低 ODR で使う。

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 26
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 26
sensor stream lsm6dsox@6a on fifo_wm incl
sensor stream lsm6dsox@6a off
imu_fifo_stats
```

Zephyr 4.5.0 の `sensor stream` はデバイスの完全な名前を要求するため、
`sensor get` や `sensor attr_set` で使えるノードラベル `lsm6dsox` ではなく
`lsm6dsox@6a` を指定する。

FIFO のワード解析はホスト上で次のようにテストできる。

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/drivers/lsm6dso \
  app/usb-console/tests/fifo_words.c app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c \
  -o /tmp/rp2040-fifo-words-test
/tmp/rp2040-fifo-words-test
```

FIFO ハードウェア処理のエラー処理と転送長は、次のホストテストで偽の I²C/ST API を
使って確認できる。実際のバスや IMU の動作は確認しない。

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/fifo_hw.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo_hw.c -o /tmp/rp2040-fifo-hw-test
/tmp/rp2040-fifo-hw-test
```

FIFO decoder の単位変換・フレーム数・分割読み出しは以下でテストできる。
このテストも偽の Sensor API 定義を使い、実機にはアクセスしない。

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/fifo_decoder.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo_decoder.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-fifo-decoder-test
/tmp/rp2040-fifo-decoder-test
```

測定用パケット集計のテスト（混在タグ、未知タグ、不正な長さ・ヘッダー）:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso -Iapp/usb-console/src \
  app/usb-console/tests/imu_measure_packet.c app/usb-console/src/imu_measure_packet.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-imu-measure-test
/tmp/rp2040-imu-measure-test
```

停止処理のホストテスト（転送中の停止要求、残バッチの受信、停止時の I²C・
バッファ不足・overrun エラー）は、偽の FIFO と RTIO を使って実行できる。

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/stream_stop.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-stream-stop-test
/tmp/rp2040-stream-stop-test
```

## DMA の比較

実験的に実装した RP2040 DMA と割り込み方式を同一ファームウェアで比較した。I²C は 1 MHz、両センサーは各 6667 Hz、測定は各 10 秒。標準 I²C ドライバー版も別途 10 秒測定した。いずれも FIFO の集計は一致し、エラーと overrun 観測はなかった。今回の DMA 実装では CPU 使用率が増加したため、カスタムドライバーを削除し、Zephyr 標準の `i2c_dw` ドライバーに戻した。

| 方式 | CPU 使用率 | FIFO ワード数 | バッチ数 | ワード集計の経過時間 |
| --- | ---: | ---: | ---: | ---: |
| 割り込み | 13.05% | 134,186 | 1,494 | 129.366 ms |
| DMA | 34.87% | 134,202 | 3,059 | 144.568 ms |
| Zephyr 標準の割り込み方式 | 15.62% | 134,138 | 2,089 | 143.352 ms |

集計の経過時間には割り込みやスケジューリングによる中断を含み、CPU 実行時間そのものではない。比較対象は今回の実装であり、すべての DMA 設計の性能を示すものではない。この時間は `imu_measure` の `decode_elapsed_ns` に表示される。
