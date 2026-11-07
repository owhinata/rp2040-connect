# ローカルドライバーの基準版

[English](README.md) | 日本語

`lsm6dso/` と `i2c_dw/` は Zephyr v4.5.0、コミット
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
ものを使用する。I²C はローカルの `i2c_dw` を使用する。RP2040 では RX・TX FIFO の
深さを 16、RX しきい値を 4 バイトとし、短い読み出しと末尾はしきい値を下げる。
受信待ちで要求を追加できない間は TX_EMPTY 割り込みを抑制する。他のコントローラーは
Zephyr の処理を維持する。標準版とローカル版は同時にコンパイルしない。Zephyr のライブラリー構成が変わった場合、
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

ローカル版は Nano RP2040 Connect でビルドと実機確認を行った。100 kHz と 1 MHz で 1〜65 バイトの読み出し、STOP なしの分割読み出し、方向変更を伴う RESTART が成功した。1 MHz で存在しないアドレスへの読み出しが NACK となった後、WHO_AM_I を正常に読めた。208 Hz の FIFO 取得中に別の I²C レジスター読み出しを 70 回行い、集計一致とエラーなしを確認した。I²C scan では IMU と ATECC608A を検出したが、別デバイスとの同時通信や物理的なバス固着からの復旧は未検証である。2 ms のタイムアウトからの復帰は後述する。

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
| `cpu_active` | アイドルフックから求めた非アイドル時間。割り込みを含み、表示分解能は 0.1% |
| `cpu_busy` | スケジューラ上の非アイドル時間。アイドル中の割り込みが含まれないことがある |
| `accounting` | エラーなく受信・FIFO のワード数とバッチ数が一致すれば `match`、不一致なら `mismatch`、エラーなどで照合できなければ `incomplete` |
| `stream_error` / `stop_error` | ストリーム処理と停止要求・待機のエラー |
| `driver_stop_error` | 割り込み解除や FIFO 停止時にドライバーが検出した最初のエラー |
| `dropped_completions` / `bufferless_completions` / `invalid_packets` | 完了通知の欠落、受信バッファなしの成功通知、不正な受信パケット |
| `terminal_seen` / `cleanup_timeout` | 終了通知の受信有無 / 終了処理 timeout |

CPU 使用率はスケジューラ上の非アイドル時間で、測定処理や他のスレッドの負荷も含む。アイドル中の割り込みは含まれないことがある。観測レートは開始時の設定と
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
imu_measure stats
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

## DMA と watermark の測定

今回の IMU FIFO 用途に対する DMA 対応は完了とし、通常ビルドで標準有効にしている。60 秒の連続取得を確認期間として十分と判断した。DMA は I²C0 の 7 ビットアドレス、STOP で終わる最終読み出しで、長さが 32〜3584 バイトの場合に使用する。書き込み、短い読み出し、その他の条件に合わない転送は割り込みで処理する。DMA チャネルの確保・設定に失敗した場合は転送開始前に割り込み方式へ切り替える。他の DMA 利用者との競合は、その利用が必要になった時点で検証する。

以下は watermark 32 ワードで同一の実験用ファームウェアを使い、両方式を比較した結果である。

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-dma-comparison -- -DEXTRA_CONF_FILE=prj_i2c_dma_comparison.conf -DEXTRA_DTC_OVERLAY_FILE=dma_comparison.overlay
west flash -d build/usb-console-dma-comparison -r uf2
```

I²C を 1 MHz、両センサーのサンプリング周波数を 6667 Hz に設定し、`i2c_dma mode off` または `i2c_dma mode on` を指定してから `imu_measure start 60` を実行する。モードはストリームと転送が停止中のときだけ切り替える。`i2c_dma stats` のカウンターは累積値なので、測定前後の差で DMA 転送数、エラー、フォールバックを確認する。

| 方式 | `cpu_active` | `cpu_busy` | FIFO ワード数 | バッチ数 | 転送時間 | DMA 転送数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 割り込み | 66.8% | 31.22% | 805,410 | 18,191 | 42.177 秒 | 0 |
| DMA | 45.1% | 37.28% | 805,118 | 17,499 | 40.641 秒 | 17,499 |

同一の比較版で各 60 秒測定した。両方式ともホスト側 FIFO 集計は一致し、I²C エラー、overrun、完了通知の欠落は観測されなかった。DMA 測定中の DMA エラーとフォールバックの増加もなかった。10 秒測定を各方式で交互に 2 回行った結果、`cpu_active` は DMA なし 66.8〜66.9%、DMA あり 45.3% だった。60 秒測定では DMA により実測の非アイドル時間が 21.7 ポイント減った。`cpu_busy` が逆に増えているのは、アイドル中の割り込みを除外し得る別のスケジューラ指標であるため。計測処理と比較版自体の負荷も値に含まれる。ホスト側の集計一致と overrun が観測されなかったことだけでは、センサー生成サンプルがすべて保持されたとは断定できない。

100 kHz と 1 MHz で短い読み出し、リピーテッドスタート、NACK 後の復帰を確認した。100 kHz、タイムアウト 2 ms の検証版では DMA のタイムアウト後に短い読み出しへ復帰する試験を 3 回連続で確認した。他の DMA 利用者との競合と、SDA/SCL が物理的に固定された状態からの復帰は未検証。


### watermark の比較

通常ビルドでは `fifo_watermark` 属性で 1〜256 ワードに設定できる。ストリーム停止中に変更し、初期値は 32 ワード。同一の比較ファームウェアで両センサーを 6667 Hz、I²C を 1 MHz とし、各条件を 10 秒ずつ測定した。

| watermark（ワード） | DMA | `cpu_active` | `cpu_busy` | FIFO ワード数 | バッチ数 |
| ---: | --- | ---: | ---: | ---: | ---: |
| 32 | 無効 | 67.0% | 34.12% | 134,194 | 3,062 |
| 32 | 有効 | 45.9% | 38.93% | 134,194 | 2,914 |
| 64 | 無効 | 60.6% | 27.32% | 134,218 | 1,939 |
| 64 | 有効 | 39.5% | 31.42% | 134,228 | 1,922 |
| 128 | 無効 | 45.7% | 15.50% | 134,118 | 1,007 |
| 128 | 有効 | 22.5% | 18.25% | 134,186 | 1,003 |

全条件でホスト側の集計は一致し、I²C エラー、バッファ不足、overrun 観測、完了通知の欠落はなかった。DMA エラーとフォールバックも増加しなかった。各条件 1 回ずつの 10 秒測定であり、計測処理の負荷を含む。watermark を上げると最初のバッチを待つ時間が長くなり、FIFO の残り容量も減る。overrun が観測されなかったことだけでは、生成されたサンプルがすべて保持されたとは断定できない。

128 ワードで追加した各 60 秒測定の結果を示す。先頭 2 行は同一の比較版、3 行目は DMA 標準有効の通常版である。

| ファームウェア | DMA | `cpu_active` | FIFO ワード数 | バッチ数 / DMA 転送数 |
| --- | --- | ---: | ---: | ---: |
| 比較版 | 無効 | 45.6% | 804,974 | 6,040 / 0 |
| 比較版 | 有効 | 22.3% | 804,918 | 6,014 / 6,014 |
| 通常版 | 有効 | 22.9% | 804,830 | 6,006 / 6,006 |

3 回ともホスト側の FIFO 集計は一致し、I²C エラーと overrun は観測されなかった。DMA 有効時の DMA エラーとフォールバックの増加もなかった。通常版の平均バッチ間隔は約 10.0 ms（60 秒 / 6,006）で、サンプリング間隔や最大遅延を示すものではない。

## CPU 負荷の計測

`imu_measure` は開始時に Zephyr のアイドルフックによる CPU 負荷の集計区間をリセットし、ストリームの停止・残データ回収後に結果を取得する。`cpu_active` は割り込み、アプリ・カーネル処理、アイドルフック自体の負荷を含み、0.1% 単位で切り捨てて表示する。開始・終了処理の時間も含む。アイドルフックの集計区間は共有されるため、測定中に別のコードが `cpu_load_get(true)` を呼ぶと結果が変わる。`cpu_busy` は別のスケジューラ統計で、アイドル中の割り込みが含まれない場合がある。

`-DEXTRA_CONF_FILE=prj_cpu_load_validation.conf` を指定した検証版では `cpu_load_validate` を使える。IMU 測定を停止してから実行する。このコマンドはアイドル区間、スレッドによる連続処理、および 2 ms ごとに 500 µs 動作するタイマー割り込みを順に測り、短時間 CPU に負荷をかけて共有集計区間をリセットする。Nano RP2040 Connect での実測はそれぞれ 0.1%、100.0%、25.7% で、タイマーは 1,000 回動作した。

ローカル I²C ドライバー、I²C 1 MHz、両センサー各 6667 Hz の 60 秒測定では `cpu_active=60.0%`、スケジューラ上の `cpu_busy=26.64%` だった。FIFO 804,954 ワードが集計一致し、I²C エラー、overrun 観測、完了通知欠落はなかった。FIFO 転送時間は合計 39.197 秒で、ほぼ同じ条件のアイドルフックなしの以前の測定は 38.909 秒だった。ただし各 1 回のため、計測オーバーヘッドを精密に求めたものではない。この以前の測定では DMA を使用しておらず、DMA との比較ではない。

以前の 6667 Hz・60 秒測定の 15.05% は、異なる I²C ドライバーと以前の FIFO 停止処理によるスケジューラ上の非アイドル時間である。今回の 26.64% も同じスケジューラ統計だが、別の実装・別の測定である。バッチ数はそれぞれ 16,393 回と 23,672 回で、バッチ処理の増加が差の一因となった可能性はある。ただし、いずれもアイドル中の割り込みを除外し得るため、この二つだけでは実際の CPU 総負荷の増減は分からない。`cpu_active` の 60.0% と以前の 15.05% も、異なる指標なので直接比較できない。



## 以前の DMA 比較

実験的に実装した RP2040 DMA と割り込み方式を同一ファームウェアで比較した。I²C は 1 MHz、両センサーは各 6667 Hz、測定は各 10 秒。標準 I²C ドライバー版も別途 10 秒測定した。いずれも FIFO の集計は一致し、エラーと overrun 観測はなかった。DMA 比較後はいったん Zephyr 標準の `i2c_dw` ドライバーに戻し、その後の FIFO 実験を経て現在のローカル版を採用した。

| 方式 | CPU 使用率 | FIFO ワード数 | バッチ数 | ワード集計の経過時間 |
| --- | ---: | ---: | ---: | ---: |
| 割り込み | 13.05% | 134,186 | 1,494 | 129.366 ms |
| DMA | 34.87% | 134,202 | 3,059 | 144.568 ms |
| Zephyr 標準の割り込み方式 | 15.62% | 134,138 | 2,089 | 143.352 ms |

上記の CPU 使用率はスケジューラ上の非アイドル時間であり、アイドル中の割り込みが方式によって異なる割合で除外され得るため、CPU の実際の総負荷を比較したものではない。集計の経過時間には割り込みやスケジューリングによる中断を含み、CPU 実行時間そのものではない。比較対象は今回の実装であり、すべての DMA 設計の性能を示すものではない。この時間は `imu_measure` の `decode_elapsed_ns` に表示される。


## 以前の実機検証結果

### shell での連続取得

| センサー | 設定 ODR | FIFO ワード | 転送回数 | 転送時間合計 | I²C エラー / バッファ不足 / overrun 観測 |
|---|---:|---:|---:|---:|---:|
| 加速度・ジャイロ | 各 26 Hz | 608 | 19 | 約 0.369 秒 | 0 / 0 / 0 |
| 加速度・ジャイロ | 各 104 Hz | 2,464 | 77 | 約 1.495 秒 | 0 / 0 / 0 |
| 加速度のみ | 104 Hz | 960 | 30 | 約 0.582 秒 | 0 / 0 / 0 |
| ジャイロのみ | 104 Hz | 1,024 | 32 | 約 0.621 秒 | 0 / 0 / 0 |
| 加速度・ジャイロ | 各 208 Hz | 3,904 | 122 | 約 2.366 秒 | 0 / 0 / 0 |

208 Hz の停止後の `kernel ps` では sensor shell のスタック最大使用量は 1,208 / 3,072
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
ストリームエラー、終了処理 timeout はいずれも 0。CPU 使用率は測定区間のスケジューラ上の非アイドル時間で、集計や他スレッドの負荷を含む。
アイドル中の割り込みは含まれないことがある。観測レートは測定区間中の受信数からの
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

ストリームは `fifo_wm incl` をサポートする。`sensor attr_set lsm6dsox all fifo_watermark <1..256>` で停止中に変更し、`sensor attr_get lsm6dsox all fifo_watermark` で確認できる。初期値は 32 ワードでリセット時に戻る。アプリからは `SENSOR_CHAN_ALL` と `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` を指定して `sensor_attr_set` / `sensor_attr_get` を使用できる。
使用するセンサーの ODR をあらかじめ非ゼロに設定する。FIFO overrun は未知の欠落が
生じた不連続として `-EOVERFLOW` を返し停止する。I²C エラーや受信バッファ不足時も
停止する。単発取得と属性変更はストリーム中 `-EBUSY` を返す。


## CPU 負荷の分布計測

スレッド別のスケジューラ統計を `imu_measure` に追加する計測版は次のようにビルドする。割り込み自体のトレースは行わない。

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-profile -- -DEXTRA_CONF_FILE=prj_cpu_profile.conf
west flash -d build/usb-console-profile -r uf2
```

センサー設定後、`imu_measure start 10` を実行すると、計測期間に各スレッドへ割り当てられた時間の割合を表示する。`imu_fifo_worker` は FIFO を読み出すワーカースレッドである。Zephyr の統計では、割り込み時間はアイドルを含む中断されたスレッドに計上される。このためスレッド別の数値には割り込みが含まれ、`cpu_busy` にはアイドル中の割り込みが含まれないことがある。各数値には計測処理のオーバーヘッドも含まれる。分布は測定終了時のみ表示し、`imu_measure result` には保存しない。

割り込みの概算には `-DEXTRA_CONF_FILE=prj_cpu_profile_irq.conf` を指定して別途ビルドする。例外ごとに 61 回に 1 回を標本抽出して全体を推定するため、周期的な偏りやネストした割り込みの影響を受け、スレッド別の時間とも重複する。フックはすべての対象割り込みに負荷を加える。このボードでは 6667 Hz で FIFO overrun が発生したため、その結果は正常な連続取得時の値とは扱えない。結果の解釈前に `accounting`、stream error、overrun 観測を確認する。`profile_overflow` が 0 以外なら集計できなかったデータがある。

Nano RP2040 Connect、I²C 1 MHz、両センサー各 6667 Hz のスレッドのみの 60 秒測定では、FIFO / 受信は 804,728 ワードで集計が一致し、エラーと overrun はなかった。スケジューラ上の時間は `imu_fifo_worker` が 10.59%、`imu_measure_tid` が 4.43%、アイドルが 84.96%。`decode_elapsed_ns` は中断を含む 755,065,688 ns だった。この周波数では割り込み時間を個別に測っておらず、アイドルに計上されることがある。

別の割り込み標本抽出版で両センサー各 3333 Hz を 10 秒測定した際も、集計は一致し、エラーと overrun はなかった。スケジューラ上の時間は FIFO ワーカー 11.49%、測定スレッド 6.06%、I²C 割り込みは標本から約 28.34%、割り込み回数は 499,126 回だった。これらは重複し、計測方法にも影響されるため合算できない。この計測時の FIFO 転送時間合計は 5.182 秒で、以前のトレースなしの 3333 Hz 測定の 4.186 秒より長く、計測が動作に影響したことを示している。6667 Hz の割り込み計測は FIFO overrun で停止したため、正常な連続取得時の値として使わない。

3333 Hz の割り込み計測では FIFO 67,078 ワード、データ 469,546 バイトに対し、全 I²C トランザクションの割り込みが 499,126 回で、FIFO データ 1 バイトあたり約 1.06 回だった。Zephyr の DesignWare ドライバーは RX FIFO のしきい値を 0 に設定しており、受信 1 バイトから RX_FULL 割り込みが発生する。レジスターアドレス書き込み、FIFO 状態読み出し、STOP 処理もあるため、この回数はおおむね 1 バイトごとの割り込みで説明でき、TX_EMPTY の繰り返しだけを示すものではない。割り込み要因の標本は割り込み計測版の `i2c_status_samples` に表示する。標本は正確な全件数ではなく、計測によりタイミングも変わる。

その後、割り込み要因を標本抽出した 3333 Hz・10 秒の測定は、FIFO 67,073 ワードで集計が一致し、I²C エラーと FIFO overrun はなかった。I²C 割り込み 499,091 回から取得した 8,182 件のマスク済み割り込み状態のうち、RX_FULL と TX_EMPTY の両方が 7,904 件、TX_EMPTY のみが 278 件、RX_FULL のみとどちらもなしは 0 件だった。TX_EMPTY のみの 278 件では RX FIFO は空だった。受信 1 バイトごとの割り込みと整合するが、同時に TX_EMPTY も多くの標本で立っている。状態の一時点の読み出しだけでは、どの条件が先に割り込みを発生させたか、TX_EMPTY のみの場合に処理が進んだかは確定できない。標本はランダムではなく周期的に取得しており、計測の負荷によって転送タイミングも変わる。


## I²C RX FIFO の実験

これは、標準相当の深さの扱いと実際の FIFO 深さ・RX しきい値を比較した過去の実験結果である。RP2040 の `IC_COMP_PARAM_1` は 0 を返すが、実際の RX・TX FIFO は各 16 エントリーである。Zephyr の汎用ドライバーは読み取った値から深さを 1 と扱う。実験後、しきい値 4 バイトをローカルドライバーに採用した。実験時の切り替えコマンドは現在のビルドには含まれない。

I²C 1 MHz・両センサー各 6667 Hz を各 10 秒測定した結果を示す。いずれも FIFO の集計が一致し、I²C エラーと overrun 観測はなかった。モード 0 も計測用カウンターなどを含む実験版であり、計測コードなしの Zephyr 標準ドライバーそのものではない。

| 方式 | I²C 割り込み | I²C ハンドラーの推定時間 | FIFO 転送時間合計 | FIFO ワード |
| --- | ---: | ---: | ---: | ---: |
| 標準相当の深さ | 947,909 | 4.006 秒（39.96%） | 9.218 秒 | 134,160 |
| 深さ 16・しきい値 1 | 890,875 | 3.913 秒（39.10%） | 6.435 秒 | 134,182 |
| 深さ 16・しきい値 4 | 271,894 | 2.855 秒（28.53%） | 6.574 秒 | 134,172 |
| 深さ 16・しきい値 8 | 238,357 | 2.686 秒（26.84%） | 6.789 秒 | 134,154 |

ハンドラー時間は 61 回に 1 回の割り込みを標本抽出し、全割り込み数から推定した値。ネストした割り込みを含むことがあり、割り込み入口・出口、ラッパー、カウンターの負荷は含まない。周期的な標本抽出による偏りもあり得る。スレッド統計と重複するほか、高速な設定では FIFO バッチが小さくなり、アプリとドライバーの処理回数が増えるため、CPU の総使用率をこの値から確定することはできない。深さ 16・しきい値 8 はハンドラー標本抽出を有効にして別途 60 秒測定し、FIFO 805,030 ワードが集計一致、I²C 割り込みは 1,429,389 回、エラーと overrun 観測はなかった。ハンドラー時間の推定は 16.118 秒（26.86%）だった。存在しないアドレスへの 8 バイト読み出しでは NACK を確認し、その後 WHO_AM_I の読み出しと 208 Hz の FIFO 取得が成功した。208 Hz のセンサー動作中に、各非標準モードで 1〜5、7〜9、15、16 バイトのレジスター読み出しも確認した。エラー復旧や他の I²C デバイスとの共有については十分に検証していない。

### ローカルドライバーの連続取得確認

I²C 1 MHz・両センサー各 6667 Hz の 60 秒測定は、FIFO / 受信が 804,916 ワード、バッチ / 転送が 23,274 回で集計が一致し、I²C エラー、overrun 観測、完了通知欠落はなかった。FIFO 転送時間は合計 38.909 秒、最大 3.739 ms。停止後の `sensor get` も成功した。集計一致はセンサーが生成した全サンプルの受信を保証しない。


## I²C の実機検証

追加の検証コマンドは通常版と分けてビルドする。

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-i2c-validation -- -DEXTRA_CONF_FILE=prj_i2c_validation.conf
west flash -d build/usb-console-i2c-validation -r uf2
```

IMU のストリームを停止した状態で `i2c_validate` を実行すると、1〜65 バイトのレジスター読み出しを固定レジスターの先頭部分と照合し、バッファの境界、STOP なしの分割読み出し、複数回の方向変更と RESTART を確認する。ボード上の LSM6DSOX にアクセスする検証であり、任意の I²C デバイス向けではない。NACK、同時利用、FIFO ストリーミングは別途確認する。

タイムアウトからの復帰には `-DEXTRA_CONF_FILE=prj_i2c_timeout.conf` でビルドし、`i2c speed i2c0 1` を設定して `i2c_validate_timeout` を実行する。2 ms の制限で 64 バイト読み出しがタイムアウトした後、短い WHO_AM_I 読み出しが成功することを確認する。実機ではこの一連の処理を 3 回連続で確認した。この短い制限は通常ファームウェアには使用しない。
