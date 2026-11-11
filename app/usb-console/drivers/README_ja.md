# ローカルドライバー

[English](README.md) | 日本語

`app/usb-console` は、Zephyr のドライバーの一部をこのディレクトリのコピーに置き換えて
ビルドします。IMU の FIFO を高いレートで読み出すための変更と、NINA-W102 用の Wi-Fi
ドライバーを含みます。

| ディレクトリ | 内容 | 元 |
|---|---|---|
| `lsm6dso/` | LSM6DSO センサードライバーに FIFO ストリームを追加 | Zephyr v4.5.0 のコピー |
| `i2c_dw/` | DesignWare I²C ドライバーに RP2040 の DMA 読み出しを追加 | Zephyr v4.5.0 のコピー |
| `sensor_shell/` | sensor シェルの修正と独自属性の追加 | Zephyr v4.5.0 のコピー |
| `nina/` | NINA-W102（SPI）の Wi-Fi・ソケットドライバー | このリポジトリで作成 |

コピー元は Zephyr v4.5.0（コミット `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`）で、各ソースの
著作権と Apache-2.0 の表示を維持しています。アプリの `CMakeLists.txt` が、Zephyr の
ドライバーライブラリー内の対象ソースをコピーに差し替えます。Kconfig、devicetree binding、
ST の HAL は Zephyr のものを使います。Zephyr 側の構成が変わって差し替え対象が見つからない
場合は、ビルドをエラーにします。

NINA ドライバーは [NINA ドライバーの説明](../../../design/nina-driver-ja.md) にまとめています。

## LSM6DSO の FIFO ストリーム

Zephyr の Sensor Streaming API（`sensor_stream()`、RTIO）で FIFO を読み出せるようにしています。
トリガーは `SENSOR_TRIG_FIFO_WATERMARK`、オプションは `SENSOR_STREAM_DATA_INCLUDE` です。

| ファイル | 役割 |
|---|---|
| `lsm6dso_fifo.c` | FIFO ワードの解析（加速度・角速度・タイムスタンプ・その他、タグカウンター） |
| `lsm6dso_fifo_hw.c` | FIFO の設定、状態の読み取り、I²C のバースト読み出し |
| `lsm6dso_fifo_irq.c` | INT1（FIFO しきい値）の割り込み |
| `lsm6dso_stream.c` | ストリームの開始・読み出し・停止（専用のワークキュー、優先度 10） |
| `lsm6dso_fifo_decoder.c` | FIFO バッファ用の Zephyr decoder（`sensor stream` の表示に使用） |
| `lsm6dso_compat.c` | 通常の単発取得の decoder との切り替え |

### 動作

- 開始時に FIFO をバイパスにしてから、加速度・角速度のバッチレート、タイムスタンプの間隔、
  watermark を設定してストリームモードにします。
- INT1 でしきい値到達を受けると、FIFO の状態を読み、たまっているワードを 1 回のバーストで
  読み出して、1 つの RTIO バッファとして渡します。バッファの先頭には
  `struct lsm6dso_fifo_packet`（ワード数、周期、感度など）が付きます。
- **FIFO オーバーラン**、I²C エラー、バッファ不足を検出すると、FIFO を止めてストリームを
  エラーで終えます。失われた個数は分からないので、連続しているように見えるデータは返しません。
  再開するかどうかは呼び出し側（`src/imu_stream.c`）が決めます。
- **停止**は `lsm6dso_fifo_request_stop()` で要求します。バッチを止め、FIFO に残ったワードを
  渡し切ってから終端の完了通知を出します。RTIO の要求を直接キャンセルすると、読み出し済みの
  バッファが消費前に解放され得るためです。
- FIFO を使っている間、通常の単発取得と属性の変更は `-EBUSY` になります。

### 時刻

FIFO にはセンサーのタイムスタンプ（公称 25 µs 単位）を入れます。間隔は属性で変えられます。
タイムスタンプの有効化は開始時に行い、止めません（ストリームを再開してもカウンターは
進み続けます）。サンプルへの対応付けと PC 時刻への変換はアプリ側で行います
（[設計書](../../../design/imu-raw-mqtt-design-ja.md) §7）。

`lsm6dso_timestamp_read()` は、センサーのタイムスタンプレジスタを、呼び出し側の時計の読み取りで
挟んで読みます。センサーの時計とボードの時計を対応付けるために使います。

decoder が返すサンプル時刻は、ホストの読み出し完了時刻と公称 ODR からの推定で、I²C の転送と
割り込み処理の遅れを含みます。正確な時刻ではありません。

### 属性

`SENSOR_CHAN_ALL` に対する独自属性です（`lsm6dso_attributes.h`）。どちらも次のストリーム開始時に
適用され、ストリーム中は変更できません。

| 属性 | シェルでの名前 | 値 | 初期値 |
|---|---|---|---|
| `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` | `fifo_watermark` | 1〜256 ワード | 32 |
| `SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION` | `fifo_timestamp_decimation` | 0、1、8、32 | 8 |

watermark を大きくすると読み出しの回数が減って負荷が下がりますが、データが届くまでの時間が
延び、FIFO（512 ワード）の余裕が減ります。

### 統計

`lsm6dso_fifo_get_stats()` で、読み出したワード数、転送の回数と時間、I²C エラー、バッファ不足、
オーバーランの観測回数、設定 ODR、感度、周波数補正値を取得できます。統計は FIFO を開始する
たびにクリアされ、`generation` が 1 進みます。

### 制限

- 対応しているのは I²C 接続だけです。
- 内部温度の取得には対応していません（FIFO にも入れていません）。
- トリガーとセンサーハブ（`CONFIG_LSM6DSO_TRIGGER`、`CONFIG_LSM6DSO_SENSORHUB`）とは併用できません。

## I²C（`i2c_dw`）

RP2040 向けに次を変えています。他のコントローラーの処理は Zephyr のままです。

- RX・TX FIFO の深さを 16、RX しきい値を 4 バイトとし、短い読み出しと末尾ではしきい値を
  下げます。受信待ちで要求を追加できない間は TX_EMPTY 割り込みを抑えます。
- **DMA による読み出し**（`CONFIG_I2C_DW_RP2040_DMA`、既定で有効）。I²C0 の 7 ビットアドレスで、
  STOP で終わる最後の読み出しが 32〜3584 バイトの場合に使います。書き込み、短い読み出し、
  条件に合わない転送は割り込みで処理します。DMA チャネルの確保や設定に失敗した場合は、転送を
  始める前に割り込み方式へ切り替えます。

```text
i2c_dma mode          現在の方式を表示
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（累積）
```

FIFO の取得だけを行った過去の測定（6667 Hz、1 MHz、各 1 回）では、watermark 128 のとき
`cpu_active` が DMA なしで 45.7%、DMA ありで 22.5% でした。

## sensor シェル（`sensor_shell`）

- フレーム数が 0 のチャンネルと、キャンセル時などのバッファのない完了通知を表示から除きます。
- `fifo_watermark` と `fifo_timestamp_decimation` を `sensor attr_set` / `attr_get` で扱えます。

`sensor stream` と `imu start` は同時に使えません。

## テスト

`./script/host_tests.sh` が、偽の Zephyr ヘッダーを使ってドライバーの次の部分を PC 上で
テストします。

| テスト | 対象 |
|---|---|
| `fifo_words` | FIFO ワードの解析 |
| `fifo_hw` | FIFO の設定手順と失敗時の巻き戻し、状態の読み取り |
| `fifo_decoder` | FIFO バッファの decoder |
| `stream_stop` | ストリームの停止、キャンセル、エラー時の終了 |
| `i2c_dma_commands` | `i2c_dma` コマンドの引数の解釈 |
| `nina_spi` | NINA の SPI フレーミングと応答の解析 |
