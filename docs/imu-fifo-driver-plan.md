# LSM6DSOX FIFO バースト取得と RP2040 I²C DMA の実装方針

## 目的と現状

Arduino Nano RP2040 Connect のオンボード IMU（LSM6DSOX）について、FIFO を使った
複数サンプルのバースト取得を実装し、その後 I²C の DMA 転送に対応する。
最大 ODR 6,667 Hz での取得を対象に、欠落・転送時間・CPU 使用率を実機で評価する。

- Zephyr: `v4.5.0`。依存関係は `manifest/west.yml` で管理する。
- アプリ: `app/usb-console`。既存の shell、ログ、Sensor API を維持する。
- IMU: I²C0、アドレス `0x6a`。ボード定義のノード名は `lsm6dsox`。
- センサー割り込み: ボード定義では GPIO24。回路図上は IMU の INT1 に接続。
- 標準ドライバー: Zephyr の `drivers/sensor/st/lsm6dso`。ボードの互換文字列も
  `st,lsm6dso`。実機は LSM6DSOX なので、差異に注意する。
- I²C ドライバー: Zephyr の `drivers/i2c/i2c_dw.c`。
- 両ドライバーの取得元は `zephyr/`。このディレクトリは Git 管理対象外なので、
  カスタム版は本リポジトリの管理対象に置く。
- 現在の `lsm6dso` は Sensor Fetch/Get API を提供し、FIFO をバイパスモードにする。
- 現在の `CONFIG_I2C_DW_LPSS_DMA` は Intel LPSS 向け。RP2040 の DMA にそのまま
  流用しない。RP2040 には I²C0 用の TX/RX DMA 要求がある。

以前、実機では `i2c speed i2c0 3` により 1 MHz を設定し、IMU アドレス `0x6a` の
検出と `sensor get lsm6dsox` の取得を確認した。一方、100 kHz では見えていた暗号
チップの `0x60` は 1 MHz のスキャンでは見えなかった。原因は未特定であり、他の
バス利用者も含む 1 MHz での安定動作は保証できていない。

## 必須条件

**ステップ 2 の時点で、カスタム LSM6DSOX ドライバーは Zephyr の Sensor Driver
I/F を満たす。** アプリ独自のレジスター直接操作だけで FIFO を実装しない。
既存の `sensor get` や `sensor attr_set` を維持しつつ、FIFO 用には Zephyr の
Read and Decode / Streaming API の実装を検討する。必要な `submit`、decoder、
watermark trigger の対応範囲を、実装前に Zephyr 4.5.0 の API と照合する。

同じ IMU または I²C コントローラーを、標準ドライバーとコピーしたドライバーが
同時に初期化・操作しないようにする。I²C0 の shell 操作、IMU、他デバイスは同じ
I²C ドライバーを通し、設定変更・同時転送・排他を明確に扱う。

## ステップ 1: 既存ドライバーをコピーして同等動作を確認

1. Zephyr 4.5.0 の `lsm6dso` と `i2c_dw` の実装、関連ヘッダー、Kconfig、CMake、
   必要な devicetree binding の依存関係を確認し、本リポジトリにコピーする。
2. 元の著作権・ライセンス表示を維持し、コピー元のバージョン・コミットを記録する。
3. カスタム版のみが対象デバイスにバインドされる Kconfig / devicetree / ビルド構成を
   用意する。標準ドライバーを無効化する範囲と、他の I²C インスタンスへの影響に注意。
4. FIFO・DMA の機能拡張はまだ加えず、既存の動作と出力を維持する。

完了条件:

- ビルドが成功し、実機で USB shell が利用できる。
- `i2c scan i2c0`、`i2c speed`、`sensor get lsm6dsox`、
  `sensor attr_set ... sampling_frequency ...` が既存と同等に動く。
- 使用中の IMU・I²C コントローラーに登録されたドライバーがそれぞれ一つである。
- 適切な範囲で、NACK や無効引数などの既存のエラー動作も確認する。

## ステップ 2: Sensor I/F を満たす FIFO バースト取得

1. LSM6DSOX の FIFO mode、sensor ODR、FIFO batch rate、watermark、割り込み、
   FIFO タグの処理を追加する。Linux IIO の `st_lsm6dsx` の設定・FIFO 実装を参考にし、
   実装と I/F は Zephyr に合わせる。
2. FIFO を有効にした状態でも Sensor Driver I/F を満たす。通常の Fetch/Get および
   属性操作との共存・排他の仕様を明確にする。複数サンプル取得は Zephyr の
   Read and Decode / Streaming I/F と整合する形で設計する。
3. FIFO watermark を契機に複数の 7 バイトワードを一度の I²C 複合転送で取得し、
   タグを使って加速度とジャイロを区別する。割り込みハンドラーで長い I²C 転送を
   行わず、ワーカーなどに処理を委譲する。
4. FIFO overrun、ホスト側バッファ不足、I²C エラーを識別・計測する。欠落件数が
   分からない場合は件数を推測せず、不連続の発生を通知する。
5. I²C は 1 MHz を候補とする。取得中の速度・バス共有の管理と、他デバイスへの
   影響を含めて実機評価する。

この段階では I²C DMA は使わない。まず通常の I²C バースト転送で検証する。
対象は加速度のみ、ジャイロのみ、両方を検討し、それぞれ低い ODR から開始して
6,667 Hz に上げる。サンプル出力先・計測時間・許容欠落は、実装時に目的に合わせて
定める。測定データをテキストで逐次出力すると性能測定に影響するため注意する。

FIFO は非圧縮で最大 512 ワード、1 ワードはタグ 1 バイト＋データ 6 バイト。両センサーを
各 6,667 Hz にすると FIFO データだけで約 93.3 kB/s、I²C の ACK を含むデータ部分だけで
約 840 kbit/s となる。バス処理や状態確認のオーバーヘッドは別途必要なので、1 MHz で
無欠落の連続取得が可能かは実機で測定する。バッファのタイムスタンプは、取得時刻と
センサーのサンプル生成時刻を混同しない。

完了条件:

- 既存の Sensor I/F が使え、FIFO バッチ取得も同じドライバーを通して行える。
- 加速度・ジャイロのデータ種別を正しく識別でき、FIFO 追加後も通常取得を検証する。
- ODR、実際のサンプル数、FIFO overrun、バッファ欠落、I²C エラー、転送時間、
  CPU 使用率を記録する。6,667 Hz の両センサーで無欠落を達成できない場合は明記する。
- 開始・停止・再開、センサー設定の変更、timeout やバス異常時の回復を検証する。

## ステップ 3: RP2040 I²C DMA 対応

1. コピーした I²C ドライバーを RP2040 の DMA TX/RX に対応させる。
2. レジスターアドレスの送信、repeated START、読み出し要求、受信、STOP の順序と
   DMA の同期を設計する。DMA 対応の有無にかかわらず、Zephyr I²C API の意味を維持する。
3. 通常転送、短い転送、並行要求、NACK、タイムアウト、転送中止・復帰時のリソース
   解放と割り込みの競合を検証する。RP2040 の DMA abort に関する errata も確認する。
4. ステップ 2 の IMU ドライバーの Sensor I/F は変更せず、DMA 有効・無効の性能を比較。

完了条件:

- I²C shell、IMU、他の I²C 利用者が同じコントローラーを安全に共有できる。
- データの正確性と欠落状況を、DMA 無効時と比較できる。
- I²C 転送時間、CPU 使用率、エラー数、FIFO overrun を記録する。
- DMA は CPU 負荷の低減が目的であり、I²C バス速度そのものは増えないことを前提に評価する。

## 参考資料

- [Zephyr Sensor Read and Decode](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/read_and_decode.html)
- [Zephyr 6DoF FIFO streaming sample](https://docs.zephyrproject.org/latest/samples/sensor/6dof_fifo_stream/README.html)
- [Linux IIO st_lsm6dsx FIFO 実装](https://github.com/torvalds/linux/blob/master/drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_buffer.c)
- [ST LSM6DSOX アプリケーションノート AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf)
- [Arduino Nano RP2040 Connect 回路図](https://content.arduino.cc/assets/ABX00053-schematics.pdf)
- [Raspberry Pi Pico SDK ハードウェア API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html)
