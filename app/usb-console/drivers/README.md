# ローカルドライバーの基準版

`lsm6dso/` と `i2c_dw/` は Zephyr v4.5.0、コミット
`41d10b59730406bbab4ca9f1fbbe9e96aafb4966` の各ドライバーからコピーした。
各ソースの著作権・Apache-2.0 ライセンス表示を維持している。コピーした
ソースに FIFO や DMA の変更はない。

アプリの CMake は Zephyr のドライバーライブラリー内の対象ソースをコピーに
置き換える。Kconfig、devicetree binding、ST の共通バス層と HAL は Zephyr の
ものを使用する。`CONFIG_LSM6DSO` および `CONFIG_I2C_DW` は有効のままであり、
同一 compatible を持つ全インスタンスにローカル版が適用される。標準版と
ローカル版は同時にコンパイルしない。Zephyr のライブラリー構成が変わった場合、
置換対象が見つからなければ構成エラーにする。

この構成はビルドと、100 kHz と 1 MHz での I²C scan と sensor get、および
100 kHz での加速度・ジャイロの sampling_frequency 設定を実機で確認した。
NACK・無効引数などの確認は残っている。FIFO と DMA は未実装。
