# NuttX 版

[English](README.md) | 日本語

`zephyr/app/usb-console` の IMU 測定ファームウェアを NuttX 13.1.0 に移植したものを置くディレクトリです。
現在は移植中で、環境構築だけができます。

## 環境構築

sudo を使わずに、必要なものをすべて `nuttx/` の下に用意します。ホストには次のツールが必要です。

- `git`、`curl` または `wget`、`tar`、`xz`、`sha256sum`
- `python3`（`venv` モジュールを含む。Ubuntu では `python3-venv`）
- `make`、`gcc`（NuttX がホスト用のツールをビルドするため）

リポジトリのルートで、セットアップスクリプトを読み込みます。

```bash
source nuttx/script/setup.sh
```

スクリプトは、まだ無いものだけを取得します。既にあるものには触れないので、何度読み込んでも
かまいません。新しいターミナルでは再度読み込んでください。取得するものと版は次のとおりです。
版はスクリプトの冒頭で固定しています。

| 置き場所 | 内容 | 版 |
|---|---|---|
| `nuttx/nuttx` | NuttX 本体 | タグ `nuttx-13.1.0`（shallow clone） |
| `nuttx/nuttx-apps` | NuttX のアプリ | タグ `nuttx-13.1.0`（shallow clone） |
| `nuttx/pico-sdk` | Raspberry Pi Pico SDK。boot2（flash の second stage）のビルドにだけ使います | タグ `2.2.0`（shallow clone、サブモジュールなし） |
| `nuttx/toolchain/` | Arm GNU Toolchain（arm-none-eabi、x86_64 Linux 用） | 13.2.Rel1（SHA-256 を検証） |
| `nuttx/.venv` | Python の venv。NuttX の `make menuconfig`・`olddefconfig` が使う kconfiglib | kconfiglib 14.1.0 |
| `nuttx/stmemsc/` | ST の LSM6DSO レジスタドライバー（`lsm6dso_reg.c`、`lsm6dso_reg.h`） | lsm6dso-pid v3.2.2（Zephyr の hal_st と同じ内容。SHA-256 を検証） |

あわせて次のことを行います。

- `nuttx/nuttx/external` → `nuttx/drivers`、`nuttx/nuttx-apps/external` → `nuttx/apps` の symlink を
  張ります。NuttX はここから、このリポジトリのドライバーとアプリをビルドします。
- `nuttx/patches/nuttx/*.patch` と `nuttx/patches/apps/*.patch` があれば、それぞれ `nuttx/nuttx` と
  `nuttx/nuttx-apps` に適用します。適用済みのパッチは当て直しません。
- 現在のシェルに `PICO_SDK_PATH` を設定し、`PATH` にツールチェーンの `bin`、`nuttx/.venv/bin`、
  `nuttx/script` を加えます。

`nuttx/script/kconfig-tweak` は、NuttX の `tools/configure.sh`（から呼ばれる `tools/sethost.sh`）が
`.config` の書き換えに使う `kconfig-tweak` の代わりです。kconfiglib には含まれていないため、
NuttX が使う機能だけを実装しています。

取得したものは Git の管理外です。版を変えるときは、スクリプト冒頭の値を直してから、該当する
ディレクトリを消して読み込み直します。
