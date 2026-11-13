# NuttX 版

[English](README.md) | 日本語

`zephyr/app/usb-console` の IMU 測定ファームウェアを NuttX 13.1.0 に移植したものを置くディレクトリです。
現在は移植中です。今あるのは、環境構築、OS に依存しない共通ソースのホストテスト、ボード定義とその
ビルド、USB コンソールと書き込みスクリプト、RAM ログ、キーと値の保存（`kv`）、NINA-W102 との SPI の
やり取り（`nina version`）です。ファームウェアは
起動して USB コンソール（CDC ACM）に nsh のプロンプトを出すところまでで、測定の機能はまだありません。

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
- `nuttx/patches/nuttx/*.patch` と `nuttx/patches/apps/*.patch` を、それぞれ `nuttx/nuttx` と
  `nuttx/nuttx-apps` に名前の順に適用します（下の「NuttX へのパッチ」）。適用したものはクローンの
  `.git/nano-patches/` に記録し、同じものは当て直しません。パッチを足す・消す・書き換えると、次に
  読み込んだときに、記録にあるパッチを逆順に外してから今のパッチを当て直します。記録がないのに
  クローンに変更があるときは何もせずに止まるので、`git -C nuttx/nuttx checkout -- .`（`nuttx-apps` も
  同様）で戻してから読み込み直します。
- 現在のシェルに `PICO_SDK_PATH` を設定し、`PATH` にツールチェーンの `bin`、`nuttx/.venv/bin`、
  `nuttx/script` を加えます。

### NuttX へのパッチ

NuttX 13.1.0 本体（`nuttx/patches/nuttx/`）と nuttx-apps（`nuttx/patches/apps/`）へのパッチです。
シェルを Zephyr 版と同じ振る舞いにするものと、ボード側の実装に置き換える in-tree のファイルをビルドから
外すものがあります。各ファイルの先頭に目的と対応する Zephyr の箇所を書いてあります。

| パッチ | 内容 |
|---|---|
| `nuttx/0001-cdcacm-no-at-protocol` | CDC ACM の通信インターフェースのプロトコルを 1（AT コマンド）から 0 にする（Zephyr 版と同じ。ModemManager がモデムの候補として扱わない） |
| `nuttx/0002-cdcacm-drop-output-without-dtr` | PC 側でポートを開いているプログラムがない間（DTR が落ちている間）の出力を捨てる |
| `nuttx/0003-serial-canonical-ctrl-c-eof` | 標準入力を読むコマンドの読み込みを Ctrl-C・Ctrl-D で終わらせる |
| `nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom` | `CONFIG_ALARM_ARCH` のとき `rp2040_timerisr.c`（SysTick の周期割り込み）を、`CONFIG_ARCH_IDLE_CUSTOM` のとき `rp2040_idle.c` をビルドから外す（システム時計とアイドルループをボード側で持つため） |
| `apps/0001-readline-echo-in-readline` | 行の入力中のエコーをシリアルドライバーではなく readline が行う（行に入れた文字と改行だけを返す） |
| `apps/0002-readline-cr-lf-line-end` | CR、LF、CR LF、LF CR のどれでも 1 回の Enter にする |
| `apps/0003-readline-ctrl-c` | Ctrl-C で入力中の行を捨てて新しいプロンプトを出す |
| `apps/0004-readline-full-line` | 行が満杯になったら、行を分けずに後の文字を捨てる |

`nuttx/script/kconfig-tweak` は、NuttX の `tools/configure.sh`（から呼ばれる `tools/sethost.sh`）が
`.config` の書き換えに使う `kconfig-tweak` の代わりです。kconfiglib には含まれていないため、
NuttX が使う機能だけを実装しています。

取得したものは Git の管理外です。版を変えるときは、スクリプト冒頭の値を直してから、該当する
ディレクトリを消して読み込み直します。

## ビルド

```bash
nuttx/script/build.sh
```

カスタムボード `nuttx/boards/arduino-nano-rp2040-connect/` を、唯一の構成 `configs/imu/defconfig` で
ビルドし、`nuttx/build/nuttx.uf2` を作ります。スクリプトが `setup.sh` を自分で読み込むので、
セットアップスクリプトを読み込んでいないシェルでも動きます。今のファームウェアは最小構成で、
USB コンソールに nsh が出るだけです（「シェル」の節を参照）。

スクリプトは次のことを行います。

1. まだ configure していないとき、または前回の configure から defconfig、ボードの `Kconfig`、
   `nuttx/drivers/` と `nuttx/apps/` の下の `Kconfig`、アプリのディレクトリの構成、パッチ、NuttX と
   apps の commit のどれかが変わったとき（`nuttx/build/.configure-stamp` に記録）は、NuttX を
   configure します（`tools/configure.sh -l -a ../nuttx-apps`）。NuttX はボードの `Kconfig` を読み込み
   `nuttx/apps/Kconfig` を生成するのを configure のときにしか行わないため、`make distclean` から
   やり直します。それ以外のときは差分ビルドです。
2. `nuttx/nuttx` で `make` を実行します。
3. イメージを検査します。`.boot2` が 0x10000000 にあって 256 バイトであること（`PICO_SDK_PATH` が
   無いと、NuttX はエラーを出さずに boot2 の無いイメージを作ります）、`nuttx.bin` が ELF の
   flash に載せる範囲とちょうど一致すること、イメージが 0x10ffe000 より前で終わることです。
   flash の末尾 8 KB（0x10ffe000〜0x10ffffff）は、Zephyr 版と同じく KV のためにイメージから
   外してあります。
4. `nuttx/nuttx/nuttx.bin`（0x10000000 からの生のイメージ。boot2 を含む）を
   `nuttx/script/bin2uf2.py` で `nuttx/build/nuttx.uf2` に変換し、パスとサイズを表示します。
   変換の前に boot2 のチェックサムを確かめ、256 バイトのペイロードと RP2040 のファミリー ID
   （0xe48bff56）で書き出します。NuttX 自身の UF2 生成（picotool）は切ってあります。

どの段階でも、失敗するとエラーで止まります。

| オプション | 動作 |
|---|---|
| `--clean` | `make distclean` から configure し直す |
| `--menuconfig` | 必要なら configure し、`make menuconfig` を実行する（ビルドはしない） |
| `--savedefconfig` | `make savedefconfig` を実行し、結果に `CONFIG_APPS_DIR` を加えて `configs/imu/defconfig` に書き戻す（ビルドはしない） |
| `-j N` | make の並列数（既定は CPU の数） |
| `-v` | make の出力を詳しくする（`V=1`） |

`--menuconfig` で変えた内容は、次に configure するまで `nuttx/nuttx/.config` にだけ残ります。
残すには `--savedefconfig` で保存してください。ほかの生成物は、ELF が `nuttx/nuttx/nuttx`、生の
イメージが `nuttx/nuttx/nuttx.bin` です。NuttX が `nuttx/drivers/` と `nuttx/apps/` に書き出すものを
含め、ビルドの生成物は Git の管理外です。

## 書き込み

```bash
nuttx/script/flash.sh            # nuttx/build/nuttx.uf2 を書き込む
nuttx/script/flash.sh --build    # ビルドしてから書き込む
```

UF2 は RP2040 の USB ブートローダー（BOOTSEL。`RPI-RP2` という USB ストレージが現れる状態）から
書き込みます。最初の 1 回は、ボードに Zephyr 版（または `bootsel` コマンドの無いもの）が載っているので、
手動で BOOTSEL に入れます。ボードを USB に接続して RESET ボタンを 2 回押します。入れない場合は、`REC` と
`GND` をジャンパー線で接続したまま RESET ボタンを押し、ストレージが現れたらジャンパー線を外します。
そのあとで `flash.sh` を実行します。2 回目以降は、`flash.sh` が NuttX 版の USB コンソールに `bootsel` を
送って BOOTSEL に入れるので、ボタンの操作は要りません。

スクリプトは、sudo を使わずに次のことを行います。

1. UF2 が RP2040 用の UF2 であること（マジックナンバー、ファミリー ID 0xe48bff56）を確かめます。
2. ストレージを探します。`lsblk` で、ベンダーとモデルが RP2040 のブート ROM のもの（`RPI`、`RP2`）で
   ある USB ディスクの、ラベル `RPI-RP2` のパーティションを探します。無ければ NuttX 版の USB コンソール
   （`/dev/serial/by-id/usb-*NuttX_USB_Console*`）を探し、`bootsel` を送ってストレージが現れるのを
   待ちます。どちらも無ければ、手動で BOOTSEL に入れる方法を表示し（Zephyr 版のコンソールがあれば
   そのことも表示します）、ストレージが現れるのを待ちます。ストレージや NuttX 版のコンソールが複数
   あるときは、別のボードに書き込まないよう止まります。`/dev/ttyACM*` の番号やマウントパスは決め打ち
   しません。
3. ストレージがマウントされていなければ、デスクトップが自動でマウントするのを数秒待ち、それでも
   なければ `udisksctl mount`（sudo 不要）を試します。それも失敗したら、手動でマウントするよう
   表示して止まります。
4. ストレージの `INFO_UF2.TXT` が RP2040 のブート ROM のもの（`Model: Raspberry Pi RP2`、
   `Board-ID: RPI-RP2`）であることを確かめ、UF2 を `cp` でコピーして `sync` します。
5. ストレージが消えて NuttX 版の USB コンソールが現れるのを待ち、その `/dev/serial/by-id/` の
   パスを表示します。

| 引数・オプション | 動作 |
|---|---|
| `UF2` | 書き込む UF2（既定は `nuttx/build/nuttx.uf2`） |
| `--build` | 先に `nuttx/script/build.sh` を実行する |
| `--wait 秒` | ストレージを待つ時間と、そのあとコンソールを待つ時間（既定は 60） |
| `--dry-run` | ストレージとコンソールを探して（`lsblk` と `/dev/serial/by-id/` だけ）、何をするかを表示する。ビルド、マウント、ストレージの読み出し、送信、コピー、待機はしない |

## シェル

コンソールとシェルは、Zephyr 版（Zephyr のシェル）と同じく USB CDC ACM のシリアルポートで、NuttX 版
では nsh です。USB デバイスのシリアル番号は Zephyr 版と同じチップ ID の 16 進 16 桁なので、ポートは
`/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_<チップ ID>-if00` として現れます。ここに端末を接続します。

```bash
picocom /dev/serial/by-id/usb-NuttX_NuttX_USB_Console_*-if00
```

- プロンプトは `nsh$ ` です。Enter を押すと新しいプロンプトが出ます。シェルは PC が USB デバイスを
  構成した時点で始まり、NuttX 標準の USB コンソールのように Enter を 3 回押すのを待ちません。ポートを
  閉じて開き直してもシェルは続きます。ケーブルを抜くとシェルは終わり、接続し直すと新しいシェルが
  始まります。
- PC 側でポートを開いているプログラムがない間（DTR が落ちている間）の出力は捨てます。再起動や
  書き込みの後にポートを開いても、古いプロンプトは届きません（届くと、USB の再列挙で ECHO が有効に
  戻った PC の tty がそれを送り返し、最初のコマンドが壊れるため）。そのため、開いた直後は何も
  表示されず、Enter（または Ctrl-C）で最初のプロンプトが出ます。DTR を上げない端末からでも、何か
  送れば以後の出力は届きます。
- 入力はエコーバックされます。Enter は CR（端末ソフト）、LF、CR LF（`host/imu_host`）、LF CR の
  どれでもよく、どれも 1 行を 1 回だけ実行します（Zephyr 版と同じ）。
- 1 行は 511 文字まで入力できます（Zephyr 版と同じ）。それより後の文字は Enter まで捨て、エコーも
  しません。上下の矢印で直前の 8 行を呼び出せ、Tab でコマンド名を補完し、Ctrl-A、Ctrl-E、Ctrl-K、
  Ctrl-U、Ctrl-W で行を編集できます。
- Ctrl-C は入力中の行を捨てて改行し、新しいプロンプトを出します（空の行でも同じ。Zephyr 版と同じ）。
  コマンドの実行中に押した Ctrl-C は、標準入力を読んでいるコマンド（引数なしの `cat` など）なら
  その読み込みを終わらせ（Ctrl-D も同じ）、そうでなければ次のプロンプトで行を消す操作として
  扱われます。実行中のコマンドを止めることはありません（Zephyr 版でも実行中のコマンドは止まりません）。
  終わらないコマンド（`while`/`until` のループ、`watch`）は外してあります。
- `help` でコマンドの一覧が出ます。今は大半が nsh のコマンド（`uname`、`ps`、`free`、`echo`、`sleep`
  など）で、この移植のコマンドは移植が進むにつれて加わります。存在しないコマンドを打つと
  `nsh: <名前>: command not found` と出ます。
- `reboot` でボードを再起動します（Zephyr 版の `kernel reboot`）。引数を付けても（`reboot --help` でも）
  再起動します。
- `bootsel` は、メッセージを送り終えてから、ボードを BOOTSEL（`RPI-RP2` のストレージ）で再起動します。
  引数を付けると（`bootsel --help` など）使い方を表示するだけです。`flash.sh` のための開発用の
  コマンドで、`CONFIG_NANO_APP_BOOTSEL` で外せます。
- ログは RAM に置き、Zephyr 版と同じくコンソールには出しません。`dmesg` で表示します（「ログ」の節）。
- シェル（nsh のセッション）と、そこから起動するコマンドは優先度 104 で動きます（Zephyr 版のシェルの
  スレッドと同じ位置。「スレッドの優先度とスタック」の節）。

UART0（GPIO0 が TX、GPIO1 が RX。Arduino の D1/D0）はコンソールではありません。Zephyr 版と同じく
ピンは UART に割り当てますが、何も出力しません。

## ログ（`dmesg`）

Zephyr 版の `src/log_ram.c` と同じ RAM ログです（`nuttx/drivers/log/`）。

- 4096 バイトのリングバッファーを `.noinit` に置きます。起動処理はこの領域を消さないので、`reboot` などの
  リセットをまたいで内容が残ります。起動時に先頭の情報（マジックナンバー
  `0x4c4f4752` とその反転、開始位置、長さ）を確かめ、壊れていれば空にします。電源を切ると消えます。
  いっぱいになると古いバイトから上書きします。
- `dmesg` は保存されている内容をそのまま表示し、空なら `No logs.` と出します。引数は見ません。
  Zephyr 版と同じく 96 バイトずつ取り出して表示し、表示中に新しいログで押し出された部分は飛ばし、
  合わせて 8192 バイトで打ち切ります。NuttX 標準の `dmesg`（`/dev/kmsg`）は外してあります。
- 行の形式は Zephyr 版のログと同じ `[時:分:秒.ミリ秒,マイクロ秒] <inf> モジュール名: 本文` で、改行は
  LF だけです。レベルは `<err>`、`<wrn>`、`<inf>`、`<dbg>` で、`<dbg>` では `モジュール名.関数名: ` に
  なります。時は 2 桁以上で、99 時間を超えると桁が増えます。時刻はボードの単調時計（TIMER、
  起動からの µs）で、再起動すると 0 から数え直します（Zephyr 版と同じ見え方）。
- `syslog()` の出力（カーネルや nsh のメッセージ、PANIC の報告）も同じリングに入ります。Zephyr 版の
  `printk` と同じく、前置きを付けずにそのまま入れ、改行は CR LF にします（syslog の
  `CONFIG_SYSLOG_CRLF`。Zephyr 版の printk の文字列と同じ）。
- ソースからは Zephyr と同じ書き方で使います。`#include "app_log.h"`（`nuttx/drivers/include/`。
  カーネル側・ボード・アプリのどれからも使え、割り込みハンドラーからも呼べます）のあと、ファイルごとに
  `LOG_MODULE_REGISTER(名前, LOG_LEVEL_INF)`（レベルを省くと `CONFIG_NANO_LOG_DEFAULT_LEVEL`、既定 3。
  include の前に `LOG_LEVEL` を定義していればその値）、本文で `LOG_ERR()`、`LOG_WRN()`、`LOG_INF()`、
  `LOG_DBG()`、`LOG_ERR_RATELIMIT()` などを書きます。ファイルのレベルより上のメッセージはコンパイル時に
  捨てます。実行時のレベル変更（Zephyr の `log` コマンド）はありません。`<syslog.h>` の `LOG_ERR`
  （syslog の優先度）は同名のマクロで置き換えるので、`app_log.h` を読み込んだファイルでは
  `syslog(LOG_ERR, ...)` の代わりに `APP_LOG_SYSLOG_ERR` を使います。
- 1 行は呼び出したスレッド（または割り込み）の中で、そのスタック上の 192 バイトのバッファーに整形して
  から、割り込みを禁止してリングに書きます。本文が長いと 192 バイトに収まるところで切れます。
  `LOG_INF()` 1 回に約 42 µs、スタックを約 500 バイト使います（実機で 100 回の平均）。Zephyr 版の
  既定（deferred）では、呼び出し側は記録するだけで、整形は優先度の低いログスレッドが後で行います。

## キーと値の保存（`kv`）

設定と校正値を flash の末尾 8 KiB（`0x10ffe000`〜`0x10ffffff`、4 KiB の消去単位 2 つ）に保存します。
Zephyr 版の `storage_partition` と同じ場所です。API とコマンドは Zephyr 版の `src/kv_store.[ch]` と同じで、
保存形式は Zephyr 版と互換がありません。

- 領域: ボードの `include/board.h`（`BOARD_KV_OFFSET`、`BOARD_KV_SIZE`）とリンカースクリプト
  （`scripts/nano-flash.ld` の `__kv_start`、`__kv_end`。この領域はイメージの flash 領域から外してあり、
  `build.sh` もイメージの終端を確かめます）で決まります。`src/nano_flash_kv.c` がこの範囲だけを見せる
  MTD で、消去と書き込みのたびに、対象がこの範囲に収まることを、定数とリンカースクリプトの値の両方と
  照らしてから（起動時に両者が一致し、イメージが手前で終わることを確かめてから）flash に触れます。
  範囲の外なら何もせずエラーにします。
- 形式: NuttX の NVS（`drivers/mtd/mtd_config_nvs.c`、`CONFIG_MTD_CONFIG_NVS`・`CONFIG_MTD_CONFIG_NAMED`、
  名前は 63 文字まで）を `/dev/config` として使い、Zephyr 版と同じく `app_kv/<キー>` の名前で保存します。
  MTD は書き込みの単位を 1 バイトと見せ（項目を詰めて置くため）、flash には 256 バイトのページ単位で、
  データの外を 0xFF にして書きます。
- Zephyr 版の後で初めて起動したとき: 起動時に、領域がこの形式でも全面消去済みでもなければ（Zephyr 版の
  データが残っているとき）、領域を消去してから使います。そのとき `dmesg` に
  `<wrn> fs_nvs: Erasing the key-value area: not in this build's format` が出ます。いつもの起動では
  `<inf> fs_nvs: 2 Sectors of 4096 bytes` だけが出ます（Zephyr 版の起動時の `fs_nvs` の 1 行目と同じ。
  2・3 行目の書き込み位置は出しません）。値は引き継がれないので、下の手順で退避して書き戻します。
  NuttX 版の後で Zephyr 版に戻したときの Zephyr 版の動作は確かめていません。
- コマンド（優先度 104、スタック 3072。サブコマンドの扱いは下の「コマンドのサブコマンド」）:

  ```
  kv set <キー> <文字列>       文字列をそのまま保存
  kv sethex <キー> <16 進数>   1〜128 バイトを 16 進数の対で保存（大文字も可）
  kv get <キー>                値を小文字の 16 進数で表示
  kv delete <キー>             削除（無いキーでも Deleted）
  ```

  キーは英数字・`_`・`-`・`.` の 1〜32 文字、値は 1〜128 バイトです。`sys.` で始まるキーは
  `Reserved key` になります。出力は Zephyr 版と同じです（`Saved`、`Deleted`、`KV write failed: -22`、
  `KV read failed: -2`、`Expected 1-128 bytes as hexadecimal pairs`、`Invalid hexadecimal value` など）。
  `kv` だけ、または知らないサブコマンドではヘルプを、引数の数が違うと `<サブコマンド>: wrong parameter count`
  とサブコマンドのヘルプを出します（Zephyr のシェルと同じ文）。Zephyr 版と違う点: 引数は nsh が解釈する
  （`$`、`;`、`#`、`>` などは引用符で囲む）、エラーの行は標準エラー（同じコンソール）に出る、終了状態は
  成功で 0・失敗で 1。
- 容量: 生きている項目はすべて 1 つの消去単位（4 KiB）に収まる必要があります（もう 1 つは整理用）。
  1 項目は「キーの長さ + 8（`app_kv/` と終端）+ 値の長さ + 16」バイトで、使えるのは 4048 バイトです。
  32 文字のキーと 128 バイトの値なら 184 バイトで 22 項目まで（実機で 21 項目と `imu.port` が入り、
  次で `KV write failed: -28`）。Zephyr 版（settings on NVS、同じ 4 KiB×2）も 1 項目が名前と値の 2 件
  （183 バイト）で、ほぼ同じです。同じ値を保存し直しても書き込みません。
- 書き換えと整理: 書き換えは新しい項目を足して古い項目に印を付け、1 つの消去単位が埋まると、生きている
  項目をもう 1 つへ写して元を消去します（NVS のガベージコレクション）。flash を消去・書き込みする間は
  XIP が止まるので、RAM 上のコードで割り込みを禁止して行います。禁止する時間は 4 KiB の消去 1 回が
  33〜40 ms、256 バイトの書き込み 1 回が 0.37〜0.51 ms です（実機、TIMER で測定。消去 200 回余り、
  書き込み 3 万回余り）。項目を 1 つ書き換えると、ふつうはページの書き込みが 4 回です。Zephyr 版の
  flash ドライバーも、消去は 1 回（4 KiB）ごと、書き込みは 1 回の呼び出し（項目のデータ、最大 2 ページ）
  ごとに割り込みを禁止します。
- 電源断: NVS はデータの後に CRC 付きの索引を書き、古い項目の印は新しい項目を書いた後に付けます。
  起動時に、書きかけの項目や途中で止まった整理を NVS が直します。電源を切る試験はしていません。
- API（`nuttx/apps/imu/kv_store.[ch]`。ヘッダーは Zephyr 版と同じ）: `kv_store_init()`・`set`・`get`・
  `delete` は Zephyr 版と同じ引数・戻り値です（キーや値が不正なら `-EINVAL`、無いキーは `-ENOENT`、
  バッファーが小さければ `-ENOBUFS`、いっぱいなら `-ENOSPC`）。`/dev/config` が無いときは `-ENODEV`
  です。呼び出しのたびに `/dev/config` を開いて閉じ、排他はドライバーのミューテックス（優先度継承あり）
  が行うので、どのスレッドからも呼べます。`/dev/config` はボードが起動時に用意し、init が Zephyr 版の
  `main()` と同じ位置で `kv_store_init()` を呼びます（失敗すると `Could not initialize settings: <値>`）。

### コマンドのサブコマンド

`kv`・`nina` など、この移植のサブコマンドを持つコマンドは、共通の部品 `nuttx/apps/subcmd/`
（`subcmd_main()`）で、Zephyr のシェルと同じ出力にしています。Zephyr の `SHELL_CMD()`（引数の数を検査しない）は
`SUBCMD()`、`SHELL_CMD_ARG()`（必須と省略可の数）は `SUBCMD_ARG()` で表します。

- コマンドだけ、または知らない語: 一番深く一致したコマンドのヘルプ（`<名前> - <説明>`、`Subcommands:`、
  各サブコマンドの行）。
- 引数の数が違う: `<サブコマンド>: wrong parameter count`（標準エラー）とそのサブコマンドのヘルプ。
- サブコマンドの直後の語に `*` か `?` があり、どのサブコマンドにも一致しないときは、Zephyr のシェルと
  同じくそこで語の読み取りを止めます（`kv get a*` は引数の数の誤り、`nina version *` は `nina version`）。
  一致するサブコマンドがあるとき（Zephyr のシェルは名前に展開する）は再現せず、知らない語として扱います。
- `-h`・`--help` はふつうの語です（Zephyr 版と同じ）。ヘルプの行は 80 桁で折り返しません。
- 終了状態は、成功で 0、失敗・ヘルプ・引数の誤りで 1 です。

### Zephyr 版からの移し替え

```bash
# Zephyr 版が動いている間に退避する（host/recordings/kv-backup.json、Git 管理外）
host/.venv/bin/python script/kv_transfer.py backup --usb
# NuttX 版を書き込んだ後で書き戻す（Wi-Fi を登録しない場合）
host/.venv/bin/python script/kv_transfer.py restore --usb --no-wifi
```

`restore` は `kv sethex` で書き、`kv get` で読み返して確かめます（`imu.session` は書き戻しません）。
校正値を書いた後に `imu cal reload` を送るので、`imu` コマンドがまだ無い今の NuttX 版では、書き込みと
読み返しが済んだ後、そこでエラーになって終わります。

## NINA-W102（`nina`）

Wi-Fi モジュール NINA-W102 とは SPI1 でやり取りします。NINA のファームウェアは Zephyr 版と同じもの
（Arduino の NINA ファームウェア 3.0.2 に修正を当てたもの、`firmware/nina-fw/README.md`）で、書き換えません。
今あるのは、コマンドを 1 つ送って応答を受け取る転送層（`nina_request()`、`nina_ip_config()`）と
`nina version` です。Wi-Fi の接続とソケットはまだありません。

```
nsh$ nina version
NINA firmware: 3.0.2
```

- `nina version` は Zephyr 版と同じ出力です。失敗すると `NINA version request failed: <値>` を標準エラー
  （同じコンソール）に出し、終了状態は 1 です。`nina` だけ、または知らない語ではヘルプを出します。
  `version` は Zephyr 版と同じく引数の数を検査しないので、余分な語は無視します（優先度 104、スタック 3072）。
- 配線（ボードの `include/board.h` の `BOARD_NINA_*`。Zephyr のボード定義
  `arduino_nano_connect.dts` と同じ）:

  | 信号 | GPIO | 設定 |
  |---|---|---|
  | SCK | 14 | SPI1。プルなし、4 mA、スルーレート低速、シュミットトリガーなし |
  | MOSI（TX） | 11 | 同上 |
  | MISO（RX） | 8 | 同上、入力有効 |
  | CS | 9 | GPIO の出力、active low。プルなし。起動時に非アクティブ（high） |
  | READY | 10 | GPIO の入力、active low、プルアップ |
  | RESET | 3 | GPIO の出力、active low。プルなし |
  | IRQ（NINA の GPIO0） | 2 | プルダウン。リセット中は high を出力し、その後は入力 |

- SPI: モード 0（CPOL 0、CPHA 0）、8 ビット、MSB が先、`CONFIG_NANO_NINA_SPI_FREQUENCY`（既定 8000000。
  Zephyr 版の `CONFIG_NINA_SPI_FREQUENCY`）。PL022 は clk_peri（125 MHz）を偶数で割るので、実際は
  7.8125 MHz です（Zephyr 版も同じ値）。NuttX の SPI1 ドライバー（`rp2040_spi.c`）をポーリングで使います
  （Zephyr 版も割り込み・DMA なし）。`CONFIG_RP2040_SPI_DMA` は切ってあります（有効だと 4 バイトを超える
  転送が DMA になり、その完了待ちに期限がありません）。CS は SPI ドライバーではなく転送層が GPIO で動かし、
  応答を待つ間も保持します。SPI1 の `/dev/spi1` は作りません（`CONFIG_RP2040_SPI_DRIVER` なし）。
- NINA の初期化は Zephyr 版と同じく最初の要求のときに行います: CS を非アクティブ、READY を入力、IRQ を
  high、RESET をアクティブにし、10 ms 後にリセットを解き、750 ms 待ってから IRQ を low にして入力に戻します。
  そのため起動（書き込み・`reboot`）後の最初の `nina version` は約 0.76 秒かかります（実機で 761 ms。
  2 回目以降は約 0.6〜0.7 ms）。起動時にはボードが SPI1 とそのピン、CS だけを用意します。
- 要求の手順と値は Zephyr 版の `drivers/nina/nina_wifi.c` と同じです（`nuttx/drivers/nina/nina_spi.c`）。
  READY を 100 µs ごと（tick に切り上げて 1 tick 足すので、実際は 100〜200 µs ごと）に見て、送信前は
  2000 ms、CS を下げてから NINA が READY を戻すまで 20 ms、応答の準備は 10000 ms で `-ETIMEDOUT`。
  応答の先頭（0xE0）は最大 1000 バイト探します。要求は再帰可能なミューテックス（優先度継承あり）で
  直列化し、呼び出したスレッドの優先度で SPI を動かします。
- 実測（実機、TIMER で測定。`nina_request(0x37)` を間を置かずに 1000 回）: 要求 1 回は平均 591 µs
  （最小 427、最大 697）、CPU 時間は約 360 µs（`cpu_load_get()` の差から）。1 バイトの転送は 1 回あたり
  約 5.1 µs、まとめての転送は 1 バイトあたり読み出し 1.26 µs・書き込み 1.22 µs（線上の 8 ビットは 1.02 µs）。
  READY を 1 回見るたびの待ちは約 200 µs、その CPU 時間は約 50 µs です。

## I2C0（`i2c`）

I2C0 には LSM6DSOX（0x6A）と ATECC608A（0x60）が載っています。ドライバーはボード側の
`src/nano_i2c.c` で、Zephyr 版が使っている DesignWare I2C のドライバー
（`zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c`）のコントローラーの部分を写したものです。NuttX の
`rp2040_i2c.c`（`CONFIG_RP2040_I2C`）は使いません。`/dev/i2c0` として登録し、NuttX 標準の `i2c` ツールで
使えます。条件に合う読み出しは DMA で行います（下の「DMA での読み出し（`i2c_dma`）」）。

```
nsh$ i2c get -a 6a -r 0f
READ Bus: 0 Addr: 6a Subaddr: 0f Value: 6c
```

- 配線（ボードの `include/board.h` の `BOARD_I2C0_*`）: SDA が GPIO12、SCL が GPIO13。Zephyr 版の
  pinctrl と同じく、I2C の機能、内蔵プルなし、4 mA、スルーレート低速、シュミットトリガーあり、入力有効。
  起動時に I2C0 のブロックをリセットしてから設定します。
- 速さは転送ごとのメッセージの周波数で決まり、100000、400000、1000000 だけを受け付けます（Zephyr の
  速さの区分。ほかの値は `-EINVAL`）。レジスターの値は Zephyr 版と同じです: 100 kHz は
  `IC_SS_SCL_HCNT`/`LCNT` が 500/625、400 kHz は `IC_FS_SCL_HCNT`/`LCNT` が 93/109、1 MHz は 31/31
  （どちらも `IC_CON` の速さは FAST）、`IC_FS_SPKLEN` は 1、`IC_SDA_HOLD` はリセット値の 1。`IC_CON` は
  100 kHz で 0x363、それ以外で 0x365 です（RX FIFO が満杯のときバスを止めるビットを含む。Zephyr 版と同じく
  DMA が有効のときだけ付け、`i2c_dma mode off` の後の転送では 0x163・0x165）。
- DMA を使わない転送は割り込みで進めます。読み出しは RX FIFO（16 段）の空きの分だけ要求を積み、4 バイトずつ
  受け取ります。1 回の転送の期限は 1 秒で、過ぎると abort して `-ETIMEDOUT` を返します。相手が応答
  しない（NACK）などの失敗は `-EIO` で、`dmesg` に Zephyr 版と同じ `<err> i2c_dw: NACK on i2c@40044000`
  などが残ります（頻度制限あり）。失敗の後の転送はそのまま使えます。転送はバスのロックで直列化します。
- 1 回の転送のアドレスは 1 つ、周波数は全メッセージで同じでなければなりません。10 ビットアドレスは
  使えません。最後のメッセージの後は `I2C_M_NOSTOP` があっても STOP を出します（Zephyr 版と同じ）。
- 実測（実機、TIMER と `cpu_load_get()`、間を置かずに繰り返した平均）:

  | 転送 | 100 kHz | 400 kHz | 1 MHz | CPU 時間 | 割り込み |
  |---|---|---|---|---|---|
  | WHO_AM_I（1 バイト書いて 1 バイト読む） | 411 µs | 128 µs | 95 µs | 約 95 µs | 5 回（1 MHz では約 3 回） |
  | 256 バイト読み出し | 21.6 ms | 4.37 ms | 1.73 ms | 約 0.7 ms | 68〜80 回 |
  | 3584 バイト読み出し | 298 ms | 59.7 ms | 23.3 ms | 約 8.3〜8.5 ms | 900〜1040 回 |

  3584 バイトの所要時間を 1 ビットあたりにすると、100 kHz で 9.24 µs、400 kHz で 1.85 µs、1 MHz で
  0.72 µs です。この表は DMA を使わないときの値です（DMA との比較は次の節）。

### DMA での読み出し（`i2c_dma`）

Zephyr 版の `CONFIG_I2C_DW_RP2040_DMA` と同じく（`CONFIG_NANO_I2C0_DMA`、既定で有効）、次の条件をすべて
満たす読み出しを DMA で行います。外れる読み出しと書き込みは割り込みで行います。

- 転送の最後のメッセージ（STOP で終わる）で、32〜3584 バイト、7 ビットアドレス、DMA が有効
  （`i2c_dma mode on`、起動時は on）。
- DMA のチャネルを 2 本使います（コマンド語を送る TX と、データを受け取る RX）。チャネルの確保や設定に
  失敗したときは、転送を始める前に割り込みでの処理に切り替え、`dma_fallbacks` を数えます。
- 期限・エラー値は割り込みの処理と同じです（1 秒で `-ETIMEDOUT`、NACK などは `-EIO`）。失敗の後は
  DMA と I2C を止めてから戻ります。DMA のチャネルが 100 ms 以内に止まらないときは I2C0 を隔離し
  （`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`）、以後の転送はすべて `-EIO`
  になります（再起動で戻ります）。詳しい手順は `nuttx/drivers/README_ja.md` の「I2C0」の節にあります。
- NuttX の DMA 層（`CONFIG_RP2040_DMAC`）は切ってあり、DMA はボード側の `src/nano_dma.c`（Zephyr の
  RP2040 用 DMA ドライバーと同じ動作）だけが使います。

```text
i2c_dma mode          現在の方式を表示（i2c_dma=on または i2c_dma=off）
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（起動からの累積）
```

出力は Zephyr 版と同じです（`dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0`、`on`・`off`
以外は `Expected on or off`）。切り替えは Zephyr 版と同じく値を変えるだけで、進行中の転送を待ったり
止めたりしません。「止まっているときだけ」は使い方の約束で、コマンドは確かめません。優先度 104、
スタック 3072 です。

実測（実機、1 MHz、LSM6DSOX のレジスターを 1 バイト書いてから読む転送、TIMER と `cpu_load_get()`、
間を置かずに繰り返した平均。割り込みの回数は 1 回の転送で数えた I2C0 の割り込み（レジスターを書く
メッセージの分を含む）と DMA の割り込み）:

| 読み出し | 方式 | 所要時間 | CPU 時間 | 割り込み |
|---|---|---|---|---|
| 256 バイト | DMA | 1.79 ms | 約 0.26 ms | I2C 3 回、DMA 2 回 |
| 256 バイト | 割り込み | 1.76 ms | 約 0.76 ms | I2C 84 回 |
| 3584 バイト | DMA | 22.9 ms | 約 0.75 ms | I2C 2〜3 回、DMA 2 回 |
| 3584 バイト | 割り込み | 23.1 ms | 約 8.9 ms | I2C 1139 回 |

所要時間と CPU 時間には、`ioctl()` で転送を渡す処理とレジスターを書くメッセージの分が含まれます
（1000 回・200 回・50 回の平均）。DMA で 256 バイトを 1000 回続けて読んでエラー 0、2 つのタスクから
同時に 500 回ずつ読んでもエラー 0（バスのロックで 1 回ずつ進みます）。DMA と割り込みで同じ範囲を
読んだ内容は、値の変わるレジスター（出力、状態、タイムスタンプ）を除いて一致しました。

### `i2c` ツールと Zephyr 版の `i2c` コマンド

Zephyr 版の `i2c` コマンド（Zephyr のシェル）は移さず、NuttX 標準の `i2c` ツール（`CONFIG_SYSTEM_I2CTOOL`）
に任せます。オプション（`-a` アドレス、`-r` レジスター、`-f` 周波数など）は次に変えるまで残ります。
周波数の既定は 100000（Zephyr 版の起動時の速さ）です。アドレス・レジスター・書く値は 16 進、`dump` の
バイト数は 10 進です。

| Zephyr 版 | NuttX 版 | 違い |
|---|---|---|
| `i2c scan i2c@40044000` | `i2c dev 03 77` | 確かめ方が違う（Zephyr 版は 1 バイト書き込み、NuttX 版は 1 バイト読み出し）。どちらも 0x60 と 0x6a が見える。`-z` は付けない（0 バイトの書き込みはできず、アドレスごとに 1 秒で時間切れになる） |
| `i2c read_byte i2c@40044000 6a 0f` | `i2c get -a 6a -r 0f` | |
| `i2c read i2c@40044000 6a 10 10` | `i2c dump -a 6a -r 10 16` | 最大 256 バイト（Zephyr 版は 64）。Zephyr 版のバイト数は 16 進 |
| `i2c direct_read i2c@40044000 6a 1` | `i2c get -a 6a` | `-r` を付けなければレジスターを送らない |
| `i2c write_byte i2c@40044000 6a 07 5a` | `i2c set -a 6a -r 07 5a` | |
| `i2c write i2c@40044000 6a 07 5a 00` | なし | NuttX 版は 1 回に 1 つの値（`-w 16` で 2 バイト） |
| `i2c speed i2c@40044000 3` | `-f 1000000`（以後のコマンドに残る） | Zephyr 版はバスの設定を変え、`imu start` も 1 MHz に変える。NuttX 版はツールの設定だけ |
| `i2c recover i2c@40044000` | `i2c reset` | どちらもバスを取って放すだけ（このボードにはリカバリーの処理がない） |

`i2c verf` は書き込んで読み返すコマンドなので、センサーの設定を書き換えます。`i2c` ツールは優先度 100 で
動きます（Zephyr 版のシェルのコマンドは 104 に当たる位置）。

## LSM6DSOX（IMU）

ドライバーは `nuttx/drivers/lsm6dso/lsm6dso.c` で、Zephyr 版の `drivers/lsm6dso/lsm6dso.c`（FIFO・校正・ゼロ点の
変更を含む Zephyr の LSM6DSO ドライバー）の OS を呼ぶ部分だけを置き換えた写しです。レジスターの操作は ST の
`lsm6dso_reg.c`（lsm6dso-pid v3.2.2。`script/setup.sh` が `nuttx/stmemsc/` に取得。Zephyr の hal_st のものと
LF に直して同一）で行います。FIFO のストリーム（INT1 の割り込みと RTIO）も Zephyr 版のファイルをそのまま写して
動かしています（下）。取得サービス（`imu start` など）はまだ移していません。

- **起動時の初期化**: ボードの起動処理（`nano_bringup()`）が、I2C0 の初期化の直後、KV（NVS）のマウントより前に
  初期化します。Zephyr 版がデバイスを `main()` より前に初期化し（`POST_KERNEL`、優先度 90）、`main()` で NVS を
  マウントするのと同じ順です。`dmesg` の最初の 2 行は Zephyr 版と同じ文です:

  ```text
  [00:00:00.007,154] <inf> LSM6DSO: Initialize device lsm6dsox@6a
  [00:00:00.008,475] <inf> LSM6DSO: chip id 0x6c
  [00:00:00.028,920] <inf> fs_nvs: 2 Sectors of 4096 bytes
  ```

  失敗したとき（チップ ID が違う、I2C の失敗）は Zephyr 版と同じ `<err>` の行を残し、デバイスは「準備
  できていない」状態のまま起動を続けます。
- **初期化の手順と値**は Zephyr 版と同じです: ユーザーのレジスターバンクを選ぶ → `WHO_AM_I` → I3C を
  無効 → センサーハブのマスターが動いていれば止めて 300 µs → ソフトウェアリセットと 100 µs →
  加速度の電力モード・フルスケール・ODR → ジャイロの電力モード・フルスケール・ODR → FIFO をバイパス →
  BDU。設定値は Zephyr 版のデバイスツリー（`app.overlay` を含む）と同じで、ボードの `include/board.h` の
  `BOARD_LSM6DSOX_*` にあります（加速度 ±8 g、ジャイロ ±2000 dps、ODR は両方停止、高性能モード）。
  初期化後のレジスターは `CTRL1_XL`=0x0c、`CTRL2_G`=0x0c、`CTRL3_C`=0x44、`CTRL9_XL`=0xe2、
  `FIFO_CTRL1`〜`4`・`COUNTER_BDR_REG1`/`2`・`INT1_CTRL`・`INT2_CTRL`・`CTRL4_C`〜`CTRL8_XL`・`CTRL10_C` が 0 です
  （`i2c dump -a 6a -r 00 32` で確認できます）。
- **I2C**: 起動時の速さはデバイスツリーの 100 kHz で、`i2c_configure()`（Zephyr の API の写し）で 400 kHz・1 MHz に
  変えられます（Zephyr 版の `imu start` は 1 MHz にします）。NuttX の I2C はメッセージごとに周波数を持つので、
  この速さは I2C0 のデバイス（`nuttx/drivers/compat`）が覚えていて、ドライバーの転送だけに使います。`i2c`
  ツールは自分の `-f` の周波数で動き、ドライバーの速さを変えません（Zephyr 版の `i2c speed` はバスの設定を
  変えます）。レジスターの読み出しは 1 回の転送（レジスター番号の書き込み → RESTART → 読み出し）、書き込みは
  STOP 付きの 1 メッセージで、Zephyr の `i2c_burst_read_dt()`・`i2c_write_dt()` と同じ形です。FIFO の読み出しは
  32 バイト以上なので DMA を使います。
- **単発の取得**（`sensor_sample_fetch()`・`sensor_channel_get()`）、**属性**（`sensor_attr_set()` の ODR と
  フルスケール）、FIFO の開始・読み出し・停止の関数、センサーのタイムスタンプの読み取り、校正値の参照と
  その場でのゼロ点の取り直し（`lsm6dso_cal_zero()`）は Zephyr 版と同じ動作です。ODR は指定値以上で最小の
  段（12、26、52、104、208、417、833、1667、3333、6667 Hz）、加速度のフルスケールは m/s² を g に四捨五入して
  2・4・8・16 g、ジャイロは rad/s を dps に四捨五入して 125・250・500・1000・2000 dps だけを受け付けます
  （ほかは `-EINVAL`）。FIFO を使っている間の単発取得と属性の変更は `-EBUSY` です。
- **FIFO のストリーム**: `lsm6dso_stream.c`・`lsm6dso_fifo_irq.[ch]` は Zephyr 版のファイルを変更せずに
  写したもので、Zephyr 版と同じく `sensor_stream()` で始まり、INT1（GPIO24）の立ち上がりエッジの割り込み →
  システムワークキュー（`sysworkq`）→ FIFO のワーカー（`imu_fifo_worker`）が FIFO を読んで RTIO の完了に
  します。ウォーターマークに届かないときは 20 ms ごとに見直し、`lsm6dso_fifo_request_stop()` で残りを最後の
  端数のバッチとして渡してから `-ECANCELED` で終えます。動かすための Zephyr のカーネル（ワークキュー、
  セマフォ、`k_poll()`）、RTIO、GPIO、センサーの非同期の API は互換層（`nuttx/drivers/compat/`）にあり、
  ボードの起動処理が `zephyr_compat_initialize()` で `sysworkq` と `RTIO WQ 0` を始めます。
  実機での確認（1 MHz、6667 Hz、ウォーターマーク 128、`imu_stream.c` と同じ RTIO の寸法。T11 の時点の
  一時的な試験コマンドで、消費側は受け取って解放するだけ）: 10 秒で完了 1070 前後、加速度・ジャイロ各
  約 67080 ワード（センサーの発振器が約 0.6 % 速い。`freq_fine` 4）、エラー・オーバーラン・`i2c_dma` の
  エラーとフォールバックは 0、INT1 の割り込みから完了を受け取るまで平均 5.5 ms（ほぼ FIFO の読み出しの
  時間）、CPU 負荷 26 %。60 秒の連続でも同じでした。
- 呼ぶ側のための Zephyr の API の写し（`struct device`、`struct sensor_value`、チャネル・属性の列挙、
  `sensor_*()`、I2C の一部、カーネル、RTIO、GPIO）は `nuttx/drivers/compat/` にあります。詳しくは
  `nuttx/drivers/README_ja.md` の「LSM6DSOX」と「Zephyr の API の互換層」の節を見てください。

## システム時計

カーネルの時計は tickless で、1 tick は 100 µs です（Zephyr 版の `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`、
`CONFIG_TICKLESS_KERNEL=y` と同じ刻み）。周期割り込みはなく、次に期限の来る待ちの時刻に RP2040 の TIMER
（64 ビット、1 MHz、水晶から）の ALARM0 を合わせます（`nuttx/boards/arduino-nano-rp2040-connect/src/`
の `nano_oneshot.c`・`nano_timerisr.c`、NuttX の `drivers/timers/arch_alarm.c`）。ALARM1〜3 は使って
いません。設定は `CONFIG_NANO_TICKLESS`（ボードの `Kconfig`。既定で有効）、`CONFIG_SCHED_TICKLESS=y`、
`CONFIG_USEC_PER_TICK=100` です。

- 待ち時間は tick に切り上げ、さらに 1 tick 足します（`usleep(1000)` は 1000〜1100 µs と起床の遅れ）。
  Zephyr 版の `k_sleep(K_MSEC(1))` と同じ規則です。
- `clock_gettime()` の `CLOCK_MONOTONIC`・`CLOCK_REALTIME` は TIMER の値をそのまま µs 単位で返します
  （`clock_getres()` は tick の長さ 100 µs を返します）。Zephyr 版のカーネル時計は CPU クロックの
  SysTick で、TIMER とは源が違います。アプリはどちらでも、時刻の基準に TIMER を直接読みます。
- ボードの単調時計は `nano_mono_us()`（`<arch/board/nano_timer.h>`）で、TIMER の TIMERAWH・TIMERAWL を
  上位、下位、上位の順に読み、上位が一致するまで読み直します（Zephyr 版の `time_sync_mono_us()` と
  同じ読み方）。副作用もロックも無いので、カーネル側・アプリ・割り込みハンドラーのどこからでも呼べ、
  1 回は約 0.3 µs です。値は起動からの µs で、TIMER は起動のたびに（`reboot` でも）0 から数え直します。
  `CLOCK_MONOTONIC` と同じカウンターなので、両者は読み出しの時間差（数 µs）の範囲で一致します。
- 同じ優先度のスレッドは 20 ms ずつの時分割（`CONFIG_RR_INTERVAL=20`。Zephyr 版の
  `CONFIG_TIMESLICE_SIZE=20`）です。時分割の残り時間を計る wdog は、スレッドが待ちに入っても
  取り消されないので、時分割の残り時間より長く眠ると、眠っている間に割り込みが 1 回余分に
  入ります（NuttX 13.1.0 の動作）。
- `up_udelay()`・`up_mdelay()`・`up_ndelay()` は TIMER を直接読みながら µs 単位で待ちます（割り込み
  禁止中でも使えます）。
- カウンターと tick・時刻の換算は、ボードが `arch_alarm.c` の weak 関数（`up_timer_gettick()`、
  `up_timer_gettime()`、`up_alarm_tick_start()`、`up_alarm_cancel()`、`up_udelay()`、`up_ndelay()`）を
  置き換えて、1 MHz と 100 µs に特化した計算（`src/nano_timeconv.h`。100 での除算を 1 バイトずつ
  32 ビットの乗算とシフトで行う）でします。Cortex-M0+ には除算命令がなく、NuttX の汎用の換算
  （`ONESHOT_COUNT`）は 64 ビットの除算になり、`clock_systime_ticks()` 1 回に約 5.5 µs、tick から
  カウンターへの換算 1 回に約 11 µs かかるためです（置き換え後は約 1.3 µs と約 0.6 µs）。結果は
  64 ビットの全域で汎用の換算と同じです（ホストテスト `nano_timeconv`）。換算が 100 µs の tick を
  前提にするので、`CONFIG_USEC_PER_TICK` が 100 でなければビルドが止まります。
- 起床の遅れ（tick の境界から `usleep()` が戻るまで）は約 17〜36 µs、`usleep(1000)` 1 回あたりの
  CPU 時間は約 30〜52 µs です（置き換え前はそれぞれ約 51〜55 µs、約 104〜114 µs。どちらも
  一時的な計測コードを入れた版で、1000 回の平均を TIMER で測った値。flash から実行するコードの
  配置で XIP キャッシュの当たり方が変わり、ビルドごとにこの程度ばらつきます）。

## CPU 負荷

`cpu_load_get(bool reset)`（`#include "cpu_load.h"`。ボードの `src/nano_idle.c`）は、Zephyr 版の
`CONFIG_CPU_LOAD`（`CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK`）と同じ意味の値を返します。

- アイドルループ（`up_idle()`。`CONFIG_ARCH_IDLE_CUSTOM=y` でボードのものに置き換えています）は、割り込みを
  禁止して TIMER を読み、`WFI` で割り込みを待ち、起きたら TIMER を読んでその間をアイドル時間に足してから
  割り込みを許可します。起こした割り込みのハンドラーは許可した後で動くので、アイドル中に入った
  割り込みの処理時間も負荷に数えます（Zephyr 版と同じ）。Cortex-M0+ の `WFI` は割り込み禁止中でも
  保留中の割り込みで戻るので、禁止してから `WFI` までの間に来た割り込みでも眠り続けません。
- 値は、前回 `reset=true` で呼んでから（初回は起動から）の `1000 × (全体 − アイドル) / 全体` を切り捨てた
  千分率です。全体の時間が 0 なら 0 を返します。`reset=true` の呼び出しも、その時点までの値を返してから
  次の区間を始めます。
- 時間の源は TIMER（1 µs）です（Zephyr 版は CPU クロックの周期を µs に換算）。1 回のアイドルの区間の
  誤差は ±1 µs で偏りはなく、区間の数が多くても結果への影響は 0.1‰ より十分小さい見積もりです。
- 実機での値（各 1〜2 回）: 何もしない 10 秒で 0（アイドルでない時間は 10 秒中 0.3〜0.4 ms）、
  優先度の低いビジーループで 1000、10 ms ずつの動作と待ちで 499。割り込みだけが多い状況として、
  約 196 µs ごとの wdog（毎秒 5099 回）で、ハンドラーの中身が 0 µs のとき 50、50 µs のとき 300
  （中身だけなら 260）で、割り込み 1 回あたり約 8〜10 µs の入口・出口の処理（タイマー割り込み、wdog、
  次のアラームの設定）も負荷に入ります。`cpu_load_get()` 1 回は約 4.8 µs です。

## スレッドの優先度とスタック

優先度は `NuttX = 118 − Zephyr` で対応させ、`nuttx/drivers/include/priorities.h` に定数（`NANO_PRIO_*`）と
スタックの大きさ（`NANO_STACK_*`）を置きます。Zephyr 版のアプリのスレッド（優先度 8〜11）は 110〜107、
シェルのスレッド（14）は 104 です。

| スレッド | Zephyr | NuttX | スタック |
|---|---|---|---|
| `wifi_reconnect_tid`、`nina_scan_tid`、`nina_connect_tid` | 8 | 110 | 2048、1536、1024 |
| `time_sync_tid`、`imu_stream_tid`、`nina_monitor_tid` | 9 | 109 | 2048、2048、1536 |
| `imu_fifo_worker` | 10 | 108 | 2048 |
| `imu_raw_mqtt_tid` | 11 | 107 | 2048 |
| NINA のイベント（NuttX 版だけ） | – | 106 | – |
| `sysworkq`（システムワークキュー。互換層。INT1 の処理） | −1（協調） | 119 | 1024 |
| `RTIO WQ 0`（RTIO のワークキュー。互換層。単発取得。`ps` では `RTIO_WQ_0`） | 0 | 118 | 1024 |
| `main`（NuttX は init。起動処理の後シェルの優先度に下げる） | 0 | 118 | 2048 |
| シェル（nsh のセッション、コマンド） | 14 | 104 | 3072 |

- Zephyr の協調スレッド（負の優先度。`sysworkq`、`net_mgmt` など）は 118 より上になりますが、NuttX には
  他のスレッドに横取りされない協調スレッドが無く、より高い優先度のスレッドには横取りされます。今の移植で
  協調スレッドに当たるのは `sysworkq`（119）だけで、それより高い優先度のスレッドはありません。
- `imu_fifo_worker`、`sysworkq`、`RTIO WQ 0` のスタックは Zephyr と同じく静的な配列です
  （`K_THREAD_STACK_DEFINE()`、NuttX の `kthread_create_with_stack()`）。`ps` の STACK は NuttX が先頭に
  置く領域の分だけ小さく出ます（2048 → 2008、1024 → 976）。
- `CONFIG_PRIORITY_INHERITANCE=y` です。ミューテックスは優先度継承あり、セマフォは既定で継承なし
  （NuttX 13.1.0 の `sem_init()` は flags を 0 にします）なので、通知用のセマフォはそのままで継承しません。
- 割り込み用のスタック（`CONFIG_ARCH_INTERRUPTSTACK=2048`。Zephyr 版の `CONFIG_ISR_STACK_SIZE=2048` と
  同じ大きさ）: スレッドはプロセススタック（PSP）、割り込みハンドラーはメインスタック（MSP）で動き、
  割り込みの処理は割り込まれたスレッドのスタックを使いません。割り込まれたスレッドのスタックに積まれる
  のは、ハードウェアが退避する 32 バイト（とコンテキストの切り替えのときの保存領域）だけです（Zephyr 版と
  同じ）。実機では、何もしていないときに割り込みの中でログを 1 行出しても、アイドルスレッドの使用量は
  304 バイトのまま変わらず（割り込み用のスタックが無かったときは 452 から 712 バイトに増えた）、割り込み
  用のスタックの使用量は通常 364 バイト、割り込みの中でログを出した後で 624 バイトでした。割り込み 1 回の
  コストは変わりません（毎秒約 5000 回の空の wdog で、10 秒間のアイドルでない時間が有無とも 500.0 ms。
  各 3 回）。割り込み用のスタックの使用量は `ps` には出ません（PANIC の報告の `irq` の行に出ます）。
- スタックの番兵（`CONFIG_NANO_STACK_SENTINEL`。ボードの `Kconfig`、既定で有効）: Zephyr 版の
  `CONFIG_STACK_SENTINEL` と同じく、各スタックの最下位のワード（`CONFIG_STACK_COLORATION` が書いた値）を、
  スレッドの切り替えのとき（切り替えで外れるスレッド）と、割り込みハンドラーの出口ごと（割り込まれた
  スレッド。ハードウェアが退避した値がそのスタックに積まれている）に確かめ、書き換わっていれば PANIC で
  止めます（報告は割り込み用のスタックの上で作り、RAM ログに入ります）。割り込み用のスタック自体は、
  Zephyr 版と同じく番兵で確かめません。スケジューラーの計測用のフック 2 つ（切り替えの `sched_note_suspend()` と、
  `irq_dispatch()` が各ハンドラーの前後で呼ぶ `sched_note_irqhandler()`。`src/nano_stackcheck.c`）で
  行います。Cortex-M0+ では周辺の割り込みは `arm_doirq()` を通らず（`exception_direct()`）、
  切り替えのフックだけでは割り込みの出口で確かめられないためです。割り込みはすべて同じ優先度で
  入れ子にならないので、どの出口も Zephyr の「入れ子でない割り込みの出口」に当たります。NuttX の
  `CONFIG_STACKCHECK_MARGIN` は切り替えのときにしか確かめず、`CONFIG_DEBUG_ASSERTIONS`（すべての
  DEBUGASSERT）も要るので使いません。割り込み 1 回あたり約 1.0 µs 増えます（約 200 µs ごとの wdog、
  毎秒 4999 回で、5 秒間のアイドルでない時間が 225.2 ms から 250.2 ms に。各 3 回）。
- `ps` の STACK・USED・FILLED でスタックの使用量を見られます。

## テスト

```bash
nuttx/script/host_tests.sh
```

OS に依存しない共通ソースのテストを、PC 上でビルドして実行します。ボードは不要で、ホストの `cc`（C17）だけを
使います。セットアップスクリプトも不要ですが、`lsm6dso_driver` だけは `script/setup.sh` が取得する ST の
`lsm6dso_reg.c`（`nuttx/stmemsc/`）を使い、無ければ `skip` と出して飛ばします。FIFO ワードの解析、バッチとペイロード、送信キュー、
status と info の JSON、時刻の対応付け、センサーの tick と時計の対応、NTP の推定、FIFO パケットの復号、物理単位への
換算と校正値の保存形式、静止中のジャイロのゼロ点の追従、ボードの時間の換算（`nano_timeconv`）、
CPU 負荷の計算（`nano_cpuload`）、RAM ログのリング・`dmesg` の取り出し・行の前置き・頻度制限（`log_ram`）、
KV 領域の範囲の検査・ページ単位の書き込み・起動時の内容の判定（`nano_flash_kv`）、KV のキーの規則
（`kv_name`）と `kv` コマンドの 16 進数の変換（`kv_hex`）、コマンドのサブコマンドの扱い（`subcmd`）、
NINA の SPI の転送層（`nina_spi`）、I2C0 のドライバーの OS に依存しない部分（`nano_i2c_dw`）、I2C0 の DMA
読み出しのコマンド語（`i2c_dma_commands`）、`i2c_dma` コマンドの出力（`i2c_dma_app`。ドライバー側は偽物）、
LSM6DSO の FIFO 制御（`fifo_hw`）、LSM6DSO のドライバーと FIFO のストリーム（`lsm6dso_driver`）、ストリームの
状態機械（`stream_stop`）、互換層の RTIO（`zephyr_rtio`）とワーク（`zephyr_work`）を確かめます。テストは 1 つずつビルドして
実行し、失敗したところで止まります。ビルドの生成物は一時ディレクトリに置き、終了時に消します。

次のファイルは Zephyr 版（`zephyr/app/usb-console`）の同名ファイルを変更せずに写したものです。
Zephyr 版を直したときは、写しも同じ内容に置き換えます。

| 置き場所 | Zephyr 版の置き場所 | 内容 |
|---|---|---|
| `nuttx/apps/imu/` | `src/` | `imu_raw_{batch,packet,queue,status,time}`、`imu_tick_clock`、`time_offset`、`imu_cal_codec.c`、`imu_cal.h`、`kv_store.h` |
| `nuttx/drivers/lsm6dso/` | `drivers/lsm6dso/` | `lsm6dso_{fifo,cal,zero,fifo_decoder,fifo_hw,fifo_irq}`、`lsm6dso_stream.c`、`lsm6dso_fifo_packet.h`、`lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h` |
| `boards/arduino-nano-rp2040-connect/src/` | `drivers/i2c_dw/` | `i2c_dw_rp2040_commands.[ch]` |
| `nuttx/tests/` | `tests/` | 上のファイルを対象にするテスト（`i2c_dma_commands.c`、`fifo_hw.c`、`stream_stop.c` を含む。`stream_stop.c` は `../drivers/lsm6dso/lsm6dso_stream.c` を読み込む） |
| `nuttx/tests/fakes/` | `tests/fakes/` | テスト用の Zephyr のヘッダーと ST のヘッダーの代わり（`zephyr/drivers/sensor.h`、`zephyr/drivers/sensor_clock.h`、`zephyr/drivers/i2c.h`、`zephyr/rtio/rtio.h`、`lsm6dso_reg.h`） |

`nuttx/tests/fakes/nuttx/config.h`（空）は NuttX 版だけのもので、`<nuttx/config.h>` を読む `nuttx/apps` の
ソースをホストテストでビルドするために置いています（`i2c_dma_app`）。

`nuttx/tests/nano_timeconv.c` は NuttX 版だけのテストです。`src/nano_timeconv.h` の換算を、C の
`/`・`%` と、NuttX 13.1.0 の汎用の換算（`oneshot.h`・`clkcnt.h` の計算を 1 MHz・100 µs で書き写した
もの）と比べます。対象は 1 バイトずつの除算が正しい範囲の全数、境界値（0、99、100、2^32 前後、
2^32×100 前後、2^63、2^64−1 など）の前後 300、最初の 10^7 µs の全数、2^32 の倍数 256 個の前後 2000、
乱数 400 万個です。

`nuttx/tests/nano_cpuload.c` と `nuttx/tests/log_ram.c` も NuttX 版だけのテストです。前者は
`src/nano_cpuload.h` の計算を Zephyr の `cpu_load_get_cpu()` の手順（リセット、開いている区間、0 の扱い、
切り捨て）と比べます。後者は `nuttx/drivers/log/log_ring.c` を、Zephyr 版の `log_ram.c` の 1 バイトずつの
書き込みと乱数の書き込み 2 万回で比べ、行の前置きを Zephyr の `log_output.c` の時刻の計算と乱数 20 万個で
比べ、`dmesg` の取り出し（上書き中の読み飛ばし、8192 バイトでの打ち切り）と頻度制限を確かめます。

`nuttx/tests/nano_flash_kv.c`、`kv_name.c`、`kv_hex.c` も NuttX 版だけのテストです。`nano_flash_kv` は
`src/nano_flash_kv.h` の範囲の検査を境界の前後と、flash の先頭と末尾 1 MiB のすべてのページ・消去単位で
確かめ、ページ単位の書き込みを、ページ境界の前後のさまざまな位置と長さで、書いたバイトだけが変わる
ことで確かめます。起動時の判定は、NuttX の NVS が書く索引を受け付け、Zephyr 版の NVS の索引の並び
（NuttX の CRC がたまたま合うものを含む）を受け付けないことを確かめます。`kv_name` は Zephyr 版の
`kv_store.c` から分けたキーの検査（`apps/imu/kv_name.c`）を、`kv_hex` は Zephyr の `lib/utils/hex.c` の
写し（`apps/kv/kv_hex.c`）を確かめます。

`nuttx/tests/subcmd.c` は `apps/subcmd/subcmd.c` を、`kv` と `nina` の表（Zephyr 版の記録した出力と同じ
ヘルプ）、入れ子の表、省略可の引数で確かめます。`nuttx/tests/nina_spi.c` は `drivers/nina/nina_spi.c` を、
偽の SPI・GPIO・時計・ロックで確かめます。Zephyr 版の `tests/nina_spi.c` の項目（広いパラメーターと詰め、
まとめての転送、0x75・0x7a の 3 つの応答パラメーター、長さの検査、転送の失敗）に加えて、狭いパラメーター、
版の要求、応答の先頭探し（1000 バイト）、各 `-EPROTO`・`-EMSGSIZE`・`-EINVAL`、0x27、`nina_ip_config()`
（0x21）、READY の期限（2000・20・10000 ms を 100 µs ごとに見る回数）、最初の要求のリセット手順、CS と
転送の順序、ロックを確かめます。

`nuttx/tests/nano_i2c_dw.c` は `src/nano_i2c_dw.h` を、Zephyr 版の `i2c_dw.c`・`i2c_dw.h` と Zephyr の
`i2c.h` の該当部分をテストの中に書き写したものと比べます。SCL のカウント（125 MHz で 500/625、93/109、31/31、
クロック 1〜400 MHz と速さの区分のすべての組み合わせ）、受け付ける周波数、NuttX のメッセージのフラグから
Zephyr のフラグへの変換、`IC_DATA_CMD` に書く語の列（`i2c` ツールの `get`・`set`・`set -s`・`dump`・`dev`、
センサーのレジスターの読み書き、3 つのメッセージなど。前の転送の向きが次の転送の最初の RESTART に効くことを
含めて、転送を続けて流す）、`IC_CON` の値とレジスターの配置を確かめます。

`nuttx/tests/lsm6dso_driver.c` も NuttX 版だけのテストです。`drivers/lsm6dso/lsm6dso.c` を、ST の本物の
`lsm6dso_reg.c`、stmemsc の層、互換層（`drivers/compat`）、ボードの `src/nano_lsm6dso.c` と一緒にビルドし、
偽の I2C マスターの先の偽の LSM6DSOX（レジスターのバンク、アドレスの自動増加、ソフトウェアリセット、FIFO、
タイムスタンプ）に対して動かします。偽物のヘッダーは `nuttx/tests/lsm6dso_fakes/`（ボードの `board.h` は本物を
読みます）。確かめるのは、初期化のログの文と順序、書き込むレジスターと値の順序（リセット後の 100 µs を
含む）、初期化後のレジスター、感度、各段階での I2C の失敗とそのログ、`device_is_ready()`、すべての転送の
アドレス・速さ（100 kHz と `i2c_configure()` の後の値）・メッセージの形、単発取得の読むレジスターと換算
（`lsm6dso_cal.c` の結果と物理量、`val1`・`val2` の符号、1 軸と 3 軸）、属性（ODR の段、m/s² と rad/s の
丸め、受け付けない値とチャネル、`-EBUSY`）、FIFO の開始・読み出し・一時停止・停止と統計、タイムスタンプの
読み取り、`lsm6dso_cal_zero()` です。FIFO のストリームも、`lsm6dso_stream.c`・`lsm6dso_fifo_irq.c`・
`lsm6dso_compat.c`・デコーダーと互換層（ワーク、RTIO、`gpio0`、既定の iodev とデコーダー）を、偽のカーネル
（`nuttx/tests/zephyr_fakes/`。1 スレッドで、ワークキューとタイマーはテストが回す）と、偽のセンサーの
ウォーターマークに従う偽の INT1 で動かし、ストリームの開始（レジスター、INT1 の割り込み）、INT1 のエッジと
20 ms の見直しでのバッチ、2 本目（`-EBUSY`）とほかのトリガー（`-ENOTSUP`）、穏やかな停止（一時停止、最後の
端数のバッチ、`-ECANCELED`、レジスターの戻り）、キャンセル、オーバーラン（`-EOVERFLOW`）、RTIO の
ワークキューでの単発取得とデコーダーを確かめます。

`nuttx/tests/zephyr_rtio.c` と `zephyr_work.c` も NuttX 版だけのテストで、互換層の RTIO とワークを偽の
カーネルの上で確かめます。前者は `imu_stream.c` と同じ寸法のコンテキストと偽の iodev で、メモリープール
（ブロックの大きさ、`max` から `min` への割り当て、送信への結び付け、最初の適合、枯渇と解放）、送信のプール
（4 つ、解放順の再利用、ハンドルと世代）、完了（順序、16 個、あふれた完了の `xcqcnt` とそのバッファーが残ること、
`consume_sem`）、multishot（完了の前の再投入、結び付けの解除、ハンドルの維持、失敗とキャンセルでの解放）、
キャンセル（印、iodev の関数、古いハンドルと無効なハンドル、送信前のキャンセルは完了を出さない）、RTIO の
ワークキューを確かめます。後者は `submit`・`schedule`・`reschedule`・`k_work_cancel_sync()` の戻り値とフラグ、
`K_MSEC()` の tick への換算を確かめます。

NINA の Wi-Fi・ソケットを対象にするテストは、それらを移植するときに加えます。
