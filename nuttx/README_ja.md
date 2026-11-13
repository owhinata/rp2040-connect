# NuttX 版

[English](README.md) | 日本語

`zephyr/app/usb-console` の IMU 測定ファームウェアを NuttX 13.1.0 に移植したものを置くディレクトリです。
現在は移植中です。今あるのは、環境構築、OS に依存しない共通ソースのホストテスト、ボード定義とその
ビルド、USB コンソールと書き込みスクリプトです。ファームウェアは起動して USB コンソール（CDC ACM）に
nsh のプロンプトを出すところまでで、測定の機能はまだありません。

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
| `main`（NuttX は init。起動処理の後シェルの優先度に下げる） | 0 | 118 | 2048 |
| シェル（nsh のセッション、コマンド） | 14 | 104 | 3072 |

- Zephyr の協調スレッド（負の優先度。`sysworkq`、`net_mgmt` など）は 118 より上になりますが、NuttX には
  他のスレッドに横取りされない協調スレッドが無く、より高い優先度のスレッドには横取りされます。
  今の移植では使っていません。
- `CONFIG_PRIORITY_INHERITANCE=y` です。ミューテックスは優先度継承あり、セマフォは既定で継承なし
  （NuttX 13.1.0 の `sem_init()` は flags を 0 にします）なので、通知用のセマフォはそのままで継承しません。
- RP2040 の NuttX には割り込み用のスタックが無く、割り込みは実行中のスレッドのスタックで動きます。
  各スタックには割り込みの分の余裕が要ります（何もしていないときはアイドルスレッドのスタックです）。
- スタックの番兵（`CONFIG_NANO_STACK_SENTINEL`。ボードの `Kconfig`、既定で有効）: Zephyr 版の
  `CONFIG_STACK_SENTINEL` と同じく、各スタックの最下位のワード（`CONFIG_STACK_COLORATION` が書いた値）を、
  スレッドの切り替えのとき（切り替えで外れるスレッド）と、割り込みハンドラーの出口ごと（割り込まれた
  スレッド。ハンドラーはそのスタックで動いた）に確かめ、書き換わっていれば PANIC で止めます（報告は
  RAM ログに入ります）。スケジューラーの計測用のフック 2 つ（切り替えの `sched_note_suspend()` と、
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

OS に依存しない共通ソースのテストを、PC 上でビルドして実行します。ボードもセットアップスクリプトも
不要で、ホストの `cc`（C17）だけを使います。FIFO ワードの解析、バッチとペイロード、送信キュー、
status と info の JSON、時刻の対応付け、センサーの tick と時計の対応、NTP の推定、FIFO パケットの復号、物理単位への
換算と校正値の保存形式、静止中のジャイロのゼロ点の追従、ボードの時間の換算（`nano_timeconv`）、
CPU 負荷の計算（`nano_cpuload`）、RAM ログのリング・`dmesg` の取り出し・行の前置き・頻度制限（`log_ram`）を
確かめます。テストは 1 つずつビルドして
実行し、失敗したところで止まります。ビルドの生成物は一時ディレクトリに置き、終了時に消します。

次のファイルは Zephyr 版（`zephyr/app/usb-console`）の同名ファイルを変更せずに写したものです。
Zephyr 版を直したときは、写しも同じ内容に置き換えます。

| 置き場所 | Zephyr 版の置き場所 | 内容 |
|---|---|---|
| `nuttx/apps/imu/` | `src/` | `imu_raw_{batch,packet,queue,status,time}`、`imu_tick_clock`、`time_offset`、`imu_cal_codec.c`、`imu_cal.h` |
| `nuttx/drivers/lsm6dso/` | `drivers/lsm6dso/` | `lsm6dso_{fifo,cal,zero,fifo_decoder}`、`lsm6dso_fifo_packet.h` |
| `nuttx/tests/` | `tests/` | 上のファイルを対象にするテスト |
| `nuttx/tests/fakes/` | `tests/fakes/` | テスト用の Zephyr のヘッダーの代わり（`zephyr/drivers/sensor.h`） |

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

ドライバーの I2C・FIFO 制御・ストリーム処理と NINA を対象にするテストは、それらを移植するときに
加えます。
