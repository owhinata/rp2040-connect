# NuttX 版の設計と動作の説明

[English](nuttx-porting.md) | 日本語

このリポジトリーのファームウェアは、Zephyr・NuttX・Pico SDK + FreeRTOS の 3 つの OS の上でそれぞれビルドでき、それぞれを
Zephyr 版・NuttX 版・FreeRTOS 版と呼びます。この文書は、NuttX 版の各機能の動作の詳しい説明です。使い方は `nuttx/README_ja.md`、
ドライバーと互換層は `nuttx/drivers/README_ja.md`、移植の経緯と判断は `nuttx/PORTING_STATUS.md`・
`nuttx/PORTING_NOTES.md`・`nuttx/PORTING_PLAN.md` にあります。文中の「〜の節」は、この文書の節です
（「Zephyr 版とのコマンドの対応」「エラー番号（errno）」は `nuttx/README_ja.md` の節）。

## 環境構築（`setup.sh`）の動作

取得するものの表は `nuttx/README_ja.md` の「環境構築」にあります。表に書いていない扱い:

- `nuttx/pico-sdk` は、boot2 のビルドが読むものと、ボードがビルドする除算器のルーチン（`divider_hardware.S` と、それが読む
  `divider_helper.S`。`CONFIG_RP2040_PICO_DIVIDER`）だけを sparse checkout で取得します。以前の部分取得のクローンに足りない
  パスは sparse checkout に足し、そのときファイルを取得します（blob を後から取る部分取得なので、ネットワークが要ります）。
  全体を取得済みのものがあればそのまま使います。部分取得の小さいものにするには、一度削除してから `setup.sh` を
  読み込み直します。
- argtable3 のバージョンは nuttx-apps 13.1.0 の `CONFIG_SYSTEM_ARGTABLE3_VERSION` の既定です。nuttx-apps の Makefile は
  `nuttx/nuttx-apps/system/argtable3/argtable3` にクローンがあれば使い、無ければビルドのたびに GitHub から取得し、
  `make distclean` で消します。

あわせて次のことを行います。

- `nuttx/nuttx/external` → `nuttx/drivers`、`nuttx/nuttx-apps/external` → `nuttx/apps` の symlink を
  張ります。NuttX はここから、このリポジトリのドライバーとアプリをビルドします。
- `nuttx/patches/nuttx/*.patch` と `nuttx/patches/apps/*.patch` を、それぞれ `nuttx/nuttx` と
  `nuttx/nuttx-apps` に名前の順に適用します（`nuttx/README_ja.md` の「NuttX へのパッチ」）。適用したものはクローンの
  `.git/local-patches/` に記録し、同じものは当て直しません。パッチを足す・消す・書き換えると、次に
  読み込んだときに、記録にあるパッチを逆順に外してから今のパッチを当て直します。記録がないのに
  クローンに変更があるときは何もせずに止まるので、`git -C nuttx/nuttx checkout -- .`（`nuttx-apps` も
  同様）で戻してから読み込み直します。
- 現在のシェルに `PICO_SDK_PATH` を設定し、`PATH` にツールチェーンの `bin`、`nuttx/.venv/bin`、
  `nuttx/script` を加えます。

`nuttx/script/kconfig-tweak` は、NuttX の `tools/configure.sh`（から呼ばれる `tools/sethost.sh`）が
`.config` の書き換えに使う `kconfig-tweak` の代わりです。kconfiglib には含まれていないため、
NuttX が使う機能だけを実装しています。

取得したものは Git の管理外です。バージョンを変えるときは、スクリプト冒頭の値を直してから、該当する
ディレクトリを消して読み込み直します。

## ビルド（`build.sh`）の動作

スクリプトは次のことを行います。

1. まだ configure していないとき、または前回の configure から defconfig、ボードの `Kconfig` と
   `scripts/Make.defs`、`nuttx/drivers/` と `nuttx/apps/` の下の `Kconfig`、アプリのディレクトリの
   構成、パッチ、NuttX と apps の commit のどれかが変わったとき（`nuttx/build/.configure-stamp` に
   記録）は、NuttX を configure します（`tools/configure.sh -l -a ../nuttx-apps`）。NuttX はボードの
   `Kconfig` を読み込み `nuttx/apps/Kconfig` を生成するのを configure のときにしか行わず、
   `scripts/Make.defs` にある boot2 のフラグが変わっても make は boot2 を作り直さないため、
   `make distclean` からやり直します。それ以外のときは差分ビルドです。
2. `nuttx/nuttx` で `make` を実行します。
3. イメージを検査します。`.boot2` が 0x10000000 にあって 256 バイトであること（`PICO_SDK_PATH` が
   無いと、NuttX はエラーを出さずに boot2 の無いイメージを作ります）、`nuttx.bin` が ELF の
   flash に載せる範囲とちょうど一致すること、イメージが 0x10ffe000 より前で終わることです。
   flash の末尾 8 KB（0x10ffe000〜0x10ffffff）は、Zephyr 版と同じく KV のためにイメージから
   外してあります。`CONFIG_RP2040_ROM_MEM_OPS` のときは、`__start()` が `memcpy`・`memset` を呼ばないことも
   確かめます（「C ライブラリの `memcpy`・`memset`」の節）。`CONFIG_RP2040_PICO_DIVIDER` のときは、6 つの `__wrap___aeabi_*` が
   あり、libgcc の割り算（`__aeabi_idiv`、`__divsi3`、`__udivmoddi4` など）がイメージに残っていないことを確かめます
   （「整数の割り算」の節）。`script/check_ram_code.py` が、RAM のコードをリンカースクリプトの一覧と照らし、残るヒープを
   確かめます（「RAM に置くコード」の節）。
4. `nuttx/nuttx/nuttx.bin`（0x10000000 からの生のイメージ。boot2 を含む）を
   `nuttx/script/bin2uf2.py` で `nuttx/build/nuttx.uf2` に変換し、パスとサイズを表示します。
   変換の前に boot2 のチェックサムを確かめ、256 バイトのペイロードと RP2040 のファミリー ID
   （0xe48bff56）で書き出します。NuttX 自身の UF2 生成（picotool）は切ってあります。

どの段階でも、失敗するとエラーで止まります。

`--menuconfig` で変えた内容は、次に configure するまで `nuttx/nuttx/.config` にだけ残ります。
残すには `--savedefconfig` で保存してください。ほかの生成物は、ELF が `nuttx/nuttx/nuttx`、生の
イメージが `nuttx/nuttx/nuttx.bin` です。NuttX が `nuttx/drivers/` と `nuttx/apps/` に書き出すものを
含め、ビルドの生成物は Git の管理外です。

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
- Wi-Fi に繋がっている間は、同じシェルを Telnet でも使えます（「Wi-Fi 経由のシェル（Telnet）」の節）。
- `help` でコマンドの一覧が出ます。前半が nsh のコマンド（`uname`、`ps`、`free`、`echo`、`sleep`
  など）、`Builtin Apps:` の後がこの移植のコマンドと NuttX のアプリ（`wapi`、`iperf` など）です。
  Zephyr 版のコマンドとの対応は「Zephyr 版とのコマンドの対応」の節です。存在しないコマンドを打つと
  `nsh: <名前>: command not found` と出ます（Zephyr 版は `<名前>: command not found`）。
- `reboot` でボードを再起動します（Zephyr 版の `kernel reboot`）。引数を付けても（`reboot --help` でも）
  再起動します。
- `bootsel` で BOOTSEL（USB マスストレージの `RPI-RP2`）に再起動します（FreeRTOS 版と同じ。ブート ROM の
  `reset_usb_boot(0, 0)`）。確認の語はなく、引数は無視します。書き込みのときに RESET ボタンを 2 回押す
  代わりに使えます。
- `irqstack` は割り込み用のスタックの起動からの最大使用量を 1 行で出します（Zephyr 版の
  `kernel thread stacks` の `IRQ 00` の行と同じ書式。「スレッドの優先度とスタック」の節）。
- Wi-Fi は NuttX 標準の `wapi` と `ifconfig` で扱います（「Wi-Fi（`wapi`、`ifconfig`）」の節）。資格情報の
  保存は Zephyr 版と同じ `wifi cred` です（「Wi-Fi の資格情報と自動再接続（`wifi cred`）」の節）。
- ログは RAM に置き、Zephyr 版と同じくコンソールには出しません。`dmesg` で表示します（「ログ」の節）。
- シェル（nsh のセッション）と、そこから起動するコマンドは優先度 104 で動きます（Zephyr 版のシェルの
  スレッドと同じ位置。「スレッドの優先度とスタック」の節）。

UART0（GPIO0 が TX、GPIO1 が RX。Arduino の D1/D0）はコンソールではありません。Zephyr 版と同じく
ピンは UART に割り当てますが、何も出力しません。

## ログ（`dmesg`）

Zephyr 版の `src/log_ram.c` と同じ RAM ログで、FreeRTOS 版と共有します（`common/compat/log/log_ram.c`。リング・行の前置き・頻度制限は
`common/compat/log/log_ring.c`、NuttX の部分は `nuttx/drivers/include/log_ram_os_impl.h`（リングのセクション、割り込みの禁止、TIMER）と
syslog のチャネル `nuttx/drivers/log/log_ram_syslog.c`。`LOG_*` マクロは `common/compat/log/app_log.h` で、その NuttX の部分が
`nuttx/drivers/include/app_log_os.h`）。

- 8192 バイトのリングバッファーを `.noinit` に置きます（`nuttx/drivers/include/app_log_os.h` の `LOG_RAM_SIZE`。
  Zephyr 版と FreeRTOS 版は 4096 バイト。異常停止の報告が先頭から収まるように広げました）。起動処理はこの領域を消さないので、`reboot` などの
  リセットをまたいで内容が残ります。起動時に先頭の情報（マジックナンバーとその反転、開始位置、長さ）を
  確かめ、壊れていれば空にします。マジックナンバーは Zephyr 版の `0x4c4f4752` に大きさを XOR した値
  （`0x4c4f6752`）で、大きさの違うリング（FreeRTOS 版の 4096 バイトのもの `0x4c4f5752`、Zephyr 版のもの）が RAM に残っていても、
  読み違えずに空にします。電源を切ると消えます。いっぱいになると古いバイトから上書きします。
- `dmesg` は保存されている内容をそのまま表示し、空なら `No logs.` と出します。引数は見ません。
  Zephyr 版と同じく 96 バイトずつ取り出して表示し、表示中に新しいログで押し出された部分は飛ばし、
  合わせてリングの 2 倍（16384 バイト。Zephyr 版は 8192 バイト）で打ち切ります。NuttX 標準の `dmesg`（`/dev/kmsg`）は外してあります。
- 行の形式は Zephyr 版のログと同じ `[時:分:秒.ミリ秒,マイクロ秒] <inf> モジュール名: 本文` で、改行は
  LF だけです。レベルは `<err>`、`<wrn>`、`<inf>`、`<dbg>` で、`<dbg>` では `モジュール名.関数名: ` に
  なります。時は 2 桁以上で、99 時間を超えると桁が増えます。時刻はボードの単調時計（TIMER、
  起動からの µs）で、再起動すると 0 から数え直します（Zephyr 版と同じ見え方）。
- `syslog()` の出力（カーネルや nsh のメッセージ、PANIC の報告）も同じリングに入ります。Zephyr 版の
  `printk` と同じく、前置きを付けずにそのまま入れ、改行は CR LF にします（syslog の
  `CONFIG_SYSLOG_CRLF`。Zephyr 版の printk の文字列と同じ）。
- 異常停止（HardFault などの例外、`PANIC()`、assert の失敗、スタックの番兵）のとき、NuttX 版は報告
  （原因、レジスター、スタック、タスクの一覧）を RAM ログに書いてから自動で再起動します
  （`CONFIG_BOARD_RESET_ON_ASSERT=2`。再起動は `reboot` と同じ通常の起動で、BOOTSEL にはなりません）。
  RAM ログは再起動しても残るので、起動後の `dmesg` で報告を読めます。報告は、原因の行
  （`Assertion failed` と理由、ファイルと行、止まったタスク）、レジスター（PC・LR・SP を含む）、止まった
  タスクのスタック（と割り込み中なら割り込み用のスタック）の base と size、SP がスタックの外なら
  `Stack pointer ... is not within the stack`、タスクの一覧（各スタックの使用量。`irq` の行は割り込み用の
  スタック）です。スタックの内容（16 進のダンプ）は入れません（`CONFIG_SCHED_DUMP_STACK` を切ってある）。
  入れると長くなり、リングから報告の先頭（原因の行とレジスター）が押し出されるためです。
  NuttX の `CONFIG_ARCH_STACKDUMP_MAX_LENGTH` は SP がそのスタックの外のときのダンプにしか効かないので、
  この目的には使えません。Zephyr 版も致命的なエラーの後に再起動する設定です
  （`CONFIG_RESET_ON_FATAL_ERROR=y`。Zephyr 版の RAM ログは 4 KiB）。
- ソースからは Zephyr と同じ書き方で使います。`#include "app_log.h"`（`common/compat/log/`。
  カーネル側・ボード・アプリのどれからも使え、割り込みハンドラーからも呼べます）のあと、ファイルごとに
  `LOG_MODULE_REGISTER(名前, LOG_LEVEL_INF)`（レベルを省くと `CONFIG_LOG_DEFAULT_LEVEL`、既定 3。
  include の前に `LOG_LEVEL` を定義していればその値）、本文で `LOG_ERR()`、`LOG_WRN()`、`LOG_INF()`、
  `LOG_DBG()`、`LOG_ERR_RATELIMIT()` などを書きます。ファイルのレベルより上のメッセージはコンパイル時に
  捨てます。実行時のレベル変更（Zephyr の `log` コマンド）はありません。`<syslog.h>` の `LOG_ERR`
  （syslog の優先度）は同名のマクロで置き換えるので、`app_log.h` を読み込んだファイルでは
  `syslog(LOG_ERR, ...)` の代わりに `APP_LOG_SYSLOG_ERR` を使います。
- 1 行は呼び出したスレッド（または割り込み）の中で、そのスタック上の 192 バイトのバッファーに整形して
  から、割り込みを禁止してリングに書きます。本文が長いと 192 バイトに収まるところで切れます。
  Zephyr 版の既定（deferred）では、呼び出し側は記録するだけで、整形は優先度の低いログスレッドが後で行います。

## キーと値の保存（`kv`）

設定と校正値、Wi-Fi の資格情報を flash の末尾 8 KiB（`0x10ffe000`〜`0x10ffffff`、4 KiB の消去単位 2 つ）に
保存します。Zephyr 版の `storage_partition` と同じ場所に、Zephyr 版と同じ形式（Zephyr の NVS と settings）で
置くので、Zephyr 版が書いた値を NuttX 版がそのまま読み書きでき、その逆もできます。API とコマンドは Zephyr 版の
`kv_store.c` そのもの（`common/imu/kv_store.c`。3 つの OS のファームウェアが使う）です。

- 構成: `kv_store.c` → Zephyr の settings（`settings_save_one()`・`settings_load_one()`・`settings_delete()`）→
  Zephyr の NVS → 互換層の flash デバイスと flash map（`common/compat/zephyr_flash_kv.c`）→ OS の境界
  （`common/compat/flash_kv_os.h` の 5 関数。RP2040 では FreeRTOS 版と共有の `common/chip/rp2040/rp2040_flash_kv.c` が、ボードのブート ROM での flash の操作 `src/rp2040_flash_kv_os.c` の上で実装）。settings と NVS は
  Zephyr v4.5.0 のソースを改変せずに写したもの（`common/compat/subsys/` ほか。一覧はドライバー説明の互換層の
  節）で、Zephyr 版の Kconfig の値（`common/compat/zephyr_kv_config.h`）でビルドします。
- 形式（Zephyr 版の `.config` とデバイスツリーと同じ値）: NVS のセクターは 4096 バイト × 2
  （`CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT=1`、`CONFIG_SETTINGS_NVS_SECTOR_COUNT=8` のうち 8 KiB に収まる 2）、
  書き込み単位は 1 バイト、消去値は 0xFF。NVS の索引（ATE）は 8 バイト（id、offset、len、part、CRC-8）で、
  セクターの末尾から下へ、データは先頭から上へ書きます。settings は、ID 0x8000 に使っている最大の名前の ID、
  名前を 0x8001 から（終端なし）、値を名前の ID + 0x4000 に置きます。名前は `app_kv/<キー>`（`kv` のキー）と
  `wifi_cred/<番号>`（資格情報）。値のバイト列も Zephyr 版と同じです（校正値は 56 バイト、`imu.port` などは
  文字列、`imu.session` は 4 バイトのビッグエンディアン、資格情報は `struct wifi_credentials_personal` の
  224 バイト。この構造体の並びが Zephyr 版の ELF と同じことを、互換層の `<zephyr/net/wifi_credentials.h>` の
  `BUILD_ASSERT` で確かめます）。
- 領域: ボードの `include/board.h`（`BOARD_KV_OFFSET`、`BOARD_KV_SIZE`）、`flash_kv.h` の定数とリンカー
  スクリプト（`scripts/arduino-nano-rp2040-connect-flash.ld` の `__kv_start`、`__kv_end`。この領域はイメージの flash 領域から外して
  あり、`build.sh` もイメージの終端を確かめます）で決まります。読み出し・書き込み・消去のたびに、対象が
  この範囲に収まることを、互換層（定数）、RP2040 の共有の部分（定数と、起動時に一致を確かめたリンカー
  スクリプトの値。`board.h` の値はボードのファイルのコンパイル時に定数と比べる）、ボードの RAM 上の関数（ブート ROM を呼ぶ直前）の 3 か所で確かめてから flash に触れます。範囲の外なら
  何もせずエラーにします（`dmesg` に `<err> flash_kv: Refused flash ...`）。Zephyr 版の flash ドライバーは
  flash 全体を受け付けますが、NVS が触れるのはこの範囲だけです。
- 起動: ボードの起動処理（`rp2040_bringup()`、LSM6DSOX と NINA の後）は flash の操作の準備（リンカー
  スクリプトの確認、ブート ROM の関数、boot2 の写し）だけをします。NVS のマウントは、Zephyr 版と同じく
  settings の最初の呼び出し、つまり init が Zephyr の `SYS_INIT()` と同じ位置で呼ぶ資格情報の初期化の中です。
  そのとき `dmesg` に Zephyr 版と同じ 3 行が出ます（`<inf> fs_nvs: 2 Sectors of 4096 bytes`、
  `alloc wra: <セクター>, <位置>`、`data wra: <セクター>, <位置>`）。続いて init が Zephyr 版の `main()` と同じ
  位置で `kv_store_init()`（`settings_subsys_init()`）を呼びます（失敗すると
  `Could not initialize settings: <値>`）。
- 別のデータがあるとき: マウントの前に（起動後の最初の settings の呼び出しで 1 回）、領域が Zephyr の NVS か
  （どちらかのセクターの最後の 2 つの索引の場所に、NVS が書く正しい索引がある）、全面が消去済みかを
  確かめます。どちらでもなければ（NuttX 版の以前の形式、OpenMV 版などの別のファームウェアの残り）、領域を
  消さずに `dmesg` に次の 2 行を出し、settings を初期化しません。

  ```text
  <err> flash_kv: Key-value area 0xffe000-0xffffff holds the NuttX build's earlier format
  <err> flash_kv: Not used; "kv wipe confirm" erases it (all keys and Wi-Fi credentials)
  ```

  （以前の形式でないものは 1 行目が `holds data that is not Zephyr's NVS`。）`kv` の操作は `-EILSEQ`（NuttX の値は
  84。`KV read failed: -84`）を、資格情報の保存は `-ENOENT` を返し、何も書きません。`kv wipe confirm` で領域を
  消すと、その場で空の NVS をマウントし直します（`<wrn> flash_kv: Key-value area erased` と NVS の 3 行）。
  読み込み済みの校正値と資格情報の SSID の一覧は再起動まで RAM に残ります。Zephyr 版は同じ状況で、Zephyr の
  NVS のとおりに動きます: 2 つのセクターの末尾 8 バイトがどちらも書かれていればマウントが `-EDEADLK` で失敗し
  （`CONFIG_NVS_INIT_BAD_MEMORY_REGION` が無効）、KV を使えません。どちらかが消去済みなら、中断した
  ガベージコレクションとみなして（`<inf> fs_nvs: No GC Done marker found: restarting gc`）2 つのセクターを
  消し、空の KV で始めます。以前の NuttX 版の形式は、書き込み中のブロックの末尾が消去済みなので、ふつう
  後者です（偽の flash の上で確かめた。実機では確かめていない）。
- コマンド（優先度 104、スタック 3072。Zephyr 版の `SHELL_CMD_REGISTER(kv, ...)` を `common/shell/zshell.c` で
  動かす。`nuttx/apps/kv` の `main()` は `z_shell_main_kv()` を呼ぶだけ）:

  ```
  kv set <キー> <文字列>       文字列をそのまま保存
  kv sethex <キー> <16 進数>   1〜128 バイトを 16 進数の対で保存（大文字も可）
  kv get <キー>                値を小文字の 16 進数で表示
  kv delete <キー>             削除（無いキーでも Deleted）
  kv wipe confirm              KV の領域全体を消去（NuttX 版だけ。confirm が無いと消さない）
  ```

  キーは英数字・`_`・`-`・`.` の 1〜32 文字、値は 1〜128 バイトです。`sys.` で始まるキーは
  `Reserved key` になります。出力は Zephyr 版と同じです（`Saved`、`Deleted`、`KV write failed: -22`、
  `KV read failed: -2`、`Expected 1-128 bytes as hexadecimal pairs`、`Invalid hexadecimal value` など）。
  `kv` だけ、または知らないサブコマンドではヘルプを、引数の数が違うと `<サブコマンド>: wrong parameter count`
  とサブコマンドのヘルプを出します（Zephyr のシェルと同じ文）。ヘルプには `wipe` の行が加わります。
  `kv wipe` に `confirm` 以外の語を付けると
  `Type "kv wipe confirm" to erase all keys and Wi-Fi credentials` を出して何もしません。成功すると `Erased`。
  Zephyr 版と違う点: 引数は nsh が解釈する（`$`、`;`、`#`、`>` などは引用符で囲む）、エラーの行は標準エラー
  （同じコンソール）に出る、終了状態は成功で 0・失敗で 1、`kv wipe` がある。
- 容量: 生きている項目はすべて 1 つのセクター（4 KiB）に収まる必要があります（もう 1 つは整理用）。1 項目は
  名前の項目（`app_kv/` + キー、索引 8 バイト）と値の項目（値、索引 8 バイト）の 2 件で、32 文字のキーと
  128 バイトの値なら 183 バイトです。満杯の次の書き込みは `KV write failed: -28`。同じ値を保存し直しても書き
  込みません（NVS が比べる）。Wi-Fi の資格情報は 1 件が 11 + 8 + 224 + 8 = 251 バイトで、2 件まで同じ領域を
  使います。
- 書き換えと整理: 書き換えは新しい項目（値の索引）を足すだけで、古い項目には触れません。書き込み中の
  セクターが埋まると、それを閉じ、もう 1 つのセクターの生きている項目を写してから、そのセクターを消去します
  （NVS のガベージコレクション。消去は 1 回の書き換えの中で同期して行う）。flash を消去・書き込みする間は
  XIP が止まるので、RAM 上のコードで割り込みを禁止して行います。割り込みを禁止するのは、4 KiB の消去 1 回
  （実測 34〜40 ms）と 256 バイトのページの書き込み 1 回（0.4〜0.5 ms）のそれぞれの間です。既存のキーを
  書き換えると、ページの書き込みはふつう 2 回（データと索引。ページ境界をまたぐと 3 回）です。Zephyr 版の
  flash ドライバーも、消去は 1 回の呼び出し（4 KiB）ごと、書き込みは 1 回の呼び出し（1 項目のデータか
  1 つの索引）ごとに割り込みを禁止します。
- 計測中の書き込み: 計測中に KV を書くのは、シェルから `kv set`・`kv wipe`・`wifi cred add` などを打ったとき
  だけです（`imu start` の `imu.session` の保存は、取得を始める前）。そのときガベージコレクションが走れば、計測中に
  4 KiB の消去（割り込み禁止 34〜40 ms）が 1 回起こります。消去は書き込み中のセクターが埋まるたびに 1 回で、
  `imu.session` の書き換え（値 4 バイト + 索引 8 バイト）なら約 300 回に 1 回です（以前の NuttX の NVS は
  1 回に約 38 バイトを使い、約 100 回に 1 回）。実機では、計測中（6667 Hz）のこの消去 1 回で
  センサーの FIFO があふれ、オーバーランからの回復が 1 回走りました（2026-11-17、`kv set` 260 回の間に 1 回。
  受信側の欠番は 0、エポックが 1 つ進む）。読み出し（`kv_store_get()`、資格情報の読み込み）は書きませんが、
  settings が書きかけの項目（電源断の後に残った名前だけ・値だけの項目）を見つけると、その場で消す書き込みを
  します（Zephyr 版と同じ）。
- 電源断: NVS はデータを書いてから CRC 付きの索引を書き、起動時に、書きかけの項目や途中で止まった整理を
  直します。ホストテスト（`common/tests/kv_settings.c`）が、整理を何度も通る書き換えの列の、flash の書き込みと
  消去のすべて（652 回）のそれぞれで電源を切り（ページの半分だけ書く、セクターの半分だけ消す）、次の起動で
  すべてのキーが書き換えの前か後の値を持つことを確かめます。実機で電源を切る試験はしていません。
- API（`common/imu/kv_store.h`、Zephyr 版と同じ）: `kv_store_init()`・`set`・`get`・`delete` は Zephyr 版と同じ
  引数・戻り値です（キーや値が不正なら `-EINVAL`、無いキーは `-ENOENT`、バッファーが小さければ `-ENOBUFS`、
  いっぱいなら `-ENOSPC`、flash の準備ができていなければ `-ENODEV`、別のデータがあれば `-EILSEQ`）。`kv_store_*()` は
  毎回 settings の初期化を確かめてから（未初期化なら初期化し直して）動きます（Zephyr 版と同じ）。排他は settings のミューテックスと
  NVS のミューテックスで（Zephyr の `k_mutex`、互換層では NuttX の再帰ミューテックス、優先度継承あり）、
  どのスレッドからも呼べます。

### コマンドのサブコマンド

`kv`・`nina` など、この移植のサブコマンドを持つコマンドは、共通の部品 `common/shell/subcmd.c`
（`subcmd_main()`。ビルドは `nuttx/apps/subcmd/`）で、Zephyr のシェルと同じ出力にしています。Zephyr の `SHELL_CMD()`（引数の数を検査しない）は
`SUBCMD()`、`SHELL_CMD_ARG()`（必須と省略可の数）は `SUBCMD_ARG()` で表します。

- コマンドだけ、または知らない語: 一番深く一致したコマンドのヘルプ（`<名前> - <説明>`、`Subcommands:`、
  各サブコマンドの行）。
- 引数の数が違う: `<サブコマンド>: wrong parameter count`（標準エラー）とそのサブコマンドのヘルプ。
- サブコマンドの直後の語に `*` か `?` があり、どのサブコマンドにも一致しないときは、Zephyr のシェルと
  同じくそこで語の読み取りを止めます（`kv get a*` は引数の数の誤り、`nina version *` は `nina version`）。
  一致するサブコマンドがあるとき（Zephyr のシェルは名前に展開する）は再現せず、知らない語として扱います。
- `-h`・`--help` はふつうの語です（Zephyr 版と同じ）。
- ヘルプの文は Zephyr のシェル（`shell_help.c` の `formatted_text_print()`）と同じに並べます: 文の中の改行と、
  80 桁を越える語の前で行を改め、続きの行は文の始まりの桁から書きます（Zephyr のシェルは VT100 の
  カーソル移動で、この移植は空白で桁を合わせます）。文の末尾の改行は、空の行を 1 行残します（Zephyr と同じ）。
  端末の幅を変える `resize` はありません。
- 終了状態は、成功で 0、失敗・ヘルプ・引数の誤りで 1 です。

`sensor`・`imu`・`ntp`・`i2c_dma`・`i2c` は Zephyr 版と同じソース（`common/`）のコマンドの表なので、同じ部品の `zshell.c`
（`z_shell_cmd_main()`）で動きます。規則は上と同じで、加えて Zephyr のシェルと同じく、動的なサブコマンド
（デバイス名、チャネル名など）、`SHELL_HELP()` の説明と `Usage:` の行、一致するサブコマンドがあるワイルドカードの
展開（その後にハンドラーのあるサブコマンドが続くと `Error: requested multiple function executions`）、30 語
（`CONFIG_SHELL_ARGC_MAX`）を越える行（`Too many arguments in the command.`）を扱います（「センサーのコマンド
（`sensor`）」の節）。ハンドラーには、コマンドの端末のバックエンド（`struct shell`）が `sh` として渡り、
`shell_print()` などはコマンドの標準出力・標準エラーに書きます。

コマンドの登録（`SHELL_CMD_REGISTER()`）は、互換層の `<zephyr/shell/shell.h>` が組み込みコマンドの入口にします。
そのファイルがアプリの main のソース（`MAINSRC`）なら入口は `main()` です（`sensor_shell.c`、`imu_stream.c`）。
main のソースでないファイル（`time_sync.c`。`ntp` は `imu` と別のアプリ）は、ビルドが `Z_SHELL_CMD_NAMED_MAIN` を
付けてコンパイルし、入口は `z_shell_main_<名前>()`（`z_shell_main_ntp()`）になります。アプリの `main()`
（`nuttx/apps/ntp/ntp_main.c`）がそれを呼びます。

## NINA-W102（`nina`）

Wi-Fi モジュール NINA-W102 とは SPI1 でやり取りします。NINA のファームウェアは Zephyr 版と同じもの
（Arduino の NINA ファームウェア 3.0.2 に修正を当てたもの、`firmware/nina-fw/README.md`）で、書き換えません。
ドライバーは、コマンドを 1 つ送って応答を受け取る転送層（`nina_proto_request()`、`nina_proto_ip_config()`）、
`nina version`、ソケット（NuttX の usrsock。socket・close・bind・listen・accept・connect・setsockopt と
送受信）、NINA の名前解決（`nina_dns_getaddrinfo()`）、Wi-Fi のネットワークデバイス `wlan0`
（スキャン、接続、切断、状態、接続の監視）からなります（`nuttx/drivers/README_ja.md` の「ソケット（usrsock）」
「名前解決」「Wi-Fi」）。資格情報の保存と自動再接続（`wifi cred`）は下の節です。

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

- SPI: モード 0（CPOL 0、CPHA 0）、8 ビット、MSB が先、`CONFIG_NINA_SPI_FREQUENCY`（既定 8000000。
  Zephyr 版と同じ選択肢）。PL022 は clk_peri（125 MHz）を偶数で割るので、実際は
  7.8125 MHz です（Zephyr 版も同じ値）。NuttX の SPI1 ドライバー（`rp2040_spi.c`）を `CONFIG_RP2040_SPI_DMA` 付きで
  使います: `CONFIG_RP2040_SPI_DMATHRESHOLD` バイト（63）を超える交換は DMA で、スレッドはその完了を期限なしに待ち、
  短い交換はポーリングです（Zephyr 版・FreeRTOS 版と同じ境目）。ドライバーは 2 つのチャネルを待たずに取ります
  （ローカルのパッチ `nuttx/0013`）。CS は SPI ドライバーではなく転送層が GPIO で動かし、
  応答を待つ間も保持します。SPI1 の `/dev/spi1` は作りません（`CONFIG_RP2040_SPI_DRIVER` なし）。
- NINA の初期化は Zephyr 版と同じく最初の要求のときに行います: CS を非アクティブ、READY を入力、IRQ を
  high、RESET をアクティブにし、10 ms 後にリセットを解き、750 ms 待ってから IRQ を low にして入力に戻します。
  最初の要求は、起動処理が `wlan0` を用意するときに MAC アドレスを読む命令です（Zephyr 版と同じく起動時）。
  そのため USB のコンソールが現れるのはこの待ちの後です。
- 要求の手順と値は Zephyr 版と同じです（3 つの OS のファームウェアが共有する `common/drivers/nina/internal/nina_proto.c`）。
  READY は 100 µs ごとに見る代わりに、その変化の GPIO 割り込みで待ちます（Zephyr 版・FreeRTOS 版も同じ。
  仕組みは `nuttx/drivers/README_ja.md` の「READY の待ち」）。期限は同じで、送信前は
  2000 ms、CS を下げてから NINA が READY を戻すまで 20 ms、応答の準備は 10000 ms で `-ETIMEDOUT`。
  応答の先頭（0xE0）は最大 1000 バイト探します。要求は再帰可能なミューテックス（優先度継承あり）で
  直列化し、呼び出したスレッドの優先度で SPI を動かします。

### 中継モード（`nina passthrough confirm`）

PC の esptool で NINA のファームウェアを読み書きするためのモードです（Arduino の SerialNINAPassthrough の
スケッチの代わり。3 つの OS のファームウェアで同じもので、NINA のドライバーの `common/drivers/nina/nina_passthrough.c`）。`nina passthrough confirm` でボードが再起動し、USB のシリアルが
NINA の UART に繋がります（確認の語がなければ断ります）。UART は SPI の MISO・CS と同じピン（GPIO8、GPIO9）なので、
このモードでは NINA の SPI・Wi-Fi・Telnet・MQTT・時刻同期・nsh・KV は動きません。

- PC の DTR と RTS: RTS が有効 → NINA のリセットを解く、DTR が有効 → NINA の GPIO0 を LOW（起動モード）。
  ポートを開く（両方有効）と ESP32 の ROM のダウンロードモードで起動し、閉じるとリセットで止まります。
  UART のレートは PC が設定したレートに追従します（8N1）。
- 抜けるには、ポートを 300 bps にして閉じる（`stty -F /dev/ttyACM0 300`）か、RESET ボタン。NINA を通常の
  起動に戻してから、通常のファームウェアで再起動します。フラッシュには何も書きません。
- 開始・ポートを閉じるたび・終了のときに、送受信のバイト数と取りこぼしの数を RAM ログに残します（抜けたあと
  `dmesg` で見られます）。
- PC 側の esptool の手順（`ESPTOOL_CFGFILE` の `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` と
  `--before default-reset --after no-reset`。`--before no-reset` では繋がらない）はルートの `README_ja.md` の
  「NINA のファームウェアの読み書き」です。実機で 2 MB 全体の読み出しを 115200 bps と 921600 bps で行い、一致しました
  （2026-11-18）。
- 仕組みは `nuttx/drivers/README_ja.md` の「中継モード」です。

### Wi-Fi（`wapi`、`ifconfig`）

NINA は NuttX のネットワークデバイス `wlan0` です。Zephyr 版の `wifi` のコマンドの代わりに、NuttX 標準の
`wapi`（`CONFIG_WIRELESS_WAPI_CMDTOOL`、優先度 104、スタック 3072）と `ifconfig` を使います。ioctl は、コアの NuttX への
つなぎ（`common/drivers/nina/port/nuttx/nina_netdev.c`）から NINA のコアの Wi-Fi の操作（Zephyr 版・FreeRTOS 版と共有する
`common/drivers/nina/nina_wifi.c`）を呼びます（`nuttx/drivers/README_ja.md` の「Wi-Fi」）。

| Zephyr 版 | NuttX 版 |
|---|---|
| `wifi connect -s <SSID> -p <パスフレーズ> -k 1` | `wapi psk wlan0 <パスフレーズ> 3` の後に `wapi essid wlan0 <SSID> 1` |
| `wifi connect -s <SSID> -k 0`（暗号化なし） | `wapi essid wlan0 <SSID> 1`（直前に `wapi psk` を打たない） |
| `wifi disconnect` | `wapi disconnect wlan0` |
| `wifi scan` | `wapi scan wlan0` |
| `wifi status` | `wapi show wlan0` と `ifconfig wlan0`（下） |
| `net iface` | `ifconfig` |
| `wifi cred add/delete/list/auto_connect` | 同じ（`wifi cred`。下の「Wi-Fi の資格情報と自動再接続（`wifi cred`）」） |

- `wapi psk` の 3 は暗号の CCMP、WPA のバージョンの既定は WPA2 で、Zephyr 版の `-k 1`（WPA2-PSK）に当たります。`wapi psk` の
  設定は次の `wapi essid` 1 回で使って忘れ、パスフレーズも消します。ドライバーが受け付けるのは Zephyr 版と同じく
  暗号化なしと WPA2-PSK（NINA のパスフレーズの命令）だけで、ほかは `ENOTSUP` です。パスフレーズは 8〜63 文字です。
- `wapi essid` は Zephyr 版の `wifi connect` と同じく、要求を出したところで戻ります。接続の手順と期限
  （500 ms ごとに最大 60 回、つまり約 30 秒で `ETIMEDOUT`）、接続後の省電力の解除と IP の設定は Zephyr 版と
  同じです。結果は `ifconfig wlan0` で確かめます（接続すると `RUNNING` が付き、`inet addr`・`DRaddr`・`Mask` に
  NINA の DHCP の値が出ます）。
- `wifi status` の項目の見え方: State は `ifconfig` の `RUNNING`（と `wapi show` の ESSID の Flag）、SSID は
  `wapi show` の ESSID、BSSID は AP、RSSI は Sense（接続中だけ）、Interface Mode は Mode（`WAPI_MODE_MANAGED`）。
  `wapi show` の SSID・BSSID・RSSI は Zephyr 版の `wifi status` と同じくその場で NINA に尋ねます。Band、Channel、
  Security、MFP、Link Mode、Beacon Interval、DTIM、TWT、PHY の速度は出ません（Zephyr 版でも NINA からは得て
  おらず、固定値か 0 です）。
- `wapi scan` は NINA のスキャン（最大 10 件）を、BSSID・周波数・信号の強さ（dBm）・暗号化（`8000` なし、
  `0800` あり、`ffff` 不明）・SSID の列で、NINA の順に出します。暗号化の判定は Zephyr 版と同じ NINA の種類の
  対応（7 なし、5 WEP、2 WPA、4 WPA2、ほかは不明）からで、`wapi` は WEP・WPA・WPA2 を区別して表示しません。
  周波数の列は MHz です（`iperf` が `CONFIG_LIBC_FLOATINGPOINT` を選ぶので、`printf` が浮動小数点を扱います。
  無いと `*float*` と出ます）。ESSID・パッシブ・チャネルを指定したスキャンは Zephyr 版と同じく受け付けません。
  接続中のスキャンは、Zephyr 版と同じく切断として扱い（IP の設定を消す）、終わった後で再接続を頼みます。
- スキャン中の接続、接続中のスキャンは Zephyr 版と同じく `EBUSY` です。
- `wapi` は失敗したときに何も表示しません（エラーの表示が NuttX のデバッグ出力 `wlerr()` で、この構成では
  無効）。終了状態は 1 になるので、`echo $?` で分かります。Zephyr 版の `wifi` のコマンドはエラーの値を表示します。
- `ifconfig wlan0 down`・`up` は何もしません（Zephyr 版のインターフェースも常に有効）。

### Wi-Fi の資格情報と自動再接続（`wifi cred`）

接続先の資格情報を保存し、起動時と接続が切れたときに自動で繋ぎ直します。資格情報の扱いは Zephyr の
ライブラリ（`subsys/net/lib/wifi_credentials/` と settings のバックエンド）の写し、コマンドは
`wifi_credentials_shell.c` の写し（`nuttx/apps/wifi/`）、自動再接続は Zephyr 版と同じファイル
`common/imu/wifi_profile.c` です。構文・出力・エラーの文・戻り値は Zephyr 版と同じです。

```
wifi cred add -s <SSID> [-k <種類>] [-p <パスフレーズ>] [-c <チャネル>] [-b <帯域>] [-w <MFP>] [-m <BSSID>]
                                     保存（同じ SSID は置き換え。成功すると何も出さない）
wifi cred delete <SSID>              削除（Deleting network ssid: ... と出す）
wifi cred list                       保存した順の番号で一覧（パスフレーズも表示する。Zephyr 版と同じ）
wifi cred auto_connect               保存した資格情報で接続を要求する（結果は待たない）
```

```text
nsh$ wifi cred add -s example-ap -k 1 -p example-pass
nsh$ wifi cred list
  network ssid: "example-ap", ssid_len: 10, type: WPA2-PSK, password: "example-pass", password_len: 12, MFP_OPTIONAL
```

- 件数と長さ: 2 件まで（`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES=2`、
  Zephyr 版と同じ名前と値）。3 件目は `Failed to add network credentials, err: -105`（`ENOBUFS`）で、
  `dmesg` に `<err> wifi_credentials: Cannot store WiFi credentials, no space left` が出ます（Zephyr 版と同じ）。
  SSID は 1〜32 文字（`SSID too long (max 32 characters)`、`SSID not provided`）、パスフレーズは 8〜64 文字
  （`Passphrase should be minimum 8 characters`、`Passphrase too long (max 64 characters)`）。ただし NINA が受け付ける
  パスフレーズは 63 文字までです（Zephyr 版と同じ）。
- 種類（`-k`）: Zephyr の番号（0 なし、1 WPA2-PSK、2 WPA2-PSK-SHA256、3 WPA3-SAE など。`-h` のヘルプ）を
  そのまま保存します。接続できるのは Zephyr 版と同じく 0 と 1 だけで、ほかの種類の資格情報は自動接続で
  飛ばされます。`-k` なしでパスフレーズを付けると `Passphrase provided without security configuration` と
  出して保存します。`-t` は Zephyr 版と同じく `Invalid option` になります（オプションの表にあるが処理が無い）。
- `wifi cred`・`wifi` だけ、または知らない語ではヘルプを、`add` の引数の数が 1〜13 語でなければ
  `add: wrong parameter count` とヘルプを出します（「コマンドのサブコマンド」の節）。`wifi` のサブコマンドは
  `cred` だけです（ほかは `wapi`・`ifconfig`）。
- オプションの解析は NuttX の `getopt_long()` で、Zephyr の `sys_getopt_long()` と違い、オプションでない語を
  後ろへ回しません（オプションでない語より後のオプションは読まない）。知らないオプションのときに Zephyr 版が
  ログ（`<wrn> sys_getopt`）に出す行もありません。
- 保存: 1 件ずつ settings の `wifi_cred/<番号>`（番号は空いている最小のもの）に、Zephyr の
  `struct wifi_credentials_personal` のまま書きます（Zephyr のライブラリの settings のバックエンドの写し。
  読み出しも Zephyr と同じく `settings_load_subtree_direct()`）。`add`・`delete` のたびにすぐ書きます。起動時に
  init が保存された SSID を読み込みます（Zephyr のライブラリの `SYS_INIT()` と同じ位置、スレッドを始める前）。
  `kv` のキーは `app_kv/` で始まるので、`kv` からは資格情報を読めず、消せません（`wifi_cred/0` は `/` を含むので
  `KV read failed: -22`）。形式と場所が Zephyr 版と同じなので、Zephyr 版で登録した資格情報をそのまま使えます
  （逆も）。保存された項目の長さが違うと、Zephyr と同じく読み出しは `-ENOENT`（`err: -2`）になります。
- 自動再接続（スレッド `wifi_reconnect_tid`、Zephyr の優先度 8 → 110、スタック 2048）: 資格情報があれば、
  起動時（`wifi_profile_init()`）と、接続が切れたとき（監視が切断を見つけたとき、接続中のスキャンの後）に、
  4 秒待ってから保存した資格情報で接続を要求し（`NET_REQUEST_WIFI_CONNECT_STORED`。保存した順に、要求が
  受け付けられるまで）、その後は 1 秒ごとに見て、アドレスが付かないまま 32 秒たつと要求し直します。
  `wapi disconnect wlan0`（手動の切断）の後は、次に接続に成功するまで再接続しません（Zephyr 版と同じ）。
  資格情報が無ければ何もしません。接続の要求が失敗したとき（`EBUSY`・`EALREADY` 以外）は
  `<wrn> wifi_profile: Stored Wi-Fi request failed: <値>` を出します。接続の失敗そのもの（約 30 秒で `ETIMEDOUT`）は
  ログに出ません（Zephyr 版でも出ない）。
- `wifi cred auto_connect` はシェルから接続を要求するだけで、自動再接続を始めません（接続に成功すると
  始まる）。要求が受け付けられないと `An error occurred when trying to auto-connect to a network. err: <値>` と
  出します（終了状態は Zephyr 版と同じく常に 0）。
- Zephyr 版の古い形式の資格情報（KV の `sys.wifi_profile`、settings の `wifi_last/last`）を移す処理は、
  Zephyr 版のまま起動時に走ります（保存形式が同じなので、あれば Zephyr 版と同じく移します）。
- パスフレーズは `wifi cred list` の表示（Zephyr 版と同じ）のほかは、ログにもエラーの文にも出しません。

## Wi-Fi 経由のシェル（Telnet）

Zephyr 版の Telnet のシェル（`CONFIG_SHELL_BACKEND_TELNET`）と同じく、Wi-Fi に繋がっている間は TCP の
ポート 23 で USB と同じシェルを使えます（`nuttx/apps/init/rp2040_telnet.c`、`CONFIG_INIT_TELNET`。
ポートは `CONFIG_INIT_TELNET_PORT`、Zephyr 版の `CONFIG_SHELL_TELNET_PORT`）。認証と暗号化がなく、
到達できる相手はシェルのすべての操作ができるので、信頼できるネットワークでだけ使ってください。

```bash
telnet <ボードの IPv4 アドレス>
```

- 待ち受け: 起動処理（init）が `USB console started` のログより前に、待ち受けのソケットを開きます（Zephyr 版の
  Telnet のバックエンドが `main()` の前の `SYS_INIT()` で開くのと同じ位置。ログは
  `<inf> shell_telnet: Telnet shell backend initialized`）。Zephyr 版と同じく、すべてのアドレス（`INADDR_ANY`）、
  backlog 1、`SO_REUSEADDR` なしです。ソケットは NINA が持つので、Wi-Fi に繋がる前に開いたソケットが
  接続の後で受け付けを始め、Wi-Fi の切断と再接続をまたいで待ち受け続けます（Zephyr 版と同じ）。開けなければ
  Zephyr 版と同じログ（`Unable to start telnet on NET_AF_INET (<値>)` など）を出し、Telnet は使えません。
- 同時に使えるクライアントは 1 つです。接続中に 2 本目が来ると、受け付けてすぐ閉じ、
  `<err> shell_telnet: Telnet client already connected.` をログに出します。1 本目はそのまま使えます（Zephyr 版と
  同じ）。受け付けは `telnetd` のタスクが行います（Zephyr 版でこれを行う `net_socket_service` スレッドと同じ
  優先度 104、スタック 2048）。受け付けが失敗したときは `Telnet accept error (<値>)` を出し、
  `CONFIG_INIT_RETRY_MS`（100 ms）待ってから次を受け付けます（失敗が続くときに空回りしないため。Zephyr 版は
  待ちません）。
- セッションは USB と同じ nsh です（同じコマンドと出力、プロンプト `nsh$ `、挨拶文なし）。USB のシェルとは
  別のタスクで、同時に独立して動きます（Zephyr 版の 2 つのバックエンドと同じ）。優先度 104、スタック 3072
  （USB のセッションと同じ。Zephyr 版の `shell_telnet` スレッドの優先度 14 と `CONFIG_SHELL_STACK_SIZE`）。
  `ps` では `nsh_telnet` です。NuttX の Telnet ドライバー（`CONFIG_NETDEV_TELNET`、
  `CONFIG_TELNET_MAXLCLIENTS=1`）が、受け付けた接続を `/dev/telnet0` の端末にし、セッションはそれを標準入出力に
  します。NuttX 標準の Telnet デーモン（`netutils/telnetd`、`NSH_TELNET`）は使いません。2 本目の接続
  （`TELNET_MAXLCLIENTS` を超えるセッション）や `SO_REUSEADDR` の失敗（NINA のソケットは `ENOPROTOOPT`）で
  待ち受けごと終わり、セッションが挨拶文を出すためです。
- 接続した直後にプロンプトが届きます。Zephyr 版は接続しただけでは何も送りません（接続前の出力は捨て、
  プロンプトは Enter の後に出ます）。`host/imu_host` は接続直後に届いたものを読み捨ててからコマンドを送るので、
  どちらでも同じに使えます。
- エコーと行編集: USB と同じく、入力した文字は readline がエコーし、行の編集・履歴・Tab・Ctrl-C（入力中の
  行を捨てて新しいプロンプト）が使えます。NuttX の Telnet ドライバーは、行を読む間は `IAC WILL ECHO`、
  コマンドの実行中は `IAC WONT ECHO` を送って、Telnet のクライアントのローカルエコーを切り替えます。Zephyr 版の
  Telnet のシェルは接続のときにエコーを切り、オプションの交渉もしません。Zephyr のシェルはエコーが無効の間は
  行の編集・履歴・Tab・Ctrl-C のキーも扱わないので、入力の見え方と編集はクライアントのローカルエコーに
  任されます。`host/imu_host` はエコーと Telnet のコマンドを読み捨てるので、どちらでも同じ結果になります。
- 改行: 出力の改行は CR LF です（Zephyr 版と同じ。パッチ `nuttx/0006`）。入力の Enter は CR、LF、CR LF、
  CR NUL のどれでもよく、USB と同じく 1 回だけ実行します。
- 切断: クライアントが接続を閉じたとき、接続が失敗したときにセッションが終わり、次の接続を受け付けます。
  Wi-Fi が切れると NINA が接続を閉じ、セッションは終わります。PC 側の
  接続には、Wi-Fi が戻った後の最初の送信で RST が届きます。ボードの `reboot` では NINA もリセットされるので、
  PC 側に切断は届きません。
- 標準入力を読むコマンド（引数なしの `cat` など）は、Telnet では Ctrl-C・Ctrl-D で終わりません（USB の
  シリアルドライバーへのパッチ `nuttx/0003` は Telnet のドライバーには効きません）。接続を閉じると終わります。
  セッションから `&` で裏で動かしたコマンドはセッションの端末を引き継ぐので、それが終わるまでは、接続を
  閉じた後の次の接続も 2 本目と同じく断られます。
- Telnet のセッションでは、コマンドの出力が、そのコマンドのタスクのスタックの上で NINA への SPI の転送まで
  進みます（USB では CDC ACM のバッファーに入れるだけ）。そのため、この移植のコマンドのスタックは Zephyr 版の
  シェルのスタックと同じ 3072 にしてあります（`dmesg` と `irqstack` は 1024 から上げました）。

## 速度の測定（`iperf`）

Zephyr 版の `zperf`（iperf 3 の方式）の代わりに、NuttX 標準の `iperf`（nuttx-apps の `netutils/iperf`、
iperf 2 の方式）を使います。PC 側は `iperf3` ではなく `iperf`（iperf 2）を使います。既定のポートは 5001 です
（Zephyr 版は 5201）。

| Zephyr 版 | NuttX 版 | PC 側（NuttX 版のとき） |
|---|---|---|
| `zperf tcp upload <PC> 5201 30 1024` | `iperf -c <PC> -t 30 -l 1024` | `iperf -s` |
| `zperf udp upload <PC> 5201 30 500 500K` | `iperf -c <PC> -u -t 30 -l 500`（速さは指定できず、送れるだけ送る） | `iperf -s -u` |
| `zperf tcp download 5201` | `iperf -s -B 0.0.0.0` | `iperf -c <ボード> -t 30` |
| `zperf tcp download stop` | TCP のサーバーは 1 回の接続が終わると自分で終わる | – |
| – | `iperf -s -u -B 0.0.0.0 -l 512`（UDP のサーバー。止めるには別のシェルから `iperf -a`） | `iperf -c <ボード> -u -l 500 -b 500K` |

- 主なオプション: `-c <宛先>`（クライアント）、`-s`（サーバー）、`-u`（UDP）、`-p <ポート>`、`-t <秒>`（クライアントの
  時間、既定 30）、`-i <秒>`（途中経過の間隔、既定 3）、`-l <バイト>`（1 回の読み書き。既定は UDP 1472、TCP 16384）、
  `-B <アドレス>`（サーバーの bind）、`-a`（動いている `iperf` に止まるよう伝える）。
- サーバーには `-B 0.0.0.0` を付けます。付けないと `wlan0` のアドレスに bind しようとし、NINA は特定の
  アドレスへの bind を受け付けません（Zephyr 版と同じく `EADDRNOTAVAIL`、`error code: 99`）。
- UDP のデータグラムは 512 バイトまでです（NINA。Zephyr 版と同じ）。既定の 1472 バイトでは送信が
  `EMSGSIZE`（`err=90`）で終わり、受信は 512 バイトより後を捨てるので、`-l 512` 以下を付けます。`-l` と、NINA が
  データグラムを受け取れないとき（`EAGAIN`）に待って送り直す動作は、パッチ `apps/0006` で足したものです。
- TCP のサーバーは、接続が 1 回終わると終わります（NuttX の `iperf` の動作）。接続が来ないうちは戻りません
  （`accept()` の中で待つので `iperf -a` でも止まりません）。UDP のサーバーは終わらないので、別のシェル（Telnet と
  USB のもう一方）から `iperf -a` で止めます（受信の期限 10 秒ごとに `udp server recv error` を出し、`-a` の後の
  期限で終わります）。
- スレッド: コマンドの `iperf`（優先度 104、スタック 3072）と、その pthread の `iperf_traffic`（優先度 100、
  スタック 4096）、`iperf_report`（101、4096）。2 つの pthread の優先度とスタックは `iperf` のソースに固定です
  （Zephyr 版の `zperf_work_q` は優先度 15 で、最も低いアプリの優先度 14 に下げられるので 104 に当たります）。
  読み書きのバッファーはヒープから取ります（TCP は 16 KiB）。
- `iperf` は浮動小数点の `printf`（`CONFIG_LIBC_FLOATINGPOINT`）と argtable3（`CONFIG_SYSTEM_ARGTABLE3`）を選びます。

## I2C0（`i2c`）

I2C0 には LSM6DSOX（0x6A）と ATECC608A（0x60）が載っています。互換層の I2C のバス `i2c@40044000` として、Zephyr 版と
同じ `i2c` コマンドで使えます（下の「`i2c` コマンド」）。条件に合う読み出しは DMA で行います（下の「DMA での読み出し（`i2c_dma`）」）。

ドライバーは、FreeRTOS 版と共有する I2C のコア（`common/drivers/i2c_dw/`）です。Zephyr 版が使う DesignWare I2C の
ドライバー（Zephyr の `drivers/i2c/i2c_dw.c` にローカルのパッチ
`zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` を当てたもの）のコントローラーの部分の写しで、
NuttX の `rp2040_i2c.c`（`CONFIG_RP2040_I2C`）は使いません。コアの説明は[設計書](imu-raw-mqtt-design-ja.md)の §12、NuttX の
部分（`common/drivers/i2c_dw/port/nuttx/` にあるコアの NuttX へのつなぎと境界の表）は `nuttx/drivers/README_ja.md` の
「I2C0」にあります。

```
nsh$ i2c read_byte i2c@40044000 6a 0f
Output: 0x6c
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

### DMA での読み出し（`i2c_dma`）

Zephyr 版の `CONFIG_I2C_DW_RP2040_DMA` と同じく（ここでも同じ名前のシンボル、既定で有効）、次の条件をすべて
満たす読み出しを DMA で行います。外れる読み出しと書き込みは割り込みで行います。

- 転送の最後のメッセージ（STOP で終わる）で、32〜3584 バイト、7 ビットアドレス、DMA が有効
  （`i2c_dma mode on`、起動時は on）。
- DMA のチャネルを 2 本使います（コマンド語を送る TX と、データを受け取る RX）。チャネルの確保や設定に
  失敗したときは、転送を始める前に割り込みでの処理に切り替え、`dma_fallbacks` を数えます。
- 期限・エラー値は割り込みの処理と同じです（1 秒で `-ETIMEDOUT`、NACK などは `-EIO`）。失敗の後は
  DMA と I2C を止めてから戻ります。DMA のチャネルが 100 ms 以内に止まらないときは I2C0 を隔離し
  （`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`）、以後の転送はすべて `-EIO`
  になります（再起動で戻ります）。
- DMA は NuttX 自身のドライバー（`CONFIG_RP2040_DMAC`、`arch/arm/src/rp2040/rp2040_dmac.c`）です。ボードは起動時に
  そこからチャネルを 2 つ取って読み出し用に持ち続けるので、NuttX 版では `dma_fallbacks` はチャネルの設定を断られたとき
  （まだ動いている）のほかは 0 のままです。ほかのコードも `rp2040_dmac.c` を通して残りのチャネルを通常どおり使えます
  （してはいけないことと、NuttX の DMA の上の実装の細部は `nuttx/drivers/README_ja.md` の「I2C0」）。
  SPI のドライバー（`CONFIG_RP2040_SPI_DMA`）が NINA の SPI1 のためにさらに 2 つを取ります。

```text
i2c_dma mode          現在の方式を表示（i2c_dma=on または i2c_dma=off）
i2c_dma mode on|off   切り替え（ストリームと転送が止まっているときだけ）
i2c_dma stats         DMA 転送数、バイト数、エラー、割り込み方式への切り替え回数（起動からの累積）
```

コマンドは Zephyr 版・FreeRTOS 版と同じ `common/shell/i2c_dma_shell.c` を `zshell.c` で動かすもので
（`nuttx/apps/i2c_dma` は組み込みコマンドの main のソースとしてビルドするだけ）、出力・ヘルプ・エラーは同じです
（`dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0`、`on`・`off` 以外は `Expected on or off`）。
切り替えは Zephyr 版と同じく値を変えるだけで、進行中の転送を待ったり止めたりしません。「止まっているときだけ」は
使い方の約束で、コマンドは確かめません。優先度 104、スタック 3072 です。

### `i2c` コマンド

Zephyr 版の `i2c` コマンド（Zephyr の `drivers/i2c/i2c_shell.c` にローカルのパッチ 0004 を当てたもの）と同じファイル
`common/shell/i2c_shell.c` を `zshell.c` で動かします（`nuttx/apps/i2c` は組み込みコマンドの main のソースとして
ビルドするだけ。FreeRTOS 版も同じファイル）。サブコマンド（`scan`、`recover`、`read`、`read_byte`、`direct_read`、`write`、
`write_byte`、`speed`）、構文、出力、ヘルプは Zephyr 版と同じで、デバイスは互換層の I2C のバス `i2c@40044000`
（LSM6DSO のドライバーが使うもの）です。`i2c speed` はこのバスの速さを変えるので、センサーの転送の速さも変わります
（Zephyr 版と同じ）。

```
nsh$ i2c read_byte i2c@40044000 6a 0f
Output: 0x6c
```

- `i2c scan` は 0 バイトの書き込みで確かめ、ドライバーがそれを断る（`-ENOTSUP`）ので、最初のアドレスで
  `I2C: i2c@40044000 does not support this scan (zero-length writes)` を表示して止まり、失敗します（Zephyr 版と同じ。
  1 つのアドレスは `i2c direct_read i2c@40044000 6a 1` で確かめる）。
- `i2c recover` はこのボードにリカバリーの処理が無いので `-ENOTSUP` で失敗します（表示する番号は NuttX の値）。
- `i2c target` はありません（Zephyr 版も `CONFIG_I2C_TARGET` が無効）。I2C1（`CONFIG_I2C_DW_I2C1`）には互換層のデバイスが
  無いので、このコマンドからは使えません。
- 優先度 104、スタック 3072 です。`/dev/i2c0` は登録しません（NuttX の `i2c` ツールは入れていない）。

## LSM6DSOX（IMU）

ドライバーは Zephyr 版と同じ `common/drivers/lsm6dso/` です（FIFO・校正・ゼロ点の変更を含む Zephyr の LSM6DSO ドライバーを、
OS を持たないコア `lsm6dso.c`・`lsm6dso_stream.c`（FIFO のストリーム）・`lsm6dso_cal.c`・`lsm6dso_fifo.c` と、Zephyr へのつなぎ
`port/zephyr/lsm6dso_sensor.c`・`lsm6dso_sensor.h` に分けたもの）。コアは `port/lsm6dso_os.h` を通して OS に、`port/lsm6dso_stream_req.h`
を通してストリームを使う側につながり、その実装は Zephyr へのつなぎの `lsm6dso_os_impl.h` と `lsm6dso_os.c`、`lsm6dso_stream_req_impl.h`
（RTIO）です。それと
つなぎが呼ぶものは Zephyr の名前のまま互換層が提供し、NuttX 版のものはつなぎの末尾のデバイスだけです
（`common/drivers/lsm6dso/port/nuttx/lsm6dso_instance.h`。ボードの値を使う `DEVICE_DT_DEFINE()` 1 つで、Zephyr 版の
`port/zephyr/lsm6dso_instance.h` のデバイスツリーのインスタンス化のマクロの代わり）。レジスターの操作は ST の
`lsm6dso_reg.c`（lsm6dso-pid v3.2.2。`script/setup.sh` が `nuttx/stmemsc/` に取得。Zephyr の hal_st のものと
LF に直して同一）で行います。FIFO のストリーム（INT1 の割り込みと RTIO）は Zephyr 版と同じファイル
（`common/drivers/lsm6dso/port/zephyr/`）で動かしています（下）。それを使う取得サービス（`imu start` など）は次の節です。

- **起動時の初期化**: ボードの起動処理（`rp2040_bringup()`）が、I2C0 の初期化の直後、KV（NVS）のマウント（init の
  中）より前に初期化します。Zephyr 版がデバイスを `main()` より前に初期化し（`POST_KERNEL`、優先度 90）、その後で
  NVS をマウントするのと同じ順です。`dmesg` の最初の 2 行は Zephyr 版と同じ文です:

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
  （`i2c read i2c@40044000 6a 00 20` で確認できます）。
- **I2C**: 起動時の速さは Zephyr 版のデバイスツリーと同じ 1 MHz（`board.h` の `BOARD_I2C0_FREQUENCY`）で、`i2c_configure()`（Zephyr の API の写し）で
  100 kHz・400 kHz に変えられます。NuttX の I2C はメッセージごとに周波数を持つので、
  この速さは I2C0 のデバイス（互換層の NuttX の実装、`nuttx/drivers/compat`）が覚えていて、互換層を通る転送に使います。
  `i2c speed` もこの速さを変えます（Zephyr 版と同じ）。レジスターの読み出しは 1 回の転送（レジスター番号の書き込み → RESTART → 読み出し）、書き込みは
  STOP 付きの 1 メッセージで、Zephyr の `i2c_burst_read_dt()`・`i2c_write_dt()` と同じ形です。FIFO の読み出しは
  32 バイト以上なので DMA を使います。
- **単発の取得**（`sensor_sample_fetch()`・`sensor_channel_get()`）、**属性**（`sensor_attr_set()` の ODR と
  フルスケール）、FIFO の開始・読み出し・停止の関数、センサーのタイムスタンプの読み取り、校正値の参照と
  その場でのゼロ点の取り直し（`lsm6dso_cal_zero()`）は Zephyr 版と同じ動作です。ODR は指定値以上で最小の
  段（12、26、52、104、208、417、833、1667、3333、6667 Hz）、加速度のフルスケールは m/s² を g に四捨五入して
  2・4・8・16 g、ジャイロは rad/s を dps に四捨五入して 125・250・500・1000・2000 dps だけを受け付けます
  （ほかは `-EINVAL`）。FIFO を使っている間の単発取得と属性の変更は `-EBUSY` です。
- **FIFO のストリーム**: コアの `lsm6dso_stream.c` とつなぎの `lsm6dso_rtio_stream.c`・`lsm6dso_os.c`（INT1 を含む）・`lsm6dso_rtio.c`
  は Zephyr 版と同じファイル（`common/drivers/lsm6dso/`。コアの FIFO の呼び出しを使う）で、Zephyr 版と同じく `sensor_stream()` で始まり、INT1（GPIO24）の立ち上がりエッジの割り込み →
  システムワークキュー（`sysworkq`）→ FIFO のワーカー（`imu_fifo_worker`）が FIFO を読んで RTIO の完了に
  します。ウォーターマークに届かないときは 20 ms ごとに見直し、`lsm6dso_fifo_request_stop()` で残りを最後の
  端数のバッチとして渡してから `-ECANCELED` で終えます。動かすための Zephyr のカーネル（ワークキュー、
  セマフォ、`k_poll()`）、RTIO、GPIO、センサーの非同期の API は互換層（`common/compat/` と `nuttx/drivers/compat/`）にあり、
  ボードの起動処理が `zephyr_compat_initialize()` で `sysworkq` と `RTIO WQ 0` を始めます。
- 呼ぶ側のための Zephyr の API の写し（`struct device`、`struct sensor_value`、チャネル・属性の列挙、
  `sensor_*()`、I2C の一部、カーネル、RTIO、GPIO）は互換層にあります（OS に依存しない部分は `common/compat/`、NuttX の
  実装は `nuttx/drivers/compat/`）。詳しくは
  `nuttx/drivers/README_ja.md` の「LSM6DSOX」と「Zephyr の API の互換層」の節を見てください。

## センサーのコマンド（`sensor`）

コマンドの本体は Zephyr 版の sensor シェルと同じファイル `common/drivers/sensor_shell/sensor_shell.c`（Zephyr v4.5.0 の
`drivers/sensor/sensor_shell.c` に修正と LSM6DSO の FIFO の属性を加えたもの。Zephyr 版のローカルのパッチ 0006）です。Zephyr の
`drivers/sensor/sensor_shell_stream.c`・`sensor_shell.h` の写しは互換層の共通の部分（`common/compat/`。Zephyr 版は
Zephyr のものを使う）、NuttX 版の部分は `nuttx/apps/sensor/` にあります。

- `sensor_shell.c` は最初に OS ごとのヘッダー `sensor_shell_os.h` を読みます。NuttX 版（`nuttx/apps/sensor/`）は
  `sensor_shell_nuttx.h`（Zephyr 版の構成の `CONFIG_SENSOR_SHELL_*` の値と、ログの `app_log.h`）を読み、
  `sensor get` が結果の表示を待つ 2 行（`sensor_shell_os_get_arm()`・`sensor_shell_os_get_wait()`。下の「出力先」）を
  `sensor_shell_nuttx.c` の関数にします。Zephyr 版（パッチを当てたツリーの `drivers/sensor/sensor_shell_os.h`）の同名の
  ヘッダーでは、この 2 行は何もしません。ログのレベル `CONFIG_SENSOR_LOG_LEVEL` は前置きヘッダーにあります
  （`nuttx/drivers/README_ja.md` の「Zephyr の API の互換層」）。
- `sensor_shell_stream.c`（写し）: 先頭の説明と OS ごとのヘッダー `sensor_shell_os.h` のインクルード、処理スレッドが呼ぶ
  コールバックを OS のもの（`sensor_shell_os_processing_callback()`。NuttX 版は、元のコールバックの後で待っている
  `sensor get` に知らせる `sensor_shell_nuttx.c` の関数）にした 1 行。
- `sensor_shell.h` は無変更の写しです。

コマンドの表（`SHELL_STATIC_SUBCMD_SET_CREATE()`、`SHELL_DYNAMIC_CMD_CREATE()`、`SHELL_CMD_REGISTER()`）も
Zephyr 版の文のままで、互換層の `<zephyr/shell/shell.h>` と `common/shell/zshell.c`（端末のバックエンドは NuttX の部分
`nuttx/apps/subcmd/zshell_os_impl.h`）が、Zephyr のシェルと同じ規則
（語の照合、動的なサブコマンド、ワイルドカードの展開、引数の数の検査、`SHELL_HELP()` の説明と `Usage:` の行と
その折り返し）で組み込みコマンド `sensor` にします。

```text
nsh$ sensor
sensor - Sensor commands
Subcommands:
  get       : Get sensor data.
              Channel names are optional. All channels are read when no channels
              are provided.
              Usage: get <device_name> [<channel name 0> .. <channel name N>]
  attr_set  : Set the sensor's channel attribute.
              Usage: attr_set <device_name> <channel_name> <attribute_name> <value>
  ...
nsh$ sensor get lsm6dsox accel_xyz
channel type=0(accel_x) index=0 shift=4 num_samples=1 value=<時刻>ns (<m/s²>)
...
channel type=3(accel_xyz) index=0 shift=4 num_samples=1 value=<時刻>ns, (<x>, <y>, <z>)
```

- サブコマンドは `get`、`attr_set`、`attr_get`、`stream`、`trig` です。Zephyr 版の構成には `CONFIG_SENSOR_INFO`
  が無いので `info` は無く、`sensor info` は `sensor` のヘルプになります（Zephyr 版と同じ）。構文、出力の書式
  （`channel type=... index=... shift=... num_samples=... value=...ns (...)`、`attr_get` の `%.6f`）、エラーの文は
  Zephyr 版と同じです。エラー番号は NuttX の番号です（例: `ENOTSUP` は Zephyr 版の 134 に対して 138）。
- デバイス名: `get`・`attr_set`・`attr_get`・`trig` はノード名 `lsm6dsox@6a` とノードのラベル `lsm6dsox` の
  どちらでも受け付けます。`stream` はノード名 `lsm6dsox@6a` だけです（`device_get_binding()`。Zephyr 版と
  同じ）。ヘルプに出るデバイスの一覧も `lsm6dsox@6a` です。ワイルドカード（`*`、`?`）はノード名の後でだけ
  チャネル名に展開されます（例: `sensor get lsm6dsox@6a gyro_?`。Zephyr 版と同じ）。
- 属性は `fifo_watermark`（1〜256、既定 32）・`fifo_timestamp_decimation`（0・1・8・32、既定 8）と、
  `sampling_frequency`・`full_scale`（「LSM6DSOX（IMU）」の節）です。ストリームの間（`sensor stream`、`imu start`）は
  `-16`（`-EBUSY`）で拒否します。`imu start` は watermark を KV の値（既定 128）で上書きします。起動直後は ODR が
  止まっているので、`sensor stream` は `sampling_frequency` を設定するまで `Read failed: -22` で終わります
  （Zephyr 版と同じ）。
- `imu start` とは同時に使えません。ストリーム中の `imu start` は `The IMU FIFO is in use (sensor stream?)`、
  計測中の `sensor stream` と `sensor get` は `Read failed: -16` です（ドライバーは Zephyr 版の写しで、`imu_stream.c` は Zephyr 版と同じファイル）。
- 校正値は `sensor get` と `sensor stream` の物理量に掛かります。`sensor stream` の間は、静止しているたびに
  角速度のバイアスが自動で更新されます（`imu cal` の `tracked at rest`）。
- `trig` は、Zephyr 版の構成と同じくドライバーにトリガーが無いので（`CONFIG_LSM6DSO_TRIGGER_NONE`）、
  `Error while setting trigger 1 on device lsm6dsox (-38)`（`-ENOSYS`）になります。
- Tab 補完はコマンド名だけです（nsh）。Zephyr 版はサブコマンド、デバイス名、チャネル名、属性名まで補完します。

**出力先**: Zephyr 版では、`sensor get` と `sensor stream` の結果を処理スレッド（`sensor_shell_tid`、
優先度 0、スタック 3072）が、コマンドを実行したシェル（USB か Telnet）に書きます。NuttX 版も同じスレッド
（優先度 118、スタック 3072。init が起動時に始めます）が書きます。

- 書き先は互換層のシェルのバックエンド（`zshell.c`）です。コマンドの標準出力の端末（USB は `/dev/ttyACM0`、
  Telnet は `/dev/telnet0`）ごとに 1 つあり、処理スレッドは 1 行ごとにその端末を開き、1 回の `write()` で
  書いて閉じます。コマンド自身の出力は、その標準出力と標準エラーです。
- `sensor get` は、処理スレッドが結果を書き終えるまで（最長 2 秒）待ってからプロンプトに戻ります。Zephyr 版は
  プロンプトの後に結果を出し、プロンプトを描き直します。nsh はプロンプトを描き直せないので、こうしています。
- `sensor stream` の出力は、プロンプトに戻った後も続きます。nsh はプロンプトや入力中の行を描き直さないので、
  入力の途中に出力が挟まります（Zephyr 版は 1 行ごとにプロンプトと入力中の行を描き直します）。止めるには、
  どの端末からでも `sensor stream lsm6dsox@6a off` を実行します（`reboot` でも止まります）。
- Telnet のクライアントが切れると、次のクライアントが繋がるまでの出力は捨て（端末の装置が無く開けない）、
  次に繋いだクライアントに出力が続きます（Zephyr 版の Telnet のバックエンドと同じ）。処理スレッドは書く間しか
  端末を開いていないので、次の接続を妨げません（`Telnet client already connected.` になりません）。
- PC が USB のポートを閉じている間の出力は、ほかの出力と同じく捨てます（「シェル」の節）。

## IMU の取得（`imu`）

取得サービスは `common/imu/imu_stream.c` で、Zephyr 版と同じファイルです（`nuttx/apps/imu/` が組み込みコマンド
`imu` の main のソースとしてビルドする）。OS を呼ぶ部分は互換層（`common/compat/` と `nuttx/drivers/compat/`。Zephyr の名前のまま）が
提供し、定数、分岐、順序、待ち時間、カウンター、出力の文は Zephyr 版のものがそのまま使われます。末尾の
コマンドの登録（`SHELL_CMD_REGISTER()`）も Zephyr 版の文のままで、互換層がそれを組み込みコマンド `imu` の
`main()` にします（「コマンドのサブコマンド」）。

```text
nsh$ imu
imu - IMU RAW streaming
Subcommands:
  start   : Start streaming to the MQTT server (settings: kv imu.*)
  stop    : Stop streaming
  cal     : Show the conversion used by sensor get/stream: [reload|zero]
  status  : Show counters and errors of the current or last run
```

- `imu start`・`imu stop`・`imu status` の構文と出力は Zephyr 版と同じです（`imu` と、何もしていないときの
  `imu status` は Zephyr 版で記録した出力と文字単位で同じ。`imu status` の項目の並びと桁も同じ）。設定は
  Zephyr 版と同じ KV のキー（`imu.host`、`imu.port`、`imu.device`、`imu.odr`、`imu.watermark`）から読みます。
- 取得はスレッド `imu_stream_tid`（Zephyr の優先度 9 → 109、スタック 2048 の静的な配列）で動きます。Zephyr が
  `K_THREAD_DEFINE()` のスレッドを `main()` の前に始めるのと同じく、init が FreeRTOS 版と共有のアプリのスレッドの表
  （`common/imu/app_init.c` の `app_threads_start()`、`K_THREAD_DEFINE_START()`）から、`main()` 相当の処理の前に始めます。コマンド `imu` 自体はシェルの優先度（104）の組み込みコマンドです。
- セッション ID は、前半 8 バイトがチップ ID（USB のシリアル番号と同じ。Zephyr の `hwinfo_get_device_id()` と
  同じバイト列）、続く 4 バイトが KV の `imu.session`（4 バイトのビッグエンディアン。`imu start` のたびに +1 して
  保存）、残りが 0 です。Zephyr 版の通し番号は引き継がず、NuttX 版では 1 から始まります。
- init は 3 つの OS のファームウェアで共有の Zephyr 版の `main()` の本体（`common/imu/app_main.c` の `app_main()`）を呼び、
  `kv_store_init()` → `imu_cal_init()`（失敗すると `Could not set up the IMU calibration: <値>`）→
  `time_sync_init()` → `wifi_profile_init()` の順になります。

### 校正（`imu cal`）

`common/imu/imu_cal.c` は Zephyr 版と同じファイルです（値の形式の `imu_cal_codec.c`・`imu_cal.h` も同じく `common/imu/` にあります）。
KV のキー（`imu.cal.accel.<µg/LSB>`、`imu.cal.gyro.<µdps/LSB>`）と値（56 バイト）、現在の感度のキーだけを
使うこと、`imu cal`・`imu cal reload`・`imu cal zero` の構文・出力・エラーの文は Zephyr 版と同じです
（Zephyr 版の README の「校正値の適用」、設計書 §10）。

```text
nsh$ imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)
```

- init が `kv_store_init()` の後に `imu_cal_init()` を呼び、ドライバーに「感度から校正値を引く関数」を
  登録します（Zephyr 版の `main()` と同じ位置）。ドライバーは登録時、`imu cal reload`、レンジの設定時に
  KV を読みます。
- 校正値が掛かるのはドライバーの物理量の出力（単発取得の `sensor_channel_get()` と FIFO のデコーダー）だけで、
  **MQTT で送る RAW には掛かりません**（Zephyr 版と同じ）。`sensor get` と `sensor stream` の値で確かめられます
  （「センサーのコマンド（`sensor`）」の節）。
- `imu cal zero` は計測中は `A measurement is running; it tracks the gyro bias itself` で拒否します。
  計測中も `imu cal` と `imu cal reload` は使えます。取り直した角速度のバイアスは RAM にだけ持ちます。
- エラーは stderr に出て、終了状態は 1 です（`kv` と同じ。成功は 0）。引数が 2 つ以上なら
  `cal: wrong parameter count` とヘルプです（Zephyr のシェルと同じ）。

### MQTT の送信

`common/imu/imu_raw_mqtt.c`・`imu_raw_mqtt.h` は Zephyr 版と同じファイルです（`imu_raw_mqtt.h` が読む
`<zephyr/net/net_ip.h>` は、互換層のものが `<netinet/in.h>` を読みます）。`CONFIG_IMU_RAW_MQTT`（既定で
有効）で入り、無ければ代わりの `common/imu/imu_raw_mqtt_null.c`（Zephyr 版がネットワークに繋がっていないときと同じく、
バッチをすべて捨てて数える）になります。トピック（`imu/<device>/raw`・`status`・`info`）と QoS（raw と
status は 0、info は 1 で retain）、クライアント ID（`imu-<device>`）、keepalive（60 秒）、クリーン
セッションなし、送信の順（接続のたびに info、status、キューのバッチ。status は 1 秒ごとと状態の変化時、
入力の確認は 100 ms ごと、キューが空なら 5 ms 待つ）、再接続の間隔（1 秒）、CONNACK の待ち（5 秒）、
`imu start` の最初の接続の待ち（最大 3 秒）、切断中のバッチを捨てて数えること、送信に失敗したバッチを
再送しないこと、`imu stop` で最後の status を最大 0.5 秒待つこと、`imu status` の `mqtt` の各項目と
それが増える事象は Zephyr 版と同じです。

- スレッド `imu_raw_mqtt_tid`（Zephyr の優先度 11 → 107、スタック 2048 の静的な配列）。init が起動処理の
  最初に始め、`imu start` を待ちます。
- MQTT のクライアントは、Zephyr の MQTT ライブラリ（`subsys/net/lib/mqtt/`）のうち使う部分の写し
  （`common/imu/mqtt_min.c`、API は互換層の `<zephyr/net/mqtt.h>`。ソケットの層は `common/imu/mqtt_min_sock.c`）です。MQTT 3.1.1、TCP だけで、
  送るバイト列、既定値、受信の分け方は Zephyr のライブラリと同じです（ホストテスト `mqtt_min` は同じ
  テストを Zephyr の本物のライブラリでも動かします）。PUBLISH は固定ヘッダーと本体を 2 つのベクターの
  `sendmsg()` 1 回で渡し、NINA のドライバーがそれを 1 つの送信の命令にまとめます（Zephyr 版と同じ）。
- ソケットはブロッキングのまま使います（Zephyr 版と同じ）。接続は最大 10 秒、空きのない送信は最大
  10 秒待ってから失敗します（NINA のドライバーの期限。`nuttx/drivers/README_ja.md` の「ソケット（usrsock）」）。
- 互換層の `zsock_socket()`・`zsock_poll()`・`zsock_recv()` は NuttX の `socket()`・`poll()`・`recv()` なので、
  ソケットは普通の usrsock のソケットです: `nina_event` が NINA に問い合わせ（20 ms ごと、ソケットの事象があれば
  IRQ の線のエッジ）、見つけたものをイベントにします（`nuttx/drivers/README_ja.md` の「ソケット（usrsock）」）。
  Zephyr 版は `imu_raw_mqtt_tid` だけが問い合わせます（受信の確認のたびに `CMD_POLL` を 1 回、100 ms ごと）。
- 受信の確認（`zsock_poll()`、100 ms ごと、待ち時間 0）と CONNACK の待ち（`zsock_poll()`、5000 ms）は、usrsock が
  持っているイベントで答え、CONNACK の待ちはイベントで終わります。NINA が報告する接続の異常（POLL_ERROR、
  または poll の失敗）は `ZSOCK_POLLHUP` です（usrsock の `REMOTE_CLOSED`。Zephyr 版は `ZSOCK_POLLERR`）。
  `imu_raw_mqtt.c` はどちらでも接続を終えます。受信（`zsock_recv()` の `ZSOCK_MSG_DONTWAIT`）がドライバーに届くのは
  usrsock が `RECVFROM_AVAIL` を持っている間だけです。2 バイトずつ 2 回に分けて読む CONNACK の 2 回目は、求めた
  長さをすべて受け取った受信のあとドライバーが `RECVFROM_AVAIL` を保つので届きます（次の受信は NINA に尋ねます。
  Zephyr 版の受信も毎回尋ねます）。
- Zephyr 版との違い: `lock_wait_*`・`publish_*` の時間は、Zephyr 版は 125 MHz の SysTick の周期
  （`k_cycle_get_*()`。送信中は遅れる）を µs にしたもの、NuttX 版は TIMER（1 µs）です（互換層の
  `k_cycle_get_*()`）。

## 時刻同期（`ntp`）

`common/imu/time_sync.c` は Zephyr 版と同じファイルです（`nuttx/apps/imu/` がビルドする）。OS を呼ぶ部分
（ソケット、`net_mgmt` のイベント、実時間時計）は互換層が Zephyr の名前のまま提供します。`time_sync_mono_us()` は
3 つの OS のファームウェアで共有する TIMER の読み出し（`common/chip/rp2040` の `rp2040_timer_us()`）です。ファイルが使う Kconfig の選択肢
（`CONFIG_NINA_NTP_*`）は、NuttX 版にも同じ名前であります。末尾の
コマンドの登録は、互換層が `z_shell_main_ntp()` にし、組み込みコマンド `ntp`（`nuttx/apps/ntp/`）の `main()` が
それを呼びます（「コマンドのサブコマンド」）。NTP の
パケットと応答の検査、t1（送信の直前）と t4（応答を受け取った直後）を読む位置、推定（`time_offset.c`。
無変更の写し。直近 8 回のうち往復が最短の応答を使う）、60 秒応答が無ければ無効、±500 ppm で寄せる逆行しない
時計、実時間時計を前にだけ進めること、問い合わせの周期・待ち・再試行、カウンター、`ntp status` の出力は
Zephyr 版と同じです。`CONFIG_TIME_SYNC`（既定で有効）で入り、無ければ代わりの `common/imu/time_sync_null.c`
（`time_sync_mono_us()` だけ本物で、同期は取れない）になります。

```text
nsh$ ntp
ntp - Network time synchronization
Subcommands:
  status  : Show time synchronization status
```

- スレッド `time_sync_tid`（Zephyr の優先度 9 → 109、スタック 2048）。init が起動処理の最初に始めます。
  Wi-Fi の接続の成功（`NET_EVENT_WIFI_CONNECT_RESULT`）で問い合わせを始め、切断
  （`NET_EVENT_WIFI_DISCONNECT_RESULT`）で止めます。接続のたびに推定を取り直します（別のネットワークかも
  しれないため）。未接続の間は何もしません。
- 問い合わせ先は `CONFIG_NINA_NTP_SERVER`（Zephyr 版と同じ。既定は空で、デフォルト
  ゲートウェイ）、ポートは `CONFIG_NINA_NTP_PORT`（123）、周期は `CONFIG_NINA_NTP_INTERVAL_MS`
  （2000 ms）です。応答は 250 ms 待ち、来なければ `timeouts`、検査に通らなければ `rejected` に数えて次の
  周期へ進みます。アドレスが決まらない、ソケットを作れない、送信に失敗した、などのときは `last_error` に
  入れて 5 秒後にやり直します。
- 応答の待ち方: ファイルは `zsock_recv()` を `ZSOCK_MSG_DONTWAIT` で約 1 ms ごとに呼びます。Zephyr 版では NINA の
  ドライバーがそのたびにソケットの状態を問い合わせ（`CMD_POLL`）、届いていれば読みます。こちらでは NuttX の
  `recv(MSG_DONTWAIT)` で、usrsock は受信可の印（`RECVFROM_AVAIL`）を持つまで自分で `EAGAIN` を返します。印は
  `nina_event` がソケットの poll から立て（最初は NINA 側のソケットを作る最初の送信の直後、以後 20 ms ごと、
  ソケットの事象があれば IRQ の線のエッジ）、受信はその poll の結果を使って poll を重ねずにデータグラムを
  読みます。そのため t4 は `nina_event` が応答に気づいたときに読みます: ソケットの事象があれば応答のエッジの
  すぐ後、無ければ最大 20 ms 遅れ、その遅れが往復に入り、半分がオフセットに入ります（推定は直近 8 回のうち往復が
  最短のものを使う）。`nuttx/drivers/README_ja.md` の「ソケット（usrsock）」の節。
- Zephyr 版との違い: `k_uptime_get()`（応答の待ち 250 ms、60 秒の判定、`last_reply_ms_ago`）は NuttX 版では
  TIMER から求めます。Zephyr 版は SysTick のカーネル時計で、送信中は遅れます。
- 実時間時計（`CLOCK_REALTIME`）は、推定より 20 ms 以上遅れているときだけ推定に合わせ（`realtime_clock_steps`
  に数える）、進んでいるときは戻しません。日付や時刻の表示には使えますが、精密な時刻には
  `time_sync_unix_us()` を使います。実時間時計の表示と設定は nsh の `date` です（Zephyr 版の `date`。
  `CONFIG_DATE_SHELL=y` に対応して `CONFIG_NSH_DISABLE_DATE` を外してある）。表示は UTC で、書式は nsh のもの
  です:

| Zephyr 版 | NuttX 版 |
|---|---|
| `date get` | `date`（例 `Sat, Nov 14 03:23:03 2026`） |
| `date set 2026-11-14 03:23:03` | `date -s "Nov 14 03:23:03 2026"` |

## システム時計

カーネルの時計は tickless で、1 tick は 100 µs です（Zephyr 版の `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`、
`CONFIG_TICKLESS_KERNEL=y` と同じ刻み）。周期割り込みはなく、次に期限の来る待ちの時刻に RP2040 の TIMER
（64 ビット、1 MHz、水晶から）の ALARM0 を合わせます（`nuttx/boards/arduino-nano-rp2040-connect/src/`
の `rp2040_oneshot.c`・`rp2040_timerisr_alarm.c`、NuttX の `drivers/timers/arch_alarm.c`）。ALARM1〜3 は使って
いません。設定は `CONFIG_RP2040_TICKLESS`（ボードの `Kconfig`。既定で有効）、`CONFIG_SCHED_TICKLESS=y`、
`CONFIG_USEC_PER_TICK=100` です。

- 待ち時間は tick に切り上げ、さらに 1 tick 足します（`usleep(1000)` は 1000〜1100 µs と起床の遅れ）。
  Zephyr 版の `k_sleep(K_MSEC(1))` と同じ規則です。
- `clock_gettime()` の `CLOCK_MONOTONIC`・`CLOCK_REALTIME` は TIMER の値をそのまま µs 単位で返します
  （`clock_getres()` は tick の長さ 100 µs を返します）。Zephyr 版のカーネル時計は CPU クロックの
  SysTick で、TIMER とは源が違います。アプリはどちらでも、時刻の基準に TIMER を直接読みます。
- ボードの単調時計は `rp2040_timer_us()`（`common/chip/rp2040/include/rp2040_timer_us.h`。3 つの OS のファームウェアで共有し、必ず
  インラインにする）で、TIMER の TIMERAWH・TIMERAWL を上位、下位、上位の順に読み、上位が一致するまで読み直します。副作用もロックも無いので、カーネル側・アプリ・割り込みハンドラーのどこからでも呼べます。
  値は起動からの µs で、TIMER は起動のたびに（`reboot` でも）0 から数え直します。
  `CLOCK_MONOTONIC` と同じカウンターなので、両者は読み出しの時間差（数 µs）の範囲で一致します。
- 同じ優先度のスレッドは 20 ms ずつの時分割（`CONFIG_RR_INTERVAL=20`。Zephyr 版の
  `CONFIG_TIMESLICE_SIZE=20`）です。時分割の残り時間を計る wdog は、スレッドが待ちに入っても
  取り消されないので、時分割の残り時間より長く眠ると、眠っている間に割り込みが 1 回余分に
  入ります（NuttX 13.1.0 の動作）。
- `up_udelay()`・`up_mdelay()`・`up_ndelay()` は TIMER を直接読みながら µs 単位で待ちます（割り込み
  禁止中でも使えます）。
- カウンターと tick・時刻の換算は、ボードが `arch_alarm.c` の weak 関数（`up_timer_gettick()`、
  `up_timer_gettime()`、`up_alarm_tick_start()`、`up_alarm_cancel()`、`up_udelay()`、`up_ndelay()`）を
  置き換えて、1 MHz と 100 µs に特化した計算（`src/rp2040_timeconv.h`。100 での除算を 1 バイトずつ
  32 ビットの乗算とシフトで行う）でします。Cortex-M0+ には除算命令がなく、NuttX の汎用の換算
  （`ONESHOT_COUNT`）は 64 ビットの除算になり遅いためです。結果は
  64 ビットの全域で汎用の換算と同じです（ホストテスト `rp2040_timeconv`）。換算が 100 µs の tick を
  前提にするので、`CONFIG_USEC_PER_TICK` が 100 でなければビルドが止まります。

## CPU 負荷

`cpu_load_get(bool reset)`（`#include "cpu_load.h"`。ボードの `src/rp2040_idle_cpuload.c`）は、Zephyr 版の
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

### スレッドごとの CPU 時間（`ps` の CPU 列）

NuttX 標準のクリティカルセクションの監視（`CONFIG_SCHED_CRITMONITOR`）が持つ、スレッドごとの CPU 時間の
積算を使います（`CONFIG_SCHED_CPULOAD_CRITMONITOR`。パッチ `nuttx/0007`・`nuttx/0008`）。最長時間の監視
（`CONFIG_SCHED_CRITMONITOR_MAXTIME_*`）は既定の −1 のままで使いません。上の `cpu_load_get()`（`imu status` の
`cpu_active`）は別の仕組みで、これを有効にしても値の測り方は変わりません。

- スレッドの切り替えのたびに、外れるスレッドへ前の切り替えからの時間を足します。時間の源はボードの perf の
  カウンター（`src/rp2040_perf.c`、`CONFIG_RP2040_PERF`。TIMER の 1 µs。Cortex-M0+ にはサイクルカウンターが
  ありません）です。Zephyr 版の `CONFIG_SCHED_THREAD_USAGE` も、切り替え（PendSV）のときだけ CPU の
  サイクル数を足します。
- 割り込みの処理の時間は、Zephyr 版でも NuttX 版でも割り込まれたスレッドの時間に入ります（アイドル中の割り込みは
  アイドルスレッドに入ります）。そのため、アイドルスレッドの割合は、割り込みの処理を負荷に数える
  `cpu_active` の残り（100 % − `cpu_active`）とは一致しません。
- 読める値:
  - `ps` の CPU 列と `/proc/<tid>/loadavg`: そのスレッドの割合（0.1 % 単位）。移動平均で、全体の積算が
    `CONFIG_SCHED_CPULOAD_TIMECONSTANT`（2 秒）を超えるたびに全スレッドの積算を半分にします（常に動いて
    いれば約 1 秒ごとに半分になり、直近の 1〜2 秒の重みが大きくなります）。`ps` は行ごとに読むので行の
    合計は 100 % にならないことがあり、`ps` を実行しているシェルの分も入ります。
  - `/proc/cpuload`: 100 % − アイドルスレッドの割合（同じ移動平均）。
  - `/proc/<tid>/critmon`: `最長の連続実行時間,起動からの実行時間の合計`（秒、ナノ秒の桁まで）。合計は
    減らないので、2 回読んだ差から区間の割合を求められます（Zephyr 版の `kernel thread list` の
    `Total execution cycles` に当たります）。最長の値は読むたびに 0 に戻ります。
- Zephyr 版・FreeRTOS 版の `kernel ps`・`ps` の CPU は前回のそのコマンドからの区間の割合（0.1 % 単位、最初は
  起動から。見たい区間の最初と終わりに 1 回ずつ打つ）、Zephyr 版の `kernel thread list` の
  `Total execution cycles` は起動からの累積のサイクル数です。NuttX の `ps` の CPU 列は上の移動平均です。
- nsh の `top`（同じ値を繰り返し表示する）は無効にしています（Zephyr 版に無いコマンドのため）。
- PANIC の報告のタスクの一覧にも CPU の列が出ます。

## スレッドの優先度とスタック

優先度は `NuttX = 118 − Zephyr` で対応させ、`common/compat/priorities.h` に定数（`APP_PRIO_*`）と
スタックの大きさ（`APP_STACK_*`）を置きます（換算 `APP_PRIO_FROM_ZEPHYR()` と NuttX だけのスレッドは
`nuttx/drivers/include/priorities_os.h`）。Zephyr 版のアプリのスレッド（優先度 8〜11）は 110〜107、
シェルのスレッド（14）は 104 です。

| スレッド | Zephyr | NuttX | スタック |
|---|---|---|---|
| `wifi_reconnect_tid`、`nina_wifi_tid`（NINA のスキャン・接続の試み・接続の監視を 1 つずつ行う） | 8 | 110 | 2048、2048 |
| `time_sync_tid`、`imu_stream_tid` | 9 | 109 | 2048、2048 |
| `imu_fifo_worker` | 10 | 108 | 2048 |
| `imu_raw_mqtt_tid` | 11 | 107 | 2048 |
| `nina_event`（NINA のソケットの状態を usrsock のイベントにする。NuttX 版だけ。最初のソケットで起動） | – | 106 | 1536 |
| `sysworkq`（システムワークキュー。互換層。INT1 の処理） | −1（協調） | 119 | 1024 |
| `net_mgmt`（ネットワークのイベント。互換層。Wi-Fi のイベントのコールバック） | −1（協調） | 119 | 2048 |
| `RTIO WQ 0`（RTIO のワークキュー。互換層。単発取得。`ps` では `RTIO_WQ_0`） | 0 | 118 | 1024 |
| `sensor_shell_tid`（`sensor get`・`sensor stream` の結果の表示） | 0 | 118 | 3072 |
| `main`（NuttX は init。起動処理の後シェルの優先度に下げる） | 0 | 118 | 2048 |
| init、`nina_pt_rx`、`nina_pt_tx`（NINA の中継モードのときだけ。NuttX の init と、カーネルスレッド 2 つ） | – | 120 | 2048 |
| シェル（nsh のセッション、コマンド。USB の `nsh`、Telnet の `nsh_telnet`） | 14 | 104 | 3072 |
| `telnetd`（Telnet の受け付け。Zephyr 版は `net_socket_service`） | 15 → 14 | 104 | 2048 |
| `iperf_traffic`・`iperf_report`（`iperf` の pthread。Zephyr 版の `zperf_work_q` に当たる） | 15 → 14 | 100・101 | 4096・4096 |

- Zephyr の協調スレッド（負の優先度。`sysworkq`、`net_mgmt` など）は 118 より上になりますが、NuttX には
  他のスレッドに横取りされない協調スレッドが無く、より高い優先度のスレッドには横取りされます。今の移植で
  協調スレッドに当たるのは `sysworkq` と `net_mgmt`（どちらも 119）で、それより高い優先度のスレッドは
  ありません。同じ 119 の 2 つは時分割で交代します（Zephyr の協調スレッドどうしは自分で譲るまで交代しない）。
- `K_THREAD_DEFINE()` のスレッド（`nina_wifi_tid`、`imu_stream_tid`、
  `imu_raw_mqtt_tid`、`time_sync_tid`、`wifi_reconnect_tid`、`sensor_shell_tid`）は、
  Zephyr 版が `main()` の前に始めるのに合わせて、init が起動処理の最初に始めます。
- `imu_stream_tid`、`imu_raw_mqtt_tid`、`time_sync_tid`、`imu_fifo_worker`、`sysworkq`、`RTIO WQ 0`、`net_mgmt`、`nina_wifi_tid`、`wifi_reconnect_tid`、`sensor_shell_tid` のスタックは Zephyr と同じく静的な配列です
  （`K_THREAD_DEFINE()`・`K_THREAD_STACK_DEFINE()`、NuttX の `kthread_create_with_stack()`）。`ps` の STACK は
  NuttX が先頭に置く領域の分だけ小さく出ます（2048 → 2008。`imu_stream_tid` は 2000、1024 → 976）。
- `CONFIG_PRIORITY_INHERITANCE=y` です。ミューテックスは優先度継承あり、セマフォは既定で継承なし
  （NuttX 13.1.0 の `sem_init()` は flags を 0 にします）なので、通知用のセマフォはそのままで継承しません。
- 割り込み用のスタック（`CONFIG_ARCH_INTERRUPTSTACK=2048`。Zephyr 版の `CONFIG_ISR_STACK_SIZE=2048` と
  同じ大きさ）: スレッドはプロセススタック（PSP）、割り込みハンドラーはメインスタック（MSP）で動き、
  割り込みの処理は割り込まれたスレッドのスタックを使いません。割り込まれたスレッドのスタックに積まれる
  のは、ハードウェアが退避する 32 バイト（とコンテキストの切り替えのときの保存領域）だけです（Zephyr 版と
  同じ）。割り込み用のスタックの使用量は `ps` には出ないので、`irqstack` で
  見ます（Zephyr 版の `kernel thread stacks` の最後の行と同じ書式。値はスタックの塗りつぶし
  （`CONFIG_STACK_COLORATION`）から求めた起動からの最大で、PANIC の報告の `irq` の行と同じ）:

  ```
  nsh$ irqstack
  <アドレス> IRQ 00                           (real size 2048):	unused <未使用>	usage <使用量> / 2048 (<割合> %)
  ```
- スタックの番兵（`CONFIG_RP2040_STACK_SENTINEL`。ボードの `Kconfig`、既定で有効）: Zephyr 版の
  `CONFIG_STACK_SENTINEL` と同じく、各スタックの最下位のワード（`CONFIG_STACK_COLORATION` が書いた値）を、
  スレッドの切り替えのとき（切り替えで外れるスレッド）と、割り込みハンドラーの出口ごと（割り込まれた
  スレッド。ハードウェアが退避した値がそのスタックに積まれている）に確かめ、書き換わっていれば PANIC に
  します（報告は割り込み用のスタックの上で作り、RAM ログに入ります。その後は自動で再起動します。
  「ログ（`dmesg`）」の節）。割り込み用のスタック自体は、
  Zephyr 版と同じく番兵で確かめません。スケジューラーの計測用のフック 2 つ（切り替えの `sched_note_suspend()` と、
  `irq_dispatch()` が各ハンドラーの前後で呼ぶ `sched_note_irqhandler()`。`src/rp2040_stackcheck.c`）で
  行います。Cortex-M0+ では周辺の割り込みは `arm_doirq()` を通らず（`exception_direct()`）、
  切り替えのフックだけでは割り込みの出口で確かめられないためです。割り込みはすべて同じ優先度で
  入れ子にならないので、どの出口も Zephyr の「入れ子でない割り込みの出口」に当たります。NuttX の
  `CONFIG_STACKCHECK_MARGIN` は切り替えのときにしか確かめず、`CONFIG_DEBUG_ASSERTIONS`（すべての
  DEBUGASSERT）も要るので使いません。
- `ps` の STACK・USED・FILLED でスタックの使用量を、CPU でスレッドごとの CPU 時間の割合（「CPU 負荷」の節）を見られます。

## C ライブラリの `memcpy`・`memset`

NuttX へのパッチ `nuttx/0010` の `CONFIG_RP2040_ROM_MEM_OPS`（ボードの `defconfig` で有効）が、`memcpy`・`memset`・
`__aeabi_memcpy*`・`__aeabi_memset*` を RP2040 のブート ROM のルーチンにします（Pico SDK の `pico_mem_ops` と同じ使い分け。
Zephyr 版は Zephyr の SoC へのローカルのパッチで同じことをします）。`LIBC_ARCH_MEMCPY`・
`LIBC_ARCH_MEMSET` を選び、`libs/libc/string` のバイト単位のループを外します。コードはチップのディレクトリーの
`rp2040_rom_mem_ops.S`・`rp2040_rom_mem_lookup.c` です。ROM のルーチンは初回の呼び出しで ROM の関数表から引いて `.bss` に
置くので（リセットの前から RAM に残った値は使わない）、`.bss` をゼロにする前には呼べません。GCC は `rp2040_start.c` の
`.bss` のゼロ化と `.data` のコピーのループを `memset`・`memcpy` の呼び出しに変えるので、パッチがループに
`START_LOOP_BARRIER()`（メモリーを clobber する空の asm）を入れてループのまま残し、`build.sh` がその結果を確かめます。

## 整数の割り算

`CONFIG_RP2040_PICO_DIVIDER`（ボードの `Kconfig`、既定で有効）で、コンパイラーが出す整数の割り算（`__aeabi_idiv`・`__aeabi_idivmod`・
`__aeabi_uidiv`・`__aeabi_uidivmod`・`__aeabi_ldivmod`・`__aeabi_uldivmod`）を、libgcc のシフトと減算のループの代わりに
SIO のハードウェアの除算器で行います。Pico SDK の `pico_divider`（`divider_hardware.S`。`nuttx/pico-sdk` から、ボードの
`src/pico_sdk` の最小のヘッダーでビルド）を、ボードの `scripts/Make.defs` の `ld --wrap` でつなぎます。

- ルーチンは割り込みを止めません（`PICO_DIVIDER_DISABLE_INTERRUPTS` 0）。割り算の途中に割り込みハンドラーが割り算をすると、
  ルーチンが DIRTY を見て除算器を退避・復元します。
- タスクが割り算の途中で切り替わる場合は、NuttX へのパッチ `nuttx/0009` の `CONFIG_RP2040_HW_DIVIDER`（`CONFIG_RP2040_PICO_DIVIDER` が
  select する）が退避します。ARMv6-M の切り替えはすべて `arm_doirq()` を通るので、タスクが変わるときに、古いタスクの除算器を
  DIRTY のときだけ `struct xcptcontext` の 20 バイトに置き、新しいタスクのものを戻します（割り算の途中でなかったタスクは
  DIRTY が落ちた状態で戻る）。`arm_sigdeliver()` の最後の切り替えの前にも退避します。FreeRTOS 版（Pico SDK のポート）は
  切り替えのたびに 4 語を退避します。
- 0 での割り算は Pico SDK のルーチンの結果です（Zephyr 版・FreeRTOS 版と同じ）。32 ビットは飽和した商（0xffffffff、
  0x7fffffff、0x80000000。0 / 0 は 0）と余り 0（libgcc は商 0 と被除数）、64 ビットは libgcc と同じです。
- ルーチンは SIO とスタックだけを使うので、`__start()` の最初から、割り込みハンドラーと異常停止の処理でも使えます。
- 試験のイメージ（2026-11-18、実機）: 退避ありのイメージは 250 秒・約 1,128 万回の割り算で食い違い 0、退避なしのイメージは 40 秒で 10 回でした。

## RAM に置くコード

センサーの 6667 Hz の経路と送信の経路のコード（互換層のカーネルの部分・ワークキュー・RTIO・GPIO、I2C0 と DMA、
LSM6DSOX のドライバー、IMU のサービスのストリーム・バッチ・時刻付け、MQTT、NINA の usrsock のつなぎ・ソケット・
プロトコル・SPI と、NuttX の DMA・SPI のドライバー）は、FreeRTOS 版と同じく 16 KiB の XIP キャッシュを通さず RAM で
動きます。オブジェクトはボードのリンカースクリプト（`scripts/arduino-nano-rp2040-connect-flash.ld`）に並べてあり、
その `.text` がコードを `.data` に回し、`__start()` が `.data` のほかの部分と一緒に RAM へ写します。定数、NuttX 自身のコード、
動かしてはいけないもの（`.data` を写す前に動くコード、`svc` を含むコード、異常停止の経路）はフラッシュに残ります。
ヒープを約 46 KiB 使い、`build.sh` が配置と、ヒープが 80 KiB 以上残ることを確かめます。一覧・決まり・RAM の見積もりは
`nuttx/drivers/README_ja.md` の「RAM に置くコード」にあります。

## シグナルハンドラー

NuttX 版のコードはシグナルハンドラーを登録しません（`sigaction()`・`signal()`、SIGEV_SIGNAL のタイマー、`kill()`・
`pthread_kill()` でハンドラーを走らせる使い方をしない）。待ちはセマフォ、sleep、ブロッキングの入出力で書きます。

シグナルハンドラーの実行中にタスクの切り替え（1 ms ごとの起床と優先度の入れ替え）が重なる負荷を掛けると、数秒以内に
ログを残さずに固まることを実機で確かめました（2026-11-18。libgcc の割り算で除算器のパッチの処理が無いイメージでも同じ）。
原因は特定しておらず、修正も入れていません。今のコードにはハンドラーの登録が無く、
通常の動作ではこの経路を通りません。

## ホストテストの詳細

次のファイルは Zephyr 版と NuttX 版（と FreeRTOS 版）が同じものを使い、リポジトリの `common/` に 1 本ずつ置いています（I2C のコアは
NuttX 版と FreeRTOS 版が使います。下の表。`common/` の全体の構成は設計書の §12）。
Zephyr 版の `CMakeLists.txt` と NuttX 版の Makefile（`apps/imu/Makefile`、`apps/nina/Makefile`、`apps/wifi/Makefile`、`apps/sensor/Makefile`、
`drivers/compat/Make.defs`、`drivers/lsm6dso/Make.defs`、`drivers/nina/Make.defs`、ボードの `src/Make.defs`）がそこを参照します。Zephyr の API を
使うもの（`common/drivers/` の FIFO の設定とストリーム、単発取得の RTIO の経路、NINA の Wi-Fi、sensor シェルと、
`common/imu/` の取得サービス・時刻同期・校正・MQTT・Wi-Fi のプロファイル）は、NuttX 版では互換層の上で動きます。NuttX 版は `common/` の
各ファイル（NINA のコア、I2C のコア、`common/chip/` を除く）に前置きヘッダー（`nuttx/drivers/compat/common_prelude.h`）を付けてコンパイルし、OS のファームウェアごとに違う数行は
OS ごとのヘッダー（`sensor_shell_os.h`、`wifi_profile_os.h`、`lsm6dso_instance.h`。Zephyr 版は `zephyr/app/usb-console/`
と `common/drivers/lsm6dso/port/zephyr/` の下、NuttX 版は `nuttx/apps/` と `common/drivers/lsm6dso/port/nuttx/` の下）に出しています（`nuttx/drivers/README_ja.md` の
「Zephyr の API の互換層」）。互換層のうち OS に依存しない部分も `common/compat/` に置き、NuttX 版はそれと NuttX の
実装（`nuttx/drivers/compat/`）を使います（Zephyr 版は使わない）。

| 置き場所 | 内容 |
|---|---|
| `common/imu/` | `imu_stream.c`、`imu_raw_{batch,packet,queue,status,time}`、`imu_tick_clock`、`time_sync.[ch]`、`time_offset`、`imu_cal_codec.c`、`imu_cal.[ch]`、`imu_raw_mqtt.[ch]`、`wifi_profile.[ch]`、`kv_store.[ch]`、`mqtt_min.[ch]`・`mqtt_min_sock.c`（互換層の上でビルドする NuttX 版・FreeRTOS 版の MQTT クライアントとそのソケットの層。Zephyr 版は使わない）、`imu_raw_mqtt_null.c`・`time_sync_null.c`（`CONFIG_IMU_RAW_MQTT`・`CONFIG_TIME_SYNC` が無いときの代わり） |
| `common/drivers/lsm6dso/` | ドライバー。直下にコア `lsm6dso.c`・`lsm6dso_stream.c`・`lsm6dso_cal.c`・`lsm6dso_fifo.c`（OS を持たない。`internal/` に `lsm6dso_core.h`・`lsm6dso_stream.h`・`lsm6dso_fifo_packet.h`、`port/` に境界 `lsm6dso_os.h`・`lsm6dso_stream_req.h`）、`include/` に使う側のヘッダー `lsm6dso_cal.h`・`lsm6dso_fifo.h` と拡張 API `lsm6dso.h`（`common/compat` を通して `<zephyr/drivers/sensor/lsm6dso.h>`）、`port/zephyr/` に Zephyr へのつなぎ（`lsm6dso_sensor.[ch]`・`lsm6dso_decoder.[ch]`・`lsm6dso_rtio.c`・`lsm6dso_rtio_stream.c`・`lsm6dso_os_impl.h`・`lsm6dso_os.c`・`lsm6dso_stream_req_impl.h`。Zephyr 版のモジュール）、`port/nuttx/`・`port/freertos/` に OS ごとの `lsm6dso_instance.h` |
| `common/drivers/nina/` | NINA のコア: 直下（ヘッダーは `include/`）に `nina_wifi.[ch]`・`nina_sock.[ch]`・`nina_dns.[ch]`・esptool の中継 `nina_passthrough.[ch]`、`internal/` に `nina_proto.[ch]`・`nina_wait.[ch]`、`port/` にコアの境界のヘッダー `nina_os.h`・`nina_bus.h`（中継のものは `nina_passthrough_bus.h`・`nina_passthrough_os.h`）（3 つの OS のファームウェアが使う。Zephyr の API を使わず、NuttX 版では前置きヘッダーなしでコンパイルする。境界は `nuttx/drivers/README_ja.md` の「NINA-W102」）。コアのつなぎは `port/zephyr/`（Zephyr 版と FreeRTOS 版）と `port/nuttx/`（NuttX 版: `wlan0`、アプリの Wi-Fi 管理、`nina_os.c`、usrsock、中継のホスト側 `nina_passthrough_os.c`、RP2040 の SPI と線 `nina_bus_rp2040.c`、中継のモジュール側 `nina_passthrough_bus_rp2040.c`） |
| `common/drivers/sensor_shell/` | `sensor_shell.c`（Zephyr 版のツリーにパッチ 0006 を当てたファイル） |
| `common/drivers/stmemsc/` | `stmemsc.h`、`stmemsc_i2c.c`、`stmemsc_mdelay.c`（Zephyr の stmemsc の I2C の部分の写し。NuttX 版と FreeRTOS 版が使い、前置きヘッダーなしでコンパイルする。Zephyr 版は Zephyr のものを使う） |
| `common/drivers/i2c_dw/` | I2C のコア: 直下に `i2c_dw.c`（I2C0 と、`CONFIG_I2C_DW_I2C1`（既定で無効）のとき I2C1）、`internal/` にその定義 `i2c_dw.h`・`i2c_dw_registers.h`・`i2c_dw_rp2040.h` と DMA での読み出し `i2c_dw_rp2040_dma_priv.h`（DMA のコマンド語を含む。Zephyr 版はパッチ 0003 に同じ関数を持つ）（ファイル名はパッチ後の Zephyr のドライバーに合わせる）、`include/` に `i2c_dma` の関数の宣言 `i2c_dw_rp2040_dma.h`、`port/` に境界のヘッダー `i2c_dw_os.h` と DMA 読み出しの `i2c_dw_dma_os.h`（NuttX 版と FreeRTOS 版が使う。前置きヘッダーなしでコンパイルする。境界は `nuttx/drivers/README_ja.md` の「I2C0」）。コアのつなぎは `port/nuttx/`（NuttX 版: `i2c_dw_os_impl.h`・`i2c_dw_instance.h`、起動の `i2c_dw_controllers_rp2040.h`、DMA の部分 `i2c_dw_dma_os_rp2040.c`。`nuttx/drivers/i2c_dw/Make.defs` がビルドする）と `port/freertos/`（FreeRTOS 版） |
| `common/chip/rp2040/` | どの OS も持たない RP2040 のコード: `include/rp2040_timer_us.h`（TIMER の読み出し）、`include/rp2040_watchdog_reboot.h`、KV の領域の flash の境界 `rp2040_flash_kv.c`（`include/rp2040_flash_kv.h`。ボードの flash の操作は `include/rp2040_flash_kv_os.h` で宣言）。FreeRTOS 版と共有し、ボードの `src/Make.defs` が前置きヘッダーなしでコンパイルする |
| `common/compat/` | 互換層の OS に依存しない部分（Zephyr の API のヘッダー `zephyr/...`。`i2c_dma` の状態 `zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` は I2C のコアの `common/drivers/i2c_dw/include/i2c_dw_rp2040_dma.h` を読む。`zephyr_compat.h`、ワーク、RTIO、ブロックのプール、デバイスの一覧、`gpio0`、`net_mgmt`、Wi-Fi の要求、sensor の既定の iodev）と、互換層の上でビルドする NuttX 版・FreeRTOS 版が使う Zephyr のライブラリーの写し（`sensor_shell_stream.c`・`sensor_shell.h`、`wifi_credentials.c`・`wifi_credentials_backend_settings.c`・`wifi_credentials_internal.h`）、境界のヘッダー（`priorities.h`、`cpu_load.h`、`gpio_os.h`）。Zephyr 版は使わない |
| `common/shell/` | `i2c_shell.c`（Zephyr 版のツリーにパッチ 0004 を当てたファイル。`nuttx/apps/i2c` がビルドする）、`subcmd.[ch]`（サブコマンドの表の処理）、`zshell.c`（`<zephyr/shell/shell.h>` で書いたコマンドの処理: 語の照合、ワイルドカード、引数の数の検査、ヘルプ）とその OS との境界 `zshell_os.h`（NuttX 版の部分は `nuttx/apps/subcmd/zshell_os_impl.h`）。前置きヘッダーなしでコンパイルする。Zephyr 版は使わない。ほかに `i2c_dma_shell.c`（3 つの OS のファームウェアの `i2c_dma` コマンド。Zephyr のシェルの API で書いてあり、NuttX 版では `nuttx/apps/i2c_dma` が前置きヘッダー付きでビルドする）と `nina_shell.c`（Zephyr 版と FreeRTOS 版の `nina` コマンド。NuttX 版は自前の `nuttx/apps/nina`） |
| `common/compat/log/` | `log_ring.[ch]`（RAM ログのリング、行の前置き、頻度制限）、`app_log.h`（`LOG_*` マクロ。NuttX の部分は `nuttx/drivers/include/app_log_os.h`）、`log_ram.c`・`log_ram.h`（RAM ログと `LOG_*` マクロの関数。FreeRTOS 版と共有。NuttX の部分は `nuttx/drivers/include/log_ram_os_impl.h`、syslog のチャネルは `nuttx/drivers/log/log_ram_syslog.c`）、`fault_record.[ch]`（異常停止の記録の形式、CRC-32、watchdog の scratch への写し、表示の行。FreeRTOS 版だけが使う）。Zephyr 版は使わない |
| `common/tests/` | 上のファイルを対象にするテスト 37 本と、Zephyr のツリーとの照合 `zephyr_copies`・`zephyr_sensor_shell_patched`（`common/tests/run.sh` が実行し、3 つの OS のファームウェアの `host_tests.sh` が最初に呼ぶ。互換層の `zephyr_rtio`・`zephyr_work`・`mqtt_min` は偽のカーネルの上、NINA のコアの `nina_proto`・`nina_sock`・`nina_wifi`・`nina_dns`・`nina_passthrough` はテストが定義する境界の偽物の上。`nina_core_independent` はコアの各ファイルを C ライブラリーと自分のヘッダーだけでコンパイルし、`i2c_dw_public_headers` は I2C のコアの `include/` をそのディレクトリーだけでコンパイルし、LSM6DSO の `lsm6dso_core_independent`・`lsm6dso_public_headers` はそのコアと `include/` に同じことをする）。`lsm6dso_stream` はコアのストリーム `lsm6dso_stream.c` を、境界のホストでの実装（`common/tests/lsm6dso_os_host/`）とコアの FIFO の呼び出しの偽物の上で動かす |
| `common/tests/fakes/` | テスト用の Zephyr のヘッダーと ST のヘッダーの代わり（`zephyr/drivers/sensor.h`、`zephyr/drivers/sensor_clock.h`、`zephyr/drivers/i2c.h`、`zephyr/rtio/rtio.h`、`lsm6dso_reg.h`） |
| `common/tests/zephyr_fakes/` | 互換層の偽のカーネル: OS ごとのヘッダー（`zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h`・`priorities_os.h`、`zephyr/sys/__assert.h`）とその関数（`zephyr_kernel_host.[ch]`。1 スレッドで、ワークキューとタイマーはテストが回す）。NuttX 版のテストも使う |

`common/` のソースを対象にするテストのうち、NuttX 版の互換層の実装（ソケット、I2C、シェルなど）やボードのコードと
偽のカーネル（`common/tests/zephyr_fakes/`）で動かすもの（`lsm6dso_driver`、`imu_cal`、`imu_raw_mqtt`、`sensor_shell`、
`time_sync`、`wifi_cred`、`zephyr_compat_misc`）は `nuttx/tests/` にあり、`nuttx/script/host_tests.sh` が前置き
ヘッダーを付けてビルドします。`common/tests/mqtt_min.c` を Zephyr の MQTT ライブラリーの上で動かすもの（Zephyr の
ツリーと NuttX 版の `<zephyr/net/socket.h>` が要る）も `nuttx/script/host_tests.sh` がビルドします。

次のファイルは Zephyr の写しです。Zephyr 版を直したときは、写しにも同じ変更を入れます。
`common/compat/sensor_shell_stream.c`・`sensor_shell.h` は Zephyr の写しで、違いは「センサーのコマンド（`sensor`）」の節に
書いたものだけです。

`nuttx/tests/fakes/nuttx/config.h`（空）は NuttX 版だけのもので、`<nuttx/config.h>` を読む `nuttx/apps` の
ソースをホストテストでビルドするために置いています（`nina_passthrough`: `apps/nina/nina_main.c`）。

`nuttx/tests/rp2040_timeconv.c` は NuttX 版だけのテストです。`src/rp2040_timeconv.h` の換算を、C の
`/`・`%` と、NuttX 13.1.0 の汎用の換算（`oneshot.h`・`clkcnt.h` の計算を 1 MHz・100 µs で書き写した
もの）と比べます。対象は 1 バイトずつの除算が正しい範囲の全数、境界値（0、99、100、2^32 前後、
2^32×100 前後、2^63、2^64−1 など）の前後 300、最初の 10^7 µs の全数、2^32 の倍数 256 個の前後 2000、
乱数 400 万個です。

`nuttx/tests/nina_os.c` は、NINA の READY の割り込みでの待ち（コアの `wait_ready()` の下の `common/drivers/nina/port/nuttx/nina_os.c`、
`common/compat/zephyr_gpio.c` を通る）を、NuttX のセマフォと時計、GPIO バンク 0 の偽物の上で確かめます。待ちの途中の
READY の変化（両方の向き）、1 回の待ちのあとの期限でのタイムアウト、コアが読んだ直後の変化（待ちがすぐ返る）、
前から残った起床とラッチされたエッジ（捨てる）、待ちの間だけエッジが有効なこと、セマフォの数が 1 を超えないこと、
Wi-Fi の操作のロックと、Wi-Fi のスレッドを起こす待ちです。

`nuttx/tests/nina_passthrough.c` は、`apps/nina` の `nina passthrough`（ヘルプの順、確認の語、引数の数、メッセージ、
`confirm` のときだけドライバーの要求を呼ぶこと）を確かめます。中継そのもの（`common/drivers/nina/nina_passthrough.c`: 線、出口、
レート、UART1 の語、リング）は `common/tests/nina_passthrough.c` が確かめます。その NuttX 側
（`common/drivers/nina/port/nuttx/nina_passthrough_os.c`、`nina_passthrough_bus_rp2040.c`）はホストテストで確かめていません。

`common/tests/cpu_load.c` と `common/tests/log_ring.c` は、Zephyr 版が使わない共通の部分のテストです。前者は
`common/compat/cpu_load.h` の計算を Zephyr の `cpu_load_get_cpu()` の手順（リセット、開いている区間、0 の扱い、
切り捨て）と比べます。後者は `common/compat/log/log_ring.c` を、Zephyr 版の `log_ram.c` の 1 バイトずつの
書き込みと乱数の書き込み 2 万回で比べ、行の前置きを Zephyr の `log_output.c` の時刻の計算と乱数 20 万個で
比べ、`dmesg` の取り出し（上書き中の読み飛ばし、リングの 2 倍での打ち切り）と頻度制限を、各 OS のファームウェアのリングの大きさ（4096 バイトと 8192 バイト）で確かめます（`common/tests/log_ram.c` も同じ）。
`common/tests/log_ram.c` は `common/compat/log/log_ram.c` を境界のホストの部分（`common/tests/log_ram_os_host/`）の上で動かし、
`LOG_*` マクロの行（前置き、レベル、関数名、192 バイトでの切り詰め）、そのままのバイト、起動時の確認、時計の上の頻度制限、
すべての読み書きが割り込みの禁止の中にあることを確かめます。

KV のテストは `common/tests/` にあります（`common/tests/run.sh`）。`kv_settings` は `common/imu/kv_store.c` を、
写した Zephyr の settings と NVS（警告を出さずにそのままビルド）、互換層の flash デバイスと `kv wipe` と一緒に、
偽の flash（`common/tests/kv_flash_fake.c`: 8 KiB の RAM、書き込みはビットを落とすだけ、4 KiB 単位の消去、
範囲の外や揃っていない要求はテストを止める）の上で動かします。起動のたびに子プロセスで動かし、flash だけを
引き継ぎます。確かめること: NVS の大きさと位置が Zephyr 版と同じこと、flash の上のバイト列（空の領域に
Zephyr の NVS が書く最初の索引、`app_kv/imu.port` と `app_kv/imu.session` の名前・値・最大の名前の ID の索引と
データ。CRC はテストの中で 1 ビットずつ計算し、写した `crc8_ccitt()` とも比べる）、読み書きと削除、キーと値の
上限、再起動の後、整理（消去 10 回以上）、満杯（`-ENOSPC`、削除はできる）、別のデータ（乱数、1 バイトだけ
書かれた領域、以前の NuttX 版の形式: 使わず、触れず、ログが 1 回、`kv wipe` の後に使える）、flash デバイスが
領域の外を断ること、Zephyr の NVS そのものが両方のセクターの末尾が書かれた領域で `-EDEADLK` になること、
電源断（上の「キーと値の保存」）。`flash_kv` は `common/compat/flash_kv.h` の範囲の検査を境界の前後と、
flash の先頭と末尾 1 MiB のすべてのページ・消去単位で確かめ、ページ単位の書き込みを、ページ境界の前後の
さまざまな位置と長さで、書いたバイトだけが変わることで確かめます。起動時の判定は、Zephyr の NVS の索引と
NuttX の NVS の索引をそれぞれ受け付け、相手の形式の索引の並び（CRC がたまたま合うものを含む、それぞれ
20 万通り）を受け付けないことを確かめます。`hex` は Zephyr の `lib/utils/hex.c` の写しを確かめます。
`zephyr_copies` は、Zephyr のツリー（`zephyr/zephyr`）があるとき、写した 23 本（KV の NVS と settings、fnmatch、getopt、`wifi cred` のシェルなど）が v4.5.0 の元と同じことを
`cmp` で確かめます。`nuttx/tests/kv_cmd.c` は `kv` のコマンドを `z_shell_main_kv()` から上の KV の上で動かし、
すべてのサブコマンドの出力（以前の NuttX 版と同じ文）、`sys.` の拒否、上限、`kv wipe`、別のデータがあるときの
`-EILSEQ` を確かめます。`nuttx/tests/wifi_cred.c` は資格情報を同じ KV の上で確かめます。

`common/tests/subcmd.c` は `common/shell/subcmd.c` を、`kv` と `nina` の表（Zephyr 版の記録した出力と同じ
ヘルプ）、入れ子の表、省略可の引数、ヘルプの文の並べ方（改行、80 桁での折り返し、末尾の改行）で確かめます。

`nuttx/tests/shell_register.c` は `imu`・`ntp`・`kv` のコマンドの登録を確かめます（`kv` は以前の NuttX 版の
`subcmd` の表に `wipe` を足したものと比べる）。`common/imu/imu_stream.c`・
`time_sync.c` の `SHELL_STATIC_SUBCMD_SET_CREATE()` から `SHELL_CMD_REGISTER()` までの文を `host_tests.sh` が切り出し、
互換層の `<zephyr/shell/shell.h>` と `Z_SHELL_CMD_NAMED_MAIN` でビルドして `common/shell/zshell.c`（NuttX のバックエンド
`apps/subcmd/zshell_os_impl.h` と）で動かし、同じ表を
`subcmd` の表に書いたもの（`subcmd_main()`。ハンドラーには `sh` として NULL が渡る）と比べます。ヘルプ（コマンドだけ、
`-h`・`--help`、知らない語、ワイルドカード）、引数の数の誤り、各ハンドラーへの引数について、終了状態、ハンドラーの
呼び出しと `argc`・`argv`、標準出力、標準エラー、両方を 1 つのファイルにしたときの順序が同じこと（ハンドラーは
`shell_print()`・`shell_warn()`・`shell_error()` で行を書く。標準出力は NuttX 版と同じ 64 バイトの行バッファー）と、
違う場合（一致するワイルドカードの後のサブコマンド、30 語を越える行）の Zephyr のシェルの出力を確かめます。
`nuttx/tests/i2c_dma_app.c` は `i2c_dma` のコマンド（`common/shell/i2c_dma_shell.c`）を同じやり方で
`zshell.c` の上で動かし、2026-11-18 までの NuttX 版のコマンド（同じ表を `subcmd` で書いたもの）と比べ、出力の文と
違う場合を確かめます（I2C0 のコアの側は偽物）。

`nuttx/tests/wifi_cred.c` は `common/compat/` の Zephyr のライブラリの写し（`wifi_credentials.c`、settings の
バックエンド）と `wifi cred` のコマンド（`wifi_credentials_shell.c` の写し、`subcmd`、互換層の
`zephyr_wifi_utils.c`）を、名前付きの項目を持つ偽の settings で確かめます: 起動時の読み込み、保存する名前
（`wifi_cred/<番号>`。`app_kv/` と重ならない）と中身、長さの境界（SSID 32/33、パスフレーズ 7/8/64/65）、
3 件目の `-105` とログ、同じ SSID の置き換えと番号の順の一覧、各オプションとエラーの文、長さの違う保存値、
保存の失敗、`auto_connect`、`wifi`・`wifi cred` のヘルプ。期待する出力は Zephyr のソースが出す文です。`common/tests/nina_proto.c` は `common/drivers/nina/internal/nina_proto.c` を、
偽の SPI・GPIO・時計・ロックで確かめます。広いパラメーターと詰め、
まとめての転送、0x75・0x7a の 3 つの応答パラメーター、長さの検査、転送の失敗、狭いパラメーター、
ファームウェアのバージョンの要求、応答の先頭探し（1000 バイト）、各 `-EPROTO`・`-EMSGSIZE`・`-EINVAL`、0x27、`nina_proto_ip_config()`
（0x21）、READY の期限（2000・20・10000 ms を 100 µs ごとに見る回数）、最初の要求のリセット手順、CS と
転送の順序、ロックを確かめます。

`common/tests/i2c_dw.c` は `common/drivers/i2c_dw/internal/i2c_dw.h` を、Zephyr 版の `i2c_dw.c`・`i2c_dw.h` と Zephyr の
`i2c.h` の該当部分をテストの中に書き写したものと比べます。SCL のカウント（125 MHz で 500/625、93/109、31/31、
クロック 1〜400 MHz と速さの区分のすべての組み合わせ）、受け付ける周波数、NuttX のメッセージのフラグから
Zephyr のフラグへの変換、`IC_DATA_CMD` に書く語の列（`i2c` ツールの `get`・`set`・`set -s`・`dump`・`dev`、
センサーのレジスターの読み書き、3 つのメッセージなど。前の転送の向きが次の転送の最初の RESTART に効くことを
含めて、転送を続けて流す）、`IC_CON` の値とレジスターの配置を確かめます。

`nuttx/tests/lsm6dso_driver.c` も NuttX 版だけのテストです。LSM6DSO のドライバー（`common/drivers/lsm6dso/` のコアとつなぎの
`port/zephyr/lsm6dso_sensor.c`。NuttX 版の `lsm6dso_instance.h` と）を、ST の本物の `lsm6dso_reg.c`、stmemsc の層、互換層（`drivers/compat`）、ボードの `src/rp2040_lsm6dso.c` と一緒にビルドし、
偽の I2C マスターの先の偽の LSM6DSOX（レジスターのバンク、アドレスの自動増加、ソフトウェアリセット、FIFO、
タイムスタンプ）に対して動かします。偽物のヘッダーは `nuttx/tests/lsm6dso_fakes/`（ボードの `board.h` は本物を
読みます）。確かめるのは、初期化のログの文と順序、書き込むレジスターと値の順序（リセット後の 100 µs を
含む）、初期化後のレジスター、感度、各段階での I2C の失敗とそのログ、`device_is_ready()`、すべての転送の
アドレス・速さ（100 kHz と `i2c_configure()` の後の値）・メッセージの形、単発取得の読むレジスターと換算
（`lsm6dso_cal.c` の結果と物理量、`val1`・`val2` の符号、1 軸と 3 軸）、属性（ODR の段、m/s² と rad/s の
丸め、受け付けない値とチャネル、`-EBUSY`）、FIFO のレジスターの操作（static の関数を使うため、テストは `lsm6dso.c` を
読み込む。受け付けない引数、タイムスタンプの間引きごとの設定、設定のどの転送が失敗してもバイパスに戻ること、
一時停止、INT1 へのしきい値の割り当て、状態のビット、バースト読み出しの範囲）、FIFO の開始・読み出し・一時停止・停止と統計、
タイムスタンプの読み取り、`lsm6dso_cal_zero()` です。ドライバーの待ち（`k_busy_wait()`、`k_msleep()`、`k_usleep()`）とサイクル
カウンター（`k_cycle_get_32()`）はテストの偽物で、待ちをレジスターの書き込みの列の中に記録し、眠った時間を
足し上げます。ミューテックスとスピンロックは偽のカーネルのものです。FIFO のストリームも、コアの `lsm6dso_stream.c`、
つなぎの `lsm6dso_rtio_stream.c`・`lsm6dso_os.c`・`lsm6dso_rtio.c`・デコーダーと互換層（ワーク、RTIO、`gpio0`、既定の iodev とデコーダー）を、偽のカーネル
（`common/tests/zephyr_fakes/`。1 スレッドで、ワークキューとタイマーはテストが回す）と、偽のセンサーの
ウォーターマークに従う偽の INT1 で動かし、ストリームの開始（レジスター、INT1 の割り込み）、INT1 のエッジと
20 ms の見直しでのバッチ、2 本目（`-EBUSY`）とほかのトリガー（`-ENOTSUP`）、穏やかな停止（一時停止、最後の
端数のバッチ、`-ECANCELED`、レジスターの戻り）、キャンセル、オーバーラン（`-EOVERFLOW`）、RTIO の
ワークキューでの単発取得とデコーダーを確かめます。

`nuttx/tests/imu_cal.c` も NuttX 版だけのテストです。`lsm6dso_driver.c` を読み込んで同じ偽のセンサーを使い
（`lsm6dso_driver` の確認を先に一通り動かします）、`common/imu/imu_cal.c` を偽の KV と一緒にビルドして確かめます:
起動時の読み込み（キー 2 つ）、保存した値が単発取得の出力に掛かること（`M·(raw−bias)`）、長さ・形式のバージョン・種類・感度・
予約バイトが合わない値や長すぎる値は公称の変換になること、レンジの変更で別の感度のキーを読むことと戻ること、
`reload`（消したキーは公称に戻る、追従したバイアスを捨てる）、`zero`（静止時、保存したバイアスから遠い、
ストリーム中、I2C のエラー）、使い方の誤り、IMU が準備できていないとき、`imu cal` の各行（バイアスの小数 2 桁の
丸めを含む）と出力先（stdout と stderr）。

`nuttx/tests/sensor_shell.c` も NuttX 版だけのテストです。`lsm6dso_driver.c` を読み込んで同じ偽のセンサーを使い、
`common/drivers/sensor_shell/sensor_shell.c` と `apps/sensor/` を `common/shell/zshell.c` と一緒にビルドし、コマンドの標準出力と標準エラーを 1 つの
ファイル（端末の代わり）に向けて確かめます: `sensor` のヘルプ（`info` が無いこと、`SHELL_HELP()` の説明と
`Usage:` の行の 80 桁での折り返し）、引数の数の誤りとヘルプと動的なデバイスの一覧、ワイルドカード（サブコマンド
やチャネル名への展開、一致しないときに語の読み取りが止まること、ハンドラーが 2 つになるときのエラー）、
デバイス名（ノード名とラベル、`stream` はノード名だけ）、`get`（全チャネル、指定、番号、不明な名前、行の書式）、
`attr_get`・`attr_set`（FIFO の属性、値の解析、不明な名前、ドライバーのエラー、`%.6f`、全チャネルと全属性）、
`stream`（引数の誤り、開始、処理スレッドのコールバックが出すトリガーの行とバッチの平均、ストリーム中の
`attr_set` の拒否、`off`）、`trig`。結果はコマンドが戻った後に、処理スレッドのコールバックがコマンドの端末
（バックエンドのパス）に書きます。

`common/tests/zephyr_rtio.c` と `zephyr_work.c` は、互換層の共通の部分の RTIO とワークを偽の
カーネル（`common/tests/zephyr_fakes/`）の上で確かめます（`common/tests/run.sh` が実行する）。前者は `imu_stream.c` と同じ寸法のコンテキストと偽の iodev で、メモリープール
（ブロックの大きさ、`max` から `min` への割り当て、送信への結び付け、最初の適合、枯渇と解放）、送信のプール
（4 つ、解放順の再利用、ハンドルと世代）、完了（順序、16 個、あふれた完了の `xcqcnt` とそのバッファーが残ること、
`consume_sem`）、multishot（完了の前の再投入、結び付けの解除、ハンドルの維持、失敗とキャンセルでの解放）、
キャンセル（印、iodev の関数、古いハンドルと無効なハンドル、送信前のキャンセルは完了を出さない）、RTIO の
ワークキューを確かめます。後者は `submit`・`schedule`・`reschedule`・`k_work_cancel_sync()` の戻り値とフラグ、
`K_MSEC()` の tick への換算を確かめます。

`nuttx/tests/zephyr_compat_misc.c` も NuttX 版だけのテストで、取得サービスのために互換層に足した部品を
確かめます: `K_MUTEX_DEFINE()`、`K_THREAD_DEFINE()`（名前、優先度 9 → 109、スタック、入口と引数）と
`K_THREAD_DEFINE_START()`、`k_uptime_get()` の丸め（Zephyr の `k_ticks_to_ms_floor64()` を 100 µs の tick に
使ったものと、桁上がりの前後と 32 ビットの tick の一周をまたぐ値で比べる）、`k_sleep(K_MSEC(1))` の tick 数、
`sys_put_be32()`・`sys_get_be32()`、`snprintk()`、`hwinfo_get_device_id()`（偽のチップ ID）、`shell_print()`・
`shell_error()` の出力先、`zsock_inet_pton()`・`zsock_inet_ntop()`。

`nuttx/tests/zephyr_mutex.c` は互換層の `k_mutex`（`drivers/compat/zephyr_mutex.c`）を、NuttX 向けと同じ
分岐（`__NuttX__`）で、NuttX 13.1.0 の再帰ミューテックスを模した偽物（全部 0 のものはスレッド 0 が持っている）の
上でビルドして確かめます: `K_MUTEX_DEFINE()`、`k_mutex_init()`、全部 0 のまま初期化していない `k_mutex`（MQTT の
クライアントの `mqtt_client_init()` の前の状態）が Zephyr と同じく最初の lock で待たずに取れること、再帰、
他のスレッドが持っているときの `-EBUSY`・`-EAGAIN`。

`common/tests/nina_sock.c` は `common/drivers/nina/nina_sock.c` を、命令とパラメーターを検査して決まった応答を返す偽の
`nina_proto_request()` と偽の時計で確かめます: 対応するソケットの種類、10 個の表と `EMFILE`、NINA 側のソケットを
最初に使うときに作ること（種類とプロトコル、FIONBIO）、bind、listen（backlog を 1〜255 に丸める）、accept
（相手のアドレスとポートのバイト順、`EAGAIN`、`EMFILE`）、connect（UDP、TCP の処理中、SO_ERROR の各値の
errno、エラーフラグ、10000 ms の期限、poll の失敗、失敗後に NINA 側のソケットを閉じること）、close、
setsockopt、`nina_event` が何を問い合わせ、何をイベントにするか、`nina_sock_poll_revents()`（Zephyr 版と FreeRTOS 版の
`do_poll()` のソケット 1 つ分）。送受信では、TCP の送信（1 回 3584 バイトまで、
応答の検査、`MSG_DONTWAIT`、待つ送信の各回と 100 ms ごとの状態確認・10000 ms の期限、同じ呼び出しの続きと
新しい呼び出しの区別）、`sendmsg`（複数のベクターを 1 つの命令にまとめること、受け付けないもの）、UDP の送信
（アドレスの検査、接続した相手、512 バイトの上限、NINA が受け取らなかったデータグラム）、TCP と UDP の受信
（先に poll すること、サイズのパラメーター、送信元のポートのバイト順、接続した相手以外を捨てること、空の
データグラム）、データを見つけた poll を次の受信の最初の poll に使うこと。

`common/tests/nina_dns.c` は `common/drivers/nina/nina_dns.c` を、偽の `nina_proto_request()` とロックで確かめます: ヒントの
既定値と受け付けないフラグ・ファミリー・ソケットの種類、サービス（ポート番号）、数値のアドレス（NINA に問い合わせ
ない）、名前（ロックを持ったまま命令 0x34 と 0x35）、失敗ごとの `EAI_*`。

`common/tests/nina_wifi.c` は、コアの Wi-Fi の操作（`common/drivers/nina/nina_wifi.c`）を、モジュール・`nina_os.h`・つなぎの
偽物の上で確かめます（接続とその確認とスレッド、切断、監視、スキャン、状態）。`nuttx/tests/nina_netdev.c` は `wlan0`
（`common/drivers/nina/port/nuttx/nina_netdev.c`）を、NuttX の `<nuttx/wireless/wireless.h>` と、デバイス・コア・管理の偽物の上で
確かめます（起動、コアの結果のデバイスへの反映、wapi の ioctl、スレッド）。`nuttx/tests/nina_wifi_mgmt.c` は、アプリの
管理（`nina_wifi_mgmt.c`）を互換層の Wi-Fi の要求とインターフェースの下で確かめます。
