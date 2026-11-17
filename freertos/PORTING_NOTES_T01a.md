# T01a 事前調査の結果（2026-11-17）

計画（`freertos/PORTING_PLAN.md`）のタスク T01 のうち、`common/` への切り出し（P1・P2）に要る事実を集めた。
未追跡のファイル（commit しない）。調べた時点のコミットは `0f14ce0f02dbb4084bf305b096d0eced5dd22bea`。

記号: **[確]** 実行して確かめた（コマンドと結果を書く）、**[読]** ソースを読んだだけ（推測を含む）、
**[未]** 確かめられなかった。

実験は追跡ファイルを変えずに、作業用ディレクトリー
`/tmp/claude-1000/-home-ouwa-work-rp2040-connect/ad5eefab-694c-41dd-9cad-b9e1036f99c2/scratchpad/t01a/` に
Zephyr のアプリと NuttX のワークスペースを写して行った（写しは最後に消した。ログと実験用の
`CMakeLists.txt` は残してある: `zexp/mv/zephyr/app/usb-console/CMakeLists.txt`）。

---

## 1. Zephyr のビルドから `common/` を参照する方法

### 1.1 結論

- **[確]** `target_sources()` にも `use_local_driver_sources()` の差し替え先にも、アプリの外のファイルを指定できる。
  写しのアプリで、A 層と同一の C 層のファイル（T10・T11 に当たる 33 本。`imu_cal.h` は `src` に残した）と `sensor_shell.c` を `../../../common/...` へ
  移し、CMake を下の 1.3 のように直してビルドすると、**`zephyr.bin` が基準とバイト単位で一致した**（`cmp` で差なし）。
  `zephyr.elf` は一致しない（デバッグ情報のパスが変わる）。
- **[確]** 差し替えの関数（`CMakeLists.txt` 13〜30 行）は `"${CMAKE_CURRENT_SOURCE_DIR}/${replacement_dir}/${filename}"` を
  作るだけなので、`replacement_dir` に `../../../common/drivers/sensor_shell` を渡しても動いた（`compile_commands.json` に
  `.../usb-console/../../../common/drivers/sensor_shell/sensor_shell.c` が載り、`zephyr.bin` は一致）。計画のリスク
  （165〜166 行「同じ差し替えが効くかは T01 で確かめる」）は解消。
- **[確]** アプリの外のソースのオブジェクトは `CMakeFiles/<target>.dir/<絶対パスの先頭の / を除いたもの>/x.c.obj` に置かれる
  （例: `CMakeFiles/app.dir/tmp/.../common/imu/imu_raw_batch.c.obj`）。ビルドディレクトリーの中なので Git には影響しない。
- **[確]** リンク順は `target_sources()` に並べた順のまま。順を変えなければ配置も変わらない（上の一致がその確認）。

### 1.2 インクルードの解決（今の形）

`compile_commands.json` から読んだ、ターゲットごとの `-I`（**[確]**）:

| ターゲット | 我々のソース | 我々のディレクトリーの `-I` |
|---|---|---|
| `app` | `src/*.c`、`drivers/nina/*.c` | `src`、`drivers/lsm6dso`（`CMakeLists.txt` 11 行） |
| `drivers__sensor__st__lsm6dso` | `drivers/lsm6dso/*.c` | **無い**（`-I ZEPHYR_BASE/drivers/sensor/st/stmemsc` だけ。同じディレクトリーのヘッダーは「インクルードしたファイルのディレクトリー」で見つかっている） |
| `drivers__i2c` | `drivers/i2c_dw/*.c` | 無い（`-I ZEPHYR_BASE/drivers/i2c` だけ。36 行） |
| `drivers__sensor` | `drivers/sensor_shell/sensor_shell.c` | 無い（`-I ZEPHYR_BASE/drivers/sensor`。40 行。`"sensor_shell.h"` は Zephyr 本体のものが見つかる） |

我々のソースの `#include "..."` をすべて洗い、ディレクトリーをまたぐものを挙げる（`grep -n '#include "'`、**[確]**）:

| ファイル:行 | インクルード | 今の解決 | `common/` へ移したとき |
|---|---|---|---|
| `src/imu_raw_packet.h:7` | `"../drivers/lsm6dso/lsm6dso_fifo.h"` | ヘッダーのディレクトリーからの相対 | 計画の構成（`common/imu` と `common/drivers/lsm6dso`）なら相対のまま解決する（**[確]**） |
| `src/imu_stream.c:30,31` | `"../drivers/lsm6dso/lsm6dso_fifo.h"`、`_stats.h` | 同上 | `imu_stream.c` が `src` に残りヘッダーが移ると、`-I common/imu` に `../drivers/lsm6dso/...` を足した形で偶然見つかる（**[確]**: ninja の deps で `common/drivers/lsm6dso/lsm6dso_fifo.h`）。`"lsm6dso_fifo.h"` に直して `-I common/drivers/lsm6dso` で探すほうが分かりやすい |
| `drivers/sensor_shell/sensor_shell.c:20` | `"../lsm6dso/lsm6dso_attributes.h"` | 同じディレクトリーからの相対 | **`lsm6dso_attributes.h` だけ移すと壊れる**（**[確]**: `fatal error: ../lsm6dso/lsm6dso_attributes.h`）。`"lsm6dso_attributes.h"` と `-I` に直すか、`sensor_shell.c` と同時に移す |
| `src/imu_cal.h:7` | `"lsm6dso_cal.h"` | `-I drivers/lsm6dso` | `-I common/drivers/lsm6dso` |
| `src/main.c:6` | `"../drivers/nina/nina_proto.h"` | 相対 | `nina_proto.h` を移す（T21）ときに直す |
| `drivers/nina/nina_net.c:9` | `"wifi_profile.h"` | `-I src` | T12 で `wifi_profile.h` の置き場所を決める |
| `drivers/lsm6dso/*.c`、`lsm6dso.h` | `"lsm6dso.h"`、`"lsm6dso_fifo*.h"`、`"lsm6dso_cal.h"`、`"lsm6dso_zero.h"`、`"lsm6dso_reg.h"` など | 同じディレクトリー（`lsm6dso_reg.h` は hal_st の `-I`） | 分かれると見つからない。`drivers__sensor__st__lsm6dso` に `target_include_directories()` で、残るディレクトリーと `common/drivers/lsm6dso` の両方を足す（**[確]**） |
| `drivers/lsm6dso/lsm6dso.h:19` | `<stmemsc.h>` | Zephyr 本体の `drivers/sensor/st/stmemsc` | 変わらない |
| `drivers/i2c_dw/i2c_dw.c:43,55,56,61` | `"i2c_dw_rp2040_commands.h"`、`"i2c_dw.h"`、`"i2c_dw_registers.h"`、`"i2c-priv.h"` | 同じディレクトリー、`i2c-priv.h` は Zephyr 本体 | コマンド語を移すと `drivers__i2c` に `-I common/drivers/i2c_dw` が要る（**[確]**） |
| `tests/stream_stop.c:195` | `"../drivers/lsm6dso/lsm6dso_stream.c"` | 相対 | `common/tests` に置けば `common/drivers/lsm6dso/lsm6dso_stream.c` に解決する（**[読]**） |
| `tests/nina_spi.c:7` | `"../drivers/nina/nina_wifi.c"` | 相対 | Zephyr だけのテスト（P4 まで） |

### 1.3 実験で使った CMake（要点）

```cmake
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../common)
cmake_path(NORMAL_PATH COMMON_DIR)
target_sources(app PRIVATE ... ${COMMON_DIR}/imu/imu_raw_batch.c ...)   # 元と同じ順
target_include_directories(app PRIVATE src drivers/lsm6dso ${COMMON_DIR}/imu ${COMMON_DIR}/drivers/lsm6dso)
target_sources(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw/i2c_dw_rp2040_commands.c)
target_include_directories(drivers__i2c PRIVATE "${ZEPHYR_BASE}/drivers/i2c" ${COMMON_DIR}/drivers/i2c_dw)
use_local_driver_sources(drivers__sensor "(^|/)sensor_shell\\.c$" ../../../common/drivers/sensor_shell)
target_include_directories(drivers__sensor PRIVATE ${ZEPHYR_BASE}/drivers/sensor ${COMMON_DIR}/drivers/lsm6dso)
target_include_directories(drivers__sensor__st__lsm6dso PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/lsm6dso ${COMMON_DIR}/drivers/lsm6dso)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo.c ...)
```

`sensor_shell.c` の `"../lsm6dso/lsm6dso_attributes.h"` は写しの中で `"lsm6dso_attributes.h"` に直した（バイナリーは同じ）。

- **[読]** Zephyr はアプリのディレクトリーに `-fmacro-prefix-map=<app>=CMAKE_SOURCE_DIR` を付ける（`compile_commands.json`）。
  `common/` には付かない。今は `__FILE__` を使う箇所が像に無い（6.3）ので影響しないが、将来のために
  `zephyr_compile_options(-fmacro-prefix-map=${COMMON_DIR}=COMMON)` を足しておく案がある（足してもバイナリーは変わらない
  はず。**[未]**）。

---

## 2. NuttX のビルドから `common/` を参照する方法

### 2.1 今の仕組み（**[読]**、根拠の行）

| 部分 | 実行するディレクトリー（cwd） | オブジェクトの名前と置き場所 | ソースの見つけ方 |
|---|---|---|---|
| カーネル側 `nuttx/drivers`（`libexternal.a`） | `nuttx/nuttx/external` → symlink → 実体は `nuttx/drivers` | `COBJS = $(CSRCS:.c=$(OBJEXT))`（`drivers/Makefile` 30 行）: cwd に基本名で置く（例 `nuttx/drivers/lsm6dso.o`） | 各 `Make.defs` の `VPATH += :nina`、`DEPPATH += --dep-path nina`（例 `drivers/nina/Make.defs` 31〜32 行） |
| アプリ `nuttx/apps/*`（`libapps.a`） | `nuttx/nuttx-apps/external/<app>` → 実体 `nuttx/apps/<app>` | `COBJS = $(CSRCS:%=$(PREFIX)%$(SUFFIX)$(OBJEXT))`、`SUFFIX = cwd の / を . に変えたもの`（`nuttx-apps/Application.mk` 53、83 行）。例 `nuttx/apps/imu/imu_raw_batch.c.home.ouwa.work.rp2040-connect.nuttx.apps.imu.o`。アーカイブには `_1` の付いた写しを入れる（同 280〜292 行） | `VPATH += :.`、`DEPPATH += --dep-path . --obj-path .`（同 180〜183 行） |
| ボード `boards/.../src`（`libboard.a`） | `nuttx/nuttx/boards/arm/rp2040/common`（`board` はボードの `src` への symlink） | cwd に基本名（例 `nuttx/nuttx/boards/arm/rp2040/common/nano_i2c.o`、**[確]** `find`） | `VPATH += :board`（ボードの `src/Make.defs` 末尾） |

- `.gitignore`（ルート 24〜44 行）は `nuttx/drivers/**/*.o`、`nuttx/apps/**/*.o` などを除外している。オブジェクトの置き場所は
  ソースの場所でなく cwd で決まるので、**`common/` にオブジェクトは置かれない**（2.3 で確認）。`.gitignore` の変更は要らない。
- `make clean`: `CLEAN` マクロは `rm -f *$(OBJEXT) *$(LIBEXT) ... $(OBJS) $(BIN)`（`nuttx/nuttx/tools/Config.mk` 737〜739 行）で cwd
  だけを消す。`distclean` は cwd の `.ddc`・`Make.dep`・`.depend`（`drivers/Makefile` 63〜66 行、`Application.mk` 414〜417 行）。
  どちらも `common/` には触れない（**[確]** 2.3）。

### 2.2 注意点: `VPATH` は相対パスにする（mkdeps）

- **[確]** `VPATH`・`DEPPATH` を絶対パス（`$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common/...`）にすると、`make depend` で
  `# ERROR: File "/.../common/drivers/lsm6dso/lsm6dso_fifo_hw.c" not found at any location` になって止まった。
  `%.ddc: %.c` の規則（`tools/Config.mk` 231〜232 行）が VPATH で見つけたパス（`$<`、絶対パス）を mkdeps に渡し、mkdeps は
  それを `--dep-path` の各ディレクトリーの後ろに**連結して** `stat()` する（`tools/mkdeps.c` 860〜915 行）ため。
- **[確]** cwd からの相対パスなら動く: カーネル側 `../../common/drivers/lsm6dso`（cwd は物理パスの `nuttx/drivers`）、アプリ
  `../../../common/imu`（cwd は `nuttx/apps/imu`）、ボード `board/../../../../common/drivers/i2c_dw`（`board` の symlink を
  カーネルが物理的にたどる）。`Make.dep` には `lsm6dso_fifo.o: ../../common/drivers/lsm6dso/lsm6dso_fifo.c ...` と載り、
  `common/` のヘッダーに `touch` すると `make -n` が依存するオブジェクトを作り直す（**[確]**）。
- `CFLAGS` の `-I` は絶対パス（`$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)...`）でよい（**[確]**）。

### 2.3 実験（**[確]**）

NuttX のワークスペースを写し、Zephyr と同じ 33 本に `imu_cal.h` を足した 34 本（アプリの `imu_raw_*`・`imu_tick_clock`・`time_offset`・`imu_cal_codec.c`・
**`imu_cal.h`**、ドライバーの `lsm6dso_fifo*`・`_cal`・`_zero`・`_fifo_hw`・`_fifo_decoder`・`_stream.c`・`_fifo_irq`・`_stats.h`・
`_attributes.h`、ボードの `i2c_dw_rp2040_commands.[ch]`）を写しの外の `common/` へ移して、次を足した:

- `drivers/lsm6dso/Make.defs`: `VPATH += :../../common/drivers/lsm6dso`、`DEPPATH += --dep-path ../../common/drivers/lsm6dso`、
  `-I common/drivers/lsm6dso`、**`-I $(TOPDIR)/external/lsm6dso`**（移した `lsm6dso_stream.c` が残った `lsm6dso.h` を探すため。
  今はカーネル側に無い）。
- ボードの `src/Make.defs`: `VPATH`・`DEPPATH`・`-I` を `common/drivers/i2c_dw` に。
- `apps/imu/Makefile`: `VPATH`・`DEPPATH` と `-I common/imu`・`-I common/drivers/lsm6dso`。`apps/init`・`wifi`・`kv`・`sensor` の
  `Makefile` にも `-I` の 2 つ（`imu_cal.h` → `lsm6dso_cal.h` を読むため）。
- `apps/sensor/sensor_shell.c:41` の `"../lsm6dso/lsm6dso_attributes.h"` を `"lsm6dso_attributes.h"` に（今は
  `-I $(TOPDIR)/external/include` に `../lsm6dso/...` を足した形で `nuttx/drivers/lsm6dso` を見つけている）。

結果: `SOURCE_DATE_EPOCH=1763337600 ./script/build.sh --clean -j 1` で **`nuttx.bin` が基準とバイト単位で一致**。
`common/` の下には `.c`/`.h` 以外のファイルが無い（`find common -type f ! -name '*.[ch]'` が空）。オブジェクトは元の場所
（`apps/imu/*.o` 2 つずつ、`nuttx/drivers/*.o` 39 個、ボードは `nuttx/nuttx/boards/arm/rp2040/common/i2c_dw_rp2040_commands.o`）。

途中で分かったこと:

- **`imu_cal.h` は `imu_cal_codec.c` と一緒に移す**（**[確]**）。アプリの `apps/imu` には `-I .` が無く、`common/imu/imu_cal_codec.c`
  から `"imu_cal.h"` が見つからなかった（`fatal error: imu_cal.h`）。Zephyr は `-I src` があるので通る。一般に、
  **移した `.c` が元のディレクトリーに残るヘッダーを読むなら、元のディレクトリーを `-I` に足す必要がある**（NuttX のカーネル側と
  アプリ、Zephyr のドライバーのターゲット）。
- `build.sh` の構成の入力（`config_inputs()`、`script/build.sh` 87〜109 行）は `drivers`・`apps` の `Kconfig` だけを見る。
  `common/` に `Kconfig` を置かないなら変更は要らない（**[読]**）。

### 2.4 実験で起きた事故と、写しで試すときの注意

`tar` で写したワークスペースには、`configure` が作る**絶対パスの symlink**（`nuttx/arch/arm/src/board` →
`/home/.../nuttx/nuttx/boards/arm/rp2040/common` など 6 本）が元を指したまま残る。写しで `distclean` すると、それを
たどって**元のワークスペースの `nuttx/nuttx/boards/arm/rp2040/common/board`（symlink）と、そこのオブジェクトを消した**。
元の `build.sh --clean` も同じ理由で止まったので、symlink を `ln -s` で作り直してから `build.sh --clean` で元に戻した
（`git -C nuttx/nuttx status` と `git -C nuttx/nuttx-apps status` は作業前と同じパッチの `M` だけ）。写しで試すときは、
先に `find <写し> -type l -lname '/home/.../*'` の symlink を写しの中へ向け直す。

---

## 3. ホストテスト

### 3.1 比較（**[確]** 両方のスクリプトを実行して全部 `ok`）

| テスト | Zephyr（`zephyr/script/host_tests.sh`） | NuttX（`nuttx/script/host_tests.sh`） | テストのソース |
|---|---|---|---|
| `fifo_words`、`imu_raw_packet`、`imu_raw_batch`、`imu_raw_queue`、`imu_raw_status`、`imu_raw_time`、`imu_tick_clock`、`time_offset` | 24〜27 行 | 29〜32 行 | 同一（`cmp`） |
| `fifo_hw`、`fifo_decoder`、`lsm6dso_cal`、`lsm6dso_zero`、`stream_stop` | 30〜41 行 | 35、107、156〜160 行 | 同一 |
| `i2c_dma_commands` | 44〜45 行 | 185〜186 行 | 同一（対象のソースも同一） |
| `nina_spi` | 46 行（`nina_wifi.c` を `#include`、`tests/nina_fakes`、`-std=gnu17 -Wall` だけ） | 176 行（`nina_spi.c`、偽の `nano_nina.h`・`nina_os.h`） | **別物**（差 497 行）。NuttX 版は Zephyr 版の場合を含み、さらに多い |
| `host/imu_host` の構文 | 48〜51 行 | 無し | — |
| `lsm6dso_driver`、`imu_cal`、`sensor_shell`、`zephyr_rtio`、`zephyr_work`、`zephyr_compat_misc`、`zephyr_mutex`、`mqtt_min`（と Zephyr の MQTT ライブラリーの上）、`nano_timeconv`、`nano_cpuload`、`log_ram`、`nano_flash_kv`、`kv_name`、`kv_hex`、`subcmd`、`nina_sock`、`nina_dns`、`nano_i2c_dw`、`i2c_dma_app`、`time_sync`、`imu_raw_mqtt`、`wifi_cred` | 無し | 36〜226 行 | NuttX だけ（互換層・NuttX の写し・ボードのコードを試す） |

偽ヘッダー:

| ディレクトリー | Zephyr | NuttX | 差 |
|---|---|---|---|
| `tests/fakes/`（`lsm6dso_reg.h`、`zephyr/drivers/i2c.h`・`sensor.h`・`sensor_clock.h`、`zephyr/rtio/rtio.h`） | ある | ある | 同一（`diff -r`）。NuttX だけ `fakes/nuttx/config.h`（空。`i2c_dma_app` 用） |
| `tests/nina_fakes/`（`nina_test.h`、`zephyr/device.h`・`kernel.h`・`drivers/gpio.h`・`spi.h`） | ある | 無い | Zephyr の `nina_wifi.c` を Zephyr の API のまま試すため |
| `lsm6dso_fakes/`、`zephyr_fakes/`（`zephyr_os_host.h`、`zephyr_kernel_host.[ch]`）、`zephyr_mutex_fakes/`、`time_sync_fakes/`、`mqtt_ref/` | 無い | ある | NuttX だけ |

フラグ: 共通のテストは両方 `-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections`、`-Wl,--gc-sections`。インクルードは
Zephyr が `-Itests/fakes -Isrc -Idrivers/lsm6dso`、NuttX が `-Itests/fakes -Iapps/imu -Idrivers/lsm6dso -Iapps`（`-Iapps` は
`imu_raw_packet.h` の `"../drivers/lsm6dso/..."` を `apps/../drivers` で解決するため。`host_tests.sh` 13〜14 行）。
`common/` の構成ならこの `-Iapps` は要らない（**[読]**、1.2 の 1 行目）。

NuttX のテストは対象のソースを相対パスで `#include` する（`tests/imu_raw_mqtt.c:90` `"../apps/imu/imu_raw_mqtt.c"`、`time_sync.c:82`、
`nina_spi.c:22`、`stream_stop.c:195`）。ソースを移すと、この行も直す。

### 3.2 `common/tests` の形の提案（**[読]**、未実装）

```
common/tests/
  run.sh             共通のテストを作って走らせる（引数: 出力ディレクトリー）。両方の host_tests.sh が呼ぶ
  fakes/             今の tests/fakes（同一の 5 本）
  *.c                今の同一の 14 本（T10）
  （T20 以降）zephyr_fakes/、lsm6dso_fakes/ と、互換層・共通ドライバーのテスト（zephyr_rtio、zephyr_work、
               lsm6dso_driver、imu_cal、sensor_shell、mqtt_min、time_sync、imu_raw_mqtt、wifi_cred、nina_sock、
               nina_dns、nina_spi（NuttX 版）、subcmd、log_ram）を、そのソースが common/ へ移るタスクで移す
zephyr/script/host_tests.sh   common/tests/run.sh ＋ Zephyr だけのもの（nina_spi の Zephyr 版と nina_fakes、
                              imu_host の構文）。P4（Z01）で nina_spi の Zephyr 版は消える
nuttx/script/host_tests.sh    common/tests/run.sh ＋ NuttX だけのもの（nano_timeconv、nano_cpuload、nano_flash_kv、
                              nano_i2c_dw、i2c_dma_app、kv_name、kv_hex、zephyr_mutex（NuttX の rmutex の模型）、
                              mqtt_min の Zephyr 版の参照）
```

- 共通のテストは今どちらのスクリプトでも同じフラグなので、`run.sh` に 1 度だけ書けば 2 回走らせる意味はない。両方の
  スクリプトから呼ぶのは、計画の「ホストテスト 2 つ」の確認（T10）を保つため。
- `ok  <名前>` の出力の順は今 2 つのスクリプトで違う（NuttX は途中に `lsm6dso_driver` などが入る）。比べる人がいないので
  揃えなくてよい。

---

## 4. ファイルの分類

### 4.1 一覧

組の作り方: 同名のもの（`zephyr/app/usb-console/src` ↔ `nuttx/apps/imu`・`apps/wifi`・`drivers/include`・`drivers/log`、`drivers/<x>` ↔
`nuttx/drivers/<x>`、`tests` ↔ `nuttx/tests`）と、NuttX 版の先頭のコメントが元を書いているもの（`nina_wifi.c` → `nina_spi.c`、
`nina_socket.c` → `nina_sock.c`、`i2c_dw.c` → `nano_i2c.c`、`i2c_dw.h` → `nano_i2c_dw.h`、`i2c_dw_registers.h` →
`nano_i2c_dw_registers.h`、`sensor_shell.c` → `apps/sensor/sensor_shell.c`、`log_ram.c` → `drivers/log/log_ram.c`＋`log_ring.c`＋
`apps/dmesg`、`kv_store.c` → `apps/imu/kv_store.c`＋`kv_name.c`＋`apps/kv`、`main.c` → `apps/nina`・`apps/init`）。差の行数は
`diff a b | grep -c '^[<>]'`（**[確]**、作業用ディレクトリーの `classify.txt`）。

**(a) バイト単位で同一（37 本＋テスト 15 本＋偽ヘッダー 5 本）** — 計画の「約 37 本」と一致。

| ファイル | 層 | 置き場所 | タスク |
|---|---|---|---|
| `imu_raw_batch.[ch]`、`imu_raw_packet.[ch]`、`imu_raw_queue.[ch]`、`imu_raw_status.[ch]`、`imu_raw_time.[ch]`、`imu_tick_clock.[ch]`、`time_offset.[ch]`、`imu_cal_codec.c`、**`imu_cal.h`** | A | `common/imu` | T10（`imu_cal.h` は 2.3 の理由で `imu_cal_codec.c` と一緒に） |
| `kv_store.h`、`time_sync.h` | A（宣言だけ） | `common/imu` | T12（使う `.c` と一緒に。先に移してもよい） |
| `lsm6dso_fifo.[ch]`、`lsm6dso_fifo_packet.h`、`lsm6dso_cal.[ch]`、`lsm6dso_zero.[ch]`、`lsm6dso_fifo_decoder.[ch]` | A | `common/drivers/lsm6dso` | T10 |
| `lsm6dso_fifo_hw.[ch]`、`lsm6dso_stream.c`、`lsm6dso_fifo_irq.[ch]`、`lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h` | C（`fifo_hw` は ST の `stmdev_ctx_t` だけ） | `common/drivers/lsm6dso` | T11（`lsm6dso_attributes.h` は 1.2 の `sensor_shell.c` の行を直すか、T12 まで待つ） |
| `nina_proto.h` | B | `common/drivers/nina` | T21 |
| `i2c_dw_rp2040_commands.[ch]` | A | `common/drivers/i2c_dw` | T10（計画では i2c_dw は T22 だが、同一なので先に移せる） |
| テスト 15 本（`fifo_words`、`imu_raw_*` 6 本、`imu_tick_clock`、`time_offset`、`fifo_hw`、`fifo_decoder`、`lsm6dso_cal`、`lsm6dso_zero`、`stream_stop`、`i2c_dma_commands`）と `tests/fakes` の 5 本 | — | `common/tests` | T10 |

**(b) 少差分（11 本）** — 差の種類と、差を無くす方法（**[読]**。方法はどれも未実装・未確認）。

| ファイル | 差（行） | 差の種類 | 差を無くす方法 | 層・置き場所・タスク |
|---|---|---|---|---|
| `imu_cal.c` | 11 | 先頭コメント、`<nuttx/config.h>` | 下の「前置きヘッダー」で `<nuttx/config.h>` を消す。コメントは文書へ | A/C、`common/imu`、T12 |
| `imu_raw_mqtt.h` | 2 | `<zephyr/net/net_ip.h>` ↔ `<netinet/in.h>`（`struct in_addr`） | 互換層に `<zephyr/net/net_ip.h>` を置き `<netinet/in.h>` を読ませる | C、`common/imu`、T12（計画の T12 の一覧に無い） |
| `imu_raw_mqtt.c` | 28 | 先頭コメント、NuttX のヘッダー 7 行（`config.h`、`arpa/inet.h`、`netdb.h`、`netinet/in.h`、`poll.h`、`sys/ioctl.h`、`sys/socket.h`）を互換層より前に | 前置きヘッダー | C、`common/imu`、T12 |
| `imu_stream.c` | 64 | 先頭コメント、NuttX のヘッダー 4 行と `"subcmd.h"`、`%d` → `PRId32`（1 か所）、シェルの登録（`SHELL_STATIC_SUBCMD_SET_CREATE`・`SHELL_CMD_REGISTER` → `subcmd` の表と `main()`、`sh` に NULL を渡す 4 つの包み） | `PRId32` を Zephyr 版にも入れる（Zephyr の `int32_t` は `int`: `zephyr_stdint.h` 36 行。`PRId32` は `"d"` で像は変わらないはず）。登録は互換層の `SHELL_*` マクロ（`zshell.c`）を NuttX でも使う。ただし今の `SHELL_CMD_ARG_REGISTER()` は `main()` を作る（`compat/zephyr/shell/shell.h` 280〜288 行）ので、`MAINSRC` でないファイル（`time_sync.c`）用に名前付きの入口を作る形が要る。出力（ヘルプ、引数の誤り）が `subcmd` と同じかを `subcmd` のテストで確かめる | C、`common/imu`、T12 |
| `time_sync.c` | 59 | 先頭コメント、NuttX のヘッダー 6 行と `"subcmd.h"`、Kconfig の名前（`CONFIG_NANO_NINA_NTP_*` → `CONFIG_NINA_NTP_*` の `#define` 3 行）、`<zephyr/devicetree.h>`・`<zephyr/sys/sys_io.h>` の削除、`time_sync_mono_us()` の中身（`DT_REG_ADDR(DT_NODELABEL(timer))` と `sys_read32()` ↔ `nano_mono_us()`）、シェルの登録 | Kconfig: NuttX の選択肢を同じ名前（`NINA_NTP_*`）にするか、前置きヘッダーで対応させる。時計: 互換層の `devicetree.h` に `timer` の節（reg 0x40054000）、`<zephyr/sys/sys_io.h>` に `sys_read32()` を置けば Zephyr の文のまま（NuttX の像は変わる。T12 は像が一致しないタスク）。登録は `imu_stream.c` と同じ | C、`common/imu`、T12 |
| `wifi_profile.c` | 13 | 先頭コメント、`<nuttx/config.h>`、`<zephyr/logging/log.h>` ↔ `"app_log.h"`、`extern const k_tid_t` ↔ `extern k_tid_t` | 互換層に `<zephyr/logging/log.h>`（`app_log.h` を読む）を置く。`const`: 互換層の `K_THREAD_DEFINE()` は起動時に `k_tid_t`（`pid_t`）を書く（`compat/zephyr/kernel.h` 311〜333 行）。`k_tid_t` を静的なスレッドの構造体へのポインターにすれば `const` にできる（`k_current_get()` との比較の方法も変わる。T20 の範囲） | C、`common/imu`、T12 |
| `wifi_profile.h` | 7 | 先頭コメントだけ | コメントを文書へ | C、`common/imu`（NuttX はカーネル側の `nina_net.c` も読むので `drivers/include` から `-I` で）、T12（計画の T12 の一覧に無い） |
| `nina_net.c` | 11 | 先頭コメント、`<nuttx/config.h>` | 前置きヘッダー | C、`common/drivers/nina`、T12 |
| `lsm6dso_compat.c` | 20 | 先頭コメント、`<nuttx/config.h>`、`<inttypes.h>`、ログ（`<zephyr/logging/log.h>`・`CONFIG_SENSOR_LOG_LEVEL` ↔ `app_log.h`・`LSM6DSO_COMPAT_LOG_LEVEL`）、`%u`/`%d` → `PRIu32`/`PRId32` 2 か所 | `PRI*` と `<inttypes.h>` を Zephyr 版にも。互換層の `<zephyr/logging/log.h>` と、NuttX で `CONFIG_SENSOR_LOG_LEVEL` を 3 に（前置きヘッダーか Kconfig） | C、`common/drivers/lsm6dso`、T12 |
| `sensor_shell.c` | 26 | 先頭コメント、`"sensor_shell_nuttx.h"`、`sensor get` が結果の表示を待つ 2 行（`sensor_shell_nuttx_get_arm()`・`_wait()`）、`PRIu32`/`PRId32` 1 か所 | `PRI*` を Zephyr 版にも。待ちの 2 行は、OS ごとのヘッダー（`sensor_shell_os.h`。Zephyr 版は空のマクロ）を読む形にして Zephyr の文にも同じ 2 行を入れる（Zephyr の像は変わらない）。`"../lsm6dso/lsm6dso_attributes.h"` も直す（1.2） | C、`common/drivers/sensor_shell`、T12 |
| `lsm6dso.h` | 61 | 先頭コメント、`<nuttx/config.h>`、`<zephyr/types.h>` ↔ `<stdbool.h>`・`<stdint.h>`、`<stmemsc.h>` ↔ `"stmemsc.h"`、`"lsm6dso_os.h"`、`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` ↔ `LSM6DSO_BUS_I2C`（SPI の部分を削除）、`CONFIG_SENSOR_ASYNC_API` ↔ `CONFIG_NANO_LSM6DSO_STREAM`（5 か所）、`struct k_mutex`・`struct k_spinlock` ↔ `lsm6dso_mutex_t`・`lsm6dso_spinlock_t`（3 か所） | 互換層に `<zephyr/types.h>` と `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()`（`st_lsm6dso, i2c` だけ 1）、NuttX で `CONFIG_SENSOR_ASYNC_API` を定義、`lsm6dso_os.h` の型は互換層の型の typedef なので Zephyr の型をそのまま使う（4.3）、`<stmemsc.h>` は `-I` で | C、`common/drivers/lsm6dso`、T13 |

「前置きヘッダー」: Zephyr はすべてのファイルに `-imacros autoconf.h` を付けている（`compile_commands.json`）。NuttX でも
`common/` のファイルに `-include <NuttX の前置き>`（`<nuttx/config.h>` と、互換層の `<zephyr/sys/atomic.h>` より先に要る NuttX の
ヘッダー: `compat/zephyr/kernel.h` 50〜63 行と `imu_raw_mqtt.c` の 7 行）を付ければ、`<nuttx/config.h>` と NuttX のヘッダーの
行がファイルから消える。置き場所は `nuttx/drivers/compat`（D2）。NuttX の像が変わるかは **[未]**（宣言とマクロだけなので
変わらないと考えるが、T12 の最初に `compare.py` で確かめる）。

**(c) 構造が別（同じ役割だが 1 本にできない、または大きく違う）**

| Zephyr | NuttX | 差 | 扱い |
|---|---|---|---|
| `drivers/lsm6dso/lsm6dso.c` | `drivers/lsm6dso/lsm6dso.c` | 313 | 4.2。T13 |
| `drivers/nina/nina_dns.c` | `drivers/nina/nina_dns.c` | 142 | 名前解決の流れは同じ。NuttX 版は `nina_os.h` とPOSIX の `addrinfo`・`EAI_*`・`malloc()`、Zephyr 版はソケットのオフロード・`DNS_EAI_*`・`k_heap`。B 層、`common/drivers/nina`、T21。**Zephyr 版の載せ替えは計画の Z01 に入っていない**（下の食い違い） |
| `drivers/nina/nina_wifi.c` | `drivers/nina/nina_spi.c` | 126 | 同じ手順を `nina_os.h`・`nano_nina.h` の境界で書いたもの。B、T21、Zephyr は Z01 |
| `drivers/nina/nina_socket.c` | `drivers/nina/nina_sock.c`（＋`nina_usrsock.c`・`nina_poller.c`） | 1013 | B（`nina_sock.c`）と D2（usrsock）。T21、Z01 |
| `drivers/i2c_dw/i2c_dw.c`、`i2c_dw.h`、`i2c_dw_registers.h` | ボードの `nano_i2c.c`、`nano_i2c_dw.h`、`nano_i2c_dw_registers.h` | 1870、461、207 | D1 と D2。T22、Z02 |
| `src/kv_store.c` | `apps/imu/kv_store.c`＋`kv_name.c`、`apps/kv/kv_main.c`・`kv_hex.c` | 235 | 名前の規則（`kv_name.c`）は A。T12 か F04 で `common/` へ（計画に無い） |
| `src/log_ram.c` | `drivers/log/log_ram.c`＋`log_ring.c`、`apps/dmesg` | 252 | `log_ring.c` は A（T23）。`log_ram.c` は OS ごと |
| `tests/nina_spi.c` | `tests/nina_spi.c` | 497 | 3.1 |

**(d) 片方だけ**

- Zephyr だけ: `src/main.c`（D2）、`drivers/lsm6dso/lsm6dso_shub.c`・`lsm6dso_trigger.c`（Zephyr 本体の写し。`CONFIG_LSM6DSO_SENSORHUB`・
  `_TRIGGER` が無効でビルドされない。差し替えの対象にはなっている: `CMakeLists.txt` 38 行）、`tests/nina_fakes/*`。
- NuttX だけ: 互換層の全部（5 節）、`apps/subcmd`（5.3）、`apps/imu/mqtt_min*.c`・`imu_raw_mqtt_null.c`・`time_sync_null.c`・`kv_name.[ch]`、
  `apps/wifi/*`（Zephyr の `wifi_credentials` の写しと `settings_nvs.c`）、`apps/sensor/sensor_shell_stream.c`・`sensor_shell.h`（Zephyr 本体の
  写し）・`sensor_shell_nuttx.[ch]`、`apps/init`・`dmesg`・`i2c_dma`・`irqstack`・`kv`・`nina`・`ntp`、`drivers/lsm6dso/lsm6dso_os.[ch]`・
  `stmemsc*`・`st_lsm6dso_reg.c`、`drivers/nina/nina_os.[ch]`・`nina_netdev.c`・`nina_usrsock.[ch]`・`nina_poller.c`・`wifi_profile_null.c`、
  `drivers/include/*`、ボードの全部、テスト（3.1）。
  - `mqtt_min.c`・`wifi_credentials*.c`・`sensor_shell_stream.c`・`sensor_shell.h`・`zephyr_work.c`・`zephyr_rtio*.c`・`sensor_default_rtio.c` は Zephyr
    **本体**の写しで、FreeRTOS 版も使う（`common/`）。Zephyr 版は本体のものを使う。

### 4.2 `lsm6dso.c` の差（313 行）の内訳（**[確]** `diff` の行を正規表現で数えた。概数）

| 種類 | 行（−Zephyr / ＋NuttX） | 差を無くす方法 |
|---|---|---|
| ミューテックス（`k_mutex_init/_lock/_unlock` ↔ `lsm6dso_os_mutex_*`） | −35 / +36 | 互換層の `k_mutex` をそのまま使う（`lsm6dso_os_mutex_*` は `k_mutex_*(.., K_FOREVER)` を呼ぶだけ: `lsm6dso_os.c` 33〜46 行） |
| スピンロック（`k_spin_lock/_unlock`、`k_spinlock_key_t`） | −9 / +10 | 同上（`lsm6dso_os.c` 48〜57 行） |
| `k_busy_wait`、`k_msleep`、`k_usleep` | −4 / +5 | 互換層に足す（NuttX: `up_udelay()`、`nxsched_msleep()`・`nxsched_usleep()`。丸めは今の `lsm6dso_os.c` と同じ） |
| `k_cycle_get_32`、`k_cyc_to_ns_floor64` | −2 / +3 | 互換層に `k_cycle_get_32()` はある（TIMER）。`k_cyc_to_ns_floor64()` を足す（×1000） |
| Kconfig の名前（`CONFIG_SENSOR_ASYNC_API` ↔ `CONFIG_NANO_LSM6DSO_STREAM`、`#if CONFIG_LSM6DSO_TRIGGER` ↔ `#ifdef`） | −8 / +10 | NuttX で `CONFIG_SENSOR_ASYNC_API` を定義。`#if` は Zephyr 版を `#ifdef` に（Zephyr では値が 1 か未定義なので同じ意味） |
| バスの条件（`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY`） | −2 / +2 | 互換層の `devicetree.h` にマクロ |
| インクルード、ログ（`LOG_MODULE_REGISTER` のレベル、`<zephyr/logging/log.h>` ↔ `app_log.h`、`<zephyr/sys/byteorder.h>` を外して `sys_get_le32()` をファイル内に） | 約 −6 / +9 | 互換層の `log.h`。`sys_get_le32()` は互換層の `byteorder.h` にすでにある（`nuttx/drivers/README_ja.md` 348 行）ので Zephyr の行に戻せる |
| 先頭のコメント（置き換えの表） | +約 35 | 文書へ |
| 末尾のデバイスの生成（Zephyr の `LSM6DSO_DEFINE()` と `DT_INST_FOREACH_STATUS_OKAY_VARGS` 約 98 行 ↔ ボードの値の `DEVICE_DT_DEFINE()` 約 37 行） | −約 98 / +約 37 | **本当に OS ごとの部分**。`lsm6dso_init` と `lsm6dso_driver_api` が `static` なので、別の `.c` にすると `static` を外すことになる。`lsm6dso.c` の末尾で OS ごとの断片（例 `#include "lsm6dso_instance.h"`、Zephyr 版とNuttX 版を OS のディレクトリーに置き、`-I` で選ぶ）を読む形なら `static` のまま |

上をすべて行えば、差は末尾の断片だけになる（**[読]**）。

### 4.3 `lsm6dso_os.[ch]`・`nina_os.[ch]` を互換層に吸収するか（意見）

- **`lsm6dso_os` は互換層に吸収するのがよい**（T13）。理由: (1) 中身は互換層の `k_mutex`・`k_spin_lock()` の呼び出しと、
  `up_udelay()`・`nxsched_msleep()`・`nxsched_usleep()`・TIMER の読み出しだけ（`lsm6dso_os.c` 全体）、(2) 型
  （`lsm6dso_mutex_t` など）はすでに互換層の型の typedef（`lsm6dso_os.h` 31〜36 行）、(3) LSM6DSOX は C 層のまま互換層の上で
  動かすと決めた（計画 27〜30 行）ので、独自の境界を持つ理由が無い、(4) 吸収すると `lsm6dso.c` の差の約 2/3 が消える（4.2）。
  ホストテストへの影響: `tests/lsm6dso_driver.c` 106〜160 行が `lsm6dso_os_*` の偽物を定義し、`busy%u`・`sleep%dms` のイベントと
  眠った時間を記録している。吸収後は同じ記録を `k_busy_wait()`・`k_msleep()`・`k_usleep()`・`k_cycle_get_32()` の偽物で行う。
  偽のカーネル（`tests/zephyr_fakes/zephyr_kernel_host.c`）には今それらが無い（`k_cycle_get_*` は宣言だけ: `kernel.h` 386〜391 行）ので、
  テストのファイルで定義すればよい。`imu_cal`・`sensor_shell` のテストは `lsm6dso_driver.c` を `#include` しているので一緒に変わる。
- **`nina_os` は B 層の境界として残すのがよい**。計画の B 層は「狭い境界（`nina_os.h`・`nano_nina.h`）だけに依存」（計画 22 行）で、
  互換層に吸収すると NINA のコアが互換層全体に依存する。関数は 7 つ（ロック 2、DNS のロック 2、sleep 2、ms の時計 1）で、
  FreeRTOS と Zephyr（P4）で直接書くのも小さい。ホストテスト（`tests/nina_spi.c` 138〜172 行）は、ロックの失敗（`lock_result`）、
  `sleep_us` が常に 100 であること、`sleep%u` のイベント、偽の時計を使っており、互換層の `k_mutex_lock(K_FOREVER)`（失敗しない）
  では同じ試験ができない。`tests/nina_sock.c`・`nina_dns.c` も `nina_os_uptime_ms()`・DNS のロックを偽物にしている。

---

## 5. 互換層の OS 依存箇所

### 5.1 ヘッダー（`nuttx/drivers/compat/zephyr/...`）（**[確]** `grep`、**[読]** 分類）

| ヘッダー | NuttX への依存 | 分類の案 |
|---|---|---|
| `kernel.h` | `#ifdef __NuttX__` で `<nuttx/config.h>`・`<nuttx/irq.h>`・`<nuttx/mutex.h>`・`<nuttx/semaphore.h>`・`<nuttx/wdog.h>`・`<arch/board/nano_timer.h>`（54〜63 行）、`Z_USEC_PER_TICK = CONFIG_USEC_PER_TICK`（87〜91 行）、`struct k_mutex`（`rmutex_t`）、`struct k_sem`（`sem_t`）、`struct _timeout`（`struct wdog_s`）、`k_tid_t = pid_t`（112〜157 行。ホストは `zephyr_os_host.h`: 158〜162 行）、`gettid()`・`k_current_get()`（262〜275 行）、`k_cycle_get_*()` = `nano_mono_us()`（376〜391 行）、`"priorities.h"`（`K_PRIO_PREEMPT` = `NANO_PRIO_FROM_ZEPHYR`。75、256 行） | 型と時計と優先度の対応は **OS ごとのヘッダー**（`zephyr_os.h`）へ。残り（`k_timeout_t`、`K_MSEC()`、ワーク、`k_poll` の型、`K_THREAD_DEFINE()` の形、宣言）は共通 |
| `spinlock.h` | `sched_lock()`＋`up_irq_save()`（25〜47 行） | OS ごと（FreeRTOS は `taskENTER_CRITICAL()` 相当） |
| `sys/atomic.h` | NuttX の `<nuttx/atomic.h>` を先に読んで名前を引き取る（22〜24 行） | 演算は共通、名前の衝突の処理は NuttX ごと |
| `sys/clock.h` | `clock_gettime()`・`clock_settime()`（27〜53 行） | OS ごと |
| `net/net_if.h` | `struct net_driver_s`、`<netinet/in.h>` の `in_addr`（27〜42 行） | OS ごと（FreeRTOS はネットワークデバイスが無い） |
| `net/socket.h` | POSIX のソケットのヘッダー一式、`"nina_dns.h"`・`"nina_sockopt.h"`、`setsockopt(NINA_SOL, ...)`、`ioctl(FIONREAD)`（66〜110 行） | OS ごと（FreeRTOS は計画 F09 のとおり `nina_sock.c` を直接呼ぶ） |
| `drivers/i2c.h` | `struct i2c_master_s`、`i2c_compat_bind()`（67〜90 行） | バスの型は OS ごと、API は共通 |
| `net/wifi_mgmt.h`、`net/wifi_credentials.h` | `<nuttx/config.h>`、`CONFIG_NANO_WIFI_CREDENTIALS*` | 共通（設定の名前だけ前置きか Kconfig で揃える） |
| `shell/shell.h` | `struct shell` の `pid_t cmd_pid`（59 行） | 型を OS ごとに（5.3） |
| `sys/__assert.h`、`rtio/rtio.h`、`sensor_clock.h`、`hwinfo.h`、`devicetree.h`、`device.h`、`gpio.h`、`sensor.h`、`dsp/*`、`sys/util.h`・`byteorder.h`・`printk.h`・`mem_blocks.h`・`mpsc_lockfree.h`・`iterable_sections.h`・`cpu_load.h`、`net/mqtt.h`・`net_mgmt.h`・`wifi_utils.h`、`settings/settings.h`、`rtio/work.h`、`zephyr_compat.h` | 無し、または `#ifdef __NuttX__` の `<nuttx/config.h>` だけ | 共通（`common/compat`） |

ホストテストの仕組み（**[確]** 読んで確認）: `kernel.h` は `__NuttX__` が無いと `"zephyr_os_host.h"`（`tests/zephyr_fakes/`）を読み、
そこが `struct k_spinlock`・`k_spin_lock()`（二重取得を `assert`）、`struct k_mutex`（深さ）、`struct k_sem`（数）、`struct _timeout`
（予約の印）、`k_tid_t`（`int`）を定義する（`zephyr_os_host.h` 全体）。関数は `zephyr_kernel_host.c`（1 スレッド、待つと失敗）。

**分ける線の提案**: `kernel.h`・`spinlock.h` の `#ifdef __NuttX__ ... #else #include "zephyr_os_host.h"` を、
`#include <zephyr_os.h>`（OS の型・`Z_USEC_PER_TICK`・`k_current_get()`・`k_cycle_get_*()`・`K_PRIO_PREEMPT()`・スピンロック）に
置き換え、`zephyr_os.h` を OS ごとのディレクトリー（`nuttx/drivers/compat/`、`freertos/compat/`、`common/tests/zephyr_fakes/`）に置いて
`-I` の順で選ぶ。`sys/clock.h`・`net/net_if.h`・`net/socket.h`・`drivers/i2c.h` のバスの型は OS ごとのディレクトリーに丸ごと置く。

### 5.2 `.c`（**[確]** `grep`、**[読]** 分類）

| ソース | NuttX への依存 | 分類の案 |
|---|---|---|
| `zephyr_work.c`、`zephyr_rtio.c`、`zephyr_mem_blocks.c`、`sensor_default_rtio.c`、`zephyr_wifi_utils.c` | `<nuttx/config.h>` だけ（`#ifdef __NuttX__`） | 共通 |
| `zephyr_device.c`、`zephyr_wifi_mgmt.c` | `<nuttx/config.h>`、`CONFIG_NANO_*` の名前 | 共通（名前を揃える） |
| `zephyr_rtio_workq.c`、`zephyr_net_mgmt.c` | `kthread_create_with_stack()`（`rtio_workq.c` 158 行、`net_mgmt.c` 198 行）、`NANO_PRIO_*`・`NANO_STACK_*` | スレッドの作成を互換層の OS 部分の関数（例 `z_thread_create_static()`）にすれば共通 |
| `zephyr_gpio.c` | ボードの `<nano_gpio.h>`（`nano_gpio_configure_input()` など）と `k_spin_lock()` だけ（7 行目のコメントも同じ） | 共通（`nano_gpio.h` を境界のヘッダーにする） |
| `zephyr_hwinfo.c`、`zephyr_sensor_clock.c` | `nano_chip_id()`（`<arch/board/board.h>`）、`nano_mono_us()`（`<arch/board/nano_timer.h>`） | 共通（境界のヘッダーを OS ごとに。計画 T23 の「時計・チップ ID の境界」） |
| `zephyr_i2c.c` | `<nuttx/i2c/i2c_master.h>`、`I2C_TRANSFER()`（80 行） | OS ごと |
| `zephyr_kernel.c`（55 か所）、`zephyr_mutex.c` | `sem_t`・`nxsem_*`、wdog、kthread、`nxsched_*`、`rmutex_t` | OS ごと（`nuttx/drivers/compat`） |
| `zephyr_net_if.c` | `netdev_lock()`、`struct net_driver_s` の `d_ipaddr` など | OS ごと |

### 5.3 アプリ側の互換（**[確]** `grep`）

- **`subcmd.c`**（295 行）が使うもの: `<ctype.h>`、`<fnmatch.h>`（`fnmatch()` 2 か所）、`<stdio.h>`（`printf()` 5、`fprintf(stderr)` 1、
  `fputs()` 1、`fflush(stdout)` 2）、`<string.h>`（`strlen`・`strcmp`）、`<stdlib.h>`。**OS の呼び出しは無い**。共通に置ける。
  前提は「`stdout`・`stderr` がそのコマンドの端末」（NuttX はタスクごとに持つ）。FreeRTOS で Telnet のセッションを持つなら、
  スレッドごとの出力先が要る（**[読]**）。
- **`zshell.c`**（962 行）が使うもの:
  - 1〜290 行（バックエンド）: `<sys/stat.h>`（`stat()`・`fstat()`、`st_ino`）、`<dirent.h>`（`opendir("/dev")`・`readdir()`・`closedir()`、
    108〜140 行）、`<fcntl.h>`（`fcntl(STDOUT_FILENO, F_GETPATH)`、`open(O_WRONLY | O_APPEND)`）、`<unistd.h>`（`write()`・`close()`・
    `getpid()`）、`<pthread.h>`（`pthread_mutex_t g_backends_lock = PTHREAD_MUTEX_INITIALIZER`、97、197、216 行）、`calloc()`・`free()`、
    `snprintf()`・`vsnprintf()`。端末を `/dev/ttyACM0`・`/dev/telnet0` のパスで見分け、コマンドのタスクの `pid` で「自分の出力か」を決める
    （251、786 行）。**NuttX の「コマンド = タスク、端末 = デバイスファイル」の前提に依存**。
  - 291〜331 行（`shell_device_filter()`・`shell_device_get_binding()`）: 互換層のデバイスの一覧だけ。共通。
  - 332〜962 行（照合、ワイルドカード、ヘルプ、`argv`、`execute()`・`exec_cmd()`）: `<fnmatch.h>`、`<stdio.h>`、`<string.h>`、`memcpy()`。
    ほぼ共通。ただし `exec_cmd()` がバックエンドの `cmd_pid` に `getpid()` を書く（786 行）。
  - 提案（T23）: `zshell.c` を、照合とヘルプの部分（`common/shell/zshell.c`）と、バックエンドの部分（`nuttx/apps/subcmd/zshell_backend.c`、
    `z_shell_backend_vfprintf()` と「今のコマンドのバックエンド」を返す関数、`cmd_pid` を OS の型で）に分ける。FreeRTOS はバックエンドを
    コンソールと Telnet の 2 つの `struct shell` で書く（Zephyr のシェルのインスタンスと同じ形）。
- **`apps/wifi/settings_nvs.c`**: `open("/dev/config")`・`ioctl(CFGDIOC_GETCONFIG/SETCONFIG/DELCONFIG)`・`close()`、`<nuttx/mtd/configdata.h>`、
  `MTD_CONFIG_NAMED` の検査（11〜45、85〜114 行）。**NuttX 専用**。FreeRTOS は計画どおり KV の形式を決めてから別に書く（F04）。
- **`apps/imu/mqtt_min.c`**（1751 行）: `<sys/types.h>`・`<sys/socket.h>`・`<sys/uio.h>`・`<netinet/in.h>`（`struct sockaddr`・`struct msghdr`・
  `struct iovec`）、互換層の `k_mutex` と `k_uptime_get()`（196〜226 行）、ソケットは `mqtt_min_sock_*()`。**OS の呼び出しは無い**。共通に
  置ける（FreeRTOS は POSIX のソケットのヘッダーの型を互換層で用意する。newlib には無い: T01b の範囲）。
- **`apps/imu/mqtt_min_sock.c`**（56 行）: `zsock_*()` を呼ぶだけ。`<nuttx/config.h>` と `<sys/socket.h>` を除けば共通。

---

## 6. バイナリーの比べ方と基準

### 6.1 基準（**[確]**）

場所: `freertos/build/baseline/`（`freertos/build/` は Git 管理外: ルートの `.gitignore` 68 行）。コミット
`0f14ce0f02dbb4084bf305b096d0eced5dd22bea`、作業ツリーは `?? CLAUDE.md`・`?? freertos/PORTING_*.md` だけ（各 `snapshot/commit.txt`）。

| ディレクトリー | 中身 | 作り方 |
|---|---|---|
| `zephyr/snapshot/` | **Zephyr の基準**: `zephyr.elf`・`zephyr.map`・`zephyr.bin`（＝`image.bin`）、`objdump-d.txt`、`size-A.txt`、`nm-S.txt`、`readelf-s.txt`、`commit.txt`、`obj/`（我々の 32 個のオブジェクトの正規化した内容） | `cd zephyr && ZEPHYR_SDK_INSTALL_DIR="$PWD/zephyr-sdk-1.0.1" .venv/bin/west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console`（約 14 秒）の後に `python3 freertos/build/baseline/compare.py snapshot zephyr freertos/build/baseline/zephyr/snapshot` |
| `zephyr/run1/`、`run2/` | 同じ手順の 2 回（ELF、map、bin、`run1` は `compile_commands.json` も） | 再現性の確認用 |
| `nuttx/snapshot/` | **NuttX の基準**（同じ中身、`nuttx.elf`、オブジェクト 90 個） | `cd nuttx && SOURCE_DATE_EPOCH=1763337600 ./script/build.sh --clean -j 1`（約 86 秒）の後に `compare.py snapshot nuttx ...` |
| `nuttx/j1a/`、`j1b/` | 上と同じ手順の 2 回 | 再現性の確認用 |
| `nuttx/run1/`、`run2/` | `./script/build.sh --clean`（既定の `-j`、日時の固定なし）の 2 回 | 再現しないことの確認用 |
| `nuttx/example-jN/` | 既定の `-j` の 1 回のスナップショット | `compare.py` の試験用 |

`zephyr.bin` 362540 バイト（リンカーの表示は `BOOT_FLASH: 256 B`、`FLASH: 362284 B`）、`nuttx.bin` 278956 バイト（`-j 1`。既定の `-j` では 278948 バイトだった）。

### 6.2 再現性（**[確]**）

- **Zephyr**: `west build -p always` を 2 回 → `zephyr.bin`・`zephyr.elf`・`zephyr.map` がすべて一致（`cmp`）。さらに、アプリを
  別の絶対パス（作業用ディレクトリー）に写してビルドしても **`zephyr.bin` は一致**（ELF は 33 バイト目から違う）。
- **NuttX、既定の `build.sh --clean`**: 2 回で **一致しない**（`cmp -l` で 40454 バイト）。原因は 2 つ:
  1. `uname` の版の文字列の日時（`2b5c0de5-dirty Nov 17 2026 07:27:01` ↔ `07:27:34`。`__DATE__`・`__TIME__`）。
  2. `libapps.a` の中の順（`nuttx.map` の差: `sensor_shell_stream.c...o`、`settings_nvs.c...o`、`ntp_main.c...o` などの位置）。各アプリが
     並列に `flock` で同じアーカイブへ足す（`Application.mk` 293〜299 行、`tools/Config.mk` 450〜455 行のコメント）ので、終わった順に
     入り、リンク順が変わる。文字列の配置も動く。
- **NuttX、`SOURCE_DATE_EPOCH=1763337600 ./script/build.sh --clean -j 1`**: 2 回で `nuttx.bin`・ELF・map が**一致**。GCC は
  `SOURCE_DATE_EPOCH` で `__DATE__`・`__TIME__` を固定する。NuttX の写しを別の絶対パスでビルドしても `nuttx.bin` は一致した
  （アプリのオブジェクトの名前に絶対パスが入るが、像には入らない）。
- 既定の `-j` と `-j 1` の像を `compare.py` で比べると、セクションの大きさ（`.text` 274968 ↔ 274976、`.flash_section` 3676 ↔ 3668）と
  ポインターを含むデータ 60 個だけが違い、**関数のコード（アドレスを除く）とオブジェクトはすべて同じ**だった。
  → 実機に書く像（既定の `-j`）は、同じソースでもビルドのたびに配置が変わる。`CLAUDE.md` の「配置だけで `cpu_active` が
  0.6〜0.7 ポイント動く」は、ソースを変えなくても起こりうる（計画のリスク 163〜164 行に関係）。

### 6.3 移動で像に出うる違いの要因（**[読]**、一部 **[確]**）

| 要因 | Zephyr | NuttX |
|---|---|---|
| `__FILE__` | 我々のソースに無い（`grep`）。Zephyr の `__ASSERT` は `CONFIG_ASSERT` が無効で消える（互換層の `__assert.h` のコメント）。`zephyr.bin` の文字列にパスが無い（**[確]** `strings`。`*** Booting Zephyr OS build v4.5.0 ***` だけ） | 我々のソースに無い。NuttX の `assert()`・`ASSERT()`・`DEBUGASSERT()` は `CONFIG_ASSERTIONS_FILENAME`・`CONFIG_DEBUG_ASSERTIONS_FILENAME` が無いとファイル名も行も 0（`nuttx/include/assert.h` 57〜76 行。`.config` に無い: **[確]**）。`CONFIG_NDEBUG=y`、`CONFIG_DEBUG_ASSERTIONS` なし |
| `__func__`、`__LINE__` | 我々のソースに無い | `app_log.h` の `LOG_DBG` が `__func__` を使う（119、162 行）。関数名なので移動では変わらない |
| ログのモジュール名 | `LOG_MODULE_REGISTER(名前, ...)` の名前は文字で書いてあり、パスから作らない | `app_log.h` の `LOG_MODULE_REGISTER` も同じ |
| リンク順 | `target_sources()` の順と、差し替えたターゲットの中の順（**[確]** 順を保てば一致） | カーネル側は `CSRCS` の順（`ARCHIVE_REBUILD`）、アプリは名前の順に並べたオブジェクト（`SORTOBJS`）と、アプリの間は並列なら不定（6.2） |
| オブジェクト名・パス | `CMakeFiles/.../<絶対パス>.obj`。像には入らない（**[確]**） | 基本名（カーネル、ボード）、`<名前>.c.<cwd>.o`（アプリ）。ソースの場所では変わらない（**[確]**） |
| インクルードの解決順 | 引用符のインクルードは「インクルードしたファイルのディレクトリー」→ `-I` の順。移すと**別の同名ヘッダーが見つかる**おそれ（例: ドライバーのターゲットには Zephyr 本体の `drivers/sensor` が `-I` にある。`sensor_shell.h` は今も本体のものを使っている） | 同じ。移す途中で旧い場所と新しい場所の両方に同名のヘッダーが残ると、どちらを読むかが変わる |
| デバッグ情報 | ELF のパスは変わる。像（`.bin`）には入らない | 同じ |
| 版の文字列 | Zephyr 本体の `git describe`（アプリは関係しない） | NuttX 本体の Git のハッシュ（`CONFIG_VERSION_BUILD`）と日時（`SOURCE_DATE_EPOCH` で固定） |

### 6.4 「移動だけ」のタスクの後に比べるもの（提案）

計画の「`.text`・`.rodata` を比べ、パスの文字列と配置の違いだけなら動作は同じとみなす」（計画 85〜88 行）は、実際には
**像そのものが一致する**ことを確かめられる（1.1、2.3 で、T10＋T11 に当たる移動の後に両方とも一致した）。そこで:

1. 移動だけのタスク（T10、T11、T20 のうち移動だけの部分、T21・T23 の移動の部分）の合格の条件は「**像（`.bin`）がバイト単位で一致**」。
   - Zephyr: `west build -p always ...` の後 `cmp zephyr/build/usb-console/zephyr/zephyr.bin freertos/build/baseline/zephyr/snapshot/image.bin`
   - NuttX: `SOURCE_DATE_EPOCH=1763337600 nuttx/script/build.sh --clean -j 1` の後 `cmp nuttx/nuttx/nuttx.bin freertos/build/baseline/nuttx/snapshot/image.bin`
   - `.elf` は比べない（デバッグ情報のパスで必ず違う）。
2. 一致しないときは、スナップショットを取って比べる:
   ```
   python3 freertos/build/baseline/compare.py snapshot zephyr /tmp/.../after-zephyr
   python3 freertos/build/baseline/compare.py compare freertos/build/baseline/zephyr/snapshot /tmp/.../after-zephyr
   ```
   出力は 1. 像の一致、2. セクションの大きさ、3. シンボル（型と大きさ。アドレスは見ない）の増減、4. アドレスを除いた関数の
   コードの差（FUNC のシンボルだけ。データは数だけ）、5. オブジェクトごとの差（コードと再配置、確保されるセクションのバイト、
   定義したシンボル。ソースの基本名で対応させるので移動の前後で比べられる）と、違うオブジェクトの読める文字列の増減。
   - 許してよい差: リンク順だけの変化（3〜5 に何も出ず、2 とデータの数だけが出る。6.2 の `-j` の違いがその例）、パスの文字列
     （5 の文字列に出る。今は無い）。それ以外は原因を説明できるまで進めない（計画のとおり）。
3. ソースを変えるタスク（T12、T13）は像が一致しない。`compare.py` の 5 で、**変えたファイルのオブジェクトだけ**が違うこと、
   `PRId32` への書き換えのように意味を変えない変更では Zephyr 側のオブジェクトが変わらないことを確かめる（**[未]**: T12 で
   初めて使う）。
4. 基準はタスクの commit ごとに取り直す（「移動だけ」のタスクは前のタスクの後の像と比べる）。

`compare.py` は既定の `-j` と `-j 1` の像、同じ像どうしで試した（**[確]**、6.2 の最後の項）。移動の前後のオブジェクトの比較（5）は、
実験の写しが基準と像で一致したので出番がなく、移動の前後では使っていない（**[未]**）。

---

## 確かめられなかったこと

- 写しの上の移動（1、2）の後のホストテスト（実行したのはビルドと像の比較だけ）。
- 「前置きヘッダー」（`-include`）、互換層への `<zephyr/logging/log.h>`・`<zephyr/net/net_ip.h>`・`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` の追加、
  `PRId32` への書き換え、`lsm6dso_os` の吸収が像を変えないか（どれも提案。4.1、4.2）。
- `zephyr_compile_options(-fmacro-prefix-map=...)` を足したときの像。
- `compare.py` の 5（オブジェクトの比較）で実際に差が出る場面（移動の実験ではすべて一致し、差の表示を試せていない）。
- `zshell.c` の分け方（5.3）は読んだだけで、試していない。
- FreeRTOS 側の未確認事項（計画 T01 の後半。T01b の範囲）。

---

## 計画（`freertos/PORTING_PLAN.md`）と食い違う点・直すべき点

1. **「移動だけ」の合格の条件を「像の一致」にできる**（計画 85〜88 行は `.text`・`.rodata` の比較と、パスの文字列・配置の違いの
   許容）。像にパスの文字列は無く、T10＋T11 に当たる移動の後に両方の像が一致した（1.1、2.3）。比べ方は 6.4。
2. **NuttX の既定のビルドは再現しない**（日時の文字列と、並列ビルドでの `libapps.a` の順）。比較は
   `SOURCE_DATE_EPOCH=<固定> ./script/build.sh --clean -j 1` で行うと計画に書く（6.2）。実機に書く像も、同じソースで配置が変わりうる
   （`cpu_active` の比較の注意。計画のリスク 163〜164 行）。
3. 計画のリスク「`use_local_driver_sources()` の差し替えが効くか」（165〜166 行）は解消: 外のディレクトリーで効く（1.1）。ただし
   **Zephyr のドライバーのターゲット（`drivers__sensor__st__lsm6dso`、`drivers__i2c`、`drivers__sensor`）には我々のディレクトリーの
   `-I` が無く**、同じディレクトリーの検索に頼っている。ファイルを分けるタスクごとに `target_include_directories()` を足す（1.2）。
4. **NuttX の `VPATH`・`DEPPATH` は相対パスにする**（絶対パスでは `make depend` が止まる。2.2）。カーネル側には、移した `.c` が残った
   ヘッダーを読むための `-I $(TOPDIR)/external/lsm6dso` も要る（2.3）。
5. **`imu_cal.h` は T10 で `imu_cal_codec.c` と一緒に移す**（NuttX のアプリに `-I .` が無い。2.3）。T10 の A 層の一覧に `imu_cal.h` を、
   また同一のヘッダー `kv_store.h`・`time_sync.h` の扱い（T10 か T12）を書く。
6. **T11 で `lsm6dso_attributes.h` を移すと、両方の `sensor_shell.c` の `"../lsm6dso/lsm6dso_attributes.h"` が壊れる**（Zephyr は
   **[確]**、NuttX は `-I external/include` の `..` に頼っている）。T11 で両方のこの 1 行を直すか、`lsm6dso_attributes.h` を T12 に回す。
   同じく `imu_stream.c`・`imu_raw_packet.h` の `"../drivers/lsm6dso/..."` は `common/` の構成でも解決するが、`"lsm6dso_fifo.h"` に
   直すほうがよい（1.2）。
7. **T12 の一覧に `imu_raw_mqtt.h`（2 行）と `wifi_profile.h`（7 行）が無い**。どちらも少差分（4.1）。少差分は 11 本（計画 13 行の
   「10 本」）。
8. **T21 で `nina_dns.c` を `common/` へ移すが、P4 の Z01 は `nina_wifi.c` と `nina_socket.c` だけを載せ替える**。Zephyr 版の
   `nina_dns.c`（ソケットのオフロードと `k_heap`）が写しとして残り、完了の確認 4（計画 178 行）を満たさない。Z01 に `nina_dns.c` を
   足すか、残す理由を書く。
9. **T13 の「`lsm6dso_os.*` を互換層に吸収するか」は「吸収する」を提案**（4.3）。互換層に `k_busy_wait()`・`k_msleep()`・`k_usleep()`・
   `k_cyc_to_ns_floor64()` を足し、`tests/lsm6dso_driver.c` の偽物を書き換える。**`nina_os.*` は B 層の境界として残す**。
10. **T12 のシェルの登録**: NuttX 版の `imu_stream.c`・`time_sync.c` は `subcmd` の表に書き換えている（`sensor_shell.c` は互換層の
    `SHELL_CMD_REGISTER()` のまま）。差を無くすには NuttX でも互換層の `SHELL_*` マクロ（`zshell.c`）を使い、`main()` でない入口を
    作れるようにする（4.1）。出力が `subcmd` と同じかの確認が要る。
11. **T23 の `subcmd`・シェル API の共通化**: `subcmd.c` は stdio と `fnmatch()` だけで共通に置けるが、`zshell.c` の前半（バックエンド）は
    NuttX のタスク・デバイスファイル・`pthread`・`/dev` の走査に依存する（5.3）。「照合とヘルプ」と「バックエンド」に分ける作業を
    T23 に書く。互換層の `shell.h` の `pid_t cmd_pid` も OS の型にする。
12. T20 の線（5.1、5.2）: `kernel.h`・`spinlock.h` の OS の型は `zephyr_os.h`（OS ごと）へ。`zephyr_rtio_workq.c`・`zephyr_net_mgmt.c` は
    スレッドの作成（`kthread_create_with_stack()`）だけが NuttX 依存なので、互換層の OS 部分にスレッド作成の関数を足せば共通にできる。
    `zephyr_gpio.c`・`zephyr_hwinfo.c`・`zephyr_sensor_clock.c` はボードの境界（`nano_gpio.h`、チップ ID、`nano_mono_us()`）だけに依存する。
13. `kv_name.c`（KV の名前の規則）、`log_ring.c` は A 層だが、計画の一覧ではそれぞれ T23（`log_ring`）だけで、`kv_name.c` は
    どのタスクにも無い（F04 の前に `common/` へ移すなら書く）。
14. `i2c_dw_rp2040_commands.[ch]` は両方で同一（A 層）なので、T22 を待たず T10 で移せる（**[確]** 実験で移した）。
15. 作業の注意（計画の「進め方」に足すとよいこと）: NuttX のワークスペースを写して試すときは、`configure` が作る絶対パスの symlink を
    先に写しの中へ向け直す。そうしないと写しの `distclean` が元のワークスペースのリンクとオブジェクトを消す（2.4。今回起きて、元に戻した）。
