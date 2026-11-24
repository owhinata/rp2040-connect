# NINA-W102 のドライバー（`nina`）

[English](README.md) | 日本語

このリポジトリーのファームウェアは、同じファームウェアを Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上で
それぞれビルドしたもので、それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。この文書は、4 つの OS のファームウェアが
共有する NINA-W102（ESP32）のドライバーの説明です。

ドライバーは NINA-W102 を、SPI 経由の Wi-Fi・ソケット・名前解決として使えるようにし、PC の esptool で NINA のフラッシュを
読み書きするための中継モードも持ちます。中身は OS の定義を持たないコア（このディレクトリーの直下と `include/`・`internal/`）で、
各 OS の側は `port/` の狭い境界を実装するつなぎ（`port/<OS>/`、`port/pico_sdk/`）だけを書きます。Zephyr のソケットと
Wi-Fi 管理の API へのつなぎ（`port/zephyr/` の一部）は、FreeRTOS 版・ThreadX 版も互換層（Zephyr の API を Zephyr と同じ名前で
提供する層。[`common/compat/`](../../compat/README_ja.md)）の上でビルドして使います。

NINA 側は Arduino の NINA ファームウェア 3.0.2 にパッチ 0001〜0006 を当てたものを前提にします
（[ファームウェアの説明](../../../firmware/nina-fw/README.md)。ビルドと書き込みもそこにあります）。0003〜0006 の無いファームウェアでも、
その機能を使わずに動きます（下の「ファームウェアの追加機能」）。設計の要約は[設計書](../../../design/imu-raw-mqtt-design-ja.md)の §9.1
（「NINA のドライバー」「NINA の中継モード」「NINA のファームウェアの追加機能」）にあります。

## ディレクトリー

| 場所 | ファイル | 役割 |
|---|---|---|
| 直下 | `nina_wifi.c` | Wi-Fi の操作: 接続（接続と DHCP の結果の待ちを含む）・切断・スキャン・状態・接続の監視。これらを 1 つずつ行うスレッド `nina_wifi_tid` の本体（2 秒ごとの監視の確認は、スキャン中や接続の試み中にも行う）。起きたことはつなぎのコールバック（`struct nina_wifi_ops`）に渡す |
| 直下 | `nina_sock.c` | ソケットの表（10 個）と NINA のソケット命令: `nina_socket_create()`・`do_bind()`・`do_listen()`・`do_accept()`・`do_connect()`・`do_close()`・`do_setsockopt()` と、送受信の `udp_send()`・`do_send()`・`do_sendmsg()`・`udp_recv()`・`do_recv()`（送受信の待ちのループは要求 1 回で 1 周）。`nina_sock_poll_revents()`（ソケット 1 つ分の `do_poll()`）は Zephyr 版・FreeRTOS 版・ThreadX 版だけが使う |
| 直下 | `nina_dns.c` | 名前解決（モジュールのリゾルバー） |
| 直下 | `nina_passthrough.c` | esptool の中継: 再起動の印、DTR・RTS とモジュールの線の対応、レート、16 KiB のバッファーを通した両方向のデータ、出口、数、LED。2 つの境界 `nina_passthrough_bus.h`（モジュール側）と `nina_passthrough_os.h`（ホスト側）の上 |
| `include/` | `nina_wifi.h`、`nina_dns.h`、`nina_passthrough.h` | ドライバーを使う側のヘッダー |
| `internal/` | `nina_proto.c` | SPI のコマンド・応答のフレーミング（`nina_proto_request()`、`nina_proto_ip_config()`）、モジュールのリセット、READY の待ち |
| `internal/` | `nina_proto.h` | プロトコルのバイトの名前（フレーミング、ドライバーが送るコマンド、その応答の値）と、`nina_proto.c` の要求の関数・モジュールの機能ビットの宣言 |
| `internal/` | `nina_sock.h` | `nina_sock.c` の宣言（つなぎ向け） |
| `internal/` | `nina_wait.c`・`nina_wait.h` | ソケットの待ちのループの、パスの間の待ち（間隔、または IRQ の線のエッジ） |
| `port/` | `nina_os.h`、`nina_bus.h`、`nina_passthrough_os.h`、`nina_passthrough_bus.h` | OS とボードが実装する境界（下の「OS が実装する境界」） |
| `port/zephyr/` | `nina_os.c`、`nina_bus.c`、`nina_net.c`、`nina_socket.c`、`nina_passthrough_os.c`、`nina_passthrough_bus_rp2040.c`、`posix/`、`Kconfig`、`dts/bindings/wifi/arduino,nina-spi.yaml` | Zephyr の API へのつなぎ。`nina_net.c`・`nina_socket.c` は FreeRTOS 版・ThreadX 版も互換層の上でビルドする |
| `port/nuttx/` | `nina_os.c`、`nina_bus_rp2040.c`、`nina_usrsock.c`・`nina_usrsock.h`、`nina_poller.c`、`nina_netdev.c`、`nina_wifi_mgmt.c`・`nina_wifi_mgmt.h`、`nina_net.h`、`nina_passthrough_os.c`、`nina_passthrough_bus_rp2040.c` | NuttX の API へのつなぎ |
| `port/pico_sdk/` | `nina.cmake`、`nina_bus_rp2040.c`、`nina_bus_dma_rp2040.c`・`nina_bus_dma_rp2040.h`、`nina_bus_dma_os.h`、`nina_board_rp2040.h`、`nina_passthrough_os.c`、`nina_passthrough_bus_rp2040.c` | Pico SDK の上の OS（FreeRTOS 版・ThreadX 版）が共有するつなぎ |
| `port/freertos/` | `nina_os.c`、`nina_bus_dma_os_impl.h` | FreeRTOS の部分（ロックと待ち） |
| `port/threadx/` | `nina_os.c`、`nina_bus_dma_os_impl.h` | ThreadX の部分（ロックと待ち） |

`nina` コマンド（`nina version`・`nina passthrough`）は `common/shell/nina_shell.c` で、Zephyr 版・FreeRTOS 版・ThreadX 版が使います
（NuttX 版は同じ構文と出力の `nuttx/apps/nina`）。チップとボードに固有のものは、つなぎと同じ場所に `_rp2040` の付いた名前で
置きます（SPI と線の `nina_bus_rp2040.c`、中継のモジュール側の `nina_passthrough_bus_rp2040.c`。Zephyr 版の SPI と線は devicetree から
引くので、その `nina_bus.c` はチップに依存しません）。

## コアの動作

### 役割分担

Wi-Fi と TCP/IP は NINA のファームウェア（lwIP）が処理します。RP2040 側は SPI のコマンドで Wi-Fi の接続とソケットを操作する
だけで、パケットは扱いません。Zephyr から見ると、Wi-Fi 管理 API とソケットのオフロードです。

### SPI の転送層

- コマンドは 1 つずつ処理されます。要求は呼び出したスレッドの文脈で同期して処理し、NINA のロック（`nina_os_lock()`。
  再帰可能で優先度継承あり）で直列化します。CS を下げてから応答を読み終えるまで（READY を待つ間も）CS を保持します。
  バスは `nina_bus_select()`〜`nina_bus_deselect()` のあいだだけ取ります。
- NINA のリセットは最初の要求のときに 1 回だけ行います（10 ms、750 ms）。起動時にはつなぎが SPI1 とそのピンを設定し、
  CS を非アクティブにするだけです（`nina_bus_initialize()` はコアが最初の要求のときに呼びます）。
- SPI はモード 0・8 ビット・MSB first で、`CONFIG_NINA_SPI_FREQUENCY`（既定 8 MHz）です。PL022 は clk_peri（125 MHz）を偶数で
  割るので、実際は 7.8125 MHz です。
- 64 バイト以上の転送（128 バイトずつの長い読み出し、ソケットの書き込みのパラメーター）は DMA で行い、そのあいだ呼んだスレッドは
  眠ります。それより短い転送（コアの 1 バイトの転送など）はポーリングです。4 つの OS のファームウェアとも同じ境目です（実装は
  下の「各 OS のつなぎ」）。
- NINA の READY（GPIO10）の変化は、GPIO のエッジ割り込みで待ちます。待つ前に割り込みを有効にしてから READY を読むので、
  読んだあとの変化を取りこぼしません。転送中は割り込みを止めます。期限（コマンドの前 2 秒、応答まで 10 秒
  （`nina_proto_ip_config()` は 2 秒）、CS を有効にしたあと 20 ms）を過ぎると `-ETIMEDOUT` です。
  割り込みを使うかどうかは実行時に決まります。最初のリセットのあと `nina_os_ready_start()` が READY のコールバックを登録し、
  登録に失敗したときは 100 µs ごとに READY を読みます（期限と結果は同じ）。
- 1 回の TCP 送信は最大 3584 バイトです（`CONFIG_NINA_TCP_MAX_IO`）。NINA 側のコマンドバッファに収まる上限で、RAW バッチ 1 個
  （最大 2448 バイト）を 1 コマンドで送れます。

### ソケット

すべてのソケットを、ファームウェア 3.0.2 の BSD ソケット相当の命令（0x70〜0x7F：socket、close、bind、listen、accept、connect、
send、recv、sendto、recvfrom、ioctl、poll、getsockopt）で扱います。各命令は NINA 内の lwIP のソケット関数を 1 回呼ぶだけで、
ソケット同士は独立しています。ソケットの表と命令は `nina_sock.c` で、それを OS のソケットの呼び出しにするのはつなぎです
（Zephyr 版・FreeRTOS 版・ThreadX 版は Zephyr のソケットのオフロード `port/zephyr/nina_socket.c`、NuttX 版は usrsock）。

- **枠は 10 個**です。NINA の lwIP の上限（`CONFIG_LWIP_MAX_SOCKETS=10`）に合わせています。Telnet の待ち受け、MQTT、NTP、
  速度測定が共有します。11 個目は `EMFILE` です。
- **必ず非ブロッキング**にします。命令は NINA の SPI ハンドラーの中で実行されるので、ブロックする呼び出しは Wi-Fi の管理を
  含む全体を止めます。NINA 側のソケットは bind・connect で初めて作り、作成直後に非ブロッキングにします。接続完了・受信・
  送信可能の待ちは、poll 命令を 20 ms 間隔で繰り返します（尋ねるのは、Zephyr のオフロードでは呼んだスレッド、NuttX 版では
  スレッド `nina_event`。ファームウェアがソケットのイベントを通知する場合は、IRQ の線のエッジで起きます。下の
  「ファームウェアの追加機能」）。
- **失敗と「0 バイト」は応答で区別できません**。受信は先に poll で読めることを確かめます（読めるのに 0 バイトなら相手が閉じた）。
  TCP 送信で 0 が返ったときは、`CONFIG_NINA_SEND_RETRY_MS`（既定 1 ms）後に poll 命令で書き込み可能を確かめ、可能になってから
  データを送り直します（全量を毎回 SPI で送り直すことはしません。ファームウェアに依りません）。poll のエラーフラグで切断を
  判定します。接続の成否は SO_ERROR で確認します。
- **TCP の接続**は最長 10 秒待ちます。送信は、送信バッファが空かない状態が 10 秒続くとエラーにします。AP が消えたときの
  切断検出に約 9 秒かかるのはこのためです。
- **UDP** は 1 データグラム 512 バイトまでです。受信バッファに収まらない分は lwIP が捨てます（データグラムの境界は保たれます）。
  `connect` した UDP ソケットは、他の送信元からのデータグラムをドライバーで読み捨てます。受信は 1 回 512 バイトまでです。
- `bind` は任意アドレス（0.0.0.0）のみ、IPv4 のみです。マルチキャストには対応していません。
- ソケットオプション（`setsockopt`）は 2 つだけ受け付けます。
  - 受信タイムアウト（`SO_RCVTIMEO`）：ドライバー内の受信待ちに反映します（NuttX 版では NuttX が扱います）。NINA には渡しません。
  - `TCP_NODELAY`：受け付けますが、**効果はありません**。NINA の命令では TCP 層のオプションを設定できないため、まとめ送り
    （Nagle）は有効なままです。iperf3 のクライアントが制御用の接続にこの設定を要求し、失敗すると中止するので、受け付けだけ
    行っています。

  それ以外のオプションは `ENOPROTOOPT` を返します。
- 確認していないもの：TCP クライアントを複数同時に開く場合、10 ソケット同時、接続先が応答しないときのタイムアウトの経路。

### 名前解決

`nina_dns.c` は、名前を NINA の名前解決の命令（0x34 で名前を渡し、0x35 でアドレスを読む）で IPv4 のアドレス 1 つにします。
数値のアドレスは NINA に尋ねません。2 つの命令の間はロック（`nina_os_dns_lock()`）で守ります（NINA の名前解決の結果は 1 つしか
ないため）。これを使うのは、時刻同期の NTP サーバーを名前で指定したときだけです（既定はゲートウェイ）。Zephyr 版・FreeRTOS 版・
ThreadX 版は、ソケットのオフロードの `getaddrinfo()` として登録し（`NINA_OS_DNS_OFFLOAD`。結果は 2048 バイトの専用ヒープから取る）、
NuttX 版は専用の関数で使います（下の「NuttX」）。

### Wi-Fi

- 2.4 GHz のオープンと WPA2-PSK に対応しています。オープン接続は実機で試していません。WPA・WPA3・WEP はコアが断ります
  （`-ENOTSUP`）。
- 操作は 1 つのスレッド（`nina_wifi_tid`）で行います。監視は 2 秒ごとに、接続中なら NINA の状態（0x20）を見て（スキャン中や
  接続の試み中も、そのポーリングの sleep の間で）、2 回続けて未接続なら切断として IP の設定を消し、切断のイベントを出して
  再接続を要求します。接続は NINA に SSID（とパスフレーズ）を渡した後、500 ms ごとに最大 60 回状態を見て、接続できたら省電力を
  切り（0x17）、IP の設定（0x21）を読んでインターフェースに入れ、結果のイベント（失敗は `-ETIMEDOUT` など）を出します。要求の
  世代（`generation`／`request_generation`）が変わった古い接続は結果を出さずに終わります。パスフレーズはログに出しません。
- 接続時と再接続時に NINA の省電力を OFF にします。省電力が有効だと、送信が数百 ms 単位で止まることがあるためです。設定の成否を
  確認するには、パッチを当てたファームウェアが必要です。
- 認証情報は Zephyr の `wifi cred`（Settings/NVS の形式）に保存します。起動時と予期しない切断のあとは、保存済みの設定で接続を試み
  続けます。手動の切断（`wifi disconnect`）のあとは再接続しません: コアは接続の要求のたびに `wifi_profile_connection_allowed()`
  （`common/imu/wifi_profile.c`）を呼び、再接続のスレッドからの要求を、手動の切断の後は `-ECANCELED` で断ります。複数の設定を
  登録した場合、どれに繋がるかは保証しません。
- ファームウェア 3.0.2 はスキャン中に Wi-Fi を止めるため、接続は一時的に切れます（ドライバーは切断として扱い、スキャンのあとで
  繋ぎ直します）。接続を保つファームウェアでは切れません（下の節）。結果は最大 10 件です。
- 起動時、インターフェースの初期化で NINA から MAC アドレス（0x22）と追加機能を読みます（`nina_wifi_init()`）。これが NINA への
  最初の命令なので、ここで NINA のリセットが入ります。

### ファームウェアの追加機能（パッチ 0003〜0006）

`firmware/nina-fw/` のパッチ 0003〜0006 を当てたファームウェアは、命令 0x6A で追加機能のビットを返します（仕様は
[`firmware/nina-fw/README.md`](../../../firmware/nina-fw/README.md) の「Host interface of 0003 to 0006」）。ドライバーはインターフェースの
初期化（MAC アドレスの読み出しのあと）で 1 回だけ尋ね、ビットごとに動作を切り替えます。3.0.2 など命令を持たないファームウェアは
`0xEF 0x00 0xEE` と答えるので、3 バイトを読んだところで「機能なし」と判定し、追加機能を使わずに動きます。`nina version` の 2 行目に、
モジュールのビットと使っているビットを表示します。4 つの OS のファームウェアで動作は同じです。

| ビット | 機能 | ドライバーの動作 |
|---|---|---|
| 0 | 接続中のスキャンで接続を保つ | 接続中のスキャンを切断として扱いません（アドレスはそのまま、切断のイベントも繋ぎ直しもありません）。スキャン中に接続が切れたときは、接続監視（2 秒ごと）がほかの切断と同じく検出します |
| 1 | 非同期のスキャン | 0x36 で始め、0x6B を 100 ms ごとに尋ね、終わったら 0x27 で結果を読みます。NINA のロックは命令ごとにしか持たないので、スキャン中も送信は止まりません。10 秒たっても終わらなければ 0x27 を送らずに失敗（`-ETIMEDOUT`）にします。始められない・失敗は `-EIO`、接続や切断で止められたときは `-ECANCELED` です |
| 2 | ソケットのイベントを IRQ の線で通知 | ソケットの待ち（connect、受信、`poll`、送信の再試行、NuttX 版の `nina_event`）が、20 ms（送信は 1 ms）の間隔の代わりに、GPIO2 の立ち上がりのエッジの割り込みで起きます（最長 100 ms で聞き直す） |

ビット 2 の待ち方（`internal/nina_wait.h`。NuttX 版の `nina_event` の周は下の「NuttX」）:

- ファームウェアの規則どおり、イベントを登録しうる命令（poll、recv、accept、send、connect）の前にエッジの数を控え、「まだ」の
  答えのあとは、数が変わるか 100 ms たつまで待って尋ね直します。命令の前のエッジは 1 回余分に尋ねるだけで、取りこぼしたエッジは
  最長 100 ms の遅れになるだけです（正しさは尋ね直しが保ちます）。
- 線はすべてのソケットで共通なので、エッジのたびに待っているすべてのスレッドが起きます（つなぎがエッジを数え、待っている全
  スレッドを起こす）。起きても何も見つからないことが続いたスレッドは、次に待つ前に少し休みます（1、2、4、8、16 ms、以後 20 ms）。
- 送信は、データを送って 0 バイトだったパスのあとはビット 2 が無いときと同じく 1 ms 待ち、poll だけをしたパス（書き込み可能で
  なかった）のあとにエッジを待ちます。待ちは呼び出しの期限（受信の `SO_RCVTIMEO`、`poll` の timeout、connect と送信の 10 秒）を
  越えません。
- エッジの割り込みを使えなかったとき（`nina_os_event_start()` が IRQ の線のコールバックの登録かエッジの有効化に失敗したとき）は、
  ビット 2 があっても、ビット 2 が無いときの間隔で待ちます。使っているビットからビット 2 が外れます。

### esptool の中継（`nina passthrough confirm`）

`nina passthrough confirm` で、USB のシリアル（CDC ACM）を NINA の UART（ESP32 の UART0、ROM のブートローダーが使う）へ中継する
モードで再起動します。PC の esptool で NINA のフラッシュを読み書きするためのもので、Arduino の SerialNINAPassthrough スケッチの
代わりです（RP2040 のファームウェアは書き換えません）。中継そのもの（要求の印、線の対応、レート、バッファーを通した両方向の
データ、出口、数、LED）は `nina_passthrough.c` で、ホスト側（USB のシリアル、時刻）とモジュール側（UART1、モジュールの線、
watchdog の scratch レジスター）は各 OS のつなぎです。`CONFIG_NINA_PASSTHROUGH`（既定で有効）。

| RP2040 | 中継モード | 通常のファームウェア |
|---|---|---|
| GPIO8 | UART1 TX → モジュールの UART0 RX | SPI1 RX（MISO） |
| GPIO9 | UART1 RX ← モジュールの UART0 TX | CS |
| GPIO2 | モジュールの GPIO0（起動モード） | IRQ |
| GPIO3 | モジュールの RESETN（アクティブ low） | RESET |
| GPIO6 | LED | LED |

- 配線：UART1 は SPI の MISO・CS と同じピンなので、中継の間は NINA の SPI を使えません。どの OS のファームウェアも、中継は通常の
  起動とは別の起動で、NINA の SPI を起こしません。
- 線：RTS が有効でリセットを解除し、DTR が有効で GPIO0 を LOW にします（スケッチの mbed の分岐と同じ）。ポートを開くと ROM の
  ダウンロードモードで起動し、閉じるとリセットに保持します。DTR が無効のままリセットを解除した（NINA のファームウェアで起動した）
  ときは、750 ms 後に GPIO0 を離し、GPIO8 を駆動しません。UART1 のレートはホストの line coding に従います（8N1）。
- 入り方：コマンドが watchdog の scratch 0〜3 に印を書いて watchdog で再起動します（`common/chip/rp2040` の
  `rp2040_watchdog_reboot()`）。起動時に印を読んですぐ消すので、RESET やそのあとの再起動は通常の起動です。フラッシュ（KV）には
  何も書きません。
- 取りこぼし：モジュール → PC は、UART1 の受信割り込みが 16 KiB のバッファー（中継モードに入るときに確保）へ移します。PC が読まない
  間を 921600 bps で 178 ms 分保持し、あふれた分は数えます。PC → モジュールは、UART の FIFO に入りきらない間はホスト側の受信を止める
  （ホストが待つ）ので失われません。
- 出方：ポートを 300 bps にして閉じる（例 `stty -F /dev/ttyACM0 300`）か、RESET。NINA を自分のファームウェアで起動し直して
  （GPIO0 HIGH、RESETN を 10 ms LOW、750 ms 後に GPIO0 を離す）から、watchdog で再起動します。
- RAM のログ（出たあとの `dmesg`）に、開始・ポートを閉じるたび・終了の行が残ります（送受信のバイト数、USB が無い間に捨てた数、
  バッファーからあふれた数と最大の滞留、UART のオーバーラン・ブレーク・フレーミングエラー、レート）。
- esptool 5.3.0 は `--before no-reset` では繋がりません。`ESPTOOL_CFGFILE` の `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` と
  `--before default-reset --after no-reset` で RTS を有効にしたまま使います（手順は[ルートの README](../../../README_ja.md) の
  「NINA のファームウェアの読み書き」）。

### TLS について

TLS には対応していません。次の点が未解決です。

- **乱数源**：ビルドは Zephyr のテスト用の非暗号乱数を使っています。RP2040 には暗号用の乱数源がありません。ボード上の ATECC608A は
  設定とデータが未ロックで、Random コマンドの応答が 2 回同一だったため、この状態では使えません。ESP32 のハードウェア乱数を SPI 経由で
  取得するには、NINA のファームウェアに命令を追加する必要があります。
- **証明書の検証**：NINA 側の TLS を使う方法は、IP アドレス指定の接続ではホスト名を検証できず、証明書の日付検証も有効になって
  いません。
- **時刻**：証明書の有効期限の検証には信頼できる時刻が要りますが、NTP の応答は認証されません。

乱数源がない場合に、テスト用の乱数へ黙って切り替えるような実装にはしません。

### 無線区間の調査

送信の遅れや ACK の挙動を無線フレームの段階で調べる手順は、
[Wi-Fi モニターによる TCP ACK 調査手順](../../../design/wifi-monitor-debug-ja.md) にあります。

## OS が実装する境界

コアが OS とボードに求めるものは `port/` のヘッダーです。どれも、コアが共有になる前に使っていた Zephyr の呼び出しの意味で
書いてあります。

| 境界 | 満たすこと |
|---|---|
| `port/nina_os.h` | `nina_os_lock()`・`nina_os_unlock()`: 再帰可能で優先度継承のあるロック（Zephyr 版の `K_MUTEX_DEFINE(nina_lock)`）。`nina_os_lock()` は 0 か負の errno を返し、失敗は `nina_proto_request()`・`nina_proto_ip_config()` がそのまま返す。`nina_os_wifi_lock()`・`nina_os_wifi_unlock()`: Wi-Fi の操作のロック。`nina_os_wifi_post()`・`nina_os_wifi_wait()`: Wi-Fi のスレッドを起こす（数は最大 1。待ちは post か、tick に切り上げて 1 tick 足した時間で終わる。要求はコアが自分で持つ）。`nina_os_dns_lock()`・`nina_os_dns_unlock()`: 名前解決の 2 つの命令の間に持つ、優先度継承のあるロック。`nina_os_sleep_us()`・`nina_os_sleep_ms()`: Zephyr の `k_sleep()` と同じく tick に切り上げて 1 tick 足した時間眠る（シグナルでは終わらない）。`nina_os_uptime_ms()`: ms の時計（切り捨て。`k_uptime_get()`）。`nina_os_ready_start()`・`nina_os_ready_arm()`・`nina_os_ready_wait()`・`nina_os_ready_disarm()`: READY の変化を割り込みで待つ（`nina_os_ready_start()` がコールバックを登録し、失敗したらコアは 100 µs ごとに読む）。`nina_os_event_start()`・`nina_os_event_seq()`・`nina_os_event_wait()`: IRQ の線（GPIO2）の立ち上がりのエッジを数え、待っている全スレッドを起こす（`nina_os_event_start()` が失敗したらビット 2 が無いときの間隔で待つ） |
| `port/nina_bus.h` | `nina_bus_initialize()`: コアが最初の要求のときに呼ぶ。SPI とそのピンを設定して CS を非アクティブにする（READY・RESET・IRQ には触れない。2 回目からは何もしない）。使えないとき（中継モードのあいだ）は `-ENODEV`。`nina_bus_pin_configure()`・`nina_bus_pin_set()`・`nina_bus_pin_get()`: 論理値（1 がアクティブ）。極性とプルはボードが知る（CS GPIO9 アクティブ low、READY GPIO10 アクティブ low・プルアップ、RESET GPIO3 アクティブ low、IRQ GPIO2 アクティブ high・プルダウン）。`nina_bus_select()`・`nina_bus_deselect()`: 要求の 1 つの段階（コマンド、応答）のあいだ、OS にほかの利用者がいればバスを取り（Zephyr と NuttX のバスのロック）、モジュールのモードとレートにして CS をアクティブにする。`nina_bus_deselect()` で CS を非アクティブにしてバスを返す。そのあいだバスはモジュールのもの。`nina_bus_transceive()`: モード 0・8 ビット・MSB first の同期転送（RX が NULL なら書くだけ）、CS には触れない |
| `port/nina_passthrough_os.h` | 中継のホスト側: OS とその USB のシリアル（時計、ホストへの書き込み、ログ）。つなぎは `nina_passthrough_run()` も実装し、そのイベントでコアを呼ぶ（`nina_passthrough.h`） |
| `port/nina_passthrough_bus.h` | 中継のモジュール側: 要求の語と再起動、モジュールの RESETN と GPIO0、モジュールの UART0 への UART、LED。チップとボードのものなので、各 OS がチップごとに（RP2040 のレジスターか SDK の上で）実装する。受信側の関数は UART の割り込みからも呼ばれる |
| 構成 | `CONFIG_NINA_TCP_MAX_IO`（既定 3584）と `CONFIG_NINA_SEND_RETRY_MS`（既定 1）。`nina_sock.h` が読み、無ければ既定値を使う |
| C ライブラリ | `nina_sock.[ch]`・`nina_dns.[ch]` は POSIX の `<sys/socket.h>`・`<sys/types.h>`・`<sys/uio.h>`・`<netinet/in.h>`・`<netinet/tcp.h>`・`<arpa/inet.h>`・`<poll.h>`・`<netdb.h>` の型と定数（`struct sockaddr_in`、`struct iovec`、`struct addrinfo`、`AF_INET`、`MSG_DONTWAIT`、`POLLIN`、`EAI_*` など）と、`inet_pton()`・`htons()`・`malloc()`・`free()` を使う。`nina_sock_poll_revents()` が返す `POLL*` と `EAI_*` の値は、呼び出す側と同じヘッダーのもの |

Pico SDK の上の OS は、このほかに長い転送の DMA の待ち `port/pico_sdk/nina_bus_dma_os.h` を `port/<OS>/nina_bus_dma_os_impl.h` で
実装します（下の「Pico SDK」）。

## 各 OS のつなぎ

### Zephyr

コアの Zephyr の API へのつなぎは `port/zephyr/` です。アプリの `CMakeLists.txt` がそこからビルドし、つなぎの devicetree binding は
`DTS_ROOT` で、オプションはアプリの `Kconfig`（`rsource`）で読み込みます。

| ファイル | 役割 |
|---|---|
| `nina_os.c` | 境界 `nina_os.h` の Zephyr 版：mutex、Wi-Fi のスレッドを起こすセマフォ（時間を限った待ち）、sleep、時計、READY の割り込み待ち、IRQ の線（GPIO2）のエッジの待ち |
| `nina_bus.c` | 境界 `nina_bus.h` の Zephyr 版：devicetree の `spi1` と 4 本の線（CS、READY、RESET、IRQ）、Zephyr の GPIO・SPI の呼び出し。`CONFIG_NINA_PASSTHROUGH` のとき、`nina_passthrough_active()` が真の間（esptool の中継）はバスなし |
| `nina_net.c` | コアの上の Zephyr の Wi-Fi 管理とネットワークインターフェース：デバイス（`NET_DEVICE_DT_OFFLOAD_DEFINE()`）、Zephyr のパラメーターの確認と受け渡し、結果を Zephyr の構造体とイベントに、アドレス・ネットマスク・ゲートウェイと dormant をインターフェースに、Wi-Fi のプロファイルの再接続をコアのフック、スレッド `nina_wifi_tid`（`K_THREAD_DEFINE()`、2048 バイト、優先度 8） |
| `nina_socket.c` | ソケットのオフロード（ファイル記述子、errno、`O_NONBLOCK`、`SO_RCVTIMEO`、待ちのループと sleep、poll）と、名前解決を `getaddrinfo` のオフロードとして登録 |
| `nina_passthrough_os.c` | 中継の境界 `nina_passthrough_os.h` の Zephyr 版：CDC ACM のポートと USB スタックの通知、1 ms の tick、起動（`SYS_INIT`）と `main()` での出口 |
| `nina_passthrough_bus_rp2040.c` | 中継のモジュール側（`nina_passthrough_bus.h`）を RP2040 のレジスターで：watchdog の scratch レジスターと再起動、UART1 とその割り込み、GPIO2・GPIO3・GPIO8、LED |
| `posix/` | コアが使う POSIX のソケットの名前（`struct sockaddr_in`、`AF_INET`、`POLLIN` など）を Zephyr の名前に対応させるヘッダー（コアと `nina_socket.c` だけが読む） |
| `Kconfig`、`dts/bindings/wifi/arduino,nina-spi.yaml` | ドライバーのオプション（`CONFIG_WIFI_NINA_LOCAL`、`CONFIG_NINA_SPI_FREQUENCY`、`CONFIG_NINA_TCP_MAX_IO`、`CONFIG_NINA_SEND_RETRY_MS`）と `nina_w102` のノードの devicetree binding。`CONFIG_NINA_PASSTHROUGH` はアプリの `Kconfig` |

Zephyr の標準機能のうち、Telnet シェル、MQTT クライアント、zperf がこのソケットの上で動きます。

- **SPI**: SPI の周波数はデバイスツリーの NINA のノードの `spi-max-frequency` で、SPI のバスと CS もそのノードの親と `reg` から
  取ります。PL022 のドライバーは DMA のモードで動かします（`CONFIG_SPI_PL022_DMA`。`app.overlay` の `spi1` の `dmas` でチャネル 10 と
  11、起動時に確保。I2C0 の読み出しは 0 から空いているチャネルを取ります）。`CONFIG_SPI_PL022_DMA_MIN_LEN` バイト（64）以上の転送は
  2 つのチャネルで行い、そのあいだスレッドはその割り込みを待ちます。それより短い転送は DMA なしのときと同じくポーリングです
  （ローカルのパッチ `zephyr/0007-spi-pl022-dma-min-len.patch`）。どちらの経路でも、CS が下がっているあいだバスはモジュールのもの
  です: `nina_bus_select()` は 0 バイトの転送で `SPI_LOCK_ON` のドライバーのロックを取り、しきい値がその転送をポーリングの経路に
  回します（しきい値の無い DMA のモードではこの転送が DMA を始めてしまうので、`nina_bus.c` はその構成ではビルドできません）。DMA の
  エラー（ドライバーが `<err> spi_pl022: dma:... callback gets error` を記録）と、長さぶんの時間 + 200 ms
  （`CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE`）で終わらない転送（`Timeout waiting for transfer complete`。RX のオーバーランでバイトが
  失われたときもこれになります）は、そのエラーで要求が失敗します。パッチがそのあとチャネルを止めます。
- **READY の待ち**（`nina_os.c`）: 1 回の待ちは最長 10 ms で、そのたびに READY を読み直します。
- **ソケット**: Zephyr 側のファイル記述子の表は `CONFIG_ZVFS_OPEN_MAX=16` で確保しています。既定では `CONFIG_NET_MAX_CONTEXTS` から
  大きさが決まるので、その値だけを減らすと表が足りなくなり、Telnet の受け付けが失敗し続けます。Zephyr 自前の TCP/IP は通信を
  扱いません（`net conn` は常に空です）。そのためのパケットバッファと接続の管理表は、`prj.conf` で最小にしています。待ちは呼び出した
  スレッドで行います（connect は 20 ms ごとに 10 秒まで、poll は 20 ms ごと。MQTT の受信の確認は 100 ms ごと、時刻同期は応答を待つ間
  1 ms ごと）。`poll` は、1 つのソケットでモジュールへの問い合わせが失敗しても呼び出し全体を失敗させず、そのソケットに `POLLERR` を
  返します。待ちの途中で別のスレッドが閉じたソケットには `POLLNVAL` を返します。NINA 以外の記述子を含む `poll` は、ビット 2 を使って
  いても最長 20 ms ごとに見直します（その記述子の変化の見方はビット 2 が無いときと同じ）。
- **中継モード**: ホスト側は Zephyr の USB デバイススタックの上の `nina_passthrough_os.c`、モジュール側は RP2040 のレジスターの
  `nina_passthrough_bus_rp2040.c` です。
  - 起動時（`POST_KERNEL` の 0）に印を読んで消します。
  - 中継の起動でも Zephyr は通常どおり起動しますが、NINA のコアには SPI のバスが無いように見せます（`nina_bus_initialize()` が
    失敗する）。Wi-Fi のインターフェースの初期化、Telnet の待ち受けのソケット、`nina version` などは `-ENODEV` で失敗し、SPI1 と
    NINA の線に触りません（RAM のログに Telnet の待ち受けの `<err>` が 2 行残ります）。`main()` は設定・校正・時刻同期・Wi-Fi の
    接続を行わずに中継を動かします。CDC ACM の割り込みのコールバックを中継のものに差し替えるので（`POST_KERNEL` の 99）、シェルは
    入力を受けず、出力もポートに出ません。ログは RAM だけに出ます。
  - UART1 の受信割り込みは優先度 0、FIFO に 4 バイトで発生します。バッファーは `malloc()` で確保します。PC → モジュールは 1 パケット
    （64 バイト）ずつで、UART の FIFO に入りきらない分は送信割り込みが書き、その間は CDC ACM の受信を止めます。DTR・RTS とレートは
    USB のスタックの通知（システムのワークキュー）と 1 ms ごとの確認で反映します。データは CDC ACM のコールバック（専用のワーク
    キュー、次の項目）で動かします。
- **CDC ACM の専用のワークキュー**: [Zephyr 版のポート](../../../design/zephyr-port-ja.md#cdc-acm-の専用のワークキュー)にあります。

### NuttX

コアの NuttX の API へのつなぎは `port/nuttx/` です（RP2040 の SPI と線の `nina_bus_rp2040.c` もここ）。`nuttx/drivers/nina/` には
`Kconfig` と `Make.defs` だけが残ります。NuttX は `nuttx/drivers` を `external/` としてビルドし（`nuttx/nuttx/external` は
`script/setup.sh` が作るそこへの symlink）、その `Makefile` が `nina/Make.defs` を、`Kconfig` が `nina/Kconfig` を読むからです。
コアとつなぎの `.c` は `nina/Make.defs` が `VPATH`（`nuttx/drivers` からの相対パス）で参照し、オブジェクトは `nuttx/drivers` に
できます。どちらにも前置きヘッダーを付けません（構成の値は、NuttX のヘッダーが読む `<nuttx/config.h>` から入ります）。
コアのヘッダーは `include/`・`internal/`・`port/`、つなぎのヘッダーは `port/nuttx/` にあり、カーネル側（`nina/Make.defs`）、ボード、
`nina`・`imu`・`init` のアプリがインクルードパスに足します。

| 境界 | NuttX 版の実装 |
|---|---|
| `nina_os.h` | `port/nuttx/nina_os.c`。Wi-Fi の操作のロックは `rmutex_t`、Wi-Fi のスレッドを起こすのはセマフォ 1 つ |
| `nina_bus.h` | `port/nuttx/nina_bus_rp2040.c` |
| `nina_passthrough_os.h` | `port/nuttx/nina_passthrough_os.c`（cdcacm の上） |
| `nina_passthrough_bus.h` | `port/nuttx/nina_passthrough_bus_rp2040.c`（RP2040 のレジスター） |
| 構成 | `nuttx/drivers/nina/Kconfig` |
| C ライブラリ | NuttX の libc |

**SPI の転送層**

- SPI は NuttX の `rp2040_spi.c` を使います。転送ごとに `SPI_LOCK` を取り、最初の転送でモード 0・8 ビット・
  `CONFIG_NINA_SPI_FREQUENCY` を設定します。`SPI_SELECT` は使いません。
- `CONFIG_RP2040_SPI_DMA`（`imu` の構成で有効）のもとで、ドライバーは `CONFIG_RP2040_SPI_DMATHRESHOLD` バイト（63）を超える交換
  （つまり 64 バイトから）を NuttX の DMA のドライバーの 2 つのチャネルで行い、そのあいだスレッドはドライバーのセマフォで待ちます
  （TX と RX の 2 つの完了、DMA_IRQ_0。時間の上限なし）。それより短い交換はポーリングです。ドライバーは 2 つのチャネルを初期化の
  とき（NINA の最初の要求）に `rp2040_dmachannel_trywait()` で取ります（ローカルのパッチ
  `nuttx/0013-rp2040-spi-dma-channels-trywait`。空きが 2 つ無ければすべての交換がポーリング）。`SPI_EXCHANGE()` は結果を返さないので、
  コアは DMA のエラーを知れません。ドライバーは `spierr()`（`CONFIG_DEBUG_SPI_ERROR`。無効）でだけ通知します。
- READY の待ち: `nina_os_ready_arm()` がセマフォを 0 にしてから、望む向きのエッジ（準備できた＝立ち下がり、準備でなくなった＝
  立ち上がり）を `common/compat/port/nuttx/rp2040_gpio_os.c`（`gpio_os_irq_configure()`。止めていた間にラッチされたエッジを消して
  から有効にする）で有効にし、コアはそのあとで READY を読みます。読んだあとの変化はセマフォに残るので、待ちはすぐ返ります。
  割り込みは LSM6DSOX の INT1（GPIO24）と同じ GPIO バンク 0 の 1 本の IRQ で、互換層の `gpio0` のコールバックがセマフォを post する
  だけです（数は 1 まで）。`nina_os_ready_wait()` はタイムアウト付きの待ち（ms を tick に切り上げて 1 tick 足す）を 1 回で行います
  （10 ms に区切りません）。期限（`nina_os_uptime_ms()`）と待ちのタイムアウトはどちらも TIMER から読む tickless の tick なので、
  割り込みを止めている間（フラッシュの消去）にずれません。`nina_os_ready_disarm()` でエッジを止め、転送中の CS による READY の変化
  では割り込みを起こしません。`gpio0` と `rp2040_gpio_os.c` は `CONFIG_LSM6DSO_STREAM` でビルドされるので、`CONFIG_NINA` はそれに
  依存します。コールバックは最初のリセットのあとに `nina_os_ready_start()` が `gpio0` に登録します。

**ソケット（usrsock）**

NuttX 版はソケットを usrsock で提供します。NuttX の `CONFIG_NET_USRSOCK_CUSTOM` で、カーネル内に `usrsock_request()` を置きます。
NuttX 自身の TCP/IP スタックは使いません（`NET_TCP_NO_STACK`、`NET_UDP_NO_STACK`）。

| ファイル | 役割 |
|---|---|
| `nina_sock.c`・`internal/nina_sock.h` | ソケットの表と命令（コア） |
| `port/nuttx/nina_usrsock.c` | usrsock の要求を上の関数に振り分け、応答（ACK）を返す |
| `port/nuttx/nina_poller.c` | スレッド `nina_event`。NINA のソケットの状態を見て usrsock のイベントにする。待つ送信の再試行の時間も計る。ファームウェアがソケットのイベントを通知するときは IRQ の線のエッジで起きる |
| `port/nuttx/nina_usrsock.h` | 上の 2 つの間の宣言 |

- 要求は、呼び出したスレッドの文脈で、その優先度のまま同期して処理します（Zephyr 版のソケット呼び出しと同じ）。usrsock は要求を
  1 つずつ渡し、その間は usrsock のロックを持っています。NINA の命令は `nina_proto_request()` の中の NINA のロックで直列化します。
  ロックの順序は常に「usrsock のロック → NINA のロック」です（`nina_event` も同じ順に取ります）。
- ソケットの番号（usrsock の usockid）は表の位置（0〜9）です。usrsock の接続の枠（`CONFIG_NET_USRSOCK_PREALLOC_CONNS`）は 12 に
  してあります（10 個と、11 個目の要求・`accept()` の新しい接続・`SOCK_CTRL` の試行の分）。
- 待ちの扱い: Zephyr 版は待ちを呼び出し元で行います。NuttX の usrsock はドライバーからのイベントを待つので、`nina_event` が同じ
  問い合わせ（NINA の poll 命令）を 20 ms ごとに行い、結果をイベントにします: 読める → `RECVFROM_AVAIL`（待ち受けのソケットでは
  接続が来ている）、書ける → `SENDTO_READY`、接続済み TCP のエラーフラグか poll の失敗 → `REMOTE_CLOSED`。UDP のエラーフラグは
  伝えません（Zephyr 版でもデータグラムの呼び出しはその後も続きます）。usrsock が既に持っているイベントのソケットは問い合わせず、
  問い合わせるものが無ければスレッドは要求で起こされるまで眠ります。ファームウェアがソケットのイベントを通知する場合は、20 ms の
  代わりに IRQ の線のエッジ（最長 100 ms）で問い合わせます。
- どのソケットも上のとおりに扱います: MQTT と時刻同期のソケット（互換層の `zsock_*()` は NuttX のソケットの呼び出し）も、Telnet・
  iperf・nsh のコマンドのソケットも同じです。`nina_event` の優先度（106）は使う側のスレッド（MQTT 107、時刻同期 109）より低いので、
  その poll は使う側が眠るか待つ間に来ます。Zephyr 版は、ソケットを使うスレッドだけが NINA に尋ねます。こちらでは、伝えるものが
  あるソケットのそれぞれに、`nina_event` が 20 ms ごとか IRQ の線のエッジで尋ねます。
- connect: 接続命令の直後に 1 回状態を確かめ（Zephyr 版のループの最初の 1 回）、終わっていなければ「処理中」と応答して、
  `nina_event` が 20 ms ごとに確かめます。期限は接続命令から 10 秒（`ETIMEDOUT`）。結果は、成功なら `SENDTO_READY`、失敗なら connect の
  要求への最後の応答で返すので、`connect()` の errno は Zephyr 版と同じです（SO_ERROR の 104/113/116/118 → `ECONNRESET`/
  `ECONNABORTED`/`ETIMEDOUT`/`EHOSTUNREACH`、それ以外とエラーフラグ → `ECONNREFUSED`）。失敗した NINA 側のソケットは閉じます。
- accept: 待ち受けのソケットに `RECVFROM_AVAIL` が立ってから要求が来ます（usrsock の仕様）。NINA に接続が無ければ `EAGAIN` で、
  ブロッキングのソケットでは usrsock が次のイベントを待ちます。新しいソケットには `SENDTO_READY` を送ります。
- 対応していない操作は Zephyr 版と同じ errno です: `getsockopt`・`getsockname`・`getpeername`・`shutdown` は `EOPNOTSUPP`、
  `TCP_NODELAY` 以外の `setsockopt` は `ENOPROTOOPT`（`SO_RCVTIMEO`・`SO_SNDTIMEO` は NuttX が自分で扱います）。`ioctl` は `ENOTTY` を
  返して NuttX 自身の処理（ネットワークデバイスの `SIOC*` など）に任せます。`socket(AF_INET, SOCK_CTRL)`（`ifconfig` などが使う）は
  `ENOTSUP` で NuttX 自身に任せます。
- 送受信の値は Zephyr 版と同じです（UDP は 512 バイトを超えると `EMSGSIZE`）。`sendmsg` は複数のベクターを 3584 バイトまで 1 つに
  つなげて 1 回の命令で書きます（MQTT のヘッダーと本体を 1 回で出すため。NuttX の usrsock はベクターを分けたまま渡します）。1 つの
  ベクターの送信は `send`・`sendto`、複数なら `sendmsg` として扱います（UDP の `sendmsg` は Zephyr 版と同じく `EOPNOTSUPP`）。
- 送受信の待ち方: 要求 1 回で Zephyr 版の待ちのループを 1 周、呼び出したスレッドで行い、周と周の間は usrsock が待ちます。
  - 待つ TCP の送信で NINA が 1 バイトも受け取らなかったときは、`SENDTO_READY` を落として `EAGAIN` で応え、
    `CONFIG_NINA_SEND_RETRY_MS` の後に `nina_event` が `SENDTO_READY` を送り、usrsock が同じ要求を出し直します（次の周）。最初の周と
    以後 100 ms ごとに NINA の poll で状態を確かめ（エラーフラグなら `EPIPE`）、最初の周から 10 秒で `EAGAIN` を返します。期限と次の
    確認の時刻はソケットごとに持ち、同じ呼び出しの続きかどうかはデータのアドレスと長さで見分けます。
  - 待たない送信（`MSG_DONTWAIT`、非ブロッキングのソケット）は、Zephyr 版と同じく受け取られなければすぐ `EAGAIN` です。UDP は NINA が
    受け取らなかったら（NINA はどの失敗も 0 バイトと答えます）待つかどうかに関わらずすぐ `EAGAIN` です。この 2 つと 10 秒の期限では
    `SENDTO_READY` を保ったまま `-ETIME` で応え、パッチ 0005 がそれを `EAGAIN` にして呼び出しを終えます（元の usrsock は待つ呼び出しへの
    `EAGAIN` を出し直し続けます）。`SENDTO_READY` を落とすのは待つ送信の周の間だけです。落としたまま次のブロッキング送信が来ると、
    usrsock はドライバーに尋ねずに待ち、周も期限も働かないためです。そのため TCP のソケットは `poll()` では常に書き込み可に見えます
    （Zephyr 版の `poll()` は NINA に尋ねます）。
  - 受信は、先に NINA の poll でデータの有無を確かめ（Zephyr 版と同じ。応答では「データなし」と空のデータグラムを区別できないため）、
    無ければ `RECVFROM_AVAIL` を落として `EAGAIN` で応えます。待つ受信は usrsock が `RECVFROM_AVAIL` を待ち、それは `nina_event` が
    20 ms ごとの poll で送ります（`SO_RCVTIMEO` は NuttX が扱います）。TCP の受信が求めた長さ（512 バイトまで）をすべて受け取ったときは
    `RECVFROM_AVAIL` を保ちます: まだ残っているかもしれず、次の受信が NINA に尋ねます（Zephyr 版の受信は毎回尋ねます）。それ以外の
    データを受け取った後は `RECVFROM_AVAIL` を落とし、`nina_event` がすぐに poll し直します。usrsock は `RECVFROM_AVAIL` が無いと受信を
    ドライバーに渡さない（`MSG_DONTWAIT` の受信は usrsock 自身が `EAGAIN` を返す）ので、受信や `poll()` が新しいデータを知るのは
    `nina_event` の poll（最大 20 ms 後、または IRQ の線のエッジ）からです（Zephyr 版はその場で尋ねます）。`RECVFROM_AVAIL` を保たないと、
    分けて読むパケット（MQTT クライアントの CONNACK: 2 バイト、続いて 2 バイト）が最初の部分で止まります。`nina_event` が次の受信の
    後になることがあるためです。
  - UDP のソケットの最初の送信（NINA 側のソケットを作る）のあとは、`nina_event` がすぐにそのソケットを poll します（ソケットのイベントが
    あるときは、この poll のあとで、時刻同期の応答などが届くと IRQ の線にエッジが出ます）。
  - データを見つけた poll（`nina_event`）は、次の受信の最初の poll の代わりに使います（その間にデータを取る者はいません）。
- TCP の受信の 0 は、相手が閉じたか接続が失敗したことを示します（NINA はどちらも 0 バイトと答えます。Zephyr 版と同じ）。

**ビット 2 の待ち**

- `port/nuttx/nina_os.c` の GPIO2 の割り込み（`common/compat/port/nuttx/rp2040_gpio_os.c`、互換層の `gpio0` のコールバック）がエッジを
  数え、待っている全スレッドの `sem_t` を post し（ブロードキャスト）、`nina_event` も起こします（`nina_poller_edge()`）。
- `nina_event` は、前の周の前に控えたエッジの数（または受信の要求が命令の前に控えた数）から数が変わったら、20 ms を待たずに次の周を
  始めます。変わらなければ 100 ms（接続中のソケットがあればその期限まで）で周を始めます。エッジで始めた周が何も伝えないことが続くと、
  次のエッジまでに少し間を置きます（1、2、4、8、16 ms、以後 20 ms）。
- 待つ TCP 送信の次の周: poll だけをした周（書き込み可能でなかった）のあとは、その要求が命令の前に控えた数から変わったとき（最長
  100 ms、送信の期限まで）、データを送って 0 バイトだった周のあとはビット 2 が無いときと同じく 1 ms 後です。

**名前解決**

`nina_dns.c` を専用の関数 `nina_dns_getaddrinfo()`・`nina_dns_freeaddrinfo()`（`include/nina_dns.h`）で使い、互換層の
`<zephyr/net/socket.h>` が `zsock_getaddrinfo()`・`zsock_freeaddrinfo()`・`struct zsock_addrinfo` をこれにつなぎます。NuttX の
`getaddrinfo()`（`CONFIG_LIBC_NETDB`。DNS サーバーへ自分で UDP で問い合わせる）は使わず、`CONFIG_NINA_DNS` は `CONFIG_LIBC_NETDB` と
同時には有効にできません。エラーは NuttX の `EAI_*` の値で、名前は Zephyr 版の `DNS_EAI_*` と同じです（名前が引けない `EAI_NONAME`、
SPI の失敗 `EAI_AGAIN`、アドレスの長さが違う `EAI_FAIL`、ヒントの誤り `EAI_BADFLAGS`・`EAI_FAMILY`・`EAI_SOCKTYPE`、サービスの誤り
`EAI_SERVICE`、メモリー不足 `EAI_MEMORY`）。結果は Zephyr 版の 2 KB の専用ヒープではなく `malloc()` で取ります。

**Wi-Fi（ネットワークデバイス `wlan0`）**

| ファイル | 役割 |
|---|---|
| `nina_wifi.c`・`include/nina_wifi.h` | コアの Wi-Fi の操作 |
| `port/nuttx/nina_netdev.c` | NINA を NuttX のネットワークデバイス `wlan0` として登録し、wapi の無線の ioctl をコアの呼び出しにし、コアの結果を `wlan0`（アドレス、キャリア）に反映し、スレッドを始める |
| `port/nuttx/nina_wifi_mgmt.[ch]` | アプリが使う Zephyr の Wi-Fi 管理（互換層の `net_mgmt()`）をコアの上で：`net_if_get_default()` のインターフェース、`NET_REQUEST_WIFI_CONNECT`（`NET_REQUEST_WIFI_CONNECT_STORED` が保存したネットワークごとに出す）、接続のイベント、Wi-Fi のプロファイルの再接続をコアのフックに（`CONFIG_WIFI_PROFILE` のとき `common/imu/wifi_profile.c`。無ければ接続はすべて許し、再接続しない） |
| `port/nuttx/nina_net.h` | `nina_net_initialize()`（ボードの起動処理）と `nina_net_start()`（init が実行するアプリのスレッドの表、`common/imu/app_init.c`） |

- 起動: ボードの起動処理が `nina_net_initialize()` を LSM6DSOX の後、KV の前に呼びます（Zephyr 版がインターフェースを `POST_KERNEL` で
  センサーの後に初期化し、NVS を `main()` で用意するのと同じ順）。`net_mgmt` のスレッドを始めて互換層のインターフェースを登録し、
  `wlan0` を `NET_LL_IEEE80211` で登録して `IFF_UP`・キャリアなしにし、NINA から MAC アドレスと追加機能を読みます（Zephyr と同じ順）。
  スレッド `nina_wifi_tid` は、Zephyr が静的なスレッドを始めるところで init が `nina_net_start()` で始めます（アプリのスレッドの表、
  `common/imu/app_init.c` の `app_threads_start()` の最初の項目）。静的なスタックのカーネルスレッド（`kthread_create_with_stack()`）で、
  優先度とスタックは `priorities.h` の値（110。2048 バイト）、動き出してから `nina_net_start()` が戻ります。
- インターフェースの値: IPv4 のアドレス・ネットマスク・ゲートウェイは `wlan0` の `d_ipaddr`・`d_netmask`・`d_draddr`、MAC アドレスは
  `d_mac` に書きます。NuttX の `ifconfig`（`/proc/net/wlan0`）と `SIOCGIFADDR` などはそこを読みます。Zephyr の「休止（dormant）」は
  `IFF_RUNNING` が無い状態です（接続すると `RUNNING` が付く）。`wlan0` には `d_ifup`・`d_ifdown` が無く、`ifconfig wlan0 down`・`up` は
  何もしません（Zephyr 版のインターフェースも常に有効）。NuttX 自身の TCP/IP は使わないので、ARP と ICMP は切ってあります
  （`CONFIG_NET_ARP`・`CONFIG_NET_ICMP`）。`CONFIG_NET_ETHERNET` は、`NET_LL_IEEE80211` の登録（`netdev_register()` が
  `CONFIG_NET_ETH_PKTSIZE` を使う）と MAC アドレスの欄（`d_mac.ether`）に要るので残します。
- 資格情報の保存と自動再接続（`nuttx/apps/wifi/`。Zephyr 版と同じファイル `common/imu/wifi_profile.c` と、Zephyr の
  `subsys/net/lib/wifi_credentials/` の写し、`wifi cred`。[`design/nuttx-porting-ja.md`](../../../design/nuttx-porting-ja.md) の
  「Wi-Fi の資格情報と自動再接続（`wifi cred`）」）は、互換層の `net_mgmt()`（`NET_REQUEST_WIFI_CONNECT_STORED`）、
  `net_mgmt_add_event_callback()`（`NET_EVENT_WIFI_CONNECT_RESULT`）、`net_if_get_wifi_sta()`、`net_if_ipv4_get_global_addr()`、
  `k_current_get()` を Zephyr 版と同じ形で使い、`nina_wifi_mgmt.c` がそれをコアの上で受けます。

wapi の ioctl（`d_ioctl`。呼び出したスレッドで処理）は次のとおりです。

| ioctl | 処理 |
|---|---|
| `SIOCSIWSCAN` | `nina_wifi_scan()`。ESSID の指定、パッシブ、チャネルの指定は、Zephyr 版のドライバーと同じく `-ENOTSUP` で断る |
| `SIOCGIWSCAN` | スキャン中は `-EAGAIN`、失敗ならそのエラー、終わっていれば結果（コアのスキャンのコールバックで集めた最大 10 件を、NINA の順に、BSSID・チャネル・RSSI（dBm）・SSID・暗号化の有無の無線イベントにする。暗号化の種類が不明なら有無を付けない）。バッファーが足りなければ `-E2BIG` |
| `SIOCSIWAUTH` | WPA のバージョンと pairwise の暗号を覚える |
| `SIOCSIWENCODEEXT` | パスフレーズを覚える。この 2 つで覚えた設定は次の `SIOCSIWESSID` で使って忘れる（パスフレーズは消す）ので、`wapi psk` を付けない `wapi essid` は暗号化なしの接続になる |
| `SIOCSIWESSID` | 覚えた設定で `nina_wifi_connect()`。WPA2 → PSK、無し → 暗号化なし、WPA・WPA3・WEP はコアが断る（`-ENOTSUP`）。その前に Zephyr の Wi-Fi 管理と同じ鍵の検査をする（`-EINVAL`: WEP の鍵が 5・10・13・26 バイト以外、ほかの鍵が 8〜64 バイト以外、鍵の要るセキュリティで鍵が無い）。WPA のバージョンを設定せずにパスフレーズだけがあれば PSK（Zephyr の `wifi connect` で `-k` を省いたときと同じ）。フラグ 0（wapi の `disconnect` が送る乱数の SSID）は何もしない |
| `SIOCSIWAP` | 00:00:00:00:00:00 か ff:ff:ff:ff:ff:ff なら `nina_wifi_disconnect()`。ほかは `-ENOTSUP` |
| `SIOCGIWESSID`、`SIOCGIWAP`、`SIOCGIWSENS` | `nina_wifi_status()`（NINA に尋ねる）の SSID、BSSID、RSSI。未接続なら SSID は空、BSSID は 0、RSSI は `-ENOTCONN` |
| `SIOCGIWMODE`、`SIOCSIWMODE` | managed（`IW_MODE_INFRA`）だけ |

ほかの無線の ioctl は `-ENOTTY` です。接続の結果は `wapi essid` の戻りでは分からず（Zephyr 版の `wifi connect` も要求を出して戻る）、
`ifconfig` の `RUNNING` と IP アドレス、`wapi show` の ESSID で確かめます。

**中継モード**

ホスト側は cdcacm の上の `port/nuttx/nina_passthrough_os.c`（NINA のドライバーと一緒にビルド。`nuttx/drivers/nina/Make.defs`）、
モジュール側は RP2040 のレジスターの `port/nuttx/nina_passthrough_bus_rp2040.c`、`CONFIG_NINA_PASSTHROUGH`（既定 y）です。

- 入り方: `nina passthrough confirm`（`nuttx/apps/nina`）が印を書いて再起動します（Pico SDK の `watchdog_reboot(0, 0, 0)` と同じ）。
  `rp2040_boardinitialize()` が最初に印を読んで消すので、RESET・以後の再起動・異常停止は通常の起動になります。
- 中継モードの起動: `rp2040_bringup()` は USB（CDC ACM、同じ記述子とシリアル番号）だけを用意し、SPI1・NINA・I2C0・センサー・`wlan0`・
  KV のフラッシュを起こしません。init タスクは nsh・Telnet・Wi-Fi・スレッドの代わりに `nina_passthrough_run()` を実行します（優先度 120）。
- 線とレート: cdcacm のコールバック（`CAIOC_REGISTERCB`。USB の割り込みから呼ばれる）が、`CAIOC_GETCTRLLINE`・`CAIOC_GETLINECODING` で
  新しい状態を読み、その場でピンに反映します。ホストの変化はどれほど短くても順に反映されます。GPIO0 を 750 ms 後に離すこと、閉じた
  ときのログ、LED（GPIO6）の 5 Hz の点滅、出口は init タスクがセマフォで待って行います。
- モジュール → PC: UART1 の受信割り込み（FIFO に 4 バイト、または 32 ビット時間の無受信で発生）がリング（中継モードのときだけヒープから
  取る）へ移し、カーネルスレッド `nina_pt_rx` が `/dev/ttyACM0` に書きます。921600 bps で FIFO の残り 28 バイト（300 µs）が割り込みの
  遅れの許容です。
- PC → モジュール: `nina_pt_tx` が `/dev/ttyACM0` を読んで UART1 の送信 FIFO に入れ、満杯の間は送信割り込みを待ちます。入れ終わるまで
  次を読まないので、シリアルドライバーの受信バッファーが埋まると cdcacm がパケットを受け取らず（ホストが待つ）、取りこぼしません。
- `/dev/ttyACM0` はコンソールではないので変換もエコーもありません（termios のフラグも 0 にします）。
- 出口では、モジュールを `init_nina()` と同じ手順（GPIO0 を HIGH、RESETN を 10 ms LOW、750 ms、GPIO0 を入力）で通常の起動に戻します。
- シグナルは使いません（待ちはセマフォ、sleep、ブロッキングの read・write）。NuttX の UART1 ドライバー（`CONFIG_RP2040_UART1`）は
  起動時にピンを取るので使いません（UART1 のレジスターを直接扱う）。

### Pico SDK（FreeRTOS・ThreadX で共有）

FreeRTOS 版と ThreadX 版は、Pico SDK の上の部分 `port/pico_sdk/` を共有し、OS の部分（`port/freertos/`・`port/threadx/` の
`nina_os.c` と `nina_bus_dma_os_impl.h`。互換層を使わずその OS と Pico SDK で直接書いたもの）だけが違います。ドライバーは
`port/pico_sdk/nina.cmake`（`pico_sdk_nina()`）がビルドに入れます。アプリの起動のコードはドライバーの何も呼びません（コアが最初の
要求で SPI1 を設定し、モジュールをリセットします）。

| ファイル | 役割 |
|---|---|
| `nina_bus_rp2040.c` | 境界 `nina_bus.h`: SPI1 を 7.8125 MHz で、CS・READY・RESET・IRQ の 4 本（Pico SDK の `hardware_spi`・`hardware_gpio`。Zephyr のドライバーと同じ順と値で設定）。64 バイト未満の転送はポーリング、それ以上は `nina_bus_dma_rp2040.c` |
| `nina_bus_dma_rp2040.c`・`.h` | 長い転送の DMA（`hardware_dma`・`hardware_irq`）。待ちは `nina_bus_dma_os.h` の上 |
| `nina_bus_dma_os.h` | DMA の待ちの OS の部分の境界: 呼んだスレッドを DMA の割り込みが起こすか時間切れまで眠らせる。OS の部分 `port/<OS>/nina_bus_dma_os_impl.h` が型・マクロと inline の関数を定義する |
| `nina_board_rp2040.h` | ボードの線（Zephyr の `arduino_nano_connect.dts`・`-pinctrl.dtsi` と NuttX のボードの `board.h` の `BOARD_NINA_*` と同じ） |
| `nina_passthrough_os.c` | 中継のホスト側（TinyUSB と Pico SDK、スケジューラーなし） |
| `nina_passthrough_bus_rp2040.c` | 中継のモジュール側: UART1、モジュールの線、watchdog の scratch レジスター、LED（`hardware_uart`・`hardware_irq`・`hardware_watchdog`） |
| `nina.cmake` | ビルドの定義 |

- **DMA**（`nina_bus_dma_rp2040.c`）: 64 バイト以上の転送（SOCKET_SEND の本体、長い読み出し）は、バスを最初に使うとき（通常のモード
  だけ。空きが 2 つ無ければすべてポーリングのまま）に `dma_claim_unused_channel(false)` で取って持ち続ける 2 つのチャネルで行います。
  RX を先に（SSPDR からバッファーへ、またはバイト 1 つの捨て先へ。DREQ 19）、次に SSPDMACR の TXDMAE・RXDMAE、次に TX（バッファーから
  SSPDR へ。DREQ 18）、どちらも 8 ビット。呼んだスレッドは、RX のチャネルの DMA_IRQ_1 の割り込み（`irq_add_shared_handler()` で加えた
  ハンドラーで、自分のチャネルだけを見る）まで眠ります。そのあと SSPDMACR を戻し、SPI1 をポーリングの転送のあとと同じ状態（RX の
  FIFO が空、オーバーランの印なし）にします。2 ms + 長さ / 256 ms で終わらなければ両方のチャネルを中断して `-EIO`、バイトが失われた
  とき（オーバーランの印）とチャネルのバスのエラーも `-EIO`。コアはこれを要求のエラーとして返します（ソケットの呼び出しが `EIO` で
  失敗する）。3 つとも `dmesg` に記録します（`<err> nina_bus: SPI1 DMA did not end in ...`・`SPI1 RX overrun by DMA ...`・
  `SPI1 DMA bus error ...`。それぞれ 5 秒に 1 行まで、抑えた数を添える）。中断しても止まらないチャネルと、起動時に空きが 2 つ無い
  ときは 1 回だけ記録し、以後の転送はすべてポーリングです。短い転送とスケジューラーが動く前の転送はポーリングです。
- **READY と IRQ の線の割り込み**: コアの READY の待ちは、READY のエッジ割り込み（GPIO10）で眠り、最長 10 ms ごとに READY を読み
  直します（割り込みを止めている間は tick が止まり、あとで TIMER に追いつくので、10 ms ごとに区切ります）。つなぎは GPIO10 と GPIO2 の
  ための `IO_IRQ_BANK0` の raw ハンドラーを自分で持ちます（`gpio_add_raw_irq_handler_masked()`。GPIO24 のための GPIO バンク 0
  （`common/compat/port/pico_sdk/gpio_os_rp2040.c`）のハンドラーと並び、それぞれ自分のピンだけを確認・クリアする）。この 2 つと DMA の
  ハンドラーは Pico SDK の共有の割り込みハンドラーの枠を使います（枠の数と割り込みの優先度は
  [FreeRTOS 版のポートの「割り込み」](../../../design/freertos-port-ja.md#割り込み)）。FreeRTOS と ThreadX のつなぎの `nina_os_ready_start()` は失敗しません。
- **ソケットと Wi-Fi**: Zephyr へのつなぎの `port/zephyr/nina_socket.c`（ソケットのオフロード、優先度 40）と `port/zephyr/nina_net.c`
  （Zephyr の Wi-Fi 管理とインターフェース、スレッド `nina_wifi_tid`）を変えずに、互換層の上でビルドします。Zephyr 版と同じく、呼んだスレッドで NINA に問い合わせて待ちます（NuttX 版の usrsock と `nina_event` のような、
  イベントのスレッドはありません）。互換層のソケット層・fd の表・`net_if`・`net_mgmt`・`wifi` コマンドは
  [互換層の説明](../../compat/README_ja.md)にあります。コアの POSIX の名前は互換層の `common/compat/libc_net/include/` で、つなぎの
  `net_*` の名前は互換層の `<zephyr/net/net_ip.h>` です（Zephyr 版の `port/zephyr/posix/` は逆向きの対応）。`CONFIG_NINA_TCP_MAX_IO`
  （3584）・`CONFIG_NINA_SEND_RETRY_MS`（1）・`CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY`（40）は Zephyr 版の値で、共有ソースの前置き
  （`common_prelude.h`）にあります。名前解決の結果は 2048 バイトのヒープから取ります（解放されていない結果が約 36 個になると
  `EAI_MEMORY`）。
- **中継モード**: `main()` は何より先に印を見て、タスク（スレッド）もスケジューラーも SPI1・NINA・I2C・KV も起こさずに、1 つのループで
  TinyUSB（`tud_task()`）と UART1 を中継します。モジュール → PC の UART1 の受信割り込みとコアのその部分は RAM で動きます
  （`NINA_PASSTHROUGH_ISR_SECTION`）。LED は 5 Hz で点滅します。
- NINA のコアのつなぎ（`nina_os.c`）は RAM に置くコードの一覧に入っています（[FreeRTOS 版のポートの説明](../../../design/freertos-port-ja.md)）。

### FreeRTOS

`port/freertos/nina_os.c`（境界 `nina_os.h`）と `port/freertos/nina_bus_dma_os_impl.h` です。

- NINA のロック・名前解決のロック・Wi-Fi の操作のロックは、優先度継承のある FreeRTOS の再帰ミューテックスです。
- Wi-Fi のスレッドを起こすのはそのタスク通知 2 です（要求はコアが自分で持つ）。
- READY の待ちは、READY のエッジ割り込みが与える 2 値セマフォで眠ります。
- ソケットのイベント: IRQ の線（GPIO2）の立ち上がりのエッジの割り込みで、つなぎが数を数え、待っている全タスクを起こします
  （待つタスクを並べた表から、それぞれのタスク通知 2 で）。
- SPI の DMA の待ちは、呼んだタスクの NINA のタスク通知（番号 2）です。
- sleep は「切り上げ + 1 tick」、時計は TIMER の ms です。
- つなぎのオブジェクトは静的で、どれかを最初に使うときに作ります。

### ThreadX

`port/threadx/nina_os.c`（境界 `nina_os.h`）と `port/threadx/nina_bus_dma_os_impl.h` です。互換層を通さず ThreadX と Pico SDK で直接
書いてあり、待ちのオブジェクトはドライバーが持つ `TX_SEMAPHORE` です。

- `TX_SEMAPHORE` を使い、スレッド自身の起こしや停止と再開を使わない理由は [ThreadX 版のポート](../../../design/threadx-port-ja.md#互換層のカーネルの部分とドライバーの待ち)にあります。
- 3 つのロックは優先度継承のある `TX_MUTEX`、READY の待ちと Wi-Fi のスレッドの起こしは `TX_SEMAPHORE` です。最初の待ちの前の Wi-Fi の
  スレッドへの post は残ります（Zephyr 版と同じ）。
- ソケットのイベントの待ちは、待つたびに自分のスタックにセマフォを作り、終わりで消します（イベントの割り込みが待ち手の一覧をたどって
  全員を起こす）。
- SPI の DMA の待ち（`nina_bus_dma_os_impl.h`）は転送のためのセマフォ 1 つです（転送は `nina_os_lock()` の中で 1 つずつ）。
- オブジェクトは最初の使用で作り、消しません（作成済みの一覧に残る）。
- 時間つきの待ちとスリープは、時間を tick に切り上げて 1 tick 足します（Zephyr の規則。ThreadX の n の待ちは今の tick + n で終わる）。
  時計は tick ではなく TIMER の ms です。

## テスト

コアのテストは `common/tests/run.sh` が実行し、各 OS の `host_tests.sh` がそれに続けて自分のつなぎのテストを実行します。どれもボードは
要りません。

| テスト | 場所 | 対象 |
|---|---|---|
| `nina_core_independent` | `common/tests/run.sh` | コアの各ファイル（直下と `include/`・`internal/`、`port/` の境界のヘッダー）が C ライブラリー・POSIX のソケットの型・自分のヘッダーだけでコンパイルできること |
| `nina_public_headers` | `common/tests/run.sh` | `include/` のヘッダーがそのディレクトリーだけでコンパイルできること（`internal/`・`port/` を読まない） |
| `nina_proto` | `common/tests/nina_proto.c` | SPI のフレーミング、応答の解析（ファームウェアのエラーの応答 0xEF を含む）、READY の期限、リセットの手順、CS の順、追加機能の問い合わせ（0x6A）（偽の境界と時計）。どれも READY の割り込み待ちを始められたときと始められなかったとき（100 µs ごとに読む）の両方で |
| `nina_wait` | `common/tests/nina_wait.c` | ソケットの待ち（`nina_wait.c`）：間隔（イベントを始める前と、始められなかったあと）、IRQ の線のエッジ、100 ms の時間切れ、期限、休み（偽の時計と線） |
| `nina_sock` | `common/tests/nina_sock.c` | ソケットの表と命令（偽の `nina_proto_request()` と時計） |
| `nina_wifi` | `common/tests/nina_wifi.c` | Wi-Fi の操作（`nina_wifi.c`）を偽の境界とつなぎで：接続とその確認、そのスレッド（500 ms ごとの確認、省電力の設定、アドレス、60 回、試行中の切断）、切断、監視（2 回で切れたとみなす）、3.0.2 とパッチを当てたファームウェアのスキャンとその終わり方、状態。1 つのスレッドの状態の組み合わせ（スキャン中と接続の試み中の、期限どおりの監視の確認、スキャン中に見つかる切断、結果の取得中に断る要求、期限ちょうどの要求、確認を挟んだ 10 秒と 30 秒の上限） |
| `nina_dns`、`nina_dns_offload` | `common/tests/nina_dns.c` | 名前解決（`nina_dns_offload` は Zephyr 版・FreeRTOS 版・ThreadX 版と同じ `NINA_OS_DNS_OFFLOAD` の形） |
| `nina_passthrough` | `common/tests/nina_passthrough.c` | 中継のコア（`nina_passthrough.c`）を偽の境界で：再起動の印と異常停止の記録の区別、DTR・RTS とモジュールの線の対応、レートと出口、バッファー（2 つのスレッドでも）、起動、両方向のデータとその数（ブレーク、オーバーラン、バッファーの満杯、ホストが無いとき）、tick（GPIO0 を離す時刻、LED）、出口の手順 |
| `nina_boundary` | `zephyr/script/host_tests.sh` | 共有の `nina_proto.c` を `port/zephyr/` の `nina_bus.c`・`nina_os.c`（Wi-Fi の操作のロックと、Wi-Fi のスレッドを起こす待ちも）に載せたもの：フレーミング、線と設定の対応、リセットの手順、READY の割り込み待ち（コールバックの登録、セマフォのリセット、エッジの向き、転送前の無効化、10 ms の上限）と、コールバックを登録できなかったときのポーリング。IRQ の線のエッジの待ち（始められないとき、GPIO2 のコールバックと立ち上がりのエッジ、エッジの数、エッジで終わる待ち、時間切れ、待つ直前のエッジ） |
| `nina_boundary_passthrough` | `zephyr/script/host_tests.sh` | 上と同じものを `CONFIG_NINA_PASSTHROUGH` で。中継の間はバスが無く、線にも SPI にも触れずに `-ENODEV`、中継でなくなれば通常どおり |
| `nina_boundary_spi_dma`・`nina_bus_spi_dma_threshold` | `zephyr/script/host_tests.sh` | 上と同じものを `prj.conf` のバスの構成（PL022 の DMA のモードと `CONFIG_SPI_PL022_DMA_MIN_LEN` 64）で。しきい値の無い DMA のモードでは `nina_bus.c` がビルドできないこと |
| `passthrough` | `zephyr/script/host_tests.sh` | 中継の Zephyr 側 `port/zephyr/nina_passthrough_os.c` と `nina_shell.c` の `nina passthrough` を本物のコアと、偽の Zephyr と偽のモジュール側で：起動時の印の扱い、USB の通知と tick での線とレート、両方向のデータと割り込みの受け渡し、USB が無い間の破棄、出口の手順、コマンド |
| `nina_socket` | `zephyr/script/host_tests.sh` | ソケットのオフロード `port/zephyr/nina_socket.c` を共有のコアに載せたもの：ファイル記述子、待ちのループと sleep（イベントを始められなかったあと）、オプション、accept、sendmsg、データグラム、poll、リゾルバーの登録。イベントを始めたあとの IRQ の線のエッジの待ち（パスの間のエッジ、時間切れ、期限、送信の待ち方、NINA 以外の記述子を含む `poll`） |
| `nina_net` | `zephyr/script/host_tests.sh` | Zephyr へのつなぎ `port/zephyr/nina_net.c` を、偽の Zephyr の宣言と偽のコアで：デバイス、インターフェースの初期化、スキャンのパラメーターと結果、接続・切断・状態、コアのコールバックのインターフェースと Zephyr のイベントへの反映、プロファイルのフック、スレッドのスタックと優先度（1 つ） |
| `nina_os` | `nuttx/tests/nina_os.c` | `port/nuttx/nina_os.c` の READY の割り込み待ち（コアの `wait_ready()` と）、GPIO2 の割り込みと待ち、Wi-Fi の操作のロックと Wi-Fi のスレッドを起こす待ち（偽の GPIO バンクとカーネル）。`nina_event` のエッジでの周は確かめていない |
| `nina_netdev` | `nuttx/tests/nina_netdev.c` | `wlan0` と wapi の ioctl をコアの上で |
| `nina_wifi_mgmt` | `nuttx/tests/nina_wifi_mgmt.c` | アプリの Wi-Fi 管理 |
| `nina_passthrough` | `nuttx/tests/nina_passthrough.c` | `nina passthrough` の確認の語と引数とメッセージ。NuttX 側（割り込み・スレッド・cdcacm との組み合わせ）は確かめていない |
| `nina` | `freertos/tests/nina/` | NINA の SPI と線の境界（偽の DMA の上で長い転送の DMA も: しきい値、SPI1 の終わりの状態、共有の DMA_IRQ_1、時間切れと中断、失われたバイト、バスのエラー、それぞれのあとの転送、止まらないチャネル、空きの無いチャネル、記録するメッセージ）と、`nina_os.h` の FreeRTOS へのつなぎ（互換層なし。tick 1 ms と 4 ms で 2 回ビルド）: READY の割り込み待ち、IRQ の線のエッジの待ち（2 つのタスクを 1 つのエッジで起こす、時間切れ）、共有のバンクの割り込み、ロック、Wi-Fi のスレッドを起こす待ち、sleep の tick 数、tick の一周をまたぐ時計 |
| `socket` | `freertos/tests/socket/` | 互換層の上の NINA のソケット: fd、待ちのループ、オプション、poll、fd のロック、名前解決 |
| `wifi` | `freertos/tests/wifi/` | インターフェース、イベント、資格情報、再接続、`wifi`・`ifconfig`、ファームウェアのビットによるスキャン（接続を保つ、非同期、失敗、終わらない、スキャン中の切断、3.0.2） |
| `nina` | `threadx/tests/nina/` | FreeRTOS 版のテストの本体（`freertos/tests/nina/main.c`）を `threadx/tests/nina/nina_test_os.c` と一緒に（ThreadX の Linux ポートの上）。SPI と線の境界と DMA、READY とイベントの割り込み、ロック、スリープ、時計。加えて、カーネルの前の呼び出し、最初の待ちの前の Wi-Fi の post、2 つのミューテックスにまたがる優先度継承、待つ直前の窓 |
