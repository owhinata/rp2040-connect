# ドライバー（NuttX 版）

[English](README.md) | 日本語

`nuttx/drivers/` は NuttX のカーネル側でビルドするこの移植のコードです（`nuttx/nuttx/external` からの
symlink で、`libexternal.a` になります）。ボード側の部分は `nuttx/boards/arduino-nano-rp2040-connect/src/`
にあります。Zephyr 版のドライバー説明は `zephyr/app/usb-console/drivers/README_ja.md` です。コマンドの
使い方と Zephyr 版との違いは `nuttx/README_ja.md`、各機能の動作の詳しい説明は `design/nuttx-porting-ja.md` に
あります。

## NINA-W102（`nina`）

NINA のドライバーは、OS に依存しないコア（リポジトリの `common/drivers/nina/`。ほかの版と共有する）と、その NuttX の
API へのつなぎ（`common/drivers/nina/port/nuttx/`。RP2040 の SPI と線の `nina_bus_rp2040.c` もここ）に分かれています。`nuttx/drivers/nina/` には
`Kconfig` と `Make.defs` だけが残ります。NuttX は `nuttx/drivers` を `external/` としてビルドし（`nuttx/nuttx/external` は
`script/setup.sh` が作るそこへの symlink）、その `Makefile` が `nina/Make.defs` を、`Kconfig` が `nina/Kconfig` を読むからです。
コアとつなぎの `.c` は `nina/Make.defs` が `VPATH`（`nuttx/drivers` からの相対パス）で参照し、オブジェクトは `nuttx/drivers` に
できます。どちらにも前置きヘッダーを付けません（構成の値は、NuttX のヘッダーが読む `<nuttx/config.h>` から入ります）。
コアのヘッダーは `common/drivers/nina/`、つなぎのヘッダーは `common/drivers/nina/port/nuttx/` にあり、カーネル側
（`nina/Make.defs`）、ボード、`nina`・`imu`・`init` のアプリがインクルードパスに足します。

コアが OS とボードに求めるもの（境界）:

| 境界 | NuttX 版の実装 | 満たすこと |
|---|---|---|
| `common/drivers/nina/port/nina_os.h` | `common/drivers/nina/port/nuttx/nina_os.c` | `nina_os_lock()`・`nina_os_unlock()`: 再帰可能で優先度継承のあるロック（Zephyr 版の `K_MUTEX_DEFINE(nina_lock)`）。`nina_os_wifi_lock()`・`nina_os_wifi_unlock()`: Wi-Fi の操作のロック（`rmutex_t`）。`nina_os_wifi_post()`・`nina_os_wifi_wait()`: スキャンと接続のスレッドへの要求（それぞれセマフォ 1 つ、数は最大 1）。`nina_os_lock()` は 0 か負の errno を返し、失敗は `nina_proto_request()`・`nina_proto_ip_config()` がそのまま返す。`nina_os_dns_lock()`・`nina_os_dns_unlock()`: 名前解決の 2 つの命令の間に持つ、優先度継承のあるロック。`nina_os_sleep_us()`・`nina_os_sleep_ms()`: Zephyr の `k_sleep()` と同じく tick に切り上げて 1 tick 足した時間眠り、シグナルでは終わらない。`nina_os_uptime_ms()`: カーネルの時計の ms（切り捨て。`k_uptime_get()`）。`nina_os_ready_start()`・`nina_os_ready_arm()`・`nina_os_ready_wait()`・`nina_os_ready_disarm()`: READY の変化を割り込みで待つ（`nina_os_ready_start()` がコールバックを登録し、失敗したらコアは 100 µs ごとに読む。下の「READY の待ち」）。`nina_os_event_start()`・`nina_os_event_seq()`・`nina_os_event_wait()`: IRQ の線（GPIO2）の立ち上がりのエッジを数え、待っている全スレッドを起こす（`nina_os_event_start()` が失敗したらビット 2 が無いときの間隔で待つ。下の「ファームウェアの追加機能」） |
| `common/drivers/nina/port/nina_bus.h` | `port/nuttx/nina_bus_rp2040.c` | `nina_bus_initialize()`: コアが最初の要求のときに呼ぶ。SPI とそのピンを設定して CS を非アクティブにする（READY・RESET・IRQ には触れない。2 回目からは何もしない）。使えないとき（中継モードのあいだ）は `-ENODEV`。`nina_bus_pin_configure()`・`nina_bus_pin_set()`・`nina_bus_pin_get()`: 論理値（1 がアクティブ）。極性とプルはボードが知る（CS GPIO9 アクティブ low、READY GPIO10 アクティブ low・プルアップ、RESET GPIO3 アクティブ low、IRQ GPIO2 アクティブ high・プルダウン）。`nina_bus_transceive()`: モード 0・8 ビット・MSB first の同期転送（RX が NULL なら書くだけ）、CS には触れない |
| 構成 | `nina/Kconfig` | `CONFIG_NINA_TCP_MAX_IO`（既定 3584）と `CONFIG_NINA_SEND_RETRY_MS`（既定 1）。`nina_sock.h` が読み、無ければ既定値を使う |
| C ライブラリ | NuttX の libc | `nina_sock.[ch]`・`nina_dns.[ch]` は POSIX の `<sys/socket.h>`・`<sys/types.h>`・`<sys/uio.h>`・`<netinet/in.h>`・`<netinet/tcp.h>`・`<arpa/inet.h>`・`<poll.h>`・`<netdb.h>` の型と定数（`struct sockaddr_in`、`struct iovec`、`struct addrinfo`、`AF_INET`、`MSG_DONTWAIT`、`POLLIN`、`EAI_*` など）と、`inet_pton()`・`htons()`・`malloc()`・`free()` を使う。`nina_sock_poll_revents()` が返す `POLL*` と `EAI_*` の値は、呼び出す側と同じヘッダーのもの |

### SPI の転送層

| ファイル | 役割 |
|---|---|
| `common/drivers/nina/internal/nina_proto.c` | コマンドと応答のフレーミング（`nina_proto_request()`、`nina_proto_ip_config()`）。3 つの版が使う |
| `common/drivers/nina/internal/nina_proto.h` | 公開する宣言。Zephyr 版も同じファイルを使う |
| `common/drivers/nina/port/nina_os.h`、`common/drivers/nina/port/nuttx/nina_os.c` | ロック（再帰可能なミューテックス、優先度継承あり）、sleep、ms の時計、READY の割り込みでの待ち。Zephyr の `K_MUTEX`・`k_sleep()`・`k_uptime_get()` に当たる |
| `common/drivers/nina/port/nina_bus.h`、`port/nuttx/nina_bus_rp2040.c` | SPI1 と 4 本の信号線（CS、READY、RESET、IRQ）。Zephyr のデバイスツリーと SPI・GPIO ドライバーが行っていた設定に当たる |

- 要求は呼び出したスレッドの文脈で同期して処理します。ミューテックスで直列化し、CS を下げてから
  応答を読み終えるまで（READY を待つ間も）CS を保持します。
- NINA のリセットは最初の要求のときに 1 回だけ行います（10 ms、750 ms）。起動時にはボードが SPI1 と
  そのピンを設定し、CS を非アクティブにするだけです。
- SPI は NuttX の `rp2040_spi.c` をポーリングで使います（DMA なし）。転送ごとに `SPI_LOCK` を取り、
  最初の転送でモード 0・8 ビット・`CONFIG_NINA_SPI_FREQUENCY` を設定します。`SPI_SELECT` は
  使いません。
- READY の待ち: コアの `wait_ready()` は READY（GPIO10、アクティブ low）を 100 µs ごとに見る代わりに、
  その変化の割り込みで眠ります（Zephyr 版・FreeRTOS 版と同じ）。`nina_os_ready_arm()` が
  セマフォを 0 にしてから、望む向きのエッジ（準備できた＝立ち下がり、準備でなくなった＝立ち上がり）を
  ボードの `src/rp2040_gpio_os.c`（`gpio_os_irq_configure()`。止めていた間にラッチされたエッジを消してから有効にする）で
  有効にし、コアはそのあとで READY を読みます。読んだあとの変化はセマフォに残るので、待ちはすぐ返ります。
  割り込みは LSM6DSOX の INT1（GPIO24）と同じ GPIO バンク 0 の 1 本の IRQ で、互換層の `gpio0` のコールバックが
  セマフォを post するだけです（数は 1 まで）。`nina_os_ready_wait()` はタイムアウト付きの待ち（ms を tick に
  切り上げて 1 tick 足す）を 1 回で行います。期限（`nina_os_uptime_ms()`）と待ちのタイムアウトはどちらも
  TIMER から読む tickless の tick なので、割り込みを止めている間（フラッシュの消去）にずれません（FreeRTOS 版は
  その間 tick が止まり、あとで TIMER に追いつくので、10 ms ごとに区切ります）。`nina_os_ready_disarm()` でエッジを止め、転送中の CS による
  READY の変化では割り込みを起こしません。`gpio0` と `src/rp2040_gpio_os.c` は `CONFIG_LSM6DSO_STREAM` で
  ビルドされるので、`CONFIG_NINA` はそれに依存します。
  コールバックは最初のリセットのあとに `nina_os_ready_start()` が `gpio0` に登録します。登録に失敗したときは、
  コアは 100 µs ごとに READY を読みます（期限と結果は同じ）。
- 期限と配線は `design/nuttx-porting-ja.md` の「NINA-W102（`nina`）」の節にあります。
- `common/drivers/nina/internal/nina_proto.c` はホストテスト（`common/tests/nina_proto.c`。`common/tests/run.sh` が走らせる）で、偽のボードと時計を
  使って確かめます。READY の割り込みでの待ち（`common/drivers/nina/port/nuttx/nina_os.c` とコアの `wait_ready()`）は `nuttx/tests/nina_os.c`
  （`script/host_tests.sh`）が、偽の GPIO バンクとカーネルで確かめます。

### ソケット（usrsock）

Zephyr 版はソケットのオフロード（`common/drivers/nina/port/zephyr/nina_socket.c`）で、NuttX 版は usrsock です。NuttX の
`CONFIG_NET_USRSOCK_CUSTOM` で、カーネル内に `usrsock_request()` を置きます。NuttX 自身の TCP/IP スタックは
使いません（`NET_TCP_NO_STACK`、`NET_UDP_NO_STACK`）。

| ファイル | 役割 |
|---|---|
| `common/drivers/nina/nina_sock.c`・`internal/nina_sock.h` | ソケットの表（10 個）と NINA のソケット命令: `nina_socket_create()`・`do_bind()`・`do_listen()`・`do_accept()`・`do_connect()`・`do_close()`・`do_setsockopt()` と、送受信の `udp_send()`・`do_send()`・`do_sendmsg()`・`udp_recv()`・`do_recv()`（送受信の待ちのループは要求 1 回で 1 周）。3 つの版が使う。`nina_sock_poll_revents()`（ソケット 1 つ分の `do_poll()`）は Zephyr 版と FreeRTOS 版だけが使う。OS に依存しない |
| `common/drivers/nina/port/nuttx/nina_usrsock.c` | usrsock の要求を上の関数に振り分け、応答（ACK）を返す |
| `common/drivers/nina/port/nuttx/nina_poller.c` | スレッド `nina_event`。NINA のソケットの状態を見て usrsock のイベントにする。待つ送信の再試行の時間も計る。ファームウェアがソケットの事象を通知するときは IRQ の線のエッジで起きる |
| `common/drivers/nina/port/nuttx/nina_usrsock.h` | 上の 2 つの間の宣言 |

- 要求は、呼び出したスレッドの文脈で、その優先度のまま同期して処理します（Zephyr 版のソケット呼び出しと
  同じ）。usrsock は要求を 1 つずつ渡し、その間は usrsock のロックを持っています。NINA の命令は
  `nina_proto_request()` の中の NINA のロックで直列化します。ロックの順序は常に「usrsock のロック → NINA の
  ロック」です（`nina_event` も同じ順に取ります）。
- ソケットの番号（usrsock の usockid）は表の位置（0〜9）です。NINA 側のソケットは Zephyr 版と同じく、
  bind・connect で初めて作り、すぐに非ブロッキングにします。11 個目の `socket()` は `EMFILE` です。
  usrsock の接続の枠（`CONFIG_NET_USRSOCK_PREALLOC_CONNS`）は 12 にしてあります（10 個と、11 個目の
  要求・`accept()` の新しい接続・`SOCK_CTRL` の試行の分）。
- 待ちの扱い: Zephyr 版は待ちを呼び出し元で行います（connect は 20 ms ごとに 10 秒まで、poll は 20 ms ごと）。
  NuttX の usrsock はドライバーからのイベントを待つので、`nina_event` が同じ問い合わせ（NINA の poll 命令）を
  20 ms ごとに行い、結果をイベントにします: 読める → `RECVFROM_AVAIL`（待ち受けのソケットでは接続が来ている）、
  書ける → `SENDTO_READY`、接続済み TCP のエラーフラグか poll の失敗 → `REMOTE_CLOSED`。UDP のエラー
  フラグは伝えません（Zephyr 版でもデータグラムの呼び出しはその後も続きます）。usrsock が既に持っている
  イベントのソケットは問い合わせず、問い合わせるものが無ければスレッドは要求で起こされるまで眠ります。
  ファームウェアがソケットの事象を通知する場合（下の「ファームウェアの追加機能」）は、20 ms の代わりに IRQ の線の
  エッジ（最長 100 ms）で問い合わせます。
- どのソケットも上のとおりに扱います: MQTT と時刻同期のソケット（互換層の `zsock_*()` は NuttX のソケットの
  呼び出し）も、Telnet・iperf・nsh のコマンドのソケットも同じです。`nina_event` の優先度（106）は使う側のスレッド
  （MQTT 107、時刻同期 109）より低いので、その poll は使う側が眠るか待つ間に来ます。Zephyr 版は、ソケットを使う
  スレッドだけが NINA に尋ねます（MQTT の受信の確認は 100 ms ごと、時刻同期は応答を待つ間 1 ms ごと）。こちらでは、
  伝えるものがあるソケットのそれぞれに、`nina_event` が 20 ms ごとか IRQ の線のエッジで尋ねます。
- connect: 接続命令の直後に 1 回状態を確かめ（Zephyr 版のループの最初の 1 回）、終わっていなければ
  「処理中」と応答して、`nina_event` が 20 ms ごとに確かめます。期限は接続命令から 10 秒（`ETIMEDOUT`）。
  結果は、成功なら `SENDTO_READY`、失敗なら connect の要求への最後の応答で返すので、`connect()` の errno は
  Zephyr 版と同じです（SO_ERROR の 104/113/116/118 → `ECONNRESET`/`ECONNABORTED`/`ETIMEDOUT`/
  `EHOSTUNREACH`、それ以外とエラーフラグ → `ECONNREFUSED`）。失敗した NINA 側のソケットは閉じます。
- accept: 待ち受けのソケットに `RECVFROM_AVAIL` が立ってから要求が来ます（usrsock の仕様）。NINA に
  接続が無ければ `EAGAIN` で、ブロッキングのソケットでは usrsock が次のイベントを待ちます。新しい
  ソケットには `SENDTO_READY` を送ります。
- 対応していない操作は Zephyr 版と同じ errno です: `getsockopt`・`getsockname`・`getpeername`・`shutdown` は
  `EOPNOTSUPP`、`TCP_NODELAY` 以外の `setsockopt` は `ENOPROTOOPT`（`SO_RCVTIMEO`・`SO_SNDTIMEO` は NuttX が
  自分で扱います）。`ioctl` は `ENOTTY` を返して NuttX 自身の処理（ネットワーク
  デバイスの `SIOC*` など）に任せます。`socket(AF_INET, SOCK_CTRL)`（`ifconfig` などが使う）は `ENOTSUP` で
  NuttX 自身に任せます。
- 送受信の値は Zephyr 版と同じです: TCP の送信は 1 回 3584 バイトまで（`CONFIG_NINA_TCP_MAX_IO`）、
  UDP は 512 バイトまで（超えると `EMSGSIZE`）、受信は 1 回 512 バイトまで。`sendmsg` は複数のベクターを
  3584 バイトまで 1 つにつなげて 1 回の命令で書きます（MQTT のヘッダーと本体を 1 回で出すため。NuttX の
  usrsock はベクターを分けたまま渡します）。1 つのベクターの送信は `send`・`sendto`、複数なら `sendmsg` として
  扱います（UDP の `sendmsg` は Zephyr 版と同じく `EOPNOTSUPP`）。
- 送受信の待ち方: 要求 1 回で Zephyr 版の待ちのループを 1 周、呼び出したスレッドで行い、周と周の間は
  usrsock が待ちます。
  - 待つ TCP の送信で NINA が 1 バイトも受け取らなかったときは、`SENDTO_READY` を落として `EAGAIN` で
    応え、`CONFIG_NINA_SEND_RETRY_MS`（既定 1 ms。Zephyr 版と同じ）の後に
    `nina_event` が `SENDTO_READY` を送り、usrsock が同じ要求を出し直します（次の周）。最初の周と以後
    100 ms ごとに NINA の poll で状態を確かめ（エラーフラグなら `EPIPE`）、最初の周から 10 秒で `EAGAIN` を
    返します。期限と次の確認の時刻はソケットごとに持ち、同じ呼び出しの続きかどうかはデータのアドレスと
    長さで見分けます。
  - 待たない送信（`MSG_DONTWAIT`、非ブロッキングのソケット）は、Zephyr 版と同じく受け取られなければすぐ
    `EAGAIN` です。UDP は NINA が受け取らなかったら（NINA はどの失敗も 0 バイトと答えます）待つかどうかに
    関わらずすぐ `EAGAIN` です。この 2 つと 10 秒の期限では `SENDTO_READY` を保ったまま `-ETIME` で応え、
    パッチ 0005 がそれを `EAGAIN` にして呼び出しを終えます（元の usrsock は待つ呼び出しへの `EAGAIN` を
    出し直し続けます）。`SENDTO_READY` を落とすのは待つ送信の周の間だけです。落としたまま次のブロッキング
    送信が来ると、usrsock はドライバーに尋ねずに待ち、周も期限も働かないためです。そのため TCP の
    ソケットは `poll()` では常に書き込み可に見えます（Zephyr 版の `poll()` は NINA に尋ねます）。
  - 受信は、先に NINA の poll でデータの有無を確かめ（Zephyr 版と同じ。応答では「データなし」と空の
    データグラムを区別できないため）、無ければ `RECVFROM_AVAIL` を落として `EAGAIN` で応えます。待つ受信は
    usrsock が `RECVFROM_AVAIL` を待ち、それは `nina_event` が 20 ms ごとの poll で送ります（`SO_RCVTIMEO` は
    NuttX が扱います）。TCP の受信が求めた長さ（512 バイトまで）をすべて受け取ったときは `RECVFROM_AVAIL` を
    保ちます: まだ残っているかもしれず、次の受信が NINA に尋ねます（Zephyr 版の受信は毎回尋ねます）。それ以外の
    データを受け取った後は `RECVFROM_AVAIL` を落とし、`nina_event` がすぐに poll し直します。usrsock は
    `RECVFROM_AVAIL` が無いと受信をドライバーに渡さない（`MSG_DONTWAIT` の受信は usrsock 自身が `EAGAIN` を
    返す）ので、受信や `poll()` が新しいデータを知るのは `nina_event` の poll（最大 20 ms 後、または IRQ の線の
    エッジ）からです（Zephyr 版はその場で尋ねます）。`RECVFROM_AVAIL` を保たないと、分けて読むパケット（MQTT
    クライアントの CONNACK: 2 バイト、続いて 2 バイト）が最初の部分で止まります。`nina_event` が次の受信の後に
    なることがあるためです。
  - UDP のソケットの最初の送信（NINA 側のソケットを作る）のあとは、`nina_event` がすぐにそのソケットを poll
    します（ソケットの事象があるときは、この poll のあとで、時刻同期の応答などが届くと IRQ の線にエッジが出ます）。
  - データを見つけた poll（`nina_event`）は、次の受信の最初の poll の代わりに使います（その間に
    データを取る者はいません）。
- TCP の受信の 0 は、相手が閉じたか接続が失敗したことを示します（NINA はどちらも 0 バイトと答えます。
  Zephyr 版と同じ）。UDP は接続した相手（`connect()`）以外からのデータグラムを捨てます（Zephyr 版と同じ）。
- `common/drivers/nina/nina_sock.c` はホストテスト（`common/tests/nina_sock.c`）で、偽の `nina_proto_request()` と時計を使って
  確かめます。

### 名前解決

`common/drivers/nina/nina_dns.c`（3 つの版が使う）は、名前を NINA の名前解決の
命令（0x34 で名前を渡し、0x35 でアドレスを読む）で IPv4 のアドレス 1 つにします。数値のアドレスは NINA に
尋ねません。Zephyr 版はソケットのオフロードの `getaddrinfo()` として登録し、`zsock_getaddrinfo()` がこれを
使います。NuttX 版は専用の関数 `nina_dns_getaddrinfo()`・`nina_dns_freeaddrinfo()`（`common/drivers/nina/include/nina_dns.h`）で、互換層の
`<zephyr/net/socket.h>` が `zsock_getaddrinfo()`・`zsock_freeaddrinfo()`・`struct zsock_addrinfo` をこれに
つなぎます。NuttX の `getaddrinfo()`（`CONFIG_LIBC_NETDB`。DNS サーバーへ自分で UDP で問い合わせる）は
使わず、`CONFIG_NINA_DNS` は `CONFIG_LIBC_NETDB` と同時には有効にできません。エラーは NuttX の
`EAI_*` の値で、名前は Zephyr 版の `DNS_EAI_*` と同じです（名前が引けない `EAI_NONAME`、SPI の失敗
`EAI_AGAIN`、アドレスの長さが違う `EAI_FAIL`、ヒントの誤り `EAI_BADFLAGS`・`EAI_FAMILY`・`EAI_SOCKTYPE`、
サービスの誤り `EAI_SERVICE`、メモリー不足 `EAI_MEMORY`）。2 つの命令の間はミューテックスで守ります
（NINA の名前解決の結果は 1 つしかないため）。結果は Zephyr 版の 2 KB の専用ヒープではなく `malloc()` で
取ります。ホストテストは `common/tests/nina_dns.c` です。これを使うのは、時刻同期の NTP サーバーを名前で
指定したときだけです（既定はゲートウェイ）。

### Wi-Fi（ネットワークデバイス `wlan0`）

| ファイル | 役割 |
|---|---|
| `common/drivers/nina/nina_wifi.c`・`include/nina_wifi.h` | コアの Wi-Fi の操作（ほかの版と同じファイル）。スキャン・接続・切断・状態・接続の監視と、3 つのスレッド `nina_scan_tid`・`nina_connect_tid`・`nina_monitor_tid` の本体。起きたことはつなぎのコールバック（`struct nina_wifi_ops`）に返す |
| `common/drivers/nina/port/nuttx/nina_netdev.c` | NINA を NuttX のネットワークデバイス `wlan0` として登録し、wapi の無線の ioctl をコアの呼び出しにし、コアの結果を `wlan0`（アドレス、キャリア）に反映し、3 つのスレッドを始める。NuttX 版だけ |
| `common/drivers/nina/port/nuttx/nina_wifi_mgmt.[ch]` | アプリが使う Zephyr の Wi-Fi 管理（互換層の `net_mgmt()`）をコアの上で：`net_if_get_default()` のインターフェース、`NET_REQUEST_WIFI_CONNECT`（`NET_REQUEST_WIFI_CONNECT_STORED` が保存したネットワークごとに出す）、接続のイベント、Wi-Fi のプロファイルの再接続をコアのフックに（`CONFIG_WIFI_PROFILE` のとき `common/imu/wifi_profile.c`。無ければ接続はすべて許し、再接続しない） |
| `common/drivers/nina/port/nuttx/nina_net.h` | `nina_net_initialize()`（ボードの起動処理）と `nina_net_start()`（init が実行するアプリのスレッドの表、`common/imu/app_init.c`） |

- 操作は Zephyr 版と同じです: 監視は 2 秒ごとに、接続中なら NINA の状態（0x20）を見て、2 回続けて未接続なら切断として
  IP の設定を消し、切断のイベントを出して再接続を頼みます。接続は NINA に SSID（とパスフレーズ）を渡した後、
  500 ms ごとに最大 60 回状態を見て、接続できたら省電力を切り（0x17）、IP の設定（0x21）を読んで
  インターフェースに入れ、結果のイベント（失敗は `-ETIMEDOUT` など）を出します。要求の世代
  （`generation`／`request_generation`）が変わった古い接続は結果を出さずに終わります。スキャンは接続中なら
  先に切断として扱い、終わった後に再接続を頼みます（ファームウェアが接続を保つ場合を除く。下の「ファームウェアの
  追加機能」）。手動の切断は再接続を止めます（`wifi_profile_pause_reconnect()`）。
  パスフレーズはログに出しません。
- 起動: ボードの起動処理が `nina_net_initialize()` を LSM6DSOX の後、KV の前に呼びます（Zephyr 版がインターフェースを
  `POST_KERNEL` でセンサーの後に初期化し、NVS を `main()` で用意するのと同じ順）。`net_mgmt` のスレッドを始めて互換層の
  インターフェースを登録し、`wlan0` を `NET_LL_IEEE80211` で登録して `IFF_UP`・キャリアなしにし、NINA から MAC アドレス
  （0x22）と追加機能を読みます（`nina_wifi_init()`。Zephyr と同じ順）。これが NINA への最初の命令なので、ここで NINA の
  リセット（10 ms、750 ms）が入ります。3 つのスレッドは、Zephyr が静的なスレッドを始めるところで init が
  `nina_net_start()` で始めます（アプリのスレッドの表、`common/imu/app_init.c` の `app_threads_start()` の最初の項目）。静的なスタックのカーネルスレッド（`kthread_create_with_stack()`）で、優先度とスタックは
  `priorities.h` の値（110、110、109。1536、1024、1536 バイト）、1 つが動き出してから次を始めます。
- インターフェースの値: IPv4 のアドレス・ネットマスク・ゲートウェイは `wlan0` の `d_ipaddr`・`d_netmask`・
  `d_draddr`、MAC アドレスは `d_mac` に書きます。NuttX の `ifconfig`（`/proc/net/wlan0`）と `SIOCGIFADDR` などは
  そこを読みます。Zephyr の「休止（dormant）」は `IFF_RUNNING` が無い状態です（接続すると `RUNNING` が付く）。
  `wlan0` には `d_ifup`・`d_ifdown` が無く、`ifconfig wlan0 down`・`up` は何もしません（Zephyr 版の
  インターフェースも常に有効）。NuttX 自身の TCP/IP は使わないので、ARP と ICMP は切ってあります
  （`CONFIG_NET_ARP`・`CONFIG_NET_ICMP`）。`CONFIG_NET_ETHERNET` は、`NET_LL_IEEE80211` の登録
  （`netdev_register()` が `CONFIG_NET_ETH_PKTSIZE` を使う）と MAC アドレスの欄（`d_mac.ether`）に要るので
  残します。

wapi の ioctl（`d_ioctl`。呼び出したスレッドで処理）は次のとおりです。

| ioctl | 処理 |
|---|---|
| `SIOCSIWSCAN` | `nina_wifi_scan()`。ESSID の指定、パッシブ、チャネルの指定は、Zephyr 版のドライバーと同じく `-ENOTSUP` で断る |
| `SIOCGIWSCAN` | スキャン中は `-EAGAIN`、失敗ならそのエラー、終わっていれば結果（コアのスキャンのコールバックで集めた最大 10 件を、NINA の順に、BSSID・チャネル・RSSI（dBm）・SSID・暗号化の有無の無線イベントにする。暗号化の種類が不明なら有無を付けない）。バッファーが足りなければ `-E2BIG` |
| `SIOCSIWAUTH` | WPA の版と pairwise の暗号を覚える |
| `SIOCSIWENCODEEXT` | パスフレーズを覚える。この 2 つで覚えた設定は次の `SIOCSIWESSID` で使って忘れる（パスフレーズは消す）ので、`wapi psk` を付けない `wapi essid` は暗号化なしの接続になる |
| `SIOCSIWESSID` | 覚えた設定で `nina_wifi_connect()`。WPA2 → PSK、無し → 暗号化なし、WPA・WPA3・WEP はコアが断る（`-ENOTSUP`）。その前に Zephyr の Wi-Fi 管理と同じ鍵の検査をする（`-EINVAL`: WEP の鍵が 5・10・13・26 バイト以外、ほかの鍵が 8〜64 バイト以外、鍵の要るセキュリティで鍵が無い）。版を設定せずにパスフレーズだけがあれば PSK（Zephyr の `wifi connect` で `-k` を省いたときと同じ）。フラグ 0（wapi の `disconnect` が送る乱数の SSID）は何もしない |
| `SIOCSIWAP` | 00:00:00:00:00:00 か ff:ff:ff:ff:ff:ff なら `nina_wifi_disconnect()`。ほかは `-ENOTSUP` |
| `SIOCGIWESSID`、`SIOCGIWAP`、`SIOCGIWSENS` | `nina_wifi_status()`（NINA に尋ねる）の SSID、BSSID、RSSI。未接続なら SSID は空、BSSID は 0、RSSI は `-ENOTCONN` |
| `SIOCGIWMODE`、`SIOCSIWMODE` | managed（`IW_MODE_INFRA`）だけ |

ほかの無線の ioctl は `-ENOTTY` です。接続の結果は `wapi essid` の戻りでは分からず（Zephyr 版の `wifi connect` も
要求を出して戻る）、`ifconfig` の `RUNNING` と IP アドレス、`wapi show` の ESSID で確かめます。

- 資格情報の保存と自動再接続（`nuttx/apps/wifi/`。Zephyr 版と同じファイル `common/imu/wifi_profile.c` と、Zephyr の
  `subsys/net/lib/wifi_credentials/` の写し、`wifi cred`。`design/nuttx-porting-ja.md` の「Wi-Fi の資格情報と自動再接続（`wifi cred`）」）は、
  互換層の `net_mgmt()`（`NET_REQUEST_WIFI_CONNECT_STORED`）、`net_mgmt_add_event_callback()`
  （`NET_EVENT_WIFI_CONNECT_RESULT`）、`net_if_get_wifi_sta()`、`net_if_ipv4_get_global_addr()`、`k_current_get()` を
  Zephyr 版と同じ形で使い、`nina_wifi_mgmt.c` がそれをコアの上で受けます。コアは接続の要求のたびに `wifi_profile_connection_allowed()` を呼び、
  再接続のスレッドからの要求を、手動の切断の後は `-ECANCELED` で断ります（Zephyr 版と同じ）。
- ホストテスト: コアの操作は `common/tests/nina_wifi.c`（`common/tests/run.sh`）、`wlan0` と wapi の ioctl をコアの上で
  `nuttx/tests/nina_netdev.c`、アプリの管理を `nuttx/tests/nina_wifi_mgmt.c`（`script/host_tests.sh`）が確かめます。

### ファームウェアの追加機能（パッチ 0003〜0006）

コア（`nina_proto.c`、`nina_wifi.c`、`nina_sock.c`、`nina_wait.c`）の動作は Zephyr 版と同じです（説明は
`zephyr/app/usb-console/drivers/README_ja.md` の同名の節、仕様は `firmware/nina-fw/README.md` の
「Host interface of 0003 to 0006」）: インターフェースの初期化で 1 回 0x6A を尋ね（古いファームウェアは `0xEF` の
応答で「機能なし」）、ビット 0 で接続中のスキャンを切断として扱わず、ビット 1 で 0x36 → 0x6B（100 ms ごと、10 秒まで）
→ 0x27 の非同期のスキャンにし、ビット 2 でソケットの待ちを IRQ の線（GPIO2）のエッジで起こします。`nina version` は
2 行目にモジュールのビットと使っているビットを出します。TCP 送信の再試行で、全量を送り直す前に poll 命令で書き込み
可能を確かめるのは、ファームウェアに依らない変更です。

NuttX 版のビット 2 の待ち:

- `common/drivers/nina/port/nuttx/nina_os.c` の GPIO2 の割り込み（ボードの `src/rp2040_gpio_os.c`、互換層の `gpio0` のコールバック）がエッジを数え、
  待っている全スレッドの `sem_t` を post し（ブロードキャスト）、`nina_event` も起こします（`nina_poller_edge()`）。
- `nina_event` は、前の周の前に控えたエッジの数（または受信の要求が命令の前に控えた数）から数が変わったら、
  20 ms を待たずに次の周を始めます。変わらなければ 100 ms（接続中のソケットがあればその期限まで）で周を始めます。
  エッジで始めた周が何も伝えないことが続くと、次のエッジまでに少し間を置きます（1、2、4、8、16 ms、以後 20 ms）。
- 待つ TCP 送信の次の周: poll だけをした周（書き込み可能でなかった）のあとは、その要求が命令の前に控えた数から
  変わったとき（最長 100 ms、送信の期限まで）、データを送って 0 バイトだった周のあとはビット 2 が無いときと同じく 1 ms 後です。
- `nina_os_event_start()` が失敗したときは、ビット 2 があっても、ビット 2 が無いときの間隔で待ちます。
- `nina_os.c` の割り込みと待ち、Wi-Fi の操作のロックと要求は `nuttx/tests/nina_os.c`（`nina_os`）が確かめます。`nina_event` の
  エッジでの周は、ホストテストで確かめていません。

### 中継モード（`nina passthrough confirm`）

PC の esptool で NINA のフラッシュを読み書きするための、USB の CDC ACM と NINA の UART（ESP32 の UART0）の中継です。
中継そのもの（要求の印、線の対応、レート、リングを通した両方向のデータ、出口、数、LED）は 3 版で共有する NINA の
ドライバーのコア `common/drivers/nina/nina_passthrough.c`（`nina_passthrough.h`）です。NuttX 側は、cdcacm の上の
ホスト側 `common/drivers/nina/port/nuttx/nina_passthrough_os.c`（NINA のドライバーと一緒にビルド。`nuttx/drivers/nina/Make.defs`）と、
RP2040 のレジスターのモジュール側（`port/nuttx/nina_passthrough_bus_rp2040.c`）、`CONFIG_NINA_PASSTHROUGH`（既定 y）です。

| RP2040 | 中継モード | 通常のファームウェア |
|---|---|---|
| GPIO8 | UART1 TX → モジュールの UART0 RX | SPI1 RX（MISO） |
| GPIO9 | UART1 RX ← モジュールの UART0 TX | CS |
| GPIO2 | モジュールの GPIO0（起動モード） | IRQ |
| GPIO3 | モジュールの RESETN（アクティブ low） | RESET |

- 入り方: `nina passthrough confirm`（`nuttx/apps/nina`）が watchdog の scratch 0〜3 に印を書き、watchdog で
  再起動します（`common/chip/rp2040` の `rp2040_watchdog_reboot()`。Pico SDK の `watchdog_reboot(0, 0, 0)` と
  同じ）。`rp2040_boardinitialize()` が最初に印を読んで消すので、RESET・以後の再起動・異常停止は通常の起動に
  なります。フラッシュ（KV）には何も書きません。
- 中継モードの起動: `rp2040_bringup()` は USB（CDC ACM、同じ記述子とシリアル番号）だけを用意し、SPI1・NINA・I2C0・
  センサー・`wlan0`・KV のフラッシュを起こしません。init タスクは nsh・Telnet・Wi-Fi・スレッドの代わりに
  `nina_passthrough_run()` を実行します（優先度 120）。
- 線とレート: cdcacm のコールバック（`CAIOC_REGISTERCB`。USB の割り込みから呼ばれる）が、
  `CAIOC_GETCTRLLINE`・`CAIOC_GETLINECODING` で新しい状態を読み、その場でピンに反映します（RTS が有効 → リセット解除、
  DTR が有効 → GPIO0 を LOW）。ホストの変化はどれほど短くても順に反映されます。UART1 は line coding のレートに
  追従します（8N1 固定）。GPIO0 を 750 ms 後に離すこと、閉じたときのログ、LED（GPIO6）の 5 Hz の点滅、出口は
  init タスクがセマフォで待って行います。
- モジュール → PC: UART1 の受信割り込み（FIFO に 4 バイト、または 32 ビット時間の無受信で発生）が 16 KiB の
  リング（中継モードのときだけヒープから取る）へ移し、カーネルスレッド `nina_pt_rx` が `/dev/ttyACM0` に書きます。
  921600 bps で FIFO の残り 28 バイト（300 µs）が割り込みの遅れの許容で、リングは PC が読まない間の 178 ms 分です。
  溢れたバイトは数えます。
- PC → モジュール: `nina_pt_tx` が `/dev/ttyACM0` を読んで UART1 の送信 FIFO に入れ、満杯の間は送信割り込みを待ちます。
  入れ終わるまで次を読まないので、シリアルドライバーの受信バッファーが埋まると cdcacm がパケットを受け取らず
  （ホストが待つ）、取りこぼしません。
- `/dev/ttyACM0` はコンソールではないので変換もエコーもありません（termios のフラグも 0 にします）。
- 出口: ポートを 300 bps にして閉じる（`stty -F /dev/ttyACM0 300`）か RESET。モジュールを `init_nina()` と同じ手順
  （GPIO0 を HIGH、RESETN を 10 ms LOW、750 ms、GPIO0 を入力）で通常の起動に戻してから watchdog で再起動します。
- RAM のログ（抜けたあとの `dmesg`）に、開始・ポートを閉じるたび・終了の行が、送受信のバイト数、溢れ、UART の
  オーバーラン・ブレーク・フレーミングエラーの数とともに残ります。
- シグナルは使いません（待ちはセマフォ、sleep、ブロッキングの read・write）。NuttX の UART1 ドライバー
  （`CONFIG_RP2040_UART1`）は起動時にピンを取るので使いません（UART1 のレジスターを直接扱う）。
- ホストテスト `nuttx/tests/nina_passthrough.c`（`script/host_tests.sh`）が、`nina passthrough` の確認の語と引数と
  メッセージを確かめます。共有のコアは `common/tests/nina_passthrough.c`（`common/tests/run.sh`）が確かめます。
  NuttX 側（割り込み・スレッド・cdcacm との組み合わせ）はホストテストでは確かめていません。
- 実機（2026-11-18）: esptool で `read-flash 0 0x200000` を 115200 bps（192 秒）と 921600 bps（26.1 秒）で行い、2 つの読み出しが
  一致し、取りこぼしとオーバーランは 0 でした。300 bps で閉じると通常の起動に戻りました。

`bootsel` コマンド（`nuttx/apps/bootsel`）は、ボードの `rp2040_reset_usb_boot()`（`include/rp2040_restart.h`）で
ブート ROM の `reset_usb_boot(0, 0)`（活動 LED なし、マスストレージと PICOBOOT の両方を有効）を呼び、BOOTSEL
（`RPI-RP2`）に再起動します。

## I2C0

I2C0 と I2C1 のドライバーは、FreeRTOS 版と共有する I2C のコアです（コアそのもの、Zephyr 版のドライバーとの関係、
DMA を使うほかのコードの規則の共通の部分は[設計書](../../design/imu-raw-mqtt-design-ja.md)の §12）。NuttX の
`rp2040_i2c.c`（`CONFIG_RP2040_I2C`）は使いません。コアはリポジトリの `common/drivers/i2c_dw/` にあり、NINA のドライバーと同じ
3 層です。直下に `i2c_dw.c`、ドライバーを使う側のヘッダーは `include/`、コアの内側のヘッダーは `internal/`、2 つの境界は
`port/`、NuttX のつなぎは `port/nuttx/` にあります（ボードの `src/Make.defs` が `VPATH` でソースを参照し、4 つのディレクトリーを
include パスに足す。オブジェクトはボードのディレクトリーにできる）。

| ファイル | 役割 |
|---|---|
| `common/drivers/i2c_dw/i2c_dw.c` | I2C0 と I2C1 のコントローラーのコア。Zephyr のドライバーが有効なノードごとに作るのと同じく、それぞれ 1 つのインスタンス（`i2c_0_runtime`・`i2c_1_runtime`。`CONFIG_I2C_DW_I2C0`・`CONFIG_I2C_DW_I2C1` で作る）を持ち、両者で違う値は `I2C_DW_INST()` で選ぶ。Zephyr 版の `i2c_dw.c`（Zephyr のドライバーにローカルのパッチ 0003 を当てたもの）のコントローラーの部分（転送の状態機械、割り込みハンドラーの本体、`i2c_dw_data_ask()`・`_read()`・`_send()`、`i2c_dw_setup()`、`i2c_dw_configure()`、初期化のレジスター操作）と DMA での読み出し（`i2c_dw_rp2040_dma_read()` など）と `i2c_dma` のカウンターを、関数・分岐・順序・レジスター値・期限・エラー値を合わせて写したもの。ファイル名はパッチ後のドライバーに合わせ、DMA での読み出しとそのコマンド語は `internal/i2c_dw_rp2040_dma_priv.h`、RP2040 の FIFO の深さと RX のまとめ数は `internal/i2c_dw_rp2040.h` にあり、どちらも 1 回だけ読み込む。末尾でつなぎの `i2c_dw_instance.h` を読む |
| `common/drivers/i2c_dw/port/i2c_dw_os.h` | 境界: コアが OS に求めるもの（下の表）。先頭でつなぎの `i2c_dw_os_impl.h` を読み、その実装が満たすべき関数を宣言する |
| `common/drivers/i2c_dw/port/i2c_dw_dma_os.h` | DMA 読み出しの DMA の境界: OS 自身の DMA ドライバーに求めるもの（下の 2 つ目の表）。パッチを当てたドライバーの Zephyr の DMA API の呼び出しに 1 つずつ対応する。OS の名前を含まず、コアは DMA のレジスターに触れない |
| `common/drivers/i2c_dw/internal/i2c_dw.h` | OS に依存しない定義と純粋な関数: Zephyr 版の `i2c_dw.h` の定義、SCL のカウント、周波数から速さの区分、NuttX のフラグ（`I2C_M_*`）から Zephyr のフラグ（`I2C_MSG_*`）への変換、`IC_DATA_CMD` に書く語。ホストテスト `common/tests/i2c_dw.c` |
| `common/drivers/i2c_dw/internal/i2c_dw_registers.h` | Zephyr 版の `i2c_dw_registers.h` の写し（アクセス関数は 32 ビットの volatile のアクセスで定義。番地の型は Zephyr の `mm_reg_t` の代わりに `uintptr_t`） |
| `common/drivers/i2c_dw/include/i2c_dw_rp2040_dma.h` | `i2c_dma` コマンド（3 版で共有の `common/shell/i2c_dma_shell.c`。この版のビルドは `nuttx/apps/i2c_dma`）が読み書きする状態: DMA の有効・無効と 4 つのカウンター（`i2c_dw_rp2040_dma_get_stats()`・`_enabled()`・`_set_enabled()`）。コマンドは Zephyr の名前 `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>` で読み、それは `common/compat/zephyr/drivers/i2c/` の同名のヘッダーがこのファイルを読む形です。宣言はパッチ 0003 が Zephyr のツリーに足すヘッダーと同じ（`i2c_dw_sync` が確かめる）。`include/` のヘッダーはこれだけで、そのディレクトリーだけでコンパイルできる（`common/tests/run.sh` の `i2c_dw_public_headers`） |
| `common/drivers/i2c_dw/port/nuttx/i2c_dw_os_impl.h` | 境界の NuttX の実装: セマフォ（`sem_t`。`nxsem_post()`、`nxsem_tickwait_uninterruptible()`、`nxsem_wait_uninterruptible()`、`nxsem_reset()`）、時刻（システムの tick）、`nxsched_msleep()`、`up_irq_save()`、メッセージ（`struct i2c_msg_s` と、`i2c_dw_msg_flags()` での Zephyr のフラグへの変換）、`I2C_DW_CLOCK_SPEED`（`BOARD_PERI_FREQ`） |
| `common/drivers/i2c_dw/port/nuttx/i2c_dw_instance.h` | I2C0 と I2C1 の NuttX の部分: `struct i2c_master_s` の操作表（`I2C_TRANSFER()` は要求を確かめて周波数から速さを決め、コアの転送を呼ぶ。`I2C_RESET()`）、`irq_attach()` で登録する割り込みハンドラー、リセットとピン（`i2c_dw_pinctrl()`）は両方で共有し、`i2c_dw_i2c0_initialize()` と `i2c_dw_i2c1_initialize()`。ホストテストは `nuttx/tests/i2c_dw_instance.c` |
| `common/drivers/i2c_dw/port/nuttx/i2c_dw_dma_os_rp2040.c` | DMA の境界の NuttX の実装。NuttX 自身の DMA ドライバー `arch/arm/src/rp2040/rp2040_dmac.c`（`CONFIG_RP2040_DMAC`）の上に作る。ホストテストは `nuttx/tests/i2c_dw_dma_os.c` |
| `common/drivers/i2c_dw/port/nuttx/i2c_dw_controllers_rp2040.h` | `i2c_dw_i2c0_initialize()` と、`CONFIG_I2C_DW_I2C1` のとき `i2c_dw_i2c1_initialize()`（起動時に `rp2040_bringup()` が I2C0、I2C1 の順に呼び、`/dev/i2c0`・`/dev/i2c1` に登録する）。ボードのコードは include パスの `port/nuttx` で見つける |

境界（`port/i2c_dw_os.h`）で OS が用意するもの。どれも static inline の関数かマクロで、OS の割り込みハンドラーと転送の入口は
コアの static inline の関数を呼ぶので、コンパイラーがそれぞれを 1 つの関数にします（転送と割り込みの経路に呼び出しが
増えない）。「割り込み」の印のものは割り込みハンドラーから呼ばれます。

| 境界 | 意味（Zephyr） | NuttX 版 |
|---|---|---|
| `i2c_dw_os_sem_t` | 数を数えるセマフォ（`k_sem`。`bus_sem` は優先度継承なしの 2 値として使う） | `sem_t` |
| `i2c_dw_os_sem_give()`（割り込み） | `k_sem_give()` | `nxsem_post()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `k_sem_take(sem, K_MSEC(ms))`: 取れたら 0、期限（tick に切り上げ）が先なら 0 以外。シグナルで終わらない | `nxsem_tickwait_uninterruptible(sem, MSEC2TICK(ms))` |
| `i2c_dw_os_timepoint_t`、`i2c_dw_os_timepoint_calc(ms)`、`i2c_dw_os_sem_take_until(sem, end)` | `sys_timepoint_calc()` と `k_sem_take(sem, sys_timepoint_timeout(end))`（期限を過ぎていても待たずに 1 回取ってみる） | システムの tick（`clock_systime_ticks()`）と、残りを 0 で止めた `nxsem_tickwait_uninterruptible()` |
| `i2c_dw_os_sem_reset()` | `k_sem_reset()` | `nxsem_reset(sem, 0)` |
| `i2c_dw_os_bus_lock()`・`_unlock()` | `bus_sem` の `k_sem_take(K_FOREVER)`・`k_sem_give()`。失敗した lock の errno は転送の戻り値になる | `nxsem_wait_uninterruptible()`・`nxsem_post()` |
| `i2c_dw_os_msleep(ms)` | `k_msleep()` | `nxsched_msleep()` |
| `i2c_dw_os_uptime_ms()` | `k_uptime_get()`（マクロでもよい） | `TICK2MSEC(clock_systime_ticks())`（マクロ） |
| `i2c_dw_os_irq_lock()`・`_unlock()`（割り込み） | `irq_lock()`（`atomic_or()`）。入れ子にできる | `up_irq_save()`・`up_irq_restore()` |
| `i2c_dw_os_msg_t`、`I2C_DW_OS_MSG_BUF()`・`_LEN()`・`_FLAGS()` | 転送の 1 つのメッセージ（Zephyr の `struct i2c_msg` なら `buf`・`len`・`flags`。最後のメッセージには STOP） | `struct i2c_msg_s`、フラグは前後のメッセージから `i2c_dw_msg_flags()` で作る |
| `I2C_DW_OS_DEV_HEAD` | コントローラーのデータの先頭に置く OS のデバイス（空でもよい） | `struct i2c_master_s dev;` |
| `I2C_DW_CLOCK_SPEED` | `CONFIG_I2C_DW_CLOCK_SPEED`（MHz、125） | `BOARD_PERI_FREQ / 1000000` |
| `i2c_dw_os_prepare(dw)`（`i2c_dw_instance.h`） | `reset_line_toggle_dt()` と `pinctrl_apply_state()`: I2C0 か I2C1 のリセット、そのピンを I2C（4 mA、遅いスルー、シュミットと入力を有効。I2C0 の GPIO12・13 はプルなし、I2C1 のピンは内部プルアップ） | RESETS と `rp2040_gpio_*()`（`i2c_dw_pinctrl()`。Zephyr の `pinctrl_rpi_pico.c` の手順） |
| `i2c_dw_os_irq_enable(dw)`（同） | `IRQ_CONNECT()`・`irq_enable()`: I2C0_IRQ か I2C1_IRQ で `i2c_dw_isr_handler(dw)` を呼ぶハンドラー | `irq_attach()`・`up_enable_irq()` |
| `app_log.h` | `LOG_MODULE_REGISTER()`、`LOG_ERR()`、`LOG_ERR_RATELIMIT()`（レベルはコアが定める 3） | `common/compat/log/app_log.h` と NuttX の部分 `nuttx/drivers/include/app_log_os.h`（RAM ログ） |
| `CONFIG_I2C_DW_I2C0`・`CONFIG_I2C_DW_I2C1` | 作るインスタンス（Zephyr ではデバイスツリーの有効なノード） | `nuttx/drivers/i2c_dw/Kconfig`（既定で `I2C_DW_I2C0` は有効、`I2C_DW_I2C1` は無効） |
| `CONFIG_I2C_DW_RP2040_DMA` | I2C0 の DMA 読み出しを作る（Zephyr 版と同じ名前のシンボル） | Kconfig |

DMA の境界（`port/i2c_dw_dma_os.h`）で OS の DMA ドライバーが提供するもの。`port/nuttx/i2c_dw_dma_os_rp2040.c` の普通の関数です
（読み出しごとに呼ばれ、バイトごとではない）。呼ぶのはバスのセマフォを持った転送のスレッドだけで、コールバックは
DMA の割り込みから呼ばれます。どれも DMA のほかの利用者を待ちません。

| 境界 | 意味（Zephyr） | NuttX 版 |
|---|---|---|
| `i2c_dw_dma_os_initialize()` | DMA のデバイスの起動（Zephyr では I2C0 より前、ここでは I2C0 のコントローラーが起動したあと）。何もリセットしない | `rp2040_dmachannel_trywait()` を 2 回（空いている最も小さい番号、待たない）、その割り込みを切る。2 つのチャネルは読み出し用に持ち続ける。2 つとも取れなければ、取れた方を返して `-EBUSY` を返す |
| `i2c_dw_dma_os_is_ready()` | `device_is_ready(dma)` | 2 つのチャネルが取れた |
| `i2c_dw_dma_os_request_channel()`・`_release_channel()` | `dma_request_channel(dma, NULL)`・`dma_release_channel()`。取得は待たない（`-EBUSY`・`-ENODEV`） | 2 つのうち、実行中の読み出しが取っていないもの。返すと次の読み出しが使える |
| `i2c_dw_dma_os_config_tx()`・`_config_rx()` | 各側の `dma_config()`: 16 ビットのコマンド語を DREQ 32 の歩調で `IC_DATA_CMD` へ、そこからバイトを DREQ 33 の歩調で。優先度は通常 | `rp2040_txdmasetup()`（`RP2040_DMA_DREQ_I2C0_TX`、`RP2040_DMA_SIZE_HALFWORD`、読み出しアドレスを進める）、`rp2040_rxdmasetup()`（`RP2040_DMA_DREQ_I2C0_RX`、`RP2040_DMA_SIZE_BYTE`、書き込みアドレスを進める） |
| `i2c_dw_dma_os_start()` | `dma_start()`: チャネルの古いフラグを消し、割り込みを有効にする | チャネルのビットを INTS0 と INTE0 に書き、コールバックを付けて `rp2040_dmastart()` |
| コールバック（割り込み） | チャネルの終了（0）またはバスエラー（`-EIO`） | `rp2040_dmac.c` の DMA_IRQ_0 のハンドラーから（その状態 `EIO` を `-EIO` にする。エラーならチャネルをすぐ abort する。下を参照） |
| `i2c_dw_dma_os_stop()` | `dma_stop()`: 割り込みを切って abort を求め、待たない | チャネルの INTE0 のビットを落とし、CHAN_ABORT にビットを書く（期限なしに待つ `rp2040_dmastop()` は使わない） |
| `i2c_dw_dma_os_is_busy()` | `dma_channel_is_busy()` と、チャネルの CHAN_ABORT のビット | チャネルの CTRL_TRIG の BUSY（`rp2040_dma_register()`）、CHAN_ABORT |
| `i2c_dw_dma_os_acknowledge()` | 遅れて来た完了のフラグ（RP2040-E13）を消す | チャネルのビットを INTS0 に書く |

INTE0（セットとクリアのエイリアス経由）、INTS0、CHAN_ABORT への書き込みは、`rp2040_dmac.c` と同じく
そのチャネルのビットだけで、ほかのチャネルには触れません。

**デバイスと転送**

OS の I2C0 のデバイス（`i2c_dw_instance.h`）は、`i2c_0_runtime` の `base`（0x40044000）を設定し、セマフォを
初期化し（`device_sync_sem` と `rp2040_dma_sem` は 0、`bus_sem` は 1）、起動時に `i2c_dw_turn_on()` を 1 回呼び、
成功したら `i2c_dw_dma_os_initialize()` を呼んで `ready` を立てます（起動しなかったコントローラーは DMA のチャネルを取りません）。I2C1 のデバイスは `i2c_1_runtime`
（0x40048000）に同じことを、DMA の部分とそのセマフォを除いて行います。デバイスごとにセマフォを持つので、2 つのバスの
転送は並んで進みます。転送の入口（両方で 1 つ）は、Zephyr の API で表せない要求（メッセージの配列がない、
256 個以上、メッセージごとに違うアドレスや速さ、10 ビットアドレス、バッファーがない、準備ができていない）を断り、
メッセージが 0 個なら 0 を返し、ほかは `i2c_dw_transfer_msgs(dw, msgs, count, address, speed)` の結果を返します
（`speed` は Zephyr の速さの区分）。この関数は、0 バイトのメッセージを含む転送を、何も送る前に `-ENOTSUP` で
断ります（コントローラーは 0 バイトの転送ができない。i2c ツールの `dev -z` は、ローカルのパッチ `apps/0007-i2ctool-zero-byte-scan-not-supported` により、最初のアドレスで `i2ctool: dev: bus 0 does not support zero-byte writes; no -z scan` を表示して止まり、失敗する）。I2C0 と DMA の割り込みは同じ優先度で（`CONFIG_ARCH_IRQPRIO` は無効で、
すべての割り込みが既定の優先度）、ハンドラーと転送は 1 つのコアで動きます。

- 転送は呼び出したスレッドが進め、バイトの出し入れは割り込みで行います。呼び出し元はセマフォで完了を
  待ちます（期限 1 秒。過ぎると abort して `-ETIMEDOUT`）。転送の全体をバスのセマフォで直列化します
  （Zephyr 版と同じく優先度継承なし）。
- NuttX のメッセージは、最後のメッセージと、`I2C_M_NOSTOP` がなく次が `I2C_M_NOSTART` でないメッセージの
  後に STOP、`I2C_M_NOSTOP` の後の `I2C_M_NOSTART` でないメッセージの先頭に RESTART を付けて、Zephyr の
  メッセージと同じに扱います。向きが変わると（前の転送の最後のメッセージから見ても）RESTART を付けるのは
  Zephyr 版と同じです。
- 速さはメッセージの周波数（100 kHz、400 kHz、1 MHz）で決め、変わったときに Zephyr の `i2c_configure()` と
  同じ計算をします。認めない速さでは何も変えずに `-EINVAL` を返します（Zephyr 版は最小のカウントを
  残してしまう）。
- 移していないもの: ターゲット（スレーブ）モード、Intel LPSS の DMA、`CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`、
  SDA/SCL の期限（`CONFIG_I2C_DW_EXTENDED_SUPPORT`）、電源管理、PCIe、ほかの SoC 向けの処理、バスのリカバリーと
  確認のコールバック（このボードでは登録されない。転送の前のリカバリーは何もせず、`I2C_RESET()` は `-ENOTSUP` を返す）。

**I2C1**（`CONFIG_I2C_DW_I2C1`、既定で無効。ボード上では何もつながっていない）

- 同じコアの 2 つ目のインスタンスで、GPIO18（SDA、Arduino の D6）と GPIO19（SCL、D7）を内部プルアップ付きで使います
  （ボードの `include/board.h` の `BOARD_I2C1_SDA_GPIO`・`BOARD_I2C1_SCL_GPIO`）。常に割り込みで転送します（DMA での
  読み出しは I2C0 だけ）。`CONFIG_I2C_DRIVER` があれば `/dev/i2c1` に登録します。起動に失敗してもログに残すだけで、
  ほかは変わりません。
- defconfig を変えずに作るには、一度ビルドしたあと `printf 'CONFIG_I2C_DW_I2C1=y\nCONFIG_I2CTOOL_MAXBUS=1\n' >> nuttx/nuttx/.config`、
  `make -C nuttx/nuttx olddefconfig`、`nuttx/script/build.sh`（`--clean` を付けない。付けると defconfig に戻る）。
  i2c ツールのバス 1（`i2c dev -b 1 0x08 0x77`）には `CONFIG_I2CTOOL_MAXBUS` 1 が要ります。
- I2C1 はビルドとホストテスト（`nuttx/tests/i2c_dw_instance.c`）で確かめただけで、実際のデバイスにつないで使ったことは
  ありません。

**DMA での読み出し**（`CONFIG_I2C_DW_RP2040_DMA`、既定で有効。Zephyr 版と同じ名前のシンボル）

コアの `common/drivers/i2c_dw/internal/i2c_dw_rp2040_dma_priv.h`（`i2c_dw.c` が 1 回だけ読み込む）が、パッチ後のドライバーの
DMA の部分を同じ分岐・順序・値で写したものです。

- 使う条件: DMA が有効（`i2c_dma mode`）、DMA の準備ができている（起動時に 2 つのチャネルが取れた）、
  32〜3584 バイト（`I2C_DW_RP2040_DMA_MAX_READ`）、STOP で終わる、転送の最後のメッセージ、7 ビットアドレス。
  外れると `-EAGAIN` で割り込みの処理に戻ります。チャネルの確保か設定に失敗したときも、`dma_fallbacks` を
  数えて割り込みの処理に戻ります（転送はまだ始まっていません）。
- 読み出し: TX のチャネルが読み出しのコマンド語（16 ビット。各語に READ、先頭は RESTART が要るときだけ RESTART、
  最後に STOP）を `IC_DATA_CMD` に書き、RX のチャネルが `IC_DATA_CMD` から 8 ビットずつ受け取ります。
  `IC_DMA_TDLR` は 8、`IC_DMA_RDLR` は 0、割り込みは STOP_DET・TX_ABRT・TX_OVER・RX_OVER・RX_UNDER・
  SCL_STUCK_LOW だけ。RX、TX の順にチャネルを始め、`IC_DMA_CR` に 3 を書きます。
- 終わり: TX の完了、RX の完了、STOP の 3 つがそろうか、エラー（I2C の異常の割り込み、DMA のバスエラー）で
  終わります。待ちは転送の期限（1 秒）を全体の期限として、残りの時間で待ちます。エラーは `-EIO`、
  期限切れは `-ETIMEDOUT`。
- 後始末: `IC_DMA_CR` を 0、割り込みマスクを 0 にし、失敗なら I2C を abort、DMA の 2 チャネルを止めます
  （割り込みを外し、CHAN_ABORT を書いて、各チャネルの BUSY と CHAN_ABORT のそのビットが落ちるまで 1 ms ごとに
  最長 100 ms 待ち、遅れて来た完了（RP2040-E13）のフラグを消す）。止まらなければコントローラーを隔離し
  （`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`、`-EIO`、チャネルは解放しない）、以後の
  転送は `-EIO` です（再起動で戻ります）。止まれば割り込みを消し、失敗なら abort の完了を最長 100 ms 待ちます
  （終わらなければ隔離して `<err> i2c_dw: I2C abort did not finish; controller quarantined`）。
- `rp2040_dma_events` の更新は割り込みを禁止して行い（Zephyr の `atomic_or()`）、4 つのカウンターは
  バスのセマフォを持った転送だけが書きます（32 ビットで一周します）。

**NuttX の DMA ドライバーの上の実装**（`port/nuttx/i2c_dw_dma_os_rp2040.c`）

- DMA は NuttX 自身のドライバー `rp2040_dmac.c`（`CONFIG_RP2040_DMAC`、有効。`arm_dma_initialize()` は
  `up_initialize()` の中、ボードの起動処理より前に動く）です。割り込みの線は DMA_IRQ_0 の 1 本ですべてのチャネルを
  扱います。ローカルのパッチ `nuttx/0011-rp2040-dmachannel-trywait` が、どこでも待たない `rp2040_dmachannel_trywait()` を
  足します。空きが無いとき、またはほかのタスクが割り当てのロックを持っているときは、取った数を返してすぐ NULL を返すので、
  チャネルは失われません（`rp2040_dmachannel()` は 12 チャネルがすべて使われていると待つ）。
- `i2c_dw_dma_os_rp2040.c` は起動時に `rp2040_dmachannel_trywait()` で 2 つのチャネルを 1 度だけ取り、読み出し用に
  持ち続けます。読み出しがチャネルを待つことはなく、1 度に 1 つの読み出しには必ずチャネルがあるので、`dma_fallbacks` は
  設定を断られたとき（チャネルがまだ動いている）しか増えません。起動時に空きが 2 つ無ければ、取れた方を返し、I2C0 は
  DMA なしで起動して、読み出しは割り込みで行います（`dmesg` に `No DMA for the reads: -16`）。
- `rp2040_dmastop()` は CHAN_ABORT を期限なしに待つので、停止は `i2c_dw_dma_os_rp2040.c` が自分で CHAN_ABORT を書き、
  コアが上限付き（100 ms）で待ちます。Zephyr 版は CHAN_ABORT の全体が落ちるまで待ちます。
- NuttX のハンドラーは失敗したチャネルのエラーのフラグをトリガーのレジスターである CTRL_TRIG に書き戻すので、
  チャネルが止まったところから再び動き出すことがあり、つなぎのコールバックがそのチャネルをすぐ abort します。
- 同じ割り込みで先に処理したチャネルの状態があとのチャネルにも残るので、ほかの利用者のバスエラーが同じ割り込みで
  処理されると、この読み出しも失敗します（`-EIO`、`dma_errors` が 1）。

**DMA を使うほかのコード**

- このビルドのほかのコードも、NuttX の DMA を通常の方法で使えます（`rp2040_dmachannel()` など。残りの 10 チャネル）。
- してはいけないこと: 読み出しの 2 つのチャネルに触れる（`rp2040_dmac.c` の表で確保済みなので、その割り当てが渡す
  ことはない）。DMA ブロックをリセットする。チャネルが共有するレジスター（INTE0、INTS0、CHAN_ABORT）を丸ごと書く
  （`rp2040_dmac.c` と同じく自分のビットだけにする）。`irq_attach()` で DMA_IRQ_0 を `rp2040_dmac.c` から取り上げる。
  自分のハンドラーなしに自分のチャネルの割り込みを DMA_IRQ_1 に回す。
- `RP2040_DMAC` で選べるようになり既定で有効になる `CONFIG_RP2040_SPI_DMA` は、`imu` の構成では無効です。NINA の SPI は
  Zephyr 版と同じくポーリングのままです（有効にすると SPI のドライバーが起動時にさらに 2 つのチャネルを取ります）。

## LSM6DSOX（`lsm6dso`）

| ファイル | 役割 |
|---|---|
| `common/drivers/lsm6dso/lsm6dso.c`、`internal/lsm6dso_core.h` | ドライバーのコア（OS を持たない）。起動とリセット、レートとレンジ、単発の読み出しとその変換、FIFO のレジスターの操作・所有・ロック・統計、タイムスタンプの読み取り、使用中の校正とバイアスの追従を `struct lsm6dso` の上で行う。Zephyr 版と同じファイル（リポジトリの `common/`。前置きヘッダーを付けてビルドする。`lsm6dso/Make.defs` がドライバーのソースを `VPATH` で参照し、オブジェクトは `nuttx/drivers` にできる） |
| `common/drivers/lsm6dso/lsm6dso_stream.c`、`internal/lsm6dso_stream.h`、`internal/lsm6dso_fifo_packet.h` | コアの FIFO のストリーム（OS を持たない。下）を `struct lsm6dso_stream` の上で行う。そのバッファーの形式 |
| `common/drivers/lsm6dso/lsm6dso_cal.c`、`include/lsm6dso_cal.h` | コアの物理量への変換と校正値の適用、静止時の角速度のバイアスの追従（下） |
| `common/drivers/lsm6dso/lsm6dso_fifo.c`、`include/lsm6dso_fifo.h` | コアの FIFO のワードの解析と、FIFO の統計 |
| `common/drivers/lsm6dso/port/lsm6dso_os.h`、`port/lsm6dso_stream_req.h` | コアの境界。OS へ（2 種類のロック・待ち・サイクルカウンター・FIFO の長い読み出しの static inline の関数と、ログ。ストリームのために専用のスレッドの上のワーク、INT1、バッチの時計）と、ストリームを使う側へ（要求、バッチごとのバッファー、その完了）。この版は Zephyr 版の実装 `port/zephyr/lsm6dso_os_impl.h` と `lsm6dso_os.c`（ストリームのワークキュー `imu_fifo_worker` と INT1 の GPIO の割り込み）、`port/zephyr/lsm6dso_stream_req_impl.h`（RTIO）を互換層の上で使う（次の表） |
| `common/drivers/lsm6dso/port/zephyr/lsm6dso_sensor.[ch]` | Zephyr へのつなぎ。コアの上のデバイスと sensor API。Zephyr 版と同じファイル（Zephyr のカーネル・デバイス・センサー・GPIO・I2C・ログの呼び出しは互換層が提供する。下）。`lsm6dso_sensor.c` は末尾で `<lsm6dso_instance.h>` を読む。インクルードパスで `port/nuttx/` が `port/zephyr/` より前にあるのでこの版のものになり、`lsm6dso_sensor_init` と `lsm6dso_driver_api` は `static` のまま |
| `common/drivers/lsm6dso/port/nuttx/lsm6dso_instance.h` | デバイス。Zephyr 版のデバイスツリーのインスタンス化のマクロ（`port/zephyr/lsm6dso_instance.h`）の代わりに、ボードの値を使う `DEVICE_DT_DEFINE()` 1 つ（INT1 の `.fifo_gpio` は `<&gpio0 24 0>`） |
| `common/drivers/stmemsc/stmemsc.h`、`stmemsc_i2c.c`、`stmemsc_mdelay.c` | Zephyr の `drivers/sensor/st/stmemsc/` の I2C の部分の写し（ST の `stmdev_ctx_t` と Zephyr の I2C の API のつなぎ。`stmemsc_mdelay()` は互換層の `k_msleep()`）。FreeRTOS 版と共有する（前置きヘッダーは付けない）。`lsm6dso_sensor.h`・`lsm6dso_os_impl.h` の `<stmemsc.h>` はインクルードパスでこのファイルになる |
| `lsm6dso/st_lsm6dso_reg.c` | ST の `lsm6dso_reg.c`（`nuttx/stmemsc/`、Git 管理外）をそのままビルドするための 1 行の包み |
| `common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio.c` | ドライバーの RTIO の submit。ストリームは `lsm6dso_rtio_stream.c` へ、単発取得は sensor API のフォールバックの写しへ。Zephyr 版と同じファイル（ログは互換層の `<zephyr/logging/log.h>`、レベル `CONFIG_SENSOR_LOG_LEVEL` は前置きヘッダー） |
| `common/drivers/lsm6dso/include/lsm6dso.h` | 拡張 API（FIFO のワードとその解析 `lsm6dso_fifo.h`、校正の型 `lsm6dso_cal.h` と呼び出し、FIFO の統計、独自属性、ストリームの停止、タイムスタンプの読み取り）。`common/compat/zephyr/drivers/sensor/lsm6dso.h` を通して `<zephyr/drivers/sensor/lsm6dso.h>` |
| `common/drivers/lsm6dso/port/zephyr/lsm6dso_rtio_stream.c` | FIFO のストリームの RTIO 側（下）。デバイスの INT1 の線、RTIO の要求をコアのストリームへ渡すこと、拡張 API の停止と設定。Zephyr 版と同じファイル |
| `common/drivers/lsm6dso/port/zephyr/lsm6dso_decoder.[ch]` | ストリームと単発取得のバッファーのデコーダー（`get_decoder`）。Zephyr 版と同じファイル |
| ボードの `src/rp2040_lsm6dso.[ch]` | I2C0 を互換層のバスのデバイス（`i2c0`、100 kHz）にし、`lsm6dsox` を初期化する（`rp2040_bringup()` から） |
| ボードの `src/rp2040_gpio_os.c` | 互換層の `gpio0` のための GPIO バンク 0（入力の設定、レベルの読み出し、エッジ・レベルの割り込み）。`<gpio_os.h>`（`common/compat`） |

ドライバーが使う OS の機能の、NuttX 版での中身（コアのものは `port/zephyr/lsm6dso_os_impl.h`・`lsm6dso_stream_req_impl.h` を通し、つなぎのものは直接）:

| Zephyr | NuttX 版 |
|---|---|
| `<zephyr/logging/log.h>`、`CONFIG_SENSOR_LOG_LEVEL`（3） | 互換層の `<zephyr/logging/log.h>`（`app_log.h`。同じ行が RAM ログに入る）、レベル 3 は前置きヘッダー |
| `k_mutex_init/_lock/_unlock` | 互換層の `k_mutex`（`rmutex_t`。`K_FOREVER` の待ちはシグナルで終わってもやり直す） |
| `k_spin_lock/_unlock` | 互換層の `k_spin_lock()`（`sched_lock()` と割り込み禁止） |
| `k_busy_wait()` | 互換層のもの。`up_udelay()` |
| `k_msleep()`、`k_usleep()` | 互換層のもの。`nxsched_msleep()`、`nxsched_usleep()`（tick に切り上げて 1 tick 足す。Zephyr と同じ規則、同じ 100 µs） |
| `k_cycle_get_32()`、`k_cyc_to_ns_floor64()` | 互換層のもの。TIMER の下位 32 ビット（1 MHz）と ×1000 |
| `i2c_burst_read_dt()`（コアの FIFO の読み出し） | 互換層の I2C0 の呼び出し |
| `k_work_queue_start()` のキューの上の `k_work_delayable`、`k_sys_work_q` の上の `k_work`（ストリームのワークと INT1 のワーク） | 互換層のワークキュー（`imu_fifo_worker` は優先度 108、`sysworkq`） |
| `gpio_*_dt()`、`sensor_clock_get_cycles()`、RTIO（ストリーム） | 互換層のもの。ボードの `gpio0` と TIMER の上（下） |
| `CONFIG_SENSOR_ASYNC_API` | `CONFIG_LSM6DSO_STREAM` があれば前置きヘッダーが定義する（`LSM6DSO_LOCAL` と一緒に常に有効。Zephyr 版と同じく、無いと `lsm6dso_sensor.c` はビルドできない） |
| `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` | 互換層の `<zephyr/devicetree.h>`。`st_lsm6dso` の `i2c` だけ 1（ボードの LSM6DSOX は I2C0 の 1 個） |
| デバイスツリーのインスタンス化、`POST_KERNEL`・優先度 90 | `DEVICE_DT_DEFINE(lsm6dsox, …)`（`lsm6dso_instance.h`）、ボードの起動処理での `device_init()` |

- 初期化はボードの `rp2040_bringup()` が、I2C0 の初期化の直後、KV（NVS）のマウントより前に行います
  （Zephyr 版の初期化の順と同じ）。失敗してもデバイスが「準備できていない」だけで起動は続きます
  （`device_is_ready()` が false。Zephyr の `do_device_init()` と同じ）。
- 移していないもの: SPI 接続（Zephyr 版のデバイスツリーのインスタンスには有り得るが、FIFO は受け付けない）。
  ドライバーにはセンサーハブ、トリガー、内部温度がありません（上流のオプション `CONFIG_LSM6DSO_SENSORHUB`・
  `_TRIGGER`・`_ENABLE_TEMP`）。
- `fifo_read` の転送時間（統計の `transfer_time_ns`）は TIMER（1 µs 単位）で測ります。Zephyr 版は CPU の
  クロック（SysTick、8 ns 単位）です。

### 物理量への変換と校正

変換（`lsm6dso_cal.c`）、校正値の参照（`lsm6dso_cal_set_lookup()`・`lsm6dso_cal_reload()`・
`lsm6dso_cal_status()`）、その場でのバイアスの取り直し（`lsm6dso_cal_zero()`）、ストリーム中の追従
（`lsm6dso.c` が動かす `lsm6dso_cal.c` の `lsm6dso_zero_*()`）は Zephyr 版のコードのままで、動作も同じです（Zephyr 版のドライバー説明の
「物理量への変換と校正」）。ドライバーは KV を読みません。校正値を引く関数は `common/imu/imu_cal.c`
（Zephyr 版と同じファイル。`nuttx/apps/imu/` がビルドする）が init の `imu_cal_init()` で登録し、ドライバーは登録時、
`imu cal reload`、レンジの設定時にそれを呼びます。校正値が掛かるのは単発取得の `sensor_channel_get()` と
FIFO のデコーダーの出力で、取得サービスが MQTT で送る RAW には掛かりません。

### FIFO のストリーム

コアのストリーム `lsm6dso_stream.c` と、つなぎの `lsm6dso_rtio_stream.c`・`lsm6dso_os.c`（INT1 を含む）は Zephyr 版と同じファイル
（`common/drivers/lsm6dso/`。コアの FIFO の呼び出しを使う）で、Zephyr の名前のカーネル・RTIO・GPIO・sensor の API を互換層（下の節）から
使います。動作は Zephyr 版と同じです。

- `sensor_stream()`（`SENSOR_DT_STREAM_IODEV`、トリガーは `SENSOR_TRIG_FIFO_WATERMARK` と
  `SENSOR_STREAM_DATA_INCLUDE` の 1 つだけ。ほかは `-ENOTSUP`、2 本目は `-EBUSY`）が multishot の読み出しを
  1 つ出し、`lsm6dso_sensor_submit()` がそれをコアのストリームに渡し、ストリームが呼び出し元の文脈でワーカー
  （`imu_fifo_worker`）を起こします。
- ワーカーは最初の実行で FIFO を開始し（ウォーターマーク、加速度・ジャイロの BDR、タイムスタンプ 8 回に 1 つ、
  ストリームモード）、INT1 の立ち上がりエッジの割り込みを有効にしてから FIFO のしきい値を INT1 に出します。
  有効にした時点で INT1 がすでに高ければ、その場で 1 回処理を予約します。
- INT1 のエッジ → GPIO の割り込みが `k_work_submit()` でシステムワークキュー（`sysworkq`）に処理を入れる →
  そこでコアの `lsm6dso_stream_int1()` がワーカーを `k_work_reschedule_for_queue(.., K_NO_WAIT)` で起こす → ワーカーが
  FIFO の状態を読み、ウォーターマーク以上なら全ワードを 1 回の転送で読んで 1 バッチを完了にします。
  完了した multishot はその場でワーカーに戻り（`lsm6dso_sensor_submit()`）、ワーカーはすぐもう一度状態を見ます。
  何も無ければ 20 ms 後に見直す予約（`k_work_schedule_for_queue(.., K_MSEC(20))`）をします。エッジを
  取りこぼしてもこの見直しで読みます。
- バッファーはコンテキストのメモリープールから取ります（最小 = パケットの頭＋ウォーターマーク分、
  最大 = 頭＋512 ワード分。取れなければ `-ENOMEM` でストリームを終える）。オーバーランは `-EOVERFLOW`、
  I2C の失敗はそのエラーで終えます。終えるときは FIFO をバイパスに戻し、INT1 への出力と割り込みを止めます。
- 穏やかな停止（`lsm6dso_fifo_request_stop()`）: ワーカーがバッチ化を止め（BDR を 0）、残りのワードを
  最後の端数のバッチとして渡してから、FIFO をバイパスに戻して `-ECANCELED` で終えます。
  キャンセル（`rtio_sqe_cancel()`）は次にワーカーが動いたとき（INT1 か 20 ms の見直し）に気づき、
  バッチ化を止めずにそのまま停止して `-ECANCELED`（停止の失敗ならそのエラー）で終えます。FIFO に残った
  ワードは捨てます。
- ストリームを動かしても ODR は変えません（`imu start` が設定し、停止後もそのまま。Zephyr 版と同じ）。

## RP2040 の除算器と `memcpy`・`memset`（パッチ 0009・0010）

どちらも NuttX 本体へのパッチ（`nuttx/patches/nuttx/`）の Kconfig を、ボードの Kconfig のオプションが select します。
パッチの記号が無効のときは、パッチの入ったコードも上流のものと同じです。

- **除算器**（ボードの `CONFIG_RP2040_PICO_DIVIDER`、既定 y）: コンパイラーが出す整数の割り算（`__aeabi_idiv`・`__aeabi_idivmod`・
  `__aeabi_uidiv`・`__aeabi_uidivmod`・`__aeabi_ldivmod`・`__aeabi_uldivmod`）を、Pico SDK の `divider_hardware.S`（`nuttx/pico-sdk` から、
  ボードの `src/pico_sdk` の最小のヘッダーでビルド）へ `ld --wrap`（`scripts/Make.defs`）で回します。ルーチンは割り込みを止めず
  （`PICO_DIVIDER_DISABLE_INTERRUPTS` 0）、割り込みハンドラーが割り算の途中に入った場合は DIRTY を見て自分で退避します。
  タスクが割り算の途中で切り替わる場合は、パッチ 0009 の `RP2040_HW_DIVIDER` が退避します: ARMv6-M の切り替えはすべて
  `arm_doirq()` を通るので、タスクが変わるときに古いタスクの除算器を（DIRTY のときだけ）`struct xcptcontext` の 20 バイトに置き、
  新しいタスクのものを戻します（割り算の途中でなかったタスクは DIRTY を落として戻る）。`arm_sigdeliver()` の最後の切り替えでも
  退避します。FreeRTOS 版の Pico SDK のポートは、切り替えのたびに 4 語を退避します。0 での割り算は Pico SDK のルーチンの結果
  （32 ビットは飽和した商と余り 0、64 ビットは libgcc と同じ。3 版で同じ）です。ルーチンは SIO とスタックだけを使うので、
  `__start()` の最初から、割り込みハンドラーと異常停止の処理でも使えます。`build.sh` は `__wrap_*` の 6 つがあり、libgcc の
  割り算（`__divsi3` など）がイメージに無いことを確かめます。
- **`memcpy`・`memset`**（`CONFIG_RP2040_ROM_MEM_OPS`、defconfig で有効）: パッチ 0010 の `RP2040_ROM_MEM_OPS` が
  `LIBC_ARCH_MEMCPY`・`LIBC_ARCH_MEMSET` を select し、チップのディレクトリーの `rp2040_rom_mem_*` が `memcpy`・`memset`・
  `__aeabi_memcpy*`・`__aeabi_memset*` をブート ROM のルーチンへ飛ばします（Pico SDK の `pico_mem_ops` と同じ使い分け）。
  ルーチンは初回の呼び出しで ROM の関数表から引いて `.bss` に置くので、`.bss` をゼロにする前には呼べません。GCC は
  `rp2040_start.c` の `.bss` のゼロ化と `.data` のコピーのループを `memset`・`memcpy` の呼び出しに変えるので、パッチがループに
  `START_LOOP_BARRIER()`（メモリーを clobber する空の asm）を入れ、`build.sh` が `__start()` にそれらの呼び出しが無いことを
  確かめます。Zephyr 版は同じルーチンを Zephyr の SoC へのパッチで持ちます。
- 実機（2026-11-18、試験のイメージ）: 除算器と退避ありのイメージで 250 秒・約 1,128 万回の割り算の食い違いは 0、退避なしのイメージでは
  40 秒で 10 回でした。

## シグナルハンドラーを使わない

NuttX 版のコードは、シグナルハンドラーを登録しません（`sigaction()`・`signal()`、SIGEV_SIGNAL のタイマー、`kill()`・
`pthread_kill()` でハンドラーを走らせる使い方をしない）。シグナルハンドラーの実行中にタスクの切り替え（1 ms ごとの起床と
優先度の入れ替え）が重なる負荷を掛けると、数秒以内にログを残さずに固まることを実機で確かめました（2026-11-18。除算器の
変更の有無に依らない）。原因は特定しておらず、修正も入れていません。今のコードにはハンドラーの登録が無く、通常の動作では
この経路を通りません。待ちはセマフォ・sleep・ブロッキングの入出力で書きます。

## Zephyr の API の互換層（`compat`）

互換層は、写したドライバーとそれを呼ぶ側（取得サービス、校正、センサーのコマンド）が使う
Zephyr の API を、Zephyr の名前のまま最小限だけ用意したものです。Zephyr の sensor・RTIO・カーネルの全体は
移していません。互換層は 2 か所に分かれています。

- **OS に依存しない部分**（リポジトリの `common/compat/`）: Zephyr の API のヘッダー（`common/compat/zephyr/...`）、
  ワーク、RTIO、ブロックのプール、デバイスの一覧、`gpio0`、`net_mgmt` のイベント、Wi-Fi の要求、sensor の既定の
  iodev と、互換層の上の版が使う Zephyr のライブラリーの写し（`sensor_shell_stream.c`・`sensor_shell.h`、
  Wi-Fi の資格情報の `wifi_credentials.c`・`wifi_credentials_backend_settings.c`・`wifi_credentials_internal.h`、
  KV の NVS と settings（下の「KV: Zephyr の NVS と settings の写し」）。MQTT のクライアントは
  `common/imu/mqtt_min.c`）、境界のヘッダー（スレッドの優先度とスタックの表 `priorities.h`、`cpu_load_get()` の
  `cpu_load.h`（とその計算）、`gpio0` の下の `gpio_os.h`、KV の領域の flash の `flash_kv_os.h`）。
  シェルの `zephyr/shell/shell.h` のコマンドの処理は `common/shell/zshell.c`（下の「シェル」）。Zephyr 版は
  使いません（Zephyr 本体を使う）。
- **NuttX の実装**（`nuttx/drivers/compat/`）: カーネルの部分（`zephyr_kernel.c`・`zephyr_mutex.c`）、OS ごとの
  ヘッダー（`zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h`・`zephyr_os_shell.h`）、全体が NuttX に依存するヘッダー（`zephyr/net/socket.h`・
  `net_if.h`、`zephyr/sys/clock.h`・`__assert.h`、`zephyr/drivers/i2c.h`）と、I2C・
  インターフェース・チップ ID の `.c`、前置きヘッダー（`common_prelude.h`）。

インクルードパスは NuttX の実装（`$(TOPDIR)/external/compat`）、共通の部分（`common/compat`）の順です。共通の
`.c` はカーネル側（`compat/Make.defs`）とアプリ（`apps/sensor`・`apps/wifi`）が `VPATH` で参照し、前置き
ヘッダーを付けてコンパイルします。ホストテストは `common/tests/zephyr_fakes` に偽のカーネル（OS ごとの
ヘッダーとその関数）を置き、NuttX の実装より先にインクルードパスに入れます。

| ヘッダー（場所の無いものは `common/compat/`） | 中身 |
|---|---|
| `zephyr/device.h` | `struct device`（名前、config、API、状態、data、初期化関数）、`device_is_ready()`、`device_init()`（Zephyr の `do_device_init()` と同じ意味）、`DEVICE_DT_GET()`、`DEVICE_DT_DEFINE()`、`DEVICE_API()`、`device_get_binding()`（ノード名）、`device_get_by_dt_nodelabel()`（ノードのラベル）、`DEVICE_API_IS(sensor, dev)`（下） |
| `zephyr/devicetree.h` | `DT_NODELABEL()`、`DT_BUS()`。ノードは `gpio0`、`i2c0`、`lsm6dsox` だけ。`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()`（Zephyr と同じ定義）は `st_lsm6dso` の `i2c` で 1、ほかの互換文字列・バスでは 0（`lsm6dso.h`）。`DT_HAS_CHOSEN()` はいつも 0（Zephyr 版も `zephyr,settings-partition` を選んでいない） |
| `zephyr/kernel.h` | `k_timeout_t`（`K_NO_WAIT`、`K_FOREVER`、`K_MSEC()`、`K_SECONDS()`。単位は 100 µs の tick）、`k_mutex`、`k_sem`（`K_SEM_DEFINE`）、セマフォに対する `k_poll()`（`K_POLL_TYPE_SEM_AVAILABLE`・`K_POLL_MODE_NOTIFY_ONLY`）、ワーク（`k_work`、`k_work_delayable`、`k_work_q`、`k_work_submit()`、`k_work_cancel_sync()`、`k_work_schedule_for_queue()`、`k_work_reschedule_for_queue()`、`k_work_queue_start()`）、`K_THREAD_STACK_DEFINE()`、`K_PRIO_PREEMPT()`、`k_thread_name_set()`、システムワークキュー `k_sys_work_q`、`K_MUTEX_DEFINE()`、`K_THREAD_DEFINE()`（下）、`k_current_get()`（呼び出したスレッドの NuttX のスレッド ID。`K_THREAD_DEFINE()` の `k_tid_t` と比べられる）、`k_uptime_get()`、`k_sleep()`・`k_msleep()`・`k_usleep()`・`k_busy_wait()`（下）、`k_cycle_get_32()`・`k_cycle_get_64()`・`k_cyc_to_us_floor64()`・`k_cyc_to_us_ceil64()`・`k_cyc_to_ns_floor64()`・`sys_clock_hw_cycles_per_sec()`（下） |
| `zephyr/spinlock.h`、`zephyr/sys/atomic.h` | `k_spin_lock()`（下）、`atomic_t` と `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`、`zephyr/rtio/work.h` | RTIO（下）と RTIO のワークキュー（`rtio_work_req_alloc()`・`_submit()`） |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`、`gpio_is_ready_dt()`、`gpio_pin_configure_dt()`（入力だけ）、`gpio_pin_interrupt_configure_dt()`、`gpio_pin_get_dt()`、コールバック（`gpio_init_callback()`・`gpio_add_callback()`・`gpio_remove_callback()`） |
| `zephyr/drivers/sensor.h` | `struct sensor_value`、チャネル・属性・トリガーの列挙（Zephyr と同じ番号）、`struct sensor_driver_api`、`sensor_attr_set()`・`_get()`、`sensor_sample_fetch()`・`_chan()`、`sensor_channel_get()`、`sensor_trigger_set()`、単位の換算。非同期の API: `struct sensor_chan_spec`、デコーダー（`struct sensor_decoder_api`、`sensor_three_axis_data` など）、`SENSOR_DT_READ_IODEV()`、`SENSOR_DT_STREAM_IODEV()`、`sensor_get_decoder()`、`sensor_stream()`、`sensor_read()`、`sensor_read_async_mempool()`、`sensor_processing_with_callback()`。復号したデータの表示の書式 `PRIsensor_*_data`（Zephyr の `sensor_data_types.h`。Q 値の書式は `dsp/print_format.h`） |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`、`sensor_clock_cycles_to_ns()`（下） |
| `zephyr/drivers/i2c.h`（NuttX） | `struct i2c_dt_spec`、`i2c_burst_read_dt()`、`i2c_write_dt()`、`i2c_write_read_dt()`、`i2c_configure()`、速さの区分（`I2C_SPEED_*`、`I2C_SPEED_SET()`） |
| `zephyr/sys/util.h`、`sys/mpsc_lockfree.h`、`sys/mem_blocks.h`、`dsp/types.h` | `BIT()`、`ARRAY_SIZE()`、`MIN()`・`MAX()`、`IS_ENABLED()`、`CONTAINER_OF()`、`GENMASK()`・`GENMASK64()`・`FIELD_GET()`、`STRINGIFY()`、`KB()`、`ZTESTABLE_STATIC`、`hex2bin()`・`bin2hex()`（実装は `lib/utils/hex.c` の写し）など（Zephyr と同じく `<string.h>` も読む）、RTIO の待ち行列とブロックのプール、`q31_t` |
| `zephyr/sys/__assert.h`（NuttX） | `__ASSERT()`・`__ASSERT_NO_MSG()`（NuttX の `DEBUGASSERT()`。ホストテストは `assert()`） |
| `zephyr/dsp/print_format.h` | Zephyr の同名のヘッダーの写し（先頭の説明と `<zephyr/sys/util.h>` のインクルードだけを足した）。Q 値の表示の書式 `PRIq()`・`PRIq_arg()` |
| `zephyr/types.h` | `<stddef.h>` と `<stdint.h>`（`lsm6dso_sensor.h` が読む目的のもの） |
| `zephyr/init.h` | 中身の無いヘッダー（`lsm6dso_sensor.c` が読む。`SYS_INIT()` と初期化レベルを使うのは Zephyr 版のインスタンス化のマクロだけ） |
| `zephyr/sys/iterable_sections.h` | `STRUCT_SECTION_FOREACH()` だけ（いつも空の区間。Zephyr のリンカーの区間は無い）。sensor シェルが読み（使うのは Zephyr 版の構成に無い `CONFIG_SENSOR_INFO` のときだけ）、settings が静的なハンドラーの区間を回す（この移植には無い） |
| `zephyr/sys/printk.h`、`zephyr/sys/byteorder.h`、`zephyr/sys/cpu_load.h` | `snprintk()`（Zephyr 版の構成 `CONFIG_PICOLIBC=y` と同じく `snprintf()`）、`sys_put_be16/32()`・`sys_get_be16/32()`、`cpu_load_get()`（`cpu_load.h`。実装はボード） |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()`（下） |
| `zephyr/shell/shell.h` | `shell_print()`・`shell_info()`・`shell_warn()`・`shell_error()`・`shell_fprintf()`、`struct shell`（シェルのバックエンド。中身は OS の部分 `zephyr_os_shell.h`）、コマンドの表のマクロ（`SHELL_CMD_REGISTER()`、`SHELL_STATIC_SUBCMD_SET_CREATE()`、`SHELL_DYNAMIC_CMD_CREATE()`、`SHELL_CMD_ARG()`、`SHELL_CMD()`、`SHELL_COND_CMD()`、`SHELL_EXPR_CMD_ARG()`、`SHELL_HELP()`、`SHELL_SUBCMD_SET_END`）、`shell_device_get_binding()`・`shell_device_filter()`（下） |
| `zephyr/net/socket.h`（NuttX） | `zsock_inet_pton()`・`zsock_inet_ntop()`（C ライブラリの `inet_pton()`・`inet_ntop()`）、`zsock_getaddrinfo()`・`zsock_freeaddrinfo()`（NINA の名前解決、`nina_dns_getaddrinfo()`。エラーは NuttX の `EAI_*`）、`zsock_socket()`・`zsock_sendto()`・`zsock_close()`・`zsock_recv()`・`zsock_connect()`・`zsock_send()`・`zsock_sendmsg()`・`zsock_poll()`（NuttX の `socket()`・`sendto()`・`close()`・`recv()`・`connect()`・`send()`・`sendmsg()`・`poll()`）、`ZSOCK_MSG_DONTWAIT`・`struct zsock_pollfd`・`ZSOCK_POLLIN` など（`MSG_DONTWAIT`・`struct pollfd`・`POLLIN`。NINA のソケットでの動きは下） |
| `zephyr/sys/clock.h`（NuttX） | `sys_clock_gettime()`・`sys_clock_settime()`（`SYS_CLOCK_REALTIME`。NuttX の `clock_gettime()`・`clock_settime()` の `CLOCK_REALTIME`。0 か `-EINVAL`）、`NSEC_PER_USEC`・`USEC_PER_SEC` |
| `zephyr/net/net_if.h`（NuttX） | `struct net_if`（NuttX のネットワークデバイス 1 つ）、`net_if_get_default()`・`net_if_get_wifi_sta()`、アプリが読む IPv4 のアドレスとゲートウェイ（`net_if_ipv4_get_global_addr()`・`net_if_ipv4_get_gw()`）、`net_if_is_admin_up()`（下）。デバイスの値は NINA のつなぎが自分で書く |
| `zephyr/net/net_mgmt.h` | `net_mgmt()`（要求）、`struct net_mgmt_event_callback`、`net_mgmt_init_event_callback()`・`net_mgmt_add_event_callback()`・`_del_event_callback()`、`net_mgmt_event_notify_with_info()`（下） |
| `zephyr/net/wifi_mgmt.h` | Wi-Fi の型（セキュリティ、帯域、MFP、状態、モード。Zephyr と同じ値）、`struct wifi_scan_params`・`wifi_scan_result`・`wifi_connect_req_params`・`wifi_iface_status`・`wifi_status`、`struct wifi_mgmt_ops`、オフロードのインターフェースの API と `NET_DEVICE_DT_OFFLOAD_DEFINE()`、要求 `NET_REQUEST_WIFI_SCAN`・`_CONNECT`・`_DISCONNECT`・`_IFACE_STATUS`・`_CONNECT_STORED`（`CONFIG_WIFI_CREDENTIALS` のとき）、イベント `NET_EVENT_WIFI_SCAN_RESULT`・`_SCAN_DONE`・`_CONNECT_RESULT`・`_DISCONNECT_RESULT`、`wifi_mgmt_raise_connect_result_event()`・`_disconnect_result_event()`（下）、`wifi_security_txt()`・`wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | Zephyr の同名のヘッダーの写し: `struct wifi_credentials_header`・`_personal`、`WIFI_CREDENTIALS_FLAG_*`、`wifi_credentials_set_personal_struct()`・`_get_by_ssid_personal_struct()`・`_delete_by_ssid()`・`_for_each_ssid()`・`_is_empty()` など。設定の値（`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` など。Zephyr の名前）は OS の構成にあればそれ（NuttX 版は `nuttx/apps/wifi/Kconfig`）、無ければ Zephyr 版の値。Zephyr に無い `wifi_credentials_init()`（init が呼ぶ。Zephyr の `SYS_INIT()`）。関数は `common/compat/wifi_credentials.c`（Zephyr のライブラリーの写し）と、設定に保存する `wifi_credentials_backend_settings.c` |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()`（`_2g`・`_5g`・`_6g`）、`net_bytes_from_str()`（Zephyr では `<zephyr/net/net_ip.h>`） |
| `zephyr/settings/settings.h`、`zephyr/kvss/nvs.h`、`zephyr/sys/slist.h`、`zephyr/sys/list_gen.h` | Zephyr の同名のヘッダーの写し（無改変。下の「KV」）。settings の API（`settings_subsys_init()`、`settings_save_one()`、`settings_load_one()`、`settings_delete()`、`settings_load_subtree_direct()` など）、NVS の API と `struct nvs_fs`、単方向リスト |
| `zephyr/drivers/flash.h`、`zephyr/storage/flash_map.h` | NVS と settings が使う flash の API（`flash_read()`・`flash_write()`・`flash_erase()`・`flash_flatten()`・`flash_get_parameters()`・`flash_get_write_block_size()`・`flash_get_page_info_by_offs()`、`struct flash_parameters`）と flash map（`flash_area_open()`・`flash_area_close()`・`flash_area_get_sectors()`、`PARTITION_ID(storage_partition)`）。デバイスは KV の領域の flash だけ（`zephyr_flash_kv.c`。下） |
| `zephyr/sys/crc.h`、`zephyr/toolchain.h` | `crc8_ccitt()` などの宣言（実装は `subsys/crc/crc8_sw.c` の写し）、`__packed`・`__weak` |
| `flash_kv.h`、`flash_kv_os.h` | KV の領域の定数（`0xffe000` から 8 KiB、256 バイトのページ、4 KiB のセクター）、OS の境界（`flash_kv_os.h`。下の「KV」）、範囲の検査・ページへの分割・領域の中身の判定（Zephyr の NVS、以前の NuttX 版の形式、消去済み、それ以外） |
| `zephyr_kv_config.h` | 写した NVS と settings の前に読む Zephyr 版の Kconfig の値（`CONFIG_SETTINGS_NVS_SECTOR_COUNT` など。下の「KV」） |
| `kv_store_os.h` | `common/imu/kv_store.c` の OS の部分: `kv wipe` のサブコマンド（Zephyr 版の `zephyr/app/usb-console/src/kv_store_os.h` は空） |
| `zephyr/logging/log.h` | `LOG_MODULE_REGISTER()`、`LOG_ERR()` などのログのマクロ。ポートの `app_log.h`（`common/compat/log/`。`dmesg` の RAM ログ）を読むだけ |
| `zephyr/net/net_ip.h` | IPv4 のアドレスの型 `struct in_addr`（`imu_raw_mqtt.h`）。NuttX の `<netinet/in.h>` を読むだけ（同じメンバー `s_addr`、ネットワークバイト順） |
| `zephyr_compat.h` | `zephyr_compat_initialize()`（ボードの起動処理が呼ぶ。下）、`K_THREAD_DEFINE_START()`（アプリのスレッドの表、`common/imu/app_init.c` が呼ぶ） |
| `sensor_shell.h` | Zephyr の `drivers/sensor/sensor_shell.h` の写し（sensor シェルの共通の型。Zephyr 版は Zephyr のものを読む） |
| `zephyr_os.h`、`zephyr_os_spinlock.h`、`zephyr_os_atomic.h`、`zephyr_os_shell.h`（NuttX） | OS ごとのヘッダー（下）。NuttX の型（`rmutex_t`・`sem_t`・wdog・`pid_t`）、tick、優先度、TIMER、`k_yield()`（`sched_yield()`）、アサーション（`DEBUGASSERT()`）、ワークキューの待ち（`nxsem_wait_uninterruptible()`）、`sched_lock()`・`up_irq_save()`、`<nuttx/atomic.h>` の名前の引き取り、`struct shell`（端末のパスとコマンドのタスク） |
| `common_prelude.h`（NuttX） | 前置きヘッダー（下） |

| ソース（場所の無いものは `common/compat/`） | 中身 |
|---|---|
| `zephyr_mutex.c`（NuttX） | NuttX の上の `k_mutex`（`rmutex_t`）。Zephyr の `k_mutex` は全部 0 のとき未ロック（`lock_count` 0）で、Zephyr の MQTT ライブラリは `mqtt_client_init()` の前のクライアントのミューテックスを `mqtt_abort()` で lock する。NuttX の全部 0 のミューテックスはスレッド 0 が持っている扱いで、lock が戻らない。そこで `K_MUTEX_DEFINE()` と `k_mutex_init()` が印（`ready`）を立て、印のないもの（全部 0）は最初の lock・unlock で初期化する（互換層のスピンロックの中で 1 回だけ） |
| `zephyr_kernel.c`（NuttX） | NuttX の上の OS 部分: `k_sem`（`sem_t`、上限は `k_sem_give()` で見る）、`k_poll()`、遅延ワークのタイマー（wdog）、スレッド名、スレッドの起動（`z_thread_static_start()`。静的なスタックのカーネルスレッド。起動のデータを新しいスレッドに渡し、受け取るまで呼んだ側が待つ）、`k_sleep()`・`k_msleep()`・`k_usleep()`・`k_busy_wait()` |
| `zephyr_work.c` | ワークとワークキュー。Zephyr の `kernel/work.c` の使っている部分の写し（OS に依存しない） |
| `zephyr_init.c` | ワークキューのスレッド（`k_work_queue_start()`、`kernel/work.c` の `work_queue_main()` のループ）、システムワークキュー `k_sys_work_q`、`zephyr_compat_initialize()`、`k_uptime_get()`（TIMER の ms）。`z_thread_static_start()` の上（OS に依存しない） |
| `zephyr_rtio.c`、`zephyr_rtio_workq.c`、`zephyr_mem_blocks.c` | RTIO の executor（`subsys/rtio/rtio_executor.c`）、RTIO のワークキュー（`rtio_workq.c`。スレッド `RTIO WQ 0` は `zephyr_compat_initialize()` が `z_thread_static_start()` で起動する）、ブロックのプール（`lib/mem_blocks`） |
| `zephyr_gpio.c` | `gpio0` のデバイス（Zephyr の `gpio.h` の処理と `gpio_rpi_pico.c`）。ボードの `<gpio_os.h>` の上 |
| `zephyr_sensor_clock.c` | センサーの時計。互換層のサイクルカウンター `k_cycle_get_64()`（ボードの TIMER、`rp2040_timer_us()`、1 MHz） |
| `sensor_default_rtio.c` | Zephyr の `drivers/sensor/default_rtio_sensor.c` の写し（ログだけ置き換え）。センサーの iodev の API、単発取得のフォールバック、既定のデコーダー |
| `zephyr_device.c` | デバイスの初期化と一覧（`gpio0`、`i2c0`、`lsm6dsox` を Zephyr 版の初期化の順に、ノードのラベルと sensor の API かどうかと共に。`gpio0` は OS の構成の `CONFIG_LSM6DSO_STREAM` のとき） |
| `zephyr_i2c.c`（NuttX） | I2C（NuttX の I2C マスターの上） |
| `zephyr_hwinfo.c`（NuttX） | `hwinfo_get_device_id()`。ボードのチップ ID（`rp2040_chip_id()`） |
| `zephyr_net_if.c`（NuttX）、`zephyr_net_mgmt.c`、`zephyr_wifi_mgmt.c` | インターフェース（NuttX のネットワークデバイスの値）、`net_mgmt` のイベントの待ち行列とスレッド（スレッドは `z_thread_static_start()` で起動。優先度とスタックの大きさは `priorities.h` の `APP_PRIO_NET_MGMT`・`APP_STACK_NET_MGMT`）、Wi-Fi の要求の処理（Zephyr の `subsys/net/l2/wifi/wifi_mgmt.c` の検査の写し）とイベント、`NET_REQUEST_WIFI_CONNECT_STORED`（同じファイルの `connect_stored_command()` の写し。`CONFIG_WIFI_CREDENTIALS` のとき）。`CONFIG_NINA_NET` のときだけ。NINA のつなぎ（`common/drivers/nina/port/nuttx/nina_wifi_mgmt.c`）がコアの上でアプリに提供する |
| `zephyr_wifi_utils.c` | `wifi_security_txt()`・`wifi_band_txt()`（Zephyr の `wifi_mgmt.c`）、チャネルの検査（`wifi_utils.c`）、`net_bytes_from_str()`（`subsys/net/ip/utils.c`）の写し。`wifi cred` が使う。OS に依存しない。`CONFIG_NINA_NET` のときだけ |
| `sensor_shell_stream.c` | Zephyr の `drivers/sensor/sensor_shell_stream.c` の写し（sensor シェルの処理スレッド）。OS の `sensor_shell_os.h` を最初に読み、処理スレッドのコールバックを `sensor_shell_os_processing_callback()` にした（NuttX 版は `nuttx/apps/sensor/sensor_shell_nuttx.c`）。`apps/sensor` がビルドする |
| `wifi_credentials.c`、`wifi_credentials_backend_settings.c` | Zephyr の `subsys/net/lib/wifi_credentials/` の写し（保存した Wi-Fi の資格情報）。`apps/wifi` がビルドする。バックエンドの変更は 2 つ: 読み込みのバッファーを 8 バイトに揃えた（構造体として読むため。Cortex-M0+ は揃っていない読み出しで止まる）、番号の書式を `%zu` にした |
| `subsys/kvss/nvs/`、`subsys/settings/`、`subsys/crc/crc8_sw.c`、`lib/utils/hex.c` | Zephyr v4.5.0 の NVS、settings、CRC-8、16 進の変換の写し（無改変。下の「KV」）。`apps/imu` がビルドする |
| `zephyr_flash_kv.c` | KV の領域の flash デバイスと flash map（下の「KV」）。`apps/imu` がビルドする |
| `kv_store_wipe.c` | `kv wipe confirm`（領域を消し、settings を初期化し直す）。`apps/imu` がビルドする |

- **ワーク**: フラグ（RUNNING・CANCELING・QUEUED・DELAYED）と戻り値は `kernel/work.c` と同じです。
  `submit` はキュー済みなら何もしない（0）、実行中なら同じキューに入れ直す（2）。`schedule` は遅延中・
  キュー済みなら何もしない（実行中なら予約できる）、`reschedule` は遅延を止めてから予約し直す
  （`K_NO_WAIT` ならその場でキューに入れる）。`k_work_cancel_sync()` はキューから外し、実行中なら終わるまで
  待ちます（待てるのは 1 スレッドだけ。Zephyr は複数）。キューのスレッドは処理の後、ほかに待ちがあれば
  `sched_yield()` します（Zephyr の `k_yield()`）。遅延のタイマーは wdog で、時間は Zephyr と同じく
  「今の tick + tick 数 + 1」で切れます。移していないもの: flush、drain、plug、キューの停止、ワークの
  タイムアウト、`k_work_cancel()`、`k_work_cancel_delayable()`。
- **OS ごとのヘッダー**（互換層の）: 共通のヘッダーは OS の型を持たず、OS ごとのディレクトリー（NuttX 版は
  `nuttx/drivers/compat/`、ホストテストは `common/tests/zephyr_fakes/`）の次のヘッダーを読みます。満たすべき
  ものの一覧は、それを読む共通のヘッダーの先頭のコメントにあります。
  - `zephyr_os.h`（`<zephyr/kernel.h>` が最初に読む）: 互換層より先に要る OS のヘッダー、`Z_USEC_PER_TICK`、
    `struct k_mutex` と `Z_MUTEX_INITIALIZER()`（全部 0 のものが未ロックとして働くこと）、`struct k_sem`・
    `Z_SEM_INITIALIZER()`・`K_SEM_MAX_LIMIT`、`struct _timeout`・`_timeout_func_t`、`k_tid_t`・`k_current_get()`、
    `k_yield()`、`K_PRIO_PREEMPT()`（Zephyr の優先度を順を保って OS の優先度に）、`k_cycle_get_32()`・
    `k_cycle_get_64()`（1 MHz）、`Z_OS_ASSERT()`、任意で `Z_SEM_TAKE_FOREVER()`（ワークキューの待ち）。関数
    （`k_sem_*()`、`k_mutex_*()`、`k_poll()`、`k_sleep()` など、`z_add_timeout()`・`z_abort_timeout()`、
    互換層のすべてのスレッドを起こす `z_thread_static_start()`）は OS の `.c`（NuttX 版は `zephyr_kernel.c`・
    `zephyr_mutex.c`）。ワークキューのスレッド、`k_sys_work_q`、`k_uptime_get()`、`zephyr_compat_initialize()`
    は共有（`zephyr_init.c`）。
  - `zephyr_os_spinlock.h`（`<zephyr/spinlock.h>` が読む）: `struct k_spinlock`、`k_spinlock_key_t`、
    `k_spin_lock()`・`k_spin_unlock()`（下）。
  - `zephyr_os_atomic.h`（`<zephyr/sys/atomic.h>` が名前を定義する前に読む）: 互換層が引き取る名前を持つ OS の
    ヘッダーと、そのマクロの `#undef`（下）。ホストテストでは空。
  - `zephyr_os_shell.h`（`<zephyr/shell/shell.h>` が読む）: `struct shell`（下の「シェル」）。
  - 全体が OS ごとのヘッダー: `zephyr/net/socket.h`、`zephyr/net/net_if.h`、`zephyr/sys/clock.h`、
    `zephyr/sys/__assert.h`、`zephyr/drivers/i2c.h`。
  - 共通の `.c` とヘッダーが読む境界のヘッダー: `priorities.h`（`APP_STACK_NET_MGMT`・`APP_PRIO_RTIO_WQ`・`APP_STACK_RTIO_WQ`。`common/compat/` にあり、
    OS の部分 `priorities_os.h` の `APP_PRIO_FROM_ZEPHYR()` を読む。NuttX 版は `nuttx/drivers/include/`）、
    `gpio_os.h`（`zephyr_gpio.c`。`common/compat/` にあり、ボードが実装する）、`app_log.h`（`<zephyr/logging/log.h>`、
    `sensor_default_rtio.c`、資格情報。`common/compat/log/` にあり、OS の部分 `app_log_os.h` を読む）、`<cpu_load.h>`
    （`<zephyr/sys/cpu_load.h>`。`common/compat/`）、`sensor_shell_os.h`
    （`sensor_shell_stream.c`）。構成の値 `CONFIG_LSM6DSO_STREAM`（`zephyr_device.c`）と
    `CONFIG_WIFI_CREDENTIALS`・`_MAX_ENTRIES`・`_SAE_PASSWORD_LENGTH`・`_CONNECT_STORED_CONNECTION_TIMEOUT`（Wi-Fi の
    ヘッダーと `zephyr_wifi_mgmt.c`）は、互換層のヘッダーより先に定義されている必要があります（NuttX 版は
    `<nuttx/config.h>`、共通の `.c` には前置きヘッダーが読ませる）。
- **スピンロック**: Zephyr の 1 CPU の `k_spin_lock()` は割り込みを禁止し、その間はほかのスレッドに切り替わり
  ません（`k_sem_give()` などで起きたスレッドは解放のときに動く）。NuttX は割り込み禁止中でも
  `nxsem_post()` で起きたスレッドにすぐ切り替えるので、NuttX 版の `k_spin_lock()`（`zephyr_os_spinlock.h`）は
  `sched_lock()` と `up_irq_save()` にしています（割り込みハンドラーの中では `sched_lock()` は何もしない）。`atomic_*()` も
  これで守ります（Cortex-M0+ に排他ロード・ストアは無く、Zephyr の ARMv6-M も割り込み禁止で実装）。
- **NuttX の名前との衝突**: NuttX の `<nuttx/atomic.h>`（多くのカーネルのヘッダーが読む）にも `atomic_t` と
  `atomic_set()` などがあり、意味が違います。互換層の `<zephyr/sys/atomic.h>` はそれを先に読んでから
  （`zephyr_os_atomic.h`）、そのファイルの中で名前を引き取ります（`atomic_t` は `zephyr_atomic_t`、NuttX の
  マクロは `#undef`）。
  両方を使うファイルは NuttX のヘッダーを先に読みます（`common/` のファイルは前置きヘッダーが読む）。ボードの
  起動処理は Zephyr のヘッダーを読まず、`zephyr_compat.h` だけを読みます。
- **前置きヘッダー**（`common_prelude.h`）: Zephyr 版と共有するソースと互換層の共通の `.c`（リポジトリの
  `common/`）を NuttX 版でコンパイルするとき、ビルド定義（`apps/imu`・`apps/wifi`・`apps/sensor` の `Makefile`、
  `compat/Make.defs`、`lsm6dso/Make.defs`、`nina/Make.defs`、ボードの `src/Make.defs`）が各ファイルの前に
  `-include` で読ませます（Zephyr 版がすべての
  ファイルに `-imacros autoconf.h` で構成を読ませるのと同じ形）。中身は `<nuttx/config.h>`、互換層より先に要る
  NuttX のヘッダー（`<zephyr/kernel.h>` が先に読むもの: `<sys/types.h>`、`<nuttx/irq.h>`、`<nuttx/mutex.h>`、
  `<nuttx/semaphore.h>`、`<nuttx/wdog.h>`、`<stdio.h>`）と、共有するソースが使う Zephyr の Kconfig の値
  （`CONFIG_SENSOR_LOG_LEVEL` 3。Zephyr 版の構成と同じ値。`lsm6dso_sensor.c`・`lsm6dso_sensor.h` の
  `CONFIG_SENSOR_ASYNC_API` を、`CONFIG_LSM6DSO_STREAM` があれば定義する）です。そのため共有する
  ソースには NuttX のヘッダーの行がありません。ソケットのヘッダーは入れていません（`accept()` などの名前がソースの
  関数と重なる。必要なファイルは互換層の `<zephyr/net/socket.h>` が読む）。ホストテストも同じヘッダーを付けます
  （`__NuttX__` が無いと Kconfig の値だけ。`time_sync` と LSM6DSO のドライバーのテストは偽の `<nuttx/config.h>` を
  先に付ける）。
- **OS ごとのヘッダー**（共有するソースの）: 共有するソースのうち版ごとに違う数行は、同じ名前の小さなヘッダーに出し、Zephyr 版と
  NuttX 版がそれぞれのものを読みます（上の LSM6DSO のデバイスの `lsm6dso_instance.h` のほかに）。`sensor_shell_os.h`（`sensor_shell.c` と互換層の `sensor_shell_stream.c` が最初に読む。NuttX 版
  `nuttx/apps/sensor/` は Zephyr 版の構成の値とログを読み、`sensor get` が結果の表示を待つ 2 行と処理スレッドのコールバックを関数にする。
  Zephyr 版（パッチを当てたツリーの `drivers/sensor/sensor_shell_os.h`）は何もしない）と `wifi_profile_os.h`（再接続のスレッドの
  `wifi_reconnect_tid` の宣言。Zephyr 版 `zephyr/app/usb-console/src/` は `K_THREAD_DEFINE()` が作るとおり
  `const k_tid_t`、NuttX 版 `nuttx/apps/wifi/` は互換層の `K_THREAD_DEFINE()` がスレッドを始めるときに値を書くので
  `const` なし）です。
- **セマフォと `k_poll()`**: `k_sem_give()` は上限で止め、そのセマフォを `k_poll()` で待っているスレッドを
  起こします（取らずに知らせるだけ。Zephyr の `K_POLL_MODE_NOTIFY_ONLY`）。`k_sem_take()` は 0、`-EBUSY`
  （`K_NO_WAIT`）、`-EAGAIN`（時間切れ）。`k_poll()` はセマフォだけを扱います。
- **RTIO**: Zephyr の構成（`CONFIG_RTIO_SUBMIT_SEM`・`_CONSUME_SEM`・`_SYS_MEM_BLOCKS`、コールバックと
  ユーザーモードなし）の写しです。寸法は呼ぶ側の `RTIO_DEFINE_WITH_MEMPOOL(name, 送信数, 完了数, ブロック数,
  ブロックの大きさ, 境界)` で決まります（`imu_stream.c` は 4、16、128、64、4）。iodev の submit は
  `rtio_submit()` の呼び出し元の文脈で呼ばれます。multishot は成功すると、完了を出す前に iodev に戻り
  （プールのバッファーの結び付けを外して）、失敗かキャンセルでバッファーと送信を解放して終わります。完了の
  空きが無いと完了は捨てられて `xcqcnt` に数えられます（そのバッファーは誰も解放できず残ります。Zephyr と
  同じ）。`rtio_cqe_consume()` は完了を待たず、`consume_sem` を取ります。ハンドルは送信のプールの番号と世代で、
  解放のたびに世代が進み、古いハンドルは解決もキャンセルもされません。`rtio_sqe_cancel()` は印を付けて
  iodev のキャンセルの関数（あれば）を呼びます。プールの割り当ては `max` から `min` まで 1 ブロックずつ減らして
  最初に取れた連続ブロック（最初の適合）。Zephyr との違いは、待ち行列（`struct mpsc`）がロックフリーでなく
  割り込み禁止を使うことと、プールの空きリストを起動時（`rtio_init()`）ではなく最初の割り当てで作ることです
  （渡す順は同じ）。移していないもの: チェーンとトランザクション、executor 自身の操作（コールバック、待ち、
  遅延）、書き込みなどの操作、`rtio_sqe_copy_in()`・`rtio_cqe_copy_out()`、`rtio_pool`、完了のコールバック、
  ユーザーモード。
- **RTIO のワークキュー**: Zephyr の `rtio_workq.c` と同じく、要求 4 つのプールと 1 スレッド（`RTIO WQ 0`、
  優先度 0 → 118、スタック 1024）で、投入順に実行します。単発取得のフォールバック（`lsm6dso_rtio.c`）が
  使います。NuttX はタスク名の空白を `_` にするので、`ps` では `RTIO_WQ_0` と出ます。
- **GPIO**: ポートは `gpio0`（GPIO0〜29）だけで、入力だけです（出力は `-ENOTSUP`）。ピンの設定、割り込み
  （エッジ・レベル、論理・物理）、レベルの読み出し、コールバックは Zephyr の `gpio.h` と `gpio_rpi_pico.c`
  の写しで、ボードの `src/rp2040_gpio_os.c` が pico-sdk の `gpio_set_pulls()`・`gpio_set_function()`・
  `gpio_set_dir()`・`gpio_set_irq_enabled()` と同じレジスター操作をします（イベントを有効・無効にする前に
  `INTR` のそのピンのイベントを消す）。割り込みは NuttX の `rp2040_gpio` のバンクの割り込みから来ます
  （有効なイベントを消してからピンのハンドラーを呼ぶ）。
- **センサーの時計**: Zephyr 版はシステムクロック（SysTick、125 MHz、`k_cycle_get_64()`）、NuttX 版はボードの
  TIMER（水晶の 1 MHz）です。ストリームのバッチの時刻と単発取得の時刻は 1 µs 単位になります。
- **起動**: `zephyr_compat_initialize()` を、ボードの起動処理が I2C0（その DMA のチャネルを含む）・LSM6DSOX より前に呼び、
  システムワークキュー（`sysworkq`、優先度 119、スタック 1024）、RTIO のワークキュー、`gpio0` を用意します
  （Zephyr がそれらを `POST_KERNEL` でセンサーより先に用意するのと同じ順）。`imu_fifo_worker` は LSM6DSOX の
  初期化（`lsm6dso_stream_init()`）が作ります。
- **静的なスレッド**（`K_THREAD_DEFINE()`）: Zephyr と同じ引数で、静的なスタック、`K_PRIO_PREEMPT(prio)` の
  優先度、名前はトークンそのもの（`imu_stream_tid`）のスレッドを定義します。Zephyr はこれらを `main()` の前に
  自動で始めますが、NuttX にはその一覧が無いので、init が FreeRTOS 版と共有のアプリの表
  （`common/imu/app_init.c` の `app_threads_start()`、`common/imu/app_main.h`）から `K_THREAD_DEFINE_START(name)`
  （`zephyr_compat.h`）で 1 本ずつ、`main()` 相当の処理（`app_main()`）の前に始めます。スレッドは NuttX のカーネル
  スレッド（`kthread_create_with_stack()`）で、入口の関数が戻ると終わります。`options` と `delay` は 0 だけ
  受け付けます（ほかはビルドエラー）。
- **時間**: `k_uptime_get()` は Zephyr の `k_ticks_to_ms_floor64(k_uptime_ticks())`（tick 数を ms に切り捨て）
  と同じ値です。NuttX の tick はボードの TIMER を 100 µs で割ったもの（tickless のタイマー）なので、TIMER を
  1 回読んで 1000 で割ります（1000 が tick の倍数なので同じ値。ホストテストで確認）。64 ビットで、32 ビットの
  `clock_systime_ticks()` のような約 5 日での一周はありません。`k_sleep()` は `K_NO_WAIT` で `sched_yield()`、
  それ以外は `nxsched_ticksleep()` で「今の tick + tick 数 + 1」まで眠ります（Zephyr の `z_add_timeout()` と
  同じ切り上げ）。`k_wakeup()` が無いので戻り値は常に 0 です。`k_msleep()`・`k_usleep()` は NuttX の
  `nxsched_msleep()`・`nxsched_usleep()` で、時間を tick に切り上げ、`k_sleep()` と同じ時点に起きます（Zephyr と
  同じ規則）。0 以下は Zephyr と同じく `sched_yield()`、シグナルでは終わらず、戻り値は 0 です。`k_busy_wait()` は
  TIMER を見て待つ `up_udelay()`（ボードの `rp2040_oneshot.c`）です。この 3 つは LSM6DSO のドライバーが使います。
- **`hwinfo_get_device_id()`**: Zephyr 版の RP2040 のドライバー（`hwinfo_rpi_pico.c`、
  `CONFIG_HWINFO_RPI_PICO_CHIP_ID` なし）と同じく、外付け flash の 64 ビットの unique ID（コマンド 0x4B）を
  flash が送る順のまま返します（最大 8 バイト）。読み出しはボードが起動時に 1 回行い（`rp2040_chipid.c`）、
  ここではその写しを返します。ボードがまだ読んでいなければ `-EAGAIN` です（Zephyr のドライバーは失敗しない）。
- **シェル**: `shell_print()`・`shell_info()`・`shell_warn()`・`shell_error()` は改行を付けて書きます（Zephyr は
  情報を緑、警告を黄色、エラーを赤で出す。ここでは色を付けない）。書き先は `sh` で決まります。
  - `sh` が NULL: 呼んだタスクの標準出力（`shell_print()`・`shell_info()`）と標準エラー（`shell_warn()`・
    `shell_error()`）。`common/shell/subcmd.h` で表を書いたコマンド（`kv`、`nina` など）は NULL を渡します。
  - `sh` がバックエンド: `SHELL_CMD_REGISTER()` で登録したコマンド（`sensor`、`imu`、`ntp`）が受け取るもので、
    NuttX 版のバックエンド（`nuttx/apps/subcmd/zshell_os_impl.h`。`struct shell` は `zephyr_os_shell.h`）が
    コマンドの標準出力の端末ごとに 1 つ持ちます（USB は `/dev/ttyACM0`、
    Telnet は `/dev/telnet0`。`F_GETPATH`、Telnet の装置はそれをソケットに渡してしまうので `/dev` の中から
    `fstat()` の i-node 番号で探す）。Zephyr のシェルのインスタンスと同じく、起動中ずっと残ります。そのコマンド
    自身が書くとき（実行中のタスク）は標準出力・標準エラーに、ほかのタスク・スレッドが書くとき（sensor シェルの
    処理スレッドが、コマンドが戻った後に書くとき）は、1 回ごとに端末を開いて 1 回の `write()` で書き、閉じます。
    Telnet のクライアントがいない間は開けないので捨て、次のクライアントに続きます（Zephyr の Telnet の
    バックエンドと同じ）。Telnet のセッションの装置を書く間しか開いていないので、セッションの終わり（最後の
    記述子が閉じたとき）と次の接続を妨げません。
  - コマンドの表のマクロは Zephyr の `shell.h` と同じ形で、`SHELL_CMD_REGISTER()` が組み込みコマンドの
    入口を作ります。入口は既定では `main()` で、アプリの main のソース（`MAINSRC`）がそのまま組み込みコマンドに
    なります（`sensor_shell.c`、`imu_stream.c`）。main のソースでないファイルは、ビルド定義がそのファイルにだけ
    `Z_SHELL_CMD_NAMED_MAIN` を付けてコンパイルし、入口は `int z_shell_main_<名前>(int argc, char *argv[])` に
    なります。アプリの `main()` がそれを呼びます（`time_sync.c` の `ntp`: `apps/imu/Makefile` が
    `time_sync.c` に付け、`apps/ntp/ntp_main.c` の `main()` が `z_shell_main_ntp()` を呼ぶ）。
    `common/shell/zshell.c` の `z_shell_cmd_main()` が、Zephyr の `shell.c` の `execute()`・`exec_cmd()`・
    `cmd_precheck()`、`shell_wildcard.c`、`shell_help.c` と同じ手順で、nsh が分けた語を照合し（動的な
    サブコマンドの関数を Zephyr と同じ順に呼ぶ）、ワイルドカードを展開し、引数の数を確かめ、ヘルプ
    （`SHELL_HELP()` の説明と `Usage:` の行、80 桁での折り返し）を出し、ハンドラーを呼びます。ハンドラーの
    `argv` は Zephyr と同じく最後の語の後が NULL です。Zephyr との違い: サブコマンドの集合が動的かどうかを
    `struct shell_cmd_entry` が持つ（Zephyr は `union shell_cmd_entry` を置いたリンカーのセクションで見分ける）、
    選択モード・別名・リモートのコマンド・生の引数・Tab 補完（nsh のもの）が無い、カーソルを VT100 の
    シーケンスで動かす代わりに空白を出す。
  - `zshell.c` は OS に依存しません。OS の部分（`common/shell/zshell_os.h` が一覧を持つ）は、`struct shell`、
    バックエンドへの出力 `z_shell_backend_vfprintf()`、呼んだタスクのセッションのバックエンドを返す関数、ハンドラーの
    呼び出しの前後の印の 2 つです。ヘルプとエラーは `printf()`・`fprintf(stderr)` で出すので、コマンドを実行する
    タスクの標準出力・標準エラーがそのセッションの端末であることが前提です（NuttX ではコマンドごとのタスクが持つ）。
  - `shell_device_get_binding()`・`shell_device_filter()` は Zephyr の `shell_utils.c` と同じで、上の一覧の
    デバイスを見ます（`DEVICE_API_IS(sensor, dev)` は一覧の印。Zephyr は API の表がそのクラスのリンカーの
    セクションにあるかで見る）。
- **ネットワーク**: `zsock_inet_pton()`・`zsock_inet_ntop()` は C ライブラリの関数です。ネットワークが無効な
  間は、`imu` が `CONFIG_LIBC_IPv4_ADDRCONV` を選んで IPv4 の変換を入れます。ソケットは NINA の usrsock
  で、`zsock_socket()`・`zsock_connect()`・`zsock_send()`・`zsock_sendmsg()`・`zsock_sendto()`・`zsock_recv()`・
  `zsock_poll()`・`zsock_close()` は NuttX の関数（`socket()` … `poll()`・`close()`）です。ブロッキングの接続と送信の
  待ちと期限は NINA のドライバーのもので、受信と `poll()` は `nina_event` のイベントを待ちます（上の「ソケット
  （usrsock）」）。`poll()` の結果は、Zephyr 版の NINA の `poll()`（`nina_socket.c` の `do_poll()`）と 2 つの場合に
  違います: 接続していない TCP のソケットは `ZSOCK_POLLIN`・`ZSOCK_POLLOUT`・`ZSOCK_POLLHUP`（Zephyr 版は
  `ZSOCK_POLLHUP` だけ）、接続済みの TCP で NINA のエラーフラグか NINA の poll の失敗は `ZSOCK_POLLHUP`（usrsock の
  `REMOTE_CLOSED`。Zephyr 版は `ZSOCK_POLLERR`）です。`zsock_poll()` を受信の確認（100 ms ごと、待ち時間 0）と
  CONNACK の待ち（5000 ms）に使う `imu_raw_mqtt.c` は、どちらでも接続を終えます。
- **サイクルカウンター**: Zephyr 版の `k_cycle_get_*()` は 125 MHz の SysTick の数（負荷で遅れる）で、
  NuttX 版はボードの TIMER（`rp2040_timer_us()`、水晶の 1 MHz。センサーの時計と同じ）です。1 サイクルが 1 µs
  なので、µs への換算（`k_cyc_to_us_floor64()`・`_ceil64()`）はそのままの値です。`k_cycle_get_32()` は下位
  32 ビットで約 71.6 分で一周します（Zephyr 版は約 34 秒）。`k_cyc_to_ns_floor64()` は 1000 倍です。
  `imu_raw_mqtt.c` がキューのロックの待ちと送信の時間を、LSM6DSO のドライバーが FIFO の読み出しの時間を計るのに
  使います。
- **インターフェース**（`net_if.h`）: インターフェースは NINA の `wlan0` 1 つで、`net_if_get_default()` と
  `net_if_get_wifi_sta()` はどちらもそれを返します（`nina_wifi_mgmt.c` が登録する前は NULL）。アドレスは 1 つで、
  NINA のつなぎ（`nina_netdev.c`）が書きます。常に「優先」（IPv4 の重複検出は Zephyr 版の構成でも無い）なので、
  `net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)` はアドレスがある間それを返します。値の読み出しは
  ネットワークデバイスのロック（`netdev_lock()`。`ifconfig` の ioctl と同じ）の下で行います。`net_in_addr` は
  NuttX の `in_addr` です。インターフェースのイベント（`NET_EVENT_IF_UP` など、アドレスの追加・削除）は出しません
  （移植の中で使うものが無い）。
- **`net_mgmt`**: 要求 `net_mgmt(NET_REQUEST_X, iface, data, len)` は、Zephyr のマクロと同じく
  `net_mgmt_NET_REQUEST_X()` を呼び出したスレッドで呼びます。イベントは Zephyr 版の構成
  （`CONFIG_NET_MGMT_EVENT_THREAD`・`_QUEUE`・`_INFO`、待ち行列 5 個、空きを 10 ms 待つ）と同じく、情報を写して
  待ち行列に入れ、スレッド `net_mgmt`（優先度 −1 → 119、スタック 2048）が順に取り出して、マスクの合うコール
  バックを呼びます（`cb->info`・`info_length` を付け、コールバックの一覧のロックの下で）。10 ms で空きが
  できなければ捨てます（Zephyr 版の警告はログのレベル 0 で消えている）。イベントの番号はこの移植の
  ビット（Zephyr の 64 ビットの層と命令の値とは違う）で、同じ名前で比べます。`nina_net_initialize()` が
  インターフェースより先にスレッドを始めます（Zephyr の `net_init()` と同じ順。`nina_wifi_mgmt_initialize()`）。
- **Wi-Fi の要求**（`zephyr_wifi_mgmt.c`）: Zephyr の `wifi_mgmt.c` と同じく、インターフェースの API
  にドライバーの操作が無ければ `-ENOTSUP`（NuttX 版のインターフェースは接続だけを持つ。`nina_wifi_mgmt.c`。スキャン・切断・状態は
  wapi がコアを直接呼ぶ）、
  `IFF_UP` でなければ `-ENETDOWN`、接続の引数は同じ検査（チャネル、SSID の長さ 1〜32、パスフレーズの
  長さ 8〜64（WEP は 5/10/13/26）、セキュリティの種類ごとの必須項目）をしてからドライバーを呼びます
  （Zephyr 版の構成には wpa_supplicant が無い）。スキャンの結果は 1 件ずつ `NET_EVENT_WIFI_SCAN_RESULT`、
  終わりは `NET_EVENT_WIFI_SCAN_DONE`（`struct wifi_status`）のイベントにします。
  `NET_REQUEST_WIFI_CONNECT_STORED` は、保存した資格情報を番号の順に読み、資格情報から接続の引数を作って
  （チャネル 0 は「どれでも」、期限 0 は 30 秒、種類が範囲外なら暗号化なし、帯域と MFP はフラグから）
  `NET_REQUEST_WIFI_CONNECT` を出し、最初に受け付けられたところで止めます。戻り値は常に 0 です（Zephyr と同じ。
  Zephyr 版の構成ではこのファイルのログはレベル 0 で消えているので、ログも出しません）。
- I2C のバスは、NuttX の I2C マスターと速さを持つデバイスです（`i2c_compat_bind()` でボードが結び付ける）。
  Zephyr ではバスのドライバーが速さを持ち、`i2c_configure()` が以後のすべての転送の速さを変えます。NuttX の
  転送はメッセージごとに周波数を持つので、互換層の転送（`i2c_burst_read_dt()` など）がデバイスの速さを
  メッセージに入れます。`i2c` ツールなど、NuttX のマスターを直接使うものは自分の周波数で動きます。
- `i2c_configure()` は Zephyr の RP2040 向けドライバーが受け付ける区分（100 kHz、400 kHz、1 MHz）だけを受け付け、
  ほかは何も変えずに `-EINVAL` を返します。モードのビットは見ません（Zephyr のドライバーもコントローラーに
  固定）。DMA の不具合で I2C0 が隔離されているとき、Zephyr の `i2c_configure()` は `-EIO` を返しますが、
  互換層は速さを記録して 0 を返し、次の転送が `-EIO` になります。
- NuttX の `<nuttx/i2c/i2c_master.h>` には同じ名前の `i2c_write()`・`i2c_read()` と、Hz の値の `I2C_SPEED_*` が
  あります。互換層では、バスのデバイスに対する Zephyr の `i2c_write()`・`i2c_write_read()` を
  `i2c_compat_write()`・`i2c_compat_write_read()` という名前にしました（`_dt` の付く形は Zephyr の名前のまま）。
  `<zephyr/drivers/i2c.h>` を読むファイルでは `I2C_SPEED_*` は Zephyr の区分です（両方読むなら NuttX のものを先に）。

### KV: Zephyr の NVS と settings の写し

NuttX 版（と FreeRTOS 版）の KV は、Zephyr 版と同じバイト列を flash に書くため、Zephyr v4.5.0 の NVS と
settings のうち Zephyr 版が使う部分を改変せずに写し、Zephyr 版と同じ設定で動かします。写したファイルが元と
同じことは、Zephyr のツリー（`zephyr/zephyr`）があるときにホストテスト（`common/tests/run.sh` の
`zephyr_copies`）が `cmp` で確かめます。

| 写し（`common/compat/`） | 元（Zephyr v4.5.0） | 変更 |
|---|---|---|
| `subsys/kvss/nvs/nvs.c`、`nvs_priv.h` | `subsys/kvss/nvs/` | なし |
| `subsys/settings/src/settings.c`、`settings_store.c`、`settings_init.c`、`settings_nvs.c`、`settings_priv.h` | `subsys/settings/src/` | なし |
| `subsys/settings/include/settings/settings_nvs.h`、`settings_file.h` | `subsys/settings/include/settings/` | なし |
| `subsys/crc/crc8_sw.c` | `subsys/crc/crc8_sw.c` | なし |
| `lib/utils/hex.c` | `lib/utils/hex.c` | なし |
| `zephyr/kvss/nvs.h`、`zephyr/settings/settings.h`、`zephyr/sys/slist.h`、`zephyr/sys/list_gen.h` | `include/zephyr/...` | なし |
| `wifi_credentials_backend_settings.c` | `subsys/net/lib/wifi_credentials/` | 上の表（2 つ） |

- 設定: 写したソースは `zephyr_kv_config.h`（Zephyr 版の `.config` の値。`CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT` 1、
  `CONFIG_SETTINGS_NVS_SECTOR_COUNT` 8、`CONFIG_SETTINGS_DYNAMIC_HANDLERS`、`CONFIG_MULTITHREADING`、ログの
  レベル 3。`CONFIG_NVS_LOOKUP_CACHE`・`CONFIG_NVS_DATA_CRC`・`CONFIG_NVS_INIT_BAD_MEMORY_REGION`・
  `CONFIG_SETTINGS_NVS_NAME_CACHE` は Zephyr 版と同じく無効）を前置きしてビルドします。無効の値を `#if` で見る
  ので `-Wundef` を外し、`nvs.c` は `uint32_t` を `%d` で出す（Zephyr の `uint32_t` は `unsigned int`、newlib と
  NuttX は `unsigned long`。どちらも 32 ビット）ので `-Wformat` を外します。
- flash の上の形と、それを決める値（Zephyr 版の根拠）:

  | 値 | ここ | Zephyr 版 |
  |---|---|---|
  | 領域 | `0xffe000` から `0x2000`（`flash_kv.h`、`<zephyr/storage/flash_map.h>`） | `app.overlay` の `storage_partition` |
  | セクター | 4096 バイト × 2 | `rp2040.dtsi` の `erase-block-size` × `SECTOR_SIZE_MULT` 1、8 KiB に入る数（`settings_nvs.c`） |
  | 書き込み単位・消去値 | 1、0xFF | `flash_rpi_pico.c` の `flash_rpi_parameters`、DT の `write-block-size` |
  | 索引（ATE） | 8 バイト、`crc8_ccitt(0xff, ...)` | `nvs_priv.h`（無改変） |
  | 名前と値の ID | 0x8000、0x8001〜、+0x4000 | `settings_nvs.h`（無改変） |
  | 値 | 校正値 56 バイト、文字列、`imu.session` 4 バイト、資格情報 224 バイト | `imu_cal_codec.c`・`imu_stream.c`（共通）、`struct wifi_credentials_personal`（並びを `BUILD_ASSERT` で Zephyr 版の ELF と同じに固定。arm-none-eabi の 1 バイトの enum） |

- 前提: settings のロック（`K_MUTEX_DEFINE(settings_lock)`、全部 0 から始まるミューテックス）と NVS のロックは
  互換層の `k_mutex`（再帰。`kv wipe` は settings のロックを持ったまま settings を初期化し直す）。初期化は
  Zephyr と同じく最初の `settings_subsys_init()`（失敗したら次の呼び出しでやり直す）。
- **flash の境界**（`flash_kv_os.h`。OS ごとに実装する。NuttX 版はボードの `src/rp2040_flash_kv_os.c`、FreeRTOS 版は
  Pico SDK の `flash_range_program()`・`flash_range_erase()` を割り込みを止めて呼ぶ）:

  ```c
  int flash_kv_os_initialize(void);
  bool flash_kv_os_ready(void);
  int flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);
  int flash_kv_os_program(uint32_t page_offset, const uint8_t page[256]);
  int flash_kv_os_erase(uint32_t sector_offset);
  ```

  `initialize` は起動の処理が 1 回呼ぶ（リンカースクリプトの範囲を確かめる。何も消さない）。`offset` は flash の先頭（XIP の 0x10000000）からの位置。`ready` が偽のあいだ flash map はデバイスが無い扱い
  （`-ENODEV`）。`read` は今の flash の中身を返す（書き込み・消去の直後も）。`program` は 256 の倍数の位置の
  1 ページを書く（書かない所は 0xFF。書き込みはビットを落とすだけ）。`erase` は 4096 の倍数の位置の 1 セクターを
  消す。どれも 0 か負の errno を返し、スレッドからだけ、同時に 1 つだけ呼ばれます（互換層がロックを持つ）。
  呼ぶ側（`zephyr_flash_kv.c`）は領域の中の揃った要求しか渡しませんが、OS の実装も範囲を確かめ直し、外なら
  flash に触れずに `-EINVAL` を返します。書き込みと消去の間は割り込みを止め、そのあいだ flash 上のコードを
  動かしません（NuttX 版はページ 1 回・セクター 1 回ごと）。
- `zephyr_flash_kv.c`: flash のデバイス（`z_flash_kv_device`）と flash map。要求は領域の中だけを受け付け
  （Zephyr 版のドライバーは flash 全体）、書き込みを 256 バイトのページに分け、消去をセクターごとに境界へ渡し
  ます。`flash_area_open()` は起動後に 1 回、領域の中身を判定し、Zephyr の NVS でも消去済みでもなければログに
  出して `-EILSEQ` を返します（settings は初期化されず、何も書かない）。`kv wipe confirm` の後にもう一度判定します。
