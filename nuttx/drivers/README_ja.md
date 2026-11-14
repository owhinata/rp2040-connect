# ドライバー（NuttX 版）

[English](README.md) | 日本語

`nuttx/drivers/` は NuttX のカーネル側でビルドするこの移植のコードです（`nuttx/nuttx/external` からの
symlink で、`libexternal.a` になります）。ボード側の部分は `nuttx/boards/arduino-nano-rp2040-connect/src/`
にあります。Zephyr 版のドライバー説明は `zephyr/app/usb-console/drivers/README_ja.md` です。この文書は
移植の途中で、移植した部分だけを書いています。

## NINA-W102（`nina`）

### SPI の転送層

| ファイル | 役割 |
|---|---|
| `nina/nina_spi.c` | コマンドと応答のフレーミング（`nina_request()`、`nina_ip_config()`）。Zephyr 版の `drivers/nina/nina_wifi.c` の同名の関数と、分岐・順序・定数・エラー値を 1 対 1 で対応させた写し |
| `nina/nina_proto.h` | 公開する宣言。Zephyr 版の同名ファイルを変更せずに写したもの |
| `nina/nina_os.[ch]` | ロック（再帰可能なミューテックス、優先度継承あり）、sleep、ms の時計。Zephyr の `K_MUTEX`・`k_sleep()`・`k_uptime_get()` に当たる |
| `include/nano_nina.h`、ボードの `src/nano_nina.c` | SPI1 と 4 本の信号線（CS、READY、RESET、IRQ）。Zephyr のデバイスツリーと SPI・GPIO ドライバーが行っていた設定に当たる |

- 要求は呼び出したスレッドの文脈で同期して処理します。ミューテックスで直列化し、CS を下げてから
  応答を読み終えるまで（READY を待つ間も）CS を保持します。
- NINA のリセットは最初の要求のときに 1 回だけ行います（10 ms、750 ms）。起動時にはボードが SPI1 と
  そのピンを設定し、CS を非アクティブにするだけです。
- SPI は NuttX の `rp2040_spi.c` をポーリングで使います（DMA なし）。転送ごとに `SPI_LOCK` を取り、
  最初の転送でモード 0・8 ビット・`CONFIG_NANO_NINA_SPI_FREQUENCY` を設定します。`SPI_SELECT` は
  使いません。
- 期限と配線は `nuttx/README_ja.md` の「NINA-W102」の節にあります。
- `nina/nina_spi.c` はホストテスト（`nuttx/tests/nina_spi.c`）で、偽のボードと時計を使って確かめます。

### ソケット（usrsock）

Zephyr 版はソケットのオフロード（`drivers/nina/nina_socket.c`）で、NuttX 版は usrsock です。NuttX の
`CONFIG_NET_USRSOCK_CUSTOM` で、カーネル内に `usrsock_request()` を置きます。NuttX 自身の TCP/IP スタックは
使いません（`NET_TCP_NO_STACK`、`NET_UDP_NO_STACK`）。

| ファイル | 役割 |
|---|---|
| `nina/nina_sock.[ch]` | ソケットの表（10 個）と NINA のソケット命令。Zephyr 版の `nina_socket.c` の `nina_socket_create()`・`do_bind()`・`do_listen()`・`do_accept()`・`do_connect()`・`do_close()`・`do_setsockopt()` と、送受信の `udp_send()`・`do_send()`・`do_sendmsg()`・`udp_recv()`・`do_recv()` を、名前・分岐・順序・定数・errno の名前を保って移したもの（送受信の待ちのループは要求 1 回で 1 周）。`FIONREAD`（`nina_sock_fionread()`）と独自のソケットオプション `NINA_SO_CALLER_POLLS`・`NINA_SO_POLL`（`nina_sock_poll_revents()`、`include/nina_sockopt.h`）は NuttX 版だけ。OS に依存しない |
| `nina/nina_usrsock.c` | usrsock の要求を上の関数に振り分け、応答（ACK）を返す |
| `nina/nina_poller.c` | スレッド `nina_event`。NINA のソケットの状態を見て usrsock のイベントにする（呼び出し元だけが問い合わせるソケットは接続中だけ）。待つ送信の再試行の時間も計る |
| `nina/nina_usrsock.h` | 上の 2 つの間の宣言 |

- 要求は、呼び出したスレッドの文脈で、その優先度のまま同期して処理します（Zephyr 版のソケット呼び出しと
  同じ）。usrsock は要求を 1 つずつ渡し、その間は usrsock のロックを持っています。NINA の命令は
  `nina_request()` の中の NINA のロックで直列化します。ロックの順序は常に「usrsock のロック → NINA の
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
- 呼び出し元だけが問い合わせるソケット: 互換層の `zsock_socket()` は、作ったソケットにドライバー独自の
  ソケットオプション `NINA_SO_CALLER_POLLS`（レベル `NINA_SOL`、`include/nina_sockopt.h`）を付けます（MQTT と
  時刻同期のソケット）。印の付いたソケットは、`nina_event` が接続中の確認のほかは問い合わせず、受信の後の
  問い合わせもしません。Zephyr 版の NINA のソケットと同じく、使う側（互換層の `zsock_recv()`・`zsock_poll()`）
  だけが、呼び出したスレッドで NINA に尋ねます（下の互換層の「`zsock_poll()`」）。印の無いソケット（Telnet、
  iperf、nsh のコマンド）は上のとおり `nina_event` が問い合わせます（Zephyr 版の Telnet も socket service の
  `poll()` が 20 ms ごとに尋ねます）。印の付いたソケットでも、接続の確認（20 ms ごと、10 秒まで）と、待つ
  送信の次の周までの時間（1 ms）は `nina_event` が受け持ちます。usrsock では、要求を持ったまま待つと他の
  ソケットの要求も止まるためで、周の中の NINA の命令は呼び出したスレッドで送ります。印の付いたソケットで
  待つ受信（`MSG_DONTWAIT` なしの `recv()`）や NuttX の `poll()` を使うと来ないイベントを待ち続けるので、
  互換層の呼び出しだけで使います。
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
  自分で扱います）。例外はドライバー独自の `NINA_SOL` の 2 つ（`setsockopt` の `NINA_SO_CALLER_POLLS`、
  `getsockopt` の `NINA_SO_POLL`）です。`ioctl` は `FIONREAD`（下）のほかは `ENOTTY` を返して NuttX 自身の処理（ネットワーク
  デバイスの `SIOC*` など）に任せます。`socket(AF_INET, SOCK_CTRL)`（`ifconfig` などが使う）は `ENOTSUP` で
  NuttX 自身に任せます。
- 送受信の値は Zephyr 版と同じです: TCP の送信は 1 回 3584 バイトまで（`CONFIG_NANO_NINA_TCP_MAX_IO`）、
  UDP は 512 バイトまで（超えると `EMSGSIZE`）、受信は 1 回 512 バイトまで。`sendmsg` は複数のベクターを
  3584 バイトまで 1 つにつなげて 1 回の命令で書きます（MQTT のヘッダーと本体を 1 回で出すため。NuttX の
  usrsock はベクターを分けたまま渡します）。1 つのベクターの送信は `send`・`sendto`、複数なら `sendmsg` として
  扱います（UDP の `sendmsg` は Zephyr 版と同じく `EOPNOTSUPP`）。
- 送受信の待ち方: 要求 1 回で Zephyr 版の待ちのループを 1 周、呼び出したスレッドで行い、周と周の間は
  usrsock が待ちます。
  - 待つ TCP の送信で NINA が 1 バイトも受け取らなかったときは、`SENDTO_READY` を落として `EAGAIN` で
    応え、`CONFIG_NANO_NINA_SEND_RETRY_MS`（既定 1 ms。Zephyr 版の `CONFIG_NINA_SEND_RETRY_MS`）の後に
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
    NuttX が扱います）。受け取った後も `RECVFROM_AVAIL` を落とし、`nina_event` がすぐに poll し直します。
    usrsock は `RECVFROM_AVAIL` が無いと受信をドライバーに渡さないので、印の無いソケットでは、受信や `poll()`
    の最初の問い合わせが `nina_event` の poll まで（最大 20 ms）遅れることがあります（Zephyr 版はその場で
    尋ねます）。印の付いたソケットは、互換層が `FIONREAD` で先に尋ねるので遅れません。
  - データを見つけた poll（`nina_event`、`FIONREAD`）は、次の受信の最初の poll の代わりに使います（その間に
    データを取る者はいません）。
- TCP の受信の 0 は、相手が閉じたか接続が失敗したことを示します（NINA はどちらも 0 バイトと答えます。
  Zephyr 版と同じ）。UDP は接続した相手（`connect()`）以外からのデータグラムを捨てます（Zephyr 版と同じ）。
- `ioctl(FIONREAD)` は NuttX 版だけのもので、呼び出したスレッドでその場で NINA に poll を 1 回送り、データ
  （TCP では相手の切断も）があれば 1、無ければ 0 を返します（NINA はバイト数を教えません）。1 のときは応答に
  `RECVFROM_AVAIL` を付けるので、続く `recv(MSG_DONTWAIT)` が NINA に届きます。互換層の `zsock_recv()` が使います
  （Zephyr 版の NINA の `recv()` は呼ばれるたびに poll を 1 回送ってから読みます。時刻同期の 1 ms ごとの受信と
  MQTT のライブラリの `recv(MSG_DONTWAIT)`。下の互換層の表）。
- `getsockopt(NINA_SOL, NINA_SO_POLL)`（int）も NuttX 版だけのもので、Zephyr 版の `do_poll()` のソケット 1 つ分の
  1 周です: NINA 側のソケットがまだ無ければ尋ねずに TCP は `POLLHUP`、UDP は `POLLOUT`。あれば poll を 1 回送り、
  読める → `POLLIN`、書ける → `POLLOUT`、エラーフラグ → `POLLERR` を返します。poll の失敗はその errno です。
  互換層の `zsock_poll()` が使います。
- `nina/nina_sock.c` はホストテスト（`nuttx/tests/nina_sock.c`）で、偽の `nina_request()` と時計を使って
  確かめます。

### 名前解決

`nina/nina_dns.c` は Zephyr 版の `drivers/nina/nina_dns.c` を行単位で移したもので、名前を NINA の名前解決の
命令（0x34 で名前を渡し、0x35 でアドレスを読む）で IPv4 のアドレス 1 つにします。数値のアドレスは NINA に
尋ねません。Zephyr 版はソケットのオフロードの `getaddrinfo()` として登録し、`zsock_getaddrinfo()` がこれを
使います。NuttX 版は専用の関数 `nina_getaddrinfo()`・`nina_freeaddrinfo()`（`include/nina_dns.h`）で、互換層の
`<zephyr/net/socket.h>` が `zsock_getaddrinfo()`・`zsock_freeaddrinfo()`・`struct zsock_addrinfo` をこれに
つなぎます。NuttX の `getaddrinfo()`（`CONFIG_LIBC_NETDB`。DNS サーバーへ自分で UDP で問い合わせる）は
使わず、`CONFIG_NANO_NINA_DNS` は `CONFIG_LIBC_NETDB` と同時には有効にできません。エラーは NuttX の
`EAI_*` の値で、名前は Zephyr 版の `DNS_EAI_*` と同じです（名前が引けない `EAI_NONAME`、SPI の失敗
`EAI_AGAIN`、アドレスの長さが違う `EAI_FAIL`、ヒントの誤り `EAI_BADFLAGS`・`EAI_FAMILY`・`EAI_SOCKTYPE`、
サービスの誤り `EAI_SERVICE`、メモリー不足 `EAI_MEMORY`）。2 つの命令の間はミューテックスで守ります
（NINA の名前解決の結果は 1 つしかないため）。結果は Zephyr 版の 2 KB の専用ヒープではなく `malloc()` で
取ります。ホストテストは `nuttx/tests/nina_dns.c` です。今これを使うのは、時刻同期の NTP サーバーを名前で
指定したときだけです（既定はゲートウェイ）。

### Wi-Fi（ネットワークデバイス `wlan0`）

| ファイル | 役割 |
|---|---|
| `nina/nina_net.c` | Zephyr 版の `drivers/nina/nina_net.c` の写し（先頭の説明と `<nuttx/config.h>` だけを足したもの）。スキャン・接続・切断・状態の取得と、3 つのスレッド `nina_scan_tid`・`nina_connect_tid`・`nina_monitor_tid` |
| `nina/nina_netdev.c` | NINA を NuttX のネットワークデバイス `wlan0` として登録し、wapi の無線の ioctl を Zephyr の Wi-Fi の要求（`net_mgmt()`）にする。NuttX 版だけ |
| `nina/wifi_profile_null.c` | `nina_net.c` が呼ぶ Wi-Fi のプロファイルの関数（`wifi_profile_pause_reconnect()` など）の空の実装。保存した資格情報と自動再接続（`nuttx/apps/wifi/wifi_profile.c`、Zephyr 版の `src/wifi_profile.c` の写し）を入れない構成（`CONFIG_NANO_WIFI_PROFILE` が無いとき）だけで使う |
| `include/nina_net.h`、`include/wifi_profile.h` | `nina_net_initialize()`。`wifi_profile.h` は Zephyr 版の `src/wifi_profile.h` と同じ宣言 |

- `nina_net.c` は Zephyr の API（`net_if`、`net_mgmt`、`wifi_mgmt`、カーネルのオブジェクト）を互換層
  （下の「Zephyr の API の互換層」）で使うので、Zephyr 版と同じコードのまま動きます。値と動作は Zephyr 版と
  同じです: 監視は 2 秒ごとに、接続中なら NINA の状態（0x20）を見て、2 回続けて未接続なら切断として IP の設定を
  消し、切断のイベントを出して再接続を頼みます。接続は NINA に SSID（とパスフレーズ）を渡した後、
  500 ms ごとに最大 60 回状態を見て、接続できたら省電力を切り（0x17）、IP の設定（0x21）を読んで
  インターフェースに入れ、結果のイベント（失敗は `-ETIMEDOUT` など）を出します。要求の世代
  （`generation`／`request_generation`）が変わった古い接続は結果を出さずに終わります。スキャンは接続中なら
  先に切断として扱い、終わった後に再接続を頼みます。手動の切断は再接続を止めます（`wifi_profile_pause_reconnect()`）。
  パスフレーズはログに出しません。
- 起動: ボードの起動処理が `nina_net_initialize()` を LSM6DSOX の後、KV の前に呼びます（Zephyr 版がインターフェースを
  `POST_KERNEL` でセンサーの後に初期化し、NVS を `main()` で用意するのと同じ順）。`net_mgmt` のスレッドを始め、
  `wlan0` を `NET_LL_IEEE80211` で登録して `IFF_UP` にし、Zephyr と同じ順に `driver_init()` と `iface_init()`
  （休止の状態にし、NINA から MAC アドレス（0x22）を読む）を呼びます。これが NINA への最初の命令なので、
  ここで NINA のリセット（10 ms、750 ms）が入ります。3 つのスレッドは init が `K_THREAD_DEFINE_START()` で
  始めます。
- インターフェースの値: IPv4 のアドレス・ネットマスク・ゲートウェイは `wlan0` の `d_ipaddr`・`d_netmask`・
  `d_draddr`、MAC アドレスは `d_mac` に書きます。NuttX の `ifconfig`（`/proc/net/wlan0`）と `SIOCGIFADDR` などは
  そこを読みます。Zephyr の「休止（dormant）」は `IFF_RUNNING` が無い状態です（接続すると `RUNNING` が付く）。
  `wlan0` には `d_ifup`・`d_ifdown` が無く、`ifconfig wlan0 down`・`up` は何もしません（Zephyr 版の
  インターフェースも常に有効）。NuttX 自身の TCP/IP は使わないので、ARP と ICMP は切ってあります
  （`CONFIG_NET_ARP`・`CONFIG_NET_ICMP`）。`CONFIG_NET_ETHERNET` は、`NET_LL_IEEE80211` の登録
  （`netdev_register()` が `CONFIG_NET_ETH_PKTSIZE` を使う）と MAC アドレスの欄（`d_mac.ether`）に要るので
  残します。
- wapi の ioctl（`d_ioctl`。呼び出したスレッドで処理）:

  | ioctl | 処理 |
  |---|---|
  | `SIOCSIWSCAN` | `NET_REQUEST_WIFI_SCAN`。ESSID の指定、パッシブ、チャネルの指定は渡して、ドライバーが Zephyr 版と同じく `-ENOTSUP` で断る |
  | `SIOCGIWSCAN` | スキャン中は `-EAGAIN`、失敗ならそのエラー、終わっていれば結果（`SCAN_RESULT` のイベントで集めた最大 10 件を、NINA の順に、BSSID・チャネル・RSSI（dBm）・SSID・暗号化の有無の無線イベントにする。暗号化の種類が不明なら有無を付けない）。バッファーが足りなければ `-E2BIG` |
  | `SIOCSIWAUTH` | WPA の版と pairwise の暗号を覚える |
  | `SIOCSIWENCODEEXT` | パスフレーズを覚える。この 2 つで覚えた設定は次の `SIOCSIWESSID` で使って忘れる（パスフレーズは消す）ので、`wapi psk` を付けない `wapi essid` は暗号化なしの接続になる |
  | `SIOCSIWESSID` | 覚えた設定で `NET_REQUEST_WIFI_CONNECT`。WPA2 → `WIFI_SECURITY_TYPE_PSK`、無し → `_NONE`、WPA → `_WPA_PSK`、WPA3 → `_SAE`、WEP → `_WEP`（ドライバーが受け付けるのは NONE と PSK だけ）。版を設定せずにパスフレーズだけがあれば PSK（Zephyr の `wifi connect` で `-k` を省いたときと同じ）。フラグ 0（wapi の `disconnect` が送る乱数の SSID）は何もしない |
  | `SIOCSIWAP` | 00:00:00:00:00:00 か ff:ff:ff:ff:ff:ff なら `NET_REQUEST_WIFI_DISCONNECT`。ほかは `-ENOTSUP` |
  | `SIOCGIWESSID`、`SIOCGIWAP`、`SIOCGIWSENS` | `NET_REQUEST_WIFI_IFACE_STATUS`（NINA に尋ねる）の SSID、BSSID、RSSI。未接続なら SSID は空、BSSID は 0、RSSI は `-ENOTCONN` |
  | `SIOCGIWMODE`、`SIOCSIWMODE` | managed（`IW_MODE_INFRA`）だけ |

  ほかの無線の ioctl は `-ENOTTY` です。接続の結果は `wapi essid` の戻りでは分からず（Zephyr 版の `wifi connect` も
  要求を出して戻る）、`ifconfig` の `RUNNING` と IP アドレス、`wapi show` の ESSID で確かめます。
- 資格情報の保存と自動再接続（`nuttx/apps/wifi/`。Zephyr 版の `src/wifi_profile.c` と Zephyr の
  `subsys/net/lib/wifi_credentials/` の写し、`wifi cred`。`nuttx/README_ja.md` の「Wi-Fi の資格情報と自動再接続」）は、
  互換層の `net_mgmt()`（`NET_REQUEST_WIFI_CONNECT_STORED`）、`net_mgmt_add_event_callback()`
  （`NET_EVENT_WIFI_CONNECT_RESULT`）、`net_if_get_wifi_sta()`、`net_if_ipv4_get_global_addr()`、`k_current_get()` を
  Zephyr 版と同じ形で使います。`nina_net.c` は接続の要求のたびに `wifi_profile_connection_allowed()` を呼び、
  再接続のスレッドからの要求を、手動の切断の後は `-ECANCELED` で断ります（Zephyr 版と同じ）。

## I2C0

| ファイル | 役割 |
|---|---|
| ボードの `src/nano_i2c.c` | I2C0 のコントローラー。Zephyr 版の `drivers/i2c_dw/i2c_dw.c` のコントローラーの部分（`i2c_dw_transfer()`、`i2c_dw_isr()`、`i2c_dw_data_ask()`・`_read()`・`_send()`、`i2c_dw_setup()`、`i2c_dw_configure()`、初期化）と DMA での読み出し（`i2c_dw_rp_dma_read()` など）を、関数・分岐・順序・レジスター値・期限・エラー値を合わせて写したもの。NuttX の `struct i2c_master_s`（`I2C_TRANSFER()`、`I2C_RESET()`）として出す |
| ボードの `src/nano_i2c.h` | `nano_i2c0_initialize()`（起動時に `nano_bringup()` が呼び、`/dev/i2c0` に登録する） |
| ボードの `src/nano_i2c_dw.h` | OS に依存しない部分: Zephyr 版の `i2c_dw.h` の定義、SCL のカウント、周波数から速さの区分、NuttX のフラグ（`I2C_M_*`）から Zephyr のフラグ（`I2C_MSG_*`）への変換、`IC_DATA_CMD` に書く語。ホストテスト `nuttx/tests/nano_i2c_dw.c` |
| ボードの `src/nano_i2c_dw_registers.h` | Zephyr 版の `i2c_dw_registers.h` の写し（アクセス関数だけ NuttX 向けに定義） |
| ボードの `src/nano_dma.c`・`nano_dma.h` | DMA 読み出しが使う RP2040 の DMA。Zephyr の `drivers/dma/dma_rpi_pico.c`（`dma_config()`・`dma_start()`・`dma_stop()`・割り込み）と、DMA API のチャネルの確保・解放（`dma_request_channel()`・`dma_release_channel()`）、`i2c_dw.c` が直接呼ぶ pico-sdk の関数（`dma_channel_is_busy()`・`dma_irqn_acknowledge_channel()`・`dma_hw->abort`）をレジスターで書いたもの |
| ボードの `src/i2c_dw_rp2040_commands.[ch]` | Zephyr 版の同名ファイルを変更せずに写したもの（DMA で送るコマンド語）。ホストテスト `nuttx/tests/i2c_dma_commands.c`（これも写し） |
| `include/nano_i2c_dma.h` | `i2c_dma` コマンド（`nuttx/apps/i2c_dma`）が読み書きする状態: DMA の有効・無効と 4 つのカウンター |

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
  確認のコールバック（このボードでは登録されず、何もしない）、i2c1。
- DMA での読み出し（`CONFIG_NANO_I2C0_DMA`、既定で有効。Zephyr 版の `CONFIG_I2C_DW_RP2040_DMA`）は
  `i2c_dw.c` の DMA の部分を同じ分岐・順序・値で写したものです:
  - 使う条件: DMA が有効（`i2c_dma mode`）、DMA の準備ができている、32〜3584 バイト
    （`I2C_DW_RP2040_DMA_MAX_READ`）、STOP で終わる、転送の最後のメッセージ、7 ビットアドレス。外れると
    `-EAGAIN` で割り込みの処理に戻ります。チャネルの確保か設定に失敗したときも、`dma_fallbacks` を
    数えて割り込みの処理に戻ります（転送はまだ始まっていません）。
  - TX のチャネルが読み出しのコマンド語（16 ビット。各語に READ、先頭は RESTART が要るときだけ RESTART、
    最後に STOP）を `IC_DATA_CMD` に書き、RX のチャネルが `IC_DATA_CMD` から 8 ビットずつ受け取ります。
    `IC_DMA_TDLR` は 8、`IC_DMA_RDLR` は 0、割り込みは STOP_DET・TX_ABRT・TX_OVER・RX_OVER・RX_UNDER・
    SCL_STUCK_LOW だけ。RX、TX の順にチャネルを始め、`IC_DMA_CR` に 3 を書きます。
  - TX の完了、RX の完了、STOP の 3 つがそろうか、エラー（I2C の異常の割り込み、DMA のバスエラー）で
    終わります。待ちは転送の期限（1 秒）を全体の期限として、残りの時間で待ちます。エラーは `-EIO`、
    期限切れは `-ETIMEDOUT`。
  - 後始末: `IC_DMA_CR` を 0、割り込みマスクを 0 にし、失敗なら I2C を abort、DMA の 2 チャネルを止めます
    （割り込みを外し、CHAN_ABORT を書いて、BUSY と CHAN_ABORT が落ちるまで 1 ms ごとに最長 100 ms 待ち、
    遅れて来た完了の割り込み（RP2040-E13）を DMA_IRQ_0・1 の両方で確認）。止まらなければコントローラーを
    隔離し（`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`、`-EIO`、チャネルは解放
    しない）、以後の転送は `-EIO` です（再起動で戻ります）。止まれば割り込みを消し、失敗なら abort の
    完了を最長 100 ms 待ちます（終わらなければ隔離して
    `<err> i2c_dw: I2C abort did not finish; controller quarantined`）。
  - DMA は Zephyr の RP2040 用ドライバーと同じく、チャネル 0 を DMA_IRQ_0、ほかを DMA_IRQ_1 に割り当て、
    チャネルの確保は待たず、停止も待ちません（待つのは上の I2C 側）。NuttX の `rp2040_dmac.c`
    （`CONFIG_RP2040_DMAC`）は DMA_IRQ_0 をすべてのチャネルで使い、確保で空きを待ち、停止で期限なしに
    待つので、切ってあります（DMA を使うのはこの読み出しだけ。NINA の SPI は DMA を使いません）。
  - `rp_dma_events` の更新は割り込みを禁止して行い（Zephyr の `atomic_or()`）、4 つのカウンターは
    バスのセマフォを持った転送だけが書きます（32 ビットで一周します）。

## LSM6DSOX（`lsm6dso`）

| ファイル | 役割 |
|---|---|
| `lsm6dso/lsm6dso.c` | ドライバー本体。Zephyr 版の `drivers/lsm6dso/lsm6dso.c` の、OS を呼ぶ部分だけを置き換えた写し（関数・分岐・順序・レジスター操作・換算・エラー値・ログの文は同じ）。末尾のデバイスツリーのインスタンス化のマクロは、ボードの値を使う `DEVICE_DT_DEFINE()` 1 つに置き換え（INT1 の `.fifo_gpio` は `<&gpio0 24 0>`） |
| `lsm6dso/lsm6dso.h` | Zephyr 版の同名ファイルの写し。カーネルのオブジェクトの型（`k_mutex`、`k_spinlock`）、バスの条件（`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` → `LSM6DSO_BUS_I2C`）、ストリームの条件（`CONFIG_SENSOR_ASYNC_API` → `CONFIG_NANO_LSM6DSO_STREAM`）、`<zephyr/types.h>`（→ `<stdint.h>` 等と `<nuttx/config.h>`）だけを置き換え |
| `lsm6dso/lsm6dso_os.[ch]` | `lsm6dso.c` が使うカーネルの機能: ミューテックスと割り込み禁止（互換層の `k_mutex`・`k_spin_lock()`）、`k_busy_wait()`、`k_msleep()`・`k_usleep()`、`k_cycle_get_32()` |
| `lsm6dso/stmemsc.h`、`stmemsc_i2c.c`、`stmemsc_mdelay.c` | Zephyr の `drivers/sensor/st/stmemsc/` の I2C の部分の写し（ST の `stmdev_ctx_t` と Zephyr の I2C の API のつなぎ） |
| `lsm6dso/st_lsm6dso_reg.c` | ST の `lsm6dso_reg.c`（`nuttx/stmemsc/`、Git 管理外）をそのままビルドするための 1 行の包み |
| `lsm6dso/lsm6dso_fifo_hw.[ch]`、`lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h` | Zephyr 版の同名ファイルを変更せずに写したもの |
| `lsm6dso/lsm6dso_stream.c`、`lsm6dso_fifo_irq.[ch]` | FIFO のストリーム（下）。Zephyr 版の同名ファイルを変更せずに写したもの |
| `lsm6dso/lsm6dso_compat.c` | 単発取得の RTIO 版（ストリームでない submit のフォールバック）。Zephyr 版の写しで、ログのインクルードとレベルの名前だけを置き換え |
| `lsm6dso/lsm6dso_fifo_decoder.[ch]` | ストリームと単発取得のバッファーのデコーダー（`get_decoder`）。変更なしの写し |
| ボードの `src/nano_lsm6dso.[ch]` | I2C0 を互換層のバスのデバイス（`i2c0`、100 kHz）にし、`lsm6dsox` を初期化する（`nano_bringup()` から） |
| ボードの `src/nano_gpio.c` | 互換層の `gpio0` のための GPIO バンク 0（入力の設定、レベルの読み出し、エッジ・レベルの割り込み）。`<nano_gpio.h>`（`nuttx/drivers/include`） |

置き換えの対応（`lsm6dso.c` の先頭にも同じ表があります）:

| Zephyr | NuttX 版 |
|---|---|
| `<zephyr/logging/log.h>`、`CONFIG_SENSOR_LOG_LEVEL`（3） | `app_log.h`（同じ行が RAM ログに入る）、レベル 3 |
| `k_mutex_init/_lock/_unlock` | `lsm6dso_os_mutex_init/_lock/_unlock`（互換層の `k_mutex` = `rmutex_t`。`K_FOREVER` の待ちはシグナルで終わってもやり直す） |
| `k_spin_lock/_unlock` | `lsm6dso_os_spin_lock/_unlock`（互換層の `k_spin_lock()`） |
| `k_busy_wait()` | `up_udelay()` |
| `k_msleep()`、`k_usleep()` | `nxsched_msleep()`、`nxsched_usleep()`（tick に切り上げて 1 tick 足す。Zephyr と同じ規則、同じ 100 µs） |
| `k_cycle_get_32()`、`k_cyc_to_ns_floor64()` | TIMER の下位 32 ビット（1 MHz）と ×1000 |
| `sys_get_le32()` | 同じ計算をファイル内に書いた |
| `CONFIG_SENSOR_ASYNC_API` | `CONFIG_NANO_LSM6DSO_STREAM`（`NANO_LSM6DSO` と一緒に常に有効。Zephyr 版と同じく、無いと `lsm6dso.c` はビルドできない） |
| デバイスツリーのインスタンス化、`POST_KERNEL`・優先度 90 | `DEVICE_DT_DEFINE(lsm6dsox, …)`、ボードの起動処理での `device_init()` |

- 初期化はボードの `nano_bringup()` が、I2C0 の初期化の直後、KV（NVS）のマウントより前に行います
  （Zephyr 版の初期化の順と同じ）。失敗してもデバイスが「準備できていない」だけで起動は続きます
  （`device_is_ready()` が false。Zephyr の `do_device_init()` と同じ）。
- 移していないもの: SPI 接続、センサーハブ（`lsm6dso_shub.c`）、トリガー（`lsm6dso_trigger.c`）、内部温度。
  どれも Zephyr 版の構成で切られていて（`CONFIG_LSM6DSO_SENSORHUB`・`_TRIGGER`・`_ENABLE_TEMP` が無効）、
  ビルドされていません。該当する `#ifdef` の部分は写しに残してあります。
- `fifo_read` の転送時間（統計の `transfer_time_ns`）は TIMER（1 µs 単位）で測ります。Zephyr 版は CPU の
  クロック（SysTick、8 ns 単位）です。

### FIFO のストリーム

`lsm6dso_stream.c`・`lsm6dso_fifo_irq.[ch]` は Zephyr 版の写しそのもの（`cmp` で一致）で、Zephyr の名前の
カーネル・RTIO・GPIO・sensor の API を互換層（下の節）から使います。動作は Zephyr 版と同じです。

- `sensor_stream()`（`SENSOR_DT_STREAM_IODEV`、トリガーは `SENSOR_TRIG_FIFO_WATERMARK` と
  `SENSOR_STREAM_DATA_INCLUDE` の 1 つだけ。ほかは `-ENOTSUP`、2 本目は `-EBUSY`）が multishot の読み出しを
  1 つ出し、`lsm6dso_submit()` が呼び出し元の文脈でワーカー（`imu_fifo_worker`）を起こします。
- ワーカーは最初の実行で FIFO を開始し（ウォーターマーク、加速度・ジャイロの BDR、タイムスタンプ 8 回に 1 つ、
  ストリームモード）、INT1 の立ち上がりエッジの割り込みを有効にしてから FIFO のしきい値を INT1 に出します。
  有効にした時点で INT1 がすでに高ければ、その場で 1 回処理を予約します。
- INT1 のエッジ → GPIO の割り込みが `k_work_submit()` でシステムワークキュー（`sysworkq`）に処理を入れる →
  そこで `stream_wakeup()` がワーカーを `k_work_reschedule_for_queue(.., K_NO_WAIT)` で起こす → ワーカーが
  FIFO の状態を読み、ウォーターマーク以上なら全ワードを 1 回の転送で読んで 1 バッチを完了にします。
  完了した multishot はその場でワーカーに戻り（`lsm6dso_submit()`）、ワーカーはすぐもう一度状態を見ます。
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

## Zephyr の API の互換層（`compat`）

`nuttx/drivers/compat/` は、写したドライバーとそれを呼ぶ側（取得サービス、校正、センサーのコマンド）が使う
Zephyr の API を、Zephyr の名前のまま最小限だけ用意したものです。インクルードパスは
`$(TOPDIR)/external/compat` です。Zephyr の sensor・RTIO・カーネルの全体は移していません。

| ヘッダー | 中身 |
|---|---|
| `zephyr/device.h` | `struct device`（名前、config、API、状態、data、初期化関数）、`device_is_ready()`、`device_init()`（Zephyr の `do_device_init()` と同じ意味）、`DEVICE_DT_GET()`、`DEVICE_DT_DEFINE()`、`DEVICE_API()` |
| `zephyr/devicetree.h` | `DT_NODELABEL()`、`DT_BUS()`。ノードは `gpio0`、`i2c0`、`lsm6dsox` だけ |
| `zephyr/kernel.h` | `k_timeout_t`（`K_NO_WAIT`、`K_FOREVER`、`K_MSEC()`、`K_SECONDS()`。単位は 100 µs の tick）、`k_mutex`、`k_sem`（`K_SEM_DEFINE`）、セマフォに対する `k_poll()`（`K_POLL_TYPE_SEM_AVAILABLE`・`K_POLL_MODE_NOTIFY_ONLY`）、ワーク（`k_work`、`k_work_delayable`、`k_work_q`、`k_work_submit()`、`k_work_cancel_sync()`、`k_work_schedule_for_queue()`、`k_work_reschedule_for_queue()`、`k_work_queue_start()`）、`K_THREAD_STACK_DEFINE()`、`K_PRIO_PREEMPT()`、`k_thread_name_set()`、システムワークキュー `k_sys_work_q`、`K_MUTEX_DEFINE()`、`K_THREAD_DEFINE()`（下）、`k_current_get()`（呼び出したスレッドの NuttX のスレッド ID。`K_THREAD_DEFINE()` の `k_tid_t` と比べられる）、`k_uptime_get()`、`k_sleep()`（下）、`k_cycle_get_32()`・`k_cycle_get_64()`・`k_cyc_to_us_floor64()`・`k_cyc_to_us_ceil64()`・`sys_clock_hw_cycles_per_sec()`（下） |
| `zephyr/spinlock.h`、`zephyr/sys/atomic.h` | `k_spin_lock()`（下）、`atomic_t` と `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`、`zephyr/rtio/work.h` | RTIO（下）と RTIO のワークキュー（`rtio_work_req_alloc()`・`_submit()`） |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`、`gpio_is_ready_dt()`、`gpio_pin_configure_dt()`（入力だけ）、`gpio_pin_interrupt_configure_dt()`、`gpio_pin_get_dt()`、コールバック（`gpio_init_callback()`・`gpio_add_callback()`・`gpio_remove_callback()`） |
| `zephyr/drivers/sensor.h` | `struct sensor_value`、チャネル・属性・トリガーの列挙（Zephyr と同じ番号）、`struct sensor_driver_api`、`sensor_attr_set()`・`_get()`、`sensor_sample_fetch()`・`_chan()`、`sensor_channel_get()`、`sensor_trigger_set()`、単位の換算。非同期の API: `struct sensor_chan_spec`、デコーダー（`struct sensor_decoder_api`、`sensor_three_axis_data` など）、`SENSOR_DT_READ_IODEV()`、`SENSOR_DT_STREAM_IODEV()`、`sensor_get_decoder()`、`sensor_stream()`、`sensor_read()`、`sensor_read_async_mempool()`、`sensor_processing_with_callback()` |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`、`sensor_clock_cycles_to_ns()`（下） |
| `zephyr/drivers/i2c.h` | `struct i2c_dt_spec`、`i2c_burst_read_dt()`、`i2c_write_dt()`、`i2c_write_read_dt()`、`i2c_configure()`、速さの区分（`I2C_SPEED_*`、`I2C_SPEED_SET()`） |
| `zephyr/sys/util.h`、`sys/__assert.h`、`sys/mpsc_lockfree.h`、`sys/mem_blocks.h`、`dsp/types.h` | `BIT()`、`ARRAY_SIZE()`、`MIN()`・`MAX()`、`IS_ENABLED()`、`CONTAINER_OF()`、`GENMASK()`・`FIELD_GET()`、`STRINGIFY()` など、`__ASSERT()`、RTIO の待ち行列とブロックのプール、`q31_t` |
| `zephyr/sys/printk.h`、`zephyr/sys/byteorder.h`、`zephyr/sys/cpu_load.h` | `snprintk()`（Zephyr 版の構成 `CONFIG_PICOLIBC=y` と同じく `snprintf()`）、`sys_put_be16/32()`・`sys_get_be16/32()`・`sys_get_le32()`、`cpu_load_get()`（ボードの `cpu_load.h`） |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()`（下） |
| `zephyr/shell/shell.h` | `shell_print()`（標準出力）・`shell_warn()`・`shell_error()`（標準エラー）。Zephyr の形のコマンドの関数を、`nuttx/apps/subcmd` の組み込みコマンドから呼ぶため（下） |
| `zephyr/net/socket.h` | `zsock_inet_pton()`・`zsock_inet_ntop()`（C ライブラリの `inet_pton()`・`inet_ntop()`）、`zsock_getaddrinfo()`・`zsock_freeaddrinfo()`（NINA の名前解決、`nina_getaddrinfo()`。エラーは NuttX の `EAI_*`）、`zsock_socket()`（NuttX の `socket()` に `NINA_SO_CALLER_POLLS` を付ける。付けられなければ閉じて -1）、`zsock_sendto()`・`zsock_close()`（NuttX の `sendto()`・`close()`）、`zsock_recv()`（先に `ioctl(FIONREAD)` で NINA にその場で尋ね、あれば `recv(MSG_DONTWAIT)` で読む。`ZSOCK_MSG_DONTWAIT` のときは何も無ければ `EAGAIN`、無いときは 20 ms ごとに尋ね直す。Zephyr 版の NINA の `recv()`（`do_recv()`・`udp_recv()`）が呼ばれるたびに尋ね、待つときは 20 ms ごとに尋ね直すのに合わせるため。`SO_RCVTIMEO`・`O_NONBLOCK` は見ない）、`ZSOCK_MSG_DONTWAIT`、`zsock_connect()`・`zsock_send()`・`zsock_sendmsg()`（`connect()`・`send()`・`sendmsg()`）、`zsock_poll()`・`struct zsock_pollfd`・`ZSOCK_POLLIN` など（`poll()` と `struct pollfd`。NINA への尋ね方は下） |
| `zephyr/sys/clock.h` | `sys_clock_gettime()`・`sys_clock_settime()`（`SYS_CLOCK_REALTIME`。NuttX の `clock_gettime()`・`clock_settime()` の `CLOCK_REALTIME`。0 か `-EINVAL`）、`NSEC_PER_USEC`・`USEC_PER_SEC` |
| `zephyr/net/net_if.h` | `struct net_if`（NuttX のネットワークデバイス 1 つ）、`net_if_get_default()`・`net_if_get_wifi_sta()`、IPv4 のアドレス・ネットマスク・ゲートウェイ（`net_if_ipv4_addr_add()`・`_rm()`・`net_if_ipv4_get_global_addr()`・`net_if_ipv4_set_netmask_by_addr()`・`net_if_ipv4_set_gw()`・`net_if_ipv4_get_gw()`）、`net_if_dormant_on()`・`_off()`、`net_if_is_admin_up()`、`net_if_set_link_addr()`（下） |
| `zephyr/net/net_mgmt.h` | `net_mgmt()`（要求）、`struct net_mgmt_event_callback`、`net_mgmt_init_event_callback()`・`net_mgmt_add_event_callback()`・`_del_event_callback()`、`net_mgmt_event_notify_with_info()`（下） |
| `zephyr/net/wifi_mgmt.h` | Wi-Fi の型（セキュリティ、帯域、MFP、状態、モード。Zephyr と同じ値）、`struct wifi_scan_params`・`wifi_scan_result`・`wifi_connect_req_params`・`wifi_iface_status`・`wifi_status`、`struct wifi_mgmt_ops`、オフロードのインターフェースの API と `NET_DEVICE_DT_OFFLOAD_DEFINE()`、要求 `NET_REQUEST_WIFI_SCAN`・`_CONNECT`・`_DISCONNECT`・`_IFACE_STATUS`・`_CONNECT_STORED`（`CONFIG_NANO_WIFI_CREDENTIALS` のとき）、イベント `NET_EVENT_WIFI_SCAN_RESULT`・`_SCAN_DONE`・`_CONNECT_RESULT`・`_DISCONNECT_RESULT`、`wifi_mgmt_raise_connect_result_event()`・`_disconnect_result_event()`（下）、`wifi_security_txt()`・`wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | Zephyr の同名のヘッダーの写し: `struct wifi_credentials_header`・`_personal`、`WIFI_CREDENTIALS_FLAG_*`、`wifi_credentials_set_personal_struct()`・`_get_by_ssid_personal_struct()`・`_delete_by_ssid()`・`_for_each_ssid()`・`_is_empty()` など。設定の値（`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` など）は `nuttx/apps/wifi/Kconfig` の `NANO_WIFI_CREDENTIALS_*` から。NuttX 版だけの `wifi_credentials_init()`（init が呼ぶ。Zephyr の `SYS_INIT()`）。関数は `nuttx/apps/wifi/` |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()`（`_2g`・`_5g`・`_6g`）、`net_bytes_from_str()`（Zephyr では `<zephyr/net/net_ip.h>`） |
| `zephyr/settings/settings.h` | `settings_subsys_init()`、`settings_save_one()`、`settings_delete()`、`settings_load_one()`。名前 1 つずつを NuttX の NVS（`/dev/config`）の同じ名前の項目で読み書きする（実装は `nuttx/apps/wifi/settings_nvs.c`）。部分木の読み込みとハンドラーは無い |
| `zephyr_compat.h` | `zephyr_compat_initialize()`（ボードの起動処理が呼ぶ。下）、`K_THREAD_DEFINE_START()`（init が呼ぶ） |

| ソース | 中身 |
|---|---|
| `zephyr_kernel.c` | NuttX の上の OS 部分: `k_mutex`（`rmutex_t`）、`k_sem`（`sem_t`、上限は `k_sem_give()` で見る）、`k_poll()`、ワークキューのスレッド（静的なスタックのカーネルスレッド）、遅延ワークのタイマー（wdog）、スレッド名、`K_THREAD_DEFINE()` のスレッドの起動、`k_uptime_get()`・`k_sleep()`、`zephyr_compat_initialize()` |
| `zephyr_work.c` | ワークとワークキュー。Zephyr の `kernel/work.c` の使っている部分の写し（OS に依存しない） |
| `zephyr_rtio.c`、`zephyr_rtio_workq.c`、`zephyr_mem_blocks.c` | RTIO の executor（`subsys/rtio/rtio_executor.c`）、RTIO のワークキュー（`rtio_workq.c`）、ブロックのプール（`lib/mem_blocks`） |
| `zephyr_gpio.c` | `gpio0` のデバイス（Zephyr の `gpio.h` の処理と `gpio_rpi_pico.c`）。ボードの `<nano_gpio.h>` の上 |
| `zephyr_sensor_clock.c` | センサーの時計。ボードの TIMER（`nano_mono_us()`） |
| `sensor_default_rtio.c` | Zephyr の `drivers/sensor/default_rtio_sensor.c` の写し（ログだけ置き換え）。センサーの iodev の API、単発取得のフォールバック、既定のデコーダー |
| `zephyr_device.c`、`zephyr_i2c.c` | デバイスの初期化と I2C |
| `zephyr_hwinfo.c` | `hwinfo_get_device_id()`。ボードのチップ ID（`nano_chip_id()`） |
| `zephyr_net_if.c`、`zephyr_net_mgmt.c`、`zephyr_wifi_mgmt.c` | インターフェース（NuttX のネットワークデバイスの値）、`net_mgmt` のイベントの待ち行列とスレッド、Wi-Fi の要求の処理（Zephyr の `subsys/net/l2/wifi/wifi_mgmt.c` の検査の写し）とイベント、`NET_REQUEST_WIFI_CONNECT_STORED`（同じファイルの `connect_stored_command()` の写し。`CONFIG_NANO_WIFI_CREDENTIALS` のとき）。`CONFIG_NANO_NINA_NET` のときだけ |
| `zephyr_wifi_utils.c` | `wifi_security_txt()`・`wifi_band_txt()`（Zephyr の `wifi_mgmt.c`）、チャネルの検査（`wifi_utils.c`）、`net_bytes_from_str()`（`subsys/net/ip/utils.c`）の写し。`wifi cred` が使う。OS に依存しない。`CONFIG_NANO_NINA_NET` のときだけ |

- **ワーク**: フラグ（RUNNING・CANCELING・QUEUED・DELAYED）と戻り値は `kernel/work.c` と同じです。
  `submit` はキュー済みなら何もしない（0）、実行中なら同じキューに入れ直す（2）。`schedule` は遅延中・
  キュー済みなら何もしない（実行中なら予約できる）、`reschedule` は遅延を止めてから予約し直す
  （`K_NO_WAIT` ならその場でキューに入れる）。`k_work_cancel_sync()` はキューから外し、実行中なら終わるまで
  待ちます（待てるのは 1 スレッドだけ。Zephyr は複数）。キューのスレッドは処理の後、ほかに待ちがあれば
  `sched_yield()` します（Zephyr の `k_yield()`）。遅延のタイマーは wdog で、時間は Zephyr と同じく
  「今の tick + tick 数 + 1」で切れます。移していないもの: flush、drain、plug、キューの停止、ワークの
  タイムアウト、`k_work_cancel()`、`k_work_cancel_delayable()`。
- **スピンロック**: Zephyr の 1 CPU の `k_spin_lock()` は割り込みを禁止し、その間はほかのスレッドに切り替わり
  ません（`k_sem_give()` などで起きたスレッドは解放のときに動く）。NuttX は割り込み禁止中でも
  `nxsem_post()` で起きたスレッドにすぐ切り替えるので、互換層の `k_spin_lock()` は `sched_lock()` と
  `up_irq_save()` にしています（割り込みハンドラーの中では `sched_lock()` は何もしない）。`atomic_*()` も
  これで守ります（Cortex-M0+ に排他ロード・ストアは無く、Zephyr の ARMv6-M も割り込み禁止で実装）。
- **NuttX の名前との衝突**: NuttX の `<nuttx/atomic.h>`（多くのカーネルのヘッダーが読む）にも `atomic_t` と
  `atomic_set()` などがあり、意味が違います。互換層の `<zephyr/sys/atomic.h>` はそれを先に読んでから、
  そのファイルの中で名前を引き取ります（`atomic_t` は `zephyr_atomic_t`、NuttX のマクロは `#undef`）。
  両方を使うファイルは NuttX のヘッダーを先に読みます。ボードの起動処理は Zephyr のヘッダーを読まず、
  `zephyr_compat.h` だけを読みます。
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
  優先度 0 → 118、スタック 1024）で、投入順に実行します。単発取得のフォールバック（`lsm6dso_compat.c`）が
  使います。NuttX はタスク名の空白を `_` にするので、`ps` では `RTIO_WQ_0` と出ます。
- **GPIO**: ポートは `gpio0`（GPIO0〜29）だけで、入力だけです（出力は `-ENOTSUP`）。ピンの設定、割り込み
  （エッジ・レベル、論理・物理）、レベルの読み出し、コールバックは Zephyr の `gpio.h` と `gpio_rpi_pico.c`
  の写しで、ボードの `src/nano_gpio.c` が pico-sdk の `gpio_set_pulls()`・`gpio_set_function()`・
  `gpio_set_dir()`・`gpio_set_irq_enabled()` と同じレジスター操作をします（イベントを有効・無効にする前に
  `INTR` のそのピンのイベントを消す）。割り込みは NuttX の `rp2040_gpio` のバンクの割り込みから来ます
  （有効なイベントを消してからピンのハンドラーを呼ぶ）。
- **センサーの時計**: Zephyr 版はシステムクロック（SysTick、125 MHz、`k_cycle_get_64()`）、NuttX 版はボードの
  TIMER（水晶の 1 MHz）です。ストリームのバッチの時刻と単発取得の時刻は 1 µs 単位になります。
- **起動**: `zephyr_compat_initialize()` を、ボードの起動処理が DMA・I2C0・LSM6DSOX より前に呼び、
  システムワークキュー（`sysworkq`、優先度 119、スタック 1024）、RTIO のワークキュー、`gpio0` を用意します
  （Zephyr がそれらを `POST_KERNEL` でセンサーより先に用意するのと同じ順）。`imu_fifo_worker` は LSM6DSOX の
  初期化（`lsm6dso_stream_init()`）が作ります。
- **静的なスレッド**（`K_THREAD_DEFINE()`）: Zephyr と同じ引数で、静的なスタック、`K_PRIO_PREEMPT(prio)` の
  優先度、名前はトークンそのもの（`imu_stream_tid`）のスレッドを定義します。Zephyr はこれらを `main()` の前に
  自動で始めますが、NuttX にはその一覧が無いので、init が `main()` 相当の処理の最初に
  `K_THREAD_DEFINE_START(name)`（`zephyr_compat.h`）で 1 本ずつ始めます。スレッドは NuttX のカーネル
  スレッド（`kthread_create_with_stack()`）で、入口の関数が戻ると終わります。`options` と `delay` は 0 だけ
  受け付けます（ほかはビルドエラー）。
- **時間**: `k_uptime_get()` は Zephyr の `k_ticks_to_ms_floor64(k_uptime_ticks())`（tick 数を ms に切り捨て）
  と同じ値です。NuttX の tick はボードの TIMER を 100 µs で割ったもの（tickless のタイマー）なので、TIMER を
  1 回読んで 1000 で割ります（1000 が tick の倍数なので同じ値。ホストテストで確認）。64 ビットで、32 ビットの
  `clock_systime_ticks()` のような約 5 日での一周はありません。`k_sleep()` は `K_NO_WAIT` で `sched_yield()`、
  それ以外は `nxsched_ticksleep()` で「今の tick + tick 数 + 1」まで眠ります（Zephyr の `z_add_timeout()` と
  同じ切り上げ）。`k_wakeup()` が無いので戻り値は常に 0 です。
- **`hwinfo_get_device_id()`**: Zephyr 版の RP2040 のドライバー（`hwinfo_rpi_pico.c`、
  `CONFIG_HWINFO_RPI_PICO_CHIP_ID` なし）と同じく、外付け flash の 64 ビットの unique ID（コマンド 0x4B）を
  flash が送る順のまま返します（最大 8 バイト）。読み出しはボードが起動時に 1 回行い（`nano_chipid.c`）、
  ここではその写しを返します。ボードがまだ読んでいなければ `-EAGAIN` です（Zephyr のドライバーは失敗しない）。
- **シェル**: `shell_print()`・`shell_warn()`・`shell_error()` は改行を付けて標準出力・標準エラー・標準エラーに
  書きます（`kv` などほかのコマンドと同じ出力先。Zephyr は警告を黄色、エラーを赤で出す）。`sh` の引数は評価するだけで使いません。コマンドの登録のマクロ
  （`SHELL_CMD_REGISTER()` など）は無く、コマンドの表は `nuttx/apps/subcmd` で書きます。
- **ネットワーク**: `zsock_inet_pton()`・`zsock_inet_ntop()` は C ライブラリの関数です。ネットワークが無効な
  間は、`imu` が `CONFIG_LIBC_IPv4_ADDRCONV` を選んで IPv4 の変換を入れます。ソケットは NINA の usrsock
  で、`zsock_socket()`・`zsock_connect()`・`zsock_send()`・`zsock_sendmsg()`・`zsock_sendto()`・`zsock_close()` は
  NuttX の同じ名前の関数です（ブロッキングのソケットの待ちと期限は NINA のドライバーのもの）。
- **`zsock_poll()`**: Zephyr 版の NINA の `poll()`（`nina_socket.c` の `do_poll()`）と同じ動きです。呼び出した
  スレッドで、ソケットごとに `getsockopt(NINA_SOL, NINA_SO_POLL)`（NINA の poll 命令を 1 回）で尋ね、求めた
  `ZSOCK_POLLIN`・`ZSOCK_POLLOUT` と、`ZSOCK_POLLERR`（NINA のエラーフラグ）・`ZSOCK_POLLHUP`（NINA 側のソケットが
  まだ無い TCP）を返します。どれも無ければ `k_sleep(K_MSEC(20))` の後に尋ね直し、`k_uptime_get()` で待ち時間
  （-1 は無期限）が過ぎたら 0 を返します。NINA の poll の失敗は -1 とその errno です。NINA のソケットでない
  ファイル記述子は NuttX の `poll()` を待たずに 1 回呼んだ結果（不正なものは `ZSOCK_POLLNVAL`）で、これも
  Zephyr 版と同じです。NuttX の `poll()` と `nina_event` は使いません。`imu_raw_mqtt.c` の受信の確認（100 ms
  ごと、待ち時間 0）と CONNACK の待ち（5000 ms）が使います。
- **サイクルカウンター**: Zephyr 版の `k_cycle_get_*()` は 125 MHz の SysTick の数（負荷で遅れる）で、
  NuttX 版はボードの TIMER（`nano_mono_us()`、水晶の 1 MHz。センサーの時計と同じ）です。1 サイクルが 1 µs
  なので、µs への換算（`k_cyc_to_us_floor64()`・`_ceil64()`）はそのままの値です。`k_cycle_get_32()` は下位
  32 ビットで約 71.6 分で一周します（Zephyr 版は約 34 秒）。`imu_raw_mqtt.c` がキューのロックの待ちと送信の
  時間を計るのに使います。
- **インターフェース**（`net_if.h`）: インターフェースは NINA の `wlan0` 1 つで、`net_if_get_default()` と
  `net_if_get_wifi_sta()` はどちらもそれを返します（登録前は NULL）。アドレスは 1 つ（足すと置き換え）で、
  足したアドレスは常に「優先」（IPv4 の重複検出は Zephyr 版の構成でも無い）なので、
  `net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)` はアドレスがある間それを返します。値の読み書きは
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
  インターフェースより先にスレッドを始めます（Zephyr の `net_init()` と同じ順）。
- **Wi-Fi の要求**（`zephyr_wifi_mgmt.c`）: Zephyr の `wifi_mgmt.c` と同じく、インターフェースの API
  （`NET_DEVICE_DT_OFFLOAD_DEFINE()` で名前を付けたもの）にドライバーの操作が無ければ `-ENOTSUP`、
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
