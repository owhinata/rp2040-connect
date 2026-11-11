# Wi-Fi モニターによる TCP ACK 調査手順

Arduino Nano RP2040 Connect の NINA-W102 から PC に TCP/MQTT で送信するとき、
PC が送信した TCP ACK をアクセスポイント（AP）から NINA まで追跡する手順です。
PC 側の TCP キャプチャと、別の Wi-Fi アダプターによる 802.11 キャプチャを
同時に取得して照合します。

通常の PC 側キャプチャだけでは、PC が ACK を送ったことは分かっても、
AP が NINA 宛てに送信したか、NINA が受信したかは分かりません。無線側の
キャプチャにも見落としがあるため、記録がないことだけで未送信・未受信を
断定しないでください。

## 準備するもの

- RP2040 + NINA-W102 と、同じネットワークに接続する PC。MQTT または TCP の
  再現手段と、PC 側の待ち受けアプリケーションも必要です。
- **PC の通常通信とは別の**、Linux でモニターモードを使用できる Wi-Fi
  アダプター。NINA と同じ周波数帯・チャンネルに対応し、データフレームと
  802.11 制御フレームを取得できるものを使います。
- `iw`、`ip`、`tcpdump`、Wireshark / `tshark`。インストールとアダプターの
  モード切り替えには管理者権限が必要です。
- 復号する場合は、調査対象ネットワークの SSID と認証情報。WPA2-Personal の
  場合は NINA の接続時の 4-way ハンドシェイクもキャプチャします。

Ubuntu の例：

```bash
sudo apt update
sudo apt install iw iproute2 tcpdump wireshark tshark
```

今回使用した UGREEN CM495 は USB ID `0bda:b832`、ドライバー
`rtw89_8852bu` でした。チャンネル 2 のモニターモードで、NINA の送受信データと
802.11 ACK を取得できました。ただし、**一部の NINA のデータフレームは
PC に届いていてもモニターには記録されませんでした**。モニターモード対応と、
全フレームを確実に捕捉できることは別です。

キャプチャファイルには、同じチャンネルの他の端末の情報や、接続先情報などが
含まれることがあります。認証情報や復号後の通信内容を含むファイルは、共有先と
保存場所を限定してください。

## 機器とチャンネルの確認

例では PC の通常通信を `wlo1`、モニター用 USB アダプターを
`wlx020000000001`、NINA を `192.168.1.31`、PC を `192.168.1.27`、
TCP の待ち受けポートを `18830` とします。環境に合わせて読み替えてください。
通常通信のインターフェースをモニターモードに切り替えないよう注意します。

```bash
lsusb
ip -br link
ip -4 addr show wlo1
sudo iw dev
```

アダプターの物理デバイスにモニターモードがあるか、`iw list` の該当する
`Wiphy` の `Supported interface modes` を確認します。`monitor` が表示されても、
必要な種類のフレームを実際に取得できるかは後で確認します。

```bash
sudo iw list
nmcli -f CHAN,FREQ,SSID,BSSID device wifi list ifname wlo1
```

SSID と BSSID から NINA が接続する AP のチャンネルを選びます。今回の AP は
2.4 GHz のチャンネル 2 でした。PC が 5 GHz に接続している場合も、モニター側は
**NINA と AP 間の 2.4 GHz チャンネル**に固定します。

RP2040 の USB シリアルコンソールで `wifi status` と `net iface` を実行し、
接続状態、AP の BSSID、NINA の IP アドレスと MAC アドレスを確認します。
実験時の NINA の MAC アドレスは `02:00:00:00:00:02` でした。

## モニター用アダプターの設定

USB アダプターを NINA の近くに配置します。次の例はアダプター自身を
モニターモードに切り替える方法です。インターフェース名とチャンネルは
確認した値に変更してください。

```bash
MON=wlx020000000001
CHANNEL=2

sudo ip link set "$MON" down
sudo iw dev "$MON" set type monitor
sudo ip link set "$MON" up
sudo iw dev "$MON" set channel "$CHANNEL"
sudo iw dev "$MON" info
```

`type monitor` と正しいチャンネルになったことを確認します。NetworkManager が
設定を戻す場合は、対象の USB アダプターを一時的に管理対象から外して試し、
作業後に元へ戻します。`iw` が使えない環境でも、今回の Linux ドライバーでは
`iwconfig` によるモード切り替えが可能でした。

試験終了後は元のモードに戻します。NetworkManager の管理対象から外した場合は、
その設定も元に戻してください。

```bash
sudo ip link set "$MON" down
sudo iw dev "$MON" set type managed
sudo ip link set "$MON" up
```

## ハンドシェイクと通信の同時キャプチャ

1. まず無線側のキャプチャを開始します。WPA2 のハンドシェイクと制御フレームを
   取りこぼさないよう、この段階では TCP ポートなどのキャプチャフィルターを
   指定しません。

   ```bash
   sudo tcpdump -i "$MON" -s 0 -U -w wifi-radio.pcap
   ```

2. キャプチャが始まってから NINA を Wi-Fi に再接続します。RP2040 を再起動する
   場合、使用するファームウェアで保存済み Wi-Fi 設定への自動接続が有効である
   ことを事前に確認してください。再接続後、`wifi status` と `net iface` で
   接続・アドレスを確認します。
3. 別のターミナルで PC 側キャプチャを開始します。ここでは調査対象の TCP ポート
   に絞れます。MQTT ブローカーや試験用受信プログラムの待ち受けポートを指定します。

   ```bash
   sudo tcpdump -i wlo1 -s 0 -U -w wifi-pc.pcap 'tcp port 18830'
   ```

4. PC 側の受信プログラムを起動してから RP2040 で TCP/MQTT 送信を実行します。
   送信サイズ、周期、IMU の ODR、実行時間、PC と NINA の省電力設定を記録します。
   送信停止の最大時間や、FIFO オーバーランの有無も記録します。
5. 試験後、それぞれの `tcpdump` を `Ctrl-C` で停止し、キャプチャ件数とカーネルの
   ドロップ数を確認します。**カーネルのドロップが 0 でも、無線受信やアダプター
   内部での見落としがないことは保証されません**。
6. `wifi-radio.pcap` を Wireshark で開き、次の表示フィルターで NINA と AP の
   EAPOL フレームを確認します。

   ```text
   eapol && wlan.addr == 02:00:00:00:00:02
   ```

   4-way ハンドシェイクのメッセージ 1～4 が取得できているか確認します。
   見つからない場合、正しいチャンネルを監視しているか、キャプチャ開始後に NINA が
   再接続したか確認し、取得をやり直します。

## WPA2-Personal の復号

以下は今回成功した WPA2-Personal の例です。認証方式によって必要な鍵情報は
異なり、特に WPA3-SAE や企業向け認証ではパスフレーズだけで復号できるとは
限りません。

1. Wireshark で個人用の一時的な設定プロファイルを作成し、選択します。
2. `Edit` → `Preferences` → `Protocols` → `IEEE 802.11` で
   `Enable decryption` を有効にします。
3. 同画面の `Decryption keys` を開き、鍵の種類 `wpa-pwd` で
   `パスフレーズ:SSID` を追加します。シェルのコマンドラインや履歴には
   パスフレーズを書かないでください。SSID も NINA が実際に接続したものを使います。
4. ハンドシェイクを含む `wifi-radio.pcap` を最初から開き直します。表示フィルター
   `tcp.port == 18830` で、NINA と PC 間の TCP が解読されることを確認します。
   EAPOL が見えていても TCP が表示されなければ、SSID・認証情報・ハンドシェイク・
   接続先 AP が正しいか確認します。
5. 作業後、Wireshark の一時プロファイルに保存した鍵を削除します。pcap には
   ハンドシェイクが含まれるので、鍵を削除してもファイルの共有には注意してください。

`tshark` を使う場合も、同じ一時プロファイルを使用できます。以下の `-C` の値は
Wireshark で作成したプロファイル名です。`tcp.ack_raw` は絶対 ACK 番号で、
Wireshark の相対番号表示の設定に依存しません。

```bash
tshark -C wifi-debug -r wifi-radio.pcap \
  -Y 'tcp.port == 18830' \
  -T fields -e frame.number -e frame.time_epoch \
  -e ip.src -e ip.dst -e tcp.seq_raw -e tcp.ack_raw -e tcp.len \
  -e wlan.seq -e wlan.qos.tid -e wlan.fc.retry
```

同様の TCP フィールドを `wifi-pc.pcap` から表示すると、PC 側と無線側を照合できます。
TCP ACK 番号を比べるときは、両方で **raw の絶対番号**を使います。表示列やフィルターの
詳細は Wireshark のバージョンにより異なる場合があります。

## 正常時のシーケンス

```text
NINA                    AP                       PC
  |--- TCP データ ------>|----------------------->|
  |                      |<------ TCP ACK --------|
  |<-- ACK を含む 802.11 データフレーム ---------|
  |--- 802.11 ACK/Block ACK ---> AP              |
  |--- 次の TCP データ -->|----------------------->|
```

- TCP ACK の `Acknowledgment Number` は「次に受信したい TCP バイト番号」です。
  たとえばデータが `seq 1000:2436` なら、欠落なく受け取った側は通常 `ack 2436`
  以上を返します。TCP は累積 ACK なので、複数データ分を一つの ACK で確認することも
  あります。
- TCP ACK は暗号化された **802.11 データフレームの中**にあります。一方、802.11
  ACK / Block ACK は無線リンク上の受信確認であり、TCP ACK とは別物です。
- Block ACK では複数フレームをまとめて確認します。制御フレームには短いものもあり、
  モニターの見落としもあるため、時刻や宛先だけで対応を断定しないでください。
- PC 側キャプチャは GRO 等で複数 TCP セグメントがまとめられる場合があります。
  802.11 フレーム数と PC 側の行数をそのまま 1 対 1 で比べず、TCP の絶対シーケンス
  番号と ACK 番号で対応付けます。

## TCP ACK の確認方法

1. `wifi-pc.pcap` で `tcp.port == 18830` を表示し、NINA → PC の送信が長く
   止まった箇所を探します。停止直前のデータの終了シーケンス番号、PC → NINA の
   `tcp.ack_raw`、その後の NINA の TCP 再送と、PC の再 ACK を控えます。
2. 復号した `wifi-radio.pcap` で、控えた ACK 番号を検索します。例えば
   `tcp.ack_raw == 2436`。`ip.src` / `ip.dst` と TCP ポートも合わせて
   対象の接続であることを確認します。
3. 対象の 802.11 データフレームの `wlan.ra` / `wlan.ta`、`wlan.seq`、
   `wlan.qos.tid`、`wlan.fc.retry`、時刻を確認します。PC 側で送られた時刻と、
   無線側での観測時刻を比べ、最初の ACK なのか、TCP 再送に対する再 ACK なのか
   を区別します。
4. 同じ付近の NINA → PC データが無線側にも記録されているか確認します。
   PC には届いたデータが無線側にない場合、その区間でモニターが見落としていた
   ことが分かります。USB アダプターやドライバーによって記録時刻にも遅延・ずれが
   あり得るため、時刻だけでフレームを対応付けないでください。
5. NINA 側にも受信入口や TCP 処理の診断ログがある場合、同じ絶対 ACK 番号で
   照合します。無線側で復号できたことだけでは NINA のドライバーや lwIP まで
   届いたことは証明できません。

`wlan.seq` は **TCP シーケンス番号とは別の 802.11 側の番号**です。TID や送信元を
揃えて比較します。番号が連続していても、それだけで AP が対象フレームを
送信しなかったとは証明できません。

## 参考リンク

- [Linux Wireless: モニターモードとチャンネル設定](https://wireless.docs.kernel.org/en/latest/en/users/documentation/iw/vif.html)
- [Wireshark: WLAN キャプチャ](https://wiki.wireshark.org/CaptureSetup/WLAN)
- [Wireshark: 802.11 の復号](https://wiki.wireshark.org/HowToDecrypt802.11)
