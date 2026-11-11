#!/usr/bin/env bash
# Switch one NetworkManager Wi-Fi adapter between a temporary AP and station mode.
set -Eeuo pipefail

usage() {
    cat <<'HELP'
使い方:
  wifi_ap.sh ap <interface> [--ssid <SSID>] [--channel <1-13>] [--password-stdin]
  wifi_ap.sh station <interface>

ap       指定したアダプターで一時的な 2.4 GHz WPA2 AP を起動します。
station  このスクリプトが作成した AP を停止し、Station 用の状態に戻します。

SSID の既定値は rp2040-test、チャンネルの既定値は 6 です。
パスワードは非表示で入力します。--password-stdin を指定すると標準入力から 1 行読みます。
NetworkManager、nmcli、uuidgen、および共有接続用の dnsmasq が必要です。
HELP
}

fail() { printf 'エラー: %s\n' "$*" >&2; exit 1; }

if [[ ${1:-} == -h || ${1:-} == --help ]]; then usage; exit 0; fi
[[ $# -ge 2 ]] || { usage >&2; exit 2; }
mode=$1
interface=$2
shift 2
[[ $interface =~ ^[A-Za-z0-9_.-]{1,15}$ ]] || fail 'インターフェース名が不正です。'
[[ $mode == ap || $mode == station ]] || fail 'モードには ap または station を指定してください。'
command -v nmcli >/dev/null || fail 'nmcli が見つかりません。NetworkManager を用意してください。'
command -v uuidgen >/dev/null || fail 'uuidgen が見つかりません。'

profile_name="rp2040-wifi-ap-$interface"
profile_uuid=$(uuidgen --sha1 --namespace @url --name "rp2040-connect/wifi-ap/$interface")

profile_exists() {
    local uuids
    uuids=$(nmcli -g UUID connection show) || fail 'NetworkManager の接続一覧を取得できません。'
    [[ $'\n'${uuids}$'\n' == *$'\n'"$profile_uuid"$'\n'* ]]
}

verify_profile() {
    local actual expected
    actual=$(nmcli -g connection.id,connection.type,connection.interface-name,802-11-wireless.mode \
        connection show uuid "$profile_uuid")
    expected=$(printf '%s\n802-11-wireless\n%s\nap' "$profile_name" "$interface")
    [[ $actual == "$expected" ]] || fail '対象 UUID の設定がこのスクリプトの AP と一致しません。'
}

if [[ $mode == station ]]; then
    [[ $# -eq 0 ]] || fail 'station に追加オプションは指定できません。'
    if ! profile_exists; then
        printf '%s\n' 'このスクリプトが作成した AP 設定はありません。'
        exit 0
    fi
    verify_profile
    active=$(nmcli -g UUID connection show --active)
    if [[ $'\n'${active}$'\n' == *$'\n'"$profile_uuid"$'\n'* ]]; then
        nmcli --wait 30 connection down uuid "$profile_uuid"
    fi
    nmcli --wait 30 connection delete uuid "$profile_uuid"
    printf 'AP を停止し、%s を Station 用の状態に戻しました。\n' "$interface"
    printf '%s\n' '既存の Wi-Fi への接続が必要な場合は NetworkManager から選択してください。'
    exit 0
fi

ssid=rp2040-test
channel=6
password_stdin=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --ssid) [[ $# -ge 2 ]] || fail '--ssid に値が必要です。'; ssid=$2; shift 2 ;;
        --channel) [[ $# -ge 2 ]] || fail '--channel に値が必要です。'; channel=$2; shift 2 ;;
        --password-stdin) password_stdin=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "不明なオプションです: $1" ;;
    esac
done
[[ $channel =~ ^([1-9]|1[0-3])$ ]] || fail 'チャンネルには 1～13 を指定してください。'
LC_ALL=C
export LC_ALL
[[ ${#ssid} -ge 1 && ${#ssid} -le 32 ]] || fail 'SSID は UTF-8 で 1～32 バイトにしてください。'
if profile_exists; then
    verify_profile
    fail 'このアダプターの AP 設定は既にあります。先に station を実行してください。'
fi

if "$password_stdin"; then
    IFS= read -r password || [[ -n ${password:-} ]] || fail 'パスワードを読み取れません。'
else
    IFS= read -r -s -p 'AP の WPA2 パスワード（8～63 文字）: ' password || fail 'パスワードを読み取れません。'
    printf '\n' >&2
fi
[[ ${#password} -ge 8 && ${#password} -le 63 && $password =~ ^[[:print:]]+$ ]] || \
    fail 'パスワードには 8～63 文字の半角英数字・記号を指定してください。'

umask 077
secret_dir=$(mktemp -d "${TMPDIR:-/tmp}/rp2040-wifi-ap.XXXXXXXX")
created=false
cleanup() {
    local rc=$?
    trap - EXIT
    rm -rf -- "$secret_dir"
    if (( rc != 0 )) && "$created"; then
        nmcli --wait 30 connection delete uuid "$profile_uuid" >/dev/null 2>&1 || true
    fi
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '802-11-wireless-security.psk:%s\n' "$password" > "$secret_dir/password"
unset password

# An in-memory profile and a non-persistent secret leave no saved hotspot credentials.
nmcli --wait 30 connection add type wifi ifname "$interface" \
    con-name "$profile_name" autoconnect no save no ssid "$ssid" \
    connection.uuid "$profile_uuid" 802-11-wireless.mode ap \
    802-11-wireless.band bg 802-11-wireless.channel "$channel" \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk-flags 2 ipv4.method shared ipv6.method disabled
created=true
nmcli --wait 30 connection up uuid "$profile_uuid" ifname "$interface" \
    passwd-file "$secret_dir/password"
address=$(nmcli -g IP4.ADDRESS device show "$interface")
printf 'AP を起動しました: interface=%s SSID=%s channel=%s\n' "$interface" "$ssid" "$channel"
printf 'PC の AP 側 IPv4: %s\n' "${address:-未取得}"
printf '終了: %s station %s\n' "${0##*/}" "$interface"
