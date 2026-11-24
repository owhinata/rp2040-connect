#!/usr/bin/env bash
# Prepare the Pico SDK + FreeRTOS workspace under freertos/ without sudo.
# Source it:
#   source freertos/script/setup.sh
# Each step runs only when its result is missing, so sourcing it again is safe.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'This script must be sourced: source freertos/script/setup.sh\n' >&2
  exit 1
fi

_freertos_setup() {
  # ---- Pinned versions (change them here only) ------------------------------
  # Raspberry Pi Pico SDK.  Of its submodules only TinyUSB (the USB console)
  # is fetched.
  local pico_sdk_tag=2.3.1
  local pico_sdk_commit=079c6f39023649b154152db30f1d781e884879bc
  local -a pico_sdk_submodules=(lib/tinyusb)
  # FreeRTOS kernel.  The RP2040 port is in the kernel's own tree
  # (portable/ThirdParty/GCC/RP2040); no submodule is needed.
  local kernel_tag=V11.3.3
  local kernel_commit=89592801dd8b73329a7583719b7e3567b50910e5
  local kernel_port=portable/ThirdParty/GCC/RP2040
  # Arm GNU Toolchain (GCC 15.2), the newest release on developer.arm.com
  # when this was written (2026-11-16).  The SHA-256 is the one Arm publishes
  # next to the archive (.sha256asc).
  local arm_gnu_version=15.2.rel1
  local arm_gnu_sha256=597893282ac8c6ab1a4073977f2362990184599643b4c5ee34870a8215783a16
  # clang-format, the formatter of freertos/script/format.sh (PyPI's build of
  # LLVM's clang-format, in a Python venv).
  local clang_format_version=18.1.3
  # LSM6DSO register driver: lsm6dso-pid v3.2.2, the driver submodule of
  # STMems_Standard_C_drivers v2.13.2 (the stmemsc version in Zephyr's hal_st;
  # the same files as nuttx/script/setup.sh fetches).  Upstream files use
  # CRLF; they are stored with LF like hal_st.
  local lsm6dso_commit=f55167674138aca789c70a2970187c6322192dde
  local lsm6dso_c_sha256_crlf=0ba42733ac6242bebd4b404d6c7e45b7420f6f15cbdfc4d34b0ae966698271a1
  local lsm6dso_h_sha256_crlf=fe8eb5586cfe75148957d74a75b148b0d6ccf09e81e45d211981050b0e533546
  local lsm6dso_c_sha256=4a17a4ca5c2a33d2116171f23b65b96a2e497537c443b295b487f60b2fb389ab
  local lsm6dso_h_sha256=ee2c5a6cca9fd78a454a25f015c17d73fa809620cdc42b971b1750ba66eb4326
  # ---------------------------------------------------------------------------

  local root tool missing=() arm_gnu_file arm_gnu_url tc_dir tmp f
  local lsm6dso_url c_sum h_sum
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1

  # cmake, and gcc/g++ for picotool, which the Pico SDK builds for the host.
  for tool in git tar xz python3 cmake make gcc g++ sha256sum; do
    command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
  done
  if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
    missing+=("curl or wget")
  fi
  if (( ${#missing[@]} )); then
    printf 'Missing host tools: %s\n' "${missing[*]}" >&2
    return 1
  fi

  _freertos_fetch() {  # url dest
    if command -v curl >/dev/null 2>&1; then
      curl -fL --retry 3 -o "$2" "$1"
    else
      wget -q -O "$2" "$1"
    fi
  }

  _freertos_sha256_ok() {  # file sha256
    [[ "$(sha256sum "$1" | cut -d' ' -f1)" == "$2" ]]
  }

  # Shallow-clone a tag into dir (via a temporary name, so an interrupted clone
  # is not mistaken for a complete one). An existing clone is left as it is.
  _freertos_clone() {  # url tag commit dir
    local url=$1 tag=$2 commit=$3 dir=$4 head
    if [[ -d "${dir}/.git" ]]; then
      head="$(git -C "$dir" rev-parse HEAD 2>/dev/null)"
      if [[ "$head" != "$commit" ]]; then
        printf 'Note: %s is at %s, not %s (%s); leaving it as it is.\n' \
          "$dir" "${head:-unknown}" "$commit" "$tag" >&2
      fi
      return 0
    fi
    if [[ -e "$dir" ]]; then
      printf '%s exists but is not a Git clone; remove it and retry.\n' "$dir" >&2
      return 1
    fi
    rm -rf "${dir}.tmp"
    git -c advice.detachedHead=false clone --depth 1 --branch "$tag" "$url" "${dir}.tmp" || return 1
    head="$(git -C "${dir}.tmp" rev-parse HEAD)"
    if [[ "$head" != "$commit" ]]; then
      printf 'Tag %s of %s is %s, expected %s.\n' "$tag" "$url" "$head" "$commit" >&2
      return 1
    fi
    mv "${dir}.tmp" "$dir"
  }

  # "git submodule status" starts a line with "-" for a submodule that is not
  # initialized.
  _freertos_submodules() {  # dir submodule...
    local dir=$1 sm
    shift
    for sm in "$@"; do
      if [[ "$(git -C "$dir" submodule status -- "$sm" 2>/dev/null)" == -* ]]; then
        git -C "$dir" submodule update --init --depth=1 -- "$sm" || return 1
      fi
    done
  }

  # Pico SDK, FreeRTOS kernel ---------------------------------------------------
  _freertos_clone https://github.com/raspberrypi/pico-sdk.git "$pico_sdk_tag" "$pico_sdk_commit" \
    "${root}/pico-sdk" || return 1
  _freertos_submodules "${root}/pico-sdk" "${pico_sdk_submodules[@]}" || return 1
  _freertos_clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git "$kernel_tag" "$kernel_commit" \
    "${root}/FreeRTOS-Kernel" || return 1
  for f in "${root}/pico-sdk/lib/tinyusb/src/tusb.h" \
           "${root}/FreeRTOS-Kernel/${kernel_port}/FreeRTOS_Kernel_import.cmake"; do
    if [[ ! -f "$f" ]]; then
      printf '%s is missing.\n' "$f" >&2
      return 1
    fi
  done

  # Arm GNU Toolchain ---------------------------------------------------------
  tc_dir="${root}/toolchain/arm-gnu-toolchain-${arm_gnu_version}"
  if [[ ! -x "${tc_dir}/bin/arm-none-eabi-gcc" ]]; then
    if [[ "$(uname -s)-$(uname -m)" != Linux-x86_64 ]]; then
      printf 'The pinned toolchain is for x86_64 Linux only (this host: %s-%s).\n' \
        "$(uname -s)" "$(uname -m)" >&2
      return 1
    fi
    arm_gnu_file="arm-gnu-toolchain-${arm_gnu_version}-x86_64-arm-none-eabi.tar.xz"
    arm_gnu_url="https://developer.arm.com/-/media/Files/downloads/gnu/${arm_gnu_version}/binrel/${arm_gnu_file}"
    mkdir -p "${root}/toolchain/.download" || return 1
    if [[ ! -f "${root}/toolchain/.download/${arm_gnu_file}" ]] || \
       ! _freertos_sha256_ok "${root}/toolchain/.download/${arm_gnu_file}" "$arm_gnu_sha256"; then
      _freertos_fetch "$arm_gnu_url" "${root}/toolchain/.download/${arm_gnu_file}.part" || return 1
      if ! _freertos_sha256_ok "${root}/toolchain/.download/${arm_gnu_file}.part" "$arm_gnu_sha256"; then
        printf 'SHA-256 mismatch: %s\n' "$arm_gnu_url" >&2
        rm -f "${root}/toolchain/.download/${arm_gnu_file}.part"
        return 1
      fi
      mv "${root}/toolchain/.download/${arm_gnu_file}.part" \
        "${root}/toolchain/.download/${arm_gnu_file}" || return 1
    fi
    tmp="${root}/toolchain/.extract"
    rm -rf "$tmp" && mkdir -p "$tmp" || return 1
    # The archive holds one directory; its name is dropped.
    tar --strip-components=1 -xJf "${root}/toolchain/.download/${arm_gnu_file}" -C "$tmp" || return 1
    if [[ ! -x "${tmp}/bin/arm-none-eabi-gcc" ]]; then
      printf 'Unexpected toolchain archive layout in %s.\n' "$arm_gnu_file" >&2
      return 1
    fi
    rm -rf "$tc_dir"
    mv "$tmp" "$tc_dir" || return 1
    rm -rf "${root}/toolchain/.download"
  fi

  # Python venv with clang-format ----------------------------------------------
  if [[ ! -x "${root}/.venv/bin/python" ]]; then
    python3 -m venv "${root}/.venv" || return 1
  fi
  if [[ ! -x "${root}/.venv/bin/clang-format" ]] || \
     [[ "$("${root}/.venv/bin/python" -c 'from importlib.metadata import version; print(version("clang-format"))' 2>/dev/null)" \
        != "$clang_format_version" ]]; then
    "${root}/.venv/bin/python" -m pip install --quiet "clang-format==${clang_format_version}" || return 1
  fi

  # ST LSM6DSO register driver (freertos/stmemsc, built by
  # common/pico_sdk/drivers/lsm6dso/lsm6dso.cmake) -------------------------
  if [[ ! -f "${root}/stmemsc/lsm6dso_reg.c" ]] || [[ ! -f "${root}/stmemsc/lsm6dso_reg.h" ]] || \
     ! _freertos_sha256_ok "${root}/stmemsc/lsm6dso_reg.c" "$lsm6dso_c_sha256" || \
     ! _freertos_sha256_ok "${root}/stmemsc/lsm6dso_reg.h" "$lsm6dso_h_sha256"; then
    lsm6dso_url="https://raw.githubusercontent.com/STMicroelectronics/lsm6dso-pid/${lsm6dso_commit}"
    tmp="${root}/stmemsc/.download"
    rm -rf "$tmp" && mkdir -p "$tmp" || return 1
    _freertos_fetch "${lsm6dso_url}/lsm6dso_reg.c" "${tmp}/lsm6dso_reg.c.crlf" || return 1
    _freertos_fetch "${lsm6dso_url}/lsm6dso_reg.h" "${tmp}/lsm6dso_reg.h.crlf" || return 1
    if ! _freertos_sha256_ok "${tmp}/lsm6dso_reg.c.crlf" "$lsm6dso_c_sha256_crlf" || \
       ! _freertos_sha256_ok "${tmp}/lsm6dso_reg.h.crlf" "$lsm6dso_h_sha256_crlf"; then
      printf 'SHA-256 mismatch: lsm6dso_reg.[ch] from %s\n' "$lsm6dso_url" >&2
      return 1
    fi
    tr -d '\r' < "${tmp}/lsm6dso_reg.c.crlf" > "${tmp}/lsm6dso_reg.c" || return 1
    tr -d '\r' < "${tmp}/lsm6dso_reg.h.crlf" > "${tmp}/lsm6dso_reg.h" || return 1
    c_sum="$(sha256sum "${tmp}/lsm6dso_reg.c" | cut -d' ' -f1)"
    h_sum="$(sha256sum "${tmp}/lsm6dso_reg.h" | cut -d' ' -f1)"
    if [[ "$c_sum" != "$lsm6dso_c_sha256" ]] || [[ "$h_sum" != "$lsm6dso_h_sha256" ]]; then
      printf 'SHA-256 mismatch after converting lsm6dso_reg.[ch] to LF.\n' >&2
      return 1
    fi
    mv "${tmp}/lsm6dso_reg.c" "${tmp}/lsm6dso_reg.h" "${root}/stmemsc/" || return 1
    rm -rf "$tmp"
  fi

  # Environment ---------------------------------------------------------------
  export PICO_SDK_PATH="${root}/pico-sdk"
  export FREERTOS_KERNEL_PATH="${root}/FreeRTOS-Kernel"
  export PICO_TOOLCHAIN_PATH="$tc_dir"
  for f in "${root}/script" "${root}/.venv/bin" "${tc_dir}/bin"; do
    case ":${PATH}:" in
      *":${f}:"*) ;;
      *) PATH="${f}:${PATH}" ;;
    esac
  done
  export PATH
  printf 'FreeRTOS environment ready: %s (toolchain: %s)\n' "$root" "$tc_dir"
}

_freertos_setup
_freertos_setup_status=$?
unset -f _freertos_setup _freertos_fetch _freertos_sha256_ok _freertos_clone _freertos_submodules
if (( _freertos_setup_status != 0 )); then
  unset _freertos_setup_status
  return 1
fi
unset _freertos_setup_status
