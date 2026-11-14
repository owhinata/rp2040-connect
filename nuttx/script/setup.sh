#!/usr/bin/env bash
# Prepare the NuttX workspace under nuttx/ without sudo. Source it:
#   source nuttx/script/setup.sh
# Each step runs only when its result is missing, so sourcing it again is safe.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'This script must be sourced: source nuttx/script/setup.sh\n' >&2
  exit 1
fi

_nuttx_setup() {
  # ---- Pinned versions (change them here only) ------------------------------
  # NuttX and nuttx-apps: the same release tag for both.
  local nuttx_tag=nuttx-13.1.0
  local nuttx_commit=2b5c0de5e933281c209f954a63b99cf10c132346
  local apps_commit=3c772ea35b9245972e18999828f7b533e6fc57cc
  # pico-sdk, used only to build boot2. 2.2.0 is the version named in
  # Documentation/platforms/arm/rp2040/index.rst and used by NuttX CI.
  local pico_sdk_tag=2.2.0
  local pico_sdk_commit=a1438dff1d38bd9c65dbd693f0e5db4b9ae91779
  # Only what NuttX's boot2 build reads (arch/arm/src/rp2040/boot2/Make.defs:
  # the second stage source with pad_checksum, and its include directories)
  # is checked out, about 4 MB instead of the SDK's 31 MB.  A full clone
  # that is already there is used as it is; remove nuttx/pico-sdk to get
  # the small one.
  local -a pico_sdk_paths=(
    /LICENSE.TXT
    /src/rp2040/boot_stage2/
    /src/rp2040/hardware_regs/include/
    /src/rp2040/pico_platform/include/
    /src/rp2_common/hardware_base/include/
    /src/rp2_common/pico_platform_common/include/
    /src/rp2_common/pico_platform_compiler/include/
    /src/rp2_common/pico_platform_sections/include/
    /src/rp2_common/pico_platform_panic/include/
    /src/common/pico_base_headers/include/
    /src/boards/include/boards/pico.h
  )
  # Arm GNU Toolchain, the version used by NuttX CI (tools/ci/platforms/ubuntu.sh).
  local arm_gnu_version=13.2.rel1
  local arm_gnu_dirname=arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi
  local arm_gnu_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb
  local kconfiglib_version=14.1.0
  # LSM6DSO register driver: lsm6dso-pid v3.2.2, the driver submodule of
  # STMems_Standard_C_drivers v2.13.2 (the stmemsc version in Zephyr's hal_st).
  # Upstream files use CRLF; they are stored with LF like hal_st.
  local lsm6dso_commit=f55167674138aca789c70a2970187c6322192dde
  local lsm6dso_c_sha256_crlf=0ba42733ac6242bebd4b404d6c7e45b7420f6f15cbdfc4d34b0ae966698271a1
  local lsm6dso_h_sha256_crlf=fe8eb5586cfe75148957d74a75b148b0d6ccf09e81e45d211981050b0e533546
  local lsm6dso_c_sha256=4a17a4ca5c2a33d2116171f23b65b96a2e497537c443b295b487f60b2fb389ab
  local lsm6dso_h_sha256=ee2c5a6cca9fd78a454a25f015c17d73fa809620cdc42b971b1750ba66eb4326
  # argtable3, the command line parser that nuttx-apps' iperf selects
  # (SYSTEM_ARGTABLE3).  The tag is CONFIG_SYSTEM_ARGTABLE3_VERSION's default
  # in nuttx-apps 13.1.0.  Without a Git clone in place, its Makefile
  # downloads the tarball at build time and "make distclean" deletes it.
  local argtable3_tag=v3.2.0.7402e6e
  local argtable3_commit=7402e6ead5dc5ca7db1c321baa8c10ac243907e4
  # ---------------------------------------------------------------------------

  local root tool missing=() arm_gnu_file arm_gnu_url tc_dir tmp f
  local lsm6dso_url c_sum h_sum
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1

  for tool in git tar xz python3 make gcc sha256sum; do
    command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
  done
  if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
    missing+=("curl or wget")
  fi
  if (( ${#missing[@]} )); then
    printf 'Missing host tools: %s\n' "${missing[*]}" >&2
    return 1
  fi

  _nuttx_fetch() {  # url dest
    if command -v curl >/dev/null 2>&1; then
      curl -fL --retry 3 -o "$2" "$1"
    else
      wget -q -O "$2" "$1"
    fi
  }

  _nuttx_sha256_ok() {  # file sha256
    [[ "$(sha256sum "$1" | cut -d' ' -f1)" == "$2" ]]
  }

  # Shallow-clone a tag into dir (via a temporary name, so an interrupted clone
  # is not mistaken for a complete one). An existing clone is left as it is.
  _nuttx_clone() {  # url tag commit dir
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

  # Like _nuttx_clone, but checks out only the given paths (sparse checkout
  # patterns) and downloads only their files (a partial clone).
  _nuttx_clone_sparse() {  # url tag commit dir path...
    local url=$1 tag=$2 commit=$3 dir=$4 head
    shift 4
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
    git -c advice.detachedHead=false clone --depth 1 --filter=blob:none --sparse \
      --branch "$tag" "$url" "${dir}.tmp" || return 1
    head="$(git -C "${dir}.tmp" rev-parse HEAD)"
    if [[ "$head" != "$commit" ]]; then
      printf 'Tag %s of %s is %s, expected %s.\n' "$tag" "$url" "$head" "$commit" >&2
      return 1
    fi
    git -C "${dir}.tmp" sparse-checkout set --no-cone "$@" || return 1
    mv "${dir}.tmp" "$dir"
  }

  # Apply patches/<name>/*.patch to a clone, in name order.  What has been
  # applied is recorded in the clone's .git/nano-patches/ (a copy of each
  # applied patch and the list in "applied"), outside the work tree.  When
  # the set of patches differs from the record (a patch added, removed or
  # changed), the recorded patches are taken back in reverse order and the
  # current ones applied.  A clone with local changes but no record is left
  # alone (restore it with "git checkout -- ." first).
  _nuttx_patch() {  # clone patchdir
    local dir=$1 pdir=$2 rec p name want="" have="" i
    local -a old=()
    rec="${dir}/.git/nano-patches"
    if [[ -d "$pdir" ]]; then
      for p in "$pdir"/*.patch; do
        [[ -f "$p" ]] || continue
        want+="$(basename -- "$p") $(sha256sum < "$p" | cut -d' ' -f1)"$'\n'
      done
    fi
    [[ -f "${rec}/applied" ]] && have="$(cat "${rec}/applied")"$'\n'
    [[ "$have" == $'\n' ]] && have=""
    [[ "$want" == "$have" ]] && return 0

    if [[ ! -f "${rec}/applied" ]] && ! git -C "$dir" diff --quiet; then
      printf '%s has local changes and no record of applied patches;\n' "$dir" >&2
      printf 'restore it with "git -C %s checkout -- ." and source setup.sh again.\n' "$dir" >&2
      return 1
    fi

    # Take back what is applied, last first.
    if [[ -f "${rec}/applied" ]]; then
      mapfile -t old < <(cut -d' ' -f1 "${rec}/applied")
      for (( i = ${#old[@]} - 1; i >= 0; i-- )); do
        name=${old[i]}
        [[ -n "$name" ]] || continue
        if ! git -C "$dir" apply --reverse "${rec}/${name}"; then
          printf 'Cannot take back %s from %s; restore it with "git -C %s checkout -- ."\n' \
            "$name" "$dir" "$dir" >&2
          printf 'and remove %s, then source setup.sh again.\n' "$rec" >&2
          return 1
        fi
        printf 'Reverted %s\n' "$name"
      done
    fi
    rm -rf "$rec" && mkdir -p "$rec" && : > "${rec}/applied" || return 1

    [[ -n "$want" ]] || return 0
    for p in "$pdir"/*.patch; do
      [[ -f "$p" ]] || continue
      name="$(basename -- "$p")"
      if ! git -C "$dir" apply --check "$p"; then
        printf 'Patch %s does not apply to %s.\n' "$p" "$dir" >&2
        return 1
      fi
      git -C "$dir" apply "$p" && cp -- "$p" "${rec}/${name}" || return 1
      printf '%s %s\n' "$name" "$(sha256sum < "$p" | cut -d' ' -f1)" >> "${rec}/applied"
      printf 'Applied %s\n' "${p#"${root}/"}"
    done
  }

  _nuttx_link() {  # target linkpath
    if [[ -L "$2" ]]; then
      [[ "$(readlink "$2")" == "$1" ]] && return 0
      printf 'Note: %s points to %s, not %s; leaving it as it is.\n' \
        "$2" "$(readlink "$2")" "$1" >&2
      return 0
    fi
    if [[ -e "$2" ]]; then
      printf '%s exists and is not a symlink; leaving it as it is.\n' "$2" >&2
      return 0
    fi
    ln -s "$1" "$2"
  }

  # NuttX, nuttx-apps, pico-sdk ----------------------------------------------
  _nuttx_clone https://github.com/apache/nuttx.git "$nuttx_tag" "$nuttx_commit" \
    "${root}/nuttx" || return 1
  _nuttx_clone https://github.com/apache/nuttx-apps.git "$nuttx_tag" "$apps_commit" \
    "${root}/nuttx-apps" || return 1
  _nuttx_clone_sparse https://github.com/raspberrypi/pico-sdk.git "$pico_sdk_tag" "$pico_sdk_commit" \
    "${root}/pico-sdk" "${pico_sdk_paths[@]}" || return 1
  for f in src/rp2040/boot_stage2/boot2_at25sf128a.S src/rp2040/boot_stage2/pad_checksum; do
    if [[ ! -f "${root}/pico-sdk/${f}" ]]; then
      printf 'pico-sdk lacks %s.\n' "$f" >&2
      return 1
    fi
  done

  # Inside nuttx-apps' system/argtable3 (ignored by its .gitignore), where
  # its Makefile looks for a clone before downloading.
  _nuttx_clone https://github.com/argtable/argtable3.git "$argtable3_tag" "$argtable3_commit" \
    "${root}/nuttx-apps/system/argtable3/argtable3" || return 1

  _nuttx_patch "${root}/nuttx" "${root}/patches/nuttx" || return 1
  _nuttx_patch "${root}/nuttx-apps" "${root}/patches/apps" || return 1

  # Out-of-tree kernel code and applications (filled by later tasks).
  _nuttx_link ../drivers "${root}/nuttx/external" || return 1
  _nuttx_link ../apps "${root}/nuttx-apps/external" || return 1

  # Arm GNU Toolchain ---------------------------------------------------------
  tc_dir="${root}/toolchain/${arm_gnu_dirname}"
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
       ! _nuttx_sha256_ok "${root}/toolchain/.download/${arm_gnu_file}" "$arm_gnu_sha256"; then
      _nuttx_fetch "$arm_gnu_url" "${root}/toolchain/.download/${arm_gnu_file}.part" || return 1
      if ! _nuttx_sha256_ok "${root}/toolchain/.download/${arm_gnu_file}.part" "$arm_gnu_sha256"; then
        printf 'SHA-256 mismatch: %s\n' "$arm_gnu_url" >&2
        rm -f "${root}/toolchain/.download/${arm_gnu_file}.part"
        return 1
      fi
      mv "${root}/toolchain/.download/${arm_gnu_file}.part" \
        "${root}/toolchain/.download/${arm_gnu_file}" || return 1
    fi
    tmp="${root}/toolchain/.extract"
    rm -rf "$tmp" && mkdir -p "$tmp" || return 1
    tar -xJf "${root}/toolchain/.download/${arm_gnu_file}" -C "$tmp" || return 1
    if [[ ! -x "${tmp}/${arm_gnu_dirname}/bin/arm-none-eabi-gcc" ]]; then
      printf 'Unexpected toolchain archive layout in %s.\n' "$arm_gnu_file" >&2
      return 1
    fi
    rm -rf "$tc_dir"
    mv "${tmp}/${arm_gnu_dirname}" "$tc_dir" || return 1
    rm -rf "$tmp" "${root}/toolchain/.download"
  fi

  # Python venv with kconfiglib ----------------------------------------------
  if [[ ! -x "${root}/.venv/bin/python" ]]; then
    python3 -m venv "${root}/.venv" || return 1
  fi
  if [[ ! -x "${root}/.venv/bin/olddefconfig" ]] || \
     [[ "$("${root}/.venv/bin/python" -c 'from importlib.metadata import version; print(version("kconfiglib"))' 2>/dev/null)" \
        != "$kconfiglib_version" ]]; then
    "${root}/.venv/bin/python" -m pip install --quiet "kconfiglib==${kconfiglib_version}" || return 1
  fi

  # ST LSM6DSO register driver -------------------------------------------------
  if [[ ! -f "${root}/stmemsc/lsm6dso_reg.c" ]] || [[ ! -f "${root}/stmemsc/lsm6dso_reg.h" ]] || \
     ! _nuttx_sha256_ok "${root}/stmemsc/lsm6dso_reg.c" "$lsm6dso_c_sha256" || \
     ! _nuttx_sha256_ok "${root}/stmemsc/lsm6dso_reg.h" "$lsm6dso_h_sha256"; then
    lsm6dso_url="https://raw.githubusercontent.com/STMicroelectronics/lsm6dso-pid/${lsm6dso_commit}"
    tmp="${root}/stmemsc/.download"
    rm -rf "$tmp" && mkdir -p "$tmp" || return 1
    _nuttx_fetch "${lsm6dso_url}/lsm6dso_reg.c" "${tmp}/lsm6dso_reg.c.crlf" || return 1
    _nuttx_fetch "${lsm6dso_url}/lsm6dso_reg.h" "${tmp}/lsm6dso_reg.h.crlf" || return 1
    if ! _nuttx_sha256_ok "${tmp}/lsm6dso_reg.c.crlf" "$lsm6dso_c_sha256_crlf" || \
       ! _nuttx_sha256_ok "${tmp}/lsm6dso_reg.h.crlf" "$lsm6dso_h_sha256_crlf"; then
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
  for f in "${root}/script" "${root}/.venv/bin" "${tc_dir}/bin"; do
    case ":${PATH}:" in
      *":${f}:"*) ;;
      *) PATH="${f}:${PATH}" ;;
    esac
  done
  export PATH
  printf 'NuttX environment ready: %s\n' "$root"
}

_nuttx_setup
_nuttx_setup_status=$?
unset -f _nuttx_setup _nuttx_fetch _nuttx_sha256_ok _nuttx_clone _nuttx_clone_sparse _nuttx_patch _nuttx_link
if (( _nuttx_setup_status != 0 )); then
  unset _nuttx_setup_status
  return 1
fi
unset _nuttx_setup_status
