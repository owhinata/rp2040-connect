#!/usr/bin/env bash
# Prepare the OpenMV (MicroPython) workspace under openmv/ without sudo. Source it:
#   source openmv/script/setup.sh
# Each step runs only when its result is missing, so sourcing it again is safe.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'This script must be sourced: source openmv/script/setup.sh\n' >&2
  exit 1
fi

_openmv_setup() {
  # ---- Pinned versions (change them here only) ------------------------------
  # OpenMV firmware.  Its submodules (MicroPython and others) are pinned by
  # this commit.  The rp2 port of this release needs patches/openmv/.
  local openmv_tag=v5.0.1
  local openmv_commit=631681e5ac5ab332c920b6bf9cd3dba5cf299eac
  # OpenMV SDK (Arm GNU toolchain 14.3.Rel1, CMake and other tools).  The
  # version is the one named in openmv/SDK_VERSION at the tag above.
  local sdk_version=1.6.0
  local sdk_sha256=93003345f37b910e66bb2a41fa19f17160437693cbc0452f6d47d3c37da37a14
  # Submodules of openmv that the rp2 port builds from, and the submodules
  # of MicroPython that the board needs: the ones its CMake configuration
  # lists (UPDATE_SUBMODULES), and pico-sdk.
  local -a openmv_submodules=(lib/micropython modules/ulab lib/apriltag)
  local -a mpy_submodules=(lib/pico-sdk lib/mbedtls lib/tinyusb lib/mynewt-nimble lib/micropython-lib)
  # ---------------------------------------------------------------------------

  local root tool missing=() omv head sdk_dir sdk_file sdk_url dl tmp f sm
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  omv="${root}/openmv"

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

  # OpenMV --------------------------------------------------------------------
  # Shallow-clone the tag (via a temporary name, so an interrupted clone is
  # not mistaken for a complete one).  An existing clone is left as it is.
  if [[ -d "${omv}/.git" ]]; then
    head="$(git -C "$omv" rev-parse HEAD 2>/dev/null)"
    if [[ "$head" != "$openmv_commit" ]]; then
      printf 'Note: %s is at %s, not %s (%s); leaving it as it is.\n' \
        "$omv" "${head:-unknown}" "$openmv_commit" "$openmv_tag" >&2
    fi
  elif [[ -e "$omv" ]]; then
    printf '%s exists but is not a Git clone; remove it and retry.\n' "$omv" >&2
    return 1
  else
    rm -rf "${omv}.tmp"
    git -c advice.detachedHead=false clone --depth 1 --branch "$openmv_tag" \
      https://github.com/openmv/openmv.git "${omv}.tmp" || return 1
    head="$(git -C "${omv}.tmp" rev-parse HEAD)"
    if [[ "$head" != "$openmv_commit" ]]; then
      printf 'Tag %s of openmv is %s, expected %s.\n' "$openmv_tag" "$head" "$openmv_commit" >&2
      return 1
    fi
    mv "${omv}.tmp" "$omv" || return 1
  fi

  # "git submodule status" starts a line with "-" for a submodule that is not
  # initialized.
  for sm in "${openmv_submodules[@]}"; do
    if [[ "$(git -C "$omv" submodule status -- "$sm" 2>/dev/null)" == -* ]]; then
      git -C "$omv" submodule update --init --depth=1 -- "$sm" || return 1
    fi
  done

  # Apply patches/<name>/*.patch to a clone, in name order.  What has been
  # applied is recorded in the clone's .git/openmv-patches/ (a copy of each
  # applied patch and the list in "applied"), outside the work tree.  When
  # the set of patches differs from the record (a patch added, removed or
  # changed), the recorded patches are taken back in reverse order and the
  # current ones applied.  A clone with local changes but no record is left
  # alone (restore it with "git checkout -- ." first).
  _openmv_patch() {  # clone patchdir
    local dir=$1 pdir=$2 rec p name want="" have="" i
    local -a old=()
    rec="${dir}/.git/openmv-patches"
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

  # The rp2 port of this release does not build and its protocol code faults
  # on the Cortex-M0+; patches/openmv/ brings it up (see openmv/README.md).
  _openmv_patch "$omv" "${root}/patches/openmv" || return 1

  # OpenMV SDK ----------------------------------------------------------------
  # An SDK of the right version in OpenMV's default place (~/openmv-sdk-<ver>,
  # where "make sdk" puts it) is used as it is.  Otherwise it is installed
  # under openmv/.
  _openmv_sdk_ok() {  # dir
    [[ -f "$1/sdk.version" ]] && \
      [[ "$(tr -d '[:space:]' < "$1/sdk.version")" == "$sdk_version" ]] && \
      [[ -x "$1/gcc/bin/arm-none-eabi-gcc" ]] && [[ -x "$1/cmake/bin/cmake" ]]
  }
  if _openmv_sdk_ok "${HOME}/openmv-sdk-${sdk_version}"; then
    sdk_dir="${HOME}/openmv-sdk-${sdk_version}"
  else
    sdk_dir="${root}/openmv-sdk-${sdk_version}"
  fi
  if ! _openmv_sdk_ok "$sdk_dir"; then
    if [[ "$(uname -s)-$(uname -m)" != Linux-x86_64 ]]; then
      printf 'The pinned SDK archive is for x86_64 Linux only (this host: %s-%s).\n' \
        "$(uname -s)" "$(uname -m)" >&2
      return 1
    fi
    sdk_file="openmv-sdk-${sdk_version}-linux-x86_64.tar.xz"
    sdk_url="https://download.openmv.io/sdk/${sdk_file}"
    dl="${root}/.download"
    mkdir -p "$dl" || return 1
    if [[ ! -f "${dl}/${sdk_file}" ]]; then
      # About 0.9 GB; an interrupted download is continued.
      printf 'Downloading %s\n' "$sdk_url"
      if command -v curl >/dev/null 2>&1; then
        curl -fL --retry 3 -C - -o "${dl}/${sdk_file}.part" "$sdk_url" || return 1
      else
        wget -c -q --show-progress -O "${dl}/${sdk_file}.part" "$sdk_url" || return 1
      fi
      if [[ "$(sha256sum "${dl}/${sdk_file}.part" | cut -d' ' -f1)" != "$sdk_sha256" ]]; then
        printf 'SHA-256 mismatch: %s\n' "$sdk_url" >&2
        rm -f "${dl}/${sdk_file}.part"
        return 1
      fi
      mv "${dl}/${sdk_file}.part" "${dl}/${sdk_file}" || return 1
    fi
    tmp="${sdk_dir}.tmp"
    rm -rf "$tmp" && mkdir -p "$tmp" || return 1
    printf 'Extracting %s\n' "$sdk_file"
    tar --strip-components=1 -xJf "${dl}/${sdk_file}" -C "$tmp" || return 1
    if ! _openmv_sdk_ok "$tmp"; then
      printf 'Unexpected SDK archive layout in %s.\n' "$sdk_file" >&2
      return 1
    fi
    rm -rf "$sdk_dir"
    mv "$tmp" "$sdk_dir" || return 1
    rm -rf "$dl"
  fi

  # MicroPython's submodules for the board -------------------------------------
  for sm in "${mpy_submodules[@]}"; do
    if [[ "$(git -C "${omv}/lib/micropython" submodule status -- "$sm" 2>/dev/null)" == -* ]]; then
      git -C "${omv}/lib/micropython" submodule update --init --depth=1 -- "$sm" || return 1
    fi
  done

  # Environment ---------------------------------------------------------------
  # The compiler and CMake of the SDK go on PATH.  The SDK's make and python
  # are left out, so that they do not replace the host's in this shell.
  export OPENMV_SDK_DIR="$sdk_dir"
  for f in "${root}/script" "${sdk_dir}/cmake/bin" "${sdk_dir}/gcc/bin"; do
    case ":${PATH}:" in
      *":${f}:"*) ;;
      *) PATH="${f}:${PATH}" ;;
    esac
  done
  export PATH
  printf 'OpenMV environment ready: %s (SDK: %s)\n' "$root" "$sdk_dir"
}

_openmv_setup
_openmv_setup_status=$?
unset -f _openmv_setup _openmv_sdk_ok _openmv_patch
if (( _openmv_setup_status != 0 )); then
  unset _openmv_setup_status
  return 1
fi
unset _openmv_setup_status
