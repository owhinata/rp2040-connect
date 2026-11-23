#!/usr/bin/env bash
# Prepare the Pico SDK + ThreadX workspace under threadx/ without sudo.
# Source it:
#   source threadx/script/setup.sh
# Each step runs only when its result is missing, so sourcing it again is safe.
#
# For now it fetches the Eclipse ThreadX kernel only (threadx/threadx), which
# the host tests build on ThreadX's Linux port (threadx/script/host_tests.sh).

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'This script must be sourced: source threadx/script/setup.sh\n' >&2
  exit 1
fi

_threadx_setup() {
  # ---- Pinned versions (change them here only) ------------------------------
  # Eclipse ThreadX (MIT).  Of its tree only the kernel is used (common/ and
  # the ports ports/cortex_m0/gnu and, for the host tests, ports/linux/gnu);
  # no submodule is needed.
  local threadx_tag=v6.5.2.202603b_rel
  local threadx_commit=b2868c3d2849aab95d04cedcf8885d3f81798c86
  # ---------------------------------------------------------------------------

  local root tool missing=() f
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1

  for tool in git; do
    command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
  done
  if (( ${#missing[@]} )); then
    printf 'Missing host tools: %s\n' "${missing[*]}" >&2
    return 1
  fi

  # Shallow-clone a tag into dir (via a temporary name, so an interrupted clone
  # is not mistaken for a complete one). An existing clone is left as it is.
  _threadx_clone() {  # url tag commit dir
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

  # ThreadX ---------------------------------------------------------------------
  _threadx_clone https://github.com/eclipse-threadx/threadx.git "$threadx_tag" "$threadx_commit" \
    "${root}/threadx" || return 1
  for f in "${root}/threadx/common/inc/tx_api.h" \
           "${root}/threadx/ports/cortex_m0/gnu/inc/tx_port.h" \
           "${root}/threadx/ports/linux/gnu/inc/tx_port.h"; do
    if [[ ! -f "$f" ]]; then
      printf '%s is missing.\n' "$f" >&2
      return 1
    fi
  done

  # Environment ---------------------------------------------------------------
  export THREADX_PATH="${root}/threadx"
  case ":${PATH}:" in
    *":${root}/script:"*) ;;
    *) PATH="${root}/script:${PATH}" ;;
  esac
  export PATH
  printf 'ThreadX environment ready: %s\n' "$root"
}

_threadx_setup
_threadx_setup_status=$?
unset -f _threadx_setup _threadx_clone
if (( _threadx_setup_status != 0 )); then
  unset _threadx_setup_status
  return 1
fi
unset _threadx_setup_status
