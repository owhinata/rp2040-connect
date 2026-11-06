#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf 'This script must be sourced: source script/setup.sh\n' >&2
  exit 1
fi

_zephyr_setup() {
  local root python west manifest_path project_status sdk_version sdk_dir quoted_west
  local needs_packages=0
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  python="${root}/.venv/bin/python"
  west="${root}/.venv/bin/west"

  if [[ ! -x "$python" ]]; then
    python3 -m venv "${root}/.venv" || return 1
  fi
  if [[ ! -x "$west" ]] || ! "$python" -c 'import west' 2>/dev/null; then
    "$python" -m pip install west || return 1
    needs_packages=1
  fi

  if [[ ! -f "${root}/.west/config" ]]; then
    (cd "$root" && "$west" init -l manifest) || return 1
  fi
  manifest_path="$(cd "$root" && "$west" config manifest.path)" || return 1
  if [[ "$manifest_path" != manifest ]]; then
    printf 'Unexpected west manifest.path: %s (expected manifest)\n' "$manifest_path" >&2
    return 1
  fi

  project_status="$(cd "$root" && "$west" list -f '{name} {cloned}' 2>/dev/null)" || project_status=""
  if [[ -z "$project_status" ]] || \
     printf '%s\n' "$project_status" | grep -qvE '^(manifest |[^ ]+ cloned$)'; then
    (cd "$root" && "$west" update) || return 1
    needs_packages=1
  fi

  # These imports cover the tools used by the build and SDK commands. If an
  # environment is incomplete, install the workspace's full requirements.
  if (( needs_packages )) || ! "$python" -c 'import west, yaml, pykwalify, elftools, patoolib, requests, semver, serial' 2>/dev/null; then
    (cd "$root" && "$west" packages pip --install) || return 1
  fi

  if [[ ! -f "${root}/zephyr/SDK_VERSION" ]]; then
    printf 'Zephyr SDK_VERSION was not found.\n' >&2
    return 1
  fi
  sdk_version="$(tr -d '[:space:]' < "${root}/zephyr/SDK_VERSION")"
  sdk_dir="${root}/zephyr-sdk-${sdk_version}"
  if [[ ! -f "${sdk_dir}/sdk_version" ]] || \
     [[ ! -x "${sdk_dir}/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc" ]]; then
    (cd "$root" && env -u ZEPHYR_SDK_INSTALL_DIR HOME="$root" \
      "$west" sdk install --install-base "$root" --gnu-toolchains arm-zephyr-eabi) || return 1
  fi
  if [[ ! -f "${sdk_dir}/sdk_version" ]] || \
     [[ ! -x "${sdk_dir}/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc" ]]; then
    printf 'SDK with Arm toolchain not found at %s.\n' "$sdk_dir" >&2
    return 1
  fi

  export ZEPHYR_SDK_INSTALL_DIR="$sdk_dir"
  printf -v quoted_west '%q' "$west"
  # The path is intentionally expanded when the alias is defined.
  # shellcheck disable=SC2139
  alias west="$quoted_west"
  printf 'Zephyr environment ready: %s\n' "$root"
}

_zephyr_setup
_zephyr_setup_status=$?
unset -f _zephyr_setup
if (( _zephyr_setup_status != 0 )); then
  unset _zephyr_setup_status
  return 1
fi
unset _zephyr_setup_status
