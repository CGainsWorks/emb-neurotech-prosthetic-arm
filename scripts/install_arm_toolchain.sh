#!/usr/bin/env bash
# Install the pinned Arm GNU Toolchain (arm-none-eabi) from Arm's own release.
#
#   ./scripts/install_arm_toolchain.sh [DEST]
#
# DEST defaults to ~/.cache/arm-gnu-toolchain-<version>. Prints the bin
# directory to put on PATH. Does nothing but print it if DEST already holds the
# toolchain, which is what lets CI cache it.
#
# Arm's release rather than the distribution's gcc-arm-none-eabi package: the
# version is then the same on every machine that builds the image, the
# archive is checked against a pinned SHA-256 and the compiler is the one Arm
# supports for Cortex-M. x86_64 Linux only.
set -euo pipefail

version=14.3.rel1
sha256=8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd
dest="${1:-${HOME}/.cache/arm-gnu-toolchain-${version}}"

if [ -x "${dest}/bin/arm-none-eabi-gcc" ]; then
  echo "${dest}/bin"
  exit 0
fi

url="https://developer.arm.com/-/media/Files/downloads/gnu/${version}/binrel/arm-gnu-toolchain-${version}-x86_64-arm-none-eabi.tar.xz"
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

curl -fsSL --retry 3 -o "${tmp}/toolchain.tar.xz" "${url}"
echo "${sha256}  ${tmp}/toolchain.tar.xz" | sha256sum -c - >&2

mkdir -p "${dest}"
tar -xJf "${tmp}/toolchain.tar.xz" -C "${dest}" --strip-components=1
echo "${dest}/bin"
