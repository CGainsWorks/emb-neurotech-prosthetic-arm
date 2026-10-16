#!/usr/bin/env bash
# Install the pinned Renode release and the Python packages its test runner
# needs.
#
#   ./scripts/install_renode.sh [DEST]
#
# DEST defaults to ~/.cache/renode-<version>. The portable Linux release is
# self-contained; renode-test additionally needs Robot Framework and the
# packages in its tests/requirements.txt, which go into DEST/venv. Prints DEST,
# which is what firmware/renode/run_tests.sh wants as RENODE_DIR. Does only
# what is missing, which is what lets CI cache it.
set -euo pipefail

version=1.17.0
sha256=4ba7c68b59e2447f188ef4b4b112fcccd2802582c460beedceb63b84e5605a5f
dest="${1:-${HOME}/.cache/renode-${version}}"

if [ ! -x "${dest}/renode" ]; then
  url="https://github.com/renode/renode/releases/download/v${version}/renode-${version}.linux-portable.tar.gz"
  tmp="$(mktemp -d)"
  trap 'rm -rf "${tmp}"' EXIT
  curl -fsSL --retry 3 -o "${tmp}/renode.tar.gz" "${url}"
  echo "${sha256}  ${tmp}/renode.tar.gz" | sha256sum -c - >&2
  mkdir -p "${dest}"
  tar -xzf "${tmp}/renode.tar.gz" -C "${dest}" --strip-components=1
fi

if ! "${dest}/venv/bin/python" -c "import robot" >/dev/null 2>&1; then
  rm -rf "${dest}/venv"
  python3 -m venv "${dest}/venv"
  "${dest}/venv/bin/pip" install --quiet --disable-pip-version-check \
    -r "${dest}/tests/requirements.txt" >&2
fi

echo "${dest}"
