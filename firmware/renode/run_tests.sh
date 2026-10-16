#!/usr/bin/env bash
# Run the STM32H743 firmware image under Renode.
#
#   RENODE_DIR="$(./scripts/install_renode.sh)" ./firmware/renode/run_tests.sh
#
# RENODE_DIR is an unpacked Renode 1.17.0 portable release, with the Python
# packages renode-test needs either installed or in RENODE_DIR/venv, which is
# where scripts/install_renode.sh puts them. The image has to have been built
# first (./firmware/build_firmware.sh). This builds the host link
# tool, then runs tests/a1_board.robot and leaves the capture, the timer log
# and Robot's report in firmware/build-renode/, or in A1_WORK if that is set.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
firmware="$(cd "${here}/.." && pwd)"

: "${RENODE_DIR:?set RENODE_DIR to an unpacked Renode 1.17.0 portable release}"
if [ -x "${RENODE_DIR}/venv/bin/python" ]; then
  export PATH="${RENODE_DIR}/venv/bin:${PATH}"
fi

elf="${firmware}/build-arm/pa_firmware.elf"
test -f "${elf}" || { echo "no image at ${elf}; run ./firmware/build_firmware.sh first" >&2; exit 1; }

jobs="$( (command -v nproc >/dev/null && nproc) || echo 4)"
cmake -S "${firmware}" -B "${firmware}/build-tools" -DCMAKE_BUILD_TYPE=Release \
  -DPA_FIRMWARE_BUILD_TESTS=OFF > /dev/null
cmake --build "${firmware}/build-tools" --target pa_link -j"${jobs}" > /dev/null

work="${A1_WORK:-${firmware}/build-renode}"
rm -rf "${work}"
mkdir -p "${work}"

export A1_ELF="${elf}"
export A1_WORK="${work}"
export A1_PA_LINK="${firmware}/build-tools/pa_link"

# renode-test writes its results to the current directory.
cd "${work}"
"${RENODE_DIR}/renode-test" --keep-renode-output "${here}/tests/a1_board.robot"
