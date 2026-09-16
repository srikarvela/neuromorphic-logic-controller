#!/usr/bin/env bash
# Run the Vivado flow inside a Parallels Windows VM from macOS (Apple
# Silicon hosts can't run Vivado natively) and copy the results back into
# fpga/build/.
#
#   ./scripts/vivado_in_parallels.sh ooc         # out-of-context synthesis only
#   ./scripts/vivado_in_parallels.sh bitstream   # full PS + DMA + pipeline build
#
# How it works: the committed tree (git HEAD, so commit first) is zipped
# into a folder the VM can see through Parallels shared folders, unzipped
# to a local VM disk, built with `prlctl exec --current-user`, and the
# outputs are copied back through the same shared folder.
#
# Vivado under Windows-on-ARM x86 emulation intermittently fails to read
# its own data files while creating a block design ("couldn't read file
# .../busdef.tcl", "find_approot_file ... init.tcl"). Those runs are retried
# automatically; any other failure stops the script.
#
# Environment overrides:
#   NLC_VM          Parallels VM name                      (default "Windows 11")
#   NLC_VIVADO      vivado.bat path inside the VM          (default C:\Xilinx\Vivado\2024.1\bin\vivado.bat)
#   NLC_VM_WORKDIR  build directory inside the VM          (default C:\nlc)
#   NLC_SHARE_MAC   macOS side of a shared folder          (default ~/Downloads/nlc-vm-transfer)
#   NLC_SHARE_VM    the same folder as the VM sees it      (default Z:\Downloads\nlc-vm-transfer)
#   NLC_TRIES       attempts for flaky block-design runs   (default 4)
set -euo pipefail
cd "$(dirname "$0")/.."

MODE="${1:-bitstream}"
VM="${NLC_VM:-Windows 11}"
VIVADO="${NLC_VIVADO:-C:\\Xilinx\\Vivado\\2024.1\\bin\\vivado.bat}"
WORK="${NLC_VM_WORKDIR:-C:\\nlc}"
SHARE_MAC="${NLC_SHARE_MAC:-$HOME/Downloads/nlc-vm-transfer}"
SHARE_VM="${NLC_SHARE_VM:-Z:\\Downloads\\nlc-vm-transfer}"
TRIES="${NLC_TRIES:-4}"

case "$MODE" in
  ooc)       TCL="fpga/tcl/synth_ooc.tcl";       OUTS="ooc_utilization.rpt ooc_timing.rpt" ;;
  bitstream) TCL="fpga/tcl/build_bitstream.tcl"; OUTS="nlc.bit nlc.hwh timing_summary.rpt utilization.rpt" ;;
  *) echo "usage: $0 [ooc|bitstream]"; exit 2 ;;
esac

command -v prlctl >/dev/null || { echo "prlctl not found (Parallels Desktop Pro/Business required)"; exit 1; }
prlctl list "$VM" >/dev/null 2>&1 || { echo "Parallels VM '$VM' not found (set NLC_VM)"; exit 1; }
if [ -n "$(git status --porcelain -- rtl fpga)" ]; then
  echo "warning: uncommitted changes under rtl/ or fpga/ are NOT sent to the VM (it builds git HEAD)"
fi

vm() { prlctl exec "$VM" --current-user "$@"; }

mkdir -p "$SHARE_MAC"
rm -rf "$SHARE_MAC/out"
git archive --format=zip -o "$SHARE_MAC/nlc.zip" HEAD
vm powershell -NoProfile -Command "Remove-Item -Recurse -Force '$WORK' -ErrorAction SilentlyContinue; Expand-Archive -Path '$SHARE_VM\\nlc.zip' -DestinationPath '$WORK'; New-Item -ItemType Directory -Force '$WORK\\fpga\\build' | Out-Null" >/dev/null
echo "Copied git HEAD ($(git rev-parse --short HEAD)) to $VM:$WORK"

status=1
for i in $(seq 1 "$TRIES"); do
  log="vivado_${MODE}_try$i.log"
  echo "== Vivado $MODE, attempt $i/$TRIES (log: fpga/build/$log)"
  vm cmd /c "cd /d $WORK && rmdir /s /q fpga\\vivado 2>nul & \"$VIVADO\" -mode batch -nolog -nojournal -source $TCL > fpga\\build\\$log 2>&1" || true
  vm cmd /c "mkdir \"$SHARE_VM\\out\" 2>nul & copy /y \"$WORK\\fpga\\build\\*\" \"$SHARE_VM\\out\\\" >nul" || true
  mkdir -p fpga/build
  cp "$SHARE_MAC/out/$log" fpga/build/ 2>/dev/null || true
  grep -E "^ERROR|CRITICAL WARNING|Worst setup slack|OOC: worst|=== Wrote" "fpga/build/$log" | head -20 || true
  if grep -qE "^ERROR" "fpga/build/$log"; then
    if grep -qE "couldn't read file|find_approot_file|invalid command name \"::xgui" "fpga/build/$log"; then
      echo "-- flaky Vivado data-file read under emulation, retrying"
      continue
    fi
    echo "-- Vivado failed (see fpga/build/$log)"
    break
  fi
  for f in $OUTS; do cp "$SHARE_MAC/out/$f" fpga/build/; done
  status=0
  break
done

rm -rf "$SHARE_MAC"
if [ "$status" -eq 0 ]; then
  echo "Done: $(cd fpga/build && ls $OUTS | tr '\n' ' ')in fpga/build/"
fi
exit "$status"
