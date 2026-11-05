#!/bin/bash
# Development install of the v2 session side, for one user, on a machine
# booted into a kernel with the "Speaker Protection" interface.
#
#   packaging/install-dev.sh install <machine>    # j316, j616, j413, j700
#   packaging/install-dev.sh remove  <machine>
#
# It installs only into the user's ~/.config and restarts the user's
# PipeWire and WirePlumber, so a reboot into a kernel without the interface
# is unaffected once it is removed. The supervisor and the LV2 bundle are
# installed separately. Never run it on a kernel without the interface: the
# monitor's own speaker node is disabled while it is installed.
set -euo pipefail
cmd=${1:?install|remove}; m=${2:?machine}
here=$(cd "$(dirname "$0")" && pwd)
pw=${XDG_CONFIG_HOME:-$HOME/.config}/pipewire/pipewire.conf.d
wp=${XDG_CONFIG_HOME:-$HOME/.config}/wireplumber/wireplumber.conf.d
bin=$HOME/.local/libexec
[ -e "$here/session/fixtures/$m.json" ] || { echo "unknown machine $m"; exit 2; }
case $cmd in
install)
  card=$(cd "$here/session" && python3 -I -c "import sys;sys.path.insert(0,'.');import gen;print(gen.machine(sys.argv[1])[0]['card'])" "$m")
  [ -e /proc/asound/$card ] || { echo "no card $card"; exit 1; }
  amixer -c "$card" cget name='Speaker Protection' >/dev/null 2>&1 || { echo "kernel has no Speaker Protection control"; exit 1; }
  mkdir -p "$pw" "$wp" "$bin"
  install -m 0644 "$here/pipewire/sgv2-$m.conf" "$pw/"
  install -m 0644 "$here/wireplumber/60-sgv2-$m.conf" "$wp/"
  install -m 0755 "$here/session/speakerguardd-session" "$bin/"
  # The speaker route needs the secondary front-end on the amps (UCM sets it on
  # WirePlumber's own node, which is disabled here).
  amixer -q -c "$card" cset name='Speaker Playback Mux' Secondary || true
  systemctl --user restart pipewire wireplumber
  echo "installed; run: $bin/speakerguardd-session --machine $m (after a passing speakerguardd-selfcheck)"
  ;;
remove)
  rm -f "$pw/sgv2-$m.conf" "$wp/60-sgv2-$m.conf" "$bin/speakerguardd-session"
  systemctl --user restart pipewire wireplumber
  echo removed
  ;;
*) echo "usage: $0 install|remove <machine>"; exit 2 ;;
esac
