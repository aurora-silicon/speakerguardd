#!/bin/bash
# Clean-publish check: no Apple tuning data, or anything shaped like it, may
# enter this repository. Run by CI on every push and pull request.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
fail=0
while IFS= read -r -d '' f; do
  case "$f" in
    *lsm2-param-fill.bin|*.austrip|*.dspg|*.propstrip|*.aupreset|*speaker-thiele-small.bin|*aufx-*-appl.plist)
      echo "forbidden tuning file: $f"; fail=1 ;;
  esac
  case "/$f" in
    */AID[0-9]*/*) echo "forbidden AID folder content: $f"; fail=1 ;;
  esac
  if [[ -f $f ]] && head -c 8 -- "$f" | grep -q '^bplist0'; then
    echo "binary property list: $f"; fail=1
  fi
done < <(git ls-files -z)
exit "$fail"
