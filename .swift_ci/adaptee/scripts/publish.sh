#!/usr/bin/env bash
function Here() {
  set -o pipefail
  _self="$(basename -- $0 .sh)"; _relative="../../../"
  _here="$( cd "$( dirname "${BASH_SOURCE[0]}")/$_relative" >/dev/null 2>&1 && pwd )"
}
### Main
Here; status=1;
mkdir -p publish/lib
echo "[$_self] - ENTER [$@]"
cp -vrf build/publish/* publish/
cp build/libavionics_sim.a publish/lib/
cp -r build/documentation publish/
status=0
echo "[$_self] - LEAVE [$status]"
exit ${status}
