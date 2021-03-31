#!/usr/bin/env bash
function Here() {
  set -o pipefail
  _self="$(basename -- $0 .sh)"; _relative="../../../"
  _here="$( cd "$( dirname "${BASH_SOURCE[0]}")/$_relative" >/dev/null 2>&1 && pwd )"
}
### Main
Here; status=1
echo "[$_self] - ENTER [$@]"

set -x;
  status=0
set +x;

echo "[$_self] - LEAVE [$status]"
exit ${status}
