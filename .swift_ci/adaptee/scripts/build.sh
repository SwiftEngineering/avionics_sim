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
    mkdir -p build
    pushd build
      cmake .. && make
      status=$?
    popd

    # attempt to build jupyter notebooks into html, dont block build if fails
    python -m jupyter nbconvert --output-dir='./build/documentation' --execute --to html_embed documentation/*/*.ipynb

  set +x

echo "[$_self] - LEAVE [$status]"
exit ${status}
