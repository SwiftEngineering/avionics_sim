#!/usr/bin/env bash
function Here() {
  set -o pipefail
  _self="$(basename -- $0 .sh)"; _relative="../../../"
  _here="$( cd "$( dirname "${BASH_SOURCE[0]}")/$_relative" >/dev/null 2>&1 && pwd )"
}
### Main
Here; status=1
echo "[$_self] - ENTER [$@]::[$_here]"
export SWIFT_CISB_NEXUS_CONFIGURATION=".swift_ci/workflow/resources/nexus/mock-021.yaml"

TEST_ARGUMENTS="\
commit=559bd7e url=ssh://git@bitbucket.swiftengineering.com:7999/bare/workflow.cisb \
image=swiftx/alpine-embedded-artifacts:v0 \
status=SUCCESS type=unstable"

echo "[$_self] - TEST  [****************]:MOCK"
set -x
.swift_ci/workflow/scripts/nexus.sh "mock=true name=test-mock-cisb $TEST_ARGUMENTS" && status_mock=0
set +x
echo "[$_self] - TEST  [****************]:MOCK[$status_mock]"

echo "[$_self] - TEST  [****************]:NOMOCK"
set -x
.swift_ci/workflow/scripts/nexus.sh "name=test-nomock-cisb $TEST_ARGUMENTS" && status_nomock=0
set +x
echo "[$_self] - TEST  [****************]:NOMOCK[$status_nomock]"


[[ $status_mock -eq 0 ]] && [[ $status_nomock -eq 0 ]] && status=0
echo "[$_self] - LEAVE [$status]"
exit ${status}

}
