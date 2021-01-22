#!/usr/bin/env bash
# Script to run the Swift CI workflow OUTSIDE of CI server.
# Used by associated Swift DEV container
#

###
[[  -z "${SWIFT_CISB_ADAPTER_PATH}" ]] && export SWIFT_CISB_ADAPTER_PATH=".swift_ci/adaptee"

### Helpers
function Here() {
  set -o pipefail
  _self="$(basename -- $0 .sh)"; _relative="../../"
  _here="$( cd "$( dirname "${BASH_SOURCE[0]}")/$_relative" >/dev/null 2>&1 && pwd )"
}

declare -a on_exit_handlers
function OnExit()
{
  for i in "${on_exit_handlers[@]}"
  do
    echo "[$_self]::OnExit::Handle:[$i]"
    eval $i
  done
}
function AddOnExit()
{
  local n=${#on_exit_handlers[*]}
  on_exit_handlers[$n]="$*"
  [[ $n -eq 0 ]] && trap OnExit EXIT
}

function LoadConfiguration() {
  echo "[$_self]::Load::[$1]"
  if [[ -f "$1" ]]; then
    tmpfile=$(mktemp /tmp/pipeline-export-XXXXXXXXXX.properties)
    (cat $1 | awk '{ print "export "$0 }') > $tmpfile
    source $tmpfile
    rm -f $tmpfile
  fi

} 
function ConfigurePipeline() {

  _pipeline_tmp=$(mktemp /tmp/pipeline.XXXXXXXXXX) || \
  echo "[$_self] -- FAILED to create temporary pipeline file"

  AddOnExit "rm -f $_pipeline_tmp"; echo "[$_self] -- [$_pipeline_tmp]::[$SWIFT_CISB_ADAPTER_PATH] "

  cat >$_pipeline_tmp <<@
${SWIFT_CISB_ADAPTER_PATH}/scripts/setup.sh
${SWIFT_CISB_ADAPTER_PATH}/scripts/build.sh
${SWIFT_CISB_ADAPTER_PATH}/scripts/inspect.sh
${SWIFT_CISB_ADAPTER_PATH}/scripts/measure.sh
${SWIFT_CISB_ADAPTER_PATH}/scripts/publish.sh
${SWIFT_CISB_ADAPTER_PATH}/scripts/teardown.sh
@

LoadConfiguration "$_here/$SWIFT_CISB_ADAPTER_PATH/docker.properties"
LoadConfiguration "$_here/$SWIFT_CISB_ADAPTER_PATH/default.properties"
LoadConfiguration "$_here/$SWIFT_CISB_ADAPTER_PATH/workflow.properties"
LoadConfiguration "$_here/$SWIFT_CISB_ADAPTER_PATH/channel.properties"

printenv| sort| grep SWIFT_

}


###
### Main
Here; status=1

#### Pipeline Configuration
ConfigurePipeline

#### Pipeline Body
echo "[$_self] -- ENTER [$@]"

IFS=$'\n'
for stage in $(cat $_pipeline_tmp); do
  echo "[$_self] -- STAGE [$stage]"
  ${stage}; status=$?
  [[ $status -ne 0 ]] && break
done

echo "[$_self] -- LEAVE [$status]"
exit $status
