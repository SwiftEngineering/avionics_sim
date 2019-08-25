#!/usr/bin/env  bash
# LOCAL DEVELOPMENT #

export WORKSPACE=$(pwd)
rm -rf publish; mkdir -p publish/reports/HTML; mkdir -p publish/reports/XML

pushd tests/test_runner

   rm -f test_*_test_*.html

   cmake -DCMAKE_BUILD_TYPE=Debug .
   cmake --build .

   # Unit test
   echo "unit"
   ./avionics_sim_test_runner -UnitTests
   cp -vf test_unit_test_*.html ${WORKSPACE}/publish/reports/HTML/unit-tests.html

   # Integration test
   echo "integration"
   ./avionics_sim_test_runner -IntegrationTests
   cp -vf test_integration_test*.html ${WORKSPACE}/publish/reports/HTML/integration-tests.html

   # Coverage
   echo "coverage"
   make avionics_sim_test_runner_coverage

popd

# CPP Check
echo "check"
cppcheck --enable=all --std=c++11 --std=c99 --std=posix --xml --xml-version=2 \
"$WORKSPACE/tests/unit_tests" \
"$WORKSPACE/src" -I "$WORKSPACE/include/avionics_sim" \
2> "$WORKSPACE/publish/reports/XML/unit_report_cppcheck.xml"




