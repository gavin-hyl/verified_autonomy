#!/bin/bash
set -ex

echo "=========================================="
echo "Building PCT Planner Third-party Libraries"
echo "(Low memory mode - 2 parallel jobs)"
echo "=========================================="

cd /workspace/src/route_planner/PCT_planner/pct_planner/planner

echo ""
echo "=== Building GTSAM ==="
cd lib/3rdparty/gtsam-4.1.1
rm -rf build install
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j2 && make install
echo "GTSAM build DONE"

echo ""
echo "=== Building OSQP ==="
cd /workspace/src/route_planner/PCT_planner/pct_planner/planner/lib/3rdparty/osqp
rm -rf build install
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release
make -j2 && make install
echo "OSQP build DONE"

echo ""
echo "=== Third-party builds complete ==="
ls -la /workspace/src/route_planner/PCT_planner/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/ | head -10
ls -la /workspace/src/route_planner/PCT_planner/pct_planner/planner/lib/3rdparty/osqp/install/lib/
