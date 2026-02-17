#!/bin/bash
set -ex

ROOT_DIR=/workspace/src/route_planner/PCT_planner/pct_planner/planner

cd ${ROOT_DIR}/lib

rm -rf build
mkdir -p build
cd build

cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j2

echo ""
echo "=== Copying .so files ==="
cp ./src/a_star/a_star*.so ../  2>/dev/null || echo "WARN: a_star pybind .so not found"
cp ./src/a_star/liba_star_search.so ../ 2>/dev/null || echo "WARN: liba_star_search.so not found"
cp ./src/trajectory_optimization/traj_opt*.so ../ 2>/dev/null || echo "WARN: traj_opt .so not found"
cp ./src/trajectory_optimization/libgpmp_optimizer.so ../ 2>/dev/null || echo "WARN: libgpmp_optimizer.so not found"
cp ./src/ele_planner/ele_planner*.so ../ 2>/dev/null || echo "WARN: ele_planner .so not found"
cp ./src/ele_planner/libele_planner_lib.so ../ 2>/dev/null || echo "WARN: libele_planner_lib.so not found"
cp ./src/map_manager/py_map_manager*.so ../ 2>/dev/null || echo "WARN: py_map_manager .so not found"
cp ./src/map_manager/libmap_manager.so ../ 2>/dev/null || echo "WARN: libmap_manager.so not found"
cp ./src/common/smoothing/libcommon_smoothing.so ../ 2>/dev/null || echo "WARN: libcommon_smoothing.so not found"

echo ""
echo "=== Built .so files ==="
ls -la ${ROOT_DIR}/lib/*.so 2>/dev/null || echo "No .so files found"

echo ""
echo "=== Verify pybind modules ==="
export LD_LIBRARY_PATH=${ROOT_DIR}/lib:${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib:${ROOT_DIR}/lib/build/src/common/smoothing
export PYTHONPATH=${ROOT_DIR}/lib
python3 -c "import a_star; print('a_star OK')" 2>&1 || echo "a_star import failed"
python3 -c "import traj_opt; print('traj_opt OK')" 2>&1 || echo "traj_opt import failed"
python3 -c "import ele_planner; print('ele_planner OK')" 2>&1 || echo "ele_planner import failed"
python3 -c "import py_map_manager; print('py_map_manager OK')" 2>&1 || echo "py_map_manager import failed"

echo ""
echo "=== PCT C++ modules build DONE ==="
