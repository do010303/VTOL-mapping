#!/bin/bash
# Builds cpp/vtol_core.cpp into scripts/vtol_core*.so (pybind11 + OpenMP; apt: python3-pybind11).
# Without it the scripts run the same maths in NumPy (scripts/fastcore.py), only slower.
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out="$here/../scripts/vtol_core$(python3-config --extension-suffix)"
c++ -O3 -march=native -Wall -shared -std=c++17 -fPIC -fopenmp $(python3 -m pybind11 --includes) \
	"$here/vtol_core.cpp" -o "$out"
echo "built $out"
