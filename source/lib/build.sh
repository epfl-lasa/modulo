#!/bin/bash
CURRPATH=${PWD}
cd "${CURRPATH}/state_representation/cpp" && mkdir -p build && cd build && cmake .. && make && sudo make install
cd "${CURRPATH}/dynamical_systems" && mkdir -p build && cd build && cmake .. && make && sudo make install