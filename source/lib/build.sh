#!/bin/bash
CURRPATH=${PWD}
# cpp
cd "${CURRPATH}/state_representation" && mkdir -p build && cd build && cmake .. && make -j && sudo make install
cd "${CURRPATH}/dynamical_systems" && mkdir -p build && cd build && cmake .. && make -j && sudo make install
