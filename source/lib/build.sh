#!/bin/bash
CURRPATH=${PWD}
# cpp
cd "${CURRPATH}/state_representation/cpp" && mkdir -p build && cd build && cmake .. && make && sudo make install
cd "${CURRPATH}/dynamical_systems" && mkdir -p build && cd build && cmake .. && make && sudo make install
cd "${CURRPATH}/sml" && mkdir -p build && cd build && cmake .. && make && sudo make install
# python
cd "${CURRPATH}/state_representation/python" && pip3 install -U -r requirements.txt && python3 setup.py develop --user