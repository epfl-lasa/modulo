#!/bin/bash
CURRPATH=${PWD}
#cd ~/lib/passive_ds_control && mkdir -p build && cd build && cmake cmake -Druntests=ON .. && make && sudo make install
cd ${CURRPATH}/protocol_buffers && mkdir -p build && cd build && cmake && make && sudo make install
cd ${CURRPATH}/state_representation && mkdir -p build && cd build && cmake && make && sudo make install
cd ${CURRPATH}/dynamical_systems && mkdir -p build && cd build && cmake && make && sudo make install