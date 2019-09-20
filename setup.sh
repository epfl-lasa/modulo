#!/bin/bash
CURRPATH=${PWD}
git submodule update --init --recursive
cd ${CURRPATH}/lib/dynamical_systems && git checkout master && git pull 
cd ${CURRPATH}/lib/protocol_buffers && git checkout master && git pull
cd ${CURRPATH}/lib/state_representation && git checkout master && git pull
cd ${CURRPATH}/src/modulo_core && git checkout master && git pull
cd ${CURRPATH}/src/modulo_msgs && git checkout master && git pull