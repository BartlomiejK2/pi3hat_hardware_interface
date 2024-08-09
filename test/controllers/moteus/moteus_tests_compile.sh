#!/bin/bash

g++ -o simple_moteus_test simple_moteus_test.cpp -I../../include ../../include/3rd_libs/pi3hat/pi3hat.cc  -lbcm_host
g++ -o single_moteus_bridge_test single_moteus_bridge_test.cpp -I../../include ../../include/3rd_libs/pi3hat/pi3hat.cc ../../src/controllers/ControllerBridge.cpp ../../src/controllers/wrappers/* -lbcm_host
g++ -o double_moteus_bridge_test double_moteus_bridge_test.cpp -I../../include ../../include/3rd_libs/pi3hat/pi3hat.cc ../../src/controllers/ControllerBridge.cpp ../../src/controllers/wrappers/* -lbcm_host