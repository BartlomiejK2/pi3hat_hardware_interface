#!/bin/bash

g++ -o simple_moteus_test simple_moteus_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc  -lbcm_host  
g++ -o single_moteus_bridge_test single_moteus_wrapper_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc ../../../src/controllers/wrappers/* -lbcm_host -DTEST_MOTEUS_WRAPPER
g++ -o double_moteus_bridge_test double_moteus_wrapper_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc ../../../src/controllers/wrappers/* -lbcm_host -DTEST_MOTEUS_WRAPPER