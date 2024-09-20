#!/bin/bash

g++ -o exe/simple_moteus_test simple_moteus_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc  -lbcm_host  
g++ -o exe/single_moteus_wrapper_test single_moteus_wrapper_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc ../../../src/controllers/wrappers/* -lbcm_host -DTEST_MOTEUS_WRAPPER
g++ -o exe/double_moteus_wrapper_test double_moteus_wrapper_test.cpp -I../../../include ../../../include/3rd_libs/pi3hat/pi3hat.cc ../../../src/controllers/wrappers/* -lbcm_host -DTEST_MOTEUS_WRAPPER