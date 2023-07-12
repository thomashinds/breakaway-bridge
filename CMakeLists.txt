cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(breakaway-bridge)

idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)