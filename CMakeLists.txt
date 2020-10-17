cmake_minimum_required(VERSION 3.10)
project(huffman)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")
add_executable(huffman_encode 9.cpp main_encode.cpp my.cpp)
add_executable(huffman_decode 9.cpp main_decode.cpp my.cpp)
