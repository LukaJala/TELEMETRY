cmake_minimum_required(VERSION 3.16)
project(host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(network_utils_tests
    test_network_utils.c
    ../ReceiveTest/components/network/network_utils.c
)

target_include_directories(network_utils_tests PRIVATE
    ../ReceiveTest/components/network/include
)

add_test(NAME network_utils_tests COMMAND network_utils_tests)
