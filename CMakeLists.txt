cmake_minimum_required(VERSION 3.10)
#Allow project version
cmake_policy(SET CMP0048 NEW)
project(PMDLogger VERSION 1.0.0)

find_package(CppLinuxSerial)
if (NOT CppLinuxSerial_FOUND)
	message(FATAL_ERROR "CppLinuxSerial library not found")
endif()

if(${CMAKE_C_COMPILER_ID} STREQUAL "GNU")
  message(STATUS "Using GNU compiler, compiler ID ${CMAKE_C_COMPILER_ID}")
  #For Ubuntu 1804 need to keep the c11 std for thread check
  set(CMAKE_C_FLAGS "-std=c11 -Wall")
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -march=native")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
else()
  message(FATAL_ERROR "Unsupported version of compiler")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_executable(pmd_test
	pmd-usb-device.cc
	main.cc)
target_link_libraries(pmd_test -lCppLinuxSerial)
