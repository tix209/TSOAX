cmake_minimum_required(VERSION 3.10)

project(TSOAX)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  if(MSVC) # MSVC compiler (Win32 only)
    # Display more warnings
    set(CMAKE_CXX_FLAGS "/W3")
  elseif(UNIX OR CMAKE_COMPILER_IS_GNUCXX)
    # Clang OR Gcc (Linux, Mac OS or Win32 with MingW)
    # Displays all warnings
    set(CMAKE_CXX_FLAGS "-O0 -pedantic -Wall")
  endif()
endif()

# Enable C++14
if(UNIX OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -std=c++14")
endif()

option(BUILD_TEST "Build test routines for Tsoax." OFF)

if(BUILD_TEST)
  add_subdirectory(test)
  add_subdirectory(googletest)
endif()

include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(src)
