cmake_minimum_required(VERSION 3.25)
project(hello)

set(hello_SOURCES hello.c)
if (WIN32)
    list(APPEND hello_SOURCES hello.rc)
endif()
add_executable(hello ${hello_SOURCES})


if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Test compiling a file that exposes potential issues with case
    # sensitive paths. If files are exposed with the wrong casing, compiling
    # once will succeed, as MSVC in Wine treats paths case insensitively.
    # But a case sensitive dependency tracker in Ninja will force unnecessary
    # recompilation, if the files don't exist with the right casing.
    #
    # This is tested only with MSVC, not with clang-cl, as MSVC STL is
    # strict with what version of Clang it supports (currently requiring
    # Clang 15, while common repositories only provide Clang 14). The
    # header also triggers using a form of attributes for SAL that Clang
    # doesn't support.
    add_library(test_headers STATIC headers.cpp)
    # Disable the warning "C4467: usage of ATL attributes is deprecated".
    target_compile_options(test_headers PUBLIC -wd4467)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND DEFINED MSVCDIR)
    add_library(cxxmodules STATIC "${MSVCDIR}/modules/std.ixx")
    target_compile_options(cxxmodules PUBLIC /std:c++latest)
endif()
