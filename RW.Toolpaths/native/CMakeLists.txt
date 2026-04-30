cmake_minimum_required(VERSION 3.16)
project(boost_voronoi_wrapper VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Boost.Polygon.Voronoi is header-only â€” only the headers are needed.
# Accept the include directory directly; no CMake FindBoost module required.
# Pass via:  cmake .. -DBOOST_INCLUDEDIR=/path/to/boost_root
# or set the BOOST_ROOT / BOOST_INCLUDEDIR environment variable.
if(NOT DEFINED BOOST_INCLUDEDIR OR BOOST_INCLUDEDIR STREQUAL "")
    foreach(_candidate
            "$ENV{BOOST_ROOT}"
            "$ENV{BOOST_INCLUDEDIR}"
            "C:/local/boost"
            "C:/boost"
            "/usr/include"
            "/usr/local/include")
        if(EXISTS "${_candidate}/boost/polygon/voronoi.hpp")
            set(BOOST_INCLUDEDIR "${_candidate}")
            break()
        endif()
    endforeach()
endif()

if(NOT EXISTS "${BOOST_INCLUDEDIR}/boost/polygon/voronoi.hpp")
    message(FATAL_ERROR
        "Cannot find boost/polygon/voronoi.hpp.\n"
        "Run setup-native.ps1 (Windows) or setup-native.sh (Linux) first,\n"
        "or pass -DBOOST_INCLUDEDIR=<path> to cmake.")
endif()

message(STATUS "Boost headers: ${BOOST_INCLUDEDIR}")

add_library(boostvoronoi SHARED boost_voronoi_wrapper.cpp)

target_include_directories(boostvoronoi PRIVATE "${BOOST_INCLUDEDIR}")

if(MSVC)
    target_compile_options(boostvoronoi PRIVATE /W3 /O2 /EHsc)
else()
    target_compile_options(boostvoronoi PRIVATE -O2 -Wall -fvisibility=hidden)
    set_target_properties(boostvoronoi PROPERTIES
        C_VISIBILITY_PRESET   hidden
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON)
endif()

# â”€â”€â”€ .NET runtime package layout â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Detect the target RID so the library lands in runtimes/{rid}/native/
# relative to the install prefix (defaults to the RW.Toolpaths project dir).

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(NET_RID "win-x64")
    else()
        set(NET_RID "win-x86")
    endif()
elseif(UNIX AND NOT APPLE)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(NET_RID "linux-x64")
    else()
        set(NET_RID "linux-arm64")
    endif()
elseif(APPLE)
    set(NET_RID "osx-x64")
else()
    set(NET_RID "unknown")
endif()

message(STATUS "Target .NET RID: ${NET_RID}")

# Default install prefix to the RW.Toolpaths project directory so that
# `cmake --install .` drops the binary in the right place without extra flags.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/.."
        CACHE PATH "Install prefix (RW.Toolpaths project dir)" FORCE)
endif()

install(TARGETS boostvoronoi
    RUNTIME DESTINATION "runtimes/${NET_RID}/native"   # .dll  (Windows)
    LIBRARY DESTINATION "runtimes/${NET_RID}/native")  # .so   (Unix)

