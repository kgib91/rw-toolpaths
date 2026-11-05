cmake_minimum_required(VERSION 3.20)
project(clipper2medial LANGUAGES CXX)
set(kernel "${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/Clipper2Medial")
add_library(clipper2medial SHARED medial_polygons.cpp
  "${kernel}/src/clipper.engine.cpp" "${kernel}/src/clipper.offset.cpp" "${kernel}/src/clipper.rectclip.cpp")
target_include_directories(clipper2medial PRIVATE "${kernel}/include")
target_compile_features(clipper2medial PRIVATE cxx_std_17)
set_target_properties(clipper2medial PROPERTIES CXX_VISIBILITY_PRESET hidden)
