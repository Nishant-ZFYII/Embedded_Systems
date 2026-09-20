# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# btstack ships as its own IDF component inside bluepad32's submodule and must
# be on the component path too, or bluepad32 fails to resolve it.
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/external/bluepad32/src/components
    ${CMAKE_CURRENT_SOURCE_DIR}/external/bluepad32/external/btstack/port/esp32/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(blink)
