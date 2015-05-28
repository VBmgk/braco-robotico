cmake_minimum_required(VERSION 2.8.10)

project(hanoisim)
set(DESCRIPTION "Hanoi 6-DOF Simulator")

option(BUILD_CLI "Build command line interface" ON)
option(BUILD_GUI "Build graphical user interface (requires OpenGL+GLFW)" ON)

# Super strict warnings =)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -pedantic-errors ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
#standard_paths(${PROJECT_SOURCE_DIR} bin lib)
#standard_paths(${PROJECT_BINARY_DIR} bin lib)

#require_cxx14()
require_cxx11()
require_c11()

# enable clang-color with ninja
if("${CMAKE_GENERATOR}" STREQUAL "Ninja" AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_definitions("-Xclang -fcolor-diagnostics")
endif()

# TODO: think of Windows/MSVC...
add_definitions("-D_GNU_SOURCE")

# Bullet
find_package(Bullet REQUIRED)
include_directories(SYSTEM ${BULLET_INCLUDE_DIR})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# OpenGL
find_package(OpenGL REQUIRED)

# GLEW
find_package(GLEW REQUIRED)
include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})

# GLFW
#find_package(PkgConfig REQUIRED)
#pkg_search_module(GLFW REQUIRED glfw3)
#include_directories(SYSTEM ${GLFW_INCLUDE_DIRS})
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
set(GLFW_INSTALL OFF CACHE BOOL "")
add_subdirectory(vendor/glfw)
include_directories(vendor/glfw/include)

# ImGui
include_directories(SYSTEM imgui)
set(IMGUI_LIB imgui)
add_library(${IMGUI_LIB}
    imgui/imconfig.h
    imgui/imgui.h
    imgui/stb_rect_pack.h
    imgui/stb_textedit.h
    imgui/stb_truetype.h
    imgui/imgui.cpp
)

add_executable(hanoisim
    src/main.cpp
    src/draw.cpp
    src/draw.h
    src/physics.h
    src/robot.h
    src/utils/colors.h
    src/utils/gl.h
)

target_link_libraries(hanoisim
    ${IMGUI_LIB}
    ${BULLET_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLFW_LIBRARIES}
    glfw
)

add_custom_target(run
  COMMAND hanoisim
  DEPENDS hanoisim
  WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
)

# vim: et sw=4 ts=3 sts=4
