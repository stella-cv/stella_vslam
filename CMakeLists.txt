cmake_minimum_required(VERSION 3.1)
project(stella_vslam LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 OLD)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ----- Set build type -----

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(BUILD_SHARED_LIBS "Build stella_vslam as a shared library" ON)

# ----- Build selection -----

set(USE_PANGOLIN_VIEWER OFF CACHE BOOL "Enable Pangolin Viewer")
set(INSTALL_PANGOLIN_VIEWER OFF CACHE BOOL "Install PangolinViewer library")
set(USE_SOCKET_PUBLISHER OFF CACHE BOOL "Enable Socket Publisher")
set(INSTALL_SOCKET_PUBLISHER OFF CACHE BOOL "Install SocketPublisher library")
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples")
set(BUILD_TESTS OFF CACHE BOOL "Build tests")
set(BOW_FRAMEWORK "FBoW" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")

# ----- Set options for debugging -----

set(USE_CCACHE ON CACHE BOOL "Use ccache to accelerate build")
find_program(CCACHE_EXE ccache)
if(USE_CCACHE AND CCACHE_EXE AND NOT MSVC)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_EXE}")
    message(STATUS "ccache: Enabled")
else()
    set(USE_CCACHE OFF)
    message(STATUS "ccache: Disabled")
endif()

set(USE_SANITIZER OFF CACHE BOOL "Enable Address/Memory sanitizer (set env as ASAN_OPTIONS=detect_leaks=1)")
if(USE_SANITIZER AND NOT MSVC)
    add_compile_options(-fno-omit-frame-pointer -fsanitize=address)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    message(STATUS "Address/Memory sanitizer: ENABLED")
else()
    message(STATUS "Address/Memory sanitizer: DISABLED")
endif()

# ----- Set compiler options -----

# Note: add_compile_options() accepts only "simple" generator-expressions :-(
if(MSVC)
    # C4251: DLL export, C4244: floating->integer, C4305: double->float, C4267: size_t->any, C4127: constant condition
    add_compile_options(/W4 $<$<CONFIG:Debug>:/MTd>;$<$<CONFIG:Release>:/MT> /source-charset:utf-8 /execution-charset:utf-8 /wd4251 /wd4244 /wd4305 /wd4267 /wd4127 /bigobj)
    add_compile_options("$<$<CONFIG:Debug>:/Zo>;$<$<CONFIG:Release>:/O2>;$<$<CONFIG:None>:/O2>")
    add_link_options(/NODEFAULTLIB:library)
else()
    add_compile_options(-Wall -Wextra)
    add_compile_options(
        "$<$<CONFIG:Debug>:-Og>"
        "$<$<CONFIG:Release>:-O3>"
        "$<$<CONFIG:None>:-O3>")
endif()

set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization")
if(BUILD_WITH_MARCH_NATIVE AND NOT MSVC)
    add_compile_options(-mtune=native -march=native)
    message(STATUS "Architecture-aware optimization: ENABLED")
else()
    message(STATUS "Architecture-aware optimization: DISABLED")
endif()

# ----- Find dependencies -----

# Threads
find_package(Threads REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

# Eigen
find_package(Eigen3 3.3 REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV 3.3.1 QUIET
             COMPONENTS
             core imgcodecs videoio features2d calib3d highgui)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 QUIET
                 COMPONENTS
                 core imgcodecs videoio features2d calib3d highgui)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# Check aruco module
set(USE_ARUCO ON CACHE BOOL "Enable aruco")
list(APPEND ARUCO_CHECK_INCLUDE_DIRS
     ${OpenCV_INSTALL_PATH}/include/
     ${OpenCV_INSTALL_PATH}/include/opencv4)
unset(ARUCO_INCLUDE_DIRS CACHE)
find_path(ARUCO_INCLUDE_DIRS NAMES aruco.hpp
          PATH_SUFFIXES opencv2
          PATHS ${ARUCO_CHECK_INCLUDE_DIRS})
if(USE_ARUCO AND ARUCO_INCLUDE_DIRS)
    message(STATUS "aruco: enabled (Found in ${ARUCO_INCLUDE_DIRS})")
else()
    set(USE_ARUCO OFF)
    message(STATUS "aruco: disabled")
endif()

# ----- Install configuration -----

include(CMakePackageConfigHelpers)

# Generate cmake configuration scripts
set(STELLA_VSLAM_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(STELLA_VSLAM_VERSION_CONFIG ${STELLA_VSLAM_GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake)
set(STELLA_VSLAM_PROJECT_CONFIG ${STELLA_VSLAM_GENERATED_DIR}/${PROJECT_NAME}Config.cmake)
set(STELLA_VSLAM_TARGETS_EXPORT_NAME ${PROJECT_NAME}Targets)
set(STELLA_VSLAM_CONFIG_INSTALL_DIR lib/cmake/${PROJECT_NAME})
set(STELLA_VSLAM_NAMESPACE "${PROJECT_NAME}::")
set(STELLA_VSLAM_VERSION 0.3.13)

# Create a version config file
write_basic_package_version_file(${STELLA_VSLAM_VERSION_CONFIG}
                                 VERSION ${STELLA_VSLAM_VERSION}
                                 COMPATIBILITY SameMajorVersion)
# Create a project config file
configure_file(${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in ${STELLA_VSLAM_PROJECT_CONFIG} @ONLY)

# Install to the specified directory
install(FILES ${STELLA_VSLAM_VERSION_CONFIG} ${STELLA_VSLAM_PROJECT_CONFIG}
        DESTINATION ${STELLA_VSLAM_CONFIG_INSTALL_DIR})
install(EXPORT ${STELLA_VSLAM_TARGETS_EXPORT_NAME}
        NAMESPACE ${STELLA_VSLAM_NAMESPACE}
        DESTINATION ${STELLA_VSLAM_CONFIG_INSTALL_DIR})

# Set standard installation directories
set(RUNTIME_DESTINATION bin)
set(LIBRARY_DESTINATION lib)
set(ARCHIVE_DESTINATION lib)
set(INCLUDES_DESTINATION include)

# ----- Build -----

add_subdirectory(src)

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
