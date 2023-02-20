# ----- Find dependencies -----

# spdlog
set(spdlog_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/spdlog/include)

# json
set(json_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/json/include)

# tinycolormap
set(tinycolormap_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/tinycolormap/include)

# sqlite3
find_package(SQLite3 REQUIRED)

# g2o
find_package(g2o REQUIRED
             COMPONENTS
             g2o::core
             g2o::stuff
             g2o::types_sba
             g2o::types_sim3
             g2o::solver_dense
             g2o::solver_eigen
             g2o::solver_csparse
             g2o::csparse_extension
             OPTIONAL_COMPONENTS
             g2o::csparse)

find_package(spdlog REQUIRED)

# gtsam
set(USE_GTSAM ON CACHE BOOL "Enable gtsam")
unset(GTSAM_INCLUDE_DIRS CACHE)
find_path(GTSAM_INCLUDE_DIRS NAMES gtsam
          PATHS /usr/local/include/)
if(USE_GTSAM AND GTSAM_INCLUDE_DIRS)
    message(STATUS "gtsam: enabled (Found in ${GTSAM_INCLUDE_DIRS})")
else()
    set(USE_GTSAM OFF)
    message(STATUS "gtsam: disabled")
endif()

# Check first if CSparse is built from g2o
if(TARGET g2o::csparse)
    set(${CXSPARSE_LIBRARIES} g2o::csparse)
else()
    # CXSparse for g2o
    find_package(CXSparse)
    if(CXSPARSE_FOUND)
        include_directories(${CXSPARSE_INCLUDE_DIRS})
    endif()
    # SuiteSparse for g2o
    if(NOT SUITESPARSE_FOUND)
        if(MSVC)
            find_package(SuiteSparse CONFIG)
        else()
            find_package(SuiteSparse)
        endif()
        if(SUITESPARSE_FOUND)
            include_directories(${SUITESPARSE_INCLUDE_DIRS})
        endif()
    endif()
endif()

# Check BoW Framework
if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow QUIET)
    if(NOT fbow_FOUND)
        add_subdirectory(${PROJECT_SOURCE_DIR}/3rd/FBoW ${PROJECT_BINARY_DIR}/3rd/FBoW)
    endif()
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()

# ----- Create stella_vslam library -----

add_library(${PROJECT_NAME}
            ${CMAKE_CURRENT_SOURCE_DIR}/config.h
            ${CMAKE_CURRENT_SOURCE_DIR}/type.h
            ${CMAKE_CURRENT_SOURCE_DIR}/system.h
            ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/config.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/system.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.cc)

# Set output directory of the library
set_target_properties(${PROJECT_NAME} PROPERTIES
                      OUTPUT_NAME stella_vslam
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# ----- Compile configuration -----

# gtsam
if (USE_GTSAM)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_GTSAM)
endif()

# OpenMP
set(USE_OPENMP OFF CACHE BOOL "Use OpenMP")
if(USE_OPENMP)
    target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)
    message(STATUS "OpenMP: ENABLED")
else()
    message(STATUS "OpenMP: DISABLED")
endif()

set(USE_SSE_ORB OFF CACHE BOOL "Enable SSE3 instruction for ORB extraction")
if(USE_SSE_ORB)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-msse3>)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_SSE_ORB)
    message(STATUS "SSE3 for ORB extraction: ENABLED")
else()
    message(STATUS "SSE3 for ORB extraction: DISABLED")
endif()

set(USE_SSE_FP_MATH OFF CACHE BOOL "Enable SSE instruction for floating-point operation")
if(USE_SSE_FP_MATH)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-mfpmath=sse>)
    message(STATUS "SSE for floating-point operation: ENABLED")
else()
    message(STATUS "SSE for floating-point operation: DISABLED")
endif()

if(BOW_FRAMEWORK MATCHES "DBoW2")
    set(BoW_LIBRARY ${DBoW2_LIBS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DBOW2)
    message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${DBoW2_INCLUDE_DIRS})")
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    if(fbow_FOUND)
        set(BoW_LIBRARY ${fbow_LIBS})
        message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${fbow_INCLUDE_DIRS})")
    else()
        target_include_directories(${PROJECT_NAME}
                                PUBLIC
                                "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/FBoW/include>"
                                "$<INSTALL_INTERFACE:include/fbow>")
        set(BoW_LIBRARY fbow)
        message(STATUS "BoW framework: ${BOW_FRAMEWORK} (Using submodule)")
    endif()
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()

if(USE_ARUCO)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_ARUCO)
endif()

# Trace level log
set (ENABLE_TRACE_LEVEL_LOG OFF CACHE BOOL "Enable trace level log")
if (ENABLE_TRACE_LEVEL_LOG)
    # Remove SPDLOG_TRACE macro at compile time by switching SPDLOG_ACTIVE_LEVEL.
    # NOTE: SPDLOG_ACTIVE_LEVEL has no effect on `spdlog::debug(...)`, `spdlog::trace(...)`, etc.
    target_compile_definitions(${PROJECT_NAME} PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
    message(STATUS "trace level log: ENABLED")
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
    message(STATUS "trace level log: DISABLED")
endif()

# Determinism
set (DETERMINISTIC OFF CACHE BOOL "Use deterministic code")
if (DETERMINISTIC)
    target_compile_definitions(${PROJECT_NAME} PUBLIC DETERMINISTIC)
    message(STATUS "Determinism: ENABLED")
else()
    message(STATUS "Determinism: DISABLED")
endif()

# ----- Configure stella_vslam library -----

# Include directories
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           "$<BUILD_INTERFACE:${json_INCLUDE_DIR}>"
                           "$<BUILD_INTERFACE:${tinycolormap_INCLUDE_DIR}>"
                           "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>"
                           "$<INSTALL_INTERFACE:include/stella_vslam/3rd/json/include>"
                           "$<INSTALL_INTERFACE:include/stella_vslam/3rd/spdlog/include>"
                           "$<INSTALL_INTERFACE:include/>")

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      Threads::Threads
                      OpenMP::OpenMP_CXX
                      Eigen3::Eigen
                      yaml-cpp
                      ${SQLite3_LIBRARIES}
                      opencv_core
                      opencv_features2d
                      opencv_calib3d
                      "$<$<BOOL:${USE_ARUCO}>:opencv_aruco>"
                      g2o::core
                      g2o::stuff
                      g2o::types_sba
                      g2o::types_sim3
                      g2o::solver_dense
                      g2o::solver_eigen
                      g2o::solver_csparse
                      g2o::csparse_extension
                      "$<$<BOOL:${USE_GTSAM}>:gtsam>"
                      ${CXSPARSE_LIBRARIES}
                      ${SUITESPARSE_LIBRARIES}
                      ${BoW_LIBRARY}
                      PRIVATE
                      ${LAPACK_LIBRARIES})

# ----- Install configuration -----

set(STELLA_VSLAM_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/stella_vslam)

# Install stella_vslam library
install(TARGETS ${PROJECT_NAME}
        EXPORT ${STELLA_VSLAM_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install stella_vslam headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR})

# Install 3rd party headers
install(FILES ${json_INCLUDE_DIR}/nlohmann/json.hpp ${json_INCLUDE_DIR}/nlohmann/json_fwd.hpp
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/3rd/json/include/nlohmann)
install(DIRECTORY ${tinycolormap_INCLUDE_DIR}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/3rd/tinycolormap)
# install(DIRECTORY ${spdlog_INCLUDE_DIR}
#         DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/3rd/spdlog)

# ----- Append subdirectory -----

add_subdirectory(camera)
add_subdirectory(data)
add_subdirectory(feature)
add_subdirectory(initialize)
add_subdirectory(io)
add_subdirectory(marker_detector)
add_subdirectory(marker_model)
add_subdirectory(match)
add_subdirectory(module)
add_subdirectory(optimize)
add_subdirectory(publish)
add_subdirectory(solve)
add_subdirectory(util)

# ----- Configure IDE -----

if (CMAKE_GENERATOR MATCHES "Visual Studio")
    get_target_property(SRC_FILES ${PROJECT_NAME} SOURCES)

    # Preserve folder structure in Visual Studio 
    source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC_FILES})
endif()
