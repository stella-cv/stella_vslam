# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/base.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/aruco.h
               "$<$<BOOL:${USE_ARUCO}>:${CMAKE_CURRENT_SOURCE_DIR}/aruco.cc>"
               "$<$<NOT:$<BOOL:${USE_ARUCO}>>:${CMAKE_CURRENT_SOURCE_DIR}/aruco_disabled.cc>"
               )

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/module)
