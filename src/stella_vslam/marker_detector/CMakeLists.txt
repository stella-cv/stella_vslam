# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/base.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/aruco.h
               "$<$<BOOL:${USE_ARUCO}>:${CMAKE_CURRENT_SOURCE_DIR}/aruco.cc>"
               "$<$<NOT:$<BOOL:${USE_ARUCO}>>:${CMAKE_CURRENT_SOURCE_DIR}/aruco_disabled.cc>"
               )

if (USE_ARUCO_NANO)
    target_sources(${PROJECT_NAME}
	           PRIVATE
                   ${CMAKE_CURRENT_SOURCE_DIR}/aruconano.h
                   ${CMAKE_CURRENT_SOURCE_DIR}/aruconano.cc)
endif()

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/marker_detector)
