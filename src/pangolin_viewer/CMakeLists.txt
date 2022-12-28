# ----- Find dependencies -----

# Pangolin
find_package(Pangolin REQUIRED)

# ----- Configure PangolinViewer library -----

add_library(pangolin_viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.h
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.h
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.cc)

set_target_properties(pangolin_viewer PROPERTIES
                      OUTPUT_NAME pangolin_viewer
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
if(${Pangolin_VERSION} VERSION_GREATER 0.5)
    message(STATUS "Build pangolin_viewer with c++17")
    set_property(TARGET pangolin_viewer PROPERTY CXX_STANDARD 17)
endif()

target_include_directories(pangolin_viewer 
                           PUBLIC
                           ${Pangolin_INCLUDE_DIR})

if(${Pangolin_VERSION} VERSION_GREATER 0.5)
    target_link_libraries(pangolin_viewer
                          PUBLIC
                          ${PROJECT_NAME}
                          opencv_highgui
                          pango_display
                          pango_opengl)
else()
    target_link_libraries(pangolin_viewer
                    PUBLIC
                    ${PROJECT_NAME}
                    opencv_highgui
                    pangolin)
endif()

# ----- Install configuration -----

if(INSTALL_PANGOLIN_VIEWER)
    set(PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_viewer)

    install(TARGETS pangolin_viewer
            EXPORT ${STELLA_VSLAM_TARGETS_EXPORT_NAME}
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})

    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    install(FILES ${HEADERS}
            DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})
endif()
