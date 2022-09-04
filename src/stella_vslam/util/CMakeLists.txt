# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/angle.h
               ${CMAKE_CURRENT_SOURCE_DIR}/converter.h
               ${CMAKE_CURRENT_SOURCE_DIR}/fancy_index.h
               ${CMAKE_CURRENT_SOURCE_DIR}/image_converter.h
               ${CMAKE_CURRENT_SOURCE_DIR}/random_array.h
               ${CMAKE_CURRENT_SOURCE_DIR}/sqlite3.h
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo_rectifier.h
               ${CMAKE_CURRENT_SOURCE_DIR}/string.h
               ${CMAKE_CURRENT_SOURCE_DIR}/trigonometric.h
               ${CMAKE_CURRENT_SOURCE_DIR}/yaml.h
               ${CMAKE_CURRENT_SOURCE_DIR}/angle.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/converter.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/image_converter.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/random_array.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/sqlite3.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo_rectifier.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/yaml.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/util)
