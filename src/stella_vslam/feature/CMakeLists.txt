# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/params_factory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/params.h
               ${CMAKE_CURRENT_SOURCE_DIR}/default_params.h
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_params.h
               ${CMAKE_CURRENT_SOURCE_DIR}/sift_params.h
               ${CMAKE_CURRENT_SOURCE_DIR}/akaze_params.h
               ${CMAKE_CURRENT_SOURCE_DIR}/extractor_factory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/extractor.h
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_extractor.h
               ${CMAKE_CURRENT_SOURCE_DIR}/sift_extractor.h
               ${CMAKE_CURRENT_SOURCE_DIR}/akaze_extractor.h
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_impl.h
               ${CMAKE_CURRENT_SOURCE_DIR}/feature_type.h
               ${CMAKE_CURRENT_SOURCE_DIR}/params.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_params.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/sift_params.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/akaze_params.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_extractor.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/sift_extractor.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/akaze_extractor.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/orb_impl.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/feature)
