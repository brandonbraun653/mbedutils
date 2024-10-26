# Core Library
file(GLOB CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(${PROJECT_NAME}_lib_thread STATIC ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME}_lib_thread PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)