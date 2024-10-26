# Core non-volatile memory driver
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/nvm/*.cpp)
add_library(${PROJECT_NAME}_lib_nvm STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME}_lib_nvm PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)