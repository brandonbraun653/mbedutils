# Core logging driver
file(GLOB HW_DRIVER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(${PROJECT_NAME}_lib_hardware STATIC ${HW_DRIVER_SRC})
target_link_libraries(${PROJECT_NAME}_lib_hardware PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)