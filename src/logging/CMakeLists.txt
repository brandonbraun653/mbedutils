# Core logging driver
add_library(${PROJECT_NAME}_logging STATIC logging_driver.cpp)
target_link_libraries(${PROJECT_NAME}_logging PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)

# Bare Metal Sinks
add_library(${PROJECT_NAME}_logging_bare_metal STATIC logging_sink_bare_metal.cpp)
target_link_libraries(${PROJECT_NAME}_logging_bare_metal PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)

# STL Sinks
add_library(${PROJECT_NAME}_logging_stl STATIC logging_sink_stl.cpp)
target_link_libraries(${PROJECT_NAME}_logging_stl PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)
