# Core logging driver
add_library(${PROJECT_NAME}_logging STATIC logging_driver.cpp)
target_link_libraries(${PROJECT_NAME}_logging PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)

# Bare Metal Sinks
add_library(${PROJECT_NAME}_logging_bare_metal STATIC logging_sink_bare_metal.cpp)
target_link_libraries(${PROJECT_NAME}_logging_bare_metal PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)

# STL Sinks
add_library(${PROJECT_NAME}_logging_stl STATIC logging_sink_stl.cpp)
target_link_libraries(${PROJECT_NAME}_logging_stl PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)

# TimeSeries Database
add_library(${PROJECT_NAME}_logging_tsdb STATIC logging_sink_tsdb.cpp)
target_link_libraries(${PROJECT_NAME}_logging_tsdb PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)
