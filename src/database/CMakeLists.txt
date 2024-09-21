file(GLOB DB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(${PROJECT_NAME}_lib_db STATIC
    ${DB_SOURCES}
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb_kvdb.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb.c
)
target_link_libraries(${PROJECT_NAME}_lib_db PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)