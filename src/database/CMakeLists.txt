file(GLOB DB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(${PROJECT_NAME}_lib_db STATIC
    ${DB_SOURCES}
    ${PROJECT_SOURCE_DIR}/lib/flashdb/port/fal/src/fal.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/port/fal/src/fal_flash.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/port/fal/src/fal_partition.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb_kvdb.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb_tsdb.c
    ${PROJECT_SOURCE_DIR}/lib/flashdb/src/fdb_utils.c
)
target_link_libraries(${PROJECT_NAME}_lib_db PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    etl::etl
)

target_include_directories(${PROJECT_NAME}_lib_db PUBLIC
    ${PROJECT_SOURCE_DIR}/lib/nanopb
)