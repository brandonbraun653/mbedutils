# Import NanoPB
find_package(Nanopb REQUIRED)

# Server Library
file(GLOB RPC_SERVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/server/*.cpp)
set(RPC_SERVER_LIB_NAME ${PROJECT_NAME}_lib_rpc_server)

add_library(${RPC_SERVER_LIB_NAME} STATIC ${RPC_SERVER_SOURCES})
target_link_libraries(${RPC_SERVER_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)
target_include_directories(${RPC_SERVER_LIB_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cobs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/nanopb
    ${NANOPB_INCLUDE_DIRS}
)

# Client Library
file(GLOB RPC_CLIENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/client/*.cpp)
set(RPC_CLIENT_LIB_NAME ${PROJECT_NAME}_lib_rpc_client)

add_library(${RPC_CLIENT_LIB_NAME} STATIC ${RPC_CLIENT_SOURCES})
target_link_libraries(${RPC_CLIENT_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)
target_include_directories(${RPC_CLIENT_LIB_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cobs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/nanopb
    ${NANOPB_INCLUDE_DIRS}
)
