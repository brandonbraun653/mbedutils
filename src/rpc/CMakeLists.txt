# Import NanoPB
find_package(Nanopb REQUIRED)

# Interface Headers
set(MBED_RPC_HEADERS ${PROJECT_NAME}_lib_rpc_headers)
add_library(${MBED_RPC_HEADERS} INTERFACE)
target_include_directories(${MBED_RPC_HEADERS} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cobs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/nanopb
    ${CMAKE_CURRENT_SOURCE_DIR}/proto
    ${NANOPB_INCLUDE_DIRS}
)

# Server Library
set(RPC_SERVER_LIB_NAME ${PROJECT_NAME}_lib_rpc_server)
add_library(${RPC_SERVER_LIB_NAME} STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rpc_server.cpp
)
target_link_libraries(${RPC_SERVER_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    ${MBED_RPC_HEADERS}
    etl::etl
)

# Client Library
set(RPC_CLIENT_LIB_NAME ${PROJECT_NAME}_lib_rpc_client)
add_library(${RPC_CLIENT_LIB_NAME} STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rpc_client.cpp
)
target_link_libraries(${RPC_CLIENT_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    ${MBED_RPC_HEADERS}
)

# Proto Library
file(GLOB RPC_PROTO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.c)
set(RPC_PROTO_LIB_NAME ${PROJECT_NAME}_lib_rpc_proto)

add_library(${RPC_PROTO_LIB_NAME} STATIC
    ${RPC_PROTO_SOURCES}
    rpc_message.cpp
)
target_link_libraries(${RPC_PROTO_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${MBED_RPC_HEADERS}
    etl::etl
)

# NanoPB Library
set(NANOPB_LIB_NAME ${PROJECT_NAME}_lib_rpc_nanopb)
add_library(${NANOPB_LIB_NAME} STATIC ${NANOPB_SRCS})
target_link_libraries(${NANOPB_LIB_NAME} PRIVATE
    ${MBED_RPC_HEADERS}
    etl::etl
)

# COBS Library
set(COBS_LIB_NAME ${PROJECT_NAME}_lib_rpc_cobs)
add_library(${COBS_LIB_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/cobs/cobs.c)
target_link_libraries(${COBS_LIB_NAME} PRIVATE
    ${MBED_RPC_HEADERS}
)