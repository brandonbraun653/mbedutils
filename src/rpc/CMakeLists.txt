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
file(GLOB RPC_SERVER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/server/*.cpp)
file(GLOB RPC_SERVICE_BUILTIN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/services/*_service.cpp)
set(RPC_SERVER_LIB_NAME ${PROJECT_NAME}_lib_rpc_server)

add_library(${RPC_SERVER_LIB_NAME} STATIC
    ${RPC_SERVER_SOURCES}
    ${RPC_SERVICE_BUILTIN_SOURCES}
)
target_link_libraries(${RPC_SERVER_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    ${MBED_RPC_HEADERS}
)

# Client Library
file(GLOB RPC_CLIENT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/client/*.cpp)
set(RPC_CLIENT_LIB_NAME ${PROJECT_NAME}_lib_rpc_client)

add_library(${RPC_CLIENT_LIB_NAME} STATIC ${RPC_CLIENT_SOURCES})
target_link_libraries(${RPC_CLIENT_LIB_NAME} PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    ${MBED_RPC_HEADERS}
)

# Proto Library
file(GLOB RPC_PROTO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.c)
set(RPC_PROTO_LIB_NAME ${PROJECT_NAME}_lib_rpc_proto)

add_library(${RPC_PROTO_LIB_NAME} STATIC ${RPC_PROTO_SOURCES})
target_link_libraries(${RPC_PROTO_LIB_NAME} PRIVATE
    ${MBED_RPC_HEADERS}
)


# NanoPB Library
set(NANOPB_LIB_NAME ${PROJECT_NAME}_lib_rpc_nanopb)
add_library(${NANOPB_LIB_NAME} STATIC ${NANOPB_SRCS})
target_link_libraries(${NANOPB_LIB_NAME} PRIVATE
    ${MBED_RPC_HEADERS}
)