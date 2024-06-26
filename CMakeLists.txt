project(mbedutils VERSION 0.0.1 LANGUAGES CXX)

# --------------------------------------------------------------------------------
# Use this library to reference all the headers. For integrating project defined
# headers, the project should extend the target_include_directories with the
# appropriate path.
#
# target_include_directories(mbedutils_headers INTERFACE <projec_header_path>)
# --------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_headers INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --------------------------------------------------------------------------------
# Logging Library
# --------------------------------------------------------------------------------
file(GLOB LOGGING_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/logging/*.cpp)
add_library(${PROJECT_NAME}_logging STATIC ${LOGGING_SOURCES})
target_link_libraries(${PROJECT_NAME}_logging PRIVATE ${PROJECT_NAME}_headers)

# --------------------------------------------------------------------------------
# Mutex Libraries
# These provide some default implementations for common projects.
# --------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_mutex_stl STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/mutex/mutex_stl.cpp)
target_link_libraries(${PROJECT_NAME}_mutex_stl PRIVATE ${PROJECT_NAME}_headers)

add_library(${PROJECT_NAME}_mutex_freertos STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/mutex/mutex_freertos.cpp)
target_link_libraries(${PROJECT_NAME}_mutex_freertos PRIVATE ${PROJECT_NAME}_headers)

add_library(${PROJECT_NAME}_mutex_pico STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/mutex/mutex_pico.cpp)
target_link_libraries(${PROJECT_NAME}_mutex_pico PRIVATE ${PROJECT_NAME}_headers)
