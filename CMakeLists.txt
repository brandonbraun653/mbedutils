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
# Internal header interface library. These are used for compiling the mbedutils
# library, but are not implicitly exposed to the consumer. There may be some
# conflicts if the project uses some of the same targets.
# --------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_internal_headers INTERFACE)
target_include_directories(${PROJECT_NAME}_internal_headers INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanoprintf
)

# --------------------------------------------------------------------------------
# Sub-Libraries
# --------------------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/logging)
add_subdirectory(src/hardware)
add_subdirectory(src/rpc)

# --------------------------------------------------------------------------------
# Interfaces Library
# These provide some (but not all) interface definitions for common runtimes.
# Include in your project as needed/desired.
# --------------------------------------------------------------------------------

# STL Based Library
file(GLOB STL_INTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/stl/*.cpp)
add_library(${PROJECT_NAME}_lib_stl STATIC ${STL_INTF_SOURCES})
target_link_libraries(${PROJECT_NAME}_lib_stl PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
)

# FreeRTOS Based Library
file(GLOB FREERTOS_INTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/freertos/*.cpp)
add_library(${PROJECT_NAME}_lib_freertos STATIC ${FREERTOS_INTF_SOURCES})
target_link_libraries(${PROJECT_NAME}_lib_freertos PRIVATE
    ${PROJECT_NAME}_headers
    ${PROJECT_NAME}_internal_headers
    freertos_config  # Standard FreeRTOS integration target. Defined by the project for headers.
)

# RPI Pico Based Library
if( PICO_SDK_PATH )
    file(GLOB PICO_INTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/pico/*.cpp)
    add_library(${PROJECT_NAME}_lib_pico STATIC ${PICO_INTF_SOURCES})
    target_link_libraries(${PROJECT_NAME}_lib_pico PRIVATE
        ${PROJECT_NAME}_headers
        ${PROJECT_NAME}_internal_headers

        # These targets are defined in the pico-sdk CMake files after importing.
        hardware_dma_headers
        hardware_timer_headers
        hardware_uart_headers
        pico_sync_headers
    )
endif()

# --------------------------------------------------------------------------------
# Tooling
# --------------------------------------------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(MBEDUTILS_DOXYGEN_INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    set(MBEDUTILS_DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/docs)
    set(MBEDUTILS_DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(MBEDUTILS_DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${MBEDUTILS_DOXYFILE_IN} ${MBEDUTILS_DOXYFILE_OUT} @ONLY)
    add_custom_command(
    OUTPUT ${MBEDUTILS_DOXYGEN_OUTPUT_DIR}/html/index.xml
    COMMAND ${DOXYGEN_EXECUTABLE} ${MBEDUTILS_DOXYFILE_OUT}
    MAIN_DEPENDENCY ${MBEDUTILS_DOXYFILE_OUT} ${MBEDUTILS_DOXYFILE_IN}
    COMMENT "Generating mbedutils documentation"
    VERBATIM)

    add_custom_target(mbedutils_docs ALL DEPENDS ${MBEDUTILS_DOXYGEN_OUTPUT_DIR}/html/index.xml)
else()
    message(STATUS "Doxygen needs to be installed to generate the documentation")
endif()
