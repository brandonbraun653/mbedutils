project(mbedutils VERSION 0.0.1 LANGUAGES CXX)

# --------------------------------------------------------------------------------
# Allow NanoPB to be found. This requires submodules to be initialized.
# --------------------------------------------------------------------------------
set(NANOPB_SRC_ROOT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${NANOPB_SRC_ROOT_FOLDER}/extra)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/flashdb/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/flashdb/port/fal/inc
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
add_subdirectory(src/database)
add_subdirectory(src/logging)
add_subdirectory(src/hardware)
add_subdirectory(src/memory)
add_subdirectory(src/rpc)
add_subdirectory(src/threading)

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
