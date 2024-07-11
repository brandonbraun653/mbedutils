#!/bin/bash
# Builds the C and Python bindings for the proto files in this directory. It's not likely this script will need to be run manually,
# as the definitions are already included in the repository. However, if you need to make changes to the proto files, you can run
# this script to regenerate the bindings.
#
# It expects a few dependencies to be installed:
# - protobuf
# - grpcio-tools
# - mypy-protobuf
# - nanopb  (This should be found locally in the lib/nanopb directory)
#
# You can install these system wide with the following command:
#   pip install protobuf grpcio-tools mypy-protobuf
#
# However, It's recommended to use a project-specific virtual environment,
# then calling this script from within it. This way, you can avoid conflicts
# with other projects that may require different versions of these dependencies.

# Expects to be called from this directory
_cwd=$(pwd)

SRC_DIR=$_cwd
NPB_ROOT=$_cwd/../../../lib/nanopb
NPB_INC=$NPB_ROOT/generator/proto
DST_DIR=$_cwd

# Build the C bindings
python $NPB_ROOT/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" mbed_rpc.proto


# Build the Python bindings with mypy definitions
protoc -I="$SRC_DIR" -I="$NPB_INC" --python_out="$DST_DIR" --mypy_out="$DST_DIR" "$SRC_DIR"/*.proto