#!/bin/bash

# This script expects a few dependencies to be installed:
# - protobuf
# - grpcio-tools
# - mypy-protobuf
# - nanopb  (This should be found locally in the lib/nanopb directory)
#
# You can install these dependencies with the following commands:
# pip install protobuf grpcio-tools mypy-protobuf

# Expects to be called from this directory
_cwd=$(pwd)

SRC_DIR=$_cwd
NPB_ROOT=$_cwd/../../../lib/nanopb
NPB_INC=$NPB_ROOT/generator/proto
DST_DIR=$_cwd

# Build the C bindings
echo "Building Nanopb C-Bindings"
python $NPB_ROOT/generator/nanopb_generator.py --cpp-descriptors --output-dir="$SRC_DIR" --proto-path="$SRC_DIR" mbed_rpc.proto


# Build the Python bindings
echo "Building Python Bindings"
protoc -I="$SRC_DIR" -I="$NPB_INC" -I"$SRC_DIR"/serial_interface.proto --python_out="$DST_DIR" --mypy_out="$DST_DIR" "$SRC_DIR"/*.proto