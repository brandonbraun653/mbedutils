# Find the installation directory of the nanopb library within this virtual environment.
import sys
import os
import nanopb

__nanopb_path = os.path.dirname(nanopb.__file__)
__generator_dir = os.path.join(__nanopb_path, "generator")
__nanopb_proto_dir = os.path.join(__generator_dir, "proto")

# Add the nanopb generator directory to the system path.
if __generator_dir not in sys.path:
    sys.path.append(__generator_dir)

# Add the nanopb proto directory to the system path.
if __nanopb_proto_dir not in sys.path:
    sys.path.append(__nanopb_proto_dir)
