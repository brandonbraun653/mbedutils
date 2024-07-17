# Each generated python binding file for mbedutils has a statement near the top like this:
#   DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(<bunch of bytes>)
#
# This is loading a serialized file descriptor into the descriptor pool, and our code has a dependency on
# files loaded by nanopb_pb2. If that's not done first, our project tries to reference a descriptor that doesn't exist.
# We can fix this by importing nanopb_pb2 before importing the generated python binding file.
from nanopb.generator.proto import nanopb_pb2