# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mbed_rpc.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0embed_rpc.proto\x12\x08mbed.rpc\x1a\x0cnanopb.proto\"u\n\x06Header\x12\x12\n\x03\x63rc\x18\x01 \x02(\rB\x05\x92?\x02\x38\x10\x12\x13\n\x04size\x18\x02 \x02(\rB\x05\x92?\x02\x38\x10\x12\x16\n\x07version\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05seqId\x18\x04 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05msgId\x18\x05 \x02(\rB\x05\x92?\x02\x38\x10\"/\n\x0b\x42\x61seMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\";\n\x04Ping\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x11\n\ttimestamp\x18\x02 \x02(\r\"8\n\x14ListFunctionsRequest\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\"L\n\x15ListFunctionsResponse\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x11\n\tfunctions\x18\x02 \x03(\t*\x19\n\tErrorCode\x12\x0c\n\x08NO_ERROR\x10\x00*\x1f\n\x0f\x42uiltinServices\x12\x0c\n\x08SVC_PING\x10\x00*\x1f\n\x0f\x42uiltinMessages\x12\x0c\n\x08MSG_PING\x10\x00')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'mbed_rpc_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_HEADER'].fields_by_name['crc']._options = None
  _globals['_HEADER'].fields_by_name['crc']._serialized_options = b'\222?\0028\020'
  _globals['_HEADER'].fields_by_name['size']._options = None
  _globals['_HEADER'].fields_by_name['size']._serialized_options = b'\222?\0028\020'
  _globals['_HEADER'].fields_by_name['version']._options = None
  _globals['_HEADER'].fields_by_name['version']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['seqId']._options = None
  _globals['_HEADER'].fields_by_name['seqId']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['msgId']._options = None
  _globals['_HEADER'].fields_by_name['msgId']._serialized_options = b'\222?\0028\020'
  _globals['_ERRORCODE']._serialized_start=407
  _globals['_ERRORCODE']._serialized_end=432
  _globals['_BUILTINSERVICES']._serialized_start=434
  _globals['_BUILTINSERVICES']._serialized_end=465
  _globals['_BUILTINMESSAGES']._serialized_start=467
  _globals['_BUILTINMESSAGES']._serialized_end=498
  _globals['_HEADER']._serialized_start=42
  _globals['_HEADER']._serialized_end=159
  _globals['_BASEMESSAGE']._serialized_start=161
  _globals['_BASEMESSAGE']._serialized_end=208
  _globals['_PING']._serialized_start=210
  _globals['_PING']._serialized_end=269
  _globals['_LISTFUNCTIONSREQUEST']._serialized_start=271
  _globals['_LISTFUNCTIONSREQUEST']._serialized_end=327
  _globals['_LISTFUNCTIONSRESPONSE']._serialized_start=329
  _globals['_LISTFUNCTIONSRESPONSE']._serialized_end=405
# @@protoc_insertion_point(module_scope)