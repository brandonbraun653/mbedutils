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


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0embed_rpc.proto\x12\x08mbed.rpc\x1a\x0cnanopb.proto\"b\n\x06Header\x12\x16\n\x07version\x18\x01 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05seqId\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05msgId\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x05svcId\x18\x04 \x02(\rB\x05\x92?\x02\x38\x08\"/\n\x0b\x42\x61seMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\"s\n\x0c\x45rrorMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12)\n\x05\x65rror\x18\x02 \x02(\x0e\x32\x13.mbed.rpc.ErrorCodeB\x05\x92?\x02\x38\x08\x12\x16\n\x06\x64\x65tail\x18\x03 \x02(\x0c\x42\x06\x92?\x03\x08\x80\x02\"/\n\x0bPingMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\"\x8d\x01\n\x0e\x41\x63kNackMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x13\n\x0b\x61\x63knowledge\x18\x02 \x02(\x08\x12/\n\x0bstatus_code\x18\x03 \x01(\x0e\x32\x13.mbed.rpc.ErrorCodeB\x05\x92?\x02\x38\x08\x12\x13\n\x04\x64\x61ta\x18\x04 \x01(\x0c\x42\x05\x92?\x02\x08@\"=\n\x0bTickMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x0c\n\x04tick\x18\x02 \x02(\r\"\x80\x01\n\x0e\x43onsoleMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x19\n\nthis_frame\x18\x02 \x02(\rB\x05\x92?\x02\x38\x08\x12\x1b\n\x0ctotal_frames\x18\x03 \x02(\rB\x05\x92?\x02\x38\x08\x12\x14\n\x04\x64\x61ta\x18\x04 \x02(\x0c\x42\x06\x92?\x03\x08\x80\x01\"\x90\x01\n\x11SystemInfoMessage\x12 \n\x06header\x18\x01 \x02(\x0b\x32\x10.mbed.rpc.Header\x12\x1b\n\nsw_version\x18\x02 \x02(\tB\x07\x92?\x04\x08\x10x\x01\x12\x1e\n\rserial_number\x18\x03 \x02(\tB\x07\x92?\x04\x08\x10x\x01\x12\x1c\n\x0b\x64\x65scription\x18\x04 \x02(\tB\x07\x92?\x04\x08 x\x01*\'\n\x0fProtocolVersion\x12\x14\n\x10RPC_PROTOCOL_VER\x10\x01*\x82\x02\n\tErrorCode\x12\x10\n\x0c\x45RR_NO_ERROR\x10\x00\x12\x11\n\rERR_SVC_ASYNC\x10\x01\x12\x10\n\x0c\x45RR_SVC_BUSY\x10\x02\x12\x0f\n\x0b\x45RR_RPC_VER\x10\x03\x12\x0f\n\x0b\x45RR_MSG_VER\x10\x04\x12\x10\n\x0c\x45RR_MSG_SIZE\x10\x05\x12\x0f\n\x0b\x45RR_MSG_CRC\x10\x06\x12\x12\n\x0e\x45RR_MSG_DECODE\x10\x07\x12\x15\n\x11\x45RR_SVC_NOT_FOUND\x10\x08\x12\x15\n\x11\x45RR_MSG_NOT_FOUND\x10\t\x12\x0f\n\x0b\x45RR_SVC_MSG\x10\n\x12\x12\n\x0e\x45RR_SVC_FAILED\x10\x0b\x12\x12\n\rERR_MAX_ERROR\x10\xff\x01*\x1e\n\x0e\x42uiltinService\x12\x0c\n\x08SVC_PING\x10\x00*\x81\x01\n\x0e\x42uiltinMessage\x12\x0c\n\x08MSG_NULL\x10\x00\x12\r\n\tMSG_ERROR\x10\x01\x12\x0c\n\x08MSG_PING\x10\x02\x12\x10\n\x0cMSG_ACK_NACK\x10\x03\x12\x0c\n\x08MSG_TICK\x10\x04\x12\x0f\n\x0bMSG_CONSOLE\x10\x05\x12\x13\n\x0fMSG_SYSTEM_INFO\x10\x06*\x96\x01\n\x15\x42uiltinMessageVersion\x12\x11\n\rMSG_VER_ERROR\x10\x00\x12\x10\n\x0cMSG_VER_PING\x10\x00\x12\x14\n\x10MSG_VER_ACK_NACK\x10\x00\x12\x10\n\x0cMSG_VER_TICK\x10\x00\x12\x13\n\x0fMSG_VER_CONSOLE\x10\x00\x12\x17\n\x13MSG_VER_SYSTEM_INFO\x10\x00\x1a\x02\x10\x01')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'mbed_rpc_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_BUILTINMESSAGEVERSION']._options = None
  _globals['_BUILTINMESSAGEVERSION']._serialized_options = b'\020\001'
  _globals['_HEADER'].fields_by_name['version']._options = None
  _globals['_HEADER'].fields_by_name['version']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['seqId']._options = None
  _globals['_HEADER'].fields_by_name['seqId']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['msgId']._options = None
  _globals['_HEADER'].fields_by_name['msgId']._serialized_options = b'\222?\0028\010'
  _globals['_HEADER'].fields_by_name['svcId']._options = None
  _globals['_HEADER'].fields_by_name['svcId']._serialized_options = b'\222?\0028\010'
  _globals['_ERRORMESSAGE'].fields_by_name['error']._options = None
  _globals['_ERRORMESSAGE'].fields_by_name['error']._serialized_options = b'\222?\0028\010'
  _globals['_ERRORMESSAGE'].fields_by_name['detail']._options = None
  _globals['_ERRORMESSAGE'].fields_by_name['detail']._serialized_options = b'\222?\003\010\200\002'
  _globals['_ACKNACKMESSAGE'].fields_by_name['status_code']._options = None
  _globals['_ACKNACKMESSAGE'].fields_by_name['status_code']._serialized_options = b'\222?\0028\010'
  _globals['_ACKNACKMESSAGE'].fields_by_name['data']._options = None
  _globals['_ACKNACKMESSAGE'].fields_by_name['data']._serialized_options = b'\222?\002\010@'
  _globals['_CONSOLEMESSAGE'].fields_by_name['this_frame']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['this_frame']._serialized_options = b'\222?\0028\010'
  _globals['_CONSOLEMESSAGE'].fields_by_name['total_frames']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['total_frames']._serialized_options = b'\222?\0028\010'
  _globals['_CONSOLEMESSAGE'].fields_by_name['data']._options = None
  _globals['_CONSOLEMESSAGE'].fields_by_name['data']._serialized_options = b'\222?\003\010\200\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['sw_version']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['sw_version']._serialized_options = b'\222?\004\010\020x\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['serial_number']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['serial_number']._serialized_options = b'\222?\004\010\020x\001'
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['description']._options = None
  _globals['_SYSTEMINFOMESSAGE'].fields_by_name['description']._serialized_options = b'\222?\004\010 x\001'
  _globals['_PROTOCOLVERSION']._serialized_start=842
  _globals['_PROTOCOLVERSION']._serialized_end=881
  _globals['_ERRORCODE']._serialized_start=884
  _globals['_ERRORCODE']._serialized_end=1142
  _globals['_BUILTINSERVICE']._serialized_start=1144
  _globals['_BUILTINSERVICE']._serialized_end=1174
  _globals['_BUILTINMESSAGE']._serialized_start=1177
  _globals['_BUILTINMESSAGE']._serialized_end=1306
  _globals['_BUILTINMESSAGEVERSION']._serialized_start=1309
  _globals['_BUILTINMESSAGEVERSION']._serialized_end=1459
  _globals['_HEADER']._serialized_start=42
  _globals['_HEADER']._serialized_end=140
  _globals['_BASEMESSAGE']._serialized_start=142
  _globals['_BASEMESSAGE']._serialized_end=189
  _globals['_ERRORMESSAGE']._serialized_start=191
  _globals['_ERRORMESSAGE']._serialized_end=306
  _globals['_PINGMESSAGE']._serialized_start=308
  _globals['_PINGMESSAGE']._serialized_end=355
  _globals['_ACKNACKMESSAGE']._serialized_start=358
  _globals['_ACKNACKMESSAGE']._serialized_end=499
  _globals['_TICKMESSAGE']._serialized_start=501
  _globals['_TICKMESSAGE']._serialized_end=562
  _globals['_CONSOLEMESSAGE']._serialized_start=565
  _globals['_CONSOLEMESSAGE']._serialized_end=693
  _globals['_SYSTEMINFOMESSAGE']._serialized_start=696
  _globals['_SYSTEMINFOMESSAGE']._serialized_end=840
# @@protoc_insertion_point(module_scope)
