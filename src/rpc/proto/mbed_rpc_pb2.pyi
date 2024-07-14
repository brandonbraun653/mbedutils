"""
@generated by mypy-protobuf.  Do not edit manually!
isort:skip_file
"""

import builtins
import collections.abc
import google.protobuf.descriptor
import google.protobuf.internal.containers
import google.protobuf.internal.enum_type_wrapper
import google.protobuf.message
import sys
import typing

if sys.version_info >= (3, 10):
    import typing as typing_extensions
else:
    import typing_extensions

DESCRIPTOR: google.protobuf.descriptor.FileDescriptor

class _ErrorCode:
    ValueType = typing.NewType("ValueType", builtins.int)
    V: typing_extensions.TypeAlias = ValueType

class _ErrorCodeEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_ErrorCode.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor
    NO_ERROR: _ErrorCode.ValueType  # 0

class ErrorCode(_ErrorCode, metaclass=_ErrorCodeEnumTypeWrapper): ...

NO_ERROR: ErrorCode.ValueType  # 0
global___ErrorCode = ErrorCode

class _BuiltinServices:
    ValueType = typing.NewType("ValueType", builtins.int)
    V: typing_extensions.TypeAlias = ValueType

class _BuiltinServicesEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_BuiltinServices.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor
    SVC_PING: _BuiltinServices.ValueType  # 0

class BuiltinServices(_BuiltinServices, metaclass=_BuiltinServicesEnumTypeWrapper): ...

SVC_PING: BuiltinServices.ValueType  # 0
global___BuiltinServices = BuiltinServices

class _BuiltinMessages:
    ValueType = typing.NewType("ValueType", builtins.int)
    V: typing_extensions.TypeAlias = ValueType

class _BuiltinMessagesEnumTypeWrapper(google.protobuf.internal.enum_type_wrapper._EnumTypeWrapper[_BuiltinMessages.ValueType], builtins.type):
    DESCRIPTOR: google.protobuf.descriptor.EnumDescriptor
    MSG_PING: _BuiltinMessages.ValueType  # 0

class BuiltinMessages(_BuiltinMessages, metaclass=_BuiltinMessagesEnumTypeWrapper): ...

MSG_PING: BuiltinMessages.ValueType  # 0
global___BuiltinMessages = BuiltinMessages

@typing.final
class Header(google.protobuf.message.Message):
    """Core message header common to all types. Each functional message type **must**
    have this first in their list of declarations.
    """

    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    CRC_FIELD_NUMBER: builtins.int
    SIZE_FIELD_NUMBER: builtins.int
    VERSION_FIELD_NUMBER: builtins.int
    SEQID_FIELD_NUMBER: builtins.int
    MSGID_FIELD_NUMBER: builtins.int
    crc: builtins.int
    """CRC16 of the message for validity checks"""
    size: builtins.int
    """Size of the message in bytes"""
    version: builtins.int
    """Version of this message & RPC protocol. Upper 4 bits are RPC, lower 4 bits are message."""
    seqId: builtins.int
    """Sequence ID for the message transaction"""
    msgId: builtins.int
    """Root message identifier"""
    def __init__(
        self,
        *,
        crc: builtins.int | None = ...,
        size: builtins.int | None = ...,
        version: builtins.int | None = ...,
        seqId: builtins.int | None = ...,
        msgId: builtins.int | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing.Literal["crc", b"crc", "msgId", b"msgId", "seqId", b"seqId", "size", b"size", "version", b"version"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing.Literal["crc", b"crc", "msgId", b"msgId", "seqId", b"seqId", "size", b"size", "version", b"version"]) -> None: ...

global___Header = Header

@typing.final
class BaseMessage(google.protobuf.message.Message):
    """Root type that parsers can use to peek at messages and figure out what type the full message is."""

    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    HEADER_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    def __init__(
        self,
        *,
        header: global___Header | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing.Literal["header", b"header"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing.Literal["header", b"header"]) -> None: ...

global___BaseMessage = BaseMessage

@typing.final
class Ping(google.protobuf.message.Message):
    """Simple ping message to test RPC connection."""

    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    HEADER_FIELD_NUMBER: builtins.int
    TIMESTAMP_FIELD_NUMBER: builtins.int
    timestamp: builtins.int
    @property
    def header(self) -> global___Header: ...
    def __init__(
        self,
        *,
        header: global___Header | None = ...,
        timestamp: builtins.int | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing.Literal["header", b"header", "timestamp", b"timestamp"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing.Literal["header", b"header", "timestamp", b"timestamp"]) -> None: ...

global___Ping = Ping

@typing.final
class ListFunctionsRequest(google.protobuf.message.Message):
    """Message to request a list of all available RPC functions."""

    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    HEADER_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    def __init__(
        self,
        *,
        header: global___Header | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing.Literal["header", b"header"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing.Literal["header", b"header"]) -> None: ...

global___ListFunctionsRequest = ListFunctionsRequest

@typing.final
class ListFunctionsResponse(google.protobuf.message.Message):
    """Message to respond to a ListFunctionsRequest."""

    DESCRIPTOR: google.protobuf.descriptor.Descriptor

    HEADER_FIELD_NUMBER: builtins.int
    FUNCTIONS_FIELD_NUMBER: builtins.int
    @property
    def header(self) -> global___Header: ...
    @property
    def functions(self) -> google.protobuf.internal.containers.RepeatedScalarFieldContainer[builtins.str]: ...
    def __init__(
        self,
        *,
        header: global___Header | None = ...,
        functions: collections.abc.Iterable[builtins.str] | None = ...,
    ) -> None: ...
    def HasField(self, field_name: typing.Literal["header", b"header"]) -> builtins.bool: ...
    def ClearField(self, field_name: typing.Literal["functions", b"functions", "header", b"header"]) -> None: ...

global___ListFunctionsResponse = ListFunctionsResponse