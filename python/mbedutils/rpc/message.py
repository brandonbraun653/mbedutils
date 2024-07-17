from __future__ import annotations

import binascii
import struct
import crc
from loguru import logger
from threading import RLock
from typing import TypeVar, Optional, Generic
from mbedutils.utils import Singleton
from mbedutils.rpc.proto.mbed_rpc_pb2 import *


class CRCMismatchException(Exception):
    pass


# def crc16(data: bytes, poly: int = 0x8005, init=0x0000, refin=False, refout=True, xorout=0x0000) -> int:
#     """
#     Calculates the CRC-16 CCITT checksum of the given data.
#     Args:
#         data: Data buffer to compute the CRC of
#         poly: Polynomial to use for the CRC calculation, default is 0x8005 (CRC-16 CCITT)
#         init: Initial value of the CRC
#         refin: Reflect input bits
#         refout: Reflect output bits
#         xorout: XOR value to apply to the final CRC
#
#     Returns:
#         CRC-16 value
#     """
#     crc = init
#     for byte in data:
#         if refin:
#             byte = int(binascii.hexlify(bytes([byte])), 16)
#         for _ in range(8):
#             if (crc ^ byte) & 0x0001:
#                 crc = (crc >> 1) ^ poly
#             else:
#                 crc >>= 1
#             byte >>= 1
#     if refout:
#         crc = (crc >> 8) | (crc << 8) & 0xFFFF
#     return crc ^ xorout


class SeqIdGenerator(metaclass=Singleton):
    """ Unique identifier generator for messages. This is used for the seqId field in the message header. """
    def __init__(self):
        self._seq_id = 0
        self._lock = RLock()

    @property
    def next_uuid(self) -> int:
        with self._lock:
            self._seq_id = (self._seq_id + 1) % 65536
            return self._seq_id


# Core type for messages that inherit from this class
PBMsgType = TypeVar('PBMsgType')


class BasePBMsg(Generic[PBMsgType]):

    def __init__(self):
        self._id_gen = SeqIdGenerator()
        self._pb_msg: Optional[PBMsgType] = None

    @property
    def name(self) -> str:
        return self.__class__.__name__

    @property
    def pb_message(self) -> PBMsgType:
        return self._pb_msg

    @property
    def uuid(self) -> int:
        self._assign_seq_id_if_empty()
        return self._pb_msg.header.seqId

    @property
    def sub_id(self) -> int:
        return int(self._pb_msg.header.subId)

    @sub_id.setter
    def sub_id(self, sid: int) -> None:
        self._pb_msg.header.subId = sid

    @property
    def msg_id(self) -> int:
        return self._pb_msg.header.msgId

    @msg_id.setter
    def msg_id(self, mid: int):
        assert 0 <= mid <= 255, "Message ID must be between 0 and 255"
        self._pb_msg.header.msgId = mid

    def deserialize(self, serialized: bytes) -> int:
        """
        Decodes a number of bytes into the message type for this class
        Args:
            serialized: Buffer of data containing the message to decode

        Returns:
            How many bytes were parsed
        """
        # Calculate the CRC of the message data and compare it to the received CRC
        received_crc = struct.unpack("<H", serialized[:2])[0]
        calculated_crc = crc16(serialized[2:])
        if received_crc != calculated_crc:
            logger.error(f"Received CRC: {received_crc}, Calculated CRC: {calculated_crc}")
            raise CRCMismatchException()

        return self.pb_message.ParseFromString(serialized)

    def serialize(self) -> bytes:
        """
        Serializes the message into a byte buffer. This is formatted as:
        [CRC16 (2 bytes)] [Message Data]

        Returns:
            Serialized message
        """
        # Grab the next logical sequence ID if it hasn't been set yet. This sets temporal ordering of messages.
        self._assign_seq_id_if_empty()

        # All message modifications should be done, so calculate the CRC
        serialized_data = self.pb_message.SerializeToString()
        crc16 = crc.Calculator(crc.Crc16.XMODEM).checksum(serialized_data)

        return struct.pack("<H", crc16) + serialized_data

    def _assign_seq_id_if_empty(self):
        """
        ProtoBuf don't support default values for fields, but they do set them to 0 by default. We can
        use this to our advantage as a way to determine if the UUID has been set yet.

        Returns:
            None
        """
        if self._pb_msg and self._pb_msg.header.seqId == 0:
            self._pb_msg.header.seqId = self._id_gen.next_uuid


class AckNackPBMsg(BasePBMsg[AckNackMessage]):

    def __init__(self):
        super().__init__()
        self._pb_msg = AckNackMessage()
        self._pb_msg.header.version = BuiltinMessageVersion.MSG_VER_ACK_NACK
        self._pb_msg.header.seqId = 0
        self._pb_msg.header.msgId = BuiltinMessage.MSG_ACK_NACK
        self._pb_msg.header.svcId = 0
        self._pb_msg.acknowledge = False
        self._pb_msg.status_code = ErrorCode.ERR_NO_ERROR

    @property
    def ack(self) -> bool:
        """
        Returns:
            True if the message is a positive acknowledgement
        """
        return self._pb_msg.acknowledge

    @property
    def nack(self) -> bool:
        """
        Returns:
            True if the message is a negative acknowledgement
        """
        return not self._pb_msg.acknowledge

    @property
    def error_code(self) -> ErrorCode:
        return self._pb_msg.status_code

    @error_code.setter
    def error_code(self, sc: ErrorCode):
        self._pb_msg.status_code = sc


class PingPBMsg(BasePBMsg[PingMessage]):

    def __init__(self):
        super().__init__()
        self._pb_msg = PingMessage()
        self._pb_msg.header.msgId = BuiltinMessage.MSG_PING
        self._pb_msg.header.version = BuiltinMessageVersion.MSG_VER_ACK_NACK
        self._pb_msg.header.svcId = BuiltinService.SVC_PING
        self._pb_msg.header.seqId = 0


class TickPBMsg(BasePBMsg[TickMessage]):

    def __init__(self):
        super().__init__()
        self._pb_msg = TickMessage()
        self._pb_msg.header.msgId = BuiltinMessage.MSG_TICK
        self._pb_msg.tick = 0

    @property
    def tick(self) -> int:
        return self._pb_msg.tick


class ConsolePBMsg(BasePBMsg[ConsoleMessage]):

    def __init__(self):
        super().__init__()
        self._pb_msg = ConsoleMessage()
        self._pb_msg.header.msgId = BuiltinMessage.MSG_CONSOLE

    @property
    def frame_number(self) -> int:
        return self._pb_msg.this_frame

    @property
    def total_frames(self) -> int:
        return self._pb_msg.total_frames

    @property
    def data(self) -> bytes:
        return self._pb_msg.data


class SystemInfoPBMsg(BasePBMsg[SystemInfoMessage]):

    def __init__(self):
        super().__init__()
        self._pb_msg = SystemInfoMessage()
        self._pb_msg.header.msgId = BuiltinMessage.MSG_SYSTEM_INFO

    @property
    def software_version(self) -> str:
        return self._pb_msg.software_version

    @property
    def serial_number(self) -> str:
        return self._pb_msg.serial_number

    @property
    def description(self) -> str:
        return self._pb_msg.description
