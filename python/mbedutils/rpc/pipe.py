import binascii
import copy
import queue
import struct
import time
from enum import Enum

import crc
import mbedutils.rpc.proto.mbed_rpc_pb2 as proto
import google.protobuf.message as g_proto_msg
from typing import List, Optional, Union, Callable, Dict, Type
from cobs import cobs
from loguru import logger
from serial import Serial
from threading import Thread, Event
from queue import Queue

from mbedutils.intf.serial_intf import ISerial
from mbedutils.rpc.publisher import Publisher
from mbedutils.rpc.message import BasePBMsg, AckNackPBMsg, CRCMismatchException
from mbedutils.rpc.observer_impl import TransactionResponseObserver, PredicateObserver
from mbedutils.rpc.socket import SerialSocket
from mbedutils.sim.sim_io_pipe import ZMQPipe


class PipeType(Enum):
    """
    Enumerated type for the type of pipe to use
    """
    DEVICE = 0  # Typical serial connection like COM port
    SOCKET = 1  # Socket connection based connection
    ZMQ = 2  # ZeroMQ based connection, typically for simulators


class COBSerialPipe(Publisher):
    """
    Message server for RTX-ing COBS encoded protocol buffer messages over a serial connection. Implements
    the Publisher interface for message dispatching to observers.
    """

    def __init__(self):
        super().__init__()
        self._serial: Optional[ISerial] = None
        self._kill_event = Event()
        self._message_descriptors: Dict[int, Type[BasePBMsg]] = {}

        # TX resources
        self._tx_thread = Thread()
        self._tx_msgs: Queue[bytes] = Queue()

        # RX resources
        self._rx_thread = Thread()
        self._rx_msgs = Queue()
        self._rx_byte_buffer = bytearray()

        # Dispatch resources
        self._dispatch_thread = Thread()

    def add_message_descriptor(self, msg_type: Union[BasePBMsg, Type[BasePBMsg]]) -> None:
        """
        Adds a message descriptor to the publisher so that it can encode/decode messages
        Args:
            msg_type: Message instance to register

        Returns:
            None
        """
        assert isinstance(msg_type, BasePBMsg), "Message type must be a subclass of BasePBMsg"
        self._message_descriptors[msg_type.msg_id] = type(msg_type)

    def open(self, pipe_type: PipeType, port: Union[str, int], baud: int = None) -> None:
        """
        Opens a serial port for communication
        Args:
            pipe_type: Type of pipe connection to open
            port: Serial port connection. Assumed a COM port if a string, Socket if an integer.
            baud: Desired communication baud

        Returns:
            None
        """
        if pipe_type == PipeType.DEVICE:
            assert isinstance(port, str), "Port must be a string"
            assert isinstance(baud, int), "Baud must be an integer"
            com_device = Serial(timeout=5)
            com_device.port = port
            com_device.baudrate = baud
            com_device.exclusive = True
            self._serial = com_device
        elif pipe_type == PipeType.SOCKET:
            assert isinstance(port, int), "Port must be an integer"
            self._serial = SerialSocket(port=port)
        elif pipe_type == PipeType.ZMQ:
            assert isinstance(port, str), "Port must be a string"
            self._serial = ZMQPipe(endpoint=port)

        # Clear memory
        self._rx_msgs = Queue()

        # Open the serial port with the desired configuration
        self._serial.open()
        logger.trace(f"Opened serial port {port} at {baud} baud")

        # Spawn the IO threads for enabling communication
        self._kill_event.clear()
        self._rx_thread = Thread(target=self._rx_decoder_thread, name="RXDecoder")
        self._rx_thread.start()
        self._tx_thread = Thread(target=self._tx_encoder_thread, name="TXEncoder")
        self._tx_thread.start()
        self._dispatch_thread = Thread(target=self._rx_dispatcher_thread, name="RXDispatch")
        self._dispatch_thread.start()

        # Allow the threads to start up
        time.sleep(0.1)

    def close(self) -> None:
        """
        Closes the pipe, destroying all resources
        Returns:
            None
        """
        # Kill the threads first since they consume the serial port
        self._kill_event.set()
        self._rx_thread.join() if self._rx_thread.is_alive() else None
        self._tx_thread.join() if self._tx_thread.is_alive() else None
        self._dispatch_thread.join() if self._dispatch_thread.is_alive() else None

        # Push any remaining information and then close out resources
        if self._serial and self._serial.is_open:
            self._serial.flush()
            self._serial.close()

    def write(self, msg: BasePBMsg) -> None:
        """
        Enqueues a new message for transmission
        Args:
            msg: Message to send

        Returns:
            None
        """
        assert isinstance(msg, BasePBMsg), "Message must be a subclass of BasePBMsg"
        logger.trace(f"Sending {repr(msg)}")
        self._tx_msgs.put(msg.serialize(), block=True)

    def write_and_wait(self, msg: BasePBMsg, timeout: Union[int, float], count: Optional[int] = 1) -> Optional[
        Union[BasePBMsg, List[BasePBMsg]]]:
        """
        Writes a message to the serial pipe and waits for a response
        Args:
            msg: Message to send
            timeout: Time to wait for a response
            count: Number of responses to wait for (positive integer) or None to wait indefinitely

        Returns:
            List of messages received if count > 1, or a single message if count == 1. None if no response.
        """
        observer = TransactionResponseObserver(txn_uuid=msg.uuid, timeout=timeout, count=count)
        sub_id = self.subscribe_observer(observer)

        # Send the message and wait for the response
        self.write(msg)
        result = observer.wait()

        # Clean up the observer
        self.unsubscribe(sub_id)
        return result

    def filter(self, predicate: Callable[[BasePBMsg], bool], qty: int = 1,
               timeout: Union[int, float] = 1.0) -> List[BasePBMsg]:
        """
        Filters incoming the incoming message stream based on a user defined predicate, then returns the
        messages that fulfilled the predicate.

        Args:
            predicate: User defined predicate function
            qty: Number of messages to accept before terminating
            timeout: Time to wait for the predicate to be fulfilled

        Returns:
            List of messages that fulfilled the predicate
        """
        observer = PredicateObserver(func=predicate, qty=qty, timeout=timeout)
        sub_id = self.subscribe_observer(observer)
        results = observer.wait()
        self.unsubscribe(sub_id)
        return results

    @staticmethod
    def process_ack_nack_response(msg: BasePBMsg, error_string: str = None) -> bool:
        """
        Processes an AckNackMessage response from the ESC
        Args:
            msg: Response message instance
            error_string: Optional error string to print if the response was a NACK

        Returns:
            True if the response was an ACK, False otherwise
        """
        if not msg or not isinstance(msg, AckNackPBMsg):
            logger.error(f"No valid response from server {'. Got type ' + str(type(msg)) if msg else ''}")
            return False

        if not msg.ack:
            logger.error(f"NACK: {repr(msg.error_code)} ' -- {error_string if error_string else ''}")
            return False
        else:
            return True

    def _tx_encoder_thread(self) -> None:
        """
        Encodes queued TX packets with COBS framing and sends it on the wire
        Returns:
            None
        """
        logger.trace(f"Starting OrbitESC internal Serial message encoder thread")
        while not self._kill_event.is_set():
            # Pull the latest data off the queue
            try:
                raw_frame = self._tx_msgs.get(block=True, timeout=0.1)
            except queue.Empty:
                continue

            # Add the CRC to the message
            crc16 = crc.Calculator(crc.Crc16.XMODEM).checksum(raw_frame)
            crc_bytes = struct.pack("<H", crc16)

            # Encode the frame w/termination byte, then transmit
            encoded_frame = cobs.encode(crc_bytes + raw_frame) + b'\x00'
            self._serial.write(encoded_frame)
            logger.trace(f"Write {len(encoded_frame)} byte frame")

        logger.trace("Terminating OrbitESC internal Serial message encoder")

    def _rx_decoder_thread(self) -> None:
        """
        Thread to handle reception of data from the connected endpoint, encoded with COBS framing.
        Will parse valid COBS packets into the appropriate protocol buffer message type.

        Returns:
            None
        """
        logger.trace("Starting OrbitESC internal Serial message decoder thread")
        while not self._kill_event.is_set():
            # Allow other threads time to execute
            time.sleep(0.01)

            # Fill the cache with the raw data from the bus
            if new_data := self._serial.read_all():
                self._rx_byte_buffer.extend(new_data)
            elif not len(self._rx_byte_buffer):
                continue

            # Parse the data in the cache to extract all waiting COBS frames
            self._rx_decode_available_cobs_frames()

        logger.trace("Terminating OrbitESC internal Serial message decoder")

    def _rx_dispatcher_thread(self) -> None:
        """
        Takes new messages and dispatches them to all observers
        Returns:
            None
        """
        logger.trace("Starting OrbitESC internal Serial message dispatcher thread")
        while not self._kill_event.is_set():
            self.prune_expired_observers()
            try:
                while msg := self._rx_msgs.get(block=True, timeout=0.01):
                    self.accept(msg)
            except queue.Empty:
                continue

        logger.trace("Terminating OrbitESC internal Serial message dispatcher")

    def _rx_decode_available_cobs_frames(self) -> None:
        """
        Parses the current RX buffer for any available COBS frames and decodes them into
        protocol buffer messages.

        Returns:
            None
        """
        try:
            while True:
                # Search for the frame delimiter and extract an entire frame if it exists
                eof_idx = self._rx_byte_buffer.index(b'\x00')
                cobs_frame = self._rx_byte_buffer[:eof_idx]

                # Remove the frame from the buffer by slicing it out
                self._rx_byte_buffer = self._rx_byte_buffer[eof_idx + 1:]
                logger.trace(f"Read {len(cobs_frame) + 1} byte frame")

                # Decode the message into a higher level message type
                if pb_frame := self._decode_cobs_frame(cobs_frame):
                    if full_msg := self._decode_pb_frame(pb_frame):
                        self._rx_msgs.put(full_msg)
                        logger.trace(f"Receiving {repr(full_msg)}")

        except ValueError:
            # No more frames available
            pass

    @staticmethod
    def _decode_cobs_frame(frame: bytes) -> Optional[bytes]:
        """
        Decodes a COBS encoded frame into the original message
        Args:
            frame: COBS encoded frame

        Returns:
            Decoded message
        """
        try:
            return cobs.decode(frame)
        except cobs.DecodeError:
            # Nothing much to do here if this fails. Just move on to the next frame.
            logger.trace("Failed to decode COBS frame. Likely partially received message.")
            return None

    def _decode_pb_frame(self, frame: bytes) -> Optional[BasePBMsg]:
        """
        Decodes a NanoPB encoded frame into the original protocol buffer message
        Args:
            frame: NanoPB encoded frame

        Returns:
            Decoded protocol buffer message
        """
        # Check the CRC of the frame
        nano_pb_frame = frame[2:]
        received_crc = struct.unpack("<H", frame[:2])[0]
        crc16 = crc.Calculator(crc.Crc16.XMODEM).checksum(nano_pb_frame)

        if crc16 != received_crc:
            logger.error(f"CRC mismatch on frame. Expected {crc16}, got {received_crc}")
            return None

        # Peek the header of the message
        try:
            base_msg = proto.BaseMessage()
            base_msg.ParseFromString(nano_pb_frame)
            if base_msg.header.msgId not in self._message_descriptors.keys():
                logger.error(f"Unsupported message ID: {base_msg.header.msgId}")
                return None
        except g_proto_msg.DecodeError:
            logger.trace("Frame did not contain the expected header. Unable to parse.")
            return None

        # Now do the full decode since the claimed type is supported
        full_msg = self._message_descriptors[base_msg.header.msgId]()
        try:
            full_msg.deserialize(nano_pb_frame)
            return full_msg
        except g_proto_msg.DecodeError:
            logger.error(f"Failed to decode {full_msg.name} type")
            return None
