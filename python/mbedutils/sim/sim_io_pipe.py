import zmq
import threading
import queue
import time
from loguru import logger
from typing import Optional

from mbedutils.intf.serial_intf import ISerial


class ZMQPipe(metaclass=ISerial):
    """
    Companion class to mb::hw::sim::BidirectionalPipe in C++. This forms the other end of the
    pipe, allowing Python code to communicate with a C++ simulator with automatic reconnection.
    """

    def __init__(self, endpoint: str, bind: bool = False):
        """
        Args:
            endpoint: ZMQ endpoint like "tcp://localhost:5555"
            bind: Whether to bind or connect (only one side should bind)
        """
        self.endpoint = endpoint
        self.should_bind = bind
        self.zmq_ctx = None
        self.socket = None
        self.connected = False
        self.send_queue: queue.Queue[bytes] = queue.Queue()
        self.recv_queue: queue.Queue[bytes] = queue.Queue()
        self.transaction_thread: Optional[threading.Thread] = None
        self.socket_lock = threading.RLock()
        self._kill_signal = threading.Event()

    @property
    def is_open(self) -> bool:
        return self.connected

    def open(self) -> bool:
        """
        Open the pipe for communication
        Returns:
            bool: True if the pipe was successfully opened, False otherwise
        """
        try:
            if self.zmq_ctx is None:
                self.zmq_ctx = zmq.Context()

            with self.socket_lock:
                if self.socket is not None:
                    self.socket.close()

                self.socket = self.zmq_ctx.socket(zmq.PAIR)
                self.socket.setsockopt(zmq.LINGER, 0)
                self.socket.setsockopt(zmq.SNDHWM, 250)
                self.socket.setsockopt(zmq.RCVHWM, 250)

                if self.should_bind:
                    self.socket.bind(self.endpoint)
                else:
                    self.socket.connect(self.endpoint)

                self.connected = True
                self.flush()
        except zmq.ZMQError as e:
            logger.error(f"Failed to initialize ZMQ: {e}")
            return False

        self._kill_signal.clear()
        self.transaction_thread = threading.Thread(target=self._transaction_loop)
        self.transaction_thread.start()
        time.sleep(0.1)
        return True

    def close(self) -> None:
        """
        Close the pipe
        Returns:
            None
        """
        self._kill_signal.set()
        self.connected = False

        self.transaction_thread.join() if self.transaction_thread.is_alive() else None

        with self.socket_lock:
            if self.socket:
                self.socket.close()
                self.socket = None

        if self.zmq_ctx:
            self.zmq_ctx.term()
            self.zmq_ctx = None

    def read_all(self) -> bytes:
        try:
            return self.recv_queue.get_nowait()
        except queue.Empty:
            return b""

    def write(self, data: bytes) -> bool:
        """
        Write data to the pipe
        Args:
            data: Data to write

        Returns:
            True if the write was successful, False otherwise
        """
        try:
            self.send_queue.put(data, timeout=1.0)
            return True
        except queue.Full:
            logger.warning("Send queue full, dropping message")
            return False

    def flush(self) -> None:
        """
        Flush the pipe of any pending data in the RX and TX queues
        Returns:
            None
        """
        with self.socket_lock:
            # Flush the TX queue
            while True:
                try:
                    self.send_queue.get_nowait()
                except queue.Empty:
                    break

            # Flush the RX buffer internal to the socket
            if self.socket:
                while self.socket.poll(0) != 0:
                    try:
                        self.socket.recv(zmq.NOBLOCK)
                    except zmq.ZMQError:
                        break

    def _transaction_loop(self):
        while not self._kill_signal.is_set():
            # Don't do anything if not connected
            if not self.socket or not self.connected:
                time.sleep(0.1)
                continue

            # Receive data. Polling here also acts as a psuedo rate limiter.
            try:
                if self.socket.poll(5) != 0:
                    if data := self.socket.recv(zmq.NOBLOCK):
                        # logger.warning(f"RX {len(data)} bytes")
                        with self.socket_lock:
                            self.recv_queue.put(data)

            except zmq.ZMQError as e:
                logger.error(f"Receive error: {e}")
                self.connected = False
                break

            # Write data
            try:
                with self.socket_lock:
                    data = self.send_queue.get_nowait()

                self.socket.send(data)
                # logger.warning(f"TX {len(data)} bytes")
            except queue.Empty:
                continue
            except zmq.ZMQError as e:
                logger.error(f"Send error: {e}")
                self.connected = False
                break
