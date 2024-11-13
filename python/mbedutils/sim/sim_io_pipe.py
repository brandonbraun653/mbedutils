import zmq
import threading
import queue
import time
from loguru import logger
from typing import Callable, Optional

from mbedutils.intf.serial_intf import ISerial


class ZMQPipe(metaclass=ISerial):
    """
    Companion class to mb::hw::sim::BidirectionalPipe in C++. This forms the other end of the
    pipe, allowing Python code to communicate with a C++ simulator with automatic reconnection.
    """

    def __init__(self, endpoint: str, bind: bool = False,
                 reconnect_interval: float = 1.0,
                 max_retries: int = -1):  # -1 means infinite retries
        """
        Args:
            endpoint: ZMQ endpoint like "tcp://localhost:5555"
            bind: Whether to bind or connect (only one side should bind)
            reconnect_interval: Time in seconds between reconnection attempts
            max_retries: Maximum number of reconnection attempts (-1 for infinite)
        """

        self.endpoint = endpoint
        self.should_bind = bind
        self.reconnect_interval = reconnect_interval
        self.max_retries = max_retries
        self.retry_count = 0

        self.context = None
        self.socket = None
        self.running = False
        self.connected = False
        self.send_queue: queue.Queue[bytes] = queue.Queue()
        self.recv_queue: queue.Queue[bytes] = queue.Queue()

        self.transaction_thread: Optional[threading.Thread] = None
        self.reconnect_thread: Optional[threading.Thread] = None

        self.socket_lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        return self.running

    def open(self) -> bool:
        self.running = True
        self.retry_count = 0

        if self._init_zmq():
            self._ensure_threads_running()
        else:
            self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
            self.reconnect_thread.start()

        return True

    def close(self):
        self.running = False
        self.connected = False

        for thread in [self.transaction_thread, self.reconnect_thread]:
            if thread and thread.is_alive():
                thread.join()

        with self.socket_lock:
            if self.socket:
                self.socket.close()
                self.socket = None

        if self.context:
            self.context.term()
            self.context = None

    def read_all(self) -> bytes:
        try:
            return self.recv_queue.get_nowait()
        except queue.Empty:
            return b''

    def write(self, data: bytes):
        self.send_queue.put(data)

    def flush(self):
        pass

    def _init_zmq(self) -> bool:
        try:
            if self.context is None:
                self.context = zmq.Context()

            with self.socket_lock:
                if self.socket is not None:
                    self.socket.close()

                self.socket = self.context.socket(zmq.PAIR)
                self.socket.setsockopt(zmq.LINGER, 0)

                if self.should_bind:
                    self.socket.bind(self.endpoint)
                else:
                    self.socket.connect(self.endpoint)

                self.connected = True
                self._flush_socket()
                return True

        except zmq.ZMQError as e:
            logger.error(f"Failed to initialize ZMQ: {e}")
            self.connected = False
            return False

    def _flush_socket(self):
        """
        Flushes the socket of any pending messages
        Returns:
            None
        """
        if not self.socket:
            return

        with self.socket_lock:
            while self.socket.poll(0) != 0:
                try:
                    self.socket.recv(zmq.NOBLOCK)
                except zmq.ZMQError:
                    break

    def _reconnect_loop(self):
        while self.running and not self.connected:
            if self.max_retries != -1 and self.retry_count >= self.max_retries:
                logger.error("Max reconnection attempts reached")
                self.running = False
                break

            logger.info(f"Attempting to reconnect... (attempt {self.retry_count + 1})")
            if self._init_zmq():
                logger.info("Successfully reconnected")
                self._ensure_threads_running()
            else:
                self.retry_count += 1
                time.sleep(self.reconnect_interval)

    def _ensure_threads_running(self):
        if not self.transaction_thread or not self.transaction_thread.is_alive():
            self.transaction_thread = threading.Thread(target=self._transaction_loop)
            self.transaction_thread.start()
            time.sleep(0.1)

    def _transaction_loop(self):
        while self.running:
            # Don't do anything if not connected
            if not self.socket or not self.connected:
                time.sleep(0.1)
                continue

            # Receive data
            try:
                if self.socket.poll(0) != 0:
                    data = self.socket.recv(zmq.NOBLOCK)
                    if data:
                        self.recv_queue.put(data)

            except zmq.ZMQError as e:
                logger.error(f"Receive error: {e}")
                self.connected = False
                if self.running and self.reconnect_thread is None:
                    self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
                    self.reconnect_thread.start()

            # Write data
            try:
                data = self.send_queue.get_nowait()
                logger.info(f"Sending data: {data}")
                self.socket.send(data)
            except queue.Empty:
                continue
            except zmq.ZMQError as e:
                logger.error(f"Send error: {e}")
                self.connected = False
                if self.running and self.reconnect_thread is None:
                    self.reconnect_thread = threading.Thread(target=self._reconnect_loop)
                    self.reconnect_thread.start()

            time.sleep(0.01)
