import socket

from mbedutils.intf.serial_intf import ISerial


class SerialSocket(metaclass=ISerial):
    """ A class to simulate a serial connection over a socket. """

    def __init__(self, port: int):
        """
        Args:
            port: Port the simulator is listening on
        """
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._address = ("localhost", port)
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        """
        Open the socket connection and connect to the server
        Returns:
            None
        """
        self._socket.connect(self._address)
        self._socket.setblocking(False)
        self._is_open = True

    def close(self) -> None:
        """
        Close the socket connection
        Returns:
            None
        """
        self._socket.shutdown(socket.SHUT_RDWR)
        self._socket.close()
        self._is_open = False

    def read_all(self) -> bytes:
        """
        Read all available data from the socket
        Returns:
            Data from the socket
        """
        try:
            if self._is_open:
                return self._socket.recv(4096)
        except BlockingIOError:
            pass

        return bytes()

    def write(self, data: bytes) -> None:
        """
        Write data to the socket
        Args:
            data: Data to write

        Returns:
            None
        """
        if self._is_open:
            self._socket.sendall(data)

    def flush(self) -> None:
        """ Intentionally left blank """
        pass
