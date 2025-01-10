from mbedutils.rpc.client import RPCClient
from mbedutils.rpc.message import *
from loguru import logger
from mbedutils.rpc.observer_impl import TransactionResponseObserver


class LoggerRPCClient:

    def __init__(self, rpc_client: RPCClient, logger_id: int):
        """
        Args:
            rpc_client: RPCClient instance to use for communication
            logger_id: ID of the logger to interact with on the remote device
        """
        self._rpc_client = rpc_client
        self._logger_id = logger_id

    def erase(self) -> bool:
        """
        Erase the logger's memory

        Returns:
            True if the erase was successful, False otherwise
        """
        request = LoggerEraseRequestPBMsg()
        request.pb_message.which = self._logger_id

        response = self._rpc_client.com_pipe.write_and_wait(request, timeout=30)
        assert len(response) <= 1
        assert all(isinstance(x, LoggerEraseResponsePBMsg) for x in response)

        if response:
            return response[0].pb_message.success
        else:
            return False

    def write(self, message: bytes, level: LoggerWriteRequest.Level) -> bool:
        """
        Write a message to the logger

        Args:
            message: Message to write
            level: Log level to write the message at

        Returns:
            True if the write was successful, False otherwise
        """
        request = LoggerWriteRequestPBMsg()
        request.pb_message.which = self._logger_id
        request.pb_message.level = level
        request.pb_message.data = message

        response = self._rpc_client.com_pipe.write_and_wait(request, timeout=5)
        assert len(response) <= 1
        assert all(isinstance(x, LoggerWriteResponsePBMsg) for x in response)

        if response:
            return response[0].pb_message.success
        else:
            return False

    def read(self, count: int = 1, direction: bool = True) -> List[bytes]:
        """
        Read log entries from the logger

        Args:
            count: Number of log entries to read
            direction: True for oldest to newest, False for newest to oldest

        Returns:
            List of log entries read, or None if the read failed
        """
        # Build up the request
        request = LoggerReadRequestPBMsg()
        request.pb_message.which = self._logger_id
        request.pb_message.count = count
        request.pb_message.direction = direction

        # Send the request for logs
        timeout = 3 * count
        responses = self._rpc_client.com_pipe.write_and_wait(request, timeout, count)
        assert len(responses) <= count
        assert all(isinstance(r, LoggerReadStreamResponsePBMsg) for r in responses)

        # Check if all indexes are present, from zero to "count"
        responses.sort(key=lambda x: x.pb_message.index)
        missing_indices = set(range(count)) - set(r.pb_message.index for r in responses)

        if missing_indices:
            logger.warning(f"Missing log entries: {missing_indices}")
            return []

        return [r.pb_message.data for r in responses]
