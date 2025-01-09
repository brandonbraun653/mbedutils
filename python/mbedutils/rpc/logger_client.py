from mbedutils.rpc.client import RPCClient
from mbedutils.rpc.message import *
from loguru import logger


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
        if not isinstance(response, LoggerEraseResponsePBMsg):
            logger.warning(
                f"Expected type LoggerEraseResponsePBMsg, got {type(response)}"
            )
            return False

        return response.pb_message.success
