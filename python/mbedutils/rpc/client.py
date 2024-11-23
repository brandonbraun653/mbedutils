from __future__ import annotations

import atexit
import time
from functools import wraps
from loguru import logger
from typing import Any, Callable, Union
from threading import Event, Thread
from mbedutils.rpc.pipe import COBSerialPipe, PipeType
from mbedutils.rpc.message import *
from mbedutils.rpc.observer_impl import MessageObserver, ConsoleObserver


class NotOnlineException(BaseException):
    pass


def online_checker(method: Callable) -> Any:
    """
    Decorator to check that the server is online before allowing a method
    call to proceed. This prevents duplicate code on a variety of methods.

    Args:
        method: Function to invoke

    Returns:
        Whatever the method to invoke returns

    References:
        https://stackoverflow.com/a/36944992/8341975
    """
    @wraps(method)
    def _impl(self: RPCClient, *method_args, **method_kwargs):
        # Ensure this decorator is only used on RPCClient classes
        assert isinstance(self, RPCClient)
        if not self.is_online:
            raise NotOnlineException()

        # Execute the function as normal
        method_output = method(self, *method_args, **method_kwargs)
        return method_output

    return _impl


class RPCClient:
    """ High level client to connect with a remote RPC server listening on a port """

    def __init__(self):
        # Initialize the transport layer and add default messages
        self._transport = COBSerialPipe()
        self._transport.add_message_descriptor(PingPBMsg())
        self._transport.add_message_descriptor(ConsolePBMsg())

        self._online = False
        self._time_last_online = 0
        self._last_tick = 0
        self._kill_signal = Event()
        self._notify_signal = Event()
        self._thread = Thread(target=self._background_thread, daemon=True, name=f"SerialClient_Background")

        atexit.register(self._teardown)

    @property
    def com_pipe(self) -> COBSerialPipe:
        return self._transport

    @property
    def is_online(self) -> bool:
        return self._online

    def ping(self) -> bool:
        """
        Returns:
            True if the device was ping-able, False otherwise
        """
        sub_id = self.com_pipe.subscribe(msg=PingPBMsg, qty=1, timeout=5.0)
        self.com_pipe.write(PingPBMsg())
        responses = self.com_pipe.get_subscription_data(sub_id)
        if not responses:
            logger.warning("Node did not respond to ping")
        return bool(responses)

    def open(self, pipe_type: PipeType, port: Union[str, int], baud: int = None) -> None:
        """
        Opens a connection to the remote node
        Args:
            pipe_type: Type of pipe to open
            port: Serial port connection. Assumed a COM port if a string, Socket if an integer.
            baud: Desired communication baud
        """
        self._transport.open(pipe_type=pipe_type, port=port, baud=baud)

        # Register known observers
        self._transport.subscribe_observer(MessageObserver(func=self._observer_remote_tick, msg_type=TickPBMsg))
        self._transport.subscribe_observer(ConsoleObserver(on_msg_rx=lambda x: logger.info(x.strip('\n'))))

        self._thread.start()
        time.sleep(0.05)

    def close(self) -> None:
        return self._teardown()

    def invoke_service(self, service_id: int, request: BasePBMsg) -> Optional[BasePBMsg]:
        """
        Invokes a service on the remote node
        Args:
            service_id: Which service to invoke
            request: Request message to send

        Returns:
            Response message from the service, if any is required
        """
        pass

    def _teardown(self) -> None:
        self._kill_signal.set()
        self._thread.join() if self._thread.is_alive() else None

        self._transport.close()

    def _background_thread(self) -> None:
        """
        General purpose thread to handle background tasks that help with processing data
        Returns:
            None
        """
        while not self._kill_signal.is_set():
            self._notify_signal.wait(timeout=0.1)

            # Do a tick timeout check to ensure the node hasn't dropped off the face of the earth
            if self._online and ((time.time() - self._time_last_online) > 3.0):
                logger.warning("Remote node has gone offline")
                self._online = False

    def _observer_remote_tick(self, msg: TickPBMsg) -> None:
        """
        Looks for the tick of the remote server node to determine online/offline status
        Args:
            msg: Received bus message

        Returns:
            None
        """
        self._time_last_online = time.time()
        self._last_tick = msg.tick
        if not self._online:
            logger.info("Remote node is online")
            self._online = True
