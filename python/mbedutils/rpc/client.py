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

        for msg in get_module_messages("mbedutils.rpc.message"):
            self._transport.add_message_descriptor(msg)

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

    def sleep_on_server_time_ms(self, milliseconds: int, percent_tolerance: float = 0.1) -> None:
        """
        Sleep the current context based on the remote server's perception of time. A message is
        sent to the remote server to notify it to wait a certain amount of time before notifying us
        back. This is useful for synchronizing actions between the client and server.

        Args:
            milliseconds: How long the remote device should wait to notify us
            percent_tolerance: How much error is allowed in the actual sleep time

        Raises:
            RuntimeWarning: If the node does not respond to the sleep request
        """
        assert isinstance(milliseconds, int)
        assert milliseconds > 0
        assert milliseconds < 2 ** 32  # Remote servers are typically 32-bit microcontrollers
        assert percent_tolerance >= 0.0
        assert percent_tolerance <= 1.0

        req = NotifyTimeElapsedRequestPBMsg()
        req.pb_message.delay_time = milliseconds

        # Allow a larger timeout b/c depending on the system load, the node may take a while to respond
        # while still accurately simulating the time. This should provide reasonable bounds for testing
        # and communication.
        rsp = self.com_pipe.write_and_wait(req, timeout=2 * (milliseconds / 1000))
        if not rsp:
            raise RuntimeWarning("Node did not respond to sleep request")

        # Check the response to ensure the node slept for the correct amount of time. Generally speaking
        # the time elapsed should be within a certain percentage of the expected value.
        assert isinstance(rsp, NotifyTimeElapsedResponsePBMsg)
        percent_error = abs(rsp.pb_message.elapsed_time - milliseconds) / milliseconds
        if percent_error >= percent_tolerance:
            raise RuntimeWarning(f"Node time elapsed response was off by {percent_error:.2f}%")

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
