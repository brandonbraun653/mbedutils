import inspect
import time
import uuid
from loguru import logger
from threading import RLock
from typing import Any, Dict, List, Optional, Union
from mbedutils.rpc.observer import MessageObserver, MessageQueue


class Publisher:

    def __init__(self):
        self._publisher_data_lock = RLock()
        self._dispatch_map: Dict[Any, List[MessageObserver]] = {}
        self._registry: Dict[uuid.UUID, MessageObserver] = {}
        self._subscriptions: Dict[uuid.UUID, MessageQueue] = {}

    def close(self) -> None:
        """
        Closes the publisher and all associated subscriptions
        Returns:
            None
        """
        with self._publisher_data_lock:
            for observer in self._registry.values():
                observer.close()

            self._dispatch_map.clear()
            self._registry.clear()
            self._subscriptions

    def subscribe(
        self, msg: Any, qty: int = 0, timeout: Union[int, float, None] = None
    ) -> uuid.UUID:
        """
        Creates a subscription to the given message type. In the background, the subscription
        will accumulate messages in a queue for later retrieval.

        Args:
            msg: An object or class type derived from BaseMessage
            qty: How many messages to receive. If zero, accept infinite messages.
            timeout: How long in seconds to wait for "qty" of messages to arrive

        Returns:
            Unique ID of the subscription
        """
        with self._publisher_data_lock:
            q = MessageQueue(msg=msg, qty=qty, timeout=timeout)
            q.unique_id = self.subscribe_observer(observer=q.observer)
            self._subscriptions[q.unique_id] = q
            return q.unique_id

    def subscribe_observer(self, observer: MessageObserver) -> uuid.UUID:
        """
        Adds an observer into the OrbitESC runtime message parsing infrastructure

        Args:
            observer: The observer being added

        Returns:
            Unique identifier for the registered observer
        """
        msg_type = observer.msg_type
        if inspect.isclass(msg_type):
            instantiated_msg = msg_type()
        else:
            instantiated_msg = msg_type

        # All good. Initialize the rest of the class.
        raw_type = instantiated_msg.__class__

        with self._publisher_data_lock:
            # Initialize a new mapping to register observers against
            if raw_type not in self._dispatch_map.keys():
                self._dispatch_map[raw_type] = []

            # Allocate the new observer entry
            new_id = uuid.uuid4()
            observer.unique_id = new_id
            self._registry[new_id] = observer
            self._dispatch_map[raw_type].append(observer)

            # Open the observer for listening
            observer.open()
            return new_id

    def unsubscribe(self, unique_id: uuid.UUID) -> None:
        """
        Removes an observer or subscription from registration
        Args:
            unique_id: ID of the observer/subscription to remove

        Returns:
            None
        """
        with self._publisher_data_lock:
            if unique_id not in self._registry.keys():
                return

            # Look up the tags used for registration
            observer = self._registry[unique_id]
            observer.close()

            # Remove the registration
            self._dispatch_map[observer.msg_type].remove(observer)
            self._registry.pop(unique_id)

            try:
                del self._subscriptions[unique_id]
            except KeyError:
                pass

    def get_subscription_data(
        self,
        unique_id: uuid.UUID,
        block: bool = True,
        flush: bool = True,
        terminate: bool = True,
    ) -> List[Any]:
        """
        Gets all data associated with a subscription
        Args:
            unique_id: ID returned from the subscription
            block: Wait for the subscription's data quantity to arrive or timeout to expire
            flush: Optionally flush the data from the subscription queue
            terminate: Unsubscribe from the associated message

        Returns:
            List of messages enqueued for the subscription
        """
        try:
            # Wait for the data to arrive or timeout to occur?
            if block:
                self._subscriptions[unique_id].await_fulfillment_or_timeout()

            # Grab the data. Might not be anything there.
            sub_data = self._subscriptions[unique_id].get_messages(flush=flush)

            # Unregister the subscription with the observer infrastructure? Renders the unique_id useless.
            if terminate:
                self.unsubscribe(unique_id)

            return sub_data
        except KeyError as e:
            logger.error(f"{repr(e)}")

        # Default return value
        return []

    def accept(self, message: Optional[Any]) -> None:
        """
        Accepts a new message to distribute to all registered observers
        Args:
            message: Observed message

        Returns:
            None
        """
        if not message:
            return

        callbacks = []
        with self._publisher_data_lock:
            # Collect subscribers to all messages
            callbacks.extend(self._dispatch_map.get(type(None), []))

            # Collect subscribers for this explicit message type
            callbacks.extend(self._dispatch_map.get(message.__class__, []))

        for cb in callbacks:
            try:
                cb.observer_function(message)
            except Exception as e:
                logger.error(f"Error invoking callback {cb}: {e}")

    def prune_expired_observers(self) -> None:
        """
        Cleanup task to ensure all expired observers are pruned
        Returns:
            None
        """
        with self._publisher_data_lock:
            removed_ids = []

            for key in list(self._registry.keys()):
                observer = self._registry[key]

                if observer.timeout and (
                    (time.time() - observer.start_time) > observer.timeout
                ):
                    logger.trace(f"Observer {str(key)} expired. Removing.")
                    del self._registry[key]
                    removed_ids.append(key)

            for key in removed_ids:
                try:
                    self._subscriptions[key].notify_expiry()
                except KeyError:
                    continue
