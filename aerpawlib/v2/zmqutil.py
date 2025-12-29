"""
ZMQ utilities for aerpawlib v2 API.

This module provides ZMQ proxy and messaging utilities for inter-vehicle
and inter-process communication.

Features:
- Async-compatible pub/sub messaging
- Background proxy process management
- Type-safe message handling
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional
import json
import logging

import zmq
try:
    import zmq.asyncio
    ZMQ_ASYNC_AVAILABLE = True
except ImportError:
    ZMQ_ASYNC_AVAILABLE = False

# Use modular logging system
from .logging import get_logger, LogComponent
logger = get_logger(LogComponent.ZMQ)


# Default ports for the ZMQ proxy
ZMQ_PROXY_IN_PORT = 5570
ZMQ_PROXY_OUT_PORT = 5571


class MessageType(Enum):
    """Types of ZMQ messages used in aerpawlib."""
    STATE_TRANSITION = "state_transition"
    FIELD_REQUEST = "field_request"
    FIELD_CALLBACK = "field_callback"
    CUSTOM = "custom"


# Backward compatibility aliases
ZMQ_TYPE_TRANSITION = MessageType.STATE_TRANSITION.value
ZMQ_TYPE_FIELD_REQUEST = MessageType.FIELD_REQUEST.value
ZMQ_TYPE_FIELD_CALLBACK = MessageType.FIELD_CALLBACK.value


@dataclass
class ZMQProxyConfig:
    """Configuration for ZMQ proxy."""
    in_port: int = ZMQ_PROXY_IN_PORT
    out_port: int = ZMQ_PROXY_OUT_PORT
    bind_address: str = "*"


def run_zmq_proxy(config: Optional[ZMQProxyConfig] = None):
    """
    Run a ZMQ XSUB/XPUB proxy.

    This function blocks and should be run in a separate process.
    The proxy allows multiple publishers and subscribers to communicate
    through a central hub.

    Args:
        config: Proxy configuration. Uses defaults if not provided.

    Example:
        # In a separate process:
        from aerpawlib.v2.zmqutil import run_zmq_proxy
        run_zmq_proxy()
    """
    if config is None:
        config = ZMQProxyConfig()

    zmq_context = zmq.Context()

    p_sub = zmq_context.socket(zmq.XSUB)
    p_pub = zmq_context.socket(zmq.XPUB)

    p_sub.bind(f"tcp://{config.bind_address}:{config.in_port}")
    p_pub.bind(f"tcp://{config.bind_address}:{config.out_port}")

    logger.info(
        f"[aerpawlib] Launching ZMQ proxy on ports {config.in_port}/{config.out_port}"
    )
    print(f"[aerpawlib] launching zmq proxy")

    try:
        zmq.proxy(p_sub, p_pub)
    finally:
        p_sub.close()
        p_pub.close()
        zmq_context.term()


@dataclass
class ZMQMessage:
    """
    A typed ZMQ message.

    Attributes:
        topic: The message topic/channel
        message_type: Type of message (from MessageType enum)
        payload: The message payload (will be JSON serialized)
        sender_id: Optional identifier for the sender
    """
    topic: str
    message_type: MessageType
    payload: Any
    sender_id: Optional[str] = None

    def serialize(self) -> bytes:
        """Serialize the message for transmission."""
        data = {
            "type": self.message_type.value,
            "payload": self.payload,
            "sender_id": self.sender_id
        }
        return f"{self.topic} {json.dumps(data)}".encode('utf-8')

    @classmethod
    def deserialize(cls, data: bytes) -> ZMQMessage:
        """Deserialize a received message."""
        decoded = data.decode('utf-8')
        # Split topic from payload (topic is first word)
        parts = decoded.split(' ', 1)
        if len(parts) != 2:
            raise ValueError(f"Invalid message format: {decoded}")

        topic = parts[0]
        payload_data = json.loads(parts[1])

        return cls(
            topic=topic,
            message_type=MessageType(payload_data.get("type", "custom")),
            payload=payload_data.get("payload"),
            sender_id=payload_data.get("sender_id")
        )


class ZMQPublisher:
    """
    Async-compatible ZMQ publisher.

    Example:
        async with ZMQPublisher("localhost") as pub:
            await pub.publish("drone_position", {"lat": 35.7, "lon": -78.6})
    """

    def __init__(
        self,
        proxy_address: str = "localhost",
        proxy_port: int = ZMQ_PROXY_IN_PORT,
        sender_id: Optional[str] = None
    ):
        """
        Initialize a ZMQ publisher.

        Args:
            proxy_address: Address of the ZMQ proxy
            proxy_port: Port of the proxy's subscriber socket
            sender_id: Optional identifier for this publisher
        """
        self._proxy_address = proxy_address
        self._proxy_port = proxy_port
        self._sender_id = sender_id
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False

    async def connect(self):
        """Connect to the ZMQ proxy."""
        if ZMQ_ASYNC_AVAILABLE:
            self._context = zmq.asyncio.Context()
            self._socket = self._context.socket(zmq.PUB)
        else:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.PUB)

        self._socket.connect(f"tcp://{self._proxy_address}:{self._proxy_port}")
        self._connected = True

        # Give ZMQ time to establish connection
        await asyncio.sleep(0.1)

    async def disconnect(self):
        """Disconnect from the ZMQ proxy."""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        self._connected = False

    async def __aenter__(self) -> ZMQPublisher:
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()
        return False

    async def publish(
        self,
        topic: str,
        payload: Any,
        message_type: MessageType = MessageType.CUSTOM
    ):
        """
        Publish a message.

        Args:
            topic: The topic/channel to publish to
            payload: The message payload (must be JSON serializable)
            message_type: Type of message
        """
        if not self._connected:
            raise RuntimeError("Publisher not connected. Call connect() first.")

        message = ZMQMessage(
            topic=topic,
            message_type=message_type,
            payload=payload,
            sender_id=self._sender_id
        )

        if ZMQ_ASYNC_AVAILABLE:
            await self._socket.send(message.serialize())
        else:
            self._socket.send(message.serialize())


class ZMQSubscriber:
    """
    Async-compatible ZMQ subscriber.

    Example:
        async with ZMQSubscriber("localhost", topics=["drone_position"]) as sub:
            async for message in sub:
                print(f"Received: {message.payload}")
    """

    def __init__(
        self,
        proxy_address: str = "localhost",
        proxy_port: int = ZMQ_PROXY_OUT_PORT,
        topics: Optional[list[str]] = None
    ):
        """
        Initialize a ZMQ subscriber.

        Args:
            proxy_address: Address of the ZMQ proxy
            proxy_port: Port of the proxy's publisher socket
            topics: List of topics to subscribe to. Empty list subscribes to all.
        """
        self._proxy_address = proxy_address
        self._proxy_port = proxy_port
        self._topics = topics if topics is not None else []
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False
        self._running = False

    async def connect(self):
        """Connect to the ZMQ proxy."""
        if ZMQ_ASYNC_AVAILABLE:
            self._context = zmq.asyncio.Context()
            self._socket = self._context.socket(zmq.SUB)
        else:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.SUB)

        self._socket.connect(f"tcp://{self._proxy_address}:{self._proxy_port}")

        # Subscribe to topics
        if not self._topics:
            self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        else:
            for topic in self._topics:
                self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)

        self._connected = True
        self._running = True

    async def disconnect(self):
        """Disconnect from the ZMQ proxy."""
        self._running = False
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        self._connected = False

    async def __aenter__(self) -> ZMQSubscriber:
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()
        return False

    async def receive(self, timeout: Optional[float] = None) -> Optional[ZMQMessage]:
        """
        Receive a single message.

        Args:
            timeout: Timeout in seconds. None for blocking wait.

        Returns:
            The received message, or None if timeout occurred.
        """
        if not self._connected:
            raise RuntimeError("Subscriber not connected. Call connect() first.")

        if timeout is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
        else:
            self._socket.setsockopt(zmq.RCVTIMEO, -1)

        try:
            if ZMQ_ASYNC_AVAILABLE:
                data = await self._socket.recv()
            else:
                data = self._socket.recv()
            return ZMQMessage.deserialize(data)
        except zmq.Again:
            return None

    def __aiter__(self):
        return self

    async def __anext__(self) -> ZMQMessage:
        """Async iterator for receiving messages."""
        if not self._running:
            raise StopAsyncIteration

        message = await self.receive()
        if message is None:
            raise StopAsyncIteration
        return message

    async def iter_messages(self):
        """
        Async generator for receiving messages.

        Yields messages until stop() is called.

        Example:
            async for message in subscriber.iter_messages():
                process(message)
        """
        while self._running:
            message = await self.receive(timeout=0.1)
            if message is not None:
                yield message

    def stop(self):
        """Stop receiving messages (breaks the async iterator)."""
        self._running = False

