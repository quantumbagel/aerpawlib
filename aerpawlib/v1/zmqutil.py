"""
ZMQ Proxy utility for aerpawlib. Unchanged from legacy version.

@author: John Kesler (morzack)
"""

import zmq

from ..v2.logging import get_logger, LogComponent
from .constants import (
    ZMQ_PROXY_IN_PORT,
    ZMQ_PROXY_OUT_PORT,
)

# Configure logger
logger = get_logger(LogComponent.ZMQ)


def run_zmq_proxy():
    """
    Start a ZMQ forwarder device (XSUB/XPUB proxy).

    This proxy acts as a central hub for ZMQ-based communication between
    multiple runners. It binds to ZMQ_PROXY_IN_PORT for incoming messages
    and ZMQ_PROXY_OUT_PORT for outgoing broadcast.

    Note:
        This function is blocking and currently uses synchronous ZMQ.
        It should be called in a separate process or thread.
    """
    # Asyncio ZMQ will not be supported by v1
    # This feature will be added to v2
    zmq_context = zmq.Context()

    p_sub = zmq_context.socket(zmq.XSUB)
    p_pub = zmq_context.socket(zmq.XPUB)

    p_sub.bind(f"tcp://*:{ZMQ_PROXY_IN_PORT}")
    p_pub.bind(f"tcp://*:{ZMQ_PROXY_OUT_PORT}")

    logger.info("launching zmq proxy")
    zmq.proxy(p_sub, p_pub)
