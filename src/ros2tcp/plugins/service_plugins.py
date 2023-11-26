"""
Service Plugins

This file contains scripts for creating ROS Service client and
registering it to the TCP socket.
"""

from rclpy.node import Node
import rclpy
from typing import Any, Callable, TypeVar
from socket import socket
from ros2tcp.plugins.srv_client import ServiceClient as SCP
from functools import partial
from rclpy.exceptions import NotInitializedException

ArgT = TypeVar("ArgT")
ReturnT = TypeVar("ReturnT")


def register_service(n_name: str, s_type: Any, s_name: str, req: Any) -> SCP:
    """Used to create a service client.

    Parameters
    ----------
    n_name : str
        Name of the node
    s_type : Any
        Type of the service
    s_name : str
        Name of the service
    req : Any
        Request to send to the server

    Returns
    -------
    SCP
        Dataclass containing created service client.
    """
    try:
        node = Node(node_name=n_name)
    except NotInitializedException:
        rclpy.init()
        node = Node(node_name=n_name)

    srv = node.create_client(
        srv_type=s_type,
        srv_name=s_name
    )

    return SCP(ros_node=node, srv_client=srv, request=req)


def register_callback(
        func: Callable[[ArgT], str]) -> Callable[[ArgT], None]:
    """Decorator for registering callback once response
    received from the server and sending the message via socket.

    Parameters
    ----------
    func : Callable[[ArgT], str]
        Callback for wrapping

    Returns
    -------
    Callable[[ArgT], None]
        wrapped callback
    """
    def wrapper(**kwargs) -> None:
        """Callback wrapper."""
        client: SCP = kwargs['client']
        node = client.ros_node
        srv = client.srv_client
        tcp_sock: socket = kwargs['socket']

        client.wait_for_srv_client()

        future = srv.call_async(client.request)

        while not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        feedback = future.result()
        response: str = func(feedback)
        tcp_sock.sendall(response.encode())

    return wrapper  # type: ignore


def register_plugin(
        srv: SCP, cb: Callable[[ArgT], ReturnT],
        key: int):
    """Registers the service with socket callback.

    Parameters
    ----------
    srv : SCP
        Service client
    cb : Callable[[ArgT], ReturnT]
        callback for service client
    key : int
        key to map with socket request.

    Returns
    -------
    Tuple[int, Callable[[ArgT], ReturnT]]
        Tuple for registering new service.
    """

    func = partial(cb, client=srv)
    return (key, func)
