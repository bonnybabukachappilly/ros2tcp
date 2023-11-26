"""
Service Client

This file contains a dataclass for holding service client related requirements.
"""
from dataclasses import dataclass
from rclpy.node import Node  # type: ignore
from typing import Any
from rclpy.client import Client  # type: ignore


@dataclass
class ServiceClient:
    """
    Dataclass for holding service client related requirements

    ...

    Attributes
    ----------
    ros_node: Node
        Name of the ROS node
    srv_client: Client
        ROS Service Client
    request: Any
        Request to send to the ROS Service

    Methods
    -------
    wait_for_srv_client(self) -> None
        Wait for the ROS service to become active.
    """
    ros_node: Node
    srv_client: Client
    request: Any

    __slots__ = [
        'ros_node',
        'srv_client',
        'request'
    ]

    def wait_for_srv_client(self) -> None:
        """Wait for the ROS service to become active."""
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            print('Waiting for service')
        print('Service is active.')
