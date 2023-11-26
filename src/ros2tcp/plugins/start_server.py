"""
Start Server

To run the TCP and ROS
"""
import rclpy
from threading import Thread
from ros2tcp.ROS import BridgeInfo
from ros2tcp.TCPsocket import TCPSocketServer
from contextlib import suppress


def run_server(socket: TCPSocketServer) -> None:
    """Start TCP Socket AND Start ROS.

    Parameters
    ----------
    socket : TCPSocketServer
        Created socket instance
    """
    with suppress(RuntimeError):
        rclpy.init()

    bridge = BridgeInfo()

    try:
        th = Thread(target=socket.start_server, daemon=True)
        th.start()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        cls = socket.__class__
        cls.TERMINATE = True
    finally:
        th.join()
        bridge.destroy_node()
