"""
Bridge

File contain TCP socket monitor.
"""
from rclpy.node import Node
from ros2tcp.TCPsocket import TCPSocketServer
from std_msgs.msg import String


class BridgeInfo(Node):
    """
    Publisher monitoring TCP socket clients

    ...

    Attributes
    ----------
    _publisher: Publisher
        ROS Publisher
    create_timer: Timer
        ROS callback timer

    Methods
    -------
    callback(self) -> None
        Callback for publishing client info
    """

    def __init__(self) -> None:
        """Publisher monitoring TCP socket clients"""
        super().__init__(node_name='bridge_info', namespace='bridge_info')

        self._publisher = self.create_publisher(
            msg_type=String,
            topic='client_info',
            qos_profile=10
        )

        self.create_timer(
            timer_period_sec=0.5,
            callback=self.callback
        )

    def callback(self) -> None:
        """Callback for publishing client info."""
        msg = String()

        msg.data = str(
            [str(_cli.getpeername())
                for _cli in TCPSocketServer.CONNECTED_CLIENTS.values()]
        )

        self._publisher.publish(msg=msg)
