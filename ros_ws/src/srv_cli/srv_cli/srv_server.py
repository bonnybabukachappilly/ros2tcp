from rclpy.node import Node
from std_srvs.srv import SetBool

import rclpy


class SrvServer(Node):
    def __init__(self) -> None:
        super().__init__(node_name="service_server")
        self.srv = self.create_service(
            srv_type=SetBool,
            srv_name='test_service',
            callback=self.__callback
        )

    def __callback(self, request, response):
        if request.data:
            print(f'Request: {request.data}')
            response.success = True
            response.message = 'Data Recieved'
            print(f'Response: {response}')
            return response
        response.success = False
        response.message = ' '
        print(f'Response: {response}')
        return response


def main():
    rclpy.init()
    srv_server = SrvServer()

    print('Running Service')

    rclpy.spin(srv_server)

    rclpy.shutdown()
