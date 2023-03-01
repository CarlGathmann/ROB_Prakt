# Includes for RTDE
import rtde_io
import rtde_receive
import time

from morris_interfaces.srv import SetUrIo
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('vacuum_control')
        # UR RTDE interface
        self.declare_parameter('robot_ip', '141.83.19.252')
        self.srv = self.create_service(SetUrIo, 'vacuum_control', self.service_callback)

    def service_callback(self, request, response):
        # Get robot ip from parameters or default
        robot_ip = self.get_parameter('robot_ip').value
        self.get_logger().info('Incoming request\na: %d ip: %s' % (request.set, robot_ip) )

        # Use UR RTDE interface to set digital out 0
        rtde_io_ = rtde_io.RTDEIOInterface(robot_ip)
        rtde_receive_ = rtde_receive.RTDEReceiveInterface(robot_ip)
        rtde_io_.setStandardDigitalOut(0, request.set == 1)
        # Wait for robot to process IO request
        time.sleep(1)
        if rtde_receive_.getDigitalOutState(0):
            response.current = 1
        else:
            response.current = 0

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
