from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from morris_interface.srv import TrackingService

class TrackingService(Node):

    def __init__(self):
        super().__init__('tracking_service')
        ''' SERVICE'''
        self.srv = self.create_service(TrackingService, 'tracking_poses', self.tracking_callback)
        ''' SUBSCRIBER GRIPPER '''
        self.subscription = self.create_subscription(PoseStamped, '/VacuumGripper3/pose',
                                                     self.gripper_sub_callback, 10)
        ''' SUBSCRIBER BOARD '''
        self.subscription = self.create_subscription(PoseStamped, '/chessboard/pose', self.board_sub_callback, 10)

    ''' SUBSCRIBER GRIPPER '''
    def gripper_sub_callback(self):
        msg = PoseStamped()
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        return position, rotation

    ''' SUBSCRIBER BOARD '''
    def board_sub_callback(self):
        msg = PoseStamped()
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        return position, rotation

    def tracking_callback(self, request, response):
        if request == 'gripper':
            response.translation = self.gripper_sub_callback()[0]
            response.rotation = self.gripper_sub_callback()[1]
        else:
            response.translation = self.board_sub_callback()[0]
            response.rotation = self.board_sub_callback()[1]

        return response


def main():
    rclpy.init()

    minimal_service = TrackingService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()