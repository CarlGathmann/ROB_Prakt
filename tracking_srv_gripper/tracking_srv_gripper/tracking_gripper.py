import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from morris_interface.srv import TrackingData

class TrackingService(Node):
    def __init__(self):
        self.position = None
        self.rotation = None
        super().__init__('tracking_srv_gripper')
        ''' SERVICE'''
        self.srv = self.create_service(TrackingData, 'gripper_pose', self.tracking_callback)
        ''' SUBSCRIBER GRIPPER '''
        self.subscription_gripper = self.create_subscription(PoseStamped, '/VacuumGripper3/pose',
                                                             self.gripper_sub_callback,
                                                             10)

        self.subscription_gripper
    ''' SUBSCRIBER GRIPPER '''
    def gripper_sub_callback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def tracking_callback(self, request, response):
        self.get_logger().info(f"Request received'{request}'")
        self.get_logger().info(f"Response" + str(self.position))
        response.position = self.position
        response.rotation = self.rotation
        self.get_logger().info(f"Response{response}'")
        return response

def main():
    rclpy.init()

    minimal_service = TrackingService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
