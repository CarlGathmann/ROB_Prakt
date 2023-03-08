import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from morris_interface.srv import TrackingData

class TrackingBoard(Node):
    def __init__(self):
        self.position = None
        self.rotation = None
        super().__init__('tracking_srv_board')
        ''' SERVICE'''
        self.srv = self.create_service(TrackingData, 'board_pose', self.tracking_callback)
        ''' SUBSCRIBER BOARD '''
        self.subscription_board = self.create_subscription(PoseStamped, '/Chessboard/pose', self.board_sub_callback, 10)
        self.subscription_board


    ''' SUBSCRIBER BOARD '''
    def board_sub_callback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def tracking_callback(self, request, response):
        self.get_logger().info(f"Request received'{request}'")
        response.position = self.position
        response.rotation = self.rotation
        return response

def main():
    rclpy.init()

    minimal_service = TrackingBoard()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
