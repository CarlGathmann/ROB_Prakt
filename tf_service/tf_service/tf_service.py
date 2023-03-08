import rclpy
from rclpy.node import Node
import time
from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from morris_interface.srv import TrackingData


class TfService(Node):
    def __init__(self):

        ''' SERVICE '''
        super().__init__('tf_service')
        self.srv = self.create_service(TrackingData, 'tf_data', self.tf_service_callback)
        ''' LISTENER '''
        # Declare and acquire `target_frame` parameter
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ''' TIMER '''
        # Call on_timer function every second
        #self.timer = self.create_timer(1.0, self.on_timer)
    '''
    needed to get the msg data
    t stores the transform data as a msgs
    translation data:
        t.transform.translation.x
        t.transform.translation.y
        t.transform.translation.z
    orientation data:
        t.transform.rotation.x
        t.transform.rotation.y
        t.transform.rotation.z
        t.transform.rotation.w
    '''

    def tf_service_callback(self, request, response):
        self.get_logger().info(f"Request recived'{request}'")
        # Store frame names in variables that will be used to
        # compute transformations
        time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.25)
        # Look up for the transformation between base and wrist_3_link frames
        try:
            t = self.tf_buffer.lookup_transform('base', 'wrist_3_link', time, timeout=rclpy.duration.Duration(seconds=1.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            self.tf_service_callback(request, response)
        position = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        rotation = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                    t.transform.rotation.w]
        response.position = position
        response.rotation = rotation
        return response

def main():
    rclpy.init()

    minimal_service = TfService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
