import rclpy
from rclpy.node import Node
import time
from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from morris_interface import TfService


class TfService(Node):
    def __init__(self):
        ''' SERVICE '''
        super().__init__('tf_service')
        self.srv = self.create_service(TfService, 'tf_data', self.tf_service_callback)

        ''' LISTENER '''
        # initialisation of the subscriber
        super().__init__('tf_listener')
        # Declare and acquire `target_frame` parameter
        self.base_frame = self.declare_parameter('traget_frame', 'base').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
    def tf_service_callback(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.base_frame
        to_frame_rel = 'wrist_3_link'
        time = self.get_clock().now()
        # Look up for the transformation between base and wrist_3_link frames
        try:
            self.t = self.tf_buffer.lookup_transform(from_frame_rel, to_frame_rel, time,
                                                     timeout=rclpy.duration.Duration(seconds=0.5))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            self.robot_tf_listener()
        position = [self.t.transform.translation.x, self.t.transform.translation.y, self.t.transform.translation.z]
        rotation = [self.t.transform.rotation.x, self.t.transform.rotation.y, self.t.transform.rotation.z,
                    self.t.transform.rotation.w]
        return position, rotation

def main():
    rclpy.init()

    minimal_service = TfService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

