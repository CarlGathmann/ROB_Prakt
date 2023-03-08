import time
from threading import Thread
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from morris_interface.srv import MoveService
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e
from morris_interface.srv import TrackingData

class TfClient(Node):
    def __init__(self):
        super().__init__('tf_client_control')
        self.cli = self.create_client(TrackingData, 'tf_data')
        self.req = TrackingData.Request()

    def send_request(self, get):
        self.req.name = get
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation

class RobotControl(Node):
    def __init__(self):
        super().__init__('ur5e_control')
        ''' MOVE SERVICE '''
        self.srv = self.create_service(MoveService, 'move',
                                       self.move_executed_callback)  # service for moving/controlling the robot

    def move_executed_callback(self, request, response):
        listener = TfClient()
        callback_node = Node('callback_node')
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=callback_node,
            joint_names=ur5e.joint_names(),
            base_link_name=ur5e.base_link_name(),
            end_effector_name=ur5e.end_effector_name(),
            group_name=ur5e.MOVE_GROUP_ARM,
            callback_group=callback_group,
            ignore_new_calls_while_executing=True
        )
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(callback_node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        self.get_logger().info(f"Goal Pose: {request.position}, {request.rotation} Cartesian: {request.cartesian}")

        moveit2.move_to_pose(position=request.position, quat_xyzw=request.rotation, cartesian=request.cartesian, frame_id="base")
        moveit2.wait_until_executed()
        self.get_logger().info(f"Movement stopped")
        time.sleep(1)
        delta = float('inf')
        current_pos, current_rot = listener.send_request('get')
        self.get_logger().info(f"Service response received.")
        delta_pos = np.array(current_pos) - np.array(request.position)
        delta_rot = np.array(current_rot) - np.array(request.rotation)
        delta_matrix = quaternion_to_transformation_matrix(delta_pos, delta_rot)
        pose_delta = np.linalg.norm(delta_matrix)
        print(pose_delta)
        while pose_delta < delta - 0.001:
            delta = pose_delta
            if pose_delta < 0.001:
                response.done = True
                self.get_logger().info(f"Pose reached with error {delta}.")
                return response
            self.get_logger().info(f"Pose not yet reached")
            time.sleep(1)
            current_pos, current_rot = listener.send_request('get')
            delta_pos = np.array(request.position) - np.array(current_pos)
            delta_rot = np.array(request.rotation) - np.array(current_rot)
            delta_matrix = quaternion_to_transformation_matrix(delta_pos, delta_rot)
            pose_delta = np.linalg.norm(delta_matrix)
            self.get_logger().info(f"New error {pose_delta}")

        response.done = False
        return response


def quaternion_to_transformation_matrix(pos, rot):
    rot_matrix = Rot.from_quat(rot).as_matrix()
    final_matrix = np.zeros((4, 4))
    final_matrix[:3, :3] = rot_matrix
    final_matrix[:3, -1] = pos
    final_matrix[3] = [0, 0, 0, 1]

    return final_matrix
def main():
    rclpy.init()

    # Create node for this example
    node = RobotControl()

    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
