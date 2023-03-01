import os
from threading import Thread
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
import time
from morris_interface.srv import MoveService
from morris_interface.srv import TrackingService
from morris_interface.srv import TfService
from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

CALIBRATION_POSITIONS = [[[0.620, -0.372, 0.687], [0.176, 0.947, 0.029, 0.269]],
                         [[0.596, 0.108, 0.827], [0.172, 0.628, 0.758, -0.024]],
                         [[0.253, 0.063, 1.051], [0.053, -0.025, 0.996, 0.061]]]
positions = []
sample_list = []


# creating the publisher
# publishes the calibration matrices as arrays and
# the error as a float32
class TfClient(Node):

    def __init__(self):
        # initialisation of the subscriber
        super().__init__('tf_client')
        self.cli = self.create_client(TfService, 'tf_data')
        self.req = MoveService.Request()

    def send_request(self, get):
        self.req.get = 'get'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return future.result()

class ControlClient(Node):
    def __init__(self):
        super().__init__('control_client')
        self.cli = self.create_client(MoveService, 'move')
        self.req = MoveService.Request()
    def send_request(self, pos, rot):
        self.req.translation = pos
        self.req.rotation = rot
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return future.result()

class TrackingClient(Node):
    def __init__(self):
        super().__init__('tracking_client')
        self.cli = self.create_client(TrackingService, 'tracking_poses')
        self.req = TrackingService.Request()

    ''' 
    pose needs to be string
    either gripper for the gripper pose or 
    board for the board pose
    '''
    def send_request(self, pose):
        self.req.pose = pose
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return future.result()

def main(args=None):
    ''' ROS2 CODE STARTS HERE!! '''
    # code for the publisher
    rclpy.init(args=args)

    listener = TfClient()
    tracking_client = TrackingClient()
    move_client = ControlClient()

    for pos in CALIBRATION_POSITIONS:
        while not move_client.cli.wait_for_service() and listener.cli.wait_for_service() and tracking_client.cli.wait_for_service():
            print("not ready...")
        print("ready")
        move_client.send_request(CALIBRATION_POSITIONS[0], CALIBRATION_POSITIONS[1])
        sample.append(listener.send_request())
        sample.append(tracking_client.send_request('gripper'))
        sample_list.append(sample)

    rclpy.shutdown()

    # contains all given matrices in tuples([robot_transformation, tracking_transformation])
    lin_equation_parts = []
    # contains all solution matrices in tuples([end_effector_to_marker, robot_to_tracking_system])
    # saved_solution_matrices = []
    # for sample in sample_list:
    matrices_list = []
    matrices_list.append(get_transformation_matrices(
        [[0.7, 0.55, -0.34], [0.375, -0.763, 0.931, 0.5], [0.7, 0.55, -0.34], [0.375, -0.763, 0.931, 0.5]]))
    matrices_list.append(get_transformation_matrices(
        [[-0.7, -0.55, 0.34], [-0.375, 0.763, -0.931, -0.5], [-0.7, -0.55, 0.34], [-0.375, 0.763, -0.931, -0.5]]))

    # TODO: matrices needs to be changed to sample_list[i] when sample_collect is implemented
    for i in range(len(CALIBRATION_POSITIONS)):
        lin_equation_parts.append(create_a_i_and_b_i(matrices_list[i]))

    saved_solution_matrices = solve_lin_equation(lin_equation_parts)

    df_x = pd.DataFrame(saved_solution_matrices[0])
    df_y = pd.DataFrame(saved_solution_matrices[1])
    error_msg_data = minimize_error(matrices_list, saved_solution_matrices)

    os.makedirs('~/prakt5_ws/src/calibration_pubsub', exist_ok=True)
    df_x.to_csv('~/prakt5_ws/src/calibration_pubsub/transformation_matrix_x.csv', index=False)
    df_y.to_csv('~/prakt5_ws/src/calibration_pubsub/transformation_matrix_y.csv', index=False)


# collects robots position and tracking measurements
# a sample contains the robot position, the joint angles and the tracking measurement of that position
# robot msg -> pos(x, y, z), rot(x, y, z, w) rot is a quaternion
# tracking msg -> pos(x, y, z), rot(x, y, z, w) rot is a quaternion
# returns all samples in a list
# def sample_collect():
#   if


# sets how many positions should be accounted to calibrate the robot
CALIBRATION_POS_AMOUNT = 2

a = np.zeros((12 * CALIBRATION_POS_AMOUNT, 24))
b = np.zeros((12 * CALIBRATION_POS_AMOUNT))


# expects sample_list = [[[pos_rob], [rot_rob]], [[pos_track], [rot_track]]]
# returns the robots and the tracking systems transformation matrices
# the robots transformation from the base to end-effector or tool
# the tracking systems transformation from the systems origin to the marker
# return: matrices = [robot_transformation, tracking_transformation]
def get_transformation_matrices(sample_list):
    robot_pos = np.array(sample_list[0])
    robot_rot = np.array(sample_list[1])
    tracking_pos = np.array(sample_list[2])
    tracking_rot = np.array(sample_list[3])
    matrices = [quaternion_to_transformation_matrix(robot_pos, robot_rot),
                quaternion_to_transformation_matrix(tracking_pos, tracking_rot)]
    return matrices


# calculates the rotational matrix out of the rot quaternion([x, y, z, w]
# with the position included the method returns a transformation matrix
# return: transformation matrix tracking system to marker
def quaternion_to_transformation_matrix(pos, rot):
    s = np.linalg.norm(rot)
    rot_matrix = [[1 - 2 * s * (rot[1] ** 2 + rot[2] ** 2), 2 * s * (rot[0] * rot[1] - rot[2] * rot[3]),
                   2 * s * (rot[0] * rot[2] - rot[1] * rot[3])],
                  [2 * s * (rot[0] * rot[1] + rot[2] * rot[3]), 1 - 2 * s * (rot[0] ** 2 + rot[2] ** 2),
                   2 * s * (rot[1] * rot[2] - rot[0] * rot[3])],
                  [2 * s * (rot[0] * rot[2] - rot[1] * rot[3]), 2 * s * (rot[1] * rot[2] + rot[0] * rot[3]),
                   1 - 2 * s * (rot[0] ** 2 + rot[1] ** 2)]]
    final_matrix = np.zeros((4, 4))
    final_matrix[:3, :3] = rot_matrix
    final_matrix[:3, -1] = pos
    final_matrix[3] = [0, 0, 0, 1]

    return final_matrix


# separates the robot transformation matrix in the rotational and the translation part
# return: [rotation, translation]
def separate_transformation_matrix(robot_transformation_matrix):
    translation = robot_transformation_matrix[:3, -1]
    rotation = robot_transformation_matrix[:3, :-1]

    return [rotation, translation]


# solves the linear equation with the system described in the paper "Non-orthogonal tool/flange and robot/world
# calibration". The solution contains the following transformation matrices: a transformation matrix for the
# end-effector to the marker at the first twelve places of the vector "w" and a transformation matrix for the robots
# base to the origin of the tracking system at the last twelve places of the vector return: [endeffector_to_marker,
# robot_to_tracking_system]
def create_a_i_and_b_i(matrices):
    ident_neg = -np.identity(12)
    a_i = np.zeros((12, 24))
    b_i = np.zeros(12)
    tracking_matrix = matrices[1]
    rob_rot = separate_transformation_matrix(matrices[0])[0]
    rob_transl = separate_transformation_matrix(matrices[0])[1]

    # creating the left-hand side of the equation, the matrix a_i
    for i in range(3):
        for j in range(4):
            rob_rot_n = rob_rot * tracking_matrix[i][j]
            a_i[j * 3: j * 3 + 3, i * 3: i * 3 + 3] = rob_rot_n
    a_i[9:12, 9:12] = rob_rot
    a_i[:, -12:] = ident_neg

    # creating the right-hand side of the equation, the vector b_i
    b_i[9:12] = rob_transl.T

    return [a_i, b_i]


def solve_lin_equation(equation_parts):
    for i in range(CALIBRATION_POS_AMOUNT):
        a[i * 12: i * 12 + 12, :] = equation_parts[i][0]
        b[i * 12: i * 12 + 12] = equation_parts[i][1]

    w = np.linalg.lstsq(a, b, rcond=None)[0]
    endeffector_to_marker, robot_to_tracking_system = np.zeros((4, 4)), np.zeros((4, 4))

    # creating the solution matrices
    for i in range(3):
        endeffector_to_marker[i, :] = w[i * 4: i * 4 + 4]
        robot_to_tracking_system[i, :] = w[i * 4 + 12: i * 4 + 16]
    # sets the trivial row
    endeffector_to_marker[3, 3] = 1
    robot_to_tracking_system[3, 3] = 1

    return [endeffector_to_marker, robot_to_tracking_system]


# calculates the error in the calibration with the frobenius norm
def minimize_error(saved_given_matrices, saved_solution_matrices):
    error = 0
    for i, given_matrices in enumerate(saved_given_matrices):
        error += np.linalg.norm(
            given_matrices[0] @ saved_solution_matrices[0] - saved_solution_matrices[1] @ given_matrices[1])

    return float(error)


if __name__ == '__main__':
    main()
