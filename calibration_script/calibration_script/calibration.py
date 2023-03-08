import os
import time 
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import pandas as pd
import rclpy
from rclpy.node import Node
from morris_interface.srv import MoveService
from morris_interface.srv import TrackingData
'''
[0.049, 0.020, 0.966, 0.253]
[0.025, 0.046, 0.931, 0.362]
'''
CALIBRATION_POSITIONS = [[[0.5, -0.5, 0.45], [0.049, 0.020, 0.966, 0.253]],
                        [[0.68, -0.15, 0.46], [0.025, 0.046, 0.931, 0.362]],
                        [[0.60, -0.3, 0.53], [-0.052, -0.229, 0.971, 0.043]],
                        [[0.65, 0.03, 0.53], [-0.137, 0.214, 0.917, 0.307]],
                        [[0.441, -0.040, 0.784], [-0.085, 0.055, 0.943, 0.316]],
                        [[0.387, -0.407, 0.638], [-0.177, -0.657, 0.727, 0.088]],
                        [[0.449, -0.329, 0.562], [0.269, 0.952, -0.037, 0.142]],
                        [[0.397, -0.558, 0.752], [-0.222, 0.020, 0.933, 0.283]],
                        [[0.255, -0.400, 0.769], [-0.028, 0.002, 0.969, 0.247]],
                        [[0.260, -0.133, 0.794], [0.030, 0.006, 0.999, 0.017]],
                        [[0.323, -0.134, 0.552], [0.180, 0.007, 0.983, 0.016]],
                        [[0.094, -0.319, 0.583], [0.049, -0.016, 0.899, 0.436]],
                        [[0.292, 0.021, 0.740], [-0.082, -0.011, 0.948, -0.308]]]

TEST_AMOUNT = 10
sample_list = []


# creating the publisher
# publishes the calibration matrices as arrays and
# the error as a float32
class TfClient(Node):
    def __init__(self):
        # initialisation of the subscriber
        super().__init__('tf_client')
        self.cli = self.create_client(TrackingData, 'tf_data')
        self.req = TrackingData.Request()

    def send_request(self, get):
        self.req.name = get
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation

class ControlClient(Node):
    def __init__(self):
        super().__init__('control_client')
        self.cli = self.create_client(MoveService, 'move')
        self.req = MoveService.Request()

    def send_request(self, pos, rot, cartesian):
        self.req.position = pos
        self.req.rotation = rot
        self.req.cartesian = cartesian
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class TrackingClient(Node):
    def __init__(self):
        super().__init__('gripper_tracking_client')
        self.cli = self.create_client(TrackingData, 'gripper_pose')
        self.req = TrackingData.Request()

    ''' 
    pose needs to be string
    either gripper for the gripper pose or 
    board for the board pose
    '''
    def send_request(self, name):
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation

def main(args=None):
    ''' ROS2 CODE STARTS HERE!! '''
    # code for the publisher
    rclpy.init(args=args)

    listener = TfClient()
    move_client = ControlClient()
    gripper_client = TrackingClient()

    for pos in CALIBRATION_POSITIONS:
        pos_matrix = quaternion_to_transformation_matrix(pos[0], pos[1])
        pos_matrix = reorthogonalize_matrix(pos_matrix[:3, :3])
        pos = pos[0]
        rot = Rot.from_matrix(pos_matrix).as_quat()
        rot = [rot[0], rot[1], rot[2], rot[3]]
        sample = []
        while not move_client.cli.wait_for_service() and listener.cli.wait_for_service() and gripper_client.cli.wait_for_service():
            print("not ready...")
        print("ready")
        success = move_client.send_request(pos, rot, True)
        print('calibration: motion complete')
        if success:
            time.sleep(0.3)
            robot_sample = listener.send_request('get')
            print('calibration: tf sample collected')
            time.sleep(0.3)
            tracking_sample = gripper_client.send_request('gripper')
            print('calibration: tracking sample collected')
            time.sleep(0.3)
            print('robot_sample:' + str(robot_sample))
            print('tracking_sample:' + str(tracking_sample))
            sample.append(robot_sample)
            sample.append(tracking_sample)
            print(sample)
            sample_list.append(sample)
            print(sample_list)

        else:
            print("no sample collected!")

    # sample = [[[pos_rob], [rot_rob]], [[pos_track], [rot_track]]]
    calculation_main(sample_list)

    rclpy.shutdown()

def reorthogonalize_matrix(matrix):
    # Normalize the first column
    col1 = matrix[:, 0] / np.linalg.norm(matrix[:, 0])
    print(col1)
    # Compute the second column vector
    col2 = (matrix[:, 1] - np.dot(matrix[:, 1], col1) * col1)
    col2 = col2 / np.linalg.norm(col2)
    print(col2)
    # Compute the second column vector
    col3 = np.cross(col1, col2)
    # Construct the re-orthogonalized matrix
    matrix_orthogonal = np.stack((col1, col2, col3), axis=1)

    return matrix_orthogonal


'''
CALIBRATION METHODS
'''
# calculates the rotational matrix out of the rot quaternion([x, y, z, w]
# with the position included the method returns a transformation matrix
# return: transformation matrix tracking system to marker
def quaternion_to_transformation_matrix(pos, rot):
    rot_matrix = Rot.from_quat(rot).as_matrix()
    final_matrix = np.zeros((4, 4))
    final_matrix[:3, :3] = rot_matrix
    final_matrix[:3, -1] = pos
    final_matrix[3] = [0, 0, 0, 1]

    return final_matrix


# solves the linear equation with the system described in the paper "Non-orthogonal tool/flange and robot/world
# calibration". The solution contains the following transformation matrices: a transformation matrix for the
# end-effector to the marker at the first twelve places of the vector "w" and a transformation matrix for the robots
# base to the origin of the tracking system at the last twelve places of the vector return:
# [end_effector_to_marker, robot_to_tracking_system]
def create_a_i_and_b_i(robot, tracking):
    a_i = np.zeros((12, 24))
    b_i = np.zeros(12)
    rob_rot = robot[:3, :3]
    rob_transl = robot[:3, -1]
    # creating the left-hand side of the equation, the matrix a_i
    for i in range(3):
        for j in range(3):
            a_i[i * 4: i * 4 + 4, j * 4: j * 4 + 4] = rob_rot[i][j] * np.identity(4)
        a_i[i * 4: i * 4 + 4, i * 4 + 12: i * 4 + 16] = -tracking.T
        b_i[i * 4 + 3] = -rob_transl[i]

    return a_i, b_i


def solve_lin_equation(a_matrix, b_vector):
    w = np.linalg.lstsq(a_matrix, b_vector, rcond=None)[0]
    end_effector_to_marker = np.append(w[0:12].reshape(3, 4), [[0, 0, 0, 1]], axis=0)
    robot_to_tracking_system = np.append(w[12:24].reshape(3, 4), [[0, 0, 0, 1]], axis=0)
    print('e_to_m: ' + str(end_effector_to_marker))
    print('r_to_t: ' + str(robot_to_tracking_system))
    return end_effector_to_marker, robot_to_tracking_system


# calculates the error in the calibration
def compute_error(given_matrices, saved_solution_matrices):
    error_ident = np.linalg.inv(given_matrices[1]) @ np.linalg.inv(saved_solution_matrices[1]) \
                  @ given_matrices[0] @ saved_solution_matrices[0]

    return error_ident


def get_random_matrix():
    pos = np.random.uniform(-1.0, 1.0, (1, 3))
    rot = np.random.uniform(-1.0, 1.0, (1, 4))
    return quaternion_to_transformation_matrix(pos, rot)


def rand_test():
    np.set_printoptions(precision=3, linewidth=100000)
    x_rand = get_random_matrix()
    y_rand = get_random_matrix()
    a = np.zeros((12 * TEST_AMOUNT, 24))
    b = np.zeros((12 * TEST_AMOUNT))
    for i in range(TEST_AMOUNT):
        ri_rand = get_random_matrix()
        ti = np.linalg.inv(y_rand) @ ri_rand @ x_rand
        a_i, b_i = create_a_i_and_b_i(ri_rand, ti)
        a[i * 12: i * 12 + 12, :] = a_i
        b[i * 12: i * 12 + 12] = b_i
    end_effector_to_marker, robot_to_tracking_system = solve_lin_equation(a, b)
    end_effector_to_marker[:3, :3] = reorthogonalize_matrix(end_effector_to_marker[:3, :3])
    robot_to_tracking_system[:3, :3] = reorthogonalize_matrix(robot_to_tracking_system[:3, :3])
    print("x_rand: \n{}".format(x_rand))
    print("y_rand: \n{}".format(y_rand))
    print("x: \n{}".format(end_effector_to_marker))
    print("y: \n{}".format(robot_to_tracking_system))

    # print(compute_error())


def reorthogonalize_matrix(matrix):
    # Normalize the first column
    col1 = matrix[:, 0] / np.linalg.norm(matrix[:, 0])
    # Compute the second column vector
    col2 = (matrix[:, 1] - np.dot(matrix[:, 1], col1) * col1)
    col2 = col2 / np.linalg.norm(col2)
    # Compute the second column vector
    col3 = np.cross(col1, col2)
    # Construct the re-orthogonalized matrix
    matrix_orthogonal = np.stack((col1, col2, col3), axis=1)

    return matrix_orthogonal


def calculation_main(sample_list):
    a = np.zeros((12 * len(sample_list), 24))
    b = np.zeros((12 * len(sample_list)))
    for i, sample in enumerate(sample_list):
        robot_matrix = quaternion_to_transformation_matrix(*sample[0])
        tracking_matrix = quaternion_to_transformation_matrix(*sample[1])
        a_i, b_i = create_a_i_and_b_i(robot_matrix, tracking_matrix)
        a[i * 12: i * 12 + 12, :] = a_i
        b[i * 12: i * 12 + 12] = b_i
    end_effector_to_marker, robot_to_tracking_system = solve_lin_equation(a, b)

    end_effector_to_marker[:3, :3] = reorthogonalize_matrix(end_effector_to_marker[:3, :3])
    robot_to_tracking_system[:3, :3] = reorthogonalize_matrix(robot_to_tracking_system[:3, :3])
    end_effector_to_marker[:3, -1] = end_effector_to_marker[:3, -1] / np.linalg.norm(end_effector_to_marker[:3, -1])
    robot_to_tracking_system[:3, -1] = robot_to_tracking_system[:3, -1] / np.linalg.norm(robot_to_tracking_system[:3, -1])

    df_x = pd.DataFrame(end_effector_to_marker)
    df_y = pd.DataFrame(robot_to_tracking_system)
    # df_error = pd.DataFrame(error_matrix)

    # os.makedirs('~/floating_ws', exist_ok=True)
    df_x.to_csv('~/floating_ws/end_effector_to_marker.csv', index=False)
    df_y.to_csv('~/floating_ws/robot_to_tracking_system.csv', index=False)


if __name__ == '__main__':
    main()
