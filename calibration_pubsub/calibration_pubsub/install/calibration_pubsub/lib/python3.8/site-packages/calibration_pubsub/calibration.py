import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32


calculation_complete = False
calibration_msg_data = []
error_msg_data = 0.0
# creating the publisher
# publishes the calibration matrices as arrays and
# the error as a float32
class CalibrationPublisher(Node):

    def __init__(self):
        super().__init__('calibration_publisher')
        self.publisher_ = self.create_publisher(String, 'calibration_matrices', 10)
        self.publisher = self.create_publisher(Float32, 'calibration_error', 10)
        if calculation_complete:
            self.calibration_callback
            self.error_callback

    def calibration_callback(self):
        msg = String()
        msg.data = calibration_msg_data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    def error_callback(self):
        msg = Float32()
        msg.data = error_msg_data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

#creating the subscriber
    '''
class CalibrationSubscriber(Node):

    def __init__(self):
        super().__init__('calibration_subscriber')
        self.subscription = self.create_subscription(String, 'tracking_system_data', self.tracking_system_listener_callback, 10)
        self.subscription = self.create_subscription(String, 'robot_data', self.robot_listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def robot_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    '''
def main(args=None):
    # contains all given matrices in tuples([robot_transformation, tracking_transformation])
    lin_equation_parts = []
    # contains all solution matrices in tuples([endeffector_to_marker, robot_to_tracking_system])
    saved_solution_matrices = []
    # for sample in sample_list:
    matrices_list = []
    matrices_list.append(get_transformation_matrices(
        [[-17, 23, 97, -73, 233, 305], [0.7, 0.55, -0.34], [0.375, -0.763, 0.931, 0.5]]))
    matrices_list.append(get_transformation_matrices(
        [[17, -23, -97, 73, -233, -305], [-0.7, -0.55, 0.34], [-0.375, 0.763, -0.931, -0.5]]))

    # TODO: matrices needs to be changed to sample_list[i] when sample_collect is implemented
    for i in range(CALIBRATION_POS_AMOUNT):
        lin_equation_parts.append(create_a_i_and_b_i(matrices_list[i]))

    saved_solution_matrices = solve_lin_equation(lin_equation_parts)

    # creating the msg data
    calibration_msg_data = saved_solution_matrices
    error_msg_data = minimize_error(matrices_list, saved_solution_matrices)
    calculation_complete = True

    # ROS2 CODE STARTS HERE!!
    # code for the publisher
    rclpy.init(args=args)

    minimal_publisher = CalibrationPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    '''
    rclpy.init(args=args)

    minimal_subscriber = CalibrationSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    '''
# TODO:create publisher and subscriber Node


# collects robots position and tracking measurements
# a sample contains the robot position, the joint angles and the tracking measurement of that position
# robot msg ->
# tracking msg -> pos(x, y, z), rot(x, y, z, w) rot is a quaternion
# returns all samples in a list
# def sample_collect():


# follows the format [theta, a, alpha, d]
# shows the dh-parameters of the ur5e robot
DH_TABLE = [[0, 0, np.pi / 2, 0.1625],
            [0, -0.425, 0, 0],
            [0, -0.3922, 0, 0],
            [0, 0, np.pi / 2, 0.1333],
            [0, 0, -np.pi / 2, 0.0997],
            [0, 0, 0, 0.0996]]
# sets how many positions should be accounted to calibrate the robot
CALIBRATION_POS_AMOUNT = 2

a = np.zeros((12*CALIBRATION_POS_AMOUNT, 24))
b = np.zeros((12*CALIBRATION_POS_AMOUNT))

# expects sample_list = [[joint_angles], [pos], [rot]]
# returns the robots and the tracking systems transformation matrices
# the robots transformation from the base to end-effector or tool
# the tracking systems transformation from the systems origin to the marker
# return: matrices = [robot_transformation, tracking_transformation]
def get_transformation_matrices(sample_list):
    joint_angles = sample_list[0]
    pos = np.array(sample_list[1])
    rot = np.array(sample_list[2])
    matrices = [rob_transformation_matrix(joint_angles), quaternion_to_transformation_matrix(pos, rot)]

    return matrices


# needs angles in degree
# computes the transformation matrix from the robots base to the end-effector
def rob_transformation_matrix(joint_angles):
    matrices = []
    final_matrix = np.identity(4)

    for angle in joint_angles:
        for row in DH_TABLE:
            row[0] = np.deg2rad(angle)
            matrices.append(dh_table_to_transformation_matrix(row))

    for matrix in matrices:
        final_matrix = final_matrix @ matrix

    return final_matrix

# turns joint angles and dh parameters into a transformation matrix
# needs to be done for every joint


def dh_table_to_transformation_matrix(dh_row):
    return np.array([[np.cos(dh_row[0]), -np.cos(dh_row[2]) * np.sin(dh_row[0]), np.sin(dh_row[2]) * np.sin(dh_row[0]),
                      dh_row[1] * np.cos(dh_row[0])],
                     [np.sin(dh_row[0]), np.cos(dh_row[2]) * np.cos(dh_row[0]), -np.sin(dh_row[2]) * np.cos(dh_row[0]),
                      dh_row[1] * np.sin(dh_row[0])],
                     [0, np.sin(dh_row[2]), np.cos(dh_row[2]), dh_row[3]],
                     [0, 0, 0, 1]])


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
# calibration". The solution contains the following transformation matrices:
# a transformation matrix for the end-effector to the marker at the first twelve places of the vector "w" and
# a transformation matrix for the robots base to the origin of the tracking system at the last twelve places of the vector
# return: [endeffector_to_marker, robot_to_tracking_system]
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
            a_i[j * 3 : j * 3 + 3, i * 3 : i * 3 + 3] = rob_rot_n
    a_i[9:12, 9:12] = rob_rot
    a_i[:, -12:] = ident_neg

    # creating the right-hand side of the equation, the vector b_i
    b_i[9:12] = rob_transl.T

    return [a_i, b_i]


def solve_lin_equation(equation_parts):

    for i in range(CALIBRATION_POS_AMOUNT):
        a[i*12 : i*12 + 12, :] = equation_parts[i][0]
        b[i*12 : i*12 + 12] = equation_parts[i][1]

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
        error += np.linalg.norm(given_matrices[0] @ saved_solution_matrices[0] - saved_solution_matrices[1] @ given_matrices[1])

    return error

if __name__ == '__main__':
    main()