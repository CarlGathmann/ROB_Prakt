from threading import Thread
import numpy as np
import rclpy
from rclpy.node import Node
import time
from morris_interface.srv import MoveService
from morris_interface.srv import TrackingService
from morris_interface.srv import SetUrIo

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

class VacuumClient(Node):
    def __init__(self):
        super().__init__('vacuum_client')
        self.cli = self.create_client(SetUrIo, 'gripper_on_off')
        self.req = SetUrIo.Request()
    '''
    to enable the gripper 
    on_off = 1
    to disable the gripper
    on_off = 0
    '''
    def send_request(self, on_off):
        self.req.set = on_off
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return future.result()

def main(args=None):
    rclpy.init()

    tracking_client = TrackingClient()
    control_client = ControlClient()
    vacuum_client = VacuumClient()

    ''' GAME LOGIC '''