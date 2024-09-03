import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
# import matplotlib.pyplot as plt
from pros_car_py.spot_util import RotMatrix3D, point_to_rad
import threading
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import rclpy
import orjson
import math

class kinematics(Node):
    
    def __init__(self):
        super().__init__('kinematics')
        self.joints = []
        self.joints_target = []

        self.target = []
        self.base_current = 0
        self.car_current = 0

        self.threshold = 0.001
        self.max_iterations = 10

        self.joints_length = []
        self.arm_length = 0

        self.base_target_angle = 0
        self.shoulder_target_angle = 0
        self.elbow_target_angle = 0

        self.initialize()

    def initialize(self):

        self.joints_target[0].rotation = np.array([0, 0, 0])
        for i in range(len(self.joints)):
            self.joints_target[i] = self.joints[i]
            if i < len(self.joints) - 1:
                self.joints_length[i] = np.linalg.norm(self.joints[i + 1] - self.joints[i])
                self.arm_length += self.joints_length[i]

    ### 改成 subscriber 的 callback function
    def resolve_fabrik(self):
        self.calculate_base_rotation()

        for iteration in range(self.max_iterations):
            self.joints_target[-1] = self.target
            for i in range(len(self.joints_target) - 2, 0, -1):
                direction = (self.joints_target[i] - self.joints_target[i + 1])
                direction /= np.linalg.norm(direction)
                self.joints_target[i] = self.joints_target[i + 1] + direction * self.joints_length[i]

            self.joints_target[1] = self.joints[1]
            for i in range(1, len(self.joints_target) - 1):
                direction = (self.joints_target[i + 1] - self.joints_target[i])
                direction /= np.linalg.norm(direction)
                self.joints_target[i + 1] = self.joints_target[i] + direction * self.joints_length[i]

            self.calculate_arm_rotation()
            if np.linalg.norm(self.joints_target[-1] - self.target) ** 2 < self.threshold ** 2:
                break

        self.publish_angle_to_ros()
    ###

    def calculate_base_rotation(self): # 這邊可能要看一下改啥 在計算基底的旋轉，讓手臂能正對著目標 
        target_vector = self.target - self.joints[0]
        temp_angle = np.arctan2(target_vector[2], target_vector[0]) - np.arctan2(0, -1)

        temp_angle = np.degrees(temp_angle)
        temp_angle -= 90 - self.car_current.eulerAngles[1]
        temp_angle = (temp_angle + 360) % 360

        self.base_target_angle = temp_angle

        temp_angle = 360 - self.base_current - temp_angle
        self.joints_target[0] = np.array([0, temp_angle, 0])

    # def target_out_of_range(self):
    #     wrist2_target = self.target - self.joints_target[3]
    #     base2_shoulder = self.joints_target[1] - self.joints_target[0]
    #     self.elbow_target_angle = 0
    #     self.shoulder_target_angle = np.degrees(np.arccos(np.dot(base2_shoulder, wrist2_target) / 
    #                                                       (np.linalg.norm(base2_shoulder) * np.linalg.norm(wrist2_target))))

    def calculate_arm_rotation(self):
        j2_to_j3 = self.joints_target[3] - self.joints_target[2]
        j1_to_j2 = self.joints_target[2] - self.joints_target[1]
        j0_to_j1 = self.joints_target[1] - self.joints_target[0]
        # j0 = base

        self.return_angle_1 = np.degrees(np.arccos(np.dot(j2_to_j3, -j1_to_j2) / 
                                                       (np.linalg.norm(j2_to_j3) * np.linalg.norm(j1_to_j2))))
        self.return_angle_2 = np.degrees(np.arccos(np.dot(j1_to_j2, -j0_to_j1) / 
                                                          (np.linalg.norm(j1_to_j2) * np.linalg.norm(j0_to_j1))))


def main(args=None):
    rclpy.init(args=args)
    node = kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()