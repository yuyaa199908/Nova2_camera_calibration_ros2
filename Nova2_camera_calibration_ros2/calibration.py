import rclpy
from rclpy.node import Node
import logging

# from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import Point,Pose, Vector3, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

# add
import numpy as np
from cv_bridge import CvBridge
import cv2
# from ctypes import * # convert float to uint32
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from collections import deque

MODE = [""]
class CalibrationHandEye(Node):
    def __init__(self):
        super().__init__('calibration_hand_eye')
        self.init_param()
        self.init_value()
        self.sub_image = self.create_subscription(Image(), '/input_img', self.CB_get_img,10)
        self.sub_cmd = self.create_subscription(String(), '/input_cmd', self.CB_main,10)

    # https://qiita.com/am8/items/5c5343a21c9b27bbb2c0
    def CB_main(self,msg):
        if self.img != None and self.caminfo != None:
            ret, corners = cv2.findChessboardCorners(self.img, self.chess_size, None)
            if ret:
                flag, pose_chess2cam = self.get_chess2cam(corners)
                if flag:
                    self.pose_chess2cam_list.append(pose_chess2cam)
                else:
                    self.get_logger().info(f"failed to get pose chess -> camera !")
            else:
                self.get_logger().info(f"failed to get corners !")
        else:
            self.get_logger().info(f"failed to get image !")

    def CB_get_img(self,msg):
        self.img = CvBridge().imgmsg_to_cv2(msg, cv2.COLOR_BGR2GRAY)

    def get_chess2cam(self, corners):
        ret, rvec, tvec = cv2.solvePnP(self.objp, corners, self.cam_K, self.cam_D )
        if ret:
            r , _ = cv2.Rodrigues(rvec)
            t = tvec.reshape(3)
            p = np.eye(4)
            p[:3,:3] = r
            p[:3,3] = t
            return True, p
        else:
            return False, None

    def init_param(self):
        self.marker_dir = "./marker"
        self.marker_type = cv2.aruco.DICT_4X4_50
        self.chess_size = [6,9]
        self.chess_square_size = 0.1    #[m]

        self.cam_K = [641.9744873046875, 0.0, 637.8692016601562,
                      0.0, 641.239501953125, 364.94647216796875, 
                      0.0, 0.0, 1.0]
        #   [k1, k2, p1, p2, pk3]
        self.cam_D = [-0.057686787098646164, 0.06685127317905426,-0.0002468553720973432, 0.0007276988471858203,0.021987270563840866]

    def init_value(self):
        self.img = None
        self.caminfo = None

        self.objp = np.zeros((self.chess_size[0] * self.chess_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chess_size[0], 0:self.chess_size[1]].T.reshape((-1, 2))
        self.objp *= self.chess_square_size
        self.cam_K = np.array(self.cam_K).reshape((3,3))
        self.cam_D = np.array(self.cam_D)

        self.pose_chess2cam_list = []


def main():
    rclpy.init()
    node = CalibrationHandEye()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()