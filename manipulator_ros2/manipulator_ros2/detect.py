import sys
import numpy as np
import rclpy
import cv2
from sensor_msgs.msg import Image
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

CAM_X = 0.5     # camera x in base_link (m)
CAM_Y = 0.0     # camera y in base_link (m)
Z = 0.955       # camera height above the cube's top face (m)

class Detector(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.frame = None
        self.k = None
        self.create_subscription(Image,'/top_camera/image', self.on_image, 1)
        self.create_subscription(CameraInfo, '/top_camera/camera_info', self.on_info, 1)

    def on_image(self,msg):
        # rgb8: three bytes per pixel, rows of msg.width pixels
        self.frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

    def on_info(self,msg):
        self.k = np.array(msg.k).reshape(3, 3)


def red_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower = np.array([0, 120, 70])
    upper = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def find_cube(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:
        return None
    (u, v), (w, h), angle = cv2.minAreaRect(c)
    return u, v, angle

def pixel_to_xy(u, v, k):
    x = CAM_X + (k[1,2]-v)*Z/k[1,1]
    y = CAM_Y + (k[0,2]-u)*Z/k[0,0]
    return x, y


def main():
    rclpy.init()
    nod = Detector()
    try:
        while nod.frame is None or nod.k is None :
            rclpy.spin_once(nod, timeout_sec=0.5)
        img = nod.frame
        print('frame shape:', img.shape, 'dtype:', img.dtype)
        mask = red_mask(img)
        print('red pixels:', cv2.countNonZero(mask))
        res = find_cube(mask)
        if res is not None:
           x, y = pixel_to_xy(res[0], res[1], nod.k)
           print(f'cube at x = {x:.3f} m, y = {y:.3f} m')
        print('cube found:', res)
        print('k = ',nod.k)
    finally:
        nod.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


