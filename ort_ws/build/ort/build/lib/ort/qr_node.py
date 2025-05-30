import rclpy
from rclpy.node import Node

import cv2
from qreader import QReader
from picamera2 import Picamera2

class QRCam(Node):
    def __init__(self):
        super().__init__('qr_node')

def main():
    print('Hi from ort.')


if __name__ == '__main__':
    main()
