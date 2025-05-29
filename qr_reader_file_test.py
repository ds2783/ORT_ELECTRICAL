import time
from picamera2 import Picamera2

from qreader import QReader

import cv2

cam = Picamera2()
capture_config = cam.create_still_configuration()
cam.start()

qreader = QReader()

cam.switch_mode(capture_config)    

while True:
    time.sleep(1)
    
    cam.capture_file("temp.png")
    
    image = cv2.cvtColor(cv2.imread("temp.png"), cv2.COLOR_BGR2RGB)

    qreader_out = qreader.detect_and_decode(image=image)
    print(qreader_out)

