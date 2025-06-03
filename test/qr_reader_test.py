import time
from picamera2 import Picamera2

from qreader import QReader
import cv2

qreader = QReader()

cam = Picamera2()
cam.start()

capture_config = cam.create_still_configuration()
cam.switch_mode(capture_config)    

while True:
    time.sleep(1)
    
    img = cam.capture_array()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    qreader_out = qreader.detect_and_decode(image=image)
    print(qreader_out)

