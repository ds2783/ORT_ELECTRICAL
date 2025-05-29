import time
from picamera2 import Picamera2
import io

from qreader import QReader
import numpy as np

import cv2

cam = Picamera2()
cam.start()

qreader = QReader()

while True:
    time.sleep(1)
    data = io.BytesIO()
    cam.capture_file(data, format='png')
    
    file_bytes = np.asarray(bytearray(data.read()), dtype=np.uint8)
    
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

    qreader_out = qreader.detect_and_decode(image=img)
    print(qreader_out)

