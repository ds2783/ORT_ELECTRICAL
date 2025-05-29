import time
from picamera2 import Picamera2
import io

from qreader import QReader
import numpy as np

cam = Picamera2()
cam.start()

qreader = QReader()

while True:
    time.sleep(1)
    data = io.BytesIO()
    cam.capture_file(data, format='png')

    img = np.array(data)

    qreader_out = qreader.detect_and_decode(image=img)
    print(qreader_out)

# img = Image.open(data)
# pix = np.array(img)

