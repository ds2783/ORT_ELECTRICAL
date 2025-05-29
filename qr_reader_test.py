import time
from picamera2 import Picamera2
import io

from qreader import QReader
import numpy as np
from PIL import Image

cam = Picamera2()
cam.start()

qreader = QReader()

while True:
    time.sleep(1)
    data = io.BytesIO()
    cam.capture_file(data, format='png')

    img = Image.open(data)
    pix = np.array(data)

    qreader_out = qreader.detect_and_decode(image=pix)
    print(qreader_out)

# pix = np.array(img)

