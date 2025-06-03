import time
from picamera2 import Picamera2

from pyzbar.pyzbar import decode

cam = Picamera2()
cam.start()

capture_config = cam.create_still_configuration()
cam.switch_mode(capture_config)    

while True:
    time.sleep(1)
    
    img = cam.capture_array()
    pyzbar = decode(image=img)

    pyzbar_out = tuple(out.data.data.decode('utf-8').encode('shift-jis').decode('utf-8') for out in pyzbar)
    print(pyzbar_out)

