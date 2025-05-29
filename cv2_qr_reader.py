import time
from picamera2 import Picamera2
from cv2 import QRCodeDetector

cv2_reader = QRCodeDetector()

cam = Picamera2()
cam.start()

capture_config = cam.create_still_configuration()
cam.switch_mode(capture_config)    

while True:
    time.sleep(1)
    
    img = cam.capture_array()
    
    cv2_out = cv2_reader.detectAndDecode(img=img)[0]
    print(cv2_out)

