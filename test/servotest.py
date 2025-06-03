import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)
now = time.time()
start = time.time()
while now<(start + 10):
    now = time.time()
    for i in range(0,16):
        kit.continuous_servo[i].throttle = 1
    time.sleep(2)
    for i in range (0,16):
        kit.continuous_servo[i].throttle = -1
    time.sleep(2)
    for i in range (0,16):
        kit.continuous_servo[i].throttle = 0
