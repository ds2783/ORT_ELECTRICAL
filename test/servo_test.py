from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)
loop = True

while loop:
    angle = input("Please Input angle!")
    if angle == "q":
        loop = False
        break
    for i in range(0, 6):
        kit.servo[i].angle = float(angle)
        time.sleep(1)

