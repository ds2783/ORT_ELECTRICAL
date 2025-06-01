from pmw3901 import PMW3901
sensor = PMW3901(spi_port=0, spi_cs_gpio=8)  # BCM pin number
sensor.begin()
while True:
    if sensor.motion_ready():
        x, y = sensor.read_motion()
        print(f"X: {x}, Y: {y}")
