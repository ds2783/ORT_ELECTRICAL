  from pmw3901 import PMW3901
  sensor = PMW3901(spi_bus=0, cs_pin=7)
  sensor.begin()
  while True:
      if sensor.motion_ready():
          x, y = sensor.read_motion()
          print(f"X: {x}, Y: {y}")
