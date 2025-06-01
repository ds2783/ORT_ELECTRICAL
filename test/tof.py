import time
import board
import adafruit_vl53l4cx

i2c = board.I2C()  # SDA/SCL on your board

vl53 = adafruit_vl53l4cx.VL53L4CX(i2c)

print("VL53L4CX USB Serial Output")
model_id, module_type = vl53.model_info
print("Model ID: 0x{:0X}".format(model_id))
print("Module Type: 0x{:0X}".format(module_type))

vl53.start_ranging()

while True:
    if vl53.data_ready:
        distance = vl53.distance
        print(distance)  # Send only the raw number
        vl53.clear_interrupt()
    time.sleep(0.1)
