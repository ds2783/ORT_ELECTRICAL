from machine import Pin, SPI
import time

class PMW3901:
    def __init__(self, spi, cs_pin):
        self.cs = Pin(cs_pin, Pin.OUT)
        self.spi = spi
        self.cs.value(1)
        time.sleep(0.1)
        self._initialize()

    def _write(self, reg, value):
        self.cs.value(0)
        self.spi.write(bytearray([reg | 0x80, value]))
        self.cs.value(1)
        time.sleep_us(50)

    def _read(self, reg):
        self.cs.value(0)
        self.spi.write(bytearray([reg & 0x7F]))
        val = self.spi.read(1)
        self.cs.value(1)
        time.sleep_us(50)
        return val[0]

    def _initialize(self):
        self._write(0x7F, 0x00)
        product_id = self._read(0x00)
        if product_id != 0x49:
            raise RuntimeError("Wrong ID: 0x{:02X}".format(product_id))

        self._write(0x7F, 0x00)
        self._write(0x55, 0x01)
        time.sleep(0.05)
        self._write(0x50, 0x07)
        time.sleep(0.05)


    def read_motion(self):
        self._write(0x7F, 0x00)          # Ensure in the correct register bank
        motion = self._read(0x02)       # Read motion status (triggers capture)
        dx = self._read(0x03)
        dy = self._read(0x04)
        if dx > 127: dx -= 256
        if dy > 127: dy -= 256
        return dx, dy


# Setup SPI and CS
spi = SPI(0, baudrate=2000000, polarity=1, phase=1,
          sck=Pin(2), mosi=Pin(3), miso=Pin(0))
cs = 1  # Chip select on GP1

sensor = PMW3901(spi, cs)

while True:
    dx, dy = sensor.read_motion()
    print("DX:", dx, "DY:", dy)
    time.sleep(0.1)

