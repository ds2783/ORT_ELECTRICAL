import struct
import time
from icm20948 import ICM20948

class LowPassFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.last_output = [0.0, 0.0, 0.0]

    def apply(self, data):
        filtered = []
        for i in range(3):
            y = self.alpha * data[i] + (1 - self.alpha) * self.last_output[i]
            self.last_output[i] = y
            filtered.append(y)
        return tuple(filtered)

class HighPassFilter:
    def __init__(self, alpha=0.9):
        self.alpha = alpha
        self.last_input = [0.0, 0.0, 0.0]
        self.last_output = [0.0, 0.0, 0.0]

    def apply(self, data):
        filtered = []
        for i in range(3):
            y = self.alpha * (self.last_output[i] + data[i] - self.last_input[i])
            self.last_input[i] = data[i]
            self.last_output[i] = y
            filtered.append(y)
        return tuple(filtered)

if __name__ == "__main__":
    imu = ICM20948()

    accel_lpf = LowPassFilter(alpha=0.3)
    accel_hpf = HighPassFilter(alpha=0.9)

    gyro_lpf = LowPassFilter(alpha=0.2)
    gyro_hpf = HighPassFilter(alpha=0.9)

    mag_filter = LowPassFilter(alpha=0.2)

    while True:
        mag_raw = imu.read_magnetometer_data()
        accel_gyro_raw = imu.read_accelerometer_gyro_data()

        ax, ay, az, gx, gy, gz = accel_gyro_raw

        accel_raw = (ax, ay, az)
        gyro_raw = (gx, gy, gz)

        accel_filtered_lpf = accel_lpf.apply(accel_raw)
        accel_filtered_hpf = accel_hpf.apply(accel_raw)

        gyro_filtered_lpf = gyro_lpf.apply(gyro_raw)
        gyro_filtered_hpf = gyro_hpf.apply(gyro_raw)

        mag_filtered = mag_filter.apply(mag_raw)

        print(f"""
Accel Raw:     {ax:6.2f} {ay:6.2f} {az:6.2f}
Accel LPF:     {accel_filtered_lpf[0]:6.2f} {accel_filtered_lpf[1]:6.2f} {accel_filtered_lpf[2]:6.2f}
Accel HPF:     {accel_filtered_hpf[0]:6.2f} {accel_filtered_hpf[1]:6.2f} {accel_filtered_hpf[2]:6.2f}
Gyro Raw:      {gx:6.2f} {gy:6.2f} {gz:6.2f}
Gyro LPF:      {gyro_filtered_lpf[0]:6.2f} {gyro_filtered_lpf[1]:6.2f} {gyro_filtered_lpf[2]:6.2f}
Gyro HPF:      {gyro_filtered_hpf[0]:6.2f} {gyro_filtered_hpf[1]:6.2f} {gyro_filtered_hpf[2]:6.2f}
Mag Raw:       {mag_raw[0]:6.2f} {mag_raw[1]:6.2f} {mag_raw[2]:6.2f}
Mag LPF:       {mag_filtered[0]:6.2f} {mag_filtered[1]:6.2f} {mag_filtered[2]:6.2f}
""")

        time.sleep(0.2)
