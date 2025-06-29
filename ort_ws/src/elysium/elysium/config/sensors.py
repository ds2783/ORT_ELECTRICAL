DISTANCE_SENSOR_REFRESH_PERIOD = 0.2
BMS_REFRESH_PERIOD = 1
BMS_DELTA_T = 0.02
BMS_ROLLING_AVERAGE_SECONDS = 5
BMS_BATTERY_CAPACITY = 5000  # in mAh
BMS_UNDERVOLT_WARN = 11  # threshold voltage for function. If each cell has 3V or less, the rover should not be moving. 
BMS_UNDERVOLT_SHUTDOWN = 9.5  # threshold voltage for the Pi to function. Under this voltage the Pi should shut down. 
BMS_SAVE_PATH = "/ros/battery_save.txt"
BMS_LOOKUP_TABLE_PATH = "/ros/ocv_lookup.csv"
IMU_SENSOR_PERIOD = 0.2
IMU_UPDATE_FREQUENCY = 80 
OPTICAL_CALIBRATION = 1
GPS_REFRESH_PERIOD = 1
OCV_ARRAY_SIZE = 1000

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

tofQoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, # Keep only up to the last 10 samples
    depth=10,  # Queue size of 10
    reliability=ReliabilityPolicy.BEST_EFFORT,  # attempt to deliver samples, 
    # but lose them if the network isn't robust
    durability=DurabilityPolicy.VOLATILE, # no attempt to persist samples. 
    # deadline=
    # lifespan=
    # liveliness=
    # liveliness_lease_duration=

    # refer to QoS ros documentation and 
    # QoSProfile source code for kwargs and what they do
)
