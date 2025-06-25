# 192.168.0.103
BASE_IP = '192.168.0.205'
PI_IP = '192.168.0.101'


COMM_PORT = 43931
PORT_MAIN_BASE = 5008
PORT_MAIN = 5020
PORT_SECONDARY = 5030


from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

tofQoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,  # Keep only up to the last 10 samples
    depth=10,  # Queue size of 10
    reliability=ReliabilityPolicy.BEST_EFFORT,  # attempt to deliver samples,
    # but lose them if the network isn't robust
    durability=DurabilityPolicy.VOLATILE,  # no attempt to persist samples.
    # deadline=
    # lifespan=
    # liveliness=
    # liveliness_lease_duration=
    # refer to QoS ros documentation and
    # QoSProfile source code for kwargs and what they do
)
