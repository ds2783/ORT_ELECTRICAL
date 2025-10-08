from modified_streaming.ortStreaming import PORT_SECONDARY_SERVER


PI_IP = "192.168.0.101"

COMM_PORT = 43931
PORT_MAIN_SERVER = 5020
PORT_MAIN = 5021
PORT_SECONDARY = 5030
PORT_SECONDARY_SERVER = 5031 


class RetCodes:
    SUCCESS = 0
    FAIL = 1
    FAIL_UNRECOGNISED_OP_CODE = 2
    FAIL_DETECTED_NO_TOF_FORWARD = 3
    FAIL_DETECTED_NO_OFS_FORWARD = 4
    FAIL_TOF_DETECTED_NO_REASONABLE_RANGE = 5


# Service
CODE_TERMINATE = 0
CODE_CONTINUE = 1


from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration

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

baseQoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

flagQoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    liveliness_lease_duration=Duration(seconds=1000),
)

queueToS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

periodicQoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
