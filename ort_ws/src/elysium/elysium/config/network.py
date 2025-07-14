DIAGNOSTIC_PERIOD = 0.5

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration

baseQoS = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0.2),
        lifespan=Duration(seconds=0.2),
        )
