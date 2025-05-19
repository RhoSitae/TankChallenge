from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

DEFAULT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)