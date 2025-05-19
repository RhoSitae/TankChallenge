# common/logger.py
import rclpy.logging

def get_logger(node):
    log = node.get_logger()
    log.set_level(rclpy.logging.LoggingSeverity.INFO)
    return log