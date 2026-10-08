"""Shared ROS-only process lifecycle; stop outputs before ROS context teardown."""
import signal
import threading
import time
import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(factory):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = factory()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        # Launch can forward a second signal after the process-group signal.
        # Keep it ignored through interpreter finalization as well as cleanup.
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            if hasattr(node, 'begin_shutdown'):
                node.begin_shutdown()
                deadline = time.monotonic() + .15
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_node()
        rclpy.try_shutdown()
