import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import threading
import math

import scout.py as ctrl

class ControllerBridge(Node):

    def __init__(self):
        super().__init__('controller_bridge')

        self.get_logger().info("Controller bridge started")

        # Subscribe to Nav2 global plan
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        self.path = []
        self.executing = False

        # Init your controller
        self.init_controller()

    def init_controller(self):
        ctrl.roboclaw = ctrl.Roboclaw("/dev/ttyACM0", 115200)
        ctrl.roboclaw.open()

        if not ctrl.innit():
            self.get_logger().error("Controller init failed")
        else:
            self.get_logger().info("Controller initialized")

        # Initialize multiprocessing variables (same as your main)
        ctrl.state = ctrl.mp.Value('i', 1)
        ctrl.theta_0 = ctrl.mp.Value('d', math.pi/2)

        ctrl.t = ctrl.mp.Value('d', 0.0)
        ctrl.t_past = ctrl.mp.Value('d', 0.0)

        ctrl.phi_r = ctrl.mp.Value('d', 0.0)
        ctrl.phi_l = ctrl.mp.Value('d', 0.0)
        ctrl.x = ctrl.mp.Value('d', 0.0)
        ctrl.y = ctrl.mp.Value('d', 0.0)

        ctrl.rw_vel = ctrl.mp.Value('d', 0.0)
        ctrl.lw_vel = ctrl.mp.Value('d', 0.0)
        ctrl.vx = ctrl.mp.Value('d', 0.0)
        ctrl.vy = ctrl.mp.Value('d', 0.0)
        ctrl.v = ctrl.mp.Value('d', 0.0)

        ctrl.rw_vel_ref = ctrl.mp.Value('d', 0.0)
        ctrl.lw_vel_ref = ctrl.mp.Value('d', 0.0)

        ctrl.x_ref = ctrl.mp.Value('d', 0.0)
        ctrl.y_ref = ctrl.mp.Value('d', 0.0)

        ctrl.r_ref = ctrl.mp.Value('d', 0.0)
        ctrl.theta_arc = ctrl.mp.Value('d', 0.0)

        ctrl.lin_vel = ctrl.mp.Value('d', 0.0)
        ctrl.wz = ctrl.mp.Value('d', 0.0)

        ctrl.d_ref = ctrl.mp.Value('d', 0.0)
        ctrl.d = ctrl.mp.Value('d', 0.0)
        ctrl.errord = ctrl.mp.Value('d', 0.0)
        ctrl.output_distance = ctrl.mp.Value('d', 0.0)
        ctrl.biasd = ctrl.mp.Value('d', 1)

        ctrl.theta_ref = ctrl.mp.Value('d', 0.0)
        ctrl.theta = ctrl.mp.Value('d', 0.0)
        ctrl.errort = ctrl.mp.Value('d', 0.0)
        ctrl.output_theta = ctrl.mp.Value('d', 0.0)

        ctrl.lock = ctrl.mp.RLock()

    def plan_callback(self, msg: Path):
        if self.executing:
            self.get_logger().warn("Already executing a path, ignoring new one")
            return

        self.get_logger().info(f"Received path with {len(msg.poses)} points")

        self.path = msg.poses

        # Run in separate thread so ROS doesn't block
        threading.Thread(target=self.execute_path).start()

    def execute_path(self):
        self.executing = True

        for i, pose_stamped in enumerate(self.path):

            pose = pose_stamped.pose
            x = pose.position.x
            y = pose.position.y

            self.get_logger().info(f"Going to waypoint {i}: ({x:.2f}, {y:.2f})")

            # Send goal to your controller
            ctrl.set_ref(x_set=x, y_set=y, vel=0.5)
            ctrl.target()

        self.get_logger().info("Finished path")
        self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()