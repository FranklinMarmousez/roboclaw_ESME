import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import math

import scout_nav2.py as ctrl


def quaternion_to_yaw(q):
    """Convert quaternion to yaw (theta)"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ControllerBridge(Node):

    def __init__(self):
        super().__init__('controller_bridge')

        self.get_logger().info("Controller bridge started")

        self.path = []
        self.executing = False

        # --- Subscribers ---
        self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # Preferred (Nav2 localization)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Fallback (if no AMCL)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.use_amcl = False

        self.init_controller()

    # -----------------------------
    # Controller Initialization
    # -----------------------------
    def init_controller(self):
        ctrl.roboclaw = ctrl.Roboclaw("/dev/ttyACM0", 115200)
        ctrl.roboclaw.open()

        if not ctrl.innit():
            self.get_logger().error("Controller init failed")
        else:
            self.get_logger().info("Controller initialized")

        ctrl.state = ctrl.mp.Value('i', 1)
        ctrl.theta_0 = ctrl.mp.Value('d', 0.0)

        ctrl.t = ctrl.mp.Value('d', 0.0)
        ctrl.t_past = ctrl.mp.Value('d', 0.0)

        ctrl.phi_r = ctrl.mp.Value('d', 0.0)
        ctrl.phi_l = ctrl.mp.Value('d', 0.0)

        ctrl.x = ctrl.mp.Value('d', 0.0)
        ctrl.y = ctrl.mp.Value('d', 0.0)

        ctrl.theta = ctrl.mp.Value('d', 0.0)

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
        ctrl.errort = ctrl.mp.Value('d', 0.0)
        ctrl.output_theta = ctrl.mp.Value('d', 0.0)

        ctrl.lock = ctrl.mp.RLock()

    # -----------------------------
    # Localization Callbacks
    # -----------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.use_amcl = True

        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)

        with ctrl.lock:
            ctrl.x.value = pose.position.x
            ctrl.y.value = pose.position.y
            ctrl.theta.value = yaw

    def odom_callback(self, msg: Odometry):
        if self.use_amcl:
            return  # ignore odom if AMCL is active

        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)

        with ctrl.lock:
            ctrl.x.value = pose.position.x
            ctrl.y.value = pose.position.y
            ctrl.theta.value = yaw

    # -----------------------------
    # Path Handling
    # -----------------------------
    def plan_callback(self, msg: Path):
        if self.executing:
            self.get_logger().warn("Already executing path")
            return

        self.get_logger().info(f"Received path: {len(msg.poses)} points")

        self.path = msg.poses

        threading.Thread(target=self.execute_path, daemon=True).start()

    def execute_path(self):
        self.executing = True

        for i, pose_stamped in enumerate(self.path):
            pose = pose_stamped.pose
            x = pose.position.x
            y = pose.position.y

            self.get_logger().info(f"Waypoint {i}: ({x:.2f}, {y:.2f})")

            ctrl.set_ref(x_set=x, y_set=y, vel=0.5)

            # Run target in separate thread (NON-BLOCKING)
            t = threading.Thread(target=ctrl.target)
            t.start()
            t.join()  # wait for this waypoint to finish (but ROS still runs)

        self.get_logger().info("Path completed")
        self.executing = False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()