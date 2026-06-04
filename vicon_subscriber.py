"""
vicon_subscriber.py

ROS2 subscriber for the ros2-vicon-receiver package by OPT4SMART.
https://github.com/OPT4SMART/ros2-vicon-receiver

Differences from the old vicon-bridge:
  - Message type : geometry_msgs/msg/PoseStamped  (NOT TransformStamped)
  - Position     : msg.pose.position  (x, y, z in metres)
  - Orientation  : msg.pose.orientation  (quaternion xyzw)
  - ROS2 version : Jazzy Jalisco

Before running the node in the lab, start the vicon receiver:
    ros2 launch vicon_receiver client.launch.py \
        hostname:=<VICON_PC_IP>                 \
        topic_namespace:=vicon

Then verify it's working:
    ros2 topic list
    ros2 topic echo /vicon/<DRONE_SUBJECT>/<DRONE_SUBJECT>

TODO before lab session:
    1. Set DRONE_SUBJECT to the subject name from Vicon Tracker
    2. Confirm VICON_PC_IP with the lab instructor
    3. Run the echo command above to verify messages flow before running drone code
"""

import threading
import math

# ── ROS2 imports ──────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[vicon] WARNING: rclpy not found — running in stub/offline mode.")


# ── TODO: set these before lab ────────────────────────────────────────────────
DRONE_SUBJECT = "dronf4"
VICON_PC_IP   = "192.168.1.23"         # default from ros2-vicon-receiver docs
TOPIC = "/ubuntus_1/Ubuntus_1/Ubuntus_1"


def quaternion_to_yaw_deg(qx, qy, qz, qw):
    """
    Extract yaw (rotation around world Z) from a quaternion.
    Returns yaw in degrees, range (-180, 180].
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
if ROS2_AVAILABLE:
    class ViconSubscriber(Node):
        def __init__(self, shared_state: dict):
            super().__init__("vicon_listener")
            self.shared = shared_state
            self.shared["vicon_ok"] = False

            self.subscription = self.create_subscription(
                PoseStamped,      # <-- PoseStamped, not TransformStamped
                TOPIC,
                self._callback,
                10
            )
            self.get_logger().info(f"Subscribed to {TOPIC}")

        def _callback(self, msg: PoseStamped):
            p = msg.pose.position     # x, y, z in metres
            o = msg.pose.orientation  # quaternion x, y, z, w

            self.shared["drone_pos"] = (p.x, p.y, p.z)
            self.shared["drone_yaw"] = quaternion_to_yaw_deg(o.x, o.y, o.z, o.w)
            self.shared["vicon_ok"]  = True


def start_vicon_thread(shared_state: dict) -> threading.Thread:
    """
    Start Vicon subscriber in a background daemon thread.
    shared_state is updated in-place as messages arrive:
        shared["drone_pos"] → (x, y, z) metres
        shared["drone_yaw"] → yaw in degrees
        shared["vicon_ok"]  → True once first message received

    Swap in mission.py:
        from vicon_subscriber import start_vicon_thread
        start_vicon_thread(self.vicon)
    """
    if not ROS2_AVAILABLE:
        print("[vicon] ROS2 not available — Vicon thread not started.")
        return None

    def _spin():
        rclpy.init()
        node = ViconSubscriber(shared_state)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    t = threading.Thread(target=_spin, daemon=True)
    t.start()
    print(f"[vicon] Subscriber thread started — listening on {TOPIC}")
    return t


# ── Offline / stub mode ───────────────────────────────────────────────────────
def start_vicon_stub(shared_state: dict, pos=(0.0, 0.0, 0.7), yaw=0.0):
    """
    Populates shared_state with a fixed fake position for offline testing.
    """
    shared_state["drone_pos"] = pos
    shared_state["drone_yaw"] = yaw
    shared_state["vicon_ok"]  = True
    print(f"[vicon] STUB MODE — fake position: {pos}, yaw: {yaw}°")


# ── Self-test ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    PASS = 0
    FAIL = 0

    def check(name, got, expected, tol=0.01):
        global PASS, FAIL
        if abs(got - expected) <= tol:
            print(f"  PASS  {name}  (got {got:.2f}°)")
            PASS += 1
        else:
            print(f"  FAIL  {name}  expected {expected}°, got {got:.2f}°")
            FAIL += 1

    print("\n── Quaternion → yaw tests ──")
    check("identity → 0°",    quaternion_to_yaw_deg(0, 0, 0, 1),          0.0)
    check("90° yaw",          quaternion_to_yaw_deg(0, 0, 0.7071, 0.7071), 90.0)
    check("180° yaw",         quaternion_to_yaw_deg(0, 0, 1, 0),          180.0, tol=0.5)
    check("-90° yaw",         quaternion_to_yaw_deg(0, 0, -0.7071, 0.7071), -90.0)

    print("\n── Stub mode test ──")
    shared = {}
    start_vicon_stub(shared, pos=(1.5, -0.3, 0.7), yaw=15.0)
    assert shared["drone_pos"] == (1.5, -0.3, 0.7)
    assert shared["drone_yaw"] == 15.0
    assert shared["vicon_ok"]  is True
    print("  PASS  stub populates shared state correctly")
    PASS += 1

    print("\n── Package info ──")
    print(f"  Topic       : {TOPIC}")
    print(f"  Message type: geometry_msgs/msg/PoseStamped")
    print(f"  Vicon PC IP : {VICON_PC_IP}")
    if ROS2_AVAILABLE:
        print("  ROS2        : rclpy found — full thread available")
    else:
        print("  ROS2        : rclpy not found — expected on laptop without ROS2")

    print(f"\n{'─'*40}")
    print(f"  {PASS} passed  /  {FAIL} failed")
    if FAIL == 0:
        print("  Math correct. Full test requires ROS2 Jazzy + Vicon in lab.")