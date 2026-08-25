"""
Stage 5C.3 - Trivial external controller : first code on the CONTROLLER side.

WHAT THIS IS
    An EXTERNAL ROS 2 node -- the mirror of the sim plant node, and the first
    thing you author that runs on the CONTROLLER end of the network seam. It:
      - PUBLISHES /joint_command  (sends target positions -- the "input" that
        commands the arm)
      - SUBSCRIBES /joint_states  (reads current positions -- the "output" that
        shows where the arm actually is)
    and prints a readable commanded-vs-actual line a few times a second.

    This runs on the NATIVE Jazzy side (system Python 3.12, /opt/ros/jazzy
    sourced) -- NOT the Isaac bundle. That is the whole point of the two-process
    architecture: the controller never imports Isaac; it talks to the sim ONLY
    over DDS topics. See the .sh wrapper (it sources native Jazzy, unlike the
    sim-side wrappers which use the bundle).

    IT IS DELIBERATELY *NOT* A FEEDBACK CONTROLLER. The command it publishes is a
    fixed setpoint that does NOT depend on the state it reads. It commands, and
    it observes -- but it does not yet THINK. Closing the feedback path (command
    computed from error = setpoint - actual) is the PID work of the next stage.
    Here the value is: prove an external node drives AND reads the loop, and get
    a clean human-readable readout instead of the `ros2 topic echo` firehose.

WHAT YOU'LL SEE
    A single line, refreshed a few times a second, e.g.:
        joint4  cmd -0.500  act -0.481  err  0.019
    showing the setpoint you commanded vs. where the arm actually is. Watch
    'act' converge toward 'cmd' after you (re)set the target -- that convergence
    is the Franka's internal PD drive doing the work, NOT this node. This is the
    commanded-vs-actual picture the PID stage will act on.

USAGE
    Edit SETPOINTS below (joint name -> target radians), or leave the default.
    ./stage5c3_trivial_controller.sh
    Ctrl+C to stop. (On stop it does NOT re-home the arm -- the sim node just
    stops receiving new commands and holds the last target.)

    Runs against the bidirectional sim node (stage5c3_bidirectional_node).
    Start the sim node first, then this.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
STATE_TOPIC   = "/joint_states"    # SUBSCRIBE: where the arm IS
COMMAND_TOPIC = "/joint_command"   # PUBLISH:   where we tell it to GO
NODE_NAME     = "trivial_controller"

# The fixed setpoint(s) this controller commands. joint name -> target radians.
# These do NOT change in response to state -- that's the point (open-loop).
SETPOINTS = {
    "panda_joint4": -0.5,
}

CMD_RATE_HZ     = 10.0    # how often we PUBLISH the command
DISPLAY_RATE_HZ = 4.0     # how often we PRINT the readable status line


class TrivialController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # OUTPUT port: publish target positions.
        self.cmd_pub = self.create_publisher(JointState, COMMAND_TOPIC, 10)

        # INPUT port: subscribe to current state, stash the latest.
        self.latest_state = None     # {joint_name: current_position} or None
        self.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)

        # Timer 1: publish the (fixed) command at CMD_RATE_HZ.
        self.create_timer(1.0 / CMD_RATE_HZ, self._publish_command)

        # Timer 2: print a readable commanded-vs-actual line at DISPLAY_RATE_HZ.
        self.create_timer(1.0 / DISPLAY_RATE_HZ, self._print_status)

        names = ", ".join(f"{k}={v:+.3f}" for k, v in SETPOINTS.items())
        self.get_logger().info(f"commanding fixed setpoint(s): {names}")
        self.get_logger().info(
            f"publishing {COMMAND_TOPIC} @ {CMD_RATE_HZ:g} Hz, "
            f"reading {STATE_TOPIC}, display @ {DISPLAY_RATE_HZ:g} Hz"
        )

    def _on_state(self, msg: JointState):
        # Build name -> position for THIS message. (velocity available too, but
        # this trivial node only displays position.)
        self.latest_state = dict(zip(msg.name, msg.position))

    def _publish_command(self):
        # Publish the FIXED setpoint. Note: this ignores latest_state entirely --
        # open-loop by design.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(SETPOINTS.keys())
        msg.position = [float(v) for v in SETPOINTS.values()]
        self.cmd_pub.publish(msg)

    def _print_status(self):
        if self.latest_state is None:
            # Haven't heard state yet -- edge may still be forming.
            print("  (waiting for /joint_states ...)", end="\r", flush=True)
            return
        # For each commanded joint, show cmd vs actual vs error on one line.
        parts = []
        for jname, target in SETPOINTS.items():
            actual = self.latest_state.get(jname)
            if actual is None:
                parts.append(f"{jname} (not in state)")
                continue
            short = jname.replace("panda_", "")
            err = target - actual
            parts.append(f"{short}  cmd {target:+.3f}  act {actual:+.3f}  err {err:+.3f}")
        # Carriage-return so the line refreshes in place instead of scrolling.
        print("   " + "   |   ".join(parts) + "        ", end="\r", flush=True)


def main():
    rclpy.init()
    node = TrivialController()
    print(f"[info] {NODE_NAME} running -- Ctrl+C to stop.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[info] stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
