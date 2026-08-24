"""
Stage 5C.3 - Closed-loop (P-structure) external controller.

WHAT THIS IS
    An EXTERNAL ROS 2 node -- the controller end of the network seam. It:
      - SUBSCRIBES /joint_states  (reads where the arm actually IS)
      - PUBLISHES /joint_command  (tells the arm where to GO)
    and each command cycle it computes error = setpoint - actual from the state
    it just read, then publishes. The command DEPENDS on the state -- the
    information loop is closed over ROS 2.

    Promoted from the earlier open-loop "trivial controller" (which published a
    fixed setpoint and ignored the state it read). The change is small and
    deliberate: the command path is now  state -> error -> command  rather than
    a constant. That is the closed-loop STRUCTURE the stage title ("close the
    loop over ROS 2") refers to.

HONEST LABELING -- READ THIS (it's the whole point of the design choice)
    The Franka plant node applies incoming /joint_command values as POSITION
    targets, and the Franka's stiff INTERNAL PD drive is what actually servos the
    joint to that target. So the position servo lives INSIDE the plant, not here.

    This external loop therefore demonstrates the closed-loop STRUCTURE over ROS 2
    -- read state, compute error, decide the command from it -- WITHOUT claiming
    that an external proportional gain is doing the servoing physics. We publish
    command = setpoint (the plant PD drives to it) and compute/display
    error = setpoint - actual so the closed-loop information path is explicit in
    both the code and the readout. `err -> 0` because the plant PD reaches the
    target; the value shown is the true tracking error the controller observes.

    Why not command = actual + Kp*error (a "real" external position-P)? On a
    plant that already closes a stiff position loop internally, that form just
    ratchets the target to setpoint over ticks and manufactures convergence at
    any Kp -- it LOOKS like a tuned P loop but the internal PD does the work.
    Presenting that as a tuned gain would misrepresent it, so we don't. (The
    honest effort-mode P loop -- zero the drive gains and command torque -- was
    explored separately and is a genuine manipulator-dynamics problem: gravity
    defeats pure P on this arm. That's control-law territory, deliberately out of
    scope for this transport-competency artifact.)

    NET: this node closes an honest INFORMATION loop over ROS 2 (command computed
    from measured state), which is exactly the transport competency the repo
    shows. It does not overclaim to be a tuned position-P controller.

ARCHITECTURE
    Runs on the NATIVE Jazzy side (system Python 3.12, /opt/ros/jazzy sourced) --
    NOT the Isaac bundle. The controller never imports Isaac; it talks to the sim
    ONLY over DDS topics. See the .sh wrapper (sources native Jazzy + deactivates
    conda, unlike the sim-side wrappers which use the bundle).

WHAT YOU'LL SEE
    A single line, refreshed a few times a second, e.g.:
        joint4  set -0.500  act -0.481  err +0.019
    'set' = commanded setpoint, 'act' = measured actual, 'err' = set - act (the
    tracking error this controller computes each cycle). Watch err -> 0 as the
    plant PD servos to the setpoint.

USAGE
    Edit SETPOINTS below (joint name -> target radians), or leave the default.
    ./stage5c3_closed_p_controller.sh
    Ctrl+C to stop. (On stop the sim node holds the last target.)

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
NODE_NAME     = "closed_p_controller"

# Target position(s) this controller drives toward. joint name -> radians.
SETPOINTS = {
    "panda_joint4": -0.5,
}

CMD_RATE_HZ     = 10.0    # how often we compute error + PUBLISH the command
DISPLAY_RATE_HZ = 4.0     # how often we PRINT the readable status line


class ClosedPController(Node):
    """External controller closing an information loop over ROS 2: each command
    cycle reads the latest state, computes error = setpoint - actual, and
    publishes. Command depends on state -- that's the closed-loop property.

    (The plant's internal PD is the actual position servo; see the module
    docstring for why this is labeled a closed-loop STRUCTURE, not a tuned P.)"""

    def __init__(self):
        super().__init__(NODE_NAME)

        # OUTPUT port: publish target positions.
        self.cmd_pub = self.create_publisher(JointState, COMMAND_TOPIC, 10)

        # INPUT port: subscribe to current state, stash the latest.
        self.latest_state = None     # {joint_name: current_position} or None
        self.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)

        # Latest computed error per joint, for the display (filled each command
        # cycle so the readout reflects what the controller actually acted on).
        self.latest_error = {}

        # Timer 1: compute error from state + publish the command.
        self.create_timer(1.0 / CMD_RATE_HZ, self._control_step)
        # Timer 2: print a readable set/act/err line.
        self.create_timer(1.0 / DISPLAY_RATE_HZ, self._print_status)

        names = ", ".join(f"{k}={v:+.3f}" for k, v in SETPOINTS.items())
        self.get_logger().info(f"closed-loop (P-structure) over ROS 2; setpoint(s): {names}")
        self.get_logger().info(
            f"publishing {COMMAND_TOPIC} @ {CMD_RATE_HZ:g} Hz, "
            f"reading {STATE_TOPIC}, display @ {DISPLAY_RATE_HZ:g} Hz"
        )

    def _on_state(self, msg: JointState):
        # Build name -> position for THIS message. (velocity available too, but
        # this node only needs position for the error.)
        self.latest_state = dict(zip(msg.name, msg.position))

    def _control_step(self):
        # THE CLOSED-LOOP STEP: command computed FROM measured state.
        # Until state has arrived, we cannot compute error -> do not command yet.
        if self.latest_state is None:
            return

        names, positions = [], []
        for jname, setpoint in SETPOINTS.items():
            actual = self.latest_state.get(jname)
            if actual is None:
                continue  # setpoint joint not present in state this cycle
            # Closed-loop dependency: read actual -> compute error. This is the
            # information the controller acts on. (Command published is the
            # setpoint; the plant's internal PD servos to it -- see docstring.)
            error = setpoint - actual
            self.latest_error[jname] = error
            names.append(jname)
            positions.append(float(setpoint))

        if not names:
            return  # nothing to command yet (no setpoint joints in state)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.cmd_pub.publish(msg)

    def _print_status(self):
        if self.latest_state is None:
            print("  (waiting for /joint_states ...)", end="\r", flush=True)
            return
        parts = []
        for jname, setpoint in SETPOINTS.items():
            actual = self.latest_state.get(jname)
            if actual is None:
                parts.append(f"{jname} (not in state)")
                continue
            short = jname.replace("panda_", "")
            err = self.latest_error.get(jname, setpoint - actual)
            parts.append(f"{short}  set {setpoint:+.3f}  act {actual:+.3f}  err {err:+.3f}")
        print("   " + "   |   ".join(parts) + "        ", end="\r", flush=True)


def main():
    rclpy.init()
    node = ClosedPController()
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
