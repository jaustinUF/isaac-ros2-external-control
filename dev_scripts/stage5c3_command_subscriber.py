"""
Stage 5C.3 - JointState command SUBSCRIBER : first EXTERNAL signal driving sim.

WHAT THIS IS
    The mirror of the publisher. 5C.3-publish put sim state OUT on /joint_states.
    This puts external commands IN: the sim SUBSCRIBES to /joint_command
    (sensor_msgs/JointState), maps the commanded positions to DOF indices by
    joint name, and applies them as position targets. The arm moves ONLY in
    response to inbound ROS traffic -- there is no internal sinusoid here.

    This is the sim node's INPUT PORT, built in isolation. It is deliberately
    subscribe-only: one wire of the control-loop block diagram at a time. The
    publisher (output port) and this subscriber (input port) get MERGED into one
    bidirectional sim node in the NEXT step -- not here.

    The command SOURCE for this step is the stock CLI (`ros2 topic pub ...`) from
    a native `rosjazzy` terminal, hand-standing-in for the eventual PID
    controller. Nothing you author runs on the command-source side yet.

DESIGN DECISIONS (settled in discussion):
    - Message type: sensor_msgs/JointState on BOTH edges (symmetry with the
      publisher; self-describing via name[], so joint ordering is explicit, not
      positional-by-convention). Matches the topic_based_ros2_control pattern.
    - Map by NAME, not by index. The inbound msg.name[] tells us which joints
      msg.position[] refers to; we look each up in the articulation's dof order.
      This means a command may address ANY subset of joints, in ANY order, and
      still land on the right DOFs -- robust to the command source's ordering.
    - Unnamed / partial commands: joints NOT mentioned in the command hold their
      current target (we start from the default pose and only overwrite the
      addressed joints).
    - QoS: default RELIABLE on the subscription -- must be compatible with the
      command publisher. `ros2 topic pub` defaults to RELIABLE too, so they
      match. (If a future publisher offers BEST_EFFORT, the edge silently never
      forms -- check `ros2 topic info /joint_command --verbose`.)

ARCHITECTURE
    Runs on the SIM side (bundled rclpy, native Jazzy NOT sourced) -- same launch
    environment as the publisher, opposite data direction. See the .sh wrapper.

    The rclpy callback and the sim step share ONE thread: the subscription fills
    a "latest command" buffer in its callback; the main loop reads that buffer
    each step and applies it. spin_once(timeout_sec=0) services callbacks
    without blocking the sim. (This is the standard single-threaded pattern --
    no locks needed because callback and loop never run concurrently.)

RUN
    ./stage5c3_command_subscriber.sh             # sim side (bundled rclpy)
    # then, in a SEPARATE `rosjazzy` terminal (native Jazzy), send a command.
    # ONE-SHOT (move joint4 to -0.5 rad):
    ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
      '{name: ["panda_joint4"], position: [-0.5]}'
    # STREAM (hold joint2 at 0.8 rad at 10 Hz):
    ros2 topic pub -r 10 /joint_command sensor_msgs/msg/JointState \
      '{name: ["panda_joint2"], position: [0.8]}'
    # MULTI-JOINT one-shot:
    ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
      '{name: ["panda_joint2","panda_joint4","panda_joint6"], position: [0.5,-1.0,1.2]}'
"""

# ----------------------------------------------------------------------------
# 1. LAUNCH THE APP FIRST.  (headless=False so you can WATCH the arm respond to
#    external commands -- the visual feedback is the point on the subscribe side.)
# ----------------------------------------------------------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ----------------------------------------------------------------------------
# 1a. ENABLE THE ROS 2 BRIDGE, then pump one app.update() -- same probe-confirmed
#     ordering as the publisher: extension first (wires bundle rclpy onto
#     sys.path), update() lets it finish loading, THEN import rclpy.
# ----------------------------------------------------------------------------
from isaacsim.core.utils.extensions import enable_extension

_ok = enable_extension("isaacsim.ros2.bridge")
print(f"[info] enable_extension('isaacsim.ros2.bridge') -> {_ok}")
simulation_app.update()

# ----------------------------------------------------------------------------
# 1b. POINT ISAAC SIM AT THE LOCAL ASSET PACK (before any asset is requested).
# ----------------------------------------------------------------------------
import os
import carb.settings

LOCAL_ASSET_ROOT = os.path.expanduser("~/isaacsim_assets/Assets/Isaac/5.1")

_settings = carb.settings.get_settings()
_settings.set("/persistent/isaac/asset_root/default", LOCAL_ASSET_ROOT)
print(f"[info] asset_root set to local: {LOCAL_ASSET_ROOT}")

# ----------------------------------------------------------------------------
# 2. NOW import sim/scene APIs AND rclpy + the message type (bundle rclpy).
# ----------------------------------------------------------------------------
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
PHYSICS_DT = 1.0 / 60.0

TOPIC_NAME = "/joint_command"
NODE_NAME  = "isaac_command_subscriber"

FRANKA_PRIM_PATH = "/World/Franka"
FRANKA_USD = os.path.join(
    LOCAL_ASSET_ROOT, "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)


class CommandSubscriberNode(Node):
    """A ROS 2 node with ONE input port: subscribes /joint_command and stashes
    the latest command as a {joint_name: target_position} dict. The sim loop
    reads that dict each step. Callback and loop share one thread (spin_once),
    so no locking is needed."""

    def __init__(self):
        super().__init__(NODE_NAME)
        # Latest command buffer: joint name -> target position (radians).
        # None until the first command arrives.
        self.latest_cmd = None
        self._n_received = 0
        self.create_subscription(JointState, TOPIC_NAME, self._on_command, 10)
        self.get_logger().info(f"subscribed to {TOPIC_NAME}")

    def _on_command(self, msg: JointState):
        # Map name[] -> position[] into a dict. Guard against a command whose
        # name/position lengths disagree (malformed).
        if len(msg.name) != len(msg.position):
            self.get_logger().warn(
                f"ignoring command: {len(msg.name)} names vs "
                f"{len(msg.position)} positions"
            )
            return
        self.latest_cmd = dict(zip(msg.name, msg.position))
        self._n_received += 1
        # Log the first command loudly (proves the edge formed), then stay quiet.
        if self._n_received == 1:
            self.get_logger().info(f"FIRST command received: {self.latest_cmd}")


def main():
    # ------------------------------------------------------------------
    # FAST-FAIL GUARD.
    # ------------------------------------------------------------------
    if not os.path.isfile(FRANKA_USD):
        raise FileNotFoundError(
            f"Franka USD not found at:\n  {FRANKA_USD}\n"
            f"Check LOCAL_ASSET_ROOT and the path under it."
        )
    print(f"[info] Franka USD found: {FRANKA_USD}")

    # ------------------------------------------------------------------
    # 3. BUILD THE WORLD.
    # ------------------------------------------------------------------
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=FRANKA_USD, prim_path=FRANKA_PRIM_PATH)
    franka = Articulation(prim_paths_expr=FRANKA_PRIM_PATH, name="franka")
    world.scene.add(franka)
    world.reset()

    # ------------------------------------------------------------------
    # 3a. ROS 2 NODE (input port).
    # ------------------------------------------------------------------
    rclpy.init()
    node = CommandSubscriberNode()
    print(f"[info] listening for sensor_msgs/JointState on {TOPIC_NAME}")

    world.reset()  # known starting pose

    # ------------------------------------------------------------------
    # Full ordered DOF names, and a name->index map for applying commands.
    # ------------------------------------------------------------------
    dof_names = list(franka.dof_names)
    name_to_idx = {n: i for i, n in enumerate(dof_names)}
    print(f"[info] {len(dof_names)} DOFs: {dof_names}")

    # Start from the known default pose. We hold this until a command arrives,
    # and only overwrite the joints a command actually addresses.
    default_pos = np.array(franka.get_joint_positions(), dtype=np.float32).reshape(-1)
    target_pos = default_pos.copy()

    # ------------------------------------------------------------------
    # 4. THE LOOP.  Each step:
    #      service ROS callbacks -> apply latest command -> WRITE targets ->
    #      step sim.
    #    The arm holds default until a /joint_command arrives; then it moves the
    #    addressed joints to their commanded positions and holds.
    # ------------------------------------------------------------------
    print("[info] waiting for commands -- send one from a rosjazzy terminal.")
    print("[info] close the window (or Ctrl+C) to stop.")

    unknown_warned = set()   # warn once per unknown joint name

    while simulation_app.is_running():
        # SERVICE ROS callbacks (non-blocking). Fills node.latest_cmd if a
        # command arrived since last step.
        rclpy.spin_once(node, timeout_sec=0.0)

        # APPLY the latest command, if any, mapping by name.
        if node.latest_cmd is not None:
            for jname, jpos in node.latest_cmd.items():
                idx = name_to_idx.get(jname)
                if idx is None:
                    if jname not in unknown_warned:
                        print(f"[warn] command names unknown joint '{jname}' -- ignoring")
                        unknown_warned.add(jname)
                    continue
                target_pos[idx] = float(jpos)

        # WRITE targets (shape (1, N) for the articulation setter).
        franka.set_joint_position_targets(target_pos.reshape(1, -1))

        # STEP the sim.
        world.step(render=True)

    # ------------------------------------------------------------------
    # 5. CLEAN SHUTDOWN.
    # ------------------------------------------------------------------
    print("[info] shutting down ROS 2 node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
