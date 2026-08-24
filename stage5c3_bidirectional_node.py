"""
Stage 5C.3 - Bidirectional sim node : the plant side of the loop, both ports.

WHAT THIS IS
    The MERGE of the two proven 5C.3 halves into ONE sim-side ROS 2 node that
    wears both hats at once:
      - OUTPUT port: publishes sensor_msgs/JointState on /joint_states every
        step (all 9 DOFs, wall-clock stamp) -- from the publisher script.
      - INPUT port: subscribes sensor_msgs/JointState on /joint_command, maps
        commanded positions to DOFs by name, applies them as position targets
        -- from the subscriber script.
    One node, one process, one spin_once servicing both. This is the "sim end of
    the loop has a publisher AND a subscriber" node -- the real plant-side node
    a controller will talk to.

    Each proven-in-isolation port is carried over verbatim in behavior; only the
    assembly is new. The one deliberate CHANGE from the publisher: NO internal
    sinusoid. The arm sits at its default pose until commanded, then holds
    whatever it is told. The command source is about to become the point, so the
    plant should be passive -- it moves only when driven from outside.

LOOP ORDER (each step):
    1. spin_once            -- service ROS callbacks (may refresh latest command)
    2. apply latest command -- overwrite addressed joints' targets (by name)
    3. WRITE targets        -- set_joint_position_targets
    4. world.step           -- advance physics one tick
    5. READ all 9 DOFs      -- positions + velocities
    6. PUBLISH JointState   -- current state out on /joint_states
    Read-AFTER-step so the published state reflects the tick just taken.

DECISIONS CARRIED FROM THE TWO HALVES (unchanged):
    - JointState on both edges; map inbound commands by name[] (any subset, any
      order); joints not addressed hold their current target.
    - Publish all 9 DOFs; wall-clock header.stamp (sim-time slaving deferred to
      the PID stage where dt matters).
    - Default RELIABLE QoS on both the publisher and the subscription -- proven
      compatible with the stock CLI on both edges this session.
    - Publish once per step -> execution-bound rate (~200 Hz on this machine,
      NOT PHYSICS_DT-locked -- confirmed by the headless-vs-windowed probe).

ARCHITECTURE
    Sim side (bundled rclpy, native Jazzy NOT sourced) -- see the .sh wrapper.
    Single-threaded: callback and loop never run concurrently (spin_once), so
    the "latest command" buffer needs no lock.

RUN
    ./stage5c3_bidirectional_node.sh             # sim side (bundled rclpy)

    # Terminal 2 (native `rosjazzy`): watch state going OUT --
    ros2 topic echo /joint_states
    # Terminal 3 (native `rosjazzy`): send commands IN --
    ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
      '{name: ["panda_joint4"], position: [-0.5]}'
    # Both at once proves the full plant-side node: command in moves the arm,
    # and the resulting motion shows up on /joint_states going out.
"""

# ----------------------------------------------------------------------------
# 1. LAUNCH THE APP FIRST. headless=False to watch the arm respond.
# ----------------------------------------------------------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ----------------------------------------------------------------------------
# 1a. ENABLE THE ROS 2 BRIDGE, then one app.update() -- probe-confirmed ordering.
# ----------------------------------------------------------------------------
from isaacsim.core.utils.extensions import enable_extension

_ok = enable_extension("isaacsim.ros2.bridge")
print(f"[info] enable_extension('isaacsim.ros2.bridge') -> {_ok}")
simulation_app.update()

# ----------------------------------------------------------------------------
# 1b. POINT ISAAC SIM AT THE LOCAL ASSET PACK.
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

STATE_TOPIC   = "/joint_states"    # OUTPUT: sim state -> world
COMMAND_TOPIC = "/joint_command"   # INPUT:  world -> sim targets
NODE_NAME     = "isaac_sim_plant"

FRANKA_PRIM_PATH = "/World/Franka"
FRANKA_USD = os.path.join(
    LOCAL_ASSET_ROOT, "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)


class SimPlantNode(Node):
    """The sim-side plant node: BOTH ports on one node.
      - publisher  on /joint_states  (output)
      - subscription on /joint_command (input) -> latest_cmd buffer
    The sim loop publishes state each step and reads latest_cmd each step.
    Single-threaded via spin_once, so latest_cmd needs no lock."""

    def __init__(self):
        super().__init__(NODE_NAME)
        # OUTPUT port.
        self.state_pub = self.create_publisher(JointState, STATE_TOPIC, 10)
        # INPUT port.
        self.latest_cmd = None          # {joint_name: target_pos} or None
        self._n_received = 0
        self.create_subscription(JointState, COMMAND_TOPIC, self._on_command, 10)
        self.get_logger().info(
            f"plant node up: publishing {STATE_TOPIC}, subscribing {COMMAND_TOPIC}"
        )

    def _on_command(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn(
                f"ignoring command: {len(msg.name)} names vs "
                f"{len(msg.position)} positions"
            )
            return
        self.latest_cmd = dict(zip(msg.name, msg.position))
        self._n_received += 1
        if self._n_received == 1:
            self.get_logger().info(f"FIRST command received: {self.latest_cmd}")

    def publish_state(self, dof_names, pos, vel):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()   # WALL-CLOCK stamp
        msg.name = dof_names
        msg.position = [float(x) for x in pos]
        msg.velocity = [float(x) for x in vel]
        # effort left empty (position control); add via get_measured_joint_efforts
        # -- NOT get_applied_ (reads 0) -- if a later stage wants it.
        self.state_pub.publish(msg)


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
    # 3a. ROS 2 NODE (both ports).
    # ------------------------------------------------------------------
    rclpy.init()
    node = SimPlantNode()

    world.reset()  # known starting pose

    # ------------------------------------------------------------------
    # DOF bookkeeping: ordered names for publishing, name->index for commands.
    # ------------------------------------------------------------------
    dof_names = list(franka.dof_names)
    name_to_idx = {n: i for i, n in enumerate(dof_names)}
    print(f"[info] {len(dof_names)} DOFs: {dof_names}")

    # Start from (and hold) the default pose until commanded.
    default_pos = np.array(franka.get_joint_positions(), dtype=np.float32).reshape(-1)
    target_pos = default_pos.copy()

    unknown_warned = set()

    print("[info] plant passive -- holding default pose until commanded.")
    print("[info] close the window (or Ctrl+C) to stop.")

    # ------------------------------------------------------------------
    # 4. THE BIDIRECTIONAL LOOP.
    # ------------------------------------------------------------------
    while simulation_app.is_running():
        # 1. SERVICE ROS callbacks (may refresh node.latest_cmd).
        rclpy.spin_once(node, timeout_sec=0.0)

        # 2. APPLY latest command (by name) onto the target vector.
        if node.latest_cmd is not None:
            for jname, jpos in node.latest_cmd.items():
                idx = name_to_idx.get(jname)
                if idx is None:
                    if jname not in unknown_warned:
                        print(f"[warn] command names unknown joint '{jname}' -- ignoring")
                        unknown_warned.add(jname)
                    continue
                target_pos[idx] = float(jpos)

        # 3. WRITE targets.
        franka.set_joint_position_targets(target_pos.reshape(1, -1))

        # 4. STEP the sim one physics tick.
        world.step(render=True)

        # 5. READ all 9 DOFs AFTER the step (state reflects the tick just taken).
        pos = np.asarray(franka.get_joint_positions()).reshape(-1)
        vel = np.asarray(franka.get_joint_velocities()).reshape(-1)

        # 6. PUBLISH state out.
        node.publish_state(dof_names, pos, vel)

    # ------------------------------------------------------------------
    # 5. CLEAN SHUTDOWN.
    # ------------------------------------------------------------------
    print("[info] shutting down ROS 2 node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
