"""
Stage 5C.3 - JointState publisher : first REAL robot signal on the ROS 2 wire.

WHAT THIS IS
    5C.2 proved the transport carries data (a std_msgs/String toy round-tripped
    across the two-process / one-DDS-graph seam). 5C.3 puts a REAL robot signal
    on that same transport: publish sensor_msgs/JointState on /joint_states from
    inside a standalone Isaac Sim script, so an external `rosjazzy` terminal can
    `ros2 topic echo /joint_states` and see live joint data.

    This is PUBLISH-SIDE ONLY (sim -> ROS). The inbound joint-command topic and
    the external PID controller come AFTER this round-trips clean -- one change
    at a time.

LINEAGE -- this is stage5a_v4_sinusoid.py with three changes:
    1. ROS 2 setup grafted into the launch order (bridge extension -> rclpy ->
       publisher), at the points the 5C.3 import probe confirmed are correct.
    2. The loop's "log to CSV / capture frame" slot is replaced by a
       "publish JointState" slot. We now read ALL 9 DOFs (not just the three
       commanded), because /joint_states conventionally carries full robot state.
    3. Viewport capture stripped entirely (nothing to look at yet), and the
       fixed-n_steps-then-keep-alive split collapsed into ONE continuous loop:
       the arm swings on the sinusoid forever (until you close the window),
       so /joint_states streams live the whole time.

DESIGN DECISIONS (settled in discussion, recorded here):
    - Publish ALL 9 Franka DOFs -- canonical /joint_states shape; costs nothing.
    - WALL-CLOCK header.stamp (node clock), NOT sim time. sim-time slaving
      (use_sim_time + /clock) is deferred to the PID stage where dt actually
      matters; keeping it out here holds 5C.3 to one new thing.
    - QoS: default RELIABLE profile for /joint_states. It's the conventional
      profile for joint state, it's what a controller's subscriber defaults to
      (so it sidesteps the QoS RxO mismatch trap later), and `ros2 topic echo`
      reads it fine.
    - Publish once per physics step -> ~60 Hz stream (PHYSICS_DT = 1/60).
      Realistic joint-state rate; no extra rate machinery needed.

RUN
    ./stage5c3_jointstate_publisher.sh          # sim side (bundled rclpy)
    # then, in a SEPARATE `rosjazzy` terminal (native Jazzy, system Python):
    ros2 topic list                              # expect /joint_states
    ros2 topic echo /joint_states                # expect live streaming data
    ros2 topic hz /joint_states                  # expect ~60 Hz
"""

# ----------------------------------------------------------------------------
# 1. LAUNCH THE APP FIRST.  (headless=False so you can see the arm swing; flip
#    to True if you only care about the topic.)
# ----------------------------------------------------------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
# simulation_app = SimulationApp({"headless": True})

# ----------------------------------------------------------------------------
# 1a. ENABLE THE ROS 2 BRIDGE, then pump one app.update().  This ordering is
#     what the 5C.3 import probe confirmed: the bridge extension is what wires
#     the bundle's rclpy + message packages onto sys.path, and the update()
#     lets the extension finish loading before we import rclpy. Doing the ROS
#     imports before this point would fail exactly the way the probe guarded
#     against.
# ----------------------------------------------------------------------------
from isaacsim.core.utils.extensions import enable_extension

_ok = enable_extension("isaacsim.ros2.bridge")
print(f"[info] enable_extension('isaacsim.ros2.bridge') -> {_ok}")
simulation_app.update()

# ----------------------------------------------------------------------------
# 1b. POINT ISAAC SIM AT THE LOCAL ASSET PACK (before any asset is requested).
#     Same carb setting as v4 -- makes all asset resolution local (no S3 hang).
# ----------------------------------------------------------------------------
import os
import carb.settings

LOCAL_ASSET_ROOT = os.path.expanduser("~/isaacsim_assets/Assets/Isaac/5.1")

_settings = carb.settings.get_settings()
_settings.set("/persistent/isaac/asset_root/default", LOCAL_ASSET_ROOT)
print(f"[info] asset_root set to local: {LOCAL_ASSET_ROOT}")

# ----------------------------------------------------------------------------
# 2. NOW it is safe to import the sim/scene APIs AND rclpy + the message type.
#    (rclpy resolves to the bridge's BUNDLED 3.11 rclpy -- confirmed by the
#    probe: .../isaacsim.ros2.bridge/jazzy/rclpy/rclpy/__init__.py)
# ----------------------------------------------------------------------------
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ----------------------------------------------------------------------------
# CONFIG -- same sinusoid drive as v4 so the arm is visibly MOVING, which makes
# /joint_states show live changing data rather than a static pose.
# ----------------------------------------------------------------------------
# Joints we DRIVE (three), by name. NOTE: we PUBLISH all 9 DOFs regardless;
# these three are just the ones the sinusoid moves so the topic looks alive.
JOINT_NAMES = ["panda_joint2", "panda_joint4", "panda_joint6"]

# Sine parameters per driven joint: (center, amplitude) in radians. Chosen to
# keep the arm in free space through the whole swing.
#                    center  amp
SINE_PARAMS = {
    "panda_joint2": (0.3,   0.4),
    "panda_joint4": (-1.2,  0.4),
    "panda_joint6": (1.0,   0.5),
}
FREQ_HZ    = 0.5              # gentle swing; slow enough to read in `echo`
PHYSICS_DT = 1.0 / 60.0      # -> ~60 Hz publish rate

# ROS 2
TOPIC_NAME = "/joint_states"
NODE_NAME  = "isaac_joint_state_publisher"

FRANKA_PRIM_PATH = "/World/Franka"
# Real 5.1 local path. Note: FrankaRobotics/FrankaPanda, NOT old Robots/Franka.
FRANKA_USD = os.path.join(
    LOCAL_ASSET_ROOT, "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)


def main():
    # ------------------------------------------------------------------
    # FAST-FAIL GUARD: confirm the Franka USD is on disk (clear error in ~1s
    # instead of a silent hang / empty stage).
    # ------------------------------------------------------------------
    if not os.path.isfile(FRANKA_USD):
        raise FileNotFoundError(
            f"Franka USD not found at:\n  {FRANKA_USD}\n"
            f"Check LOCAL_ASSET_ROOT and the path under it."
        )
    print(f"[info] Franka USD found: {FRANKA_USD}")

    # ------------------------------------------------------------------
    # 3. BUILD THE WORLD (raw Isaac Sim API) -- identical to v4.
    # ------------------------------------------------------------------
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=FRANKA_USD, prim_path=FRANKA_PRIM_PATH)
    franka = Articulation(prim_paths_expr=FRANKA_PRIM_PATH, name="franka")
    world.scene.add(franka)
    world.reset()

    # ------------------------------------------------------------------
    # 3a. ROS 2 NODE + PUBLISHER.  rclpy.init() then a Node with one publisher
    #     on /joint_states. Default QoS (RELIABLE) -- the conventional joint-
    #     state profile and what a controller subscriber will default to.
    # ------------------------------------------------------------------
    rclpy.init()
    node = Node(NODE_NAME)
    publisher = node.create_publisher(JointState, TOPIC_NAME, 10)
    print(f"[info] publishing sensor_msgs/JointState on {TOPIC_NAME} as '{NODE_NAME}'")

    # ------------------------------------------------------------------
    # Settle physics to a known pose (v4 did a 30-step warmup then reset(); we
    # keep the reset for a repeatable start, but drop the render-warmup since
    # there's no capture to keep un-blank).
    # ------------------------------------------------------------------
    world.reset()

    # ------------------------------------------------------------------
    # Resolve joint NAMES -> INDICES for the DRIVEN joints, and grab the FULL
    # ordered DOF name list for the published message.
    # ------------------------------------------------------------------
    dof_names = list(franka.dof_names)              # all 9, in articulation order
    driven_idx = [dof_names.index(n) for n in JOINT_NAMES]
    print(f"[info] driving {JOINT_NAMES} -> dof indices {driven_idx}")
    print(f"[info] publishing all {len(dof_names)} DOFs: {dof_names}")

    # Known default pose, read AFTER reset. Untouched joints hold these.
    default_pos = np.array(franka.get_joint_positions(), dtype=np.float32).copy()

    centers = np.array([SINE_PARAMS[n][0] for n in JOINT_NAMES], dtype=np.float32)
    amps    = np.array([SINE_PARAMS[n][1] for n in JOINT_NAMES], dtype=np.float32)

    # Pre-position at the sine's t=0 value (centers, since sin(0)=0) so we don't
    # start with a step jump from the default pose into the sine.
    start_tgt = default_pos.copy()
    start_tgt[0, driven_idx] = centers
    for _ in range(int(1.0 / PHYSICS_DT)):   # ~1 s to settle onto the centers
        franka.set_joint_position_targets(start_tgt)
        world.step(render=True)

    # ------------------------------------------------------------------
    # 4. THE CONTINUOUS LOOP.  Each step:
    #      compute sine -> WRITE targets -> step sim -> READ all 9 DOFs ->
    #      PUBLISH JointState -> service the ROS executor.
    #    Runs until you close the window / Ctrl+C. No fixed step count: the
    #    whole point is a live, continuously-streaming topic.
    # ------------------------------------------------------------------
    sim_time = 0.0
    target_pos = default_pos.copy()
    two_pi_f = 2.0 * np.pi * FREQ_HZ

    print("[info] streaming -- close the window (or Ctrl+C) to stop.")
    while simulation_app.is_running():
        # COMPUTE: cmd_j = center_j + amp_j * sin(2*pi*f*t) for driven joints.
        phase = two_pi_f * sim_time
        cmd = centers + amps * np.sin(phase)
        target_pos[0, driven_idx] = cmd

        # WRITE the sine targets into the PD drives.
        franka.set_joint_position_targets(target_pos)

        # STEP the sim one physics tick.
        world.step(render=True)

        # READ all 9 DOFs. Articulation getters can return (1, N) -- reshape(-1)
        # to a flat (9,) vector (the shape trap noted in 5B).
        pos = np.asarray(franka.get_joint_positions()).reshape(-1)
        vel = np.asarray(franka.get_joint_velocities()).reshape(-1)

        # BUILD the JointState message.
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()   # WALL-CLOCK stamp
        msg.name = dof_names                                  # all 9, in order
        msg.position = [float(x) for x in pos]
        msg.velocity = [float(x) for x in vel]
        # effort left empty for 5C.3 (position control; add later if wanted --
        # get_measured_joint_efforts(), NOT get_applied_ which reads 0).

        # PUBLISH, then service the executor once (non-blocking).
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

        sim_time += PHYSICS_DT

    # ------------------------------------------------------------------
    # 5. CLEAN SHUTDOWN of the ROS side when the sim window closes.
    # ------------------------------------------------------------------
    print("[info] shutting down ROS 2 node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
