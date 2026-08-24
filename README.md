# isaac-ros2-external-control

External control of a simulated Franka Panda arm over ROS 2 — a two-process
Isaac Sim ↔ native ROS 2 Jazzy architecture that closes a joint-space loop
across the DDS seam.

## What this shows

An external controller process, running in its own environment (native ROS 2
Jazzy, system Python), closes a control loop on a Franka arm simulated in NVIDIA
Isaac Sim. The two processes never share a Python interpreter — they communicate
only over ROS 2 topics carried by DDS. That seam, where an outside process drives
and reads a simulated robot over the standard robotics middleware, is the
competency this repo demonstrates. It's the same boundary that exists on real
hardware, with the physical robot replaced by simulation.

The loop uses `sensor_msgs/JointState` on both edges: the sim publishes joint
state out on `/joint_states`, and subscribes to joint commands in on
`/joint_command`, with commands mapped to joints by name (mirroring the
`topic_based_ros2_control` pattern). The external controller reads state,
computes error against a setpoint, and publishes its command from that error —
a closed information loop over ROS 2.

## The two shipped artifacts

- **`stage5c3_bidirectional_node.py`** — the sim-side plant node. One node with
  both ports: publishes `/joint_states` every physics step and subscribes
  `/joint_command`, applying commands as position targets. The arm is passive
  (holds its pose) until commanded.
- **`stage5c3_closed_p_controller.py`** — the external controller node. Reads
  `/joint_states`, computes `error = setpoint − actual`, and publishes to
  `/joint_command`. The command depends on the measured state — the loop is
  closed.

Each script has a matching `.sh` wrapper in `wrapper_scripts/`. **The wrappers
are the point, not boilerplate** — see below.

## Why the `.sh` / `.py` split matters (the architecture, made concrete)

Isaac Sim 5.1 runs Python 3.11 and ships its own bundled ROS 2 rclpy. Native ROS
2 Jazzy on Ubuntu 24.04 runs Python 3.12. A compiled rclpy C-extension is locked
to one exact Python version, so the two sides *cannot* share an rclpy in one
process — which is exactly why the architecture is two processes joined only by
DDS. The wrappers encode that split:

- **Sim side** activates the Isaac conda environment and puts the bridge's
  *bundled* rclpy on the library path; native Jazzy is deliberately not sourced.
- **Controller side** does the opposite: it sources native Jazzy (system Python
  3.12) and fully deactivates conda so nothing shadows the system interpreter.

Two environments that never meet in-process, discovering each other over DDS.
The wrapper contrast *is* the two-process/one-DDS-graph design; that's why they're
included rather than hidden.

## On the control law — an honest scope note

The controller here closes the loop *structurally* — command computed from
measured state over ROS 2 — but it is not a tuned PID. The published command is a
position setpoint; the Franka's stiff internal PD drive does the actual
servoing. A genuine external control law (effort-mode P → I → D on a single
gravity-loaded joint) was explored and then deliberately set aside, because it
turned into a manipulator-dynamics problem rather than a transport one: pure
proportional control cannot hold a gravity-loaded joint (at zero error it
produces zero torque, but a loaded joint needs nonzero holding torque to stay
put), so the honest P loop drove the joint into its mechanical limit. That's the
textbook reason the integral term — or a gravity feedforward term — exists, and
it's control-law territory well-covered ground for a controls engineer. This
repo's purpose is the *sim-transport seam*, not PID tuning, so the loop was
closed cleanly at the transport level and the control-law exploration was
recorded separately rather than forced to completion. Choosing scope
deliberately, and documenting the boundary, is part of the work.

## `dev_scripts/`

The build progression, kept to show how each piece was proven in isolation before
assembly: an import probe, the single-port publisher and subscriber (later merged
into the bidirectional node), and the open-loop controller (later promoted to the
closed-loop version). Not the shipped artifacts — the record of the incremental,
one-change-at-a-time path to them.

## Context

Part of a self-directed robotics simulation & control track: build a robot in
sim, instrument it, and close a control loop around it over ROS 2 — the same
discipline as instrumenting and controlling a physical plant. Prior work in the
series characterized the arm's joint tracking (`franka-joint-tracking`) and its
full sensor suite (`franka-sensor-characterization`).
