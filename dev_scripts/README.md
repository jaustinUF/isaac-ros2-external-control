# dev_scripts/

Superseded single-purpose scripts, kept to show the build progression — each
port proven in isolation before assembly. The shipped artifacts are in the
repository root; see the top-level README.

- `probe_jointstate_import.py` — one-shot check that `sensor_msgs/JointState` is
  importable on the sim side under real launch conditions.
- `stage5c3_jointstate_publisher.py` — single port: joint state out. Merged into
  the bidirectional node.
- `stage5c3_command_subscriber.py` — single port: commands in. Merged into the
  bidirectional node.
- `stage5c3_trivial_controller.py` — open-loop predecessor of the closed-loop
  controller (fixed setpoint, ignores state).
