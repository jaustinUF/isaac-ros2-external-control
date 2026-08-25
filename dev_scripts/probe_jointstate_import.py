"""
5C.3 probe : confirm sensor_msgs/JointState is importable on the SIM side,
under the true launch conditions (bundle rclpy, after SimulationApp + bridge
extension). This settles the one genuine unknown before any publisher is written.

WHY A SimulationApp WRAPPER AND NOT A BARE `python -c`:
    A bare import can false-negative -- outside a running SimulationApp the
    bundle's rclpy / message packages may not be on sys.path yet. This mirrors
    the exact ordering the working String toy used:
        SimulationApp -> enable bridge extension -> app.update() -> import rclpy
    so a success here means "importable under real conditions", full stop.
"""

# 1. LAUNCH THE APP FIRST (headless -- we render nothing, just test imports).
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# 2. Enable the ROS 2 bridge extension -- this is what wires the bundle's
#    rclpy + message packages onto sys.path (grep-confirmed exported name).
from isaacsim.core.utils.extensions import enable_extension
ok = enable_extension("isaacsim.ros2.bridge")
print(f"[probe] enable_extension('isaacsim.ros2.bridge') -> {ok}")

# 3. Pump one app update so the extension finishes loading.
simulation_app.update()

# 4. THE ACTUAL TEST: can we import rclpy and the JointState message type?
try:
    import rclpy
    from sensor_msgs.msg import JointState
    print("[probe] OK   import rclpy            ->", rclpy.__file__)
    print("[probe] OK   from sensor_msgs.msg import JointState ->", JointState)
    # Instantiate one, to prove the type-support .so actually resolved:
    msg = JointState()
    msg.name = ["a", "b"]
    msg.position = [0.1, 0.2]
    print("[probe] OK   JointState() instantiated:", list(msg.name), list(msg.position))
    print("[probe] RESULT: PASS -- sensor_msgs/JointState usable on sim side.")
except Exception as e:
    print("[probe] RESULT: FAIL --", type(e).__name__, e)

# 5. Clean shutdown.
simulation_app.close()
