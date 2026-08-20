# Physical Robot Dimensions

Operator measurement recorded on 2026-08-14:

- Maximum plan-view operating envelope: approximately **0.42 m x 0.42 m**.
- The widest points are the outside edges of the wheels.
- No wires, sensors, brackets, or other mounted hardware protrude beyond this box.

Treat this as the measured physical envelope of the current real-rover setup.
Navigation and collision-monitor footprints should include an appropriate safety
margin rather than using the 0.42 m box with zero clearance. Recheck the envelope
whenever wheels, wiring, sensors, or payload hardware are changed.

This measurement does not describe the existing Gazebo collision geometry, which
is defined independently by the URDF collision meshes and primitives.
