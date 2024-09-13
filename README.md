# Foundations of Robotics course project
## SCARA manipulator control and simulation
### University of Naples "Federico II" - Prof. Bruno Siciliano

# Project Description

The aim of the project is to showcase how to properly simulate and control a SCARA manipulator as requested in the Foundations of Robotics course project. In particular the assignement asks:

- To analyze velocity and force manipulability for the supporting structure, plotting the relative ellipsoids for a significant number of positions of the end-effector within the workspace.
- To plan the trajectory along a path charaterized by at least 20 points within the workspace in which there are at least one straight portion and one circular portion and also the passage for at least 3 via points.
- To implement CLIK algorithms with Jacobian inverse and transpose along the trajectory.
- Assuming to relax an operational space component, implement the CLIK algorithm with Jacobian pseudo-inverse
along the trajectory when optimizing a dexterity constraint.
- Derive the dynamic model by assuming that the angular velocity of each rotor is only due to its own spinning.
- Consider a concentrated end-effector payload of about 3 kg. Then, design: 1) a robust control; 2) an adaptive control; 3) an operational space inverse dynamics control with the adoption of an integral action to recover the steady-state error due to the uncompensated load. Simulate in MATLAB the motion of the controlled manipulator under the assumption that the desired joint trajectories for the first two controllers are generated with a 2nd-order CLIK algorithm. Implement discrete-time controllers with a sampling period of 1 ms.
