# Parallel Linked Quadruped Robot Forward Kinematics

This repository contains MATLAB code and Simulink models for simulating and visualizing the kinematics of a parallel-linked quadruped robot. The repository includes the following components:

## MATLAB Code

- **`quadruped_forward_kinematics.m`**: MATLAB script that performs the following:
  - **Parameter Setup**: Defines constants and joint angles, and converts joint angles from degrees to radians.
  - **Transformation Matrices**: Computes the transformation matrices for each segment of the robot and calculates the overall transformation for the right and left sides of the robot.
  - **Visualization**: Computes points based on transformation matrices and plots the robot's configuration with different colors for each segment.

### MATLAB Parameters

The parameters used in the MATLAB script are defined as follows:

- **`e`**: Offset distance
- **`L1`, `L2`, `L3`, `L4`**: Link lengths
- **`E`, `F`**: Additional parameters
- **`theta1_deg`, `theta2_deg`**: Active-Joint angles in degrees

![Parameter Diagram](https://github.com/Varshith-Poojary/ParallelQuadRobotKinematics_And_Simulationulation/blob/main/parameter.png)

## Simulink Models

- **`forward_kinematics.slx`**: Simulink model that performs the following:
   - Simulates the leg motion of the quadruped robot using sine wave inputs.
   - Visualizes the up and down movement of each leg.
- **`robot_model.slx`**: Simulink model that contains the structure or body of the quadruped robot, constructed using Simulink.

## How to Use

1. **Open MATLAB and Simulink**:
   - Ensure you have MATLAB and Simulink installed on your machine.

2. **Run the MATLAB Script**:
   - Open the `simulink_quadruped_forward_kinematics` folder in MATLAB.
   - Run the `model_importer.m` script.

3. **Open and Run Simulink Models**:
   - Open the Simulink models `forward_kinematics.slx`.
   - Run the simulations and visualize the results in scope as needed.

## Example Output

- The MATLAB script generates a 3D plot showing the robot's kinematic configuration with various color-coded segments.
- The Simulink models provide visual simulations of the robot's leg motion and kinematic structure.

## Notes

- Adjust parameters and angles as needed to explore different configurations of the robot.
