

# VR Teleoperation of Franka Arm with ROS 2, MoveIt Servo, and Oculus Reader

This project demonstrates VR-based teleoperation of a Franka FR3 controlled with a Meta Quest controller using ROS 2, MoveIt Servo, and Oculus Reader.

## Features
1. **Low-Level Control**: The Franka arm is controlled by the `fr3_arm_controller`, which uses joint torque commands to achieve velocity tracking.
2. **Singularity Handling**: MoveIt Servo reduces speed near singularities to prevent the arm from losing control.
3. **Smooth Trajectories**: MoveIt Servo performs online trajectory interpolation of desired velocities, ensuring smooth commands sent to the ROS controller.


## Installation

### Prerequisites
- **Real-Time Kernel**: If your system does not yet have a real-time kernel installed and tested, follow the [Franka official tutorial](https://frankaemika.github.io/docs/installation_linux.html) to install and verify it with the provided test program.
- **Docker**: Install `franka_ros2` via Docker. Refer to [libfranka-docker](https://github.com/ZorAttC/libfranka-docker/blob/main/docker_launch_files/docker-compose.yml).

### Setup Workspace
1. **Clone MoveIt2 Tutorials**:
   ```bash
   cd /ws_moveit/src
   git clone -b main https://github.com/moveit/moveit2_tutorials
   vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
   ```
   > **Warning**: MoveIt 2 is actively updated. Ensure the version of MoveIt 2 packages pulled via `moveit2_tutorials.repos` is `branch main` (tag `2.13.0`) to match the correct `moveit_servo` version. Avoid conflicts by removing any existing MoveIt installations:
   ```bash
   sudo apt remove ros-$ROS_DISTRO-moveit*
   ```

2. **Install Dependencies**:
   ```bash
   sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```

3. **Build Workspace**:
   ```bash
   cd ..
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Add `franka_vr` Package**:
   ```bash
   cd src
   git clone https://github.com/ZorAttC/franka_vr.git
   cd ..
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Execution
1. **Launch Franka Arm**:
   Open a terminal inside the Docker container:
   ```bash
   source /ros2_ws/install/setup.bash  # franka_ros2 environment
   source /ws_moveit2/install/setup.bash  # MoveIt 2 environment
   ros2 launch franka_vr franka_twist.launch.py
   ```
   After this step, RViz will display the Franka arm’s current pose.

2. **Start VR Teleoperation**:
   Open a second terminal:
   ```bash
   source /ros2_ws/install/setup.bash  # franka_ros2 environment
   source /ws_moveit2/install/setup.bash  # MoveIt 2 environment
   sh /ws_moveit2/src/oculus_reader/start_vr.sh
   ```
   This step displays the coordinate frames of the left and right hands in RViz.  
   - **Controls**: After activating the controller, press and hold `rightTrig` to enable arm movement, hold `rightGrip` to close the gripper, and release `rightGrip` to open it.

## References
- [MoveIt Servo Tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Tutorials](https://github.com/moveit/moveit_tutorials)
- [Oculus Reader](https://github.com/rail-berkeley/oculus_reader)
- [Franka ROS 2](https://github.com/frankaemika/franka_ros2)
- [Franka ROS 2 Docs](https://frankaemika.github.io/docs/franka_ros2.html)
- [Franka Installation Guide](https://frankaemika.github.io/docs/installation_linux.html)

## Limitations
1. **Planning Scene Issue**: The `planning_scene` fails to retrieve `fr3_finger_joint1` (this does not affect gripper functionality, but the gripper state in RViz won’t update).
2. **Singularity Behavior**: MoveIt Servo slows down near singularities but does not avoid them (advanced solutions like DLS control could address this).
3. **Oculus Data Stability**: Data from the Oculus Reader occasionally jumps; filtering is applied but remains unstable.
4. **Joint Limits**: No joint limits are enforced for the 7-DOF arm, which could reduce singularity occurrences if implemented.

## Troubleshooting
1. **Missing `osqp` Library**: Install `osqp` version `<=0.6.0`.
2. **RViz Fails to Start**: Run `xhost +local:docker` to resolve display issues.

## Discussion

### Avoiding Singularities
Singularities occur when the arm reaches a pose where the end-effector loses a degree of freedom in Cartesian space, causing the Jacobian matrix to become singular. This results in infinite joint velocities when mapping from Cartesian to joint space. In teleoperation, this manifests as:
1. Sudden jerks with high velocity at certain positions.
2. The arm stopping at singular positions due to velocity limits, no longer following commands.
3. Shoulder joint singularities leading to multiple solutions, potentially causing abrupt 180-degree rotations of the shoulder or wrist joints.

In teleoperation, the goal is to prevent the arm from entering singularities or to approach them at minimal speed. Near singularities, small end-effector movements can cause large joint changes, leading to poor motion quality or loss of control.

#### Strategies to Avoid Singularities
1. **Hardware Design**: Minimize singularities in the arm’s workspace through careful kinematic design.
2. **DLS Method**: Use the Damped Least Squares (DLS) approach to compute the pseudo-inverse of the Jacobian. Switch to DLS control when the Jacobian’s determinant is small, reducing joint velocities near singularities at the cost of end-effector precision.
3. **Planning Avoidance**: Avoid regions prone to singularities during trajectory planning.
4. **Predictive Slowdown**: Use the Jacobian’s norm or determinant as a criterion. Predict the arm’s direction using recent end-effector position increments and reduce speed if the criterion worsens, preventing entry into singular regions (this sacrifices tracking accuracy).

The most elegant solution for teleoperation is Option 2 (DLS), balancing responsiveness, safety (avoiding excessive joint velocities/accelerations), and slight precision trade-offs. MoveIt Servo’s current implementation resembles Option 4, slowing down near singularities to maintain control.

### Initialization
The arm’s workspace often mismatches the operator’s, especially for non-humanoid designs. Direct Euclidean mapping limits flexibility, as humans cannot reach many of the arm’s end-effector positions. Proper alignment and initialization between the arm’s workspace and the operator’s comfortable range are critical. This involves:
- Setting the VR device’s base coordinate frame.
- Adjusting pose scale and offset, typically fine-tuned through experimentation.

For non-lab commercial scenarios (e.g., data collection centers), a fast workspace initialization algorithm is needed. A preliminary idea is to provide a controller for online parameter tuning (scale and offset). While slightly risky and suited for experienced operators, this allows users to adjust parameters during specific tasks and save them for future use.
