Launch Files
============

This section describes the ROS 2 launch files used to start and visualize the Mensabot system.  
Launch files serve as entry points for different operation modes and combine multiple ROS 2 nodes into a coherent runtime configuration.  
They are not part of the public Python API and are therefore documented conceptually rather than via automatic code extraction.

Mensabot Bringup
----------------

The Mensabot bringup launch file starts the core runtime components required to operate the robot.  
It is intended to be used when bringing up the robot on real hardware or in a fully integrated system setup.

The main purpose of this launch file is to initialize the complete robot software stack, including control, state publishing, and controller management.  
It starts ROS 2 Control for hardware abstraction and controller handling, the robot state publisher to broadcast the TF tree derived from the URDF model, and the required controller spawners such as the joint state broadcaster and the base controller.

The robot model is generated at launch time by processing a Xacro file into a URDF.  
The resulting robot description is passed to both the ROS 2 Control node and the robot state publisher to ensure a consistent robot model across the system.

Controller parameters are loaded from YAML configuration files located in the directory  
``mensabot_bringup/config/``.  
These configuration files define controller types, interfaces, and tuning parameters.

The launch file is located at  
``mensabot_bringup/launch/mensabot.launch.py``.

The full robot bringup can be started with the following command:

ros2 launch mensabot_bringup mensabot.launch.py

Before launching, ensure that the correct ROS 2 workspace is sourced and that all required controllers are properly defined.  
TF frames such as ``odom`` must be available for correct operation.

Visualization in RVIZ
---------------------

The visualization launch file starts RViz with predefined configuration files to visualize the robot model and runtime data.  
It is mainly intended for development, debugging, and demonstration purposes.

This launch file provides different visualization modes, ranging from a simple model preview to a full visualization including live robot data.  
Two modes are supported: a standalone model visualization that displays only the robot model generated from the description package, and a full robot visualization that displays the robot together with live data such as TF frames, markers, and paths.

The active visualization mode is selected using a launch argument called ``standalone``.  
If ``standalone`` is set to true, RViz is started in model-only mode.  
If set to false, RViz is started with the full robot visualization setup.

RViz configuration files are loaded from the directory  
``mensabot_description/rviz/``.  
Different configuration files are used depending on the selected visualization mode.

In addition to RViz, the visualization launch file also starts helper nodes used for visualization and analysis.  
These include an odometry logger node that publishes the robot trajectory as a ``nav_msgs/Path`` and a marker visualization node that displays markers such as points and arrows for debugging and analysis.

The launch file is located at  
``mensabot_description/launch/visualization.launch.py``.

RViz can be started in standalone mode with:

ros2 launch mensabot_description visualization.launch.py standalone:=true

To start RViz with full robot visualization, use:

ros2 launch mensabot_description visualization.launch.py standalone:=false

If the robot model does not appear in RViz, verify that the description package is built and sourced correctly and that TF data is available before starting the visualization.
