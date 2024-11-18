# 3D-planning-with-bonxai

This repository aims to demonstrate how to use Bonxai for 3D planning with a quadrotor, utilizing Ardupilot for control.
> Note: This is a side project I am working on during my free time

### Setup Instructions:

#### Barebone Setup:
Here are the instructions to setup to run a small demonstration with bonxai.
My setup is using Ubunut 22.04 with host installed ROS 2 humble and gz version Harmonic and Ardupilot master version

The guide assumes you are using Ubuntu 22.04 and have the following installed:
* ROS 2 humble
* Gazebo Harmonic
* Ardupilot stable Version

If you are setting up a new system, follow these setup instruction from reference docs:
* [Setting up ROS 2](https://docs.ros.org/en/humble/Installation.html) and [MicroXRCEDDSGen](https://ardupilot.org/dev/docs/ros2.html)
* [Setting up ROS 2 with Ardupilot SITL](https://ardupilot.org/dev/docs/ros2-sitl.html)
* [Setting up ROS 2 with Ardupilot SITL in Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)

##### Troubleshooting: ROS2 Topic Visibility
If you encounter issues where no topic are visible under the `/ap` namespace when trying the `Setting up ROS 2 with Ardupilot SITL`, try the following:
```bash
export ROS_LOCALHOST_ONLY=0
```
This change ensures ROS2 can communicate across the network.
While I haven't yet figured out which specific components in the SITL setup are not using the LOCALHOST IP, and therefore why this solution works, it has proven effective in resolving the issue.
For more details, you can check the discussion [here](https://discuss.ardupilot.org/t/ros2-with-sitl-not-working/124942/3).


#### Setting Up 3D Lidar in Gazebo
The default `iris_maze` world launched with `ros2 launch ardupilot_gz_bringup iris_maze.launch.py` uses `iris_with_lidar` model available in the `ardupilot_gz_description` ros 2 pkg. By default, `iris_with_lidar` model uses a 2D lidar. In order to use a 3D lidar, we need to modify the `gpu_lidar` sensor configuration in the `iris_with_lidar/model.sdf` with following update:
```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
<gz_frame_id>base_scan</gz_frame_id>
<pose>0 0 0 0 0 0</pose>
<topic>lidar</topic>
<update_rate>15</update_rate>
<visualize>0</visualize>
<lidar>
    <scan>
    <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159265</min_angle>
        <max_angle>3.14159265</max_angle>
    </horizontal>
    <vertical>
        <samples>60</samples>
        <resolution>1</resolution>
        <min_angle>-0.523599</min_angle>
        <max_angle>0.523599</max_angle>
    </vertical>
    </scan>
    <range>
    <min>0.5</min>
    <max>10</max>
    <resolution>0.05</resolution>
    </range>
</lidar>
<visualize>true</visualize>
</sensor>

```


#### Configuring `ros_gz_bridge`
To link Gazebo 3D Lidar pointcloud topic with ROS 2, update the `ardupilot_gz_bringup/config/iris_lidar_bridge.yaml` configuration file by adding a mapping for `/lidar/points` gz topic to `/cloud_in` ros 2 topic.
Add the following lines to the end of the configuration file to do that.

```yaml
- ros_topic_name: "cloud_in"
  gz_topic_name: "/lidar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

For more information, refer to these resources:
* [gpu_lidar_bridge.launch.py](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_sim_demos/launch/gpu_lidar_bridge.launch.py) 
* [gpu_lidar.sdf](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_point_cloud/examples/gpu_lidar.sdf/)

#### Setup Instructions for Bonxai:
Follow the setup instructions for Bonxai available [here](https://github.com/facontidavide/Bonxai/tree/main/bonxai_ros).
Checkout to tag v0.6.0 for the the latest release.
Pay close attention to the description of frames and configuration params provided [here](https://github.com/facontidavide/Bonxai/tree/main/bonxai_ros#3-ros-2-node-api).
Based on the Gazebo model in this setup, the lidar frame is defined as `base_scan`, and the global frame can be set to `odom`. Below is an example configuration file used for this setup:
The configuration file I use:
```yaml
bonxai_server_node:
  ros__parameters:
    resolution: 0.1
    frame_id: "odom"
    base_frame_id: "base_scan"

    occupancy_min_z: -5.0
    occupancy_max_z: 5.0

    sensor_model:
      max_range: 10.0
      hit: 0.7
      miss: 0.4
      min: 0.12
      max: 0.97

    latch: false
```

>**A very important note**: <br>
When using Bonxai for the first time, I encountered a stability and performance issue, which is explained in detail [here](https://github.com/facontidavide/Bonxai/issues/35). The issue was related to having some `inf` values in the PCL point cloud.
The suggested fix for this problem is also discussed in the linked issue. Please refer to it to ensure smooth operation.


Once setup is complete, you can use MAVProxy to control the drone and manually create a 3D map of the environment.
From my tests, when the point cloud from Gazebo was received at an average rate of 14 Hz, the Bonxai map was being updated at an average rate of 11 Hz.

Side Note:
If you wish to get debug statements while running the `bonxai_server_node`, you can update the launch description for the node as follows:
```python
# Bonxai Server Node
bonxai_node = Node(
    package=package,
    executable='bonxai_server_node',
    name='bonxai_server_node',
    emulate_tty=True,
    parameters=[bonxai_params],
    output="screen",
    arguments=['--ros-args', '--log-level', 'DEBUG']
)
```
