# vrep_plugin_velodyne

## Citation

J. López, C. Otero, R. Sanz, E. Paz, E. Molinos and R. Barea, "A new approach to local navigation for autonomous driving vehicles based on the curvature velocity method," 2019 International Conference on Robotics and Automation (ICRA), 2019, pp. 1751-1757, doi: 10.1109/ICRA.2019.8794380.
___
V-REP plugin that publishes a full revolution of points in PointCloud2 into ROS under the topic /velodyne_points. The points are global (relative to the odom frame). This plugin is meant to be used with sensors like the built-in model velodyne VPL-16.

The reason why the point cloud is published globally is because at every simulation step the points are converted to the origin frame in order to correct the mismatch due to the movement of the vehicle. You can modify the plugin or simply create a node that reads this point cloud and converts it to the base_link (or whatever the velodyne frame is) frame.


TODO:
- Parametrize frames
- Publish local pointcloud
