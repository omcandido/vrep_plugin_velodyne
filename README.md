# vrep_plugin_velodyne

V-REP plugin that publishes a full revolution of points in PointCloud2 into ROS under the topic /velodyne_points. The points are global (relative to the odom frame). This plugin is meant to be used with sensors like the built-in model velodyne VPL-16.

The reason why the point cloud is published globally is because at every simulation step the points are converted to the origin frame in order to correct the mismatch due to the movement of the vehicle. You can modify the plugin or simply create a node that reads this point cloud and converts it to the base_link (or whatever the velodyne frame is) frame.


TODO:
- Parametrize frames
- Publish local pointcloud
