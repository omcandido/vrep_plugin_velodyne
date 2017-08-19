#include "../include/vrep_plugin_velodyne/ros_server_velodyne.h"
#include "../include/v_repLib.h"

ros::NodeHandle* ROS_server::node = NULL;

// Publishers:
ros::Publisher ROS_server::pointCloud_publisher;

bool ROS_server::initialize()
{
	int argc = 0;
	char** argv = NULL;
    ros::init(argc,argv,"vrep_velodyne");

	if(!ros::master::check())
		return(false);
	
    node=new ros::NodeHandle();


	// Enable the publishers:
    pointCloud_publisher=node->advertise<sensor_msgs::PointCloud2>("/velodyne_points",1);

	return(true);
}

ros::Publisher ROS_server::getPublisher()
{
    return ROS_server::pointCloud_publisher;
}

void ROS_server::shutDown()
{

	// Disable the publishers:
    pointCloud_publisher.shutdown();

	// Shut down:
	ros::shutdown();
}
