#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <tf/transform_datatypes.h>

#define pi 3.14
#define deg2rad(num)(num * pi / 180)
#define rad2deg(num)(num * 180/pi)

void geoCallback(const geometry_msgs::TransformStampedConstPtr& pose){
	float q_x = pose->transform.rotation.x;
	float q_y =	pose->transform.rotation.y;
	float q_z = pose->transform.rotation.z;
	float q_w = pose->transform.rotation.w;
	float x = pose->transform.translation.x;
	float y = pose->transform.translation.y;

	tf::Quaternion q(q_x, q_y, q_z, q_w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw += (-pi/2);
	fprintf(stderr, "Rotation: %g\n", rad2deg(yaw));

	static std::string fileName{"/home/cair/backup/rapyuta3/poses.txt"};
	static std::ofstream fileWrite{fileName};
	fileWrite << x << " " << y <<  " " << yaw << std::endl;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "tf_subscriber_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("trajectory_0",1, geoCallback);
	ros::spin();

	return 0;
}