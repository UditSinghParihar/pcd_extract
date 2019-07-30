#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <fstream>
#include <string>

class Pose{
private:
	float m_x;
	float m_y;
	float m_theta;
	
public:
	Pose(float &x, float &y, float &theta): m_x{x}, m_y{y}, m_theta{theta}{};
};

void geoCallback(const geometry_msgs::TransformStampedConstPtr& pose){
	ROS_INFO("Recived tf messages.");
	float q_x = pose->transform.rotation.x;
	float q_y =	pose->transform.rotation.y;
	float q_z = pose->transform.rotation.z;
	float q_w = pose->transform.rotation.w;

	Eigen::Quaternionf q(q_w, q_x, q_y, q_z);
	Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

	float x = pose->transform.translation.x;
	float y = pose->transform.translation.y;
	float theta = euler[2];

	static std::string fileName{"/home/cair/backup/rapyuta3/poses.txt"};
	static std::ofstream fileWrite{fileName};
	fileWrite << x << " " << y <<  " " << theta << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_subscriber_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("in_tf",1000, geoCallback);
    ros::spin();

    return 0;
}