#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include "/home/cair/catkin_ws2/devel/include/semantic_mapper/SemLabel.h"

void callback(const semantic_mapper::SemLabelConstPtr &msg,
	const geometry_msgs::TransformStampedConstPtr &pose){

	ROS_INFO("Label is: %d", msg->lvl);
	
	float q_x = pose->transform.rotation.x;
	float q_y =	pose->transform.rotation.y;
	float q_z = pose->transform.rotation.z;
	float q_w = pose->transform.rotation.w;

	Eigen::Quaternionf q(q_w, q_x, q_y, q_z);
	Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

	float x = pose->transform.translation.x;
	float y = pose->transform.translation.y;
	float theta = euler[2];

	static std::string fileName{"/home/cair/backup/rapyuta3/tf_label.txt"};
	static std::ofstream fileWrite{fileName};

	fileWrite << x << " " << y <<  " " << theta << " " << msg->lvl << 
	std::endl;
}

int main(int argc, char *argv[]){
	using namespace message_filters;
	ros::init(argc, argv, "tf_label");
	
	ros::NodeHandle nh;
	
	Subscriber<semantic_mapper::SemLabel> sub_label(nh, "semantic_label", 1);
	Subscriber<geometry_msgs::TransformStamped> sub_tf(nh, "in_tf", 1);
	
	typedef sync_policies::ApproximateTime<semantic_mapper::SemLabel,
		geometry_msgs::TransformStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_label, sub_tf);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();
	
	return 0;
}