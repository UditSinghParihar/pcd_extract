#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include "/home/cair/catkin_ws2/devel/include/semantic_mapper/SemLabel.h"
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define pi 3.14

void saveImg(cv::Mat &rgb){
	static int cnt = 0;
	char file_rgb[100];
	sprintf(file_rgb, "%04d_rgb.png", cnt);
	
	std::vector<int> png_parameters;
	png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION);
	png_parameters.push_back(9); 

	cv::imwrite(file_rgb , rgb, png_parameters);
	++cnt;
}

void callback(const semantic_mapper::SemLabelConstPtr &msg,
	const geometry_msgs::TransformStampedConstPtr &pose,
	const sensor_msgs::ImageConstPtr& msg_rgb){

	fprintf(stderr, "Inside callback.\n");

	ROS_INFO("Label is: %d", msg->lvl);
	
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

	static std::string fileName{"/home/cair/Desktop/tf_label.txt"};
	static std::ofstream fileWrite{fileName};

	fileWrite << x << " " << y <<  " " << yaw << " " << msg->lvl << std::endl;

	cv_bridge::CvImagePtr img_ptr_rgb;

	try{
		img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception:  %s", e.what());
		return;
	}

	cv::Mat& mat_rgb = img_ptr_rgb->image;
	saveImg(mat_rgb);
}

int main(int argc, char *argv[]){
	using namespace message_filters;
	ros::init(argc, argv, "tf_label");
	
	ros::NodeHandle nh;
	
	Subscriber<semantic_mapper::SemLabel> sub_label(nh, "semantic_label", 1);
	Subscriber<geometry_msgs::TransformStamped> sub_tf(nh, "in_tf", 1);
	Subscriber<sensor_msgs::Image> sub_rgb(nh, "camera/color/image_raw", 1);
	
	typedef sync_policies::ApproximateTime<semantic_mapper::SemLabel,
		geometry_msgs::TransformStamped, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_label, sub_tf, sub_rgb);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	ros::spin();
	
	return 0;
}