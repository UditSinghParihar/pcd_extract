#include <ros/ros.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

ros::Publisher pub;


void callback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_cloud){
	cv_bridge::CvImagePtr img_ptr_rgb;
	cv_bridge::CvImagePtr img_ptr_depth;
	cv_bridge::CvImagePtr img_ptr_cloud;


	try{
	    img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
	}
	catch (cv_bridge::Exception& e){
	    ROS_ERROR("cv_bridge exception:  %s", e.what());
	    return;
	}
	try{
	    img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
	    ROS_ERROR("cv_bridge exception:  %s", e.what());
	    return;
	}
	try{
	    img_ptr_cloud = cv_bridge::toCvCopy(*msg_cloud, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
	    ROS_ERROR("cv_bridge exception:  %s", e.what());
	    return;
	}

	cv::Mat& mat_depth = img_ptr_depth->image;
	cv::Mat& mat_rgb = img_ptr_rgb->image;
	cv::Mat& mat_cloud = img_ptr_cloud->image;

	char file_rgb[100];
	char file_depth[100];
	char file_cloud[100];
	static int cnt = 0;

	sprintf(file_rgb, "%05d_rgb.png", cnt );
	sprintf(file_depth, "%05d_depth.png", cnt );
	sprintf(file_cloud, "%05d_cloud.png", cnt );

	std::vector<int> png_parameters;
	png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );

	png_parameters.push_back( 9 ); 

	cv::imwrite(file_rgb , mat_rgb, png_parameters);
	cv::imwrite(file_depth, mat_depth, png_parameters);
	cv::imwrite(file_cloud , mat_cloud, png_parameters);
	++cnt;
}


int main(int argc, char *argv[]){
	using namespace message_filters;
	ros::init(argc, argv, "image_extraction");
	
	ros::NodeHandle nh;
	
	message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh, "/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, "/camera/depth/image_rect_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> sub_cloud(nh, "/explore/grid_image", 1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image,
		sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_rgb, sub_depth, sub_cloud);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	
	ros::spin();
	
	return 0;
}