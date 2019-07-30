#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <set>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

ros::Publisher pub;

class genCloud{
private:
	PointCloudT::Ptr cloud;
	cv::Mat& rgb;
	cv::Mat& depth;
	static int cnt;

public:
	genCloud(cv::Mat &argRgb, cv::Mat &argDepth) : cloud{new PointCloudT()}, rgb{argRgb}, depth{argDepth} {};

	void images2cloud(void){
		const float fx = 383.70, fy = 383.70, cx = 324.18, cy = 239.08, depth_threshold = 3;
		cloud->is_dense = true;
		int image_index = 0, bad_image_index = 0;
		std::set<float> list;

		for(int y=0; y<rgb.rows; ++y){
			for(int x=0; x<rgb.cols; ++x){
				float depthValue = depth.at<float>(y, x)/1000;
				if(depthValue == 0 || depthValue > depth_threshold){
					++bad_image_index;
				}
				else{			
					pcl::PointXYZRGB point;
					list.insert(depthValue);

					point.z = depthValue;
					point.x = (x - cx) * point.z / fx;
					point.y = (y - cy) * point.z / fy;
					
					// float temp_z = point.z; 
					// float temp_x = point.x;
					// float temp_y = point.y;
					// point.x = temp_z;
					// point.z = -temp_y;
					// point.y = -temp_x;
					
					point.r = rgb.at<cv::Vec3b>(y, x)[0];
					point.g = rgb.at<cv::Vec3b>(y, x)[1];
					point.b = rgb.at<cv::Vec3b>(y, x)[2];
					++image_index;
					cloud->points.push_back(point);
				}
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;

		// fprintf(stderr, "size of list: %lu\n", list.size());
		// for(auto e : list)
		// 	std::cout << e << " ";
		// std::cout << "\n--\n";	
	}

	void publish(void){
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*cloud, output);
		output.header.frame_id = "realsense_link";
		pub.publish(output);
	}

	void saveImages(void){
		static int cnt = 0;
	    char file_rgb[100];
	    char file_depth[100];

	    sprintf( file_rgb, "%04d_rgb.png", cnt );
	    sprintf( file_depth, "%04d_depth.png", cnt );

	    std::vector<int> png_parameters;
	    png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
	  
	    png_parameters.push_back( 9 ); 

	    cv::imwrite(file_rgb , rgb, png_parameters);
	    cv::imwrite(file_depth, depth, png_parameters);
	    ++cnt;
	}
};

int genCloud::cnt = 0;

void callback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth){
	// fprintf(stderr, "encoding: %s\n", msg_depth->encoding.c_str());

    cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth;
    
    try{
        img_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat& mat_depth = img_ptr_depth->image;
    cv::Mat& mat_rgb = img_ptr_rgb->image;

    mat_depth.convertTo(mat_depth, CV_32FC1);
    genCloud generator{mat_rgb, mat_depth};
    generator.images2cloud();
    generator.publish();

    // Uncomment below lines to save images
    // generator.saveImages();
}


int main(int argc, char *argv[]){
	using namespace message_filters;
	ros::init(argc, argv, "cloudify");
	
	ros::NodeHandle nh;
	
	Subscriber<sensor_msgs::Image> sub_rgb(nh, "camera/color/image_raw", 1);
	Subscriber<sensor_msgs::Image> sub_depth(nh, "camera/depth/image_rect_raw", 1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image,
		sensor_msgs::Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_rgb, sub_depth);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub = nh.advertise<sensor_msgs::PointCloud2>("modified/camera/points2/", 1);
	
	ros::spin();
	
	return 0;
}