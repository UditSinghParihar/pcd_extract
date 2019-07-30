#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

ros::Publisher pub;

class Assembler{
private:
	static PointCloudT::Ptr assembledMap;
	Eigen::Affine3f translate;

public:
	Assembler(): translate{Eigen::Affine3f::Identity()}{};

	void voxelize(const PointCloudT::Ptr cloud, float size){
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(cloud);
		voxel.setLeafSize(size, size, size);
		
		voxel.filter(*cloud);
	}

	void addCloud(PointCloudT::Ptr cloud){
		*assembledMap += *cloud;	
	}

	void transform(PointCloudT::Ptr cloud){
		pcl::transformPointCloud(*cloud, *cloud, translate);
	}

	void cb_cloud(const sensor_msgs::PointCloud2ConstPtr& input){
		PointCloudT::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud);
		voxelize(cloud, 0.1);
		transform(cloud);
		addCloud(cloud);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*assembledMap, output);
		output.header.frame_id = "map";
		pub.publish(output);
	}

	void cb_tf(const geometry_msgs::TransformStampedConstPtr& pose){
		float q_x = pose->transform.rotation.x;
		float q_y =	pose->transform.rotation.y;
		float q_z = pose->transform.rotation.z;
		float q_w = pose->transform.rotation.w;
		Eigen::Quaternionf q(q_w, q_x, q_y, q_z);
		Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

		translate.rotate(Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ()));
		
		float x = pose->transform.translation.x;
		float y = pose->transform.translation.y;
		
		translate.translation() << x, y, 0.0;
	}
};

PointCloudT::Ptr Assembler::assembledMap = boost::shared_ptr<PointCloudT>
(new PointCloudT());

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
	const geometry_msgs::TransformStampedConstPtr& pose){

	Assembler assemble;
	assemble.cb_tf(pose);
	assemble.cb_cloud(cloud);
}

int main(int argc, char *argv[]){
	using namespace message_filters;
	ros::init(argc, argv, "carto_assemble");
	
	ros::NodeHandle nh;
	
	Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "in_cloud", 1);
	Subscriber<geometry_msgs::TransformStamped> sub_tf(nh, "in_tf", 1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
		geometry_msgs::TransformStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_tf);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	
	ros::spin();
	
	return 0;
}