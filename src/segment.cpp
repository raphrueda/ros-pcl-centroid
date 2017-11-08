#include <ros/ros.h>
#include <iostream>

//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Euclidean cluster extraction specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>

std::vector<ros::Publisher> pub_centroid_vec;
std::vector<ros::Publisher> pub_segment_vec;
sensor_msgs::PointCloud2::Ptr downsampled, output;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_p (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);

	//Filter start
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	pcl_conversions::toPCL(*input, *cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud (cloudPtr);
	vg.setLeafSize (0.02f, 0.02f, 0.02f);
	vg.filter (cloud_filtered);

	//Change from pcl::PCLPointCloud2 --> sensor_msgs::PointCloud2
	pcl_conversions::moveFromPCL(cloud_filtered, *downsampled);
	//Filter end

	//Change from sensor_msgs::PointCloud2 --> pcl::PointXYZ
	pcl::fromROSMsg (*downsampled, *downsampled_XYZ);

	//Create SACSegmentation object (+ set model and method type)
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	//Create segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (400);
	seg.setDistanceThreshold (0.06);

	int i = 0, nr_points = (int) downsampled_XYZ->points.size ();

	//Contains the plane point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	while (downsampled_XYZ->points.size () > 0.3 * nr_points)
	{
		seg.setInputCloud (downsampled_XYZ);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		//Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (downsampled_XYZ);
		extract.setIndices (inliers);
		extract.setNegative (false);

		extract.filter (*cloud_plane);
		extract.setNegative (true);
		extract.filter (*cloud_f);
		downsampled_XYZ.swap (cloud_f);
		i++;
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (downsampled_XYZ);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.07);
	ec.setMinClusterSize (300);
	ec.setMaxClusterSize (1000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (downsampled_XYZ);
	ec.extract (cluster_indices);

	ros::NodeHandle nh;

	for (int i = 0; i < cluster_indices.size(); ++i)
	{
		std::string clusterName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
		ros::Publisher clusterPub = nh.advertise<sensor_msgs::PointCloud2> (clusterName, 1);
		pub_segment_vec.push_back(clusterPub);

		std::string centroidName = "/pcl_tut/centroid" + boost::lexical_cast<std::string>(i);
		ros::Publisher centroidPub = nh.advertise<geometry_msgs::Point> (centroidName, 1);
		pub_centroid_vec.push_back(centroidPub);

	}

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (downsampled_XYZ->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		geometry_msgs::Point centroid;
		centroid.x = 0;
		centroid.y = 0;
		centroid.z = 0;

		pcl::PointCloud<pcl::PointXYZ>::iterator i;
		for (i = cloud_cluster->points.begin(); i < cloud_cluster->points.end(); i++)
		{
			centroid.x += i->x;
			centroid.y += i->y;
			centroid.z += i->z;
		}

		centroid.x = (centroid.x / cloud_cluster->size()) * -1;
		centroid.y = centroid.y / cloud_cluster->size();
		centroid.z = centroid.z / cloud_cluster->size();

		sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
		pcl::toROSMsg (*cloud_cluster, *output);
		output->header.frame_id = input->header.frame_id;

		pub_segment_vec[j].publish (output);
		pub_centroid_vec[j].publish (centroid);
		std::cout << "Centroid for clustor " << j << ": [" << centroid.x << ", " << centroid.y << ", " << centroid.z << "]" << std::endl;
		++j;
	}
	//std::cout << "clusters segmented: " << j << std::endl;
}

int
main (int argc, char** argv)
{
	//ROS initialisation
	ros::init (argc, argv, "cluster_segmentation");
	ros::NodeHandle nh;

	//Create subscriber for the input
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	//Create publisher for the output
	//pub = nh.advertise<sensor_msgs::PointCloud2> ("segmented_output", 1);

	//Spin
	ros::spin ();
}
