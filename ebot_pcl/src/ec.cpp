#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
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
#include <sensor_msgs/PointCloud2.h>
// ros::Publisher pub;
std::vector<ros::Publisher> pub_vec;
sensor_msgs::PointCloud2::Ptr downsampled, output;
// pcl::PointCloud<pcl::PointXYZ>::Ptr output_p, downsampled_XYZ;

/*
Example of pcl usage where we try to use clustering to detect objects
in the scene.
Original code taken from:
http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
Usage:
    first run: rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
    then run: rosrun my_pcl_tutorial cluster_extraction
    and look at the visualization through rviz with:
    rosrun rviz rviz
    Set /base_link as the Fixed Frame, check out the published PointCloud2 topics
*/

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
//     // sensor_msgs::PointCloud2 cloud_filtered;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ  (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr output_p  (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f  (new pcl::PointCloud<pcl::PointXYZ>);
//     sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);
//     // sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);

//     pcl::fromROSMsg(*input, *cloud);

//     // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//     //         << " data points." << std::endl;

//     // Create the filtering object: downsample the dataset using a leaf size of 1cm
//     pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
//     sor.setInputCloud (cloud);
//     sor.setLeafSize (0.01f, 0.01f, 0.01f);
//     sor.filter (*downsampled);

//     // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

//     // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
//     pcl::fromROSMsg (*downsampled, *downsampled_XYZ);

//     //Create the SACSegmentation object and set the model and method type
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//     // Create the segmentation object
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     // Optional
//     seg.setOptimizeCoefficients (true);
//     // Mandatory
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);//For more info: wikipedia.org/wiki/RANSAC
//     seg.setMaxIterations (100);
//     seg.setDistanceThreshold (0.02);//How close a point must be to the model to considered an inlier

    

//     int i = 0, nr_points = (int) downsampled_XYZ->points.size ();

//     //Contains the plane point cloud
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//     // While 30% of the original cloud is still there
//     while (downsampled_XYZ->points.size () > 0.3 * nr_points)
//     {
//         // Segment the largest planar component from the remaining cloud
//         seg.setInputCloud (downsampled_XYZ);
//         seg.segment (*inliers, *coefficients);

//         if (inliers->indices.size () == 0)
//         {
//             std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//             break;
//         }

//         // Extract the planar inliers from the input cloud
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud (downsampled_XYZ);
//         extract.setIndices (inliers);
//         extract.setNegative (false);

//         // Get the points associated with the planar surface
//         extract.filter (*cloud_plane);
//         // std::cerr << "PointCloud representing the planar component: " 
//         //         << output_p->width * output_p->height << " data points." << std::endl;

//         // std::stringstream ss;
//         // ss << "table_scene_lms400_plane_" << i << ".pcd";
//         // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

//         // Remove the planar inliers, extract the rest
//         extract.setNegative (true);
//         extract.filter (*cloud_f);
//         downsampled_XYZ.swap(cloud_f);
//         i++;
//     }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (25);
    ec.setMaxClusterSize (250);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    ros::NodeHandle nh;

    //Create a publisher for each cluster
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);
        

        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);

        pub_vec.push_back(pub);
    }

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        
        //Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_cluster, *output);
        output->header.frame_id = input->header.frame_id;

        // Publish the data
        pub_vec[j].publish (output);
        ++j;
    }
   
}

int
main (int argc, char** argv)
{
    
    // Initialize ROS
    ros::init (argc, argv, "cluster_extraction");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/pcl/points_filtered3", 1, cloud_cb);
    

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_tut/object_cluster", 1);

    // Spin
    ros::spin ();
}