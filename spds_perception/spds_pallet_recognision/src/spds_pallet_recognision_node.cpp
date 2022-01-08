#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>

# define PI           3.14159265358979323846 

ros::Publisher pub;

void
send_marker ( const Eigen::Vector4f normal, const Eigen::Vector4f centroid)
{
  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "robot1/asus_fork_rgb_optical_frame";
  sphere.header.stamp = ros::Time::now();
  sphere.ns = "pallet_detection";

  sphere.id = 0;

  sphere.type = visualization_msgs::Marker::SPHERE;

  sphere.pose.position.x = centroid[0];
  sphere.pose.position.y = centroid[1];
  sphere.pose.position.z = centroid[2];

  tf2::Quaternion quaterion;

  quaterion.setRPY(normal[0],normal[1],normal[2]);

  quaterion = quaterion.normalize();

  sphere.pose.orientation.x = quaterion[0];
  sphere.pose.orientation.y = quaterion[1];
  sphere.pose.orientation.z = quaterion[2];
  sphere.pose.orientation.w = quaterion[3];

  sphere.scale.x = 0.05;
  sphere.scale.y = 0.05;
  sphere.scale.z = 0.05;

  sphere.color.g = 1.0f;
  sphere.color.a = 1.0;
  sphere.color.r = 0.0f;
  sphere.color.b = 0.0f;

  sphere.lifetime = ros::Duration();

  pub.publish(sphere);



}

void
segment_pallet (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc1 (new pcl::PointCloud<pcl::PointXYZ>),
                                      pc2 (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_segment (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_segment_up (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_segment_front (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(pcl_pc2, *pc1);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc2);

  pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients), coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliersZ (new pcl::PointIndices), inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloudZ_p (new pcl::PointCloud<pcl::PointXYZ>), cloudZ_f (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_front_filtred (new pcl::PointCloud<pcl::PointXYZ>);

  // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  // viewer.setBackgroundColor (0.0, 0.0, 0.5);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::SACSegmentation<pcl::PointXYZ> segZ;

  seg.setOptimizeCoefficients (true);

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);


  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
  segZ.setAxis(axis);
  segZ.setEpsAngle(  PI - 2.5f ); // plane can be within 30 degrees of X-Y plane
  segZ.setOptimizeCoefficients (true);

  segZ.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  segZ.setMethodType (pcl::SAC_RANSAC);
  segZ.setMaxIterations (1000);
  segZ.setDistanceThreshold (0.01);



  int i, nr_points;

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  i = 0;
  nr_points = (int) pc1->size ();

  while (pc1->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (pc1);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (pc1);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    *cloud_segment_up += *cloud_p;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    pc1.swap (cloud_f);
    i++;
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract_front;
  extract_front.setInputCloud (pc2);
  extract_front.setIndices (inliers);
  extract_front.setNegative (true);
  extract_front.filter (*cloud_front_filtred);


  pcl::ExtractIndices<pcl::PointXYZ> extractZ;
  i = 0;
  nr_points = (int) cloud_front_filtred->size ();

  while (cloud_front_filtred->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    segZ.setInputCloud (cloud_front_filtred);
    segZ.segment (*inliersZ, *coefficientsZ);
    if (inliersZ->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extractZ.setInputCloud (cloud_front_filtred);
    extractZ.setIndices (inliersZ);
    extractZ.setNegative (false);
    extractZ.filter (*cloudZ_p);

    *cloud_segment += *cloudZ_p;

    // Create the filtering object
    extractZ.setNegative (true);
    extractZ.filter (*cloudZ_f);
    cloud_front_filtred.swap (cloudZ_f);
    i++;
  }

  i = 0;
  nr_points = (int) cloud_segment->size ();

  while (cloud_segment->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_segment);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_segment);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    *cloud_segment_front += *cloud_p;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_segment.swap (cloud_f);
    i++;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>), central_point (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_segment_front);
  chull.setAlpha (0.2);
  chull.reconstruct (*cloud_hull);
  
  Eigen::Vector4f centroid;//new object for centroid calculation
  pcl::PointXYZ minpt, maxpt;//min max boundary of new cloud
  std::vector<int> pointIdxRadiusSearch; 

  pcl::compute3DCentroid(*cloud_hull, centroid);
  pcl::getMinMax3D(*cloud_hull, minpt, maxpt);

  std::cerr << "Min point: " << minpt << std::endl;
  std::cerr << "Max point: " << maxpt << std::endl;

  std::cerr << "Centroid point: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << std::endl;

  // viewer.addCoordinateSystem (0.5, "axis", 0);
  // viewer.addPointCloud<pcl::PointXYZ>(cloud_segment_front, "cloud");
  float curvature; 
  Eigen::Vector4f normal; 
  computePointNormal (*cloud_segment_front, normal, curvature);

  send_marker(normal, centroid);


  // while (!viewer.wasStopped ())
  // {
  //   viewer.spinOnce ();
  // }

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pallet_recognision");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<visualization_msgs::Marker>("output", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, segment_pallet);

  // Spin
  ros::spin ();
}