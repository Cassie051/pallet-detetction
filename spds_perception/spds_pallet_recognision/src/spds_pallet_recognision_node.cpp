#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
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

# define PI           3.14159265358979323846 

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);
  // output = *input;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);

  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pc);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (pcl_pc);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>cloud_with_normals;

  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);

//   for (int i = 0; i < 50; i++)
// {
//     cloud_with_normals.points[i].x = pcl_pc->points[i].x;
//     cloud_with_normals.points[i].y = pcl_pc->points[i].y;
//     cloud_with_normals.points[i].z = pcl_pc->points[i].z;

//     cloud_with_normals.points[i].r = 0;
//     cloud_with_normals.points[i].g = 100;
//     cloud_with_normals.points[i].b = 0;

//   //  cloud_normals -> Which you have already have; generated using pcl example code 


//     cloud_with_normals.points[i].curvature = cloud_normals->points[i].curvature;

//     cloud_with_normals.points[i].normal_x = cloud_normals->points[i].normal_x;
//     cloud_with_normals.points[i].normal_y = cloud_normals->points[i].normal_y;
//     cloud_with_normals.points[i].normal_z = cloud_normals->points[i].normal_z;

//     std::cerr << "Point value: " << cloud_with_normals.points[i].x << ", " << cloud_with_normals.points[i].y << ", " << cloud_with_normals.points[i].z;
//     std::cerr << "Normal vector X, Y, Z: " << cloud_with_normals.points[i].normal_x << ", " << cloud_with_normals.points[i].normal_y << ", " << cloud_with_normals.points[i].normal_z;
// }


 
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(pcl_pc, cloud_normals);
    viewer.addCoordinateSystem (1.0, "axis", 0);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }

  // pcl::fromPCLPointCloud2(pcl_pc,*pt_cloud);
  // pub.publish (cloud_normals);
}

class PalletSegmentation
{
  public:
  
    void segment_pallet (const sensor_msgs::PointCloud2ConstPtr& input);

  private:
    void
    set_up_segment_Z (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_pc,
                      pcl::PointIndices::Ptr &inliersZ, 
                      pcl::ModelCoefficients::Ptr &coefficientsZ,
                      pcl::SACSegmentation<pcl::PointXYZ> &segZ);
  

};

void
set_up_segment_Z (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_pc,
                  pcl::PointIndices::Ptr &inliersZ, 
                  pcl::ModelCoefficients::Ptr &coefficientsZ,
                  pcl::SACSegmentation<pcl::PointXYZ> &segZ)
{
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
  segZ.setAxis(axis);
  segZ.setEpsAngle(  30.0f * (PI/180.0f) );// plane can be within ~36.2 degrees of X-Y plane

  segZ.setOptimizeCoefficients (true);

  segZ.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  segZ.setMethodType (pcl::SAC_RANSAC);
  segZ.setMaxIterations (1000);
  segZ.setDistanceThreshold (0.01);

  segZ.setInputCloud (pcl_pc);
  // segZ.segment (*inliersZ, *coefficientsZ);
}

pcl::PointCloud<pcl::PointXYZ>
make_segment (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_pc,
              pcl::PointIndices::Ptr &inliers, 
              pcl::ModelCoefficients::Ptr &coefficients,
              pcl::SACSegmentation<pcl::PointXYZ> &seg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int) pcl_pc->size ();

  while (pcl_pc->size () > 0.3 * nr_points)
  {
    // segZment the largest planar component from the remaining cloud
    seg.setInputCloud (pcl_pc);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (pcl_pc);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    
    *segment_cloud += *cloud_p;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    pcl_pc.swap (cloud_f);
    i++;
  }
  return *segment_cloud;
}

void
segment_pallet (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);
  // output = *input;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);

  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pc);

  pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients), coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliersZ (new pcl::PointIndices), inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> segZ;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  set_up_segment_Z(pcl_pc, inliersZ, coefficientsZ, segZ);

  // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  // viewer.setBackgroundColor (0.0, 0.0, 0.5);
  // viewer.addCoordinateSystem (0.5, "axis", 0);

  pcl::PointCloud<pcl::PointXYZ> segment_pointcloud = make_segment(pcl_pc, inliersZ, coefficientsZ, segZ);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&segment_pointcloud);

  // viewer.addPointCloud<pcl::PointXYZ>(ptrCloud);
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (ptrCloud);

  while (!viewer.wasStopped ())
  {
  }


    
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

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("input", 1, segment_pallet);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}