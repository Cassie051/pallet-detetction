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


class PalletSegmentation
{
    public:
        PalletSegmentation()
        {
            set_up_seg();
            set_up_segZ();
            set_up_viewer();
        }
        ~PalletSegmentation()
        {
            delete[] pc1;
            delete[] pc2;
            delete[] cloud_segment;
            delete[] cloud_segment_up;
            delete[] cloud_segment_front;
            delete[] coefficients;
            delete[] coefficientsZ;
            delete[] inliers;
            delete[] inliersZ;
            delete[] cloud_p;
        }

        pcl::PCLPointCloud2 pcl_pc2;
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc1 (new pcl::PointCloud<pcl::PointXYZ>),
                                      pc2 (new pcl::PointCloud<pcl::PointXYZ>);

        void segment_pallet (const sensor_msgs::PointCloud2ConstPtr& input);
        
        pcl::PointCloud<pcl::PointXYZ> make_segmentation();
    private:
        int i;
        int nr_points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment (new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_segment_up (new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_segment_front (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients), coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliersZ (new pcl::PointIndices), inliers (new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>),
                                            cloudZ_p (new pcl::PointCloud<pcl::PointXYZ>), cloudZ_f (new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_front_filtred (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::SACSegmentation<pcl::PointXYZ> segZ;
        Eigen::Vector3f axis;

        pcl::ExtractIndices<pcl::PointXYZ> extract;

        void set_up_seg();
        void set_up_segZ();
        void set_up_viewer();
        
        pcl::PointCloud<pcl::PointXYZ> extract_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input);
        pcl::PointCloud<pcl::PointXYZ> extract_segZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input);
        pcl::PointCloud<pcl::PointXYZ> filter_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input, pcl::PointIndices::Ptr inliers);

}
