#include <iostream>
#include <ros/ros.h>
#include <spds_pallet_recognision/pallet_segmentation.hh>


PalletSegmentation::segment_pallet (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc1);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc2);
}

PalletSegmentation::set_up_seg()
{
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
}

PalletSegmentation::set_up_segZ()
{
    axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
    segZ.setAxis(axis);
    segZ.setEpsAngle(  PI - 2.5f ); // plane can be within 30 degrees of X-Y plane
    segZ.setOptimizeCoefficients (true);

    segZ.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    segZ.setMethodType (pcl::SAC_RANSAC);
    segZ.setMaxIterations (1000);
    segZ.setDistanceThreshold (0.01);
}

PalletSegmentation::extract_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input)
{
    i = 0;
    nr_points = (int) pc_input->size ();

    while (pc_input->size () > 0.3 * nr_points)
    {   
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (pc_input);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (pc_input);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        *cloud_segment_up += *cloud_p;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        pc_input.swap (cloud_f);
        i++;
    }
    return cloud_segment_up;
}

PalletSegmentation::extract_segZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input)
{
    i = 0;
    nr_points = (int) pc_input->size ();

    while (pc_input->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        segZ.setInputCloud (pc_input);
        segZ.segment (*inliersZ, *coefficientsZ);
        if (inliersZ->indices.size () == 0)
        {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the inliers
        extractZ.setInputCloud (pc_input);
        extractZ.setIndices (inliersZ);
        extractZ.setNegative (false);
        extractZ.filter (*cloudZ_p);
        std::cerr << "PointCloud representing the planar component: " << cloudZ_p->width * cloudZ_p->height << " data points." << std::endl;

        *cloud_segment += *cloudZ_p;

        // Create the filtering object
        extractZ.setNegative (true);
        extractZ.filter (*cloudZ_f);
        pc_input.swap (cloudZ_f);
        i++;
    }
    return cloud_segment;
}


pcl::PointCloud<pcl::PointXYZ> filter_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input, pcl::PointIndices::Ptr inliers)
{
    extract_front.setInputCloud (pc_input);
    extract_front.setIndices (inliers);
    extract_front.setNegative (true);
    extract_front.filter (*cloud_front_filtred);
    return cloud_front_filtred;
}


PalletSegmentation::make_segmentation()
{
    cloud_segment_up = extract_seg(pc1);
    cloud_front_filtred = filter_segment(pc2, inliers);
    cloud_segment = extract_segZ(cloud_front_filtred);
    cloud_segment_front = extract_seg(cloud_segment);

    return cloud_segment_front;

}