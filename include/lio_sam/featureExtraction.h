/**
 * @file featureExtraction.h
 * @brief FeatureExtraction class, which handles feature extraction from point clouds.
 * 
 * The feature extraction step follows the image projection step and is required by the map
 * optimization step.
 * 
 * Subscribe to:
 *  - "lio_sam/deskew/cloud_info": `CloudInfo` message containing point cloud data published from
 *    imageProjection.
 * 
 * Main function:
 *  - Extract edge and planar feature from a point cloud
 * 
 * Publish:
 *  - "lio_sam/feature/cloud_info": `CloudInfo` message containing new point cloud data
 *  - "lio_sam/feature/cloud_corner": `PointCloud2` message of the corner feature extracted
 *  - "lio_sam/feature/cloud_surface": `PointCloud2` message of the surface feature extracted
 */

#ifndef FEATURE_EXTRACTION_H_
#define FEATURE_EXTRACTION_H_

#include "lio_sam/utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"

struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{

public:

    rclcpp::Subscription<lio_sam::msg::CloudInfo>::SharedPtr subLaserCloudInfo;

    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    lio_sam::msg::CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction(const rclcpp::NodeOptions & options);

    void initializationValue();

    void laserCloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msgIn);

    void calculateSmoothness();

    void markOccludedPoints();

    void extractFeatures();

    void freeCloudInfoMemory();

    void publishFeatureCloud();
};

#endif  // FEATURE_EXTRACTION_H_
