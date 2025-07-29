/**
 * @file imageProjection.h
 * @brief imageProjection class, which handles the initial processing of point clouds.
 * 
 * The image projection step follows the imu preintegration step and is required by the feature
 * extraction step.
 * 
 * Subscribe to:
 *  - point cloud
 *  - imu data
 *  - odometry data
 * 
 * Main function:
 *  - get transformation guess
 *  - organize point cloud
 *  - deskew point cloud
 * 
 * Publish:
 *  - "lio_sam/deskew/cloud_info": `CloudInfo` message containing point cloud data
 */

#ifndef IMAGE_PROJECTION_H_
#define IMAGE_PROJECTION_H_

#include "lio_sam/utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int ringFlag = 0;
    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection(const rclcpp::NodeOptions & options);

    void allocateMemory();

    void resetParameters();

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg);

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg);

    bool deskewInfo();

    void imuDeskewInfo();

    void odomDeskewInfo();

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    PointType deskewPoint(PointType *point, double relTime);

    void projectPointCloud();

    void cloudExtraction();

    void interp(int& idn, const float& x_value, const float& min_x, const float& max_x, const float& max_idn);

    void publishClouds();
};

#endif  // IMAGE_PROJECTION_H_
