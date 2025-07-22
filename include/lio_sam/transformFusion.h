#ifndef TRANSFORM_FUSION_H_
#define TRANSFORM_FUSION_H_

#include "lio_sam/utility.hpp"


class TransformFusion : public ParamServer
{
public:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::CallbackGroup::SharedPtr callbackGroup;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Isometry3d lidarOdomIsometry;
    Eigen::Isometry3d imuOdomIsometryFront;
    Eigen::Isometry3d imuOdomIsometryBack;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2::Stamped<tf2::Transform> lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

public:
    TransformFusion(const rclcpp::NodeOptions & options);

    Eigen::Isometry3d odom2isometry(nav_msgs::msg::Odometry odom);

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    void publishOdomToBaseTf(tf2::Stamped<tf2::Transform> &tCur,
                             const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    void publishImuPath(const nav_msgs::msg::Odometry &imuOdometry);
};

#endif // TRANSFORM_FUSION_H_
