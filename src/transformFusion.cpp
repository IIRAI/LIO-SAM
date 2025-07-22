#include "lio_sam/transformFusion.h"


TransformFusion::TransformFusion(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_transformFusion", options)
{
    tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    callbackGroup = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto imuOdomOpt   = rclcpp::SubscriptionOptions();
    auto laserOdomOpt = rclcpp::SubscriptionOptions();
    imuOdomOpt.callback_group   = callbackGroup;
    laserOdomOpt.callback_group = callbackGroup;

    subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
        "lio_sam/mapping/odometry", qos,
        std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
        laserOdomOpt);
    subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(
        odomTopic+"_incremental", qos_imu,
        std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
        imuOdomOpt);

    pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
    pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);

    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

Eigen::Isometry3d TransformFusion::odom2isometry(nav_msgs::msg::Odometry odom)
{
    tf2::Transform t;
    tf2::fromMsg(odom.pose.pose, t);
    return tf2::transformToEigen(tf2::toMsg(t));
}

void TransformFusion::lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    lidarOdomIsometry = odom2isometry(*odomMsg);
    lidarOdomTime = stamp2Sec(odomMsg->header.stamp);
}

void TransformFusion::imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    imuOdomQueue.push_back(*odomMsg);

    // get latest odometry (at current IMU stamp)
    if (lidarOdomTime == -1)
        return;
    // remove IMU odometry messages older than or equal to the latest lidar odometry time
    while (!imuOdomQueue.empty() && stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
    {
        imuOdomQueue.pop_front();
    }

    Eigen::Isometry3d imuOdomIsometryFront = odom2isometry(imuOdomQueue.front());
    Eigen::Isometry3d imuOdomIsometryBack = odom2isometry(imuOdomQueue.back());
    Eigen::Isometry3d imuOdomIsometryIncre = imuOdomIsometryFront.inverse() * imuOdomIsometryBack;
    Eigen::Isometry3d imuOdomIsometryLast = lidarOdomIsometry * imuOdomIsometryIncre;
    auto t = tf2::eigenToTransform(imuOdomIsometryLast);
    tf2::Stamped<tf2::Transform> tCur;
    tf2::convert(t, tCur);

    // publish latest odometry
    nav_msgs::msg::Odometry imuOdometry = imuOdomQueue.back();
    imuOdometry.pose.pose.position.x = t.transform.translation.x;
    imuOdometry.pose.pose.position.y = t.transform.translation.y;
    imuOdometry.pose.pose.position.z = t.transform.translation.z;
    imuOdometry.pose.pose.orientation = t.transform.rotation;
    pubImuOdometry->publish(imuOdometry);

    if (publishOdomToBaseTF)
        publishOdomToBaseTf(tCur, odomMsg);

    publishImuPath(imuOdometry);
}

void TransformFusion::publishOdomToBaseTf(tf2::Stamped<tf2::Transform> &tCur,
                            const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    if(lidarFrame != baselinkFrame)
    {
        try
        {
            tf2::fromMsg(tfBuffer->lookupTransform(
                lidarFrame, baselinkFrame, rclcpp::Time(0)), lidar2Baselink);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(get_logger(), "%s", ex.what());
        }
        tf2::Stamped<tf2::Transform> tb(
            tCur * lidar2Baselink, tf2_ros::fromMsg(odomMsg->header.stamp), odometryFrame);
        tCur = tb;
    }
    geometry_msgs::msg::TransformStamped ts;
    tf2::convert(tCur, ts);
    ts.child_frame_id = baselinkFrame;
    tfBroadcaster->sendTransform(ts);
}

void TransformFusion::publishImuPath(const nav_msgs::msg::Odometry &imuOdometry)
{
    static nav_msgs::msg::Path imuPath;
    static double last_path_time = -1;
    double imuTime = stamp2Sec(imuOdometry.header.stamp);
    if (imuTime - last_path_time > 0.1)
    {
        last_path_time = imuTime;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = imuOdometry.header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose = imuOdometry.pose.pose;
        imuPath.poses.push_back(pose_stamped);
        while(!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
            imuPath.poses.erase(imuPath.poses.begin());
        if (pubImuPath->get_subscription_count() != 0)
        {
            imuPath.header.stamp = imuOdometry.header.stamp;
            imuPath.header.frame_id = odometryFrame;
            pubImuPath->publish(imuPath);
        }
    }
}
