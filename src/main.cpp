#include "lio_sam/transformFusion.h"
#include "lio_sam/imuPreintegration.h"
#include "lio_sam/imageProjection.h"
#include "lio_sam/featureExtraction.h"
#include "lio_sam/mapOptimization.h"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    rclcpp::executors::MultiThreadedExecutor e;
    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    auto IP = std::make_shared<ImageProjection>(options);
    auto FE = std::make_shared<FeatureExtraction>(options);
    auto MO = std::make_shared<mapOptimization>(options);

    e.add_node(ImuP);
    e.add_node(TF);
    e.add_node(IP);
    e.add_node(FE);
    e.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> LIO-SAM Started.\033[0m");
    std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO);
    e.spin();

    rclcpp::shutdown();
    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
