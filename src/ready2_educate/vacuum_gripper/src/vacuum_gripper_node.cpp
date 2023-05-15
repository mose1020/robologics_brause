#include "rclcpp/rclcpp.hpp"
#include "vacuum_gripper/vacuum_gripper.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vacuum_gripper::Vacuum_Gripper>());
    rclcpp::shutdown();
    return 0;
}