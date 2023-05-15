#include "rclcpp/rclcpp.hpp"
#include "zimmer_gp406n/gripper.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<zimmer_gp406n::Gripper>());
    rclcpp::shutdown();
    return 0;
}