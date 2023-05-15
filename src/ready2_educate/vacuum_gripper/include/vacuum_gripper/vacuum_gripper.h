#ifndef VACUUM_GRIPPER
#define VACUUM_GRIPPER

#include "rclcpp/rclcpp.hpp"
#include <kuka_eki_io_interface/kuka_eki_io_interface.h>
#include <std_srvs/srv/trigger.hpp>

namespace vacuum_gripper
{
    class Vacuum_Gripper : public rclcpp::Node
    {
        public:
            Vacuum_Gripper();
            ~Vacuum_Gripper() {};

        private:
            std::string robot_ip_ = "robot_ip";
            std::shared_ptr<kuka_eki_io_interface::KukaEkiIOInterface> _kuka_eki_io_interface;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _open_vacuum_gripper_srv;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _close_vacuum_gripper_srv;
            void open_vacuum_gripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            void close_vacuum_gripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    };
}
#endif