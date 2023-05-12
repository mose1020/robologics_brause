#include <zimmer_gp406n/gripper.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace zimmer_gp406n
{
    Gripper::Gripper() : Node("zimmer_gp406n_node")
    {
        this->declare_parameter("robot_ip");
        this->declare_parameter("eki_io_port");
        this->declare_parameter("n_io");

        std::string robot_ip;
        this->get_parameter("robot_ip", robot_ip);
        int eki_io_port;
        std::string eki_io_port_;
        this->get_parameter("eki_io_port", eki_io_port);
        eki_io_port_ = std::to_string(eki_io_port);
        int n_io;
        this->get_parameter("n_io",n_io);

        _kuka_eki_io_interface.reset(new kuka_eki_io_interface::KukaEkiIOInterface(robot_ip.c_str(), eki_io_port_.c_str(), n_io));

        _open_gripper_srv = this->create_service<std_srvs::srv::Trigger>("open_gripper", std::bind(&Gripper::open_gripper, this, _1, _2));
        _close_gripper_srv = this->create_service<std_srvs::srv::Trigger>("close_gripper", std::bind(&Gripper::close_gripper, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("zimmer_gp406n_node"), "Ready to receive commands.");
    }

    void Gripper::open_gripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
    {
        rclcpp::Rate loop_rate(50);
        const std::vector<int> set_io_pins_cmd{8, 7};
        const std::vector<int> set_io_modes_cmd{2, 2,};
        const std::vector<bool> set_target_ios_cmd{ false, true};

        bool f_pin_set = false;
        bool s_pin_set = false;
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;
        bool command_received = false;
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        // TODO: add timeout
        while (!command_received)
        {
            _kuka_eki_io_interface->eki_write_command(set_io_pins_cmd, set_io_modes_cmd, set_target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == set_io_pins_cmd[0];
            s_pin_set = io_pins[1] == set_io_pins_cmd[1];
            f_command_received = io_states[0] == set_target_ios_cmd[0];
            s_command_received = io_states[1] == set_target_ios_cmd[1];
            f_mode_set = io_types[0] == set_io_modes_cmd[0];
            s_mode_set = io_types[1] == set_io_modes_cmd[1];
            command_received = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        bool opened = false;
        const std::vector<int> io_pins_cmd{7, 8};
        const std::vector<int> io_modes_cmd{1, 1};
        const std::vector<bool> target_ios_cmd{ false, false};
        // TODO: add timeout
        while (!opened)
        {
            _kuka_eki_io_interface->eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == io_pins_cmd[0];
            s_pin_set = io_pins[1] == io_pins_cmd[1];
            f_command_received = io_states[0] == true;
            s_command_received = io_states[1] == false;
            f_mode_set = io_types[0] == io_modes_cmd[0];
            s_mode_set = io_types[1] == io_modes_cmd[1];
            opened = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        response->success = true;
    }

    void Gripper::close_gripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        rclcpp::Rate loop_rate(50);
        const std::vector<int> set_io_pins_cmd{7, 8};
        const std::vector<int> set_io_modes_cmd{2, 2};
        const std::vector<bool> set_target_ios_cmd{ false, true };


        bool f_pin_set = false;
        bool s_pin_set = false;
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;
        bool command_received = false;
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        // TODO: add timeout
        while (!command_received)
        {
            _kuka_eki_io_interface->eki_write_command(set_io_pins_cmd, set_io_modes_cmd, set_target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == set_io_pins_cmd[0];
            s_pin_set = io_pins[1] == set_io_pins_cmd[1];
            f_command_received = io_states[0] == set_target_ios_cmd[0];
            s_command_received = io_states[1] == set_target_ios_cmd[1];
            f_mode_set = io_types[0] == set_io_modes_cmd[0];
            s_mode_set = io_types[1] == set_io_modes_cmd[1];
            command_received = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        bool closed = false;
        const std::vector<int> io_pins_cmd{7, 8};
        const std::vector<int> io_modes_cmd{1, 1};
        const std::vector<bool> target_ios_cmd{ false, false };
        // TODO: add timeout
        while (!closed)
        {
            _kuka_eki_io_interface->eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);
            loop_rate.sleep();
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            f_pin_set = io_pins[0] == io_pins_cmd[0];
            s_pin_set = io_pins[1] == io_pins_cmd[1];
            f_command_received = io_states[0] == false;
            s_command_received = io_states[1] == true;
            f_mode_set = io_types[0] == io_modes_cmd[0];
            s_mode_set = io_types[1] == io_modes_cmd[1];
            closed = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
        response->success = true;
    }
}