#include <ros/ros.h>
#include <vac_control/vac_control_node.hpp>
#include <csignal>
#include <vector>

namespace {
    double compute_period;
    vac_msgs::HardwareCommand hardware_command_msg;

    ros::Publisher hardware_command_pub;
    ros::Subscriber hardware_state_sub;
    ros::ServiceServer set_base_active_srv;

    std::vector<double> move_forward = {0.0, 0.0, 0.0};
    std::vector<double> rotate_right = {0.0, 0.0, 0.0};
    std::vector<double> rotate_left  = {0.0, 0.0, 0.0};
    std::vector<double> motor_pwm    = {0.0, 0.0, 0.0};
    std::vector<double> sensor = {0.0, 0.0, 0.0};
    double sensor_threshold;

    void MainTask(const ros::TimerEvent& event);
    void UpdateHardwareState(const vac_msgs::HardwareState& msg);
    void ShutdownHandler(int signum);
    bool SetBaseActive(vac_msgs::SetBool::Request &req, vac_msgs::SetBool::Response &res);
    vac_msgs::HardwareCommand VectorToMsg(std::vector<double> motor_pwm);
    
    //scott-meyers singleton for global object
    ros::NodeHandle& nh() {
        static ros::NodeHandle nh;
        return nh;
    };

    dgz::BaseController& ControlBase()
    {
        static dgz::BaseController ControlBase;
        return ControlBase;
    };
}

int main(int argc, char** argv)
{
    // ROS setup
    ros::init(argc, argv, "vac_control_node");

    // Node setup
    nh();

    //Base Controller Object
    ControlBase();

    // SIGINT setup
    signal(SIGINT, ShutdownHandler);

    // Compute period setup
    nh().getParam("/compute_period", compute_period);

    // Publisher setup
    hardware_command_pub = nh().advertise<vac_msgs::HardwareCommand>("/control/command/hardware", 1);
    
    // Subscriber setup
    hardware_state_sub = nh().subscribe("/arduino/state/hardware", 1, UpdateHardwareState);
    
    // Service setup
    set_base_active_srv = nh().advertiseService("/control/base/set_base_active", SetBaseActive);

    // Main Task setup
    ros::Timer timer = nh().createTimer(ros::Duration(compute_period/1000.), MainTask);

    ros::spin();
}

namespace {
    void MainTask(const ros::TimerEvent& event)
    {
        nh().getParam("/sensor_threshold", sensor_threshold);
        nh().getParam("/move_forward", move_forward);
        nh().getParam("/rotate_right", rotate_right);
        nh().getParam("/rotate_left", rotate_left);

        ControlBase().SetConfig(sensor_threshold, move_forward, rotate_right, rotate_left);
        
        motor_pwm = ControlBase()(sensor);
        
        hardware_command_msg = VectorToMsg(motor_pwm);

        hardware_command_pub.publish(hardware_command_msg);
    }

    void UpdateHardwareState(const vac_msgs::HardwareState& msg)
    {
        sensor[0] = msg.sensor1;
        sensor[1] = msg.sensor2;
        sensor[2] = msg.sensor3;
    }
    
    bool SetBaseActive(vac_msgs::SetBool::Request &req, vac_msgs::SetBool::Response &res)
    {
        ControlBase().SetActive(req.data);
        res.success = true;
        res.message = "success";
        return res.success;
    }

    vac_msgs::HardwareCommand VectorToMsg(std::vector<double> motor_pwm)
    {
        vac_msgs::HardwareCommand msg;
        msg.motor1 = motor_pwm[0];
        msg.motor2 = motor_pwm[1];
        msg.motor3 = motor_pwm[2];
        return msg;
    }

    void ShutdownHandler(int signum)
    {
        hardware_command_msg.motor1 = 0.;
        hardware_command_msg.motor2 = 0.;
        hardware_command_msg.motor3 = 0.;
        hardware_command_pub.publish(hardware_command_msg);
        ROS_INFO("Shutting down...");
        ros::shutdown();
    }
}