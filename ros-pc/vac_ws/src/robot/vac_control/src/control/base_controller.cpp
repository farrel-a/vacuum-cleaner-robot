#include <vac_control/base_controller.hpp>

namespace dgz {

BaseController::BaseController(){};

std::vector<double> BaseController::operator()(std::vector<double> sensor)
{
    std::vector<double> motor_pwm = {0., 0., 0.};

    if (is_active_)
    {
        double sensor1 = sensor[0];
        double sensor2 = sensor[1];
        double sensor3 = sensor[2];

        if (sensor1 < config_.sensor_threshold)
        {
            motor_pwm[0] = config_.rotate_right[0];
            motor_pwm[1] = config_.rotate_right[1];
            motor_pwm[2] = config_.rotate_right[2];
        }
        else if (sensor3 < config_.sensor_threshold)
        {
            motor_pwm[0] = config_.rotate_left[0];
            motor_pwm[1] = config_.rotate_left[1];
            motor_pwm[2] = config_.rotate_left[2];
        }
        else if (sensor2 < config_.sensor_threshold)
        {
            motor_pwm[0] = config_.rotate_left[0];
            motor_pwm[1] = config_.rotate_left[1];
            motor_pwm[2] = config_.rotate_left[2];
        }
        else
        {
            motor_pwm[0] = config_.move_forward[0];
            motor_pwm[1] = config_.move_forward[1];
            motor_pwm[2] = config_.move_forward[2];
        }
    }

    return motor_pwm;
}
}