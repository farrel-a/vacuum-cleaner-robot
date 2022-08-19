#ifndef __VAC_BASE_CONTROLLER_HPP__
#define __VAC_BASE_CONTROLLER_HPP__

#include <vector>

namespace dgz {

class BaseController
{    
    public:
        struct Config {
            double sensor_threshold = 0.;
            std::vector<double> move_forward = {0., 0., 0.};
            std::vector<double> rotate_right = {0., 0., 0.};
            std::vector<double> rotate_left = {0., 0., 0.};
        };

        //constructor
        BaseController();

        //operator
        std::vector<double> operator()(std::vector<double> sensor);

        //getter & setter
        bool is_active() const {return is_active_;};
        void SetActive(bool is_active) {is_active_ = is_active;};
        void SetSensorThreshold(double sensor_threshold) {config_.sensor_threshold = sensor_threshold;};
        void SetMoveForward(std::vector<double> move_forward) {config_.move_forward = move_forward;};
        void SetRotateRight(std::vector<double> rotate_right) {config_.rotate_right = rotate_right;};
        void SetRotateLeft(std::vector<double> rotate_left) {config_.rotate_left = rotate_left;};
        void SetConfig(double sensor_threshold, std::vector<double> move_forward, std::vector<double> rotate_right, std::vector<double> rotate_left)
        {
            SetSensorThreshold(sensor_threshold);
            SetMoveForward(move_forward);
            SetRotateRight(rotate_right);
            SetRotateLeft(rotate_left);
        }
    
    private:
        bool is_active_ = false;
        Config config_;

};
}

#endif