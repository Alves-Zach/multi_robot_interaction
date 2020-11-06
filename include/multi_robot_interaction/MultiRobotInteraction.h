/**
 * /file LegsController.h
 * /author Emek Baris Kucuktabak
 * /brief Controller class to control simplified human legs model
 * /version 0.1
 * /date 2020-10-19
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SRC_LEGSCONTROLLER_H
#define SRC_LEGSCONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <chrono>
#include <math.h>

#include "controller_manager_msgs/SwitchController.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#define NUM_JOINTS 4

class MultiRobotInteraction {
public:
    MultiRobotInteraction(ros::NodeHandle& nodeHandle);
    void initialize();
    void advance();
    void exit();

private:
    ros::NodeHandle& nodeHandle_;

};


#endif //SRC_LEGSCONTROLLER_H
