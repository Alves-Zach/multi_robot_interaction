/**
 * /file MultiRobotInteraction.h
 * /author Emek Baris Kucuktabak
 * /brief Class to generate haptic interaction between robots
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

#include <std_srvs/SetBool.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>

class MultiRobotInteraction {
public:
    MultiRobotInteraction(ros::NodeHandle& nodeHandle);
    void initialize();
    void advance();
    void exit();

private:

    ros::NodeHandle& nodeHandle_;

    //vector of subscribers to subscribe joint states of the robots
    std::vector<ros::Subscriber> jointStateSubscribers_;

    //vectors of publishers and msgs to send joint position/velocity/torque & interaction effort commands
    std::vector<ros::Publisher> jointCommandPublishers_;
    std::vector<sensor_msgs::JointState> jointCommandMsgs_;
    std::vector<ros::Publisher> interactionEffortCommandPublishers_; // effort = either force or torque
    std::vector<geometry_msgs::Wrench> interactionEffortCommandMsgs_;

    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg, int robot_id);

    // service server to start or stop the interaction
    ros::ServiceServer startInteractionService_;
    bool startExoServiceCallback(std_srvs::SetBool::Request& req,
                                 std_srvs::SetBool::Response& res);
    bool startInteractionFlag_;

    // vector of namespaces that holds the namespace of the robots in concern
    std::vector<std::string> nameSpaces_;
    int robotsDoF_;
    int numberOfRobots_;

    Eigen::MatrixXd jointPositionMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd jointVelocityMatrix_;
    Eigen::MatrixXd jointTorqueMatrix_;

};


#endif //SRC_LEGSCONTROLLER_H
