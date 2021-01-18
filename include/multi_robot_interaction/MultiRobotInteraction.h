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

// fundamentals
#include <ros/ros.h>
#include <Eigen/Core>
#include <chrono>
#include <math.h>

// msg types
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <multi_robot_interaction/dynamic_paramsConfig.h>

class MultiRobotInteraction {
public:
    MultiRobotInteraction(ros::NodeHandle &nodeHandle);

    void initialize();
    void advance();
    void exit();
private:

    ros::NodeHandle &nodeHandle_;

    std::chrono::steady_clock::time_point time0; // time when program started
    ros::Time rosTime0_;

    bool startInteractionFlag_;
    int interaction_mode_;
    double k_interaction_; // stiffness of the interaction
    double c_interaction_; // damping constant of the interaction
    double A_desired_pos_; // Amplitude of the desired position
    double f_desired_pos; // frequency of the desired position

    //vector of subscribers to subscribe joint states of the robots
    std::vector<ros::Subscriber> jointStateSubscribers_;

    //vectors of publishers and msgs to send joint position/velocity/torque & interaction effort commands
    std::vector<ros::Publisher> jointCommandPublishers_;
    std::vector<sensor_msgs::JointState> jointCommandMsgs_;
    std::vector<ros::Publisher> interactionEffortCommandPublishers_; // effort = either force or torque
    std::vector<std_msgs::Float64MultiArray> interactionEffortCommandMsgs_;

    // joint state subscriber callback
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg, int robot_id);

    // dynamic reconfigure server and callback
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig> server_;
    void dynReconfCallback(multi_robot_interaction::dynamic_paramsConfig &config, uint32_t level);

    // service server to start or stop the interaction
    ros::ServiceServer startInteractionService_;
    bool startExoServiceCallback(std_srvs::SetBool::Request &req,
                                 std_srvs::SetBool::Response &res);

    // vector of namespaces that holds the namespace of the robots in concern
    std::vector<std::string> nameSpaces_;
    int robotsDoF_;
    int numberOfRobots_;

    Eigen::MatrixXd jointPositionMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd jointVelocityMatrix_;
    Eigen::MatrixXd jointTorqueMatrix_;

    Eigen::MatrixXd jointPositionCommandMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd jointVelocityCommandMatrix_;
    Eigen::MatrixXd jointTorqueCommandMatrix_;

    Eigen::MatrixXd interactionEffortCommandMatrix_;

    void publishJointCommands();
    void publishInteractionEffortCommand();
};

#endif //SRC_LEGSCONTROLLER_H
