/**
 * /file MultiRobotInteraction.h
 * /author Emek Baris Kucuktabak
 * /brief Class to generate haptic interaction between robots
 * /version 2.1
 * /date 2022-12-17
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
#include <CORC/X2RobotState.h>
#include <CORC/InteractionArray.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <multi_robot_interaction/dynamic_paramsConfig.h>

#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

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

    int interaction_mode_;
    Eigen::VectorXd k_joint_interaction_; // stiffness of the interaction
    Eigen::VectorXd c_joint_interaction_; // damping constant of the interaction
    Eigen::VectorXd joint_neutral_length_; // neutral length of the spring

    Eigen::VectorXd k_task_interaction_; // stiffness of the interaction at task space
    Eigen::VectorXd c_task_interaction_; // damping constant of the interaction at task space
    Eigen::VectorXd task_neutral_length_; // neutral length of the spring at task space

    //vector of subscribers to subscribe joint states of the robots
    std::vector<ros::Subscriber> robotStateSubscribers_;

    //vectors of publishers and msgs to send joint position/velocity/torque & interaction effort commands
    std::vector<ros::Publisher> jointCommandPublishers_;
    std::vector<sensor_msgs::JointState> jointCommandMsgs_;
    std::vector<ros::Publisher> interactionEffortCommandPublishers_; // effort = either force or torque
    std::vector<CORC::InteractionArray> interactionEffortCommandMsgs_;

    // joint state subscriber callback
    void robotStateCallback(const CORC::X2RobotStateConstPtr &msg, int robot_id);

    // dynamic reconfigure server and callback
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig> server_;
    void dynReconfCallback(multi_robot_interaction::dynamic_paramsConfig &config, uint32_t level);

    // vector of namespaces that holds the namespace of the robots in concern
    std::vector<std::string> nameSpaces_;
    int robotsDoF_;
    int numberOfRobots_;

    Eigen::MatrixXd jointPositionMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd jointVelocityMatrix_;
    Eigen::MatrixXd jointTorqueMatrix_;

    Eigen::MatrixXd leftAnklePositionMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd leftAnkleVelocityMatrix_; // wrt fixed frame on the stance akle. Makes sense only for the swing leg.
    Eigen::MatrixXd rightAnklePositionMatrix_; // each column corresponds to different robots
    Eigen::MatrixXd rightAnkleVelocityMatrix_; // wrt fixed frame on the stance akle. Makes sense only for the swing leg.

    Eigen::VectorXd gaitStateVector_;

    std::vector<Eigen::MatrixXd> leftAnkleJacobianInRightStance_; // each vector element represents different robot
    std::vector<Eigen::MatrixXd> rightAnkleJacobianInLeftStance_;

    Eigen::MatrixXd linkLengthsMatrix_;  // each column corresponds to different robots

    Eigen::MatrixXd interactionEffortCommandMatrix_;

    void publishInteractionEffortCommand();
    void updateAnkleState();

    bool wholeExoCommand_; // if true commands to all joint will be given in task space control, else only swing leg.

    double stiffK_, softK_, stiffC_, softC_;

    double desiredXPos_, desiredYPos_;

    // IMU joint state subscriber
    ros::Subscriber imuSubscriber_;
    
    // IMU joint state callback
    void imuCallback(const sensor_msgs::JointStateConstPtr &msg);
};

#endif //SRC_LEGSCONTROLLER_H
