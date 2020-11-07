#include <multi_robot_interaction/MultiRobotInteraction.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

MultiRobotInteraction::MultiRobotInteraction(ros::NodeHandle &nodeHandle):
    nodeHandle_(nodeHandle)
{

}

void MultiRobotInteraction::initialize() {
    // gets the namespace parameters from the parameter server
    if(!nodeHandle_.getParam("name_spaces", nameSpaces_) || !nodeHandle_.getParam("dof", robotsDoF_)){
        ROS_ERROR("Failed to get parameter from server.");
        ROS_ERROR("Node is killed");
        ros::shutdown();
    }
    numberOfRobots_ = nameSpaces_.size();
    // sizing vectors
    jointStateSubscribers_ = std::vector<ros::Subscriber>(numberOfRobots_);
    jointCommandPublishers_ = std::vector<ros::Publisher>(numberOfRobots_);
    jointCommandMsgs_ = std::vector<sensor_msgs::JointState>(numberOfRobots_);
    interactionEffortCommandMsgs_ = std::vector<geometry_msgs::Wrench>(numberOfRobots_);

    // defines the subscribers and publishers
    for(int i = 0; i< numberOfRobots_; i++){
        std::string jointStateSubscribersTopicName = "/" + nameSpaces_[i] + "/joint_states";
        jointStateSubscribers_[i] = nodeHandle_.subscribe<sensor_msgs::JointState>(jointStateSubscribersTopicName, 1,
                              boost::bind(&MultiRobotInteraction::jointStateCallback, this, _1, i));

        std::string jointCommandPublishersTopicName = "/" + nameSpaces_[i] + "/joint_commands";
        jointCommandPublishers_[i] = nodeHandle_.advertise<sensor_msgs::JointState>
                (jointCommandPublishersTopicName, 1);

        std::string interactionEffortCommandPublishersTopicName = "/" + nameSpaces_[i] + "/interaction_effort_commands";
        interactionEffortCommandPublishers_[i] = nodeHandle_.advertise<geometry_msgs::Wrench>
                (interactionEffortCommandPublishersTopicName, 1);
    }

    startInteractionService_ = nodeHandle_.advertiseService("start_interaction", &MultiRobotInteraction::startExoServiceCallback, this);
    startInteractionFlag_ = false;

    jointPositionMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);

}

void MultiRobotInteraction::advance() {
    if(!startInteractionFlag_) return;



}

void MultiRobotInteraction::exit() {

}

void MultiRobotInteraction::jointStateCallback(const sensor_msgs::JointStateConstPtr & msg, int robot_id) {

    for(int dof = 0; dof< robotsDoF_; dof++){
        std::cout<<msg->position[dof]<<std::endl;
        jointPositionMatrix_(dof, robot_id) = msg->position[dof];
        jointVelocityMatrix_(dof, robot_id) = msg->velocity[dof];
        jointTorqueMatrix_(dof, robot_id) = msg->effort[dof];
    }
}

bool MultiRobotInteraction::startExoServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    startInteractionFlag_ = req.data;

    if(startInteractionFlag_){
        ROS_INFO("Interaction started");
        res.message = "Interaction started";
    } else{
        ROS_INFO("Interaction stopped");
        res.message = "Interaction stopped";
    }

    res.success = true;
    return true;
}