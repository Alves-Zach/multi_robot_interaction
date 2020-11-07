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
    interactionEffortCommandPublishers_ = std::vector<ros::Publisher>(numberOfRobots_);
    jointCommandMsgs_ = std::vector<sensor_msgs::JointState>(numberOfRobots_);
    interactionEffortCommandMsgs_ = std::vector<std_msgs::Float64MultiArray>(numberOfRobots_);

    // define the subscribers, publishers
    for(int i = 0; i< numberOfRobots_; i++){
        std::string jointStateSubscribersTopicName = "/" + nameSpaces_[i] + "/joint_states";
        jointStateSubscribers_[i] = nodeHandle_.subscribe<sensor_msgs::JointState>(jointStateSubscribersTopicName, 1,
                              boost::bind(&MultiRobotInteraction::jointStateCallback, this, _1, i));

        std::string jointCommandPublishersTopicName = "/" + nameSpaces_[i] + "/joint_commands";
        jointCommandPublishers_[i] = nodeHandle_.advertise<sensor_msgs::JointState>
                (jointCommandPublishersTopicName, 1);

        std::string interactionEffortCommandPublishersTopicName = "/" + nameSpaces_[i] + "/interaction_effort_commands";
        interactionEffortCommandPublishers_[i] = nodeHandle_.advertise<std_msgs::Float64MultiArray>
                (interactionEffortCommandPublishersTopicName, 1);
    }

    // set dynamic parameter server
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiRobotInteraction::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    startInteractionService_ = nodeHandle_.advertiseService("start_interaction", &MultiRobotInteraction::startExoServiceCallback, this);
    startInteractionFlag_ = false;

    // initializing joint and interaction matrixes
    jointPositionMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointPositionCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
}

void MultiRobotInteraction::advance() {
    if(!startInteractionFlag_) return;

    if(interaction_mode_ == 1){ // robot 2 mimics robot 1's position
        for(int dof = 0; dof<robotsDoF_; dof++){
            jointPositionCommandMatrix_(dof, 1) = jointPositionMatrix_(dof, 0);
        }
        publishJointCommands();
    }
    else if(interaction_mode_ == 2){ // virtual haptic spring

        for(int dof = 0; dof<robotsDoF_; dof++){
            interactionEffortCommandMatrix_(dof, 0) =
                    k_interaction_*(jointPositionMatrix_(dof, 1) - jointPositionMatrix_(dof, 0));
            interactionEffortCommandMatrix_(dof, 1) =
                    k_interaction_*(jointPositionMatrix_(dof, 0) - jointPositionMatrix_(dof, 1));
        }

        publishInteractionEffortCommand();
    }
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

void MultiRobotInteraction::dynReconfCallback(multi_robot_interaction::dynamic_paramsConfig &config, uint32_t level) {

    interaction_mode_ = config.interaction_mode;
    k_interaction_ = config.k_interaction;
    c_interaction_ = config.c_interaction;
    return;
}

void MultiRobotInteraction::publishJointCommands() {

    for(int robot = 0; robot<numberOfRobots_; robot++){
        jointCommandMsgs_[robot].position.resize(robotsDoF_);
        jointCommandMsgs_[robot].velocity.resize(robotsDoF_);
        jointCommandMsgs_[robot].effort.resize(robotsDoF_);
        for(int dof = 0; dof<robotsDoF_; dof++){
            jointCommandMsgs_[robot].position[dof] = jointPositionCommandMatrix_(dof, robot);
            jointCommandMsgs_[robot].velocity[dof] = jointVelocityCommandMatrix_(dof, robot);
            jointCommandMsgs_[robot].effort[dof] = jointTorqueCommandMatrix_(dof, robot);
        }
        jointCommandPublishers_[robot].publish(jointCommandMsgs_[robot]);
    }
}

void MultiRobotInteraction::publishInteractionEffortCommand() {

    for(int robot = 0; robot<numberOfRobots_; robot++){
        interactionEffortCommandMsgs_[robot].data.resize(robotsDoF_);
        for(int dof = 0; dof<robotsDoF_; dof++){
            interactionEffortCommandMsgs_[robot].data[dof] = interactionEffortCommandMatrix_(dof, robot);
        }
        interactionEffortCommandPublishers_[robot].publish(interactionEffortCommandMsgs_[robot]);
    }
}
