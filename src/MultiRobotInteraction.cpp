#include <multi_robot_interaction/MultiRobotInteraction.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

MultiRobotInteraction::MultiRobotInteraction(ros::NodeHandle &nodeHandle):
    nodeHandle_(nodeHandle)
{
    // gets the namespace parameters from the parameter server
    if(!nodeHandle_.getParam("name_spaces", nameSpaces_) || !nodeHandle_.getParam("dof", robotsDoF_)){
        ROS_ERROR("Failed to get parameter from server.");
        ROS_ERROR("Node is killed");
        ros::shutdown();
    }
    numberOfRobots_ = nameSpaces_.size();

    // initializing joint and interaction matrixes
    jointPositionMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointPositionCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);

    k_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    c_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    neutral_length_ = Eigen::VectorXd::Zero(robotsDoF_);

}

void MultiRobotInteraction::initialize() {

    // sizing vectors
    jointStateSubscribers_ = std::vector<ros::Subscriber>(numberOfRobots_);
    interactionEffortCommandPublishers_ = std::vector<ros::Publisher>(numberOfRobots_);
    interactionEffortCommandMsgs_ = std::vector<std_msgs::Float64MultiArray>(numberOfRobots_);

    // define the subscribers, publishers
    for(int i = 0; i< numberOfRobots_; i++){
        std::string jointStateSubscribersTopicName = "/" + nameSpaces_[i] + "/joint_states";
        jointStateSubscribers_[i] = nodeHandle_.subscribe<sensor_msgs::JointState>(jointStateSubscribersTopicName, 1,
                              boost::bind(&MultiRobotInteraction::jointStateCallback, this, _1, i));

        std::string interactionEffortCommandPublishersTopicName = "/" + nameSpaces_[i] + "/desired_interaction_torque";
        interactionEffortCommandPublishers_[i] = nodeHandle_.advertise<std_msgs::Float64MultiArray>
                (interactionEffortCommandPublishersTopicName, 1);
    }

    // set dynamic parameter server
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiRobotInteraction::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    startInteractionService_ = nodeHandle_.advertiseService("start_interaction", &MultiRobotInteraction::startExoServiceCallback, this);
    startInteractionFlag_ = false;

    time0 = std::chrono::steady_clock::now(); // getting the time before program starts
    rosTime0_ = ros::Time::now();

}

void MultiRobotInteraction::advance() {
    double time = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - time0).count()/1000.0; // time in seconds

    if(interaction_mode_ == 0){ // no interaction
        interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    }
    else if(interaction_mode_ == 1){ // bi-lateral dyad
        for(int dof = 0; dof<robotsDoF_; dof++){
            interactionEffortCommandMatrix_(dof, 0) =
                    k_interaction_(dof)*(jointPositionMatrix_(dof, 0) - jointPositionMatrix_(dof, 1) - neutral_length_(dof)) +
                    c_interaction_(dof)*(jointVelocityMatrix_(dof, 0) - jointVelocityMatrix_(dof, 1));
            interactionEffortCommandMatrix_(dof, 1) = -interactionEffortCommandMatrix_(dof, 0);
        }
    } else if(interaction_mode_ == 2){ // uni-lateral dyad
        for(int dof = 0; dof<robotsDoF_; dof++){
            interactionEffortCommandMatrix_(dof, 0) = 0.0;
            interactionEffortCommandMatrix_(dof, 1) =
                    k_interaction_(dof)*(jointPositionMatrix_(dof, 1) - jointPositionMatrix_(dof, 0) - neutral_length_(dof)) +
                    c_interaction_(dof)*(jointVelocityMatrix_(dof, 1) - jointVelocityMatrix_(dof, 0));
        }
    }
    publishInteractionEffortCommand();
}

void MultiRobotInteraction::exit() {

}

void MultiRobotInteraction::jointStateCallback(const sensor_msgs::JointStateConstPtr & msg, int robot_id) {

    for(int dof = 0; dof< robotsDoF_; dof++){
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

    if(interaction_mode_ != config.interaction_mode){
        ROS_INFO_STREAM("MODE IS CHANGED TO: "<< config.interaction_mode);
        time0 = std::chrono::steady_clock::now(); // reseting the time before switching to the new interaction mode
    }

    interaction_mode_ = config.interaction_mode;

    k_interaction_(0) = config.stiffness_hip;
    c_interaction_(0) = config.damping_hip;
    neutral_length_(0) = deg2rad(config.neutral_length_hip);
    k_interaction_(1) = config.stiffness_knee;
    c_interaction_(1) = config.damping_knee;
    neutral_length_(1) = deg2rad(config.neutral_length_knee);

    k_interaction_(2) = k_interaction_(0);
    c_interaction_(2) = c_interaction_(0);
    neutral_length_(2) = neutral_length_(0);
    k_interaction_(3) = k_interaction_(1);
    c_interaction_(3) = c_interaction_(1);
    neutral_length_(3) = neutral_length_(1);

    return;
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
