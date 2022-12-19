#include <multi_robot_interaction/MultiRobotInteraction.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

MultiRobotInteraction::MultiRobotInteraction(ros::NodeHandle &nodeHandle):
    nodeHandle_(nodeHandle)
{
    // gets the namespace parameters from the parameter server
    if(!nodeHandle_.getParam("name_spaces", nameSpaces_) || !nodeHandle_.getParam("dof", robotsDoF_) ||
       !nodeHandle_.getParam("soft_k", softK_) || !nodeHandle_.getParam("stiff_k", stiffK_)||
       !nodeHandle_.getParam("soft_c", softC_) || !nodeHandle_.getParam("stiff_c", stiffC_)){
        ROS_ERROR("Failed to get parameter from server.");
        ROS_ERROR("Node is killed");
        ros::shutdown();
    }
    numberOfRobots_ = nameSpaces_.size();

    // initializing joint and interaction matrixes
    jointPositionMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointVelocityMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    jointTorqueMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    leftAnklePositionMatrix_ = Eigen::MatrixXd::Zero(2, numberOfRobots_);
    leftAnkleVelocityMatrix_ = Eigen::MatrixXd::Zero(2, numberOfRobots_);
    rightAnklePositionMatrix_ = Eigen::MatrixXd::Zero(2, numberOfRobots_);
    rightAnkleVelocityMatrix_ = Eigen::MatrixXd::Zero(2, numberOfRobots_);
    gaitStateVector_ = 4*Eigen::VectorXd::Ones(numberOfRobots_); // set to 4(flying) initially
    linkLengthsMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_ - 1, numberOfRobots_);

    leftAnkleJacobianInRightStance_.resize(numberOfRobots_);
    rightAnkleJacobianInLeftStance_.resize(numberOfRobots_);

    interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);

    // interaction parameters in joint space rendering
    k_joint_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    c_joint_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    joint_neutral_length_ = Eigen::VectorXd::Zero(robotsDoF_);

    // interaction parameters in task space rendering
    k_task_interaction_ = Eigen::VectorXd::Zero(2);
    c_task_interaction_ = Eigen::VectorXd::Zero(2);
    task_neutral_length_ = Eigen::VectorXd::Zero(2);
}

void MultiRobotInteraction::initialize() {
    // sizing vectors
    robotStateSubscribers_ = std::vector<ros::Subscriber>(numberOfRobots_);
    interactionEffortCommandPublishers_ = std::vector<ros::Publisher>(numberOfRobots_);
    interactionEffortCommandMsgs_ = std::vector<CORC::InteractionArray>(numberOfRobots_);

    // define the subscribers, publishers
    for(int i = 0; i< numberOfRobots_; i++){
        std::string robotStateSubscribersTopicName = "/" + nameSpaces_[i] + "/custom_robot_state";
        robotStateSubscribers_[i] = nodeHandle_.subscribe<CORC::X2RobotState>(robotStateSubscribersTopicName,
                1,boost::bind(&MultiRobotInteraction::robotStateCallback, this, _1, i));

        std::string interactionEffortCommandPublishersTopicName = "/" + nameSpaces_[i] + "/desired_interaction_torque";
        interactionEffortCommandPublishers_[i] = nodeHandle_.advertise<CORC::InteractionArray>
                (interactionEffortCommandPublishersTopicName, 1);
    }

    // set dynamic parameter server
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiRobotInteraction::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    time0 = std::chrono::steady_clock::now(); // getting the time before program starts
    rosTime0_ = ros::Time::now();
}

void MultiRobotInteraction::advance() {

    if (interaction_mode_ == 0) { // no interaction
        interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    } else if (interaction_mode_ == 1) { // bi-directional joint space
        interactionEffortCommandMatrix_(0, 0) = 0.0; // zero command to backpack
        for (int dof = 1; dof < robotsDoF_; dof++) {
            interactionEffortCommandMatrix_(dof, 0) =
                    k_joint_interaction_(dof) *
                    (jointPositionMatrix_(dof, 0) - jointPositionMatrix_(dof, 1) - joint_neutral_length_(dof)) +
                    c_joint_interaction_(dof) * (jointVelocityMatrix_(dof, 0) - jointVelocityMatrix_(dof, 1));
            interactionEffortCommandMatrix_(dof, 1) = -interactionEffortCommandMatrix_(dof, 0);
        }
    } else if (interaction_mode_ == 2) { // uni-directional joint space
        interactionEffortCommandMatrix_(0, 0) = 0.0; // zero command to backpack
        for (int dof = 1; dof < robotsDoF_; dof++) {
            interactionEffortCommandMatrix_(dof, 1) =
                    k_joint_interaction_(dof) *
                    (jointPositionMatrix_(dof, 1) - jointPositionMatrix_(dof, 0) + joint_neutral_length_(dof)) +
                    c_joint_interaction_(dof) * (jointVelocityMatrix_(dof, 1) - jointVelocityMatrix_(dof, 0));
        }
    } else if (interaction_mode_ == 3 || interaction_mode_ == 4) { // bi/uni directional task space

        updateAnkleState();

        int stanceLeg = -1;
        bool feasibleConditions = true; // if not feasible no interaction will be rendered
        if ((gaitStateVector_(0) != gaitStateVector_(1))) {
            ROS_WARN("Task space interaction: Gait states are not same. Commanding zero interaction");
            feasibleConditions = false;
        } else { // same gait states
            stanceLeg = gaitStateVector_(0); // 1 left stance (right ankle control), 2 right stance (left ankle control)
            if (stanceLeg != 1 && stanceLeg != 2) {
                ROS_WARN("Task space interaction: Not in single stance. Commanding zero interaction");
                feasibleConditions = false;
            }

            Eigen::MatrixXd interestedAnklePositions(2, numberOfRobots_); // x and y directions for both robot for the ankle in interest
            Eigen::MatrixXd interestedAnkleVelocities(2, numberOfRobots_); // x and y directions for both robot for the ankle in interest

            if (stanceLeg == 2) { //right stance, left ankle control
                for (int robot = 0; robot < numberOfRobots_; robot++) {
                    interestedAnklePositions.col(robot) = leftAnklePositionMatrix_.col(robot);
                    interestedAnkleVelocities.col(robot) = leftAnkleVelocityMatrix_.col(robot);
                }
            } else if (stanceLeg == 1) { //left stance, right ankle control
                for (int robot = 0; robot < numberOfRobots_; robot++) {
                    interestedAnklePositions.col(robot) = rightAnklePositionMatrix_.col(robot);
                    interestedAnkleVelocities.col(robot) = rightAnkleVelocityMatrix_.col(robot);
                }
            }

            Eigen::VectorXd desiredAnkleForce(2); // x and y calculated for robot A

            for(int axes = 0; axes <2; axes++){ // x and y
                double k, c, th0;
                desiredAnkleForce(axes) =
                k_task_interaction_(axes)*(interestedAnklePositions(axes, 0) - interestedAnklePositions(axes, 1) - task_neutral_length_(axes)) +
                c_task_interaction_(axes)*(interestedAnkleVelocities(axes, 0) - interestedAnkleVelocities(axes, 1));
            }

            interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
            if (stanceLeg == 2) { //right stance, left ankle control

                if(wholeExoCommand_){
                    interactionEffortCommandMatrix_.col(0) = leftAnkleJacobianInRightStance_[0].transpose()*desiredAnkleForce; // robot A
                    interactionEffortCommandMatrix_.col(1) = leftAnkleJacobianInRightStance_[1].transpose()*-desiredAnkleForce; // robot B

                } else{
                    interactionEffortCommandMatrix_.block<2, 1>(1,0) = // robot A
                            leftAnkleJacobianInRightStance_[0].middleCols(1, 2).transpose()*desiredAnkleForce;

                    interactionEffortCommandMatrix_.block<2, 1>(1,1) = // robot B
                            leftAnkleJacobianInRightStance_[1].middleCols(1, 2).transpose()*-desiredAnkleForce;
                }

            } else if(stanceLeg == 1){ //left stance, right ankle control

                if(wholeExoCommand_){
                    interactionEffortCommandMatrix_.col(0) = rightAnkleJacobianInLeftStance_[0].transpose()*desiredAnkleForce; // robot A
                    interactionEffortCommandMatrix_.col(1) = rightAnkleJacobianInLeftStance_[1].transpose()*-desiredAnkleForce; // robot B

                } else{
                    interactionEffortCommandMatrix_.bottomLeftCorner(2, 1) = // robot A
                            rightAnkleJacobianInLeftStance_[0].rightCols(2).transpose()*desiredAnkleForce;

                    interactionEffortCommandMatrix_.bottomRightCorner(2, 1) = // robot A
                            rightAnkleJacobianInLeftStance_[1].rightCols(2).transpose()*-desiredAnkleForce;
                }
            }

            if(interaction_mode_ == 4){ // uni directional
                // zero interaction to A
                interactionEffortCommandMatrix_.col(0) = Eigen::VectorXd::Zero(robotsDoF_);
            }
        }
        // if not feasible give 0 force
        if (!feasibleConditions) interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, numberOfRobots_);
    }
    publishInteractionEffortCommand();
}

void MultiRobotInteraction::exit() {

}

void MultiRobotInteraction::robotStateCallback(const CORC::X2RobotStateConstPtr &msg, int robot_id) {
    // access each robots state
    for(int dof = 0; dof< robotsDoF_ - 1; dof++){
        jointPositionMatrix_(dof+1, robot_id) = msg->joint_state.position[dof];
        jointVelocityMatrix_(dof+1, robot_id) = msg->joint_state.velocity[dof];
        jointTorqueMatrix_(dof+1, robot_id) = msg->joint_state.effort[dof];

        linkLengthsMatrix_(dof, robot_id) = msg->link_lengths[dof];
    }

    jointPositionMatrix_(0, robot_id) = msg->joint_state.position[robotsDoF_ - 1] + M_PI_2;
    jointVelocityMatrix_(0, robot_id) = msg->joint_state.velocity[robotsDoF_ - 1];
    jointTorqueMatrix_(0, robot_id) = msg->joint_state.effort[robotsDoF_ - 1];

    gaitStateVector_(robot_id) = msg->gait_state;
}

void MultiRobotInteraction::dynReconfCallback(multi_robot_interaction::dynamic_paramsConfig &config, uint32_t level) {

    if(interaction_mode_ != config.interaction_mode){
        ROS_INFO_STREAM("MODE IS CHANGED TO: "<< config.interaction_mode);
    }

    interaction_mode_ = config.interaction_mode;

    int connectionMode = config.connection_mode;

    if(connectionMode == 0){ // use slider
        k_joint_interaction_(1) = config.stiffness_hip;
        c_joint_interaction_(1) = config.damping_hip;

        k_joint_interaction_(2) = config.stiffness_knee;
        c_joint_interaction_(2) = config.damping_knee;

    } else if(connectionMode == 1){ // soft
        k_joint_interaction_(1) = softK_;
        c_joint_interaction_(1) = softC_;

        k_joint_interaction_(2) = softK_;
        c_joint_interaction_(2) = softC_;
    } else if(connectionMode == 2){ // stiff
        k_joint_interaction_(1) = stiffK_;
        c_joint_interaction_(1) = stiffC_;

        k_joint_interaction_(2) = stiffK_;
        c_joint_interaction_(2) = stiffC_;
    }

    joint_neutral_length_(1) = deg2rad(config.neutral_length_hip);
    joint_neutral_length_(2) = deg2rad(config.neutral_length_knee);

    k_task_interaction_(0) = config.stiffness_x;
    c_task_interaction_(0) = config.damping_x;
    task_neutral_length_(0) = config.neutral_length_x;
    k_task_interaction_(1) = config.stiffness_y;
    c_task_interaction_(1) = config.damping_y;
    task_neutral_length_(1) = config.neutral_length_y;

    k_joint_interaction_(3) = k_joint_interaction_(1);
    c_joint_interaction_(3) = c_joint_interaction_(1);
    joint_neutral_length_(3) = joint_neutral_length_(1);
    k_joint_interaction_(4) = k_joint_interaction_(2);
    c_joint_interaction_(4) = c_joint_interaction_(2);
    joint_neutral_length_(4) = joint_neutral_length_(2);

    wholeExoCommand_ = config.whole_exo;

    desiredXPos_ = config.desired_x;
    desiredYPos_ = config.desired_y;

    return;
}

void MultiRobotInteraction::publishInteractionEffortCommand() {

    for(int robot = 0; robot<numberOfRobots_; robot++){
        double time = std::chrono::duration_cast<std::chrono::milliseconds>
                              (std::chrono::steady_clock::now() - time0).count() / 1000.0; // time in seconds

       // also publish the interaction properties so that it can be logged
       // TODO, probably better to implement logger on this node as well, instead of sending this to robots for them to log
        interactionEffortCommandMsgs_[robot].custom_time = time;
        interactionEffortCommandMsgs_[robot].interaction_mode = interaction_mode_;
        interactionEffortCommandMsgs_[robot].desired_interaction.resize(robotsDoF_);
        interactionEffortCommandMsgs_[robot].joint_stiffness.resize(robotsDoF_);
        interactionEffortCommandMsgs_[robot].joint_damping.resize(robotsDoF_);
        interactionEffortCommandMsgs_[robot].joint_neutral_angle.resize(robotsDoF_);
        interactionEffortCommandMsgs_[robot].task_stiffness.resize(2);
        interactionEffortCommandMsgs_[robot].task_damping.resize(2);
        interactionEffortCommandMsgs_[robot].task_neutral_angle.resize(2);

        for(int dof = 0; dof<robotsDoF_; dof++){
            interactionEffortCommandMsgs_[robot].desired_interaction[dof] = interactionEffortCommandMatrix_(dof, robot);
            interactionEffortCommandMsgs_[robot].joint_stiffness[dof] = k_joint_interaction_(dof);
            interactionEffortCommandMsgs_[robot].joint_damping[dof] = c_joint_interaction_(dof);
            interactionEffortCommandMsgs_[robot].joint_neutral_angle[dof] = joint_neutral_length_(dof);
        }

        for(int axes = 0; axes<2; axes++){
            interactionEffortCommandMsgs_[robot].task_stiffness[axes] = k_task_interaction_(axes);
            interactionEffortCommandMsgs_[robot].task_damping[axes] = c_task_interaction_(axes);
            interactionEffortCommandMsgs_[robot].task_neutral_angle[axes] = task_neutral_length_(axes);
        }
        interactionEffortCommandMsgs_[robot].ankle_position.x = leftAnklePositionMatrix_(0, robot); //x
        interactionEffortCommandMsgs_[robot].ankle_position.y = leftAnklePositionMatrix_(1, robot); //y

        interactionEffortCommandMsgs_[robot].desired_ankle_position.x = desiredXPos_; //x
        interactionEffortCommandMsgs_[robot].desired_ankle_position.y = desiredYPos_; //y

        interactionEffortCommandPublishers_[robot].publish(interactionEffortCommandMsgs_[robot]);
    }
}

void MultiRobotInteraction::updateAnkleState() {

    for(int robot = 0; robot<numberOfRobots_; robot++){

        double th_b = jointPositionMatrix_(0, robot);
        double th_1 = jointPositionMatrix_(1, robot); double th_2 = jointPositionMatrix_(2, robot);
        double th_3 = jointPositionMatrix_(3, robot); double th_4 = jointPositionMatrix_(4, robot);
        double l_2 = linkLengthsMatrix_(0, robot); double l_3 = linkLengthsMatrix_(1, robot);
        double l_4 = linkLengthsMatrix_(2, robot); double l_5 = linkLengthsMatrix_(3, robot);

        if(wholeExoCommand_){

         // these positions are wrt to the other fixed ankle. Makes sense only for the swing ankle
        leftAnklePositionMatrix_(0, robot) = //x
                l_4*cos(th_3 + th_b) - l_2*cos(th_1 + th_b) - l_3*cos(th_1 + th_2 + th_b) + l_5*cos(th_3 + th_4 + th_b);
        leftAnklePositionMatrix_(1, robot) = //y
                l_4*sin(th_3 + th_b) - l_2*sin(th_1 + th_b) - l_3*sin(th_1 + th_2 + th_b) + l_5*sin(th_3 + th_4 + th_b);

        rightAnklePositionMatrix_(0, robot) = //x
                l_2*cos(th_1 + th_b) - l_4*cos(th_3 + th_b) + l_3*cos(th_1 + th_2 + th_b) - l_5*cos(th_3 + th_4 + th_b);
        rightAnklePositionMatrix_(1, robot) = //y
                l_2*sin(th_1 + th_b) - l_4*sin(th_3 + th_b) + l_3*sin(th_1 + th_2 + th_b) - l_5*sin(th_3 + th_4 + th_b);

        } else {

            leftAnklePositionMatrix_(0, robot) = //x
                    -l_2 * cos(th_1 + th_b) - l_3 * cos(th_1 + th_2 + th_b);
            leftAnklePositionMatrix_(1, robot) = //y
                    -l_2 * sin(th_1 + th_b) - l_3 * sin(th_1 + th_2 + th_b);

            rightAnklePositionMatrix_(0, robot) = //x
                    -l_4 * cos(th_3 + th_b) - l_5 * cos(th_3 + th_4 + th_b);
            rightAnklePositionMatrix_(1, robot) = //y
                    -l_4 * sin(th_3 + th_b) - l_5 * sin(th_3 + th_4 + th_b);
        }

        Eigen::MatrixXd tempLeftJac(2, robotsDoF_);
        Eigen::MatrixXd tempRightJac(2, robotsDoF_);

        tempLeftJac(0, 0) = l_2*sin(th_1+th_b)-l_4*sin(th_3+th_b)+l_3*sin(th_1+th_2+th_b)-l_5*sin(th_3+th_4+th_b);
        tempLeftJac(0, 1) = l_2*sin(th_1+th_b)+l_3*sin(th_1+th_2+th_b);
        tempLeftJac(0, 2) = l_3*sin(th_1+th_2+th_b);
        tempLeftJac(0, 3) = -l_4*sin(th_3+th_b)-l_5*sin(th_3+th_4+th_b);
        tempLeftJac(0, 4) = -l_5*sin(th_3+th_4+th_b);
        tempLeftJac(1, 0) = -l_2*cos(th_1+th_b)+l_4*cos(th_3+th_b)-l_3*cos(th_1+th_2+th_b)+l_5*cos(th_3+th_4+th_b);
        tempLeftJac(1, 1) = -l_2*cos(th_1+th_b)-l_3*cos(th_1+th_2+th_b);
        tempLeftJac(1, 2) = -l_3*cos(th_1+th_2+th_b);
        tempLeftJac(1, 3) = l_4*cos(th_3+th_b)+l_5*cos(th_3+th_4+th_b);
        tempLeftJac(1, 4) = l_5*cos(th_3+th_4+th_b);

        tempRightJac(0, 0) = -l_2*sin(th_1+th_b)+l_4*sin(th_3+th_b)-l_3*sin(th_1+th_2+th_b)+l_5*sin(th_3+th_4+th_b);
        tempRightJac(0, 1) = -l_2*sin(th_1+th_b)-l_3*sin(th_1+th_2+th_b);
        tempRightJac(0, 2) = -l_3*sin(th_1+th_2+th_b);
        tempRightJac(0, 3) = l_4*sin(th_3+th_b)+l_5*sin(th_3+th_4+th_b);
        tempRightJac(0, 4) = l_5*sin(th_3+th_4+th_b);
        tempRightJac(1, 0) = l_2*cos(th_1+th_b)-l_4*cos(th_3+th_b)+l_3*cos(th_1+th_2+th_b)-l_5*cos(th_3+th_4+th_b);
        tempRightJac(1, 1) = l_2*cos(th_1+th_b)+l_3*cos(th_1+th_2+th_b);
        tempRightJac(1, 2) = l_3*cos(th_1+th_2+th_b);
        tempRightJac(1, 3) = -l_4*cos(th_3+th_b)-l_5*cos(th_3+th_4+th_b);
        tempRightJac(1, 4) = -l_5*cos(th_3+th_4+th_b);

        leftAnkleJacobianInRightStance_[robot] = tempLeftJac;
        rightAnkleJacobianInLeftStance_[robot] = tempRightJac;


        Eigen::VectorXd leftVel(2); Eigen::VectorXd rightVel(2);
        if(robot == 0){
            leftVel = jointVelocityMatrix_.block<2, 1>(1, 0);
            rightVel = jointVelocityMatrix_.bottomLeftCorner(2,1);
        } else if(robot == 1){
            leftVel = jointVelocityMatrix_.block<2, 1>(1, 1);
            rightVel = jointVelocityMatrix_.bottomRightCorner(2,1);
        }

        // all joints
        if(wholeExoCommand_) {
            leftAnkleVelocityMatrix_.col(robot) =
                    leftAnkleJacobianInRightStance_[robot] * jointVelocityMatrix_.col(robot);
            rightAnkleVelocityMatrix_.col(robot) =
                    rightAnkleJacobianInLeftStance_[robot] * jointVelocityMatrix_.col(robot);
        }else {

            // simplified only swing joints
            leftAnkleVelocityMatrix_.col(robot) = leftAnkleJacobianInRightStance_[robot].middleCols(1, 2) * leftVel;
            rightAnkleVelocityMatrix_.col(robot) = rightAnkleJacobianInLeftStance_[robot].rightCols(2) * rightVel;
        }
    }
}
