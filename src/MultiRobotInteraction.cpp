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

    // initializing joint and interaction matrixes
    jointPositionMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, 2);
    jointVelocityMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, 2);
    jointTorqueMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, 2);
    leftAnklePositionMatrix_ = Eigen::MatrixXd::Zero(2, 2);
    leftAnkleVelocityMatrix_ = Eigen::MatrixXd::Zero(2, 2);
    rightAnklePositionMatrix_ = Eigen::MatrixXd::Zero(2, 2);
    rightAnkleVelocityMatrix_ = Eigen::MatrixXd::Zero(2, 2);
    gaitStateVector_ = 4*Eigen::VectorXd::Ones(2); // set to 4(flying) initially
    linkLengthsMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_ - 1, 2);

    interactionEffortCommandMatrix_ = Eigen::MatrixXd::Zero(robotsDoF_, 2);

    // interaction parameters in joint space rendering
    k_joint_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    emg_joint_interaction_ = Eigen::VectorXd::Zero(5);
    c_joint_interaction_ = Eigen::VectorXd::Zero(robotsDoF_);
    joint_neutral_length_ = Eigen::VectorXd::Zero(robotsDoF_);

    // interaction parameters in task space rendering
    k_task_interaction_ = Eigen::VectorXd::Zero(2);
    c_task_interaction_ = Eigen::VectorXd::Zero(2);
    task_neutral_length_ = Eigen::VectorXd::Zero(2);
}

void MultiRobotInteraction::initialize() {
    // define the subscribers, publishers
    // Define the EXO subscriber
    exoRobotStateSubscriber = nodeHandle_.subscribe<CORC::X2RobotState>("/custom_robot_state", 1,
                                                            boost::bind(&MultiRobotInteraction::robotStateCallback,
                                                                        this, _1));

    // Define the interaction torque publisher
    interactionTorquePublisher = nodeHandle_.advertise<CORC::InteractionArray>("/desired_interaction_torque", 1);

    // Interaction effort message
    interactionEffortCommandMsg = CORC::InteractionArray();
    
    // Define a subscriber to the IMU data
    imuSubscriber_ = nodeHandle_.subscribe<sensor_msgs::JointState>("/imu_joint_states", 1,
                                                            boost::bind(&MultiRobotInteraction::imuCallback,
                                                                        this, _1));

    // Define a subscriber to the stiffness data
    stiffnessSubscriber_ = nodeHandle_.subscribe<std_msgs::Float32MultiArray>("/stiffness", 1,
                                                            boost::bind(&MultiRobotInteraction::stiffnessCallback,
                                                                        this, _1));

    // set dynamic parameter server
    dynamic_reconfigure::Server<multi_robot_interaction::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&MultiRobotInteraction::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    time0 = std::chrono::steady_clock::now(); // getting the time before program starts
    rosTime0_ = ros::Time::now();
}

void MultiRobotInteraction::advance() {

    // updateAnkleState();
    
    // interaction effort command matrix
    interactionEffortCommandMatrix_(0) = 0.0; // zero command to backpack
    for (int dof = 1; dof < robotsDoF_; dof++) {
        interactionEffortCommandMatrix_(dof, 1) =
                k_joint_interaction_(dof) * emg_joint_interaction_(dof) *
                (jointPositionMatrix_(dof, 1) - jointPositionMatrix_(dof, 0)) +
                c_joint_interaction_(dof) * (jointVelocityMatrix_(dof, 1) - jointVelocityMatrix_(dof, 0));
    }

    // Setting both of the interaction effort command matrix to be the same
    for (int dof = 0; dof < robotsDoF_; dof++){
        interactionEffortCommandMatrix_(dof, 0) = interactionEffortCommandMatrix_(dof, 1);
    }

    // ROS_INFO_STREAM("Interaction forces: " << interactionEffortCommandMatrix_(4, 1));

    // Print joint differences
    // ROS_INFO_STREAM("Joint differences: " << (jointPositionMatrix_(4, 1) - jointPositionMatrix_(4, 0)));

    publishInteractionEffortCommand();
}

void MultiRobotInteraction::exit() {

}

void MultiRobotInteraction::stiffnessCallback(const std_msgs::Float32MultiArrayConstPtr &msg) {
    if (useEMG_) {
        // Order of joints is leftHip, leftKnee, rightHip, rightKnee
        for (int dof = 0; dof < 4; dof++) {
            // Put the stiffness from the message into the emg interaction array
            // Value comes in from 0-100
            emg_joint_interaction_(dof+1) = msg->data[dof];

            // Change the range from 0-100 to 0.0-2.0
            emg_joint_interaction_(dof+1) *= 0.05;

            // Offset the value to start at 0.5 so multipliers range from 0.5-2.5
            emg_joint_interaction_(dof+1) += 0.5;
        }
    } else{
        for (int dof = 0; dof < 4; dof++) {
            emg_joint_interaction_(dof+1) = 1.0;
        }
    }
}

void MultiRobotInteraction::imuCallback(const sensor_msgs::JointStateConstPtr &msg) {
    // Order of joints is backpack, leftHip, leftKnee, rightHip, rightKnee
    // Store the imu data into the joint matricies
    for (int dof = 0; dof < robotsDoF_ - 1; dof++) {
        jointPositionMatrix_(dof+1, 0) = msg->position[dof];
        jointVelocityMatrix_(dof+1, 0) = msg->velocity[dof];
        jointTorqueMatrix_(dof+1, 0) = msg->effort[dof];
    }

    // If the state is mirrored, swap right and left joints
    if (mirrorInteraction_) {
        jointPositionMatrix_(1, 0) = msg->position[2];
        jointVelocityMatrix_(1, 0) = msg->velocity[2];
        jointTorqueMatrix_(1, 0) = msg->effort[2];

        jointPositionMatrix_(2, 0) = msg->position[3];
        jointVelocityMatrix_(2, 0) = msg->velocity[3];
        jointTorqueMatrix_(2, 0) = msg->effort[3];

        jointPositionMatrix_(3, 0) = msg->position[0];
        jointVelocityMatrix_(3, 0) = msg->velocity[0];
        jointTorqueMatrix_(3, 0) = msg->effort[0];

        jointPositionMatrix_(4, 0) = msg->position[1];
        jointVelocityMatrix_(4, 0) = msg->velocity[1];
        jointTorqueMatrix_(4, 0) = msg->effort[1];
    }
}

void MultiRobotInteraction::robotStateCallback(const CORC::X2RobotStateConstPtr &msg) {
    // access each robots state
    for(int dof = 0; dof< robotsDoF_ - 1; dof++){
        jointPositionMatrix_(dof+1, 1) = msg->joint_state.position[dof];
        jointVelocityMatrix_(dof+1, 1) = msg->joint_state.velocity[dof];
        jointTorqueMatrix_(dof+1, 1) = msg->joint_state.effort[dof];

        linkLengthsMatrix_(dof) = msg->link_lengths[dof];
    }

    jointPositionMatrix_(0, 1) = msg->joint_state.position[robotsDoF_ - 1] + M_PI_2;
    jointVelocityMatrix_(0, 1) = msg->joint_state.velocity[robotsDoF_ - 1];
    jointTorqueMatrix_(0, 1) = msg->joint_state.effort[robotsDoF_ - 1];

    gaitStateVector_(1) = msg->gait_state;
}

void MultiRobotInteraction::dynReconfCallback(multi_robot_interaction::dynamic_paramsConfig &config, uint32_t level) {

    if(interaction_mode_ != config.interaction_mode){
        ROS_INFO_STREAM("MODE IS CHANGED TO: "<< config.interaction_mode);
    }

    useEMG_ = config.use_EMG;

    interaction_mode_ = config.interaction_mode;

    // Set the mirrored boolean based on the interaction mode
    if(interaction_mode_ == 2){
        mirrorInteraction_ = true;
    } else {
        mirrorInteraction_ = false;
    }

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
    } else if(connectionMode == 3){ // Both
        k_joint_interaction_(1) = config.stiffness_both;
        c_joint_interaction_(1) = config.damping_both;

        k_joint_interaction_(2) = config.stiffness_both;
        c_joint_interaction_(2) = config.damping_both;
    }

    joint_neutral_length_(1) = deg2rad(config.neutral_length_hip);
    joint_neutral_length_(2) = deg2rad(config.neutral_length_knee);

    // k_task_interaction_(0) = config.stiffness_x;
    // c_task_interaction_(0) = config.damping_x;
    // task_neutral_length_(0) = config.neutral_length_x;
    // k_task_interaction_(1) = config.stiffness_y;
    // c_task_interaction_(1) = config.damping_y;
    // task_neutral_length_(1) = config.neutral_length_y;

    k_joint_interaction_(3) = k_joint_interaction_(1);
    c_joint_interaction_(3) = c_joint_interaction_(1);
    joint_neutral_length_(3) = joint_neutral_length_(1);
    k_joint_interaction_(4) = k_joint_interaction_(2);
    c_joint_interaction_(4) = c_joint_interaction_(2);
    joint_neutral_length_(4) = joint_neutral_length_(2);

    // wholeExoCommand_ = config.whole_exo;

    // desiredXPos_ = config.desired_x;
    // desiredYPos_ = config.desired_y;

    return;
}

void MultiRobotInteraction::publishInteractionEffortCommand() {

    double time = std::chrono::duration_cast<std::chrono::milliseconds>
                          (std::chrono::steady_clock::now() - time0).count() / 1000.0; // time in seconds

   // also publish the interaction properties so that it can be logged
   // TODO, probably better to implement logger on this node as well, instead of sending this to robots for them to log
    interactionEffortCommandMsg.custom_time = time;
    interactionEffortCommandMsg.interaction_mode = interaction_mode_;
    interactionEffortCommandMsg.desired_interaction.resize(robotsDoF_);
    interactionEffortCommandMsg.joint_stiffness.resize(robotsDoF_);
    interactionEffortCommandMsg.joint_damping.resize(robotsDoF_);
    interactionEffortCommandMsg.joint_neutral_angle.resize(robotsDoF_);
    interactionEffortCommandMsg.task_stiffness.resize(2);
    interactionEffortCommandMsg.task_damping.resize(2);
    interactionEffortCommandMsg.task_neutral_angle.resize(2);

    for(int dof = 0; dof<robotsDoF_; dof++){
        interactionEffortCommandMsg.desired_interaction[dof] = interactionEffortCommandMatrix_(dof, 1);
        interactionEffortCommandMsg.joint_stiffness[dof] = k_joint_interaction_(dof);
        interactionEffortCommandMsg.joint_damping[dof] = c_joint_interaction_(dof);
        interactionEffortCommandMsg.joint_neutral_angle[dof] = joint_neutral_length_(dof);
    }

    for(int axes = 0; axes<2; axes++){
        interactionEffortCommandMsg.task_stiffness[axes] = k_task_interaction_(axes);
        interactionEffortCommandMsg.task_damping[axes] = c_task_interaction_(axes);
        interactionEffortCommandMsg.task_neutral_angle[axes] = task_neutral_length_(axes);
    }
    interactionEffortCommandMsg.ankle_position.x = leftAnklePositionMatrix_(0); //x
    interactionEffortCommandMsg.ankle_position.y = leftAnklePositionMatrix_(1); //y

    interactionEffortCommandMsg.desired_ankle_position.x = desiredXPos_; //x
    interactionEffortCommandMsg.desired_ankle_position.y = desiredYPos_; //y

    interactionTorquePublisher.publish(interactionEffortCommandMsg);
}

void MultiRobotInteraction::updateAnkleState() {
    // Setting robot to always be 0
    int robot = 0;

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
