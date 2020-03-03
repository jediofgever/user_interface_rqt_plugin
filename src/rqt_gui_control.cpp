/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 17:09:48
 * @modify date 2020-02-18 17:09:48
 * @desc [description]
 */
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <user_interface_rqt_plugin/rqt_gui_control.h>
#include <QStringList>

namespace user_interface_rqt_plugin {

/**
 * @brief Construct a new RQTGUIControlPlugin::RQTGUIControlPlugin object
 *
 */
RQTGUIControlPlugin::RQTGUIControlPlugin() : rqt_gui_cpp::Plugin(), widget_(0) {
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("RQTGUIControlPlugin");
}

/**
 * @brief Destroy the RQTGUIControlPlugin::RQTGUIControlPlugin object
 *
 */
RQTGUIControlPlugin::~RQTGUIControlPlugin() {}

/**
 * @brief initilize plugin
 *
 * @param context
 */
void RQTGUIControlPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
    static const std::string PLANNING_GROUP = "manipulator";

    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    ROS_INFO("INITIAZLIZED RQT CONTROL PLUGIN");
    listener_ = new tf::TransformListener();
    robot_controller_ptr_ = new RobotController();

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    // timers to update displays of Robot states and IO states
    timer = new QTimer(widget_);
    timer_io = new QTimer(widget_);

    // init pretty popup object pointer
    popUp = new PopUp();

    // timer is invoked each 10 ms
    timer->start(10);

    // MANUAL OPERATIONS
    // connect push buttons and chechk boxes of Mnaul operations to their functions
    // TO SEND ROBOT TO A TARGET POSE OR TARGET JOINT
    ui_.SendRobotToGoal->connect(ui_.SendRobotToGoal, SIGNAL(pressed()), this, SLOT(SendRobotToGoal()));
    ui_.SendRobotToJointGoal->connect(ui_.SendRobotToJointGoal, SIGNAL(pressed()), this, SLOT(SendRobotToJointGoal()));
    ui_.stop->connect(ui_.stop, SIGNAL(pressed()), this, SLOT(StopTrajManual()));
    ui_.pause->connect(ui_.pause, SIGNAL(pressed()), this, SLOT(PauseTrajManual()));
    timer->connect(timer, SIGNAL(timeout()), this, SLOT(UpdateRobotStatusDisplay()));
    ui_.load_tool_data->connect(ui_.load_tool_data, SIGNAL(stateChanged(int)), this, SLOT(LoadDataofTool()));
    ui_.load_joint_data->connect(ui_.load_joint_data, SIGNAL(stateChanged(int)), this, SLOT(LoadDataofJoints()));

    // ROBOT JOGGERS
    ui_.x_plus->connect(ui_.x_plus, SIGNAL(pressed()), this, SLOT(XPlus()));
    ui_.x_minus->connect(ui_.x_minus, SIGNAL(pressed()), this, SLOT(XMinus()));
    ui_.y_plus->connect(ui_.y_plus, SIGNAL(pressed()), this, SLOT(YPlus()));
    ui_.y_minus->connect(ui_.y_minus, SIGNAL(pressed()), this, SLOT(YMinus()));
    ui_.z_plus->connect(ui_.z_plus, SIGNAL(pressed()), this, SLOT(ZPlus()));
    ui_.z_minus->connect(ui_.z_minus, SIGNAL(pressed()), this, SLOT(ZMinus()));

    // to send robot to home or scanning pose
    ui_.home->connect(ui_.home, SIGNAL(pressed()), this, SLOT(Home()));
    ui_.to_scan_pose->connect(ui_.to_scan_pose, SIGNAL(pressed()), this, SLOT(ToScanPose()));

    // AUTO ROBOT OPERATIONS
    ////////////////////////////////////////////////////////////////////////////
    ui_.start_3d_detection->connect(ui_.start_3d_detection, SIGNAL(pressed()), this, SLOT(Start3DDetection()));
    ui_.start_pick_place->connect(ui_.start_pick_place, SIGNAL(pressed()), this, SLOT(StartPickPlace()));
    ui_.stop_2->connect(ui_.stop_2, SIGNAL(pressed()), this, SLOT(StopTrajAuto()));
    ui_.pause_2->connect(ui_.pause_2, SIGNAL(pressed()), this, SLOT(PauseTrajAuto()));
    ui_.restart->connect(ui_.restart, SIGNAL(pressed()), this, SLOT(restart()));
    ui_.io_on->connect(ui_.io_on, SIGNAL(pressed()), this, SLOT(setIOON()));
    ui_.io_off->connect(ui_.io_off, SIGNAL(pressed()), this, SLOT(setIOFF()));
    ui_.reset_all_io->connect(ui_.reset_all_io, SIGNAL(pressed()), this, SLOT(resetAllIO()));
    ui_.continue_2->connect(ui_.continue_2, SIGNAL(pressed()), this, SLOT(continu()));

    // Robot IO states will be updated every other 500 ms
    timer_io->start(500);
    if (youAreRunningonRealRobot) {
        timer_io->connect(timer_io, SIGNAL(timeout()), this, SLOT(UpdateRobotIOStatusDisplay()));
    }
}

/**
 * @brief shut down plugin
 *
 */
void RQTGUIControlPlugin::shutdownPlugin() {
    // TODO unregister all publishers here
}

/**
 * @brief Read the spin box values for target pose from X,Y,Z, RX,RY,RZ, AND EXECUTE ROBOT MOVEMENT TO THAT GIVEN TARGET
 * POSE USING WORLD METHOD OR JOINT METHOD
 *
 */
void RQTGUIControlPlugin::SendRobotToGoal() {
    int method = ui_.RobotMoveMethod->currentIndex();

    // WORLD METHOD
    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        // Recieve Updated pose information for end-effector o n callbac
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position.x = ui_.goal_x->value() * kMM2M;
        goal_pose.pose.position.y = ui_.goal_y->value() * kMM2M;
        goal_pose.pose.position.z = (ui_.goal_z->value() + 330.0) * kMM2M;
        geometry_msgs::Quaternion rot =
            EulertoQuaternion(ui_.goal_rx->value(), ui_.goal_ry->value(), ui_.goal_rz->value(), kDEG2RAD);
        goal_pose.pose.orientation = rot;
        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(goal_pose, move_group_ptr_);

        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    // JOINT METHOD
    if (method == 1) {
        ROS_INFO("SEND TO POSE USING JOINT SPACE COMMAND RECIEVED.. , EXECUTING");

        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position.x = ui_.goal_x->value() * kMM2M;
        goal_pose.pose.position.y = ui_.goal_y->value() * kMM2M;
        goal_pose.pose.position.z = (ui_.goal_z->value() + 330.0) * kMM2M;
        geometry_msgs::Quaternion rot =
            EulertoQuaternion(ui_.goal_rx->value(), ui_.goal_ry->value(), ui_.goal_rz->value(), kDEG2RAD);
        goal_pose.pose.orientation = rot;
        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinJointSpace(goal_pose, move_group_ptr_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL INN JOINT SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief Sends robot to given joint target values, joint target values are read from GUI
 *
 */
void RQTGUIControlPlugin::SendRobotToJointGoal() {
    ROS_INFO("SEND TO JOINT COMMAND RECIEVED.. , EXECUTING");
    std::vector<double> robot_joint_states(6);
    robot_joint_states.at(0) = ui_.joint_1->value() * kDEG2RAD;
    robot_joint_states.at(1) = ui_.joint_2->value() * kDEG2RAD;
    robot_joint_states.at(2) = ui_.joint_3->value() * kDEG2RAD;
    robot_joint_states.at(3) = ui_.joint_4->value() * kDEG2RAD;
    robot_joint_states.at(4) = ui_.joint_5->value() * kDEG2RAD;
    robot_joint_states.at(5) = ui_.joint_6->value() * kDEG2RAD;

    bool suc = robot_controller_ptr_->moveJointstoTargetPositions(robot_joint_states, move_group_ptr_);
    if (!suc) {
        popUp->setPopupText(
            "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
            "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
        popUp->show();

    } else {
        popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL, PROCESSING NOW.... ");
        popUp->show();
    }
}

/**
 * @brief Deprectaed for now
 *
 */
void RQTGUIControlPlugin::SendRobotToGoalTool() {
    ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
}

/**
 * @brief Pauses any robot movement if any
 *
 */
void RQTGUIControlPlugin::PauseTrajAuto() {
    popUp->setPopupText("PAUSSING ROBOT COMMAND RECEIVED, PROCESSING NOW.... ");
    popUp->show();
    ROS_INFO("PAUSSING ROBOT");
    robot_controller_ptr_->pauseRobotTrajectrory(move_group_ptr_);
}

/**
 * @brief Stopos any robot movement if any, shut downs pick and place ROS node, ROS node for pick and place needs to be
 * started again if stop button was pressed
 *
 */
void RQTGUIControlPlugin::StopTrajAuto() {
    popUp->setPopupText("STOPPING ROBOT COMMAND RECEIVED, PROCESSING NOW.... ");
    popUp->show();
    ROS_INFO("STOPPING ROBOT");
    robot_controller_ptr_->stopRobotTrajectrory(move_group_ptr_);
    system("rosnode kill real_pickplace_ros_node");
}

/**
 * @brief Gets latest robot tcp pose and joint angles and displays them on menu
 *
 */
void RQTGUIControlPlugin::UpdateRobotStatusDisplay() {
    geometry_msgs::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion current_orientation_quat;
    tf::quaternionMsgToTF(current_pose.orientation, current_orientation_quat);
    std::vector<double> current_joint_states = move_group_ptr_->getCurrentJointValues();

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(current_orientation_quat).getRPY(roll, pitch, yaw);
    ui_.tcp_x_7->display(round(current_pose.position.x / kMM2M));
    ui_.tcp_x_8->display(round(current_pose.position.y / kMM2M));
    ui_.tcp_x_9->display(round(current_pose.position.z / kMM2M - 330));
    ui_.tcp_x_10->display(round(roll / kDEG2RAD));
    ui_.tcp_x_11->display(round(pitch / kDEG2RAD));
    ui_.tcp_x_12->display(round(yaw / kDEG2RAD));

    ui_.j1->display(round(current_joint_states[0] / kDEG2RAD));
    ui_.j2->display(round(current_joint_states[1] / kDEG2RAD));
    ui_.j3->display(round(current_joint_states[2] / kDEG2RAD));
    ui_.j4->display(round(current_joint_states[3] / kDEG2RAD));
    ui_.j5->display(round(current_joint_states[4] / kDEG2RAD));
    ui_.j6->display(round(current_joint_states[5] / kDEG2RAD));
}

/**
 * @brief Recieves Robot IO states from Robot Contoller and displays them on Robot IO section of GUI
 *
 */
void RQTGUIControlPlugin::UpdateRobotIOStatusDisplay() {
    std::vector<std::string> current_values =
        robot_io_controller_.readAllIOs("192.168.0.1", "read", robot_io_port_types_.DOUT, 10);

    ui_.io_1->display(current_values[0].c_str());
    ui_.io_2->display(current_values[1].c_str());
    ui_.io_3->display(current_values[2].c_str());
    ui_.io_4->display(current_values[3].c_str());
    ui_.io_5->display(current_values[4].c_str());
    ui_.io_6->display(current_values[5].c_str());
    ui_.io_7->display(current_values[6].c_str());
    ui_.io_8->display(current_values[7].c_str());
    ui_.io_9->display(current_values[8].c_str());
    /* ui_.io_10->display(current_values[9].c_str());
     ui_.io_11->display(current_values[10].c_str());
     ui_.io_12->display(current_values[11].c_str());
     ui_.io_13->display(current_values[12].c_str());
     ui_.io_14->display(current_values[13].c_str());
     ui_.io_15->display(current_values[14].c_str());
     ui_.io_16->display(current_values[15].c_str());
     ui_.io_17->display(current_values[16].c_str());
     ui_.io_18->display(current_values[17].c_str());
     ui_.io_19->display(current_values[18].c_str());
     ui_.io_20->display(current_values[19].c_str());*/
}

/**
 * @brief   sets IO port which was selected from drop box menu of GUI to   OFF
 *
 */
void user_interface_rqt_plugin::RQTGUIControlPlugin::setIOFF() {
    int io_id_to_turn_off = ui_.comboBox->currentIndex() + 1;
    robot_io_controller_.writeSingleIO("192.168.0.1", "write", robot_io_port_types_.DOUT, io_id_to_turn_off, 0);
}

/**
 * @brief   sets IO port which was selected from drop box menu of GUI to   ON
 *
 */
void user_interface_rqt_plugin::RQTGUIControlPlugin::setIOON() {
    int io_id_to_turn_on = ui_.comboBox->currentIndex() + 1;
    robot_io_controller_.writeSingleIO("192.168.0.1", "write", robot_io_port_types_.DOUT, io_id_to_turn_on, 1);
}

/**
 * @brief Resets All IO values to 0 (OFF)
 *
 */
void user_interface_rqt_plugin::RQTGUIControlPlugin::resetAllIO() {
    for (int i = 1; i < 20; i++) {
        robot_io_controller_.writeSingleIO("192.168.0.1", "write", robot_io_port_types_.DOUT, i, 0);
    }
}

/**
 * @brief Load current pose of robot tcp to spinboxes
 *
 */
void RQTGUIControlPlugin::LoadDataofTool() {
    popUp->setPopupText("Loading Tool States, PROCESSING NOW.... ");
    popUp->show();
    geometry_msgs::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion current_orientation_quat;
    tf::quaternionMsgToTF(current_pose.orientation, current_orientation_quat);
    std::vector<double> current_joint_states = move_group_ptr_->getCurrentJointValues();

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(current_orientation_quat).getRPY(roll, pitch, yaw);
    ui_.goal_x->setValue(current_pose.position.x / kMM2M);
    ui_.goal_y->setValue(current_pose.position.y / kMM2M);
    ui_.goal_z->setValue(current_pose.position.z / kMM2M - 330);
    ui_.goal_rx->setValue(roll / kDEG2RAD);
    ui_.goal_ry->setValue(pitch / kDEG2RAD);
    ui_.goal_rz->setValue(yaw / kDEG2RAD);
}

/**
 * @brief Load current joint angle values of robot to spinboxes
 *
 */
void RQTGUIControlPlugin::LoadDataofJoints() {
    popUp->setPopupText("Loading Joint States, PROCESSING NOW.... ");
    popUp->show();
    std::vector<double> current_joint_states = move_group_ptr_->getCurrentJointValues();

    ui_.joint_1->setValue(current_joint_states[0] / kDEG2RAD);
    ui_.joint_2->setValue(current_joint_states[1] / kDEG2RAD);
    ui_.joint_3->setValue(current_joint_states[2] / kDEG2RAD);
    ui_.joint_4->setValue(current_joint_states[3] / kDEG2RAD);
    ui_.joint_5->setValue(current_joint_states[4] / kDEG2RAD);
    ui_.joint_6->setValue(current_joint_states[5] / kDEG2RAD);
}

/**
 * @brief continues to paused trejctory
 *
 */
void RQTGUIControlPlugin::continu() {
    ROS_INFO("CONTINUE ROBOT PATH ");
    robot_controller_ptr_->continueRobotTrajectrory(move_group_ptr_, listener_);
}

/**
 * @brief Pauses any trajectory being executed in manual mode
 *
 */
void RQTGUIControlPlugin::PauseTrajManual() {
    ROS_INFO("PAUSING ROBOT");
    robot_controller_ptr_->pauseRobotTrajectrory(move_group_ptr_);
    popUp->setPopupText("PAUSE ROBOT COMMAND RECEIVED, PROCESSING NOW.... ");
    popUp->show();
}

/**
 * @brief STOPS any trajectory being executed in manual mode
 *
 */
void RQTGUIControlPlugin::StopTrajManual() {
    ROS_INFO("STOPPING ROBOT");
    robot_controller_ptr_->stopRobotTrajectrory(move_group_ptr_);

    popUp->setPopupText("PAUSE ROBOT COMMAND RECEIVED, PROCESSING NOW....");
    popUp->show();
}

/**
 * @brief INitiliazes python ROS node to do maskrcnn inference on raw rgb image
 *
 */
void RQTGUIControlPlugin::Start3DDetection() {
    popUp->setPopupText(
        "STARTING 3D DETECTION NOW, IT MAY TAKE UP TO 10 SECONDS ,PLEASE  CLICK ONCE AND  WAIT,  PROCESSING NOW.... ");
    popUp->show();
    system("~/catkin_build_ws/src/ROS_NNs_FANUC_LRMATE200ID/start_maskrcnn.sh");
}

/**
 * @brief INitiliazes   ROS node to do auto pick and place
 *
 */
void RQTGUIControlPlugin::StartPickPlace() {
    popUp->setPopupText(
        "STARTING PIVK AND PLACE, IT MAY TAKE UP TO 10 SECONDS ,PLEASE  CLICK ONCE AND  WAIT,  PROCESSING NOW.... ");
    popUp->show();
    system("~/catkin_ws/src/PICK_PLACE_with_ROS_on_FANUC_ARM/arm_perception_utilities/launch/start_pickplace.sh");
}

/**
 * @brief kills pick and place node and restarts it again
 *
 */
void RQTGUIControlPlugin::restart() {
    popUp->setPopupText("GONNA KILL PICK AND PLACE NODE ");
    popUp->show();
    system("rosnode kill real_pickplace_ros_node");
    system("~/catkin_ws/src/PICK_PLACE_with_ROS_on_FANUC_ARM/arm_perception_utilities/launch/pickplace.sh");
}

/**
 * @brief NOT USED
 *
 * @param plugin_settings
 * @param instance_settings
 */
void RQTGUIControlPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                       qt_gui_cpp::Settings& instance_settings) const {}

/**
 * @brief NOT USED
 *
 * @param plugin_settings
 * @param instance_settings
 */
void RQTGUIControlPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                          const qt_gui_cpp::Settings& instance_settings) {}

/**
 * @brief converts Euler angles to  compact geometry_msgs::Quaternion
 *
 * @param robot_rx_deg
 * @param robot_ry_deg
 * @param robot_rz_deg
 * @param kDEG2RAD
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion RQTGUIControlPlugin::EulertoQuaternion(double robot_rx_deg, double robot_ry_deg,
                                                                 double robot_rz_deg, double kDEG2RAD) {
    tf2::Quaternion robot_goal_orientation_quat;
    robot_goal_orientation_quat.setRPY(
        robot_rx_deg * kDEG2RAD, robot_ry_deg * kDEG2RAD,
        robot_rz_deg * kDEG2RAD);  // Create this quaternion from roll/pitch/yaw (in radians)

    // Normalize the QUATs to make the squarred root sum of x,y,z,w equal to 1.0
    robot_goal_orientation_quat.normalize();

    // Robot Pose should be set in terms of ROS conventations geotmetry_msgs::Pose
    geometry_msgs::Quaternion robot_goal_orientation_geo_msg;

    // tf::Quaternions to geotmetry_msgs::Quaternion
    robot_goal_orientation_geo_msg = tf2::toMsg(robot_goal_orientation_quat);

    return robot_goal_orientation_geo_msg;
}

/*******************  THIS FUNCTIONS ARE VERY SIMILAR TO EACH OTHER SO THERE IS NO POINT TO COMMENTS EACH OF THEM READ
 * FOLLOWING TO UNDERSTAND THEIR LOGIC   *********************************/

// Robot JOGGER HAS TWO METHODS,  YOU CAN JOG ROBOT IN WORLD FRAME OR ROBOT TOOL FRAME, THIS OPTIONS ARE PROVIDED TO
// USER IN GUI, AFTER THE METHOD FOR JOGGING WAS SELECTED, AMOUNT TRAVEL SHOULD BE ENETERED INTO SPINBOX, AFTER THAT
// ROBOT CAN BE JOGGED ALONG X,Y, Z AXES

/**
 * @brief JOG ROBOT ALONG X IN POSTIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::XPlus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.x += (value * kMM2M);

        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);

        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = (value * kMM2M);
        current_pose_stamped.pose.position.y = 0;
        current_pose_stamped.pose.position.z = 0;

        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief JOG ROBOT ALONG X IN nEGATIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::XMinus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.x -= (value * kMM2M);
        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = (-value * kMM2M);
        current_pose_stamped.pose.position.y = 0;
        current_pose_stamped.pose.position.z = 0;
        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief JOG ROBOT ALONG Y IN POSTIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::YPlus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.y += (value * kMM2M);

        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);

        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = 0;
        current_pose_stamped.pose.position.y = (value * kMM2M);
        current_pose_stamped.pose.position.z = 0;

        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief JOG ROBOT ALONG Y IN NEGATIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::YMinus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.y -= (value * kMM2M);
        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = 0;
        current_pose_stamped.pose.position.y = (-value * kMM2M);
        current_pose_stamped.pose.position.z = 0;
        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief JOG ROBOT ALONG Z IN POSTIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::ZPlus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.z += (value * kMM2M);

        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);

        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = 0;
        current_pose_stamped.pose.position.y = 0;
        current_pose_stamped.pose.position.z = (value * kMM2M);

        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief JOG ROBOT ALONG Z IN NEGATIVE DIRECTION
 *
 */
void RQTGUIControlPlugin::ZMinus() {
    int method = ui_.RobotJogMethod->currentIndex();
    int value = ui_.jog_value->value();
    geometry_msgs::PoseStamped current_pose_stamped = move_group_ptr_->getCurrentPose();

    if (method == 0) {
        ROS_INFO("SEND TO POSE USING CARTESIAN SPACE COMMAND RECIEVED.. , EXECUTING");

        current_pose_stamped.pose.position.z -= (value * kMM2M);
        bool suc = robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(current_pose_stamped, move_group_ptr_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL IN CARTESIAN SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }

    if (method == 1) {
        ROS_INFO("SEND TO GOAL IN TOOL FRAME COMMAND RECIEVED.. , EXECUTING");
        current_pose_stamped.pose.position.x = 0;
        current_pose_stamped.pose.position.y = 0;
        current_pose_stamped.pose.position.z = (-value * kMM2M);
        bool suc =
            robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(current_pose_stamped, move_group_ptr_, listener_);
        if (!suc) {
            popUp->setPopupText(
                "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
                "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
            popUp->show();
        } else {
            popUp->setPopupText("SENDING ROBOT TO GIVEN GOAL IN TOOL SPACE, PROCESSING NOW.... ");
            popUp->show();
        }
    }
}

/**
 * @brief mOVES ROBOT TO HOME POSTION , WHERE ALL JOINT ANGLES ARE ZERO
 *
 */
void RQTGUIControlPlugin::Home() {
    std::vector<double> zeros = {0, 0, 0, 0, 0, 0};

    ROS_INFO("SEND TO JOINT COMMAND RECIEVED.. , EXECUTING");

    bool suc = robot_controller_ptr_->moveJointstoTargetPositions(zeros, move_group_ptr_);
    if (!suc) {
        popUp->setPopupText(
            "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
            "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
        popUp->show();
    } else {
        popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL, PROCESSING NOW.... ");
        popUp->show();
    }
}

/**
 * @brief mOVES ROBOT TO HOME POSTION , WHERE ALL JOINT ANGLES ARE ZERO
 *
 */
void RQTGUIControlPlugin::ToScanPose() {
    ROS_INFO("SEND TO  scan POSE USING JOINT SPACE COMMAND RECIEVED.. , EXECUTING");

    // Recieve Updated pose information for end-effector o n callback
    std::vector<double> zeros = {0, 0, 0, 0, 0, 0};

    bool suc = robot_controller_ptr_->moveJointstoTargetPositions(zeros, move_group_ptr_);

    if (!suc) {
        popUp->setPopupText(
            "ERROR !!!, THE PLANNER CANNOT PLAN A VALID PLAN FOR THE GIVEN GOAL POSE, ROBOT SELF COLLSION OR "
            "KINEMATIC CONSTRAINS MIGHT BE THE CAUSE ");
        popUp->show();
    } else {
        popUp->setPopupText("SENDING ROBOT JOINTS TO GIVEN GOAL INN JOINT SPACE, PROCESSING NOW.... ");
        popUp->show();
    }
}

/**
 * @brief
 * round after 2 decimals
 * @param var
 * @return double
 */
double RQTGUIControlPlugin::round(double var) {
    double value = (int)(var * 1000 + .5);
    return (double)value / 1000;
}
}  // namespace user_interface_rqt_plugin
PLUGINLIB_EXPORT_CLASS(user_interface_rqt_plugin::RQTGUIControlPlugin, rqt_gui_cpp::Plugin);