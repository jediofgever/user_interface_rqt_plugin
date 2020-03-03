/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:06:29
 * @modify date 2019-11-28 10:06:29
 * @desc [description]
 */
#ifndef robot_controller_H
#define robot_controller_H
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>

#include <iostream>
#include <sstream>
#include <string>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>

/**
 * @brief
 * Robot Controller class, moves Robot TCP and Joints into desired values with preffered spaces(joint/cartesian,tool)
 */
class RobotController {
   private:
    // ros node handle pointer
    ros::NodeHandle *node_handle_ptr_;
    // fake command publisher for gazebo gripper controlller
    ros::Publisher finger1_command_pub_;
    ros::Publisher finger2_command_pub_;
    ros::Publisher finger3_command_pub_;

    // Used to publish Robot Trajetory , to visualized planned path in RVIZ
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // Joint Model Groups needed by visual_tools_
    const robot_model::JointModelGroup *joint_model_group;

    /**
     * @brief Flag to track state of STOP butoon
     *
     */
    bool isStopRobotClicked;

    // curlpp objects to request, recieve stats of io PORT on robot controller,
    // Note; DIO port is assigned inside ROSfanuc controller driver to control STOPping OF TRAJECTORY EXECUTION
    curlpp::Cleanup cleaner;
    curlpp::Easy request;

    /**
     * @brief enumarator to get to know which method was used to move robot
     *
     */
    enum class RobotMoverFunctions {
        moveEndEffectortoGoalinJointSpace,
        moveEndEffectortoGoalinCartesianSpace,
        moveJointstoTargetPositions,
        moveEndEffectortoGoalinToolSpace
    };

    // Which method was the latest, to move robot ? , this variable keeps that method
    RobotMoverFunctions kLateset_RobotMove_Function_Executed;

    // When using pasuse/continue of trajectory execution , we need to keep memory of target goal(END EFFECTOR Pose)
    geometry_msgs::PoseStamped target_to_continue_pose_value;

    // When using pasuse/continue of trajectory execution , we need to keep memory of target goal(joint angles)
    std::vector<double> taget_to_continue_vector_of_joint_values;

    // set this to true to enable communication with real robot controller, if using simulator set it to false
    bool youAreRunningonRealRobot = false;

   public:
    /**
     * @brief Construct a new Robot Controller object
     *
     */
    RobotController(/* args */);

    /**
     * @brief Destroy the Robot Controller object
     *
     */
    ~RobotController();

    /**
     * @brief moves robot tcp in joint space, a straight line following is not guarenteed, returns true if a valid path
     * found
     *
     * @param robot_tcp_goal_in_joint_space
     * @param move_group_ptr_
     * @return true
     * @return false
     */
    bool moveEndEffectortoGoalinJointSpace(geometry_msgs::PoseStamped robot_tcp_goal_in_joint_space,
                                           moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves robot tcp in joint space, a straight line following is  guarenteed, returns true if a valid plan was
     * found
     *
     * @param robot_tcp_goal_in_cartesian_space
     * @param move_group_ptr_
     * @return true
     * @return false
     */
    bool moveEndEffectortoGoalinCartesianSpace(geometry_msgs::PoseStamped robot_tcp_goal_in_cartesian_space,
                                               moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves each individual joint to given target position, returns true if given joint angles met a safe plan
     *
     * @param robot_joint_states
     * @param move_group_ptr_
     * @return true
     * @return false
     */
    bool moveJointstoTargetPositions(std::vector<double> robot_joint_states,
                                     moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves robot along End-effector(TOOL) link frames, returns true if a plan without collision was found
     *
     * @param robot_tcp_goal_in_tool_space
     * @param move_group_ptr_
     * @param listener_
     * @return true
     * @return false
     */
    bool moveEndEffectortoGoalinToolSpace(geometry_msgs::PoseStamped robot_tcp_goal_in_tool_space,
                                          moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                          tf::TransformListener *listener_);

    /**
     * @brief not implemented yet, but will open the robot gripper
     *
     */
    void openGripper();

    /**
     * @brief not implemented yet, but will close the robot gripper
     *
     */
    void closeGripper();

    /**
     * @brief Set the Port2to ON DIO number = 2 , Fanuc Robot controller
     *
     */
    void setPort2toON();

    /**
     * @brief Set the Port2to OFF DIO number = 2 , Fanuc Robot controller
     *
     */
    void setPort2toOFF();

    /**
     * @brief determines which method was the latest method to move robot, also get the latest set goal, if called it
     * will move robot to the where it was going
     *
     * @param move_group_ptr_
     * @param listener_
     */
    void continueRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                  tf::TransformListener *listener_);

    /**
     * @brief sets PORT number 2 on fanuc Robot controller to OFF, which makes the driver to Abort current motion and
     * makes controller wait for another motion command
     *
     * @param move_group_ptr_
     */
    void stopRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief Stops current motion of robot, remmebers the latest method to move robot and the target , If pause was
     * clicked the robot
     *
     * @param move_group_ptr_
     */
    void pauseRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief publishes the planned path of robot, and visualizes it in RVIZ
     *
     */
    void publishRobotTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                moveit_msgs::RobotTrajectory trajectory);
};
#endif
