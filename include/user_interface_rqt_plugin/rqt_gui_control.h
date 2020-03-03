/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 17:08:58
 * @modify date 2020-02-18 17:08:58
 * @desc [description]
 */
#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <user_interface_rqt_plugin/RobotController.h>

#include <visualization_msgs/MarkerArray.h>

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QtCore/QTimer>
#include <QtWidgets/QErrorMessage>
#include <QtWidgets/QGraphicsOpacityEffect>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QProgressDialog>
#include <qt5/QtGui/QMovie>
#include <qt5/QtGui/QPainter>
#include "popup.h"
#include "real_ui.h"
#include "robot_IO.h"

/**
 * @brief namespace for GUI RQT plugin
 *
 */
namespace user_interface_rqt_plugin {

/**
 * @brief RQTGUI Plugin CLASS derived from rqt_gui_cpp::Plugin
 *
 */
class RQTGUIControlPlugin : public rqt_gui_cpp::Plugin {
    Q_OBJECT
   public:
    /**
     * @brief Construct a new RQTGUIControlPlugin object
     *
     */
    RQTGUIControlPlugin();

    /**
     * @brief Destroy the RQTGUIControlPlugin object
     *
     */
    ~RQTGUIControlPlugin();

    /**
     * @brief RQT Plugins defaults functions
     *
     * @param context
     */
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    /**
     * @brief RQT Plugins defaults functions
     *
     * @param context
     */
    virtual void shutdownPlugin();

    /**
     * @brief RQT Plugins defaults functions
     *
     * @param context
     */
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

    /**
     * @brief RQT Plugins defaults functions
     *
     * @param context
     */
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings);

    /**
     * @brief input roll pitch yaw angles in degrees , return compact Qutaernion of geometry_msgs type
     *
     * @param robot_rx_deg
     * @param robot_ry_deg
     * @param robot_rz_deg
     * @param kDEG2RAD
     * @return geometry_msgs::Quaternion
     */
    geometry_msgs::Quaternion EulertoQuaternion(double robot_rx_deg, double robot_ry_deg, double robot_rz_deg,
                                                double kDEG2RAD);

   public slots:

    // TAB MANUAL

    /**
     * @brief Sends Robot to pose goal, by reading values inputted by user
     *
     */
    void SendRobotToGoal();

    /**
     * @brief Sends Robot to joint angle goal, by reading values inputted by user
     *
     */
    void SendRobotToJointGoal();

    /**
     * @brief Sends Robot to joint angle goal, by reading values inputted by user
     *
     */
    void SendRobotToGoalTool();

    /**
     * @brief Stop current motion execution
     *
     */
    void StopTrajManual();

    /**
     * @brief Pause Current Motion execution
     *
     */
    void PauseTrajManual();

    /**
     * @brief Loads current Pose of end-effector link to spin boxes
     *
     */
    void LoadDataofTool();

    /**
     * @brief Loads current joint angles values to spinboxes
     *
     */
    void LoadDataofJoints();

    /**
     * @brief Robot Joggers MOves along X+ coordinate
     *
     */
    void XPlus();

    /**
     * @brief Robot Joggers MOves along X- coordinate
     *
     */
    void XMinus();

    /**
     * @brief Robot Joggers MOves along Y+ coordinate
     *
     */
    void YPlus();

    /**
     * @brief  Robot Joggers MOves along Y-coordinate
     *
     */
    void YMinus();

    /**
     * @briefRobot Joggers MOves along Z+ coordinate
     *
     */
    void ZPlus();

    /**
     * @brief Robot Joggers MOves along Z- coordinate
     *
     */
    void ZMinus();

    /**
     * @brief mOVES ROBOT TO DEFAULT home pose, at which all joint angles are zero
     *
     */
    void Home();

    /**
     * @brief  mOVES ROBOT TO DEFAULT object scanning pose
     *
     */
    void ToScanPose();

    // TAB AUTO

    /**
     * @brief Start Maskrcnn IMage segmentation , which also allows vision_node to subscribe this segmented image and
     * finally performing 3D detection
     *
     */
    void Start3DDetection();

    /**
     * @brief Starts real_pickplace ros node
     *
     */
    void StartPickPlace();

    /**
     * @brief stops current execition of motion if anny
     *
     */
    void StopTrajAuto();
    /**
     * @brief stops current execition of motion if anny
     *
     */
    void PauseTrajAuto();

    /**
     * @brief Updates Robot stats on display with 10 ms period of time
     *
     */
    void UpdateRobotStatusDisplay();

    /**
     * @brief Updates Robot IO stats on Display with 500 ms period of time
     *
     */
    void UpdateRobotIOStatusDisplay();

    /**
     * @brief Turn ON PORT2
     *
     */
    void setIOON();

    /**
     * @brief Turn OFF PORT2
     *
     */
    void setIOFF();
    void resetAllIO();

    /**
     * @brief Continue to robot trajectory execution
     *
     */
    void continu();

    /**
     * @brief Restart Auto Pick and place operation
     *
     */
    void restart();

    /**
     * @brief
     * round after 2 decimals
     * @param var
     * @return double
     */
    double round(double var);

   private:
    // UIwidget pointer, from QT
    Ui::RQTGUIControlPluginWidget ui_;

    // widget pointer
    QWidget* widget_;

    // which method was sleected to move robot to given pose(From dropdown menu)
    int Robot_Move_Method;

    // constants to do conversions between degree-Radian and milimeter-meter
    double const kDEG2RAD = M_PI / 180.0;
    double const kMM2M = 0.001;

    // ROS node handler to handle usual ros stuff(Subscriber , publisher)
    ros::NodeHandlePtr node_handle_ptr_;

    // move_group class pointer
    moveit::planning_interface::MoveGroupInterface* move_group_ptr_;

    // set this to true to enable communication with real robot controller, if using simulator set it to false
    bool youAreRunningonRealRobot = false;

    // Listen//access to transforms between all existing links
    tf::TransformListener* listener_;

    // Robot controller pointer , to manipulat robot according to commands from USER
    RobotController* robot_controller_ptr_;

    // Timer to display and update Robot states
    QTimer* timer;
    // Timer to display and update Robot IO states
    QTimer* timer_io;

    // Apretty QT Popup to inform user the result of interaction with Buttons
    PopUp* popUp;

    // Robot IO controller class pointer to read/write IOport of robot controller via GUI
    robot_IO robot_io_controller_;

    // A simple class , we can get a list of IO port types used in robot controller
    PortTypes robot_io_port_types_;
};

}  // namespace user_interface_rqt_plugin
#endif  // my_namespace__my_plugin_H