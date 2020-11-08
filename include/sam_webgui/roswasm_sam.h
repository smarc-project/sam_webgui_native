#ifndef ROSWASM_SAM_H
#define ROSWASM_SAM_H

#include <roswasm_webgui/roswasm_widget.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>

#include <sam_msgs/PercentStamped.h>
#include <sam_msgs/ThrusterAngles.h>
#include <sam_msgs/BallastAngles.h>
#include <sam_msgs/Leak.h>
#include <sam_msgs/ConsumedChargeFeedback.h>
#include <sam_msgs/CircuitStatusStampedArray.h>
#include <sam_msgs/ConsumedChargeArray.h>

#include <smarc_msgs/DualThrusterFeedback.h>
#include <smarc_msgs/DualThrusterRPM.h>
#include <smarc_msgs/CTDFeedback.h>

#include <uavcan_ros_bridge/CircuitStatus.h>
#include <uavcan_ros_bridge/UavcanNodeStatusNamedArray.h>

#include <cola2_msgs/DVL.h>

#include <sbg_driver/SbgEkfEuler.h>

namespace roswasm_webgui {

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher* pub);
bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher* pub);
bool draw_thruster_rpms(smarc_msgs::DualThrusterRPM& msg, roswasm::Publisher* pub);
bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub);
// // bool draw_bar_signed(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub);
// bool draw_bar_signed(const ImVec2 pos, const ImVec2 size, const int v_fb, const int rangeBar, const int v_cmd, bool draw_value = true);

extern bool guiDebug;
extern int drawTabs(int _guiMode, const std::map<int, const char*> _modeMap);

class SamActuatorWidget {
private:
    TopicWidget<sam_msgs::ThrusterAngles>* thruster_angles;
    TopicWidget<smarc_msgs::DualThrusterRPM>* thruster_rpms;
    roswasm::Publisher* rpm_pub;
    roswasm::Timer* pub_timer;
    bool rpm_pub_enabled;
    TopicWidget<sam_msgs::PercentStamped>* lcg_actuator;
    TopicWidget<std_msgs::Bool>* lcg_control_enable;
    TopicWidget<std_msgs::Float64>* lcg_control_setpoint;
    TopicWidget<sam_msgs::PercentStamped>* vbs_actuator;
    TopicWidget<std_msgs::Bool>* vbs_control_enable;
    TopicWidget<std_msgs::Float64>* vbs_control_setpoint;
    TopicWidget<sam_msgs::BallastAngles>* tcg_actuator;
    TopicWidget<std_msgs::Bool>* tcg_control_enable;
    TopicWidget<std_msgs::Float64>* tcg_control_setpoint;

public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_actuator_window);
    SamActuatorWidget(roswasm::NodeHandle* nh);
};

class SamDashboardWidget {
private:
    bool was_leak;
    TopicBuffer<sam_msgs::Leak>* leak;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;
    TopicBuffer<smarc_msgs::DualThrusterFeedback>* rpms;
    TopicBuffer<cola2_msgs::DVL>* dvl;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
    TopicBuffer<sensor_msgs::Temperature>* motorTemp;
public:
    bool is_emergency() { return was_leak; }
    void show_window(bool& show_dashboard_window);
    SamDashboardWidget(roswasm::NodeHandle* nh);
};

class SamDashboardWidget2 {
private:
    bool was_leak;
    TopicBuffer<sam_msgs::Leak>* leak;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;
    TopicBuffer<smarc_msgs::DualThrusterFeedback>* rpms;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
public:
    bool is_emergency() { return was_leak; }
    void show_window(bool& show_dashboard2_window);
    SamDashboardWidget2(roswasm::NodeHandle* nh);
};

// ---------------------------------------- SamMonitorWidget ----------------------------------------
class SamMonitorWidget {
private:
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    // TopicBuffer<uavcan_ros_bridge::CircuitStatus>* circuit;
    TopicBuffer<sam_msgs::CircuitStatusStampedArray>* circuit;
    roswasm::Subscriber* subCharge;
    void callbackCharge(const sam_msgs::ConsumedChargeArray& msg);
    std::unordered_map<int, float> circuitCharges;
    // TopicBuffer<sam_msgs::ConsumedChargeFeedback>* charge;
    TopicBuffer<uavcan_ros_bridge::UavcanNodeStatusNamedArray>* uavcan;
    TopicBuffer<sam_msgs::ConsumedChargeArray>* charge;
    TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_cmd;
    TopicBuffer<sensor_msgs::FluidPressure>* vbs_pressure;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;
    TopicBuffer<smarc_msgs::DualThrusterFeedback>* thrusters_fb;
    TopicBuffer<smarc_msgs::DualThrusterRPM>* thrusters_cmd;
    TopicBuffer<smarc_msgs::CTDFeedback>* ctd;
    TopicBuffer<cola2_msgs::DVL>* dvl;
    TopicBuffer<std_msgs::Bool>* dvl_enable_fb;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
    TopicBuffer<sensor_msgs::Temperature>* motorTemp;
    TopicBuffer<sensor_msgs::FluidPressure>* motorPressure;
    TopicBuffer<sbg_driver::SbgEkfEuler>* sbg_euler;
public:
    void show_window(bool& show_monitor_window, bool guiDebug);
    SamMonitorWidget(roswasm::NodeHandle* nh);
};
// -------------------------------------- SamMonitorWidget end --------------------------------------

class SamTeleopWidget {
private:
    bool enabled;
    sam_msgs::ThrusterAngles angles_msg;
    smarc_msgs::DualThrusterRPM rpm_msg;
    roswasm::Publisher* rpm_pub;
    roswasm::Publisher* angle_pub;
    roswasm::Timer* pub_timer;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_teleop_window);
    SamTeleopWidget(roswasm::NodeHandle* nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
