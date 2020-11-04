#include <sam_webgui/roswasm_sam.h>

#include <roswasm_webgui/imgui/imgui.h>

namespace roswasm_webgui {

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Angles cmd slider");
    ImGui::SliderFloat("", &msg.weight_1_offset_radians, -1.6f, 1.6f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Angles cmd input", &msg.weight_1_offset_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub->publish(msg);
    }

    return false;
}

bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("Percent cmd slider");
    ImGui::SliderFloat("", &msg.value, 0.0f, 100.0f, "%.2f%%");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    bool lock = ImGui::IsItemActive();

    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Percent cmd input", &msg.value, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return lock;
}

bool draw_thruster_rpms(sam_msgs::ThrusterRPMs& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Thruster 1");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_1_rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    bool lock = ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("First cmd input", &msg.thruster_1_rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Thruster 2");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_2_rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    lock = lock || ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("Second cmd input", &msg.thruster_2_rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return lock;
}

bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Hori (rad)");
    ImGui::SameLine();
    ImGui::SliderFloat("", &msg.thruster_horizontal_radians, -0.1f, 0.18f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("First cmd input", &msg.thruster_horizontal_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Vert (rad)");
    ImGui::SameLine();
    ImGui::SliderFloat("", &msg.thruster_vertical_radians, -0.1f, 0.15f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Second cmd input", &msg.thruster_vertical_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    return false;
}

SamActuatorWidget::SamActuatorWidget(roswasm::NodeHandle* nh) : rpm_pub_enabled(false), pub_timer(nullptr)
{
    //thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "core/thrust_vector_cmd", "core/thrust_fb1", "core/thrust_fb2");
    //thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "core/rpm_cmd", "core/rpm_fb1", "core/rpm_fb2");
    thruster_angles = new TopicWidget<sam_msgs::ThrusterAngles>(nh, &draw_thruster_angles, "core/thrust_vector_cmd");
    //thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_fb");
    thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_cmd");
    rpm_pub = nh->advertise<sam_msgs::ThrusterRPMs>("core/rpm_cmd");

    lcg_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/lcg_cmd", "core/lcg_fb");
    lcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/lcg/pid_enable");
    lcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/lcg/setpoint"); //, -1.6, 1.6)

    vbs_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/vbs_cmd", "core/vbs_fb");
    vbs_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/vbs/pid_enable");
    vbs_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(0., 5.), "ctrl/vbs/setpoint"); //, 0 5)
        
    tcg_actuator = new TopicWidget<sam_msgs::BallastAngles>(nh, &draw_ballast_angles, "core/tcg_cmd");
    tcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/tcg/pid_enable");
    tcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/tcg/setpoint"); //, -1.6, 1.6)

}

void SamActuatorWidget::pub_callback(const ros::TimerEvent& e)
{
    if (rpm_pub_enabled) {
        rpm_pub->publish(thruster_rpms->get_msg());
    }
}

void SamActuatorWidget::show_window(bool& show_actuator_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Actuator controls", &show_actuator_window);

    if (ImGui::CollapsingHeader("Thruster Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Angles");
        thruster_angles->show_widget();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        thruster_rpms->show_widget();
        ImGui::Checkbox("Publish RPMs at 10hz", &rpm_pub_enabled);
        ImGui::PopID();
        if (rpm_pub_enabled && pub_timer == nullptr) {
            pub_timer = new roswasm::Timer(0.08, std::bind(&SamActuatorWidget::pub_callback, this, std::placeholders::_1));
        }
        else if (!rpm_pub_enabled && pub_timer != nullptr) {
            delete pub_timer;
            pub_timer = nullptr;
        }
    }

    if (ImGui::CollapsingHeader("Longitudinal Centre of Gravity (LCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("LCG");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        lcg_actuator->show_widget();
        ImGui::PopID();
        lcg_control_enable->show_widget();
        ImGui::Text("Pitch (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        lcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Variable Buoyancy System (VBS)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("VBS");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        vbs_actuator->show_widget();
        ImGui::PopID();
        vbs_control_enable->show_widget();
        ImGui::Text("Depth (m)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        vbs_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Transversal Centre of Gravity (TCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("TCG");
        ImGui::Text("Pos (rad)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        tcg_actuator->show_widget();
        ImGui::PopID();
        tcg_control_enable->show_widget();
        ImGui::Text("Roll (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        tcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }

    ImGui::End();
}

SamDashboardWidget::SamDashboardWidget(roswasm::NodeHandle* nh) : was_leak(false)
{
    leak = new TopicBuffer<sam_msgs::Leak>(nh, "core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<sam_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
}

void SamDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status dashboard", &show_dashboard_window);

    if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        was_leak = was_leak || leak->get_msg().value;

        float sz = ImGui::GetTextLineHeight();
        std::string status_text;
        ImColor status_color;
        if (!was_leak) {
            status_text = "No leaks!";
            status_color = ImColor(0, 255, 0);
        }
        else {
            status_text = "Leak!!!!!";
            status_color = ImColor(255, 0, 0);
        }
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
        ImGui::Dummy(ImVec2(sz, sz));
        ImGui::SameLine();
        ImGui::Text("%s", status_text.c_str());

        ImGui::SameLine(150);
        ImGui::Text("Battery: %.0f%%", 100.*battery->get_msg().percentage);
    }

    if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        ImGui::SameLine(150);
        ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        ImGui::SameLine(300);
        ImGui::Text("Depth: %.2fm", depth->get_msg().data);
    }

    if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
        ImGui::SameLine(150);
        ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
        ImGui::SameLine(300);
        ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
    }

    if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
        ImGui::SameLine(300);
        ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
    }

    if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("VBS pos: %.2f%%", vbs->get_msg().value);
        ImGui::SameLine(150);
        ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
        ImGui::SameLine(300);
        ImGui::Text("RPMs: %d, %d rpm", rpms->get_msg().thruster_1_rpm, rpms->get_msg().thruster_2_rpm);
    }

    ImGui::End();
}

SamDashboardWidget2::SamDashboardWidget2(roswasm::NodeHandle* nh) : was_leak(false)
{
    leak = new TopicBuffer<sam_msgs::Leak>(nh, "core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<sam_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
}

void SamDashboardWidget2::show_window(bool& show_dashboard_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);

    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_MenuBar;
    ImGui::Begin("Experiments dashboard", &show_dashboard_window, window_flags);

    // Menu
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Strange"))
        {
            ImGui::MenuItem("Green");
            ImGui::MenuItem("Fish");
            // ImGui::MenuItem("Teleop", NULL, &show_teleop_window);
            // ImGui::MenuItem("Main menu bar", NULL, &show_app_main_menu_bar);
            // ImGui::MenuItem("Console", NULL, &show_app_console);
            // ImGui::MenuItem("Log", NULL, &show_app_log);
            // ImGui::MenuItem("Simple layout", NULL, &show_app_layout);
            // ImGui::MenuItem("Property editor", NULL, &show_app_property_editor);
            // ImGui::MenuItem("Long text display", NULL, &show_app_long_text);
            // ImGui::MenuItem("Auto-resizing window", NULL, &show_app_auto_resize);
            // ImGui::MenuItem("Constrained-resizing window", NULL, &show_app_constrained_resize);
            // ImGui::MenuItem("Simple overlay", NULL, &show_app_simple_overlay);
            // ImGui::MenuItem("Manipulating window titles", NULL, &show_app_window_titles);
            // ImGui::MenuItem("Custom rendering", NULL, &show_app_custom_rendering);
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    
    static int selectedTab = 0;
    const std::vector<const char*> tabNames{"tab1", "tab2", "tab3", "tab4"};
    for (int i = 0; i < 4; i++)
    {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if(selectedTab == i) {
            // ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.55f, 1.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        } else {
            // ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
        }
        // ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.55f, 0.4f, 1.0f));
        // ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.59f, 1.0f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        if(ImGui::Button(tabNames[i], ImVec2(50,20))){
            selectedTab = i;
        }
        ImGui::PopStyleColor(3);
        ImGui::PopID();
    }
    if(guiDebug)
    {
        ImGui::Text("Choosen tab: %d", selectedTab);
    }

    if(selectedTab == 1) {
        if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            was_leak = was_leak || leak->get_msg().value;

            float sz = ImGui::GetTextLineHeight();
            std::string status_text;
            ImColor status_color;
            if (!was_leak) {
                status_text = "No leaks!";
                status_color = ImColor(0, 255, 0);
            }
            else {
                status_text = "Leak!!!!!";
                status_color = ImColor(255, 0, 0);
            }
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
            ImGui::Dummy(ImVec2(sz, sz));
            ImGui::SameLine();
            ImGui::Text("%s", status_text.c_str());

            ImGui::SameLine(150);
            ImGui::Text("Battery: %.0f%%", 100.*battery->get_msg().percentage);
        }

        if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
            ImGui::SameLine(150);
            ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
            ImGui::SameLine(300);
            ImGui::Text("Depth: %.2fm", depth->get_msg().data);
        }

        if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
            ImGui::SameLine(150);
            ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
            ImGui::SameLine(300);
            ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
        }

        if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
            ImGui::SameLine(150);
            ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
            ImGui::SameLine(300);
            ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
        }

        if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("VBS pos: %.2f%%", vbs->get_msg().value);
            ImGui::SameLine(150);
            ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
            ImGui::SameLine(300);
            ImGui::Text("RPMs: %d, %d rpm", rpms->get_msg().thruster_1_rpm, rpms->get_msg().thruster_2_rpm);
        }
    }

    ImGui::End();
}

// --------------------------- SamMonitorWidget ---------------------------
SamMonitorWidget::SamMonitorWidget(roswasm::NodeHandle* nh)
{
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    circuit = new TopicBuffer<uavcan_ros_bridge::CircuitStatus>(nh, "core/circuit_status_fb");
    charge = new TopicBuffer<sam_msgs::ConsumedChargeFeedback>(nh, "core/consumed_charge_fb");
    uavcan = new TopicBuffer<uavcan_ros_bridge::UavcanNodeStatusNamedArray>(nh, "core/uavcan_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<sam_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
}

void SamMonitorWidget::show_window(bool& show_dashboard_window, bool guiDebug)
{
    ImGui::SetNextWindowSize(ImVec2(1000, 450), ImGuiCond_FirstUseEver);
    ImGui::Begin("Monitoring dashboard", &show_dashboard_window);
    
    // static int selectedTab = 100;
    // const std::map<int, const char*> tabs = {{0, "Setup"}, {1, "Monitor"}, {250, "Cameras"}, {300, "Control"}, {400, "Service"}, {999, "Develop"}};
    // selectedTab = drawTabs(selectedTab, tabs);

    static int selectedTab = 0;
    const std::vector<const char*> tabNames{"Hardware", "UAVCAN", "BT", "Comms", "Payloads"};
    for (int i = 0; i < 4; i++)
    {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if(selectedTab == i) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
        }
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        if(ImGui::Button(tabNames[i], ImVec2(80,20))){
            selectedTab = i;
        }
        ImGui::PopStyleColor(3);
        ImGui::PopID();
    }
    if(guiDebug)
    {
        ImGui::Text("Choosen tab: %d", selectedTab);
    }

    const std::vector<const char*> names{"Setup", "Monitor", "Control", "Service", "Experiments"};
    if(selectedTab == 0) {
        {
            ImGui::BeginChild("Battery", ImVec2(615, 80), false, 0);
            
            ImGui::Text("Batteries");
            const std::vector<std::pair<int, const char*>> batteryColumns{
                {30,"ID"},
                {70,"SoC [%]"},
                {90,"Voltage [V]"},
                {90,"Current [A]"},
                {90,"Charge [Ah]"},
                {105,"Capacity [Ah]"},
                {70,"Status"},
                {70,"Health"},
            };
            ImGui::Columns(batteryColumns.size(), "batteryTable");
            ImGui::Separator();
            
            for(int i = 0; i<batteryColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,batteryColumns[i].first); ImGui::Text("%s", batteryColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            ImGui::Text("1"); ImGui::NextColumn();
            ImGui::Text("%.1f", 100.*battery->get_msg().percentage); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().voltage); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().current); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().charge); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().capacity); ImGui::NextColumn();
            ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            ImGui::EndChild();

            // ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, 0);

            ImGui::Text("Batteries");
            const std::vector<std::pair<int, const char*>> circuitColumns{
                {30,"ID"},
                {70,"Rail [V]"},
                {90,"Voltage [V]"},
                {90,"Current [A]"},
                {70,"Status"},
            };
            ImGui::Columns(circuitColumns.size(), "circuitTable");
            ImGui::Separator();
            
            for(int i = 0; i<circuitColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,circuitColumns[i].first); ImGui::Text("%s", circuitColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();
            ImGui::Text("%d", circuit->get_msg().circuit_id); ImGui::NextColumn();
            ImGui::Text("name"); ImGui::NextColumn();
            ImGui::Text("%.2f", circuit->get_msg().voltage); ImGui::NextColumn();
            ImGui::Text("%.2f", circuit->get_msg().current); ImGui::NextColumn();
            ImGui::Text("%d", circuit->get_msg().error_flags); ImGui::NextColumn();
            // ImGui::Text("With border:");
            // ImGui::Columns(4, "mycolumns"); // 4-ways, with border
            // ImGui::SetColumnWidth(0,50);
            // ImGui::Separator();
            // ImGui::Text("ID"); ImGui::NextColumn();
            // ImGui::Text("Name"); ImGui::NextColumn();
            // ImGui::Text("Path"); ImGui::NextColumn();
            // ImGui::Text("Hovered"); ImGui::NextColumn();
            // ImGui::Separator();
            // const char* names[3] = { "One", "Two", "Three" };
            // const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
            static int selected = -1;
            // for (int i = 0; i < 2; i++)
            // {
            //     char label[32];
            //     sprintf(label, "%04d", i);
            //     if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
            //         selected = i;
            //     bool hovered = ImGui::IsItemHovered();
            //     ImGui::NextColumn();
            //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
            //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
            //     ImGui::Text("%d", hovered); ImGui::NextColumn();
            // }
            ImGui::Columns(1);
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            // ImGui::BeginChild("ChildR", ImVec2(0, 260), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            ImGui::BeginChild("ChildR", ImVec2(0, 260), false, 0);
            ImGui::Text("Temperatures");
            ImGui::Columns(4, "mycolumns"); // 4-ways, with border
            ImGui::SetColumnWidth(0,50);
            ImGui::Separator();
            ImGui::Text("ID"); ImGui::NextColumn();
            ImGui::Text("Name"); ImGui::NextColumn();
            ImGui::Text("Path"); ImGui::NextColumn();
            ImGui::Text("Hovered"); ImGui::NextColumn();
            ImGui::Separator();
            // const char* names[3] = { "One", "Two", "Three" };
            const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
            static int selected = -1;
            for (int i = 0; i < 3; i++)
            {
                char label[32];
                sprintf(label, "%04d", i);
                if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
                    selected = i;
                bool hovered = ImGui::IsItemHovered();
                ImGui::NextColumn();
                ImGui::Text("%s", names[i]); ImGui::NextColumn();
                ImGui::Text("%s", names[i]); ImGui::NextColumn();
                ImGui::Text("%d", hovered); ImGui::NextColumn();
            }
            ImGui::Columns(1);
            ImGui::EndChild();
        }
    } else if(selectedTab == 1) {
        {
            std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health);
            std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode);

            ImVec2 _avail = ImGui::GetContentRegionAvail();
            ImGui::BeginChild("UAVCAN", ImVec2(645, _avail[1]), false, 0);
            int sizzy = uavcan->get_msg().array.size();
            int sizzy2 = uavcan->get_msg().array[0].name.size();

            ImGui::Text("UAVCAN network");
            const std::vector<std::pair<int, const char*>> uavcanColumns{
                {35, "ID"},
                {260,"Name"},
                {85, "Uptime [s]"},
                {70, "Health"},
                {120,"Mode"},
                {75, "VSSC"},
            };
            ImGui::Columns(uavcanColumns.size(), "uavcanTable");
            ImGui::Separator();
            
            for(int i = 0; i<uavcanColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,uavcanColumns[i].first); ImGui::Text("%s", uavcanColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            static int selectedUavcan = -1;
            for (int i = 0; i < uavcan->get_msg().array.size(); i++)
            {
                const std::pair<ImVec4, const char*> _health = healthToColoredString(uavcan->get_msg().array[i].ns.health);
                const std::pair<ImVec4, const char*> _mode = modeToColoredString(uavcan->get_msg().array[i].ns.mode);
                char label[32];
                sprintf(label, "%d", uavcan->get_msg().array[i].id);
                if (ImGui::Selectable(label, selectedUavcan == i, ImGuiSelectableFlags_SpanAllColumns))
                {
                    selectedUavcan = i;
                }
                ImGui::NextColumn();
                ImGui::Text("%s", uavcan->get_msg().array[i].name.c_str()); ImGui::NextColumn();
                ImGui::Text("%d", uavcan->get_msg().array[i].ns.uptime_sec); ImGui::NextColumn();
                ImGui::TextColored(_health.first, "%s", _health.second); ImGui::NextColumn();
                ImGui::TextColored(_mode.first, "%s", _mode.second); ImGui::NextColumn();
                ImGui::Text("0x%04X", uavcan->get_msg().array[i].ns.vendor_specific_status_code); ImGui::NextColumn();
            }
            ImGui::Columns(1);
            ImGui::EndChild();
        }
            ImGui::SameLine();
        {
            ImGui::BeginChild("UAVCAN2", ImGui::GetContentRegionAvail(), false, 0);
            ImGui::Text("UAVCAN details");
            ImGui::BeginChild("UAVCAN3", ImVec2(0, 0), true, 0);
            ImGui::EndChild();
            ImGui::EndChild();
        }
        // {

        //     // ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        //     ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, 0);

        //     ImGui::Text("Batteries");
        //     const std::vector<std::pair<int, const char*>> circuitColumns{
        //         {30,"ID"},
        //         {70,"Rail [V]"},
        //         {90,"Voltage [V]"},
        //         {90,"Current [A]"},
        //         {70,"Status"},
        //     };
        //     ImGui::Columns(circuitColumns.size(), "circuitTable");
        //     ImGui::Separator();
            
        //     for(int i = 0; i<circuitColumns.size(); i++)
        //     {
        //         ImGui::SetColumnWidth(i,circuitColumns[i].first); ImGui::Text("%s", circuitColumns[i].second); ImGui::NextColumn();
        //     }
        //     ImGui::Separator();
        //     ImGui::Text("%d", circuit->get_msg().circuit_id); ImGui::NextColumn();
        //     ImGui::Text("name"); ImGui::NextColumn();
        //     ImGui::Text("%.2f", circuit->get_msg().voltage); ImGui::NextColumn();
        //     ImGui::Text("%.2f", circuit->get_msg().current); ImGui::NextColumn();
        //     ImGui::Text("%d", circuit->get_msg().error_flags); ImGui::NextColumn();
        //     // ImGui::Text("With border:");
        //     // ImGui::Columns(4, "mycolumns"); // 4-ways, with border
        //     // ImGui::SetColumnWidth(0,50);
        //     // ImGui::Separator();
        //     // ImGui::Text("ID"); ImGui::NextColumn();
        //     // ImGui::Text("Name"); ImGui::NextColumn();
        //     // ImGui::Text("Path"); ImGui::NextColumn();
        //     // ImGui::Text("Hovered"); ImGui::NextColumn();
        //     // ImGui::Separator();
        //     // const char* names[3] = { "One", "Two", "Three" };
        //     // const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
        //     static int selected = -1;
        //     // for (int i = 0; i < 2; i++)
        //     // {
        //     //     char label[32];
        //     //     sprintf(label, "%04d", i);
        //     //     if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
        //     //         selected = i;
        //     //     bool hovered = ImGui::IsItemHovered();
        //     //     ImGui::NextColumn();
        //     //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
        //     //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
        //     //     ImGui::Text("%d", hovered); ImGui::NextColumn();
        //     // }
        //     ImGui::Columns(1);
        //     ImGui::EndChild();
        // }
        // // ImGui::SameLine();
        // {
        //     // ImGui::BeginChild("ChildR", ImVec2(0, 260), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        //     ImGui::BeginChild("ChildR", ImVec2(0, 260), false, 0);
        //     ImGui::Text("Temperatures");
        //     ImGui::Columns(4, "mycolumns"); // 4-ways, with border
        //     ImGui::SetColumnWidth(0,50);
        //     ImGui::Separator();
        //     ImGui::Text("ID"); ImGui::NextColumn();
        //     ImGui::Text("Name"); ImGui::NextColumn();
        //     ImGui::Text("Path"); ImGui::NextColumn();
        //     ImGui::Text("Hovered"); ImGui::NextColumn();
        //     ImGui::Separator();
        //     // const char* names[3] = { "One", "Two", "Three" };
        //     const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
        //     static int selected = -1;
        //     for (int i = 0; i < 3; i++)
        //     {
        //         char label[32];
        //         sprintf(label, "%04d", i);
        //         if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
        //             selected = i;
        //         bool hovered = ImGui::IsItemHovered();
        //         ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%d", hovered); ImGui::NextColumn();
        //     }
        //     ImGui::Columns(1);
        //     ImGui::EndChild();
        // }
    } else if(selectedTab == 2) {
        // if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        //     ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        //     ImGui::SameLine(150);
        //     ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        //     ImGui::SameLine(300);
        //     ImGui::Text("Depth: %.2fm", depth->get_msg().data);
        // }

        if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
            ImGui::SameLine(150);
            ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
            ImGui::SameLine(300);
            ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
        }

        if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
            ImGui::SameLine(150);
            ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
            ImGui::SameLine(300);
            ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
        }

        if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("VBS pos: %.2f%%", vbs->get_msg().value);
            ImGui::SameLine(150);
            ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
            ImGui::SameLine(300);
            ImGui::Text("RPMs: %d, %d rpm", rpms->get_msg().thruster_1_rpm, rpms->get_msg().thruster_2_rpm);
        }
    }

    ImGui::End();
}
std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImColor(0, 255, 0),   "OK" }},       // Green
        { 1, { ImColor(235, 149, 0), "WARNING" }},  // Orange
        { 2, { ImColor(235, 0, 211), "ERROR" }},    // Magenta
        { 3, { ImColor(255, 0, 0),   "CRITICAL" }}  // Red
    };
    try
    {
        return map.at(health);
    }
    catch (std::out_of_range&)
    {
        return { ImColor(255, 0, 0), std::to_string(health).c_str() };
    }
}

std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImColor(0, 255, 0),   "OPERATIONAL" }},      // Green
        { 1, { ImColor(0, 255, 255), "INITIALIZATION" }},   // Cyan
        { 1, { ImColor(235, 149, 0), "MAINTENANCE" }},      // Orange
        { 2, { ImColor(235, 0, 211), "SOFTWARE_UPDATE" }},  // Magenta
        { 3, { ImColor(255, 0, 0),   "OFFLINE" }}           // Red
    };
    try
    {
        return map.at(mode);
    }
    catch (std::out_of_range&)
    {
        return { ImColor(255, 0, 0), std::to_string(mode).c_str() };
    }
}

SamTeleopWidget::SamTeleopWidget(roswasm::NodeHandle* nh) : enabled(false), pub_timer(nullptr)
{
    angle_pub = nh->advertise<sam_msgs::ThrusterAngles>("core/thrust_vector_cmd");
    rpm_pub = nh->advertise<sam_msgs::ThrusterRPMs>("core/rpm_cmd");
}

void SamTeleopWidget::pub_callback(const ros::TimerEvent& e)
{
    if (enabled) {
        angle_pub->publish(angles_msg);
        rpm_pub->publish(rpm_msg);
    }
}

void SamTeleopWidget::show_window(bool& show_teleop_window)
{
    ImGuiIO& io = ImGui::GetIO();
    ImVec4 col = ImGui::GetStyle().Colors[23];

    ImGui::SetNextWindowSize(ImVec2(472, 80), ImGuiCond_FirstUseEver);
    ImGui::Begin("Keyboard teleop", &show_teleop_window);

    angles_msg.thruster_vertical_radians = angles_msg.thruster_horizontal_radians = rpm_msg.thruster_1_rpm = rpm_msg.thruster_2_rpm = 0.0f;

    float sz = ImGui::GetTextLineHeight();
    ImGui::BeginGroup();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 0.75f*sz));
    ImGui::Checkbox("Teleop enabled", &enabled);
    if (enabled && pub_timer == nullptr) {
        pub_timer = new roswasm::Timer(0.08, std::bind(&SamTeleopWidget::pub_callback, this, std::placeholders::_1));
    }
    else if (!enabled && pub_timer != nullptr) {
        delete pub_timer;
        pub_timer = nullptr;
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    bool key_down = enabled && io.KeysDownDuration[263] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##left", ImGuiDir_Left) || key_down) {
        angles_msg.thruster_horizontal_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[265] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##up", ImGuiDir_Up) || key_down) {
        angles_msg.thruster_vertical_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[264] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##down", ImGuiDir_Down) || key_down) {
        angles_msg.thruster_vertical_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    key_down = enabled && io.KeysDownDuration[262] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##right", ImGuiDir_Right) || key_down) {
        angles_msg.thruster_horizontal_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::Text("Thrust Vector");
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[87] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("w") || key_down) {
        rpm_msg.thruster_1_rpm = 500;
        rpm_msg.thruster_2_rpm = 500;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[83] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("s") || key_down) {
        rpm_msg.thruster_1_rpm = -500;
        rpm_msg.thruster_2_rpm = -500;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("Forward");
    ImGui::Dummy(ImVec2(sz, 0.5f*sz));
    ImGui::Text("Reverse");
    ImGui::EndGroup();
    ImGui::EndGroup();

    ImGui::End();

    //ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
}

} // namespace roswasm_webgui
