#include <sam_webgui/roswasm_sam.h>

#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_internal.h>

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

bool draw_thruster_rpms(smarc_msgs::DualThrusterRPM& msg, roswasm::Publisher* pub)
{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Thruster 1");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_front.rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    bool lock = ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("First cmd input", &msg.thruster_front.rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Thruster 2");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_back.rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub->publish(msg);
    }
    lock = lock || ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("Second cmd input", &msg.thruster_back.rpm);
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

// // bool draw_bar_signed(sam_msgs::ThrusterAngles& msg, roswasm::Publisher* pub)
// bool draw_bar_signed(const ImVec2 pos, const ImVec2 size, const int v_fb, const int rangeBar, const int v_cmd, bool draw_value = true)
// {
//     ImVec2 motorWindowPos = ImGui::GetCursorScreenPos();
//     ImVec2 motorOrigin = ImGui::GetCursorPos();

//     // const ImVec2 pos = ImVec2(130, 60);
//     // const ImVec2 size = ImVec2(40, 100);

//     const ImVec2 barPos = ImVec2(motorWindowPos.x + pos.x, motorWindowPos.y + pos.y);
//     const bool outOfRange = std::abs(v_fb) > rangeBar ? true : false;
//     const float frac_fb =  (outOfRange) ? (v_fb < 0 ? -1.0f : 1.0f) : v_fb / rangeBar;
//     const float frac_cmd =  (std::abs(v_cmd) > rangeBar) ? (v_cmd < 0 ? -1.0f : 1.0f) : v_cmd / rangeBar;

//     // const float frac_fb = v_fb / rangeBar;
//     // const float frac_cmd = v_cmd / rangeBar;
//     const ImVec2 barLength = ImVec2((size.y/2-3)*frac_fb, (size.y/2-3)*frac_cmd);
//     const ImVec2 bar_p1 = ImVec2(barPos.x + 3, barPos.y + size.y/2);
//     const ImVec2 bar_p2 = ImVec2(barPos.x + size.x - 3, barPos.y + size.y/2);
//     if (outOfRange)
//     {
//         ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), IM_COL32(195,0,0,255)); // fill
//     }
//     else
//     {
//         ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), ImGui::GetColorU32(ImGuiCol_PlotHistogram)); // fill
//     }
//     ImGui::GetWindowDrawList()->AddLine(ImVec2(barPos.x + 2, bar_p2.y - barLength[1]), ImVec2(barPos.x + size.x - 2, bar_p2.y - barLength[1]), IM_COL32(255,0,0,255), 3); // line
//     ImGui::GetWindowDrawList()->AddRect(barPos, ImVec2(barPos.x + size.x, barPos.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBgActive)); // border
//     // ImGui::Dummy(size);

//     char label1[10];
//     sprintf(label1, "%d", v_fb);
//     const ImVec2 barLabelPos = ImVec2(motorOrigin.x + pos.x + size.x / 2, motorOrigin.y + pos.y + size.y);
//     ImGui::SetCursorPos(ImVec2(barLabelPos.x - ImGui::CalcTextSize(label1).x / 2, barLabelPos.y));
//     ImGui::Text("%s", label1);

//     return false;
// }

SamActuatorWidget::SamActuatorWidget(roswasm::NodeHandle* nh) : rpm_pub_enabled(false), pub_timer(nullptr)
{
    //thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "core/thrust_vector_cmd", "core/thrust_fb1", "core/thrust_fb2");
    //thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "core/rpm_cmd", "core/rpm_fb1", "core/rpm_fb2");
    thruster_angles = new TopicWidget<sam_msgs::ThrusterAngles>(nh, &draw_thruster_angles, "core/thrust_vector_cmd");
    //thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_fb");
    thruster_rpms = new TopicWidget<smarc_msgs::DualThrusterRPM>(nh, &draw_thruster_rpms, "core/thrusters_cmd", "core/thrusters_cmd");
    rpm_pub = nh->advertise<smarc_msgs::DualThrusterRPM>("core/thrusters_cmd");

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
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
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
    panic = new TopicBuffer<std_msgs::String>(nh, "core/panic_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
    dvl = new TopicBuffer<cola2_msgs::DVL>(nh, "core/dvl", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
    motorTemp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/motor_temp", 1000);
}

void SamDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);
    ImGui::Begin("Status dashboard", &show_dashboard_window);
    // if (strcmp(panic->get_msg().data.c_str(), "") || was_panic)
    if (!panic->get_msg().data.empty() || was_panic)
    {
        ImGui::PushID(65);
        char label[64];

        sprintf(label, ">>>>>>>>>> Panic!!!! Reason: %s <<<<<<<<<<", panic->get_msg().data.c_str());
        ImGui::PushStyleColor(ImGuiCol_Button, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, emergency_color);
        ImGui::Button(label, ImVec2(ImGui::GetWindowContentRegionWidth(), 50));

        ImGui::PopStyleColor(3);
        ImGui::PopID();
        was_panic = true;
    }

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

        ImGui::SameLine(300);
        ImGui::Text("Motor temp: %.0f °C", motorTemp->get_msg().temperature-273.15);
    }

    if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        ImGui::SameLine(150);
        ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        ImGui::SameLine(300);
        ImGui::Text("Depth: %.2fm,", depth->get_msg().data);
        ImGui::SameLine();
        ImGui::Text("Alt: %.1f m", dvl->get_msg().altitude);
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
        ImGui::Text("VBS pos: %.2f%%", vbs_fb->get_msg().value);
        ImGui::SameLine(150);
        ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
        ImGui::SameLine(300);
        ImGui::Text("RPMs: F %d, B %d rpm", rpms->get_msg().thruster_front.rpm.rpm, rpms->get_msg().thruster_back.rpm.rpm);
    }

    ImGui::End();
}

SamDashboardWidget2::SamDashboardWidget2(roswasm::NodeHandle* nh) : was_leak(false)
{
    leak = new TopicBuffer<sam_msgs::Leak>(nh, "core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
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
            ImGui::Text("VBS pos: %.2f%%", vbs_fb->get_msg().value);
            ImGui::SameLine(150);
            ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
            ImGui::SameLine(300);
            ImGui::Text("RPMs: F %d, B %d rpm", rpms->get_msg().thruster_front.rpm.rpm, rpms->get_msg().thruster_back.rpm.rpm);
        }
    }

    ImGui::End();
}

// --------------------------- SamMonitorWidget ---------------------------
SamMonitorWidget::SamMonitorWidget(roswasm::NodeHandle* nh)
{
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
    circuit = new TopicBuffer<sam_msgs::CircuitStatusStampedArray>(nh, "core/circuit_status_array_fb");
    subCharge = nh->subscribe<sam_msgs::ConsumedChargeArray>("core/consumed_charge_array_fb", std::bind(&SamMonitorWidget::callbackCharge, this, std::placeholders::_1), 10);
    charge = new TopicBuffer<sam_msgs::ConsumedChargeArray>(nh, "core/consumed_charge_array_fb");
    uavcan = new TopicBuffer<uavcan_ros_bridge::UavcanNodeStatusNamedArray>(nh, "core/uavcan_fb");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    vbs_cmd = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_cmd", 1000);
    vbs_pressure = new TopicBuffer<sensor_msgs::FluidPressure>(nh, "core/vbs_tank_pressure", 1000);
    vbs_temp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/vbs_tank_temperature", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    thrusters_fb = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
    thrusters_cmd = new TopicBuffer<smarc_msgs::DualThrusterRPM>(nh, "core/thrusters_cmd", 1000);
    ctd = new TopicBuffer<smarc_msgs::CTDFeedback>(nh, "core/ctd_fb", 1000);
    dvl = new TopicBuffer<cola2_msgs::DVL>(nh, "core/dvl", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
    motorTemp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/motor_temp", 1000);
    motorPressure = new TopicBuffer<sensor_msgs::FluidPressure>(nh, "core/motor_oil_pressure", 1000);
    sbg_euler = new TopicBuffer<sbg_driver::SbgEkfEuler>(nh, "sbg/ekf_euler", 1000);
    log = new TopicBuffer<rosgraph_msgs::Log>(nh, "/rosout_agg", 1000);
    subLog = nh->subscribe<rosgraph_msgs::Log>("/rosout_agg", std::bind(&SamMonitorWidget::callbackLog, this, std::placeholders::_1), 10);
}

void SamMonitorWidget::show_window(bool& show_dashboard_window, bool guiDebug)
{
    ImGui::SetNextWindowSize(ImVec2(1000, 450), ImGuiCond_FirstUseEver);
    ImGui::Begin("Monitoring dashboard", &show_dashboard_window);

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    uint32_t current_time_epoch = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;

    static int selectedTab = 0;
    const std::vector<const char*> tabNames{"Overview", "Electrical", "UAVCAN", "BT", "Comms", "Payloads"};
    for (int i = 0; i < tabNames.size(); i++)
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
        if(ImGui::Button(tabNames[i], ImVec2(85,20))){
            selectedTab = i;
        }
        ImGui::PopStyleColor(3);
        ImGui::PopID();
    }
    if(guiDebug)
    {
        ImGui::Text("Choosen tab: %d", selectedTab);
        ImGui::Text("%u", current_time_epoch);
    }

    const std::vector<const char*> names{"Setup", "Monitor", "Control", "Service", "Experiments"};
    if(selectedTab == 1) {
        {
            ImGui::BeginChild("Battery", ImVec2(475, 80), false, 0);
            
            ImGui::Text("Batteries");
            const std::vector<std::pair<int, const char*>> batteryColumns{
                {30,"ID"},
                {70,"SoC [%]"},
                {90,"Voltage [V]"},
                {90,"Current [A]"},
                {90,"Charge [Ah]"},
                {105,"Capacity [Ah]"},
                // {70,"Status"},
                // {70,"Health"},
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
            // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            ImGui::EndChild();
        }
        ImGui::Columns(2, "rail_split", false);
        ImGui::SetColumnWidth(0, 545);
        {
            ImGui::BeginChild("RailsL", ImVec2(0, 200), false, 0);

            std::unordered_map<int, const char*> circuitNames = {
                {1,"Main"},
                {2,"ESC1"},
                {3,"ESC2"},
                {4,"ESC3"},
                {5,"20v"},
                {6,"12v"},
                {7,"7v"},
                {8,"5v"},
                {9,"3v3"}
            };

            // std::unordered_map<int, float> circuitCharges;
            // for

            int sizzy = circuit->get_msg().array.size();
            ImGui::Text("Power rails");
            const std::vector<std::pair<int, const char*>> circuitColumns{
                {30, "ID"},
                {45, "Rail"},
                {90, "Voltage [V]"},
                {90, "Current [A]"},
                {75, "Power [W]"},
                {100, "Status"},
                {100, "Consumed [Ah]"},
            };
            ImGui::Columns(circuitColumns.size(), "circuitTable");
            ImGui::Separator();
            
            for(int i = 0; i<circuitColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,circuitColumns[i].first); ImGui::Text("%s", circuitColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            std::pair<ImVec4, const char*> circuitModeToColoredString(const std::uint8_t mode);

            static int selectedRail = -1;
            for (int i = 0; i < circuit->get_msg().array.size(); i++)
            {
                const int id = circuit->get_msg().array[i].circuit.circuit_id;
                const float voltage = circuit->get_msg().array[i].circuit.voltage;
                const float current = circuit->get_msg().array[i].circuit.current;
                const float power = voltage*current;
                const std::pair<ImVec4, const char*> _mode = circuitModeToColoredString(circuit->get_msg().array[i].circuit.error_flags);
                char label[32];
                sprintf(label, "%d", id);
                if (ImGui::Selectable(label, selectedRail == id, ImGuiSelectableFlags_SpanAllColumns))
                {
                    selectedRail = id;
                }
                ImGui::NextColumn();
                if (circuitNames.find(id) != circuitNames.end())
                {
                    ImGui::Text("%s", circuitNames[id]);
                }
                else
                {
                    ImGui::Text("-");
                }
                ImGui::NextColumn();
                ImGui::Text("%.2f", voltage); ImGui::NextColumn();
                ImGui::Text("%.3f", current); ImGui::NextColumn();
                ImGui::Text("%.2f", power); ImGui::NextColumn();
                // ImGui::Text("%d", circuit->get_msg().array[i].circuit.error_flags); ImGui::NextColumn();
                ImGui::TextColored(_mode.first, "%s", _mode.second); ImGui::NextColumn();
                if (circuitCharges.find(id) != circuitCharges.end())
                {
                    ImGui::Text("%.4f", circuitCharges[id]);
                }
                else
                {
                    ImGui::Text("-");
                }
                ImGui::NextColumn();
            }
            ImGui::Columns(1);
            // ImGui::Separator();
            ImGui::EndChild();
        }
        // {
        //     ImGui::BeginChild("ChildR", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 260), false, 0);
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
        ImGui::NextColumn();
        // ImGui::SameLine();
        {
            ImGui::BeginChild("RailsR", ImGui::GetContentRegionAvail(), false, 0);
            ImGui::Text("Power details");
            ImGui::BeginChild("RailsR2", ImVec2(0, 0), true, 0);
            ImGui::EndChild();
            ImGui::EndChild();
        }
    } else if(selectedTab == 2) {
        {
            std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health);
            std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode);

            ImVec2 _avail = ImGui::GetContentRegionAvail();
            ImGui::BeginChild("UAVCAN", ImVec2(645, _avail[1]), false, 0);
            int sizzy = uavcan->get_msg().array.size();

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
                if(uavcan->get_msg().array[i].id == 98)
                {
                    ImGui::Text("smarc.sam.uavcan_bridge.publisher");
                }
                else
                {
                    ImGui::Text("%s", uavcan->get_msg().array[i].name.c_str());
                }
                ImGui::NextColumn();
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
    } else if(selectedTab == 0) {
        const int subWindowHeight = 205;
        {
            // ImVec2 motorWindowPos = ImGui::GetCursorScreenPos();
            // ImVec2 motorOrigin = ImGui::GetCursorPos();
            
            // ImGui::PushStyleColor(ImGuiCol_ChildBg, warning_color);
            ImGui::BeginChild("Motors", ImVec2(210, subWindowHeight), true, 0);
            // ImGui::PopStyleColor(1);
            {
                ImGui::BeginChild("motorTitle", ImVec2(60, 20), false, 0);
                ImGui::Text("Motors");
                ImGui::Separator();
                ImGui::EndChild();
            }
            // ImGui::PushID(456);
            // ImGui::PopStyleColor(1);
            // ImGui::PopID();
            // {
            //     ImGui::BeginChild("motorMain", ImVec2(300, 130), false, 0);
                ImVec2 motorWindowPos = ImGui::GetCursorScreenPos();
                ImVec2 motorOrigin = ImGui::GetCursorPos();

                const float rangeBar = 1000.0f;

                const uint32_t motorMsgAgeThresh = 3;
                const bool motorMsgOld = motorMsgAgeThresh < current_time_epoch-thrusters_fb->get_msg().thruster_front.header.stamp.sec ? true : false;
                const int v_fb = motorMsgOld ? 0 : thrusters_fb->get_msg().thruster_front.rpm.rpm;
                const int v_fb2 = motorMsgOld ? 0 : thrusters_fb->get_msg().thruster_back.rpm.rpm;

                const int v_cmd = thrusters_cmd->get_msg().thruster_front.rpm ? thrusters_cmd->get_msg().thruster_front.rpm : 0;
                const int v_cmd2 = thrusters_cmd->get_msg().thruster_back.rpm ? thrusters_cmd->get_msg().thruster_back.rpm : 0;
                
                // static int v_fb = 0;
                // ImGui::SliderInt("fb", &v_fb, -1000, 1000);
                // static int v_cmd = 0;
                // ImGui::SliderInt("cmd", &v_cmd, -1000, 1000);
                // const int M2Offset = 200;
                // ImGui::Text("Front %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]); ImGui::SameLine(M2Offset);
                // ImGui::Text("Rear %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
                // ImGui::Text("Torque"); ImGui::SameLine(M2Offset);
                ImGui::Text("Pressure");
                ImGui::Text("%.2f bar", motorPressure->get_msg().fluid_pressure/100000.0f);
                // ImGui::Text("Torque");
                // ImGui::Text("%.2f Nm", thrusters_fb->get_msg().thruster_front.torque); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.2f Nm", thrusters_fb->get_msg().thruster_back.torque);
                // ImGui::Text("Current"); ImGui::SameLine(M2Offset);
                // ImGui::Text("Current");
                // ImGui::Text("%.2f A", thrusters_fb->get_msg().thruster_front.current); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.2f A", thrusters_fb->get_msg().thruster_back.current);
                // ImGui::Text("Temp"); ImGui::SameLine(M2Offset);
                // ImGui::Text("Temp");
                // ImGui::Text("%.0f °C", motorTemp->get_msg().temperature-273.15); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.0f °C", motorTemp->get_msg().temperature-273.15);
                // ImGui::SameLine();
                const int motorOffset = 100;

                const ImVec2 size = ImVec2(40, 100);
                // draw_bar_signed(pos, size, v_fb, v_cmd, rangeBar, true);
                const ImVec2 pos = ImVec2(motorOffset, -10);
                {
                    // const ImVec2 pos = ImVec2(motorOffset, -10);

                    const ImVec2 barPos = ImVec2(motorWindowPos.x + pos.x, motorWindowPos.y + pos.y);
                    const bool outOfRange = std::abs(v_fb) > rangeBar ? true : false;
                    const float frac_fb =  (outOfRange) ? (v_fb < 0 ? -1.0f : 1.0f) : v_fb / rangeBar;
                    const float frac_cmd =  (std::abs(v_cmd) > rangeBar) ? (v_cmd < 0 ? -1.0f : 1.0f) : v_cmd / rangeBar;

                    const ImVec2 barLength = ImVec2((size.y/2-3)*frac_fb, (size.y/2-3)*frac_cmd);
                    const ImVec2 bar_p1 = ImVec2(barPos.x + 3, barPos.y + size.y/2);
                    const ImVec2 bar_p2 = ImVec2(barPos.x + size.x - 3, barPos.y + size.y/2);
                    if (outOfRange)
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), IM_COL32(195,0,0,255)); // fill
                    }
                    else
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), ImGui::GetColorU32(ImGuiCol_PlotHistogram)); // fill
                    }
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(barPos.x + 2, bar_p2.y - barLength[1]), ImVec2(barPos.x + size.x - 2, bar_p2.y - barLength[1]), IM_COL32(255,0,0,255), 3); // line
                    ImGui::GetWindowDrawList()->AddRect(barPos, ImVec2(barPos.x + size.x, barPos.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBgActive)); // border
                    // ImGui::Dummy(size);

                    char label1[10];
                    sprintf(label1, "%d", v_fb);
                    const ImVec2 barLabelPos = ImVec2(motorOrigin.x + pos.x + size.x / 2, motorOrigin.y + pos.y + size.y);
                    ImGui::SetCursorPos(ImVec2(barLabelPos.x - ImGui::CalcTextSize(label1).x / 2, barLabelPos.y));
                    ImGui::Text("%s", label1);

                    const ImVec2 m1LabelPos = ImVec2(motorOrigin.x + pos.x + size.x / 2, motorOrigin.y + pos.y - 15);
                    ImGui::SetCursorPos(ImVec2(m1LabelPos.x - ImGui::CalcTextSize("Front").x / 2, m1LabelPos.y));
                    ImGui::Text("Front");
                }

                {
                    const ImVec2 pos2 = ImVec2(motorOffset + 50, -10);

                    const ImVec2 barPos2 = ImVec2(motorWindowPos.x + pos2.x, motorWindowPos.y + pos2.y);
                    const bool outOfRange2 = std::abs(v_fb2) > rangeBar ? true : false;
                    const float frac_fb2 =  (outOfRange2) ? (v_fb2 < 0 ? -1.0f : 1.0f) : v_fb2 / rangeBar;
                    const float frac_cmd2 =  (std::abs(v_cmd2) > rangeBar) ? (v_cmd2 < 0 ? -1.0f : 1.0f) : v_cmd2 / rangeBar;

                    const ImVec2 barLength2 = ImVec2((size.y/2-3)*frac_fb2, (size.y/2-3)*frac_cmd2);
                    const ImVec2 bar_p12 = ImVec2(barPos2.x + 3, barPos2.y + size.y/2);
                    const ImVec2 bar_p22 = ImVec2(barPos2.x + size.x - 3, barPos2.y + size.y/2);
                    if (outOfRange2)
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p12, ImVec2(bar_p22.x, bar_p22.y - barLength2[0]), IM_COL32(195,0,0,255)); // fill
                    }
                    else
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p12, ImVec2(bar_p22.x, bar_p22.y - barLength2[0]), ImGui::GetColorU32(ImGuiCol_PlotHistogram)); // fill
                    }
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(barPos2.x + 2, bar_p22.y - barLength2[1]), ImVec2(barPos2.x + size.x - 2, bar_p22.y - barLength2[1]), IM_COL32(255,0,0,255), 3); // line
                    ImGui::GetWindowDrawList()->AddRect(barPos2, ImVec2(barPos2.x + size.x, barPos2.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBgActive)); // border
                    // ImGui::Dummy(size);

                    char label12[10];
                    sprintf(label12, "%d", v_fb2);
                    const ImVec2 barLabelPos2 = ImVec2(motorOrigin.x + pos2.x + size.x / 2, motorOrigin.y + pos2.y + size.y);
                    ImGui::SetCursorPos(ImVec2(barLabelPos2.x - ImGui::CalcTextSize(label12).x / 2, barLabelPos2.y));
                    ImGui::Text("%s", label12);

                    const ImVec2 m2LabelPos = ImVec2(motorOrigin.x + pos2.x + size.x / 2, motorOrigin.y + pos2.y - 15);
                    ImGui::SetCursorPos(ImVec2(m2LabelPos.x - ImGui::CalcTextSize("Back").x / 2, m2LabelPos.y));
                    ImGui::Text("Back");
                }
                {
                    ImGui::SetCursorPosY(motorOrigin.y + pos.y + size.y + 20);
                    ImGui::Separator();
                    ImGui::Columns(3, "motorTable", false);
                    ImGui::SetColumnWidth(0, motorOffset);
                    ImGui::SetColumnWidth(1, 54);
                    // ImGui::SetColumnWidth(2, 50);

                    ImGui::Text("Torque  [Nm]"); ImGui::NextColumn();
                    char label1[10];
                    sprintf(label1, "%.2f", thrusters_fb->get_msg().thruster_front.torque);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // sprintf(label1, "%.2f", thrusters_fb->get_msg().thruster_back.torque);
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    ImGui::Text("%.2f", thrusters_fb->get_msg().thruster_back.torque); ImGui::NextColumn();

                    ImGui::Text("Current [A]"); ImGui::NextColumn();
                    sprintf(label1, "%.1f", thrusters_fb->get_msg().thruster_front.current);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // sprintf(label1, "%.1f", thrusters_fb->get_msg().thruster_back.current);
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::Text("%.1f", thrusters_fb->get_msg().thruster_front.current); ImGui::NextColumn();
                    ImGui::Text("%.1f", thrusters_fb->get_msg().thruster_back.current); ImGui::NextColumn();

                    ImGui::Text("Temp    [°C]"); ImGui::NextColumn();
                    sprintf(label1, "%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::Text("%.0f", motorTemp->get_msg().temperature-273.15); ImGui::NextColumn();
                    ImGui::Text("%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::Columns(1);
                }
            // }
            // ImGui::Text("System: %u", current_time_epoch);
            // ImGui::Text("Stamp:  %u", thrusters_fb->get_msg().thruster_front.header.stamp.sec);
            // ImGui::Text("diff:   %u", current_time_epoch-thrusters_fb->get_msg().thruster_front.header.stamp.sec);


            // ImGui::Text("Motors");

            // std::string text = "1";
            // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            // ImGui::Text("%s", text);

            // const std::vector<std::pair<int, const char*>> batteryColumns{
            //     {30,"ID"},
            //     {70,"SoC [%]"},
            //     {90,"Voltage [V]"},
            //     {90,"Current [A]"},
            //     {90,"Charge [Ah]"},
            //     {105,"Capacity [Ah]"},
            //     // {70,"Status"},
            //     // {70,"Health"},
            // };
            // ImGui::Columns(batteryColumns.size(), "batteryTable");
            // ImGui::Separator();
            
            // for(int i = 0; i<batteryColumns.size(); i++)
            // {
            //     ImGui::SetColumnWidth(i,batteryColumns[i].first); ImGui::Text("%s", batteryColumns[i].second); ImGui::NextColumn();
            // }
            // ImGui::Separator();

            // ImGui::Text("1"); ImGui::NextColumn();
            // ImGui::Text("%.1f", 100.*battery->get_msg().percentage); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().voltage); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().current); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().charge); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().capacity); ImGui::NextColumn();
            // // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            // // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int ctdWidth = 200;
            ImGui::BeginChild("CTD", ImVec2(ctdWidth, subWindowHeight), true, 0);

            const uint32_t ctdMsgAgeThresh = 3;
            const bool ctdMsgOld = ctdMsgAgeThresh < current_time_epoch-ctd->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (ctdMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "ctdTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, ctdWidth - 80);
            ImGui::Text("CTD"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);

            // ImGui::Text("CTD");
            ImGui::Separator();
            ImGui::Text("Conductivity %.2f mS/cm", ctd->get_msg().conductivity);
            ImGui::Text("Temperature  %.2f °C", ctd->get_msg().temperature);
            ImGui::Text("Depth        %.2f m", ctd->get_msg().depth);
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int dvlWidth = 200;
            ImGui::BeginChild("DVL", ImVec2(dvlWidth, subWindowHeight), true, 0);

            const uint32_t dvlMsgAgeThresh = 3;
            const bool dvlMsgOld = dvlMsgAgeThresh < current_time_epoch-dvl->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (dvlMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "dvlTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, dvlWidth - 80);
            ImGui::Text("DVL"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            
            ImGui::Separator();

            ImGui::Text("Velocity [m/s]");
            ImGui::Text("x %.3f", dvl->get_msg().velocity.x); //ImGui::SameLine(60);
            ImGui::Text("y %.3f", dvl->get_msg().velocity.y); //ImGui::SameLine(50);
            ImGui::Text("z %.3f", dvl->get_msg().velocity.z);
            ImGui::Separator();
            ImGui::Text("Velocity covariance");
            ImGui::Text("11: %.3f", dvl->get_msg().velocity_covariance[0]); //ImGui::SameLine(50);
            ImGui::Text("22: %.3f", dvl->get_msg().velocity_covariance[4]); //ImGui::SameLine(50);
            ImGui::Text("33: %.3f", dvl->get_msg().velocity_covariance[8]);
            ImGui::Separator();
            // ImGui::Text("Velocity covariance");
            ImGui::Text("Altitude: %.1f m", dvl->get_msg().altitude); ImGui::SameLine(60);

            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int vbsWidth = 140;
            ImGui::BeginChild("VBS", ImVec2(vbsWidth, subWindowHeight), true, 0);

            const uint32_t vbsMsgAgeThresh = 3;
            const bool vbsMsgOld = vbsMsgAgeThresh < current_time_epoch-vbs_fb->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (vbsMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "vbsTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, vbsWidth - 80);
            ImGui::Text("VBS"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            
            ImGui::Separator();
            ImGui::Text("Position: %.1f%%", vbs_fb->get_msg().value);
            ImGui::Text("Setpoint: %.1f%%", vbs_cmd->get_msg().value);
            ImGui::Separator();
            ImGui::Text("Pressure");
            ImGui::Text("%.2f bar", vbs_pressure->get_msg().fluid_pressure/100000.0f);
            ImGui::Separator();
            ImGui::Text("Variance");
            ImGui::Text("%.2f bar", vbs_pressure->get_msg().variance/100000.0f);
            ImGui::Separator();
            ImGui::Text("Temperature");
            ImGui::Text("%.1f °C", vbs_temp->get_msg().temperature-273.15);

            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int sbgWidth = 140;
            ImGui::BeginChild("SBG", ImVec2(sbgWidth, subWindowHeight), true, 0);

            const uint32_t sbgMsgAgeThresh = 3;
            const bool sbgMsgOld = sbgMsgAgeThresh < current_time_epoch-sbg_euler->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (sbgMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "sbgTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, sbgWidth - 80);
            ImGui::Text("SBG"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            ImGui::Separator();
            ImGui::Text("Heading: %.1f°", sbg_euler->get_msg().angle.z * (180.0/3.14));
            ImGui::Text("Roll:    %.1f°", sbg_euler->get_msg().angle.x * (180.0/3.14));
            ImGui::Text("Pitch:   %.1f°", sbg_euler->get_msg().angle.y * (180.0/3.14));

            ImGui::EndChild();
        }
        
    } else if(selectedTab == 3) {
        {
            ImGui::BeginChild("btLog", ImVec2(600, 200), true, 0);

            ImGui::Text("Log");
            if(guiDebug)
            {
                ImGui::SameLine(); ImGui::Text("List size: %lu", btLogList.size());
            }
        
            ImGui::Separator();
            // const int listLimit = 5;

            // const bool aboutBT = strcmp(log->get_msg().name.c_str(), "/sam/sam_bt") == 0 ? true : false;
            // if(aboutBT)
            // {
            //     btLogList.push_front(log->get_msg());
            //     const int listOversize = btLogList.size() - listLimit;
            //     for(int i = 0; i < listOversize; i++)
            //     {
            //         btLogList.pop_back();
            //     }
            //     // ImGui::Text("%s", log->get_msg().msg.c_str());
            // }

            
            {
                ImGui::Columns(2, "btLogList");
                ImGui::Separator();

                ImGui::SetColumnWidth(0, 100); ImGui::Text("Function"); ImGui::NextColumn();
                ImGui::SetColumnWidth(1, 400); ImGui::Text("Message"); ImGui::NextColumn();

                ImGui::Separator();


                static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                // static int selectedLog = -1;
                for (auto msgIt = btLogList.begin(); msgIt != btLogList.end(); msgIt++)
                {
                    char logLabel[32];
                    sprintf(logLabel, "%s", msgIt->function.c_str());
                    if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                    {
                        selectedLog = msgIt;
                    }
                    ImGui::NextColumn();
                    ImGui::Text("%s", msgIt->msg.c_str()); ImGui::NextColumn();
                    // if (circuitNames.find(id) != circuitNames.end())
                    // {
                    //     ImGui::Text("%s", circuitNames[id]);
                    // }
                    // else
                    // {
                    //     ImGui::Text("-");
                    // }
                    // ImGui::NextColumn();
                    // ImGui::Text("%.2f", voltage); ImGui::NextColumn();
                    // ImGui::Text("%.3f", current); ImGui::NextColumn();
                    // ImGui::Text("%.2f", power); ImGui::NextColumn();
                    // ImGui::Text("%d", circuit->get_msg().array[i].circuit.error_flags); ImGui::NextColumn();
                    // if (circuitCharges.find(id) != circuitCharges.end())
                    // {
                    //     ImGui::Text("%.4f", circuitCharges[id]);
                    // }
                    // else
                    // {
                    //     ImGui::Text("-");
                    // }
                    // ImGui::NextColumn();
                }
            }
            ImGui::EndChild();
        }
        {
            ImGui::BeginChild("btLogUnfiltered", ImVec2(600, 200), true, 0);

            ImGui::Text("Log unfiltered");
            ImGui::Separator();
            ImGui::Text(">>%s<<", log->get_msg().name.c_str());
            ImGui::Text("%s", log->get_msg().msg.c_str());

            ImGui::EndChild();
        }
    }

    ImGui::End();
}
void SamMonitorWidget::callbackCharge(const sam_msgs::ConsumedChargeArray& msg)
{
    for (int i = 0; i < msg.array.size(); i++)
    {
        circuitCharges[msg.array[i].circuit_id] = msg.array[i].charge;
    }
}
void SamMonitorWidget::callbackLog(const rosgraph_msgs::Log& msg)
{
    const int btListLimit = 5;

    const bool aboutBT = strcmp(msg.name.c_str(), "/sam/sam_bt") == 0 ? true : false;
    if(aboutBT)
    {
        btLogList.push_front(msg);
        const int btListOversize = btLogList.size() - btListLimit;
        for(int i = 0; i < btListOversize; i++)
        {
            btLogList.pop_back();
        }
        // ImGui::Text("%s", msg.msg.c_str());
    }
    // for (int i = 0; i < msg.array.size(); i++)
    // {
    //     circuitCharges[msg.array[i].circuit_id] = msg.array[i].charge/1000.0;
    // }
}
std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "OK" }},       // Green (dirty)
        { 1, { ImVec4(0.87f, 0.57f, 0.0f, 1.00f),   "WARNING" }},  // Orange
        { 2, { ImColor(235, 0, 211),                "ERROR" }},    // Magenta
        { 3, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f),     "CRITICAL" }}  // Red
    };
    try
    {
        return map.at(health);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(health).c_str() };
    }
}

std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "OPERATIONAL" }},      // Green (dirty)
        { 1, { ImColor(0, 255, 255),                "INITIALIZATION" }},   // Cyan
        { 2, { ImVec4(0.87f, 0.57f, 0.0f, 1.00f),   "MAINTENANCE" }},      // Orange
        { 3, { ImColor(235, 0, 211),                "SOFTWARE_UPDATE" }},  // Magenta
        { 7, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f),     "OFFLINE" }}           // Red
    };
    try
    {
        return map.at(mode);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(mode).c_str() };
    }
}

std::pair<ImVec4, const char*> circuitModeToColoredString(const std::uint8_t mode)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 99, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "GOOD" }},     // Green (dirty)
        { 1, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "OVERVOLTAGE" }},   // Red
        { 0, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "UNDERVOLTAGE" }},  // Red
        { 4, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "OVERCURRENT" }},   // Red
        { 8, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "UNDERCURRENT" }}   // Red
    };
    try
    {
        return map.at(mode);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(mode).c_str() };
    }
}

SamTeleopWidget::SamTeleopWidget(roswasm::NodeHandle* nh) : enabled(false), pub_timer(nullptr)
{
    angle_pub = nh->advertise<sam_msgs::ThrusterAngles>("core/thrust_vector_cmd");
    rpm_pub = nh->advertise<smarc_msgs::DualThrusterRPM>("core/thrusters_cmd");
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

    angles_msg.thruster_vertical_radians = angles_msg.thruster_horizontal_radians = 0.0f;
    rpm_msg.thruster_front.rpm = rpm_msg.thruster_back.rpm = 0;

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
        rpm_msg.thruster_front.rpm = 500;
        rpm_msg.thruster_back.rpm = 500;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[83] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("s") || key_down) {
        rpm_msg.thruster_front.rpm = -500;
        rpm_msg.thruster_back.rpm = -500;
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
