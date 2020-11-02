#include <stdio.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#define GLFW_INCLUDE_ES3
#include <GLES3/gl3.h>
#include <GLFW/glfw3.h>

#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_impl_glfw.h>
#include <roswasm_webgui/imgui/imgui_impl_opengl3.h>

#include <roswasm/roswasm.h>
#include <roswasm_webgui/roswasm_monlaunch.h>
#include <roswasm_webgui/roswasm_image.h>
#include <roswasm_webgui/roswasm_examples.h>

//#define WITH_SAM_MSGS

#include <sam_webgui/roswasm_sam.h>
//#include <iostream>

#include <unordered_set>

roswasm::NodeHandle* nh; 
roswasm_webgui::MonlaunchWidget* monlaunch_widget;
roswasm_webgui::ImageWidget* image_widget;
roswasm_webgui::SamActuatorWidget* actuator_widget;
roswasm_webgui::SamDashboardWidget* dashboard_widget;
roswasm_webgui::SamDashboardWidget2* dashboard_widget2;
roswasm_webgui::SamTeleopWidget* teleop_widget;

GLFWwindow* g_window;
//ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImVec4 emergency_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
std::string colorString = "";
float colorR, colorG, colorB;
float col[4] = {0,0,0,0};
int winSpacing = 30;
int winWindth1 = 470;
static int guiMode = 10;
static int guiModeOld = -1;
// const std::vector<const char*> tabNames{"Setup", "Monitor", "Control", "Service", "Experiments"};
const std::map<int, const char*> guiModes = {{10, "Setup"}, {20, "Monitor"}, {30, "Control"}, {40, "Service"}, {999, "Experiments"}};

ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;
bool show_image_window = true;
bool show_actuator_window = true;
bool show_dashboard_window = true;
bool show_dashboard_window2 = true;
bool show_teleop_window = true;

bool dark_mode = true;
bool emergency = false;

EM_JS(int, canvas_get_width, (), {
  return Module.canvas.width;
});

EM_JS(int, canvas_get_height, (), {
  return Module.canvas.height;
});

EM_JS(void, resizeCanvas, (), {
  js_resizeCanvas();
});

void loop()
{
  int width = canvas_get_width();
  int height = canvas_get_height();

  glfwSetWindowSize(g_window, width, height);

  ImGui::SetCurrentContext(imgui);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  {
    ImGui::SetNextWindowPos(ImVec2(winSpacing, winSpacing), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Roswasm webgui"); //, &show_another_window);
    float sz = ImGui::GetTextLineHeight();
    std::string status_text;
    ImColor status_color;
    if (nh->ok()) {
        status_text = "Connected to " + nh->get_websocket_url();
        status_color = ImColor(0, 255, 0);
    }
    else {
        status_text = "Disconnected from " + nh->get_websocket_url();
        status_color = ImColor(255, 0, 0);
    }
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::SameLine();
    ImGui::Text("%s", status_text.c_str());

    // GUI mode selection
    for (auto gm:guiModes)
    {
      if (gm.first != guiModes.begin()->first) ImGui::SameLine();
      ImGui::PushID(gm.first);
      if(guiMode == gm.first) {
          ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
      } else {
          ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
      }
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
      if(ImGui::Button(gm.second, ImVec2(80,20))){
          guiMode = gm.first;
      }
      ImGui::PopStyleColor(3);
      ImGui::PopID();
    }
    if (guiMode != guiModeOld)  //  10=Setup, 20=Monitor, 30=Control, 40=Service, 999=Experiments
    {
      guiModeOld = guiMode;
      if (guiMode == 10)
      {
        show_demo_window = false;
        show_monlaunch_window = true;
        show_image_window = true;
        show_actuator_window = true;
        show_dashboard_window = true;
        show_dashboard_window2 = true;
        show_teleop_window = true;
      } else if (guiMode == 20)
      {
        show_demo_window = false;
        show_monlaunch_window = false;
        show_image_window = true;
        show_actuator_window = false;
        show_dashboard_window = true;
        show_dashboard_window2 = true;
        show_teleop_window = false;
      }
    }

    // Window display options
    const int buttonSpacing = 180;
    ImGui::Checkbox("Launch control", &show_monlaunch_window);
    ImGui::SameLine(buttonSpacing); ImGui::Checkbox("Extra button", &show_monlaunch_window);
    ImGui::SameLine(2*buttonSpacing); bool pushDark = ImGui::Checkbox("Dark mode", &dark_mode);
    ImGui::Checkbox("Image topic", &show_image_window);
    ImGui::Checkbox("Actuator controls", &show_actuator_window);
    ImGui::Checkbox("Status dashboard", &show_dashboard_window);
    ImGui::Checkbox("Experiments", &show_dashboard_window2);
    ImGui::Checkbox("Keyboard teleop", &show_teleop_window);
    ImGui::Checkbox("Demo widgets", &show_demo_window);      // Edit bools storing our windows open/close state

    if(pushDark){
      if(dark_mode){
        ImGui::StyleColorsDark();
      } else {
        ImGui::StyleColorsLight();
      }
    }

    //ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
    ImGui::ColorEdit3("Picky", col); // Edit 3 floats representing a color
    ImGui::Text("Color H:%f S: %f V:%f", col[0], col[1], col[2]);
    colorString = "HSV(";
    colorString += std::to_string(col[0]);
    colorString += ", ";
    colorString += std::to_string(col[1]);
    colorString += ", ";
    colorString += std::to_string(col[2]);
    colorString += ")";
  //   static char * _chars = colorString.c_str();
    char *_chars = strdup(colorString.c_str());
    ImGui::InputText("Color", _chars, 64);
    float colorH, colorS, colorV;
    ImGui::InputFloat("R", &colorR, 0.0f, 255.0f);
    ImGui::InputFloat("G", &colorG, 0.0f, 255.0f);
    ImGui::InputFloat("B", &colorB, 0.0f, 255.0f);
    ImGui::ColorConvertRGBtoHSV(colorR, colorG, colorB, colorH, colorS, colorV);
    ImGui::Text("Color H:%f S: %f V:%f", colorH, colorS, colorV);
    ImGui::End();
  }

  if (show_monlaunch_window) {
    ImGui::SetNextWindowPos(ImVec2(winSpacing, 260), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 500), ImGuiCond_FirstUseEver);
    monlaunch_widget->show_window(show_monlaunch_window);
  }

  if (show_image_window) {
    ImGui::SetNextWindowPos(ImVec2(1072, winSpacing), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 430), ImGuiCond_FirstUseEver);
    image_widget->show_window(show_image_window);
  }

  if (show_demo_window) {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver);
      ImGui::ShowDemoWindow(&show_demo_window);
  }

  if (show_actuator_window) {
      ImGui::SetNextWindowPos(ImVec2(winWindth1+2*winSpacing, 500), ImGuiCond_FirstUseEver);
      actuator_widget->show_window(show_actuator_window);
  }

  if (show_dashboard_window) {
      ImGui::SetNextWindowPos(ImVec2(winWindth1+2*winSpacing, winSpacing), ImGuiCond_FirstUseEver);
      dashboard_widget->show_window(show_dashboard_window);
  }

  if (show_dashboard_window2) {
      ImGui::SetNextWindowPos(ImVec2(800, 120), ImGuiCond_FirstUseEver);
      dashboard_widget2->show_window(show_dashboard_window2);
  }

  if (show_teleop_window) {
      ImGui::SetNextWindowPos(ImVec2(1072, 500), ImGuiCond_FirstUseEver);
      teleop_widget->show_window(show_teleop_window);
  }

  ImGui::Render();

  int display_w, display_h;
  glfwMakeContextCurrent(g_window);
  glfwGetFramebufferSize(g_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  if (dashboard_widget->is_emergency()) {
      glClearColor(emergency_color.x, emergency_color.y, emergency_color.z, emergency_color.w);
      emergency = true;
  }
  else {
      glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  }
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwMakeContextCurrent(g_window);
}


int init()
{
  if( !glfwInit() )
  {
      fprintf( stderr, "Failed to initialize GLFW\n" );
      return 1;
  }

  //glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

  // Open a window and create its OpenGL context
  int canvasWidth = 800;
  int canvasHeight = 600;
  g_window = glfwCreateWindow( canvasWidth, canvasHeight, "SAM GUI", NULL, NULL);
  if( g_window == NULL )
  {
      fprintf( stderr, "Failed to open GLFW window.\n" );
      glfwTerminate();
      return -1;
  }
  glfwMakeContextCurrent(g_window); // Initialize GLEW

  // Create game objects
  // Setup Dear ImGui binding
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  ImGui_ImplGlfw_InitForOpenGL(g_window, false);
  ImGui_ImplOpenGL3_Init();

  // Setup style
  // ImGui::StyleColorsLight();
  // ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();
  if(dark_mode){
        ImGui::StyleColorsDark();
      } else {
        ImGui::StyleColorsLight();
      }

  // Load Fonts
  io.Fonts->AddFontDefault();
  /*
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 23.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 18.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 26.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 32.0f);
  */

  imgui =  ImGui::GetCurrentContext();

  // Cursor callbacks
  glfwSetMouseButtonCallback(g_window, ImGui_ImplGlfw_MouseButtonCallback);
  glfwSetScrollCallback(g_window, ImGui_ImplGlfw_ScrollCallback);
  glfwSetKeyCallback(g_window, ImGui_ImplGlfw_KeyCallback);
  glfwSetCharCallback(g_window, ImGui_ImplGlfw_CharCallback);

  resizeCanvas();

  return 0;
}


void quit()
{
  glfwTerminate();
}


extern "C" int main(int argc, char** argv)
{
  if (init() != 0) return 1;

  if (argc < 3) {
      printf("Rosbridge server ip and port not provided!");
      return 1;
  }

  std::string rosbridge_ip(argv[1]);
  std::string rosbridge_port(argv[2]);

  nh = new roswasm::NodeHandle(rosbridge_ip, rosbridge_port);
  monlaunch_widget = new roswasm_webgui::MonlaunchWidget(nh);
  image_widget = new roswasm_webgui::ImageWidget(nh);
  actuator_widget = new roswasm_webgui::SamActuatorWidget(nh);
  dashboard_widget = new roswasm_webgui::SamDashboardWidget(nh);
  dashboard_widget2 = new roswasm_webgui::SamDashboardWidget2(nh);
  teleop_widget = new roswasm_webgui::SamTeleopWidget(nh);

  #ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(loop, 20, 1);
  #endif

  quit();

  return 0;
}
