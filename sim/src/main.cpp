/*
 * This file is part of the hanoi-6dof project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Dense>

#if 1
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#else
#include <GL/glew.h>
#define GLFW_DLL
#include <GLFW/glfw3.h>
#endif

#include <imgui.h>
#include "draw.h"
#include "utils/colors.h"

#include <btBulletDynamicsCommon.h>
#include "physics.h"

template <typename T> constexpr T SQ(T X) { return X * X; }

struct DebugDrawer : public btIDebugDraw {
  DebugDrawer() {}
  virtual ~DebugDrawer() {}

  virtual void drawLine(const btVector3 &from, const btVector3 &to,
                        const btVector3 &fromColor, const btVector3 &toColor) {
    glBegin(GL_LINES);
    glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

  virtual void drawLine(const btVector3 &from, const btVector3 &to,
                        const btVector3 &color) {
    drawLine(from, to, color, color);
  }

  virtual void drawContactPoint(const btVector3 &pointOnB,
                                const btVector3 &normalOnB, btScalar distance,
                                int /*lifeTime*/, const btVector3 &color) {
    btVector3 to = pointOnB + normalOnB * distance;
    const btVector3 &from = pointOnB;
    glColor4f(color.getX(), color.getY(), color.getZ(), 1.f);
    glBegin(GL_LINES);
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

  virtual void draw3dText(const btVector3 &location,
                          const char * /*textString*/) {
    glRasterPos3f(location.x(), location.y(), location.z());
  }

  virtual void reportErrorWarning(const char *warningString) {
    fputs(warningString, stderr);
  }

  virtual void drawSphere(const btVector3 &p, btScalar radius,
                          const btVector3 &color) {
    glColor4f(color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
    glPushMatrix();
    glTranslatef(p.getX(), p.getY(), p.getZ());

    int lats = 5;
    int longs = 5;

    int i, j;
    for (i = 0; i <= lats; i++) {
      btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar)(i - 1) / lats);
      btScalar z0 = radius * sin(lat0);
      btScalar zr0 = radius * cos(lat0);

      btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar)i / lats);
      btScalar z1 = radius * sin(lat1);
      btScalar zr1 = radius * cos(lat1);

      glBegin(GL_QUAD_STRIP);
      for (j = 0; j <= longs; j++) {
        btScalar lng = 2 * SIMD_PI * (btScalar)(j - 1) / longs;
        btScalar x = cos(lng);
        btScalar y = sin(lng);

        glNormal3f(x * zr0, y * zr0, z0);
        glVertex3f(x * zr0, y * zr0, z0);
        glNormal3f(x * zr1, y * zr1, z1);
        glVertex3f(x * zr1, y * zr1, z1);
      }
      glEnd();
    }

    glPopMatrix();
  }

#if 0
  virtual void drawTriangle(const btVector3 &a, const btVector3 &b,
                            const btVector3 &c, const btVector3 &color,
                            btScalar alpha) {
    const btVector3 n = btCross(b - a, c - a).normalized();
    glBegin(GL_TRIANGLES);
    glColor4f(color.getX(), color.getY(), color.getZ(), alpha);
    glNormal3d(n.getX(), n.getY(), n.getZ());
    glVertex3d(a.getX(), a.getY(), a.getZ());
    glVertex3d(b.getX(), b.getY(), b.getZ());
    glVertex3d(c.getX(), c.getY(), c.getZ());
    glEnd();
  }
#endif

  int debug_mode{1};
  virtual void setDebugMode(int mode) { debug_mode = mode; }
  virtual int getDebugMode() const { return debug_mode; }
  inline void setSingleDebugMode(DebugDrawModes mode, bool on) {
    debug_mode = on ? debug_mode | mode : debug_mode & ~mode;
  }
  inline bool getSingleDebugMode(DebugDrawModes mode) const {
    return (debug_mode & mode) == mode;
  }
};
static DebugDrawer debug_drawer;



static void error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW ERROR %i: %s\n", error, description);
}

static GLFWwindow *window = nullptr;
static bool mouse_pressed[2] = {false, false};

bool screen_active{false};
bool screen_drag{false};
int screen_button{0};
void set_screen_button(int button) {
  screen_button = button;
  if (!button)
    screen_drag = false;
}

// static void mouse_button_callback(GLFWwindow *window, int button, int
// action, int mods) {
static void mouse_button_callback(GLFWwindow *, int button, int action,
                                  int /*mods*/) {
  // Application
  // double _x, _y;
  // glfwGetCursorPos(window, &_x, &_y);
  // int x(_x), y(_y);
  // demoApplication->m_modifierKeys = mods;
  // demoApplication->mouseFunc(button, action, x, y);

  // ImGui
  if (action == GLFW_PRESS && button >= 0 && button < 2)
    mouse_pressed[button] = true;
}

static void cursor_pos_callback(GLFWwindow *, double xpos, double ypos) {
  constexpr double MIN_DRAG_MOVE_2{3.0};
  if (screen_button && !screen_drag) {
    if (SQ(draw_screen_x - xpos) + SQ(draw_screen_y - ypos) > MIN_DRAG_MOVE_2) {
      screen_drag = true;
      draw_screen_x = xpos;
      draw_screen_y = ypos;
    }
  } else {
    draw_screen_x = xpos;
    draw_screen_y = ypos;
  }
}

static void cursor_enter_callback(GLFWwindow *, int entered) {
  screen_active = entered == GL_TRUE;
  if (!screen_active) {
    screen_drag = false;
    screen_button = 0;
  }
}

static void scroll_callback(GLFWwindow *, double, double yoffset) {
  // Application
  // ??

  // ImGui
  ImGuiIO &io = ImGui::GetIO();
  // Use fractional mouse wheel, 1.0 unit 5 lines.
  io.MouseWheel += (float)yoffset;
}

static void key_callback(GLFWwindow *window, int key, int, int action,
                         int mods) {
  // Custom
  if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE) {
    glfwSetWindowShouldClose(window, 1);
    return;
  }

  // if (ImGui::GetWindowIsFocused() && ImGui::IsAnyItemActive()) {
  if (ImGui::IsAnyItemActive()) {

    // ImGui
    ImGuiIO &io = ImGui::GetIO();
    if (action == GLFW_PRESS)
      io.KeysDown[key] = true;
    if (action == GLFW_RELEASE)
      io.KeysDown[key] = false;
    io.KeyCtrl = (mods & GLFW_MOD_CONTROL) != 0;
    io.KeyShift = (mods & GLFW_MOD_SHIFT) != 0;


  } else if (action & (GLFW_PRESS | GLFW_REPEAT)) {
    // Application
    //switch (key) {
    //  // zoom
    //  case GLFW_KEY_MINUS: {
    //    cam_dist -= 0.05;
    //    cam_dist = (cam_dist > 0) ? cam_dist : 0.05;
    //  } break;
    //  case GLFW_KEY_EQUAL: { cam_dist += 0.05; } break;

    //  // rotate
    //  case GLFW_KEY_UP: { cam_ele += 1.0; } break;
    //  case GLFW_KEY_DOWN: {
    //    cam_ele -= 1.0;
    //    cam_ele = (cam_ele > 0) ? cam_ele : 1.0;
    //  } break;
    //  case GLFW_KEY_LEFT: { cam_azi += 1.0; } break;
    //  case GLFW_KEY_RIGHT: {
    //    cam_azi -= 1.0;
    //    cam_azi = (cam_azi > 0) ? cam_azi : 1.0;
    //  } break;
    //}
  }
}

static void char_callback(GLFWwindow *, unsigned int codepoint) {
  // ImGui
  if (codepoint > 0 && codepoint < 0x10000)
    ImGui::GetIO().AddInputCharacter((unsigned short)codepoint);
}

static void render();
static void update_imgui(GLFWwindow *window);

#if 0
static void window_refresh_callback(GLFWwindow *window) {
  // FIXME: this is probably broken, some more thinking has to go into this
  update_imgui(window);
  render();
  glfwSwapBuffers(window);
}
#endif

static bool is_active = false;
static void window_focus_callback(GLFWwindow *, int focus) {
  is_active = (focus == GL_TRUE);
  if (focus != GL_TRUE)
    set_screen_button(0);
}

inline void gui_sync(void) {
  if (!is_active) // if running in background idle avoid high cpu usage,
    // empirical parameter
    // std::this_thread::sleep_for(std::chrono::milliseconds(96));
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
}

static void window_size_callback(GLFWwindow *window, int /*width*/, int /*height*/) {
  screen_active = false;
  // glfwPollEvents();
  update_imgui(window);
  render();
  glfwSwapBuffers(window);
}

// This is the main rendering function that you have to implement and provide to
// ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by
// (0.5f,0.5f) or (0.375f,0.375f)
static void imgui_renderdrawlists(ImDrawList **const cmd_lists,
                                  int cmd_lists_count) {
  if (cmd_lists_count == 0)
    return;

  // We are using the OpenGL fixed pipeline to make the example code simpler to
  // read!
  // A probable faster way to render would be to collate all vertices from all
  // cmd_lists into a single vertex buffer.
  // Setup render state: alpha-blending enabled, no face culling, no depth
  // testing, scissor enabled, vertex/texcoord/color pointers.
  glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_SCISSOR_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  glEnable(GL_TEXTURE_2D);

  // Setup orthographic projection matrix
  const float width = ImGui::GetIO().DisplaySize.x;
  const float height = ImGui::GetIO().DisplaySize.y;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0f, width, height, 0.0f, -1.0f, +1.0f);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  // Render command lists
  for (int n = 0; n < cmd_lists_count; n++) {
    const ImDrawList *cmd_list = cmd_lists[n];
    const unsigned char *vtx_buffer =
        (const unsigned char *)&cmd_list->vtx_buffer.front();

#define OFFSETOF(TYPE, ELEMENT) ((size_t) & (((TYPE *)0)->ELEMENT))
    glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert),
                    (void *)(vtx_buffer + OFFSETOF(ImDrawVert, pos)));
    glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert),
                      (void *)(vtx_buffer + OFFSETOF(ImDrawVert, uv)));
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert),
                   (void *)(vtx_buffer + OFFSETOF(ImDrawVert, col)));
#undef OFFSETOF

    int vtx_offset = 0;
    for (size_t cmd_i = 0; cmd_i < cmd_list->commands.size(); cmd_i++) {
      const ImDrawCmd *pcmd = &cmd_list->commands[cmd_i];
      glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t) pcmd->texture_id);
      glScissor((int)pcmd->clip_rect.x, (int)(height - pcmd->clip_rect.w),
                (int)(pcmd->clip_rect.z - pcmd->clip_rect.x),
                (int)(pcmd->clip_rect.w - pcmd->clip_rect.y));
      glDrawArrays(GL_TRIANGLES, vtx_offset, pcmd->vtx_count);
      vtx_offset += pcmd->vtx_count;
    }
  }

  // Restore modified state
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
}

// NB: ImGui already provide OS clipboard support for Windows so this isn't
// needed if you are using Windows only.
static const char *imgui_getclipboard() {
  return glfwGetClipboardString(window);
}

static void imgui_setclipboard(const char *text) {
  glfwSetClipboardString(window, text);
}

void load_fonts_texture() {
  ImGuiIO &io = ImGui::GetIO();
  // ImFont* my_font1 = io.Fonts->AddFontDefault();
  // ImFont* my_font2 =
  // io.Fonts->AddFontFromFileTTF("extra_fonts/Karla-Regular.ttf", 15.0f);
  // ImFont* my_font3 =
  // io.Fonts->AddFontFromFileTTF("extra_fonts/ProggyClean.ttf", 13.0f);
  // my_font3->DisplayOffset.y += 1;
  // ImFont* my_font4 =
  // io.Fonts->AddFontFromFileTTF("extra_fonts/ProggyTiny.ttf", 10.0f);
  // my_font4->DisplayOffset.y += 1;
  // ImFont* my_font5 =
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 20.0f,
  // io.Fonts->GetGlyphRangesJapanese());

  unsigned char *pixels;
  int width, height;
  io.Fonts->GetTexDataAsAlpha8(&pixels, &width, &height);

  GLuint tex_id;
  glGenTextures(1, &tex_id);
  glBindTexture(GL_TEXTURE_2D, tex_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, width, height, 0, GL_ALPHA,
               GL_UNSIGNED_BYTE, pixels);

  // Store our identifier
  io.Fonts->TexID = (void *)(intptr_t) tex_id;
}

void init_imgui() {
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = "ssl-sim-gui.ini";
  io.LogFilename = "ssl-sim.log";
  io.DeltaTime = 1.0f / 60.0f; // Time elapsed since last frame, in seconds (in
                               // this sample app we'll override this every
                               // frame because our time step is variable)
  io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB; // Keyboard mapping. ImGui will use
                                          // those indices to peek into the
                                          // io.KeyDown[] array.
  io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
  io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
  io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
  io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
  io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
  io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
  io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
  io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
  io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
  io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
  io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
  io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
  io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
  io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
  io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
  io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

  io.RenderDrawListsFn = imgui_renderdrawlists;
  io.SetClipboardTextFn = imgui_setclipboard;
  io.GetClipboardTextFn = imgui_getclipboard;

  load_fonts_texture();
}

static void update_imgui(GLFWwindow *window) {
  ImGuiIO &io = ImGui::GetIO();

  // Setup resolution (every frame to accommodate for window resizing)
  int w, h;
  int display_w, display_h;
  glfwGetWindowSize(window, &w, &h);
  glfwGetFramebufferSize(window, &display_w, &display_h);
  io.DisplaySize = ImVec2(
      (float)display_w, (float)
      display_h); // Display size, in pixels. For clamping windows positions.

  // demoApplication->reshape(display_w, display_h);

  // Setup time step
  static double time = 0.0f;
  const double current_time = glfwGetTime();
  io.DeltaTime = (float)(current_time - time);
  time = current_time;

  // Setup inputs
  // (we already got mouse wheel, keyboard keys & characters from glfw callbacks
  // polled in glfwPollEvents())
  double mouse_x, mouse_y;
  glfwGetCursorPos(window, &mouse_x, &mouse_y);
  mouse_x *= (float)display_w / w; // Convert mouse coordinates to pixels
  mouse_y *= (float)display_h / h;
  io.MousePos = ImVec2((float)mouse_x, (float)mouse_y); // Mouse position, in
                                                        // pixels (set to -1,-1
                                                        // if no mouse / on
                                                        // another screen, etc.)
  io.MouseDown[0] = mouse_pressed[0] ||
                    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) !=
                        0; // If a mouse press event came, always pass it as
                           // "mouse held this frame", so we don't miss // click-release events that are shorter than 1 // frame.  io.MouseDown[1] = mouse_pressed[1] || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) != 0;

  // Start the frame
  ImGui::NewFrame();

  ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
              ImGui::GetIO().Framerate);
}

void render() {
#define CLEAR_COLOR LIGHT_GREY
  static ImVec4 clear_col =
      ImColor(CLEAR_COLOR[0], CLEAR_COLOR[1], CLEAR_COLOR[2]);
#undef CLEAR_COLOR
  ImGuiIO &io = ImGui::GetIO();
  glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
  glClearColor(clear_col.x, clear_col.y, clear_col.z, clear_col.w);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  draw_update_camera();
  world->debug_draw();

  glMatrixMode(GL_TEXTURE);
  glLoadIdentity();
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
  glDisable(GL_TEXTURE_GEN_R);

  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glDepthFunc(GL_LEQUAL);
  ImGui::Render();
}

void sigint_handler(int) {
  fprintf(stderr, "\rCtrl+C pressed, closing...\n");
  glfwSetWindowShouldClose(window, 1);
}

void sigint_handler_init(void) {
  // Handle SIGINT (Ctrl+C)
  struct sigaction sa;
  sa.sa_handler = sigint_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, NULL);
}

int main(int, char **) {
  sigint_handler_init();

  // GLFW init
  glfwSetErrorCallback(error_callback);
  if (!glfwInit()) {
    fprintf(stderr, "Could not initialize GLFW.\n");
    return EXIT_FAILURE;
  }

  // glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_SAMPLES, 16);

  const char *title = "Hanoi 6-DOF Simulator";
//#define FULLSCREEN_INIT
#ifdef FULLSCREEN_INIT
  {
    GLFWmonitor *mon = glfwGetPrimaryMonitor();
    const GLFWvidmode *vmode = glfwGetVideoMode(mon);
    if (vmode == nullptr) {
      fprintf(stderr, "Could not get video mode.\n");
      return EXIT_FAILURE;
    }
    draw_screen_width = vmode->width;
    draw_screen_height = vmode->height;
    window = glfwCreateWindow(vmode->width, vmode->height, title, mon, NULL);
  }
#else
  const int w{987}, h{610};
  draw_screen_width = w;
  draw_screen_height = h;
  window = glfwCreateWindow(w, h, title, NULL, NULL);
#endif
  if (window == nullptr) {
    fprintf(stderr, "Could not create GLFW window.\n");
    glfwTerminate();
    return EXIT_FAILURE;
  }

  // XXX: somehow this seems to solve some segfaults on the looped
  // glfwPollEvents
  glfwPollEvents();

  // more options
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // start GLEW extension handler
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    fprintf(stderr, "Could not load GLEW.\n");
    glfwTerminate();
    return EXIT_FAILURE;
  }

  glGetError(); // don't remove this call, it is needed for Ubuntu

  // get version info
  {
    const GLubyte *renderer = glGetString(GL_RENDERER); // get renderer string
    const GLubyte *version = glGetString(GL_VERSION);   // version as a string
    printf("Renderer: %s\n", renderer);
    printf("OpenGL version supported %s\n", version);
  }

  // Setup ImGui binding
  init_imgui();

  // my callbacks
  glfwSetKeyCallback(window, key_callback);
  glfwSetCharCallback(window, char_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_pos_callback);
  glfwSetCursorEnterCallback(window, cursor_enter_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetWindowFocusCallback(window, window_focus_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);

  // bool show_test_window = true;
  // bool show_another_window = false;

  // Create the world
  world = new World();
  world->dynamics.setDebugDrawer(&debug_drawer);

  while (!glfwWindowShouldClose(window)) {
    // ImGuiIO &io = ImGui::GetIO();
    mouse_pressed[0] = mouse_pressed[1] = false;
    update_imgui(window);

    world->step();

    // Rendering
    render();

    glfwSwapBuffers(window);
    glfwPollEvents();

    // this works so that when not actually drawing it still run at about 60fps
    gui_sync();
  }

  // Cleanup
  ImGui::Shutdown();
  glfwTerminate();

  printf("Bye.\n");
  return EXIT_SUCCESS;
}
