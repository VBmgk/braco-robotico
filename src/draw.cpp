/*
 * This file is part of the hanoi-6dof project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <cmath>
#if 1
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#else
#include <GL/glew.h>
#define GLFW_DLL
#include <GLFW/glfw3.h>
#endif
#ifndef BULLET_DYNAMICS_COMMON
  #define BULLET_DYNAMICS_COMMON
  #include <btBulletDynamicsCommon.h>
#endif
#include <imgui.h>

#include "utils/gl.h"
#include "utils/colors.h"
template <typename T> constexpr T RAD(T D) { return M_PI * D / 180.; }

#include "draw.h"

float draw_screen_x{0};
float draw_screen_y{0};
float draw_screen_width{0};
float draw_screen_height{0};
float projected_mouse_x{0};
float projected_mouse_y{0};
const float &screen_width{draw_screen_width};
const float &screen_height{draw_screen_height};
const float &screen_x{draw_screen_x};
const float &screen_y{draw_screen_y};

#ifdef CAM_ROT
#define CAM_ROT
btScalar cam_rot{0.0};
#endif
btVector3 cam_tar{0, 0, 0.0215};

btVector3 cam_pos{};
btVector3 cam_up{};
btVector3 plane_fwd{};
btVector3 plane_left{};
static const btVector3 UP{0, 0, 1};
static const btVector3 FWD{0, 1, 0};
constexpr btScalar FNEAR{0.001};
constexpr btScalar FFAR{20.0};
//constexpr btScalar ROTSTEP{1};      // degrees
//constexpr btScalar WALKSTEP{0.050}; // meters
//constexpr btScalar ZOOMSTEP{0.050}; // meters

void draw_update_camera(void) {
  if (screen_width == 0 && screen_width == 0)
    return;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  auto aspect = screen_width / screen_height;
  glFrustum(-aspect * FNEAR, aspect * FNEAR, -FNEAR, FNEAR, FNEAR, FFAR);

  auto rot_azi = btQuaternion{UP, RAD(cam_azi)};
  plane_fwd = btMatrix3x3{rot_azi} * FWD;
  plane_left = UP.cross(plane_fwd);
  auto rot_ele = btQuaternion{plane_left, RAD(cam_ele)};
  auto cam_fwd = btMatrix3x3{rot_ele} * plane_fwd;
#ifdef CAM_ROT
  auto rot_rot = btQuaternion{cam_fwd, RAD(cam_rot)};
  cam_up = btMatrix3x3{rot_rot} * cam_fwd.cross(plane_left);
#else
  auto cam_up = cam_fwd.cross(plane_left);
#endif
  // auto cam_pos = cam_tar - cam_dist * cam_fwd;
  cam_pos = cam_tar - cam_dist * cam_fwd;
  myglLookAt(cam_pos[0], cam_pos[1], cam_pos[2], cam_tar[0], cam_tar[1],
             cam_tar[2], cam_up[0], cam_up[1], cam_up[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  ImGui::Text("cam_ele: %f", cam_ele);
  ImGui::Text("cam_azi: %f", cam_azi);
  ImGui::Text("cam_pos: %f, %f, %f", cam_pos[0], cam_pos[1], cam_pos[2]);
  ImGui::Text("cam_tar: %f, %f, %f", cam_tar[0], cam_tar[1], cam_tar[2]);
  ImGui::Text("cam_up: %f, %f, %f", cam_up[0], cam_up[1], cam_up[2]);
  ImGui::Text("cam_dist: %f", cam_dist);
  ImGui::Text("screen_width: %f", screen_width);
  ImGui::Text("screen_height: %f", screen_height);
  ImGui::Text("screen_x: %f", screen_x);
  ImGui::Text("screen_y: %f", screen_y);
}
