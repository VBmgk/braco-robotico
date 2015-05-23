/*
 * This file is part of the hanoi-6dof project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DRAW_H
#define DRAW_H
#ifndef BULLET_DYNAMICS_COMMON
 #define BULLET_DYNAMICS_COMMON
 #include <btBulletDynamicsCommon.h>
#endif

void draw_update_camera(void);

extern float draw_screen_x;
extern float draw_screen_y;
extern float draw_screen_width;
extern float draw_screen_height;
extern float projected_mouse_x;
extern float projected_mouse_y;

#ifndef CAM_PAR
 #define CAM_PAR
 static btScalar cam_ele{45.0};
 static btScalar cam_azi{0.0};
 static btScalar cam_dist{0.6};
#endif

#endif
