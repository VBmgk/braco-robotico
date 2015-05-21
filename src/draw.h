/*
 * This file is part of the hanoi-6dof project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DRAW_H
#define DRAW_H

void draw_update_camera(void);

extern float draw_screen_x;
extern float draw_screen_y;
extern float draw_screen_width;
extern float draw_screen_height;
extern float projected_mouse_x;
extern float projected_mouse_y;

#endif
