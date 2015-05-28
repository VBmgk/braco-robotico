/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef WORLD_H
#define WORLD_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

struct FieldGeometry;

struct World *new_world();
void delete_world(struct World *world);

void world_step(struct World *world);
void world_step_delta(struct World *world, float time_step, int max_substeps,
                      float fixed_time_step);

unsigned int world_get_frame_number(const struct World *world);
double world_get_timestamp(const struct World *world);

struct btDiscreteDynamicsWorld *world_bt_dynamics(struct World *world);

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
