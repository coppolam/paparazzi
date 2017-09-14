/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * Steven_guided_ctrl.h
 *
 *  Created on: Sep 11, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#ifndef STEVEN_GUIDED_CTRL_H_
#define STEVEN_GUIDED_CTRL_H_

extern void guided_ctrl_init(void);
extern void guided_ctrl_per(void);

extern bool setForwardVelocity(float velx);
extern bool stopFlying(void);
extern bool goLand(void);
extern bool trackVelocity(void);
extern bool setForwardAndTrack(float velx);
extern bool hoverGuided(void);
extern bool checkVicinity(void);
extern bool trackRelPos(void);
extern bool trackOther(void);

#endif /* STEVEN_GUIDED_CTRL_H_ */
