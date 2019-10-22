/*
 * (C) Copyright 2019
 * Adam Boardman
 *
 * Hall sensor client registrations
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MEDIATEK_HALL_H__
#define __MEDIATEK_HALL_H__

extern int hall_register_client(struct notifier_block *nb);
extern int hall_unregister_client(struct notifier_block *nb);

#define HALL_FCOVER_OPEN        (1)
#define HALL_FCOVER_CLOSE       (0)

#endif //__MEDIATEK_HALL_H__