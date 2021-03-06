/**
 * head.h
 *
 * I2C driver for the Glowforge head.
 *
 * Copyright (C) 2020 Scott Wiederhold <s.e.wiederhold@gmail.com>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef KERNEL_SRC_HEAD_H_
#define KERNEL_SRC_HEAD_H_

#include <linux/i2c.h>

int head_probe(struct i2c_client *client, const struct i2c_device_id *id);

int head_remove(struct i2c_client *client);

#endif
