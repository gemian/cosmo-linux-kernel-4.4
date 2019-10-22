/*
 * (C) Copyright 2009
 * MediaTek <www.MediaTek.com>
 *
 * MT6516 Sensor IOCTL & data structure
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

#ifndef __HALL_H__
#define __HALL_H__

#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/delay.h>

//#define HALL_DEBUG

#define HALL_TAG "[hall] "
#ifdef HALL_DEBUG
#define HALL_FUN(f) printk(KERN_ERR HALL_TAG "%s\n", __FUNCTION__)
#define HALL_LOG(fmt, arg...)	printk(KERN_ERR HALL_TAG fmt, ##arg)
#else
#define HALL_FUN(f)
#define HALL_LOG(fmt, arg...)
#endif

void kpd_wakeup_src_setting(int enable);

#endif				/* __HALL_H__ */