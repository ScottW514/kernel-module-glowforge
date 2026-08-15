/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * notifiers.h
 *
 * Module-wide notifier chains for fault handling.
 *
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
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
 */

#ifndef KERNEL_SRC_NOTIFIERS_H_
#define KERNEL_SRC_NOTIFIERS_H_

#include <linux/notifier.h>

/**
 * Subsystems may register callbacks with this notifier list to be called if the
 * "deadman switch" is tripped, i.e. a userspace process controlling the stepper
 * driver has crashed.
 * See cnc_api.c for more information on the deadman switch.
 *
 * The chain is blocking: handlers make their hardware safe over sleeping
 * buses (PIC register zeroing is a sync SPI transfer, thermal safing
 * reconfigures PWMs), so it may only be traversed from process context.
 * The one trip point is the pulse device release. The panic path must
 * not call it: nothing that sleeps can run at panic time, so panic
 * safing is limited to the atomic motion stop (see cnc_panic_handler)
 * with the hardware watchdog behind it.
 */
extern struct blocking_notifier_head dms_notifier_list;

#define dms_notifier_chain_register   blocking_notifier_chain_register
#define dms_notifier_chain_unregister blocking_notifier_chain_unregister
#define dms_notifier_call_chain       blocking_notifier_call_chain

#endif
