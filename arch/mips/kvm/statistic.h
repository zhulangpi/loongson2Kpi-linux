/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright (C) 2016-2018 Zhang Shuangshuang <zhangshuangshuang@loongson.cn>
 *
 */

#ifndef __KVMMIPS_STATISTIC_H__
#define __KVMMIPS_STATISTIC_H__

#include <linux/kvm_host.h>
#include <asm/kvm_host.h>

extern void kvmmips_stat_init(struct kvm_vcpu* vcpu);
extern void kvmmips_stat_exits(struct kvm_vcpu* vcpu, int type);
extern void kvmmips_stat_cp0_uses(struct kvm_vcpu* vcpu, int type);
extern void kvmmips_print_stat(struct kvm_vcpu* vcpu);

#endif
