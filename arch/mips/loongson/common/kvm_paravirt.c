/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* Copyright (C) 2017  Loongson Technologies, Inc.  All rights reserved.
* Authors: Zhang Shuangshuang <zhangshuangshuang@kict.ac.cn>
*/
#include <linux/kallsyms.h>
#include <asm/page.h>

/*hash table for H2G(host kernel vaddr -> guest kernel vaddr)
 * and V2M (guest kernel vaddr -> host kernel vaddr).
 * this should be consider for PAGE_SIZE
 */
#define H2G_SIZE ((2ULL << 32) /(2 << PAGE_SHIFT))
#define G2H_SIZE ((2ULL << 32) /(2 << PAGE_SHIFT))

unsigned long H2G[H2G_SIZE][2] = {{0}};
unsigned long G2H[G2H_SIZE] = {0};

unsigned long host_kaddress_v2p(unsigned long host_vaddr)
{
	return host_vaddr & 0xffffffff;
}

unsigned long host_kaddress_v2p_high(unsigned long host_vaddr)
{
	return host_vaddr & 0xffffffff00000000;
}

unsigned long guest_kaddress_v2p(unsigned long guest_vaddr)
{
	if((guest_vaddr & 0xffffffff80000000) == 0x4000000080000000)
		return guest_vaddr & 0x0fffffff;

	if((guest_vaddr & 0xffffffff00000000) == 0x4000000100000000)
		return guest_vaddr & 0xffffffff;

	printk("ERROR: %s:%s:%d: guest_vaddr(0x%lx)\n",
		__FILE__, __func__, __LINE__, guest_vaddr);
	while(1);
}

static void kvmmips_fill_H2G(unsigned long host_vaddr, unsigned long guest_vaddr)
{
	int index = host_kaddress_v2p_high(host_vaddr) ? 1 : 0;
	H2G[host_kaddress_v2p(host_vaddr) >> PAGE_SHIFT][index] = guest_vaddr & PAGE_MASK;
}

static void kvmmips_fill_G2H(unsigned long guest_vaddr, unsigned long host_vaddr)
{
	G2H[guest_kaddress_v2p(guest_vaddr) >> PAGE_SHIFT] = host_vaddr & PAGE_MASK;
}

static unsigned long kvmmips_get_guest_vaddr_from_hash(unsigned long host_vaddr)
{
	unsigned long guest_vaddr;
	int index = host_kaddress_v2p_high(host_vaddr) ? 1 : 0;

	guest_vaddr = H2G[host_kaddress_v2p(host_vaddr) >> PAGE_SHIFT][index];

	if(guest_vaddr == 0)
		return 0;

	return guest_vaddr | (host_vaddr & (~PAGE_MASK));
}

static unsigned long kvmmips_get_host_vaddr_from_hash(unsigned long guest_vaddr)
{
	unsigned long host_vaddr;

	if(((guest_vaddr & 0xffffffff80000000) != 0x4000000080000000)
		&& ((guest_vaddr & 0xffffffff00000000) != 0x4000000100000000)) {
		dump_stack();
		printk("ERROR: %s:%s:%d: guest_vaddr(0x%lx)\n",
			__FILE__, __func__, __LINE__, guest_vaddr);
		while(1);
	}

	host_vaddr = G2H[guest_kaddress_v2p(guest_vaddr) >> PAGE_SHIFT];

	if(host_vaddr == 0)
		return 0;

	return host_vaddr | (guest_vaddr & (~PAGE_MASK));
}

extern unsigned long kvmmips_get_host_vaddr_from_hypervisor(unsigned long guest_vaddr);
unsigned long kvmmips_get_host_vaddr(unsigned long guest_vaddr)
{
	unsigned long host_vaddr;

	if((host_vaddr = kvmmips_get_host_vaddr_from_hash(guest_vaddr)) != 0)
		return host_vaddr;

	host_vaddr = kvmmips_get_host_vaddr_from_hypervisor(guest_vaddr);
	kvmmips_fill_G2H(guest_vaddr, host_vaddr);
	kvmmips_fill_H2G(host_vaddr, guest_vaddr);

	return host_vaddr;
}

unsigned long kvmmips_get_guest_vaddr(unsigned long host_vaddr)
{
	unsigned long guest_vaddr;

	if((guest_vaddr = kvmmips_get_guest_vaddr_from_hash(host_vaddr)) != 0)
		return guest_vaddr;
	dump_stack();
	printk("ERROR: %s:%s:%d: host_vaddr(0x%lx)\n",
		__FILE__, __func__, __LINE__, host_vaddr);
	while(1);
}

pte_t kvmmips_get_guest_pte(pte_t host_pte)
{
	unsigned long temp;
	pte_t guest_pte = host_pte;

	if(host_pte.pte != 0) {
		temp = ((host_pte.pte << 1) & ~0xfff) | 0x9800000000000000;
		temp = kvmmips_get_guest_vaddr(temp);
		guest_pte.pte = ((temp & 0x0ffffffff) >> 1) | (host_pte.pte & 0x7ff);
	}

	return guest_pte;
}

pte_t kvmmips_get_host_pte(pte_t guest_pte)
{
	unsigned long temp;
	pte_t host_pte = guest_pte;

	if(guest_pte.pte != 0) {
		temp = ((guest_pte.pte << 1) & ~0xfff) | 0x4000000100000000;
		temp = kvmmips_get_host_vaddr(temp);
		host_pte.pte = (temp >> 1) | (guest_pte.pte & 0x7ff);
	}

	return host_pte;
}

//These functions cannot be defined as inline.
noinline void kvmmips_insert_swpd(unsigned long kvmmips_swapper_pg_dir)
{
	__asm__ __volatile__(
		"\tli	$2, 10002\n"
		"\ttgei	$2, 0\n"
	);
}

noinline unsigned long kvmmips_get_host_vaddr_from_hypervisor(unsigned long guest_vaddr)
{
	register unsigned long v0 asm("v0");

	__asm__ __volatile__(
		"\tli	$2, 10003\n"
		"\ttgei	$2, 0\n"
	);

	return v0;
}

noinline void kvmmips_insert_pgcurrent(unsigned long kvmmips_pg_current)
{

	__asm__ __volatile__(
		"\tli	$2, 10004\n"
		"\ttgei	$2, 0\n"
	);
}

noinline void kvmmips_local_flush_tlb_all(void)
{

	__asm__ __volatile__(
		"\tli	$2, 10005\n"
		"\ttgei	$2, 0\n"
	);
}

noinline void kvmmips_local_flush_tlb_page(unsigned long entryhi)
{

	__asm__ __volatile__(
		"\tli	$2, 10006\n"
		"\ttgei	$2, 0\n"
	);
}
