/*
 * This file provides functions for block I/O operations on swap/file.
 *
 * Copyright (C) 1998,2001-2005 Pavel Machek <pavel@ucw.cz>
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * This file is released under the GPLv2.
 */

#include <linux/bio.h>
#include <linux/kernel.h>
#include <linux/pagemap.h>
#include <linux/swap.h>

#include "power.h"

/**
 *	submit - submit BIO request.
 *	@rw:	READ or WRITE.
 *	@off	physical offset of page.
 *	@page:	page we're reading or writing.
 *	@bio_chain: list of pending biod (for async reading)
 *
 *	Straight from the textbook - allocate and initialize the bio.
 *	If we're reading, make sure the page is marked as dirty.
 *	Then submit it and, if @bio_chain == NULL, wait.
 */
static int submit(int rw, struct block_device *bdev, sector_t sector,
		struct page *page, struct bio **bio_chain)
{
	const int bio_rw = rw | REQ_SYNC;
	struct bio *bio;

	bio = bio_alloc(__GFP_WAIT | __GFP_HIGH, 1);
	bio->bi_sector = sector;
	bio->bi_bdev = bdev;
	bio->bi_end_io = end_swap_bio_read;

    //将一个完整的page添加到bio的bio_vec中
	if (bio_add_page(bio, page, PAGE_SIZE, 0) < PAGE_SIZE) {
		printk(KERN_ERR "PM: Adding page to bio failed at %llu\n",
			(unsigned long long)sector);
		bio_put(bio);
		return -EFAULT;
	}

	lock_page(page);
	bio_get(bio);

    //如果未指定bio_chain则直接处理该bio，指定了则链入
	if (bio_chain == NULL) {
		submit_bio(bio_rw, bio);    //底层generic_make_request函数调用bio对应块设备的q->make_request_fn来处理bio
		wait_on_page_locked(page);
		if (rw == READ)
			bio_set_pages_dirty(bio);
		bio_put(bio);
	} else {
		if (rw == READ)
			get_page(page);	/* These pages are freed later */

        //bio_chain是指针变量pA的地址，pA指向一个bio实例A
        //bio变量指向刚刚创建的bio实例B，
        //B的bi_private域赋值为pA，即B的bi_private域指向A
        //pA赋值为bio变量，令pA指向B
        //本函数，输入一个指向A的指针，将新分配的B指向A，并修改指针，使其指向B
		bio->bi_private = *bio_chain;
		*bio_chain = bio;
		submit_bio(bio_rw, bio);
	}
	return 0;
}

int hib_bio_read_page(pgoff_t page_off, void *addr, struct bio **bio_chain)
{
	return submit(READ, hib_resume_bdev, page_off * (PAGE_SIZE >> 9),
			virt_to_page(addr), bio_chain);
}

int hib_bio_write_page(pgoff_t page_off, void *addr, struct bio **bio_chain)
{
	return submit(WRITE, hib_resume_bdev, page_off * (PAGE_SIZE >> 9),
			virt_to_page(addr), bio_chain);
}

//输入尾部元素
int hib_wait_on_bio_chain(struct bio **bio_chain)
{
	struct bio *bio;
	struct bio *next_bio;
	int ret = 0;

	if (bio_chain == NULL)
		return 0;

	bio = *bio_chain;
	if (bio == NULL)
		return 0;
	while (bio) {
		struct page *page;

		next_bio = bio->bi_private; //前一个元素
		page = bio->bi_io_vec[0].bv_page;
		wait_on_page_locked(page);
		if (!PageUptodate(page) || PageError(page))
			ret = -EIO;
		put_page(page);
		bio_put(bio);
		bio = next_bio;
	}
	*bio_chain = NULL;
	return ret;
}
