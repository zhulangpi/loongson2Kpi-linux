/**
 * This module can create a file in /proc directory named cputemp.
 * The temperautre value will be reflash in real time,
 * use the command "cat /proc/cputemp" to read out the value.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "temperature.h"

#if PROC_PRINT
/**
 * Variable cpu_temp value update in temperature.c
 */
static int show_cputemp(struct seq_file *m, void *v)
{
	if (cpu_temp.cpu_num == 1) {
		seq_printf(m, "CPU temperature %d°C\n", cpu_temp.temp[0]);
	} else if (cpu_temp.cpu_num == 2) {
		seq_printf(m, "CPU0 temperature %d°C\n", cpu_temp.temp[0]);
		seq_printf(m, "CPU1 temperature %d°C\n", cpu_temp.temp[1]);
	}
	return 0;
}

static void *tp_start(struct seq_file *m, loff_t *pos)
{
	unsigned long i = *pos;
	return i ? NULL : (void *)1;
}

static void *tp_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return tp_start(m, pos);
}

static void tp_stop(struct seq_file *m, void *v)
{
	/* do nothing */
}

const struct seq_operations cputemp_op = {
	.start = tp_start,
	.next  = tp_next,
	.stop  = tp_stop,
	.show  = show_cputemp,
};

static int cputemp_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cputemp_op);
}

static const struct file_operations proc_cputemp_operations = {
	.open = cputemp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init proc_cputemp_init(void)
{
	proc_create("cputemp", 0, NULL, &proc_cputemp_operations);
	return 0;
}

module_init(proc_cputemp_init);
#endif
