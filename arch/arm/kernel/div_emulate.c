/*
 *  linux/arch/arm/kernel/swp_emulate.c
 *
 *  Copyright (C) 2009 ARM Limited
 *  __user_* functions adapted from include/asm/uaccess.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Implements emulation of the SWP/SWPB instructions using load-exclusive and
 *  store-exclusive for processors that have them disabled (or future ones that
 *  might not implement them).
 *
 *  Syntax of SWP{B} instruction: SWP{B}<c> <Rt>, <Rt2>, [<Rn>]
 *  Where: Rt  = destination
 *	   Rt2 = source
 *	   Rn  = address
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/perf_event.h>

#include <asm/traps.h>
#include <asm/uaccess.h>



static unsigned long sdivcounter;
static unsigned long udivcounter;
static pid_t         previous_pid;

#ifdef CONFIG_PROC_FS
static int proc_read_status(char *page, char **start, off_t off, int count,
			    int *eof, void *data)
{
	char *p = page;
	int len;

	p += sprintf(p, "Emulated SDIV:\t\t%lu\n", sdivcounter);
	p += sprintf(p, "Emulated UDIV:\t\t%lu\n", udivcounter);
	if (previous_pid != 0)
		p += sprintf(p, "Last process:\t\t%d\n", previous_pid);

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif

static inline bool isDivideByZeroTrappingOn(void)
{
	uint32_t controlReg;

	asm("mrc p15, 0, %0, c1, c0, 0":"=r"(controlReg));

	return !!((controlReg >> 19) & 1);
}

#define GENERATE_HANDLER(nm, dispnm, typ)								\
	static int nm##_handler(struct pt_regs *regs, unsigned int instr)				\
	{												\
		typ n, m, d;										\
		if (current->pid != previous_pid) {							\
			pr_debug("\"%s\" (%ld) uses unsupported " dispnm				\
				" instruction - this will be slow\n",					\
				 current->comm, (unsigned long)current->pid);				\
			previous_pid = current->pid;							\
		}											\
		n = regs->uregs[(instr >> 16) & 15];							\
		m = regs->uregs[(instr >>  0) & 15];							\
		if (unlikely(!m)) {									\
			if (unlikely(isDivideByZeroTrappingOn()))					\
				return -1;								\
			else										\
				d = 0;									\
		} else											\
			d = n / m;									\
		regs->uregs[(instr >>  8) & 15] = d;							\
		regs->ARM_pc += 4;									\
													\
		nm ## counter++;									\
		return 0;										\
	}

GENERATE_HANDLER(sdiv, "SDIV", int32_t)
GENERATE_HANDLER(udiv, "UDIV", uint32_t)



static struct undef_hook sdiv_hook = {
	.instr_mask = 0xfff0f0f0,
	.instr_val  = 0xfb90f0f0,
	.cpsr_mask  = MODE_MASK | PSR_T_BIT | PSR_J_BIT,
	.cpsr_val   = USR_MODE | PSR_T_BIT,
	.fn	    = sdiv_handler
};

static struct undef_hook udiv_hook = {
	.instr_mask = 0xfff0f0f0,
	.instr_val  = 0xfbb0f0f0,
	.cpsr_mask  = MODE_MASK | PSR_T_BIT | PSR_J_BIT,
	.cpsr_val   = USR_MODE | PSR_T_BIT,
	.fn	    = udiv_handler
};

/*
 * Register handler and create status file in /proc/cpu
 * Invoked as late_initcall, since not needed before init spawned.
 */
static int __init div_emulation_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *res;

	res = create_proc_entry("cpu/div_emulation", S_IRUGO, NULL);

	if (!res)
		return -ENOMEM;

	res->read_proc = proc_read_status;
#endif /* CONFIG_PROC_FS */

	printk(KERN_NOTICE "Registering SDIV/UDIV emulation handler\n");
	register_undef_hook(&sdiv_hook);
	register_undef_hook(&udiv_hook);

	return 0;
}

late_initcall(div_emulation_init);
