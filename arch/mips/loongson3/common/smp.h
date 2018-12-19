#ifndef __ASM_MACH_LOONGSON3_SMP_H
#define __ASM_MACH_LOONGSON3_SMP_H

#define	LOONGSON3A_TO_BASE(cpu)	\
	0x900000003ff00000 | (((long)(cpu) & 0x0c) << 42) | (((long)(cpu) & 0x03) << 8)
#define	LOONGSON3B_TO_BASE(cpu)	\
	0x900000003ff00000 | (((long)(cpu) & 0x0c) << 42) | \
	(((cpu) & 0x0c) << 12)| (((long)(cpu) & 0x03) << 8)

#define LOONGSON3_TO_BASE(cpu) \
	(((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A) || \
	 ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A3000) || \
	 ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A2000)) ? \
	LOONGSON3A_TO_BASE(cpu) : LOONGSON3B_TO_BASE(cpu)

#define IPI_OFF_STATUS 		0x1000
#define IPI_OFF_ENABLE 		0x1004
#define IPI_OFF_SET	 	0x1008
#define IPI_OFF_CLEAR		0x100c
#define IPI_OFF_MAILBOX0	0x1020
#define IPI_OFF_MAILBOX1	0x1028
#define IPI_OFF_MAILBOX2	0x1030
#define IPI_OFF_MAILBOX3	0x1038

#endif
