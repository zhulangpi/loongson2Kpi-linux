#ifndef __ASM_MACH_LOONGSON3_SPACES_H_
#define __ASM_MACH_LOONGSON3_SPACES_H_

#ifndef CAC_BASE
#if defined(CONFIG_64BIT)
#if defined(CONFIG_DMA_NONCOHERENT) || defined(CONFIG_CPU_LOONGSON3)
#define CAC_BASE        _AC(0x9800000000000000, UL)
#else
#define CAC_BASE        _AC(0xa800000000000000, UL)
#endif /* CONFIG_DMA_NONCOHERENT || CONFIG_CPU_LOONGSON3 */
#endif /* CONFIG_64BIT */
#endif /* CONFIG_CAC_BASE */

#ifdef CONFIG_CPU_LOONGSON3
#define K_CALG_COH_SHAREABLE	3
#endif

#include <asm/mach-generic/spaces.h>
#endif
