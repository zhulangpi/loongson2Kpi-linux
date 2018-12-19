#
# Makefile for loongson3 based machines.
#

obj-y += setup.o init.o cmdline.o env.o dbg_io.o time.o reset.o irq.o \
    bonito-irq.o mem.o machtype.o platform.o efi.o boardinfo.o cpu.o \
    temperature.o cputemp.o
obj-$(CONFIG_GPIOLIB) += gpio.o

#
# Serial port support
#
obj-$(CONFIG_EARLY_PRINTK) += early_printk.o
obj-$(CONFIG_SERIAL_8250) += serial.o
obj-$(CONFIG_LOONGSON_UART_BASE) += uart_base.o

#
# Suspend Support
#

obj-$(CONFIG_LOONGSON_SUSPEND) += pm.o

obj-$(CONFIG_LOONGSON3_CPUAUTOPLUG) += loongson3_cpuautoplug.o

#
# Big Memory (SWIOTLB) Support
#
obj-$(CONFIG_SWIOTLB) += dma-swiotlb.o

obj-$(CONFIG_SMP)	+= smp.o

obj-$(CONFIG_HOTPLUG_CPU)	+= cpuplug_war.o

obj-$(CONFIG_NUMA)	+= numa.o

obj-$(CONFIG_SUSPEND) += pm.o sleep.o acpi_init.o
obj-$(CONFIG_PCI_MSI) += pci_msi.o
