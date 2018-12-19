#
# Makefile for LST loongson3 + ls2h evaluate board.
#

obj-y += setup.o init.o cmdline.o env.o time.o reset.o irq.o \
    ls2k-irq.o mem.o machtype.o platform.o efi.o boardinfo.o cpu.o ls2k-be.o
obj-$(CONFIG_GPIOLIB) += gpio.o

#
# Serial port support
#
obj-$(CONFIG_EARLY_PRINTK) += early_printk.o

#
# Suspend Support
#


obj-$(CONFIG_SMP)	+= smp.o

obj-$(CONFIG_LS2K_PCIE) += ls2k-pci.o

obj-$(CONFIG_SWIOTLB) += swiotlb.o

obj-$(CONFIG_PCI_MSI) += pci_msi.o

#
#Hpet Support
#
obj-$(CONFIG_LS2K_HPET) += hpet.o
