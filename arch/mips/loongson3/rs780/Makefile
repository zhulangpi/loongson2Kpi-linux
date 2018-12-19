#
# Makefile for Loongson-3 family machines
#
obj-y			+= rs780e.o irq.o pci.o 

obj-$(CONFIG_LOONGSON_MC146818) += rtc.o

obj-$(CONFIG_RS780_HPET) += hpet.o

obj-$(CONFIG_SWIOTLB) += swiotlb.o

obj-$(CONFIG_SUSPEND) += pm.o
obj-$(CONFIG_PCI_MSI) += pci_msi.o
