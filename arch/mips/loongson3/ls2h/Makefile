#
# Makefile for LST loongson3 + ls2h evaluate board.
#

obj-$(CONFIG_LS2H_SB)	+= ls2h-sb.o irq.o ls2h-irq.o platform.o

obj-$(CONFIG_LS2H_PCIE) += ls2h-pci.o

ifdef CONFIG_LS2H_PCIE
obj-$(CONFIG_SWIOTLB) 	+= swiotlb.o
endif
obj-$(CONFIG_SUSPEND) += pm.o
obj-$(CONFIG_PCI_MSI) += pci_msi.o
