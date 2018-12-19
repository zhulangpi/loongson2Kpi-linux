snd-hda-ls2h-objs := hda_ls2h.o

snd-hda-codec-y := hda_codec.o
snd-hda-codec-$(CONFIG_PROC_FS) += hda_proc.o

snd-hda-codec-realtek-objs :=	patch_realtek.o

# common driver
obj-$(CONFIG_SND_HDA_LS2H) := snd-hda-codec.o

obj-$(CONFIG_SND_HDA_LS2H) += snd-hda-codec-realtek.o

# this must be the last entry after codec drivers;
obj-$(CONFIG_SND_HDA_LS2H) += snd-hda-ls2h.o
