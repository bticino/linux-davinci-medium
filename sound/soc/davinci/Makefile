# DAVINCI Platform Support
snd-soc-davinci-objs := davinci-pcm.o
snd-soc-davinci-objs += davinci-pcm-copyfromuser.o
snd-soc-davinci-objs += cq93vc_copy.o
snd-soc-davinci-i2s-objs := davinci-i2s.o
snd-soc-davinci-mcasp-objs:= davinci-mcasp.o
snd-soc-davinci-vcif-objs:= davinci-vcif.o

obj-$(CONFIG_SND_DAVINCI_SOC) += snd-soc-davinci.o
obj-$(CONFIG_SND_DAVINCI_SOC_I2S) += snd-soc-davinci-i2s.o
obj-$(CONFIG_SND_DAVINCI_SOC_MCASP) += snd-soc-davinci-mcasp.o
obj-$(CONFIG_SND_DAVINCI_SOC_VCIF) += snd-soc-davinci-vcif.o

# DAVINCI Machine Support
snd-soc-evm-objs := davinci-evm.o
snd-soc-sffsdr-objs := davinci-sffsdr.o
snd-soc-basi-objs := davinci-basi.o
snd-soc-dingo-objs := davinci-dingo.o
snd-soc-owl-objs := davinci-owl.o

obj-$(CONFIG_SND_DAVINCI_SOC_EVM) += snd-soc-evm.o
obj-$(CONFIG_SND_DM6467_SOC_EVM) += snd-soc-evm.o
obj-$(CONFIG_SND_DA830_SOC_EVM) += snd-soc-evm.o
obj-$(CONFIG_SND_DA850_SOC_EVM) += snd-soc-evm.o
obj-$(CONFIG_SND_DAVINCI_SOC_SFFSDR) += snd-soc-sffsdr.o
obj-$(CONFIG_SND_DAVINCI_SOC_BASI) += snd-soc-basi.o
obj-$(CONFIG_SND_DAVINCI_SOC_DINGO) += snd-soc-dingo.o
obj-$(CONFIG_SND_DAVINCI_SOC_OWL) += snd-soc-owl.o
