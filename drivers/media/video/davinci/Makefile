#
# Makefile for the davinci video device drivers.
#

# VPIF
obj-$(CONFIG_VIDEO_DAVINCI_VPIF) += vpif.o

#DM646x EVM Display driver
obj-$(CONFIG_DISPLAY_DAVINCI_DM646X_EVM) += vpif_display.o
#DM646x EVM Capture driver
obj-$(CONFIG_CAPTURE_DAVINCI_DM646X_EVM) += vpif_capture.o

# Capture: DM6446, DM355, DM365
obj-$(CONFIG_VIDEO_VPSS_SYSTEM) += vpss.o
obj-$(CONFIG_VIDEO_VPFE_CAPTURE) += vpfe_capture.o
obj-$(CONFIG_VIDEO_DM6446_CCDC) += dm644x_ccdc.o
obj-$(CONFIG_VIDEO_DM355_CCDC) += dm355_ccdc.o
obj-$(CONFIG_VIDEO_DM365_ISIF) += dm365_ccdc.o
dm355_af_driver-objs		+= dm355_af.o dm355_af_hw.o
obj-$(CONFIG_VIDEO_DM355_AF)	+= dm355_af_driver.o
dm355_aew_driver-objs		+= dm355_aew.o dm355_aew_hw.o
obj-$(CONFIG_VIDEO_DM355_AEW)	+= dm355_aew_driver.o
