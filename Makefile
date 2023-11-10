ENABLE_MEDIACTRL_CAM = $(AMLSRC_MEDIACTRL_CAM)
ENABLE_MEDIACTRL_HDMI = $(AMLSRC_MEDIACTRL_HDMI)

OUT_DIR ?= .

SUB_DIRS := \
			$(CURDIR)/amlv4l2src \
			$(CURDIR)/amlv4l2src/amlsrc \

ifeq ($(ENABLE_MEDIACTRL_CAM),true)
	SUB_DIRS += \
			$(CURDIR)/amlv4l2src/camsrc \
			$(CURDIR)/camctrl \

endif

ifeq ($(ENABLE_MEDIACTRL_HDMI), true)
	SUB_DIRS += \
				$(CURDIR)/amlv4l2src/hdmisrc \
				$(CURDIR)/hdmictrl \

endif


all:
	for d in $(SUB_DIRS); do ($(MAKE) -C $$d ) || exit 1; done

clean:
	for d in $(SUB_DIRS); do ($(MAKE) $@ -C $$d ) || exit 1; done
