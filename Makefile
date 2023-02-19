SUB_DIRS := \
			$(CURDIR)/amlv4l2src \
			$(CURDIR)/amlv4l2src/amlsrc \
			$(CURDIR)/amlv4l2src/camsrc \
			$(CURDIR)/amlv4l2src/hdmisrc \
			$(CURDIR)/camctrl \
			$(CURDIR)/hdmictrl \


all:
	for d in $(SUB_DIRS); do ($(MAKE) -C $$d ) || exit 1; done

clean:
	for d in $(SUB_DIRS); do ($(MAKE) $@ -C $$d ) || exit 1; done
