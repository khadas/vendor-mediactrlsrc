SUB_DIRS := $(CURDIR)/mediactrlsrc \
			$(CURDIR)/libamlv4l2 \
#			$(CURDIR)/client_test

all:
	for d in $(SUB_DIRS); do ($(MAKE) -C $$d ) || exit 1; done

clean:
	for d in $(SUB_DIRS); do ($(MAKE) $@ -C $$d ) || exit 1; done
