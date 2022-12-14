EXTRA_CFLAGS += ""
CFLAGS += $(EXTRA_CFLAGS) -I$(CURDIR) -I$(CURDIR)/amlsrc
CFLAGS += -I../include
CFLAGS += -rdynamic
LDLIBS += -shared -ldl

SRC_COMMON = $(shell find 'common' -name '*.c')

SRCS := amlv4l2src.c
SRCS += $(SRC_COMMON)

TARGET = libamlv4l2src.so

include ../makefile.inc
