TARGET = hdmictrl
EXTRA_CFLAGS += ""
CFLAGS = -I../include
CFLAGS += $(EXTRA_CFLAGS) -Wno-stringop-truncation
LDFLAGS += -L../
LDLIBS += -lm -lrt -lpthread -ltvclient


SRCS = \
	hdmictrl.cc

OBJDIR = objs
OBJS = $(patsubst %.cc,$(OBJDIR)/%.o,$(SRCS))



include ../makefile_bin.inc

