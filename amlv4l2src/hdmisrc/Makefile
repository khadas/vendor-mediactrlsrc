TARGET = libhdmisrc.so
SRCS = hdmisrc.c
AMLENC_LIB=$(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SRCS)))
EXTRA_CFLAGS += ""
CFLAGS += $(EXTRA_CFLAGS) -I../../include
LDFLAGS += -L../
LDLIBS += -shared -lamlv4l2src -ltvclient
LDFLAGS += -lm -lrt -lpthread -ltvclient


include ../../makefile.inc


