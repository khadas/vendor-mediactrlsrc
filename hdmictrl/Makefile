TARGET = hdmictrl
SRCS = hdmictrl.c
AMLENC_LIB=$(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(SRCS)))
EXTRA_CFLAGS += ""
CFLAGS = -I../include
CFLAGS += $(EXTRA_CFLAGS) -Wno-stringop-truncation
LDFLAGS += -L../
LDLIBS += -lm -lrt -lpthread -ltvclient


include ../makefile.inc
