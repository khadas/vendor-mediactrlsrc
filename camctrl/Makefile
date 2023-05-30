EXTRA_CFLAGS += ""
CFLAGS += $(EXTRA_CFLAGS)
CFLAGS += -D_FORTIFY_SOURCE=2 -Wno-stringop-truncation
CFLAGS += -Wall -Werror -Werror=jump-misses-init -fPIC -O2
CFLAGS = -I../include
LDLIBS += -lpthread -lmediaAPI -ltuning -lispaml -lm -ldl -llens

SRCS = \
	camctrl.cc

OBJDIR = objs
OBJS = $(patsubst %.cc,$(OBJDIR)/%.o,$(SRCS))

TARGET = camctrl

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo " [LINK] $@"
	@$(CXX) $(LDFLAGS) $+ $(LDLIBS) -o $@

$(OBJDIR)/%.o: %.cc
	@echo " [CC] $<"
	@mkdir -p $(OBJDIR)/$(dir $<)
	@$(CC) $(CFLAGS) -o $@ -c $<

.PHONY:clean

clean:
	@echo " [CLEAN] ..."
	@rm -rf $(OBJDIR) $(TARGET)

