CFLAGS += -Wall -Werror -Werror=jump-misses-init -fPIC -O2

OBJDIR = objs

OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo " [OBJS] $(OBJS)"
	@echo " [LINK] $@"
	@$(CC) $(LDFLAGS) $+ $(LDLIBS) -o $@

$(OBJDIR)/%.o: %.c
	@echo " [CC] $<"
	@mkdir -p $(OBJDIR)/$(dir $<)
	@$(CC) $(CFLAGS) -o $@ -c $<

.PHONY:clean

clean:
	@echo " [CLEAN] ..."
	@rm -rf $(OBJDIR) $(TARGET)

