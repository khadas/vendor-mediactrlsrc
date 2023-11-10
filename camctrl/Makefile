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

$(info "LDLIBS : $(LDLIBS)")

$(info "TARGET_CFLAGS : $(TARGET_CFLAGS)")
$(info "CFLAGS : $(CFLAGS)")
$(info "LDFLAGS : $(LDFLAGS)")
$(info "OUT_DIR : $(OUT_DIR)")
$(info "STAGING_DIR : $(STAGING_DIR)")
$(info "TARGET_DIR : $(TARGET_DIR)")


$(TARGET): $(OUT_DIR)/$(OBJS)
	@echo " [LINK] $@"
	@$(CXX) $(LDFLAGS) $+ $(LDLIBS) -o $(OUT_DIR)/$@

$(OUT_DIR)/$(OBJDIR)/%.o: %.cc
	@echo " [CC] $<"
	@mkdir -p $(OUT_DIR)/$(OBJDIR)
	@$(CC) $(CFLAGS) -o $@ -c $<

.PHONY:clean

clean:
	@echo " [CLEAN] ..."
	@rm -rf $(OUT_DIR)/$(OBJDIR) $(OUT_DIR)/$(TARGET)

