CFLAGS += -Wall -Werror=jump-misses-init -fPIC -Os
LDFLAGS += -Wall -Wextra -fPIC -Os


CFLAGS += \
	-I$(STAGING_DIR)/usr/include/

LDFLAGS += -L$(STAGING_DIR)/usr/lib/ \
			-L$(OUT_DIR) \
			$(LDLIBS)


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

CREATE_DIR:
	@mkdir -p $(OUT_DIR)/$(OBJDIR)

$(OUT_DIR)/$(OBJDIR)/%.o: %.c CREATE_DIR
	@echo " [Compile] $<"
	@$(CC) $(CFLAGS) -o $@ -c $<


.PHONY:clean

clean:
	@echo " [CLEAN] ..."
	@rm -rf $(OUT_DIR)/$(OBJDIR) $(OUT_DIR)/$(TARGET)

