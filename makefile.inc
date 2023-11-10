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

$(TARGET): $(SRCS)
	@echo " [TARGET] $(TARGET)"
	@echo " [SRCS] $(SRCS)"
	$(CC) $(TARGET_CFLAGS) $(CFLAGS) -D_FILE_OFFSET_BITS=64 -Wall $(SRCS) $(LDFLAGS) -shared -fPIC -o $(OUT_DIR)/$@


.PHONY:clean

clean:
	@echo " [CLEAN] ..."
	@rm -rf $(OUT_DIR) $(OUT_DIR)/$(TARGET)

