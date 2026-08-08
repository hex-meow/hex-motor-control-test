# hexfellow motor demo Makefile
#
# Native build:
#   make
#
# Cross compile (Buildroot toolchain example, see README.md):
#   env CC=$BR/output/host/bin/aarch64-linux-gcc CFLAGS_EXTRA='-mcpu=cortex-a53' make
#
# All knobs (set via environment or `make VAR=...`):
#   CC             compiler (default: gcc)
#   STRIP          strip binary (default: no strip)
#   CFLAGS_EXTRA   appended to CFLAGS
#   LDFLAGS_EXTRA  appended to LDFLAGS
#   BUILD_DIR      build output directory (default: build)

CC            ?= gcc
BUILD_DIR     ?= build
CFLAGS_EXTRA  ?=
LDFLAGS_EXTRA ?=

CFLAGS  := -std=c11 -O2 -g -Wall -Wextra -Wno-unused-parameter \
           -D_GNU_SOURCE -Isrc $(CFLAGS_EXTRA)
LDFLAGS := -lpthread -lm $(LDFLAGS_EXTRA)

LIB_SRCS := \
	src/hexfellow_canfd.c \
	src/hexfellow_sdo.c \
	src/hexfellow_mit_target.c \
	src/hexfellow_motor_init.c \
	src/hexfellow_motor_ctrl.c

LIB_OBJS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(LIB_SRCS))

APPS := demo_mit demo_velocity demo_regen
APP_BINS := $(patsubst %,$(BUILD_DIR)/%,$(APPS))

.PHONY: all clean
all: $(APP_BINS)

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/apps/%.o: src/apps/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/demo_mit: $(BUILD_DIR)/apps/demo_mit.o $(LIB_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_velocity: $(BUILD_DIR)/apps/demo_velocity.o $(LIB_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_regen: $(BUILD_DIR)/apps/demo_regen.o $(LIB_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
