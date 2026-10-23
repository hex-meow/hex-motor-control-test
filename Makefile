# hexmeow motor demo Makefile
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

# Transport layers, shared by every demo regardless of motor protocol.
COMMON_SRCS := \
	src/hexmeow_canfd.c \
	src/hexmeow_sdo.c

# hexmeow motors: CiA402 profile, compressed MIT, one-to-many RPDO.
HEXMEOW_SRCS := \
	src/hexmeow_mit_target.c \
	src/hexmeow_motor_init.c \
	src/hexmeow_motor_ctrl.c

# Meow Motor: its own 0x41xx/0x45xx object dictionary, uncompressed MIT.
MEOW_SRCS := \
	src/meow_motor_init.c \
	src/meow_motor_ctrl.c

COMMON_OBJS    := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(COMMON_SRCS))
HEXMEOW_OBJS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(HEXMEOW_SRCS))
MEOW_OBJS      := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(MEOW_SRCS))

APPS := demo_mit demo_velocity demo_regen demo_meow_mit
APP_BINS := $(patsubst %,$(BUILD_DIR)/%,$(APPS))

.PHONY: all clean
all: $(APP_BINS)

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/apps/%.o: src/apps/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/demo_mit: $(BUILD_DIR)/apps/demo_mit.o $(HEXMEOW_OBJS) $(COMMON_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_velocity: $(BUILD_DIR)/apps/demo_velocity.o $(HEXMEOW_OBJS) $(COMMON_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_regen: $(BUILD_DIR)/apps/demo_regen.o $(HEXMEOW_OBJS) $(COMMON_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_meow_mit: $(BUILD_DIR)/apps/demo_meow_mit.o $(MEOW_OBJS) $(COMMON_OBJS)
	$(CC) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
