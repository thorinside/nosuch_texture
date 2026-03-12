UNAME_S := $(shell uname -s)
TARGET ?= hardware

PLUGIN_NAME = nt_beads

# Unity build: single source file #includes all beads_dsp .cpp files
ifeq ($(TARGET),hardware)
    SOURCES = beads_plugin.cpp stl_stubs.cpp
else
    SOURCES = beads_plugin.cpp
endif

INCLUDES = -I. -I../beads_dsp/include -I../beads_dsp/src -I./distingNT_API/include

ifeq ($(TARGET),hardware)
    CXX = arm-none-eabi-g++
    CC = arm-none-eabi-gcc
    CXXFLAGS = -std=c++11 \
               -mcpu=cortex-m7 \
               -mfpu=fpv5-d16 \
               -mfloat-abi=hard \
               -mthumb \
               -Os \
               -Wall \
               -fPIC \
               -fno-rtti \
               -fno-exceptions \
               -ffunction-sections \
               -fdata-sections \
               -fno-unwind-tables \
               -fno-asynchronous-unwind-tables \
               -fno-threadsafe-statics \
               -fno-math-errno
    CFLAGS = -mcpu=cortex-m7 \
             -mfpu=fpv5-d16 \
             -mfloat-abi=hard \
             -mthumb \
             -Os \
             -Wall \
             -fPIC \
             -ffunction-sections \
             -fdata-sections \
             -fno-unwind-tables \
             -fno-asynchronous-unwind-tables \
             -fno-math-errno
    LDFLAGS = -Wl,--relocatable -Wl,--gc-sections -Wl,-u,pluginEntry -nostdlib
    OUTPUT_DIR = plugins
    BUILD_DIR = build/hardware
    OUTPUT = $(OUTPUT_DIR)/$(PLUGIN_NAME).o
    CHECK_CMD = arm-none-eabi-nm $(OUTPUT) | grep ' U '
    SIZE_CMD = arm-none-eabi-size $(OUTPUT)

else ifeq ($(TARGET),test)
    ifeq ($(UNAME_S),Darwin)
        CXX = clang++
        CC = clang
        CXXFLAGS = -std=c++11 -fPIC -O2 -Wall -fno-rtti -fno-exceptions -DNT_TEST_BUILD
        CFLAGS = -fPIC -O2 -Wall
        LDFLAGS = -dynamiclib -undefined dynamic_lookup
        EXT = dylib
    endif
    ifeq ($(UNAME_S),Linux)
        CXX = g++
        CC = gcc
        CXXFLAGS = -std=c++11 -fPIC -O2 -Wall -fno-rtti -fno-exceptions -DNT_TEST_BUILD
        CFLAGS = -fPIC -O2 -Wall
        LDFLAGS = -shared
        EXT = so
    endif

    OUTPUT_DIR = plugins
    BUILD_DIR = build/test
    OUTPUT = $(OUTPUT_DIR)/$(PLUGIN_NAME).$(EXT)
    CHECK_CMD = nm $(OUTPUT) | grep ' U ' || echo "No undefined symbols"
    SIZE_CMD = ls -lh $(OUTPUT)
endif

CPP_SOURCES = $(filter %.cpp,$(SOURCES))
C_SOURCES = $(filter %.c,$(SOURCES))
OBJECTS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CPP_SOURCES)) $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES))

.DEFAULT_GOAL := all

all: $(OUTPUT)

$(OUTPUT): $(OBJECTS)
	@mkdir -p $(OUTPUT_DIR)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^
	@echo "Built: $@"

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

hardware:
	@$(MAKE) TARGET=hardware

push: hardware
	ntpush $(OUTPUT_DIR)/$(PLUGIN_NAME).o

test:
	@$(MAKE) TARGET=test

both: hardware test

check: $(OUTPUT)
	@echo "Checking symbols in $(OUTPUT)..."
	@$(CHECK_CMD) || true
	@echo "(Only _NT_* plus memcpy/memmove/memset should remain undefined.)"
ifeq ($(TARGET),hardware)
	@echo ""
	@echo "Checking .bss footprint..."
	@bss=$$(arm-none-eabi-size -B $(OUTPUT) | awk 'NR==2 {print $$3}'); \
		printf ".bss size = %s bytes\n" "$$bss"; \
		if [ "$$bss" -gt 8192 ]; then \
			echo "ERROR: .bss exceeds 8 KiB limit! Loader will reject the plug-in."; \
			exit 1; \
		else \
			echo "OK: .bss within limit."; \
		fi
endif

size: $(OUTPUT)
	@echo "Size of $(OUTPUT):"
	@$(SIZE_CMD)

clean:
	rm -rf build plugins
	@echo "Cleaned"

help:
	@echo "NT-Beads Build System"
	@echo ""
	@echo "Targets:"
	@echo "  hardware  - Build for distingNT hardware (.o)"
	@echo "  push      - Build and push to distingNT via USB"
	@echo "  test      - Build for nt_emu testing (.dylib/.so)"
	@echo "  both      - Build both targets"
	@echo "  check     - Check undefined symbols"
	@echo "  size      - Show plugin size"
	@echo "  clean     - Remove build artifacts"

.PHONY: all hardware push test both check size clean help
