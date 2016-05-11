ARM_CMSIS=$(HOME)/arm-cmsis
STM32_CMSIS=$(HOME)/stm32-cmsis
MYLIB=$(HOME)/stm32-lib
TOOLCHAIN=$(HOME)/gcc-arm-none-eabi-4_9-2015q3/bin

MCU=STM32F10X_MD
BOARD=STM32F103_BLUEPILL

ifeq ($(MCU),STM32F10X_MD)
CPU_OPT=-mcpu=cortex-m3
LD_SCRIPT=stm32-20-64.ld
endif

OPTIONS += -D_DEBUG #-DNDEBUG

COMPILE_OPTS = -fdata-sections -ffunction-sections $(CPU_OPT) -mthumb -Wall -g -O0 -include stm32_include.h -D$(MCU) -D$(BOARD) $(OPTIONS)
INCLUDE_DIRS = -I$(ARM_CMSIS) -I$(STM32_CMSIS) -I$(MYLIB)
LIBRARY_DIRS = -L$(MYLIB) # needed to find the linker script

CFLAGS += $(COMPILE_OPTS) $(INCLUDE_DIRS)
CXXFLAGS += $(COMPILE_OPTS) $(INCLUDE_DIRS)
ASFLAGS += $(COMPILE_OPTS) -c
LDFLAGS += $(CPU_OPT) -mthumb -Wl,--gc-sections,-Map=$@.map,-cref,-u,Reset_Handler $(LIBRARY_DIRS) -T$(LD_SCRIPT)
OBJCPFLAGS +=

include $(MYLIB)/Makefile.inc

BINARIES=main

BIN_FILES=$(patsubst %,$(BINDIR)/%.bin,$(BINARIES))
CLEAN_FILES += $(BIN_FILES)

all: $(BINDIR)/main.bin

MYLIB_SRC=startup.c clock.c assert.c
MYLIB_OBJ=$(patsubst %.c, $(OBJDIR)/%.o, $(MYLIB_SRC))
$(foreach d,$(MYLIB_SRC),$(eval $(call make-obj-c,$d,$(MYLIB))))

MAIN_SRC=main.c mcp3914.c input_mcp3914.c
MAIN_OBJ=$(patsubst %.c, $(OBJDIR)/%.o, $(MAIN_SRC))
$(foreach d,$(MAIN_SRC),$(eval $(call make-obj-c,$d,.)))

ALL_OBJ=$(MAIN_OBJ) $(MYLIB_OBJ)

$(BINDIR)/main: $@ $(ALL_OBJ) $(MYLIB)/$(LD_SCRIPT)
	$(LD) $(LDFLAGS) -o $@ $(ALL_OBJ)

$(BINDIR)/main.bin: $(BINDIR)/main
	$(OBJCP) $(OBJCPFLAGS) $< $@

CLEAN_FILES += $(BINDIR)/main $(BINDIR)/main.bin $(BINDIR)/main.map

.PHONY: clean
clean:
	-rm $(CLEAN_FILES)
