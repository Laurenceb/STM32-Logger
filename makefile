# Compile the project

# Uncomment the appropriate device type and startup file
#DEVICE_TYPE = STM32F10X_LD
#STARTUP_FILE = stm32f10x_ld
DEVICE_TYPE = STM32F10X_MD
STARTUP_FILE = stm32f10x_md
#DEVICE_TYPE = STM32F10X_HD
#STARTUP_FILE = stm32f10x_hd
#DEVICE_TYPE = STM32F10X_CL
#STARTUP_FILE = stm32f10x_cl

# Set the external clock frequency
HSE_VALUE = 12000000UL

# Enable debug compilation
DEBUG = 1

# [OPTIONAL] Set the serial details for bootloading
STM32LDR_PORT = /dev/rfcomm3
STM32LDR_BAUD = 115200
# [OPTIONAL] Comment out to disable bootloader verification
STM32LDR_VERIFY = -v

# [OPTIONAL] Uncomment to use the firmware library
FWLIB = lib/STM32F10x_StdPeriph_Driver/libstm32fw.a
# [OPTIONAL] Uncomment to use the USB library
USBLIB = lib/STM32_USB-FS-Device_Driver/libstm32usb.a

# [OPTIONAL] Uncomment to link to maths library libm
LIBM = -lm

export DEBUG
export MESSAGES

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

INCLUDE_DIRS = -I . -I lib/STM32F10x_StdPeriph_Driver/inc\
 -I lib/CMSIS_CM3 -I lib/STM32F10x_StdPeriph_Driver\
 -I lib/STM32_USB-FS-Device_Driver/inc -I Util -I Sensors -I Util/USB\
 -I Util/fat_fs/inc

LIBRARY_DIRS = -L lib/STM32F10x_StdPeriph_Driver/\
 -L lib/STM32_USB-FS-Device_Driver

DEFINES = -D$(DEVICE_TYPE) -DHSE_Value=$(HSE_VALUE) -DCRT#-DSYSCLK_FREQ_72MHz=72000000

COMPILE_OPTS = $(WARNINGS) $(TARGET_OPTS) $(MESSAGES) $(INCLUDE_DIRS) $(DEFINES)
WARNINGS = -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -Winline

ifdef DEBUG
 TARGET_OPTS = -Os -g3
 DEBUG_MACRO = -DDEBUG
else	#Changed from O2 - optimisation split between control loop and rest of project, using a seperate makefile
 TARGET_OPTS = $(OPTIMISE) -finline -finline-functions-called-once\
  -funroll-loops -fno-common -fpromote-loop-indices -fno-rtti -fno-exceptions
endif

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
SIZE = arm-none-eabi-size
CFLAGS = -std=gnu99 $(COMPILE_OPTS) -flto -fuse-linker-plugin 
CXXFLAGS = $(COMPILE_OPTS)

AS = $(CC) -x assembler-with-cpp -c $(TARGET_ARCH)
ASFLAGS = $(COMPILE_OPTS)

LD = $(CC)
LDFLAGS = -Wl,--gc-sections,-Map=$(MAIN_MAP),-cref -T lib/TNT.ld -L lib\
 $(INCLUDE_DIRS) $(LIBRARY_DIRS) $(LIBM) -Os -flto -fuse-linker-plugin -fwhole-program -lnosys #-nostartfiles#-lstdc++

AR = arm-none-eabi-ar
ARFLAGS = cr

OBJCOPY = arm-none-eabi-objcopy
OBJCOPYFLAGS = -O binary

STARTUP_OBJ = lib/CMSIS_CM3/startup/gcc/startup_$(STARTUP_FILE).o

MAIN_OUT = main.elf
MAIN_MAP = $(MAIN_OUT:%.elf=%.map)
MAIN_BIN = $(MAIN_OUT:%.elf=%.bin)

MAIN_OBJS = $(sort \
 $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
 $(patsubst %.cc,%.o,$(wildcard *.cc)) \
 $(patsubst %.c,%.o,$(wildcard *.c)) \
 $(patsubst %.s,%.o,$(wildcard *.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS_CM3/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/USB/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/fat_fs/src/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/fat_fs/option/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Sensors/*.c)) \
 $(STARTUP_OBJ))

CONTROL_OBJS = $(sort $(patsubst %.c,%.o,$(wildcard Control/*.c)))

#optimisation
$(MAIN_OBJS): OPTIMISE= -Os

$(CONTROL_OBJS): OPTIMISE= -Os

#all - output the size from the elf
.PHONY: all
all: $(MAIN_BIN)
	$(SIZE) $(MAIN_OUT)

# main

$(MAIN_OUT): $(MAIN_OBJS) $(CONTROL_OBJS) $(FWLIB) $(USBLIB)
	$(LD) $(TARGET_ARCH) $^ -o $@ $(LDFLAGS)

$(MAIN_OBJS): $(wildcard *.h) $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)\
 $(wildcard lib/CMSIS_CM3/*.h)\
 $(wildcard Util*.h)\
 $(wildcard Util/USB*.h)\
 $(wildcard Util/fat_fs/inc/*.h)\
 $(wildcard Sensors*.h)

$(MAIN_BIN): $(MAIN_OUT)
	$(OBJCOPY) $(OBJCOPYFLAGS) $< $@

# fwlib

.PHONY: fwlib
fwlib: $(FWLIB)

$(FWLIB): $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE)

# usblib

.PHONY: usblib
usblib: $(USBLIB)

$(USBLIB): $(wildcard lib/STM32_USB-FS-Device_Driver/inc*.h)
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE)

#size

.PHONY: size
size: all
	@echo "Size:"
	$(SIZE) $(MAIN_OUT) $@
	@$(CAT) $@

# flash

.PHONY: flash
flash: flash-elf
#flash: flash-bin

.PHONY: flash-elf
flash-elf: all
	@cp $(MAIN_OUT) jtag/flash.elf
	@cd jtag && openocd -f flash-elf.cfg
	@rm jtag/flash.elf

.PHONY: flash-bin
flash-bin: all
	@cp $(MAIN_BIN) jtag/flash.bin
	@cd jtag && openocd -f flash-bin.cfg
	@rm jtag/flash.bin

.PHONY: upload
upload: all
	@python jtag/stm32loader.py -p $(STM32LDR_PORT) -b $(STM32LDR_BAUD)\
    -e $(STM32LDR_VERIFY) -w main.bin


# clean

.PHONY: clean
clean:
	-rm -f $(MAIN_OBJS) $(MAIN_OUT) $(MAIN_MAP) $(MAIN_BIN)
	-rm -f $(CONTROL_OBJS)
	-rm -f jtag/flash.elf jtag/flash.bin
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE) clean
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE) clean

