SRC_DIR   = ./
BUILD_DIR = build

LIBISO7816_SRC_DIR = $(SRC_DIR)/iso7816
LIBISO7816_C_SOURCES = $(call wildcard, $(LIBISO7816_SRC_DIR)/*.c)
LIBISO7816_ASM_SOURCES = $(call wildcard, $(LIBISO7816_SRC_DIR)/*.s) $(call wildcard, $(LIBISO7816_SRC_DIR)/*.S)
LIBISO7816_OBJECTS = $(patsubst $(LIBISO7816_SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(LIBISO7816_C_SOURCES))
LIBISO7816_OBJECTS += $(patsubst $(LIBISO7816_SRC_DIR)/%.s, $(BUILD_DIR)/%.o, $(LIBISO7816_ASM_SOURCES)) $(patsubst $(LIBISO7816_SRC_DIR)/%.S, $(BUILD_DIR)/%.o, $(LIBISO7816_ASM_SOURCES))

# Bare metal 'OS' sources for the STM32 Discovery F409 board
OS_SRC_DIR = $(SRC_DIR)/stm32discovery_F409/
OS_C_SOURCES = $(call wildcard, $(OS_SRC_DIR)/*.c)
OS_ASM_SOURCES = $(call wildcard, $(OS_SRC_DIR)/*.s) $(call wildcard, $(OS_SRC_DIR)/*.S)
OS_OBJECTS = $(patsubst $(OS_SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(OS_C_SOURCES))
OS_OBJECTS += $(patsubst $(OS_SRC_DIR)/%.s, $(BUILD_DIR)/%.o, $(OS_ASM_SOURCES)) $(patsubst $(OS_SRC_DIR)/%.S, $(BUILD_DIR)/%.o, $(OS_ASM_SOURCES))

BARE_METAL_GCC=arm-none-eabi-gcc
BARE_METAL_AR=arm-none-eabi-ar
BARE_METAL_OBJCOPY=arm-none-eabi-objcopy

MEM_LAYOUT_DEF=$(OS_SRC_DIR)/layout.def
MEM_LAYOUT=$(BUILD_DIR)/layout.ld

STFLASH=st-flash

CFLAGS = -nostdlib -g3 -DDEBUG_LVL=0 -Os -mthumb-interwork -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS = -nostdlib -g3 -Os -mthumb-interwork -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

ifeq ($(WOOKEY),1)
CFLAGS += -DWOOKEY
endif
ifeq ($(LEIA),1)
CFLAGS += -DLEIA
endif

all: firmware

# Compile the OS sources
embedded_os_lib:	
	@mkdir -p $(BUILD_DIR)
	$(BARE_METAL_GCC) $(CFLAGS) -I$(OS_SRC_DIR) $(OS_C_SOURCES) $(OS_ASM_SOURCES) -c	
	@mv *.o $(BUILD_DIR)
	$(BARE_METAL_AR) rcs $(BUILD_DIR)/libembeddedos.a $(OS_OBJECTS)

# Compile the ISO7816 lib
iso7816_lib:
	@mkdir -p $(BUILD_DIR)
	$(BARE_METAL_GCC) $(CFLAGS) -I$(OS_SRC_DIR) -I$(SRC_DIR)/src -I$(LIBISO7816_SRC_DIR) $(LIBISO7816_C_SOURCES) $(LIBISO7816_ASM_SOURCES) -c
	@mv *.o $(BUILD_DIR)
	$(BARE_METAL_AR) rcs $(BUILD_DIR)/libiso7816.a $(LIBISO7816_OBJECTS)

firmware: embedded_os_lib iso7816_lib 
	# Generate memory layout
	$(BARE_METAL_GCC) -E -x c $(MEM_LAYOUT_DEF) -I$(OS_SRC_DIR) | grep -v '^#' > $(MEM_LAYOUT)
	cp  $(OS_SRC_DIR)/standalone.ld $(BUILD_DIR)/
	# Link elements
	$(BARE_METAL_GCC) $(CFLAGS) $(LDFLAGS) -I$(OS_SRC_DIR) -I$(LIBISO7816_SRC_DIR) -I$(SRC_DIR)/src $(SRC_DIR)/main.c $(SRC_DIR)/src/*.c  $(BUILD_DIR)/libiso7816.a $(BUILD_DIR)/libembeddedos.a -T$(BUILD_DIR)/standalone.ld -o $(BUILD_DIR)/firmware.elf
	# Generate the hex file
	$(BARE_METAL_OBJCOPY) -O ihex $(BUILD_DIR)/firmware.elf $(BUILD_DIR)/firmware.hex
	$(BARE_METAL_OBJCOPY) -I ihex --output-target=binary $(BUILD_DIR)/firmware.hex $(BUILD_DIR)/firmware.bin

burn:
	$(STFLASH) write $(BUILD_DIR)/firmware.bin 0x8000000

clean:
	@rm -rf *.o $(BUILD_DIR)/
