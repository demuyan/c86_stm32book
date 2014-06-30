GCC_BIN = 
PROJECT = hello
OBJECTS = vectors.o hello.o rcc.o

INCLUDE_PATHS = -I. 
LINKER_SCRIPT = ./NUCLEO_F401RE.ld

############################################################################### 
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE 	= $(GCC_BIN)arm-none-eabi-size

CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
CC_FLAGS = $(CPU) -Wall -Werror -O2 -c -g

CC_SYMBOLS = -DTARGET_NUCLEO_F401RE -DTARGET_M4 -DTARGET_STM -DTARGET_NUCLEO_F401RE -DTOOLCHAIN_GCC_ARM -DTOOLCHAIN_GCC -D__CORTEX_M4 -DARM_MATH_CM4 -D__FPU_PRESENT=1 -DTARGET_FF_ARDUINO -DTARGET_FF_MORPHO

LD_FLAGS  = -mcpu=cortex-m4 -mthumb -nostdlib -nostartfiles -ffreestanding 
LD_FLAGS += -Wl,-Map=$(PROJECT).map,--cref

ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -O2
endif

all: $(PROJECT).bin $(PROJECT).hex $(PROJECT).lst size

clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJECTS)

.s.o:
	$(AS) $(CPU) -o $@ $<

.c.o:
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	@$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

size:
	$(SIZE) $(PROJECT).elf

flash:
	st-flash write $(PROJECT).bin 0x8000000
