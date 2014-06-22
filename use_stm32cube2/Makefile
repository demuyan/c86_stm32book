GCC_BIN = 
PROJECT = blink
OBJECTS = \
 src/startup_stm32f401xe.o \
 src/main.o \
 src/main.o \
 src/stm32f4xx_it.o \
 driver/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.o \
 driver/CMSIS/system_stm32f4xx.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.o \
 driver/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.o

INCLUDE_PATHS = -I/src \
  -I./include \
  -I$(HOME)/src/STM32Cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
  -I$(HOME)/src/STM32Cube/Drivers/STM32F4xx_HAL_Driver/Inc \
  -I$(HOME)/src/STM32Cube/Drivers/BSP/STM32F4xx-Nucleo \
  -I$(HOME)/src/STM32Cube/Drivers/CMSIS/Include \
# LIBRARY_PATHS = -L./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM 
# LIBRARIES = -lmbed 
LINKER_SCRIPT = src/STM32F401CE_FLASH.ld

############################################################################### 
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE 	= $(GCC_BIN)arm-none-eabi-size

CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
CC_FLAGS = $(CPU) -c -g -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections 
CC_SYMBOLS = -DUSE_HAL_DRIVER -DSTM32F401xE -DUSE_STM32F4XX_NUCLEO

LD_FLAGS  = -mcpu=cortex-m4 -mthumb -Wl,--gc-sections --specs=nano.specs -u _printf_float -u _scanf_float
LD_FLAGS += -Wl,-Map=$(PROJECT).map,--cref
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys

ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -O2
endif

all: $(PROJECT).bin $(PROJECT).hex size

clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJECTS)

.s.o:
	$(AS) $(CPU) -o $@ $<

.c.o:
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).elf: $(OBJECTS) 
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS)

$(PROJECT).bin: $(PROJECT).elf
	@$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size:
	$(SIZE) $(PROJECT).elf

flash:
	st-flash write $(PROJECT).bin 0x8000000
