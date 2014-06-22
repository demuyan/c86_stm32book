GCC_BIN = 
PROJECT = hello
OBJECTS = ./main.o 
SYS_OBJECTS = \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/startup_STM32F40x.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/cmsis_nvic.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/board.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/retarget.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/system_stm32f4xx.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_pcd.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_ltdc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_rtc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_i2s.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_sram.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_ll_usb.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_smartcard.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_hash.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_sd.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_tim_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_i2s_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_sai.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_adc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_rcc_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/mbed_overrides.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_i2c_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_pwr.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_crc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dac.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_sdram.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_ll_fsmc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_rcc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_spi.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_ll_sdmmc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_flash.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_gpio.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dma.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_nand.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_nor.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_flash_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dma_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_hash_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_pccard.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_wwdg.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_uart.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_i2c.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_pwr_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dac_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_rtc_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_irda.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_tim.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_hcd.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_usart.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_adc_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_cryp.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_cortex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_ll_fmc.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dma2d.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_rng.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_can.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_cryp_ex.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_dcmi.o \
./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/stm32f4xx_hal_iwdg.o 

INCLUDE_PATHS = -I. -I./mbed -I./mbed/TARGET_NUCLEO_F401RE -I./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM -I./mbed/TARGET_NUCLEO_F401RE/TARGET_STM -I./mbed/TARGET_NUCLEO_F401RE/TARGET_STM/TARGET_NUCLEO_F401RE
LIBRARY_PATHS = -L./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM 
LIBRARIES = -lmbed 
LINKER_SCRIPT = ./mbed/TARGET_NUCLEO_F401RE/TOOLCHAIN_GCC_ARM/NUCLEO_F401RE.ld

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
CC_SYMBOLS = -DTARGET_NUCLEO_F401RE -DTARGET_M4 -DTARGET_STM -DTARGET_NUCLEO_F401RE -DTOOLCHAIN_GCC_ARM -DTOOLCHAIN_GCC -D__CORTEX_M4 -DARM_MATH_CM4 -D__FPU_PRESENT=1 -DMBED_BUILD_TIMESTAMP=1402409083.98 -D__MBED__=1 -DTARGET_FF_ARDUINO -DTARGET_FF_MORPHO

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


$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
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
