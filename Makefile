PROJECT=thermostat

SRC = startup_stm32f072xb.s system_stm32f0xx.c stm32f0xx_hal_msp.c stm32f0xx_it.c main.c
INC = -Ilib/STM32F0xx_HAL_Driver/Inc -I. -Ilib/CMSIS/Include -Ilib/CMSIS/Device/ST/STM32F0xx/Include

DEVICE=STM32F072xB

export CC = arm-none-eabi-gcc
export OBJCOPY = arm-none-eabi-objcopy
export OBJDUMP = arm-none-eabi-objdump
export SIZE = arm-none-eabi-size

CFLAGS = -Wall -g -std=c99 -Os 
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections 
export CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJECT).map -D$(DEVICE)


.PHONY: lib $(PROJECT)

all: lib $(PROJECT)

lib:
	$(MAKE) -C lib

$(PROJECT): $(PROJECT).elf

$(PROJECT).elf: $(SRC)
	$(CC) $(CFLAGS) $(INC) $^ -o $@ -Llib -lstm32f0 -TSTM32F072RB_FLASH.ld
	$(OBJCOPY) -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin
	$(OBJDUMP) -St $(PROJECT).elf > $(PROJECT).lst
	$(SIZE) $(PROJECT).elf

program: $(PROJECT).bin
	openocd -f /usr/share/openocd/scripts/board/st_nucleo_f0.cfg st_nucleo_f0.cfg -f flash_stm.cfg -c "flash_stm $(PROJECT).bin"

clean:
	rm -f *.o
	rm -f $(PROJECT).elf
	rm -f $(PROJECT).bin
	rm -f $(PROJECT).map
	rm -f $(PROJECT).lst
	rm -f $(PROJECT).map
	$(MAKE) -C lib clean

