flash:
	openocd -f nucleo.cfg -c "flash_program $(BUILD_DIR)/$(TARGET).hex"

read-tty:
	cu -s 115200 -l /dev/ttyACM0

gdb-server:
	openocd -f board/st_nucleo_f4.cfg

gdb:
	/usr/bin/arm-none-eabi-gdb  $(BUILD_DIR)/$(TARGET).elf

.PHONY: flash read-tty gdb-server gdb

