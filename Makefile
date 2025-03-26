
# STM32 boards start flash memory at address 0x8000000
# and this board has 0x20000 (128Kb) of flash available


info:
	@echo ""
	@echo ""
	@echo "--- IMPORTANT: ----"
	@echo "Left ESC plug into channel 1 and requires any two motor-esc wires to not match color"
	@echo "Right ESC plug into channel 2 and requires motor-esc wires to match color"
	@echo ""
	@echo ""



dump:
	@mkdir -p firmware
	st-flash read firmware/some-esc-firmware.bin 0x8000000 0x20000

flash: info
	@mkdir -p firmware
	st-flash write firmware/esc-firmware.bin 0x8000000



