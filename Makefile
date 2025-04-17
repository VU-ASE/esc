
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

check-sudo:
	@if [ "$$EUID" -ne 0 ]; then \
		echo "This target must be run as root (use sudo)."; \
		exit 1; \
	fi

# Takes the firmware currently on the board and dumps it into a binary file
dump: check-sudo
	@mkdir -p firmware
	st-flash read firmware/some-esc-firmware.bin 0x8000000 0x20000

# Flashes the compiled firmware and flashes it on the connected board
flash: check-sudo info
	@mkdir -p firmware
	st-flash write firmware/esc-firmware.bin 0x8000000

# Builds the source code into the .pio folder
# (but this does not flash it yet, nor does it produce a firmware.bin file)
build:
	platformio run

flash-manual:
	platformio run --target upload



