.PHONY: build
build:
	mkdir -p build
	cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_TOOLCHAIN_FILE=/opt/stm32/toolchain/stm32f103x8.cmake ..
	cmake --build build
	sed -i 's/arm-none-eabi-//g' build/compile_commands.json

.PHONY: clean
clean:
	rm -r build

.PHONY: flash
flash:
	stm32prog -c port=swd -w build/example.elf 0x08000000 -g 0x08000000

.PHONY: console
console:
	stm32prog -c port=/dev/ttyACM0 console

.PHONY: swv
swv:
	stm32prog -c port=swd -startswv freq=72 portnumber=all
