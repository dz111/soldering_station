MCU := atmega328p
CLOCK := 16000000

CXXFLAGS = -c -g -Os -Wall -Wextra -std=c++20 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=$(MCU) -DF_CPU=$(CLOCK)L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
LDFLAGS = -Wall -Wextra -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=$(MCU) -lm

TARGET_EXEC := main
BUILD_DIR := .\build
SRC_DIR := .\src

_SRCS := $(shell dir $(SRC_DIR)\*.cpp /b)
_HEADERS := $(shell dir $(SRC_DIR)\*.h /b)
SRCS := $(_SRCS:%=$(SRC_DIR)\\%)
HEADERS := $(_HEADERS:%=$(SRC_DIR)\\%)
OBJS := $(_SRCS:%.cpp=$(BUILD_DIR)\\%.o)

PORT := COM3
BAUD := 115200

all: $(BUILD_DIR)\$(TARGET_EXEC).elf

$(BUILD_DIR)\\%.o: $(SRC_DIR)\%.cpp $(HEADERS)
	if not exist $(BUILD_DIR)\ ( mkdir $(BUILD_DIR) )
	avr-g++ $(CXXFLAGS) $< -o $@

$(BUILD_DIR)\$(TARGET_EXEC).elf: $(OBJS)
	avr-gcc $(LDFLAGS) -o $@ $(OBJS)
	@echo === Usage ===
	@echo Flash ROM 16k, using:
	@avr-size -A $@ | findstr "^.text"
	@echo SRAM 2k, using:
	@avr-size -A $@ | findstr "^.bss"
	@echo =============

$(BUILD_DIR)\$(TARGET_EXEC).hex: $(BUILD_DIR)\$(TARGET_EXEC).elf
	avr-objcopy -O ihex -R .eeprom $< $@

eeprom: $(BUILD_DIR)\$(TARGET_EXEC).elf
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $@ $(BUILD_DIR)\eeprom.eep

flash: $(BUILD_DIR)\$(TARGET_EXEC).hex
	avrdude -v -p $(MCU) -c arduino -P $(PORT) -b $(BAUD) -U flash:w:$<:i

$(BUILD_DIR)\$(TARGET_EXEC).as : $(BUILD_DIR)\$(TARGET_EXEC).elf
	avr-objdump -dS $< > $@

asm: $(BUILD_DIR)\$(TARGET_EXEC).as
	start "" notepad $<

clean:
	if exist $(BUILD_DIR)\ ( rmdir /S /Q $(BUILD_DIR)\ )
