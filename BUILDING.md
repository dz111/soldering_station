
Download avr-gcc toolchain from https://github.com/ZakKemble/avr-gcc-build

Use the following batch script to establish the build environment.

```batch
@echo off
set PATH=%PATH%;D:\avr\bin
set PATH=%PATH%;D:\avr\avr\bin
set PATH=%PATH%;D:\avr\libexec\gcc\avr\14.1.0
```

Build as follows:

```
> make
avr-g++ [...] .\src\main.cpp -o .\build\main.o
avr-gcc []... -o .\build\main.elf .\build\main.o
=== Usage ===
Flash ROM 16k, using:
.text                      12500         0
SRAM 2k, using:
.bss                        1147   8388992
=============
> make flash
avr-objcopy -O ihex -R .eeprom .\build\main.elf .\build\main.hex
avrdude -v -p atmega328p -c arduino -P COM3 -b 115200 -U flash:w:.\build\main.hex:i

avrdude: Version 7.2
         Copyright the AVRDUDE authors;
[...]
avrdude: writing 12628 bytes flash ...
Writing | ################################################## | 100% 2.04 s
avrdude: 12628 bytes of flash written
avrdude: verifying flash memory against .\build\main.hex
Reading | ################################################## | 100% 1.62 s
avrdude: 12628 bytes of flash verified

avrdude done.  Thank you.
> make clean
if exist .\build\ ( rmdir /S /Q .\build\ )
>
```
