# vscp-demo-link-stm32f103-wiz-ip20-blinky
[VSCP blinky demo](https://github.com/grodansparadis/vscp-firmware/wiki) for STM32F103C6T6 "Blue Pill" with [WIZnet IP20 module](https://wiznet.io/products/serial-to-ethernet-modules/wiz-ip20). The firmware is built with CMake and STM32CubeMX-generated code, using the GNU Arm Embedded Toolchain. It demonstrates basic GPIO control for blinking an LED and configures and sets up network connectivity. 

VSCP blink is the simplest possible VSCP application, toggling an LED on and off at a configurable interval. This project serves as a starting point for developing more complex VSCP applications on STM32 microcontrollers with (Ethernet) connectivity. The VSCP blink demo is designed to be simple and easy to understand, making it ideal for learning how to use the VSCP protocol and develop applications for embedded systems. It is implemented on different platforms, including STM32 microcontrollers, and can be used as a reference for building your own VSCP applications.

## Hardware
- [STM32F103C6T6 "Blue Pill" style development board](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) (same board layout as the C8T6 Blue Pill, but with 32 KB flash / 10 KB RAM)
- [WIZnet W5500 Ethernet module](https://www.wiznet.io/product-item/w5500/) (SPI interface)
- [ST-Link V2 programmer/debugger](https://www.st.com/en/development-tools/st-link-v2.html)
- [WIZ-IP20 IO Module](https://docs.wiznet.io/Product/Modules/Serial-to-Ethernet-Module/WIZ-IP20/ip20-io)
- [W55RP20-S2E Command Manual](https://docs.wiznet.io/Product/Chip/MCU/Pre-programmed-MCU/W55RP20-S2E/command-manual-en)
- [WIZ-IP20 Product Page](https://wiznet.io/products/serial-to-ethernet-modules/wiz-ip20)
- [WIZnet S2E Tool GUI Getting Started Guide](https://github.com/Wiznet/WIZnet-S2E-Tool-GUI/wiki/Getting-started-guide_en)

### Resources

* TIM2 is used as a millisecond timebase.
* TIM3 is used as a 1 µs free-running counter, extended to 32 bits via software overflow.

## Memory Constraints

> ⚠️ The STM32F103C6T6 has only **32 KB flash / 10 KB RAM**, and the firmware
> region (30 KB after reserving 2 KB for persistent storage) is almost full:
> a Release build currently uses about **30.2 KB (≈98 %)**, leaving only a few
> hundred bytes of headroom. RAM usage is about 4.8 KB (≈47 %).

To make the VSCP link protocol fit on this part, the project deliberately
trades features and libc conveniences for size:

- Both Debug and Release builds compile with `-Os` and LTO
  ([cmake/gcc-arm-none-eabi.cmake](firmware/cmake/gcc-arm-none-eabi.cmake)).
  A `-O0` Debug build does **not** fit.
- `printf()` must not be used. Debug output goes through the `LOGSTR()` macro
  ([vscp-projdefs.h](firmware/vscp-projdefs.h)), which is a no-op by default.
  A single `printf()`/`rand()` call drags in several KB of newlib stdio code.
- [Core/Src/libc_shim.c](firmware/Core/Src/libc_shim.c) provides small
  `strstr`/`strcmp` replacements instead of newlib's larger optimized versions.
- The VSCP AES, CRC and binary-protocol modules are disabled in
  [firmware/CMakeLists.txt](firmware/CMakeLists.txt).

When adding new code, watch the `Memory region` summary printed at the end of
every link. If the project outgrows the part, the pin-compatible
STM32F103**C8**T6 (64 KB flash / 20 KB RAM) fits the full firmware with ~45 %
flash to spare; it requires retargeting the linker script, the startup file
(`startup_stm32f103xb.s`), the `STM32F103xB` HAL define and the storage base
address.

## Persistent Flash Storage

2 KB of internal flash is reserved for persistent storage (registers, configuration) using the last two pages of the 32 KB flash:

| Region  | Start        | End          | Size  | Pages |
|---------|-------------|--------------|-------|-------|
| Firmware | `0x08000000` | `0x080077FF` | 30 KB | 0–29  |
| Storage  | `0x08007800` | `0x08007FFF` |  2 KB | 30–31 |

The linker script ([STM32F103XX_FLASH.ld](firmware/STM32F103XX_FLASH.ld)) reduces the `FLASH` region to 30 KB and defines a separate `STORAGE` region. The driver is in [Core/Inc/flash_storage.h](firmware/Core/Inc/flash_storage.h) and [Core/Src/flash_storage.c](firmware/Core/Src/flash_storage.c).

Key constraints:
- A full page (1 KB) must be **erased** before any byte in it can be written
- Writes are **16-bit (half-word)** aligned
- Flash endurance: ~**10 000** erase cycles per page

```c
#include "flash_storage.h"

// Erase both storage pages (required before first write)
flash_storage_erase();

// Write two half-words at offset 0
uint16_t cfg[] = { 0x0001, 0x1234 };
flash_storage_write(0, cfg, 2);

// Read back
uint16_t buf[2];
flash_storage_read(0, buf, 2);
```

### Setup
- Connect the W5500 to the STM32F103C6T6 according to the following pinout:
  - W5500 SCK -> PA5
  - W5500 MISO -> PA6
  - W5500 MOSI -> PA7
  - W5500 CS -> PB6
  - W5500 RST -> PB7
- Connect the ST-Link V2 to the STM32F103C6T6 for programming and debugging:
  - ST-Link SWCLK -> PA14
  - ST-Link SWDIO -> PA13
  - ST-Link GND -> GND
  - ST-Link VCC -> 3.3V (optional, can power the board from USB or external source)
- Connect an LED with a suitable resistor to PB0 for blinking (or use the onboard LED if available)
- Debug output is available on PA2 (USART2 TX) for printf debugging. Connect a serial TTL to USB adapter to PA2 and GND to view debug output on your computer.


## Build the project

Check out the repository

```bash
  git clone --recurse-submodules https://github.com/grodansparadis/vscp-demo-link-stm32f103-wiz-ip20-blinky.git
```

Go to the firmware directory

```bash
cmake -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build -j$(nproc)
```

This produces build/firmware.elf and .hex/.bin.


## Flashing the project

```bash
# OpenOCD
openocd -f interface/stlink.cfg -f target/st m32f1x.cfg \
  -c "program build/firmware.elf verify reset exit"

# ST-Link CLI (install with 'sudo apt install stlink-tools')
st-flash write build/firmware.bin 0x08000000
```

### Build Tips

- Regenerating code — CubeMX overwrites CMakeLists.txt files in cmake/stm32cubemx/ but leaves your top-level alone; keep custom code in Core/Src and your own cmake files
- VS Code — install the CMake Tools extension, point it at the toolchain file via cmake.configureSettings in settings.json
- Ninja — add -G Ninja to the cmake command for faster builds

### Testing ST_Link CLI

```bash

# Check version of st-flash
st-flash --version

# Example reply
v1.8.0

# Check connected ST-Link devices 
st-info --probe      # detects connected ST-Link

# Example reply
Found 1 stlink programmers
  version:    V2J23S9
  serial:     066EFF545454885087241237
  flash:      524288 (pagesize: 16384)
  sram:       98304
  chipid:     0x433
  dev-type:   STM32F401xD_xE
``` 

## Debugging

Use OpenOCD with GDB:

```bash
# Start OpenOCD in one terminal
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg

# In another terminal, connect with GDB
arm-none-eabi-gdb build/firmware.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) continue

# You can set breakpoints, inspect variables, etc. in GDB

# Alternatively use 'st-util' for a more user-friendly interface
# but with less features than OpenOCD
st-util
```
