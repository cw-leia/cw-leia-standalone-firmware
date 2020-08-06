[![pipeline status](https://gitlab.reseau-libre.net/leia/firmware/badges/master/pipeline.svg)](https://gitlab.reseau-libre.net/leia/firmware/-/commits/master)

# LEIA Firmware

This repository hosts the source code files of the LEIA firmware. With them, 
you can (re)build the firmware to adapt it to your needs. To do so several 
compilation options are detailed below.

## Build options

| Option         | Environment variable | Description                          |
|----------------|----------------------|--------------------------------------|
| Board Choice   | LEIA=1 (default)     | This is basically the board choice.  |
|                | WOOKEY=1             | For now, LEIA boards, WooKey boards  |
|                | DISCO407=1           | and F407 / F429 discovery boards are |
|                | DISCO429=1           | supported. Depending on the specific |
|                |                      | board selected, the pins mapping     |
|                |                      | will vary and must be adapted.       |
|                |                      |                                      |
|----------------|----------------------|--------------------------------------|
| Communication  | USB=1 (default)      | This is the prefered channel used to |
|                | USB=0                | communicate with the board. By       |
|                |                      | default, two USB /dev/ttyACMx are    |
|                |                      | used for protocol and debug purposes.|
|                |                      | USB=0 will make use of dedicated     |
|                |                      | physical UARTs on the board (with    |
|                |                      | dedicated pins).                     |
|----------------|----------------------|--------------------------------------|
| ISO7816 driver | BITBANG=0 (default)  | This is an ISO7816 specific option.  |
|                | BITBANG=1            | This will select either the use of   |
|                |                      | a physical USART for the ISO7816     |
|                |                      | communication or bitbanging GPIOs.   |
|                |                      | Each option has advantages and       |
|                |                      | drawbacks:                           |
|                |                      | - USART: since ISO7816 is hardware   |
|                |                      |   accelerated, this is fast and can  |
|                |                      |   support high frequency rates. On   |
|                |                      |   the other side, handling low level |
|                |                      |   protocol modifications (for bits   |
|                |                      |   reception and so on) is complex    |
|                |                      |   (sometimes impossible).            |
|                |                      | - Bitbang: this allows very low level|
|                |                      |   handling of the protocol, yielding |
|                |                      |   in easier modifications (for       |
|                |                      |   conformance tests, glitches, etc.).|
|                |                      |   The drawback is that the clock     |
|                |                      |   frequency must be kept at very low |
|                |                      |   rates (~1MHz to 4MHz) so that the  |
|                |                      |   MCU can keep up with bitbanging    |
|                |                      |   the protocol pins properly.        |
|                |                      |                                      |
|                |                      | NOTE: Bitbanging is not compatible   |
|                |                      | with the WooKey board because of some|
|                |                      | limitations on its pins. You can     |
|                |                      | however tune the WooKey configuration|
|                |                      | files to dedicate pins to Bitbang    |
|                |                      | (at the expanse of using an external |
|                |                      | ISO7816 pin adapter, not the one     |
|                |                      | embedded on the board).              |
|                |                      |                                      |
|----------------|----------------------|--------------------------------------|
| Debug          | DEBUG=0 (default)    | This activates or desactivates       |
|                | DEBUG=1              | the debug console. This console shows|
|                |                      | the protocol commands and responses  |
|                |                      | and can help when debugging issues.  |
|                |                      |                                      |
|----------------|----------------------|--------------------------------------|
| PLL override   | HSE_PLLM_OVERRIDE=XX | This option should be manipulated    |
|                |                      | with care as it can break the        |
|                |                      | firmware of existing boards. This    |
|                |                      | option should be used when adding a  |
|                |                      | new board or when using a flavour    |
|                |                      | of the existing boards that use a    |
|                |                      | crystal (HSE external oscillator)    |
|                |                      | that is not on par with the public   |
|                |                      | schematics of the boards.            |
|                |                      | This modifies the internal value     |
|                |                      | of the HSE (external clock) of the   |
|                |                      | MCU in MHz (e.g. HSE_PLLM_OVERRIDE=8 |
|                |                      | means a clock at 8MHz).              |
|                |                      |                                      |
|----------------|----------------------|--------------------------------------|

## Dependencies

The main compilation dependency is an `arm-none-eabi` toolchain that should
be packaged with your favourite distro, allowing bare metal firmware compilation
for ARM v7m (Cortex-M) microcontrollers.

The other required dependencies are mainly for flashing the firmware, and will depend
on the board you target:
  - For LEIA boards where the DFU protocol can be used, the `dfu-util` tool must
  be installed and used (this should be packaged in your distro).
  - For the other boards where SWD is used to flash the firmware, the `st-flash`
  tool is necessary. This is packaged on some distros, but can also be compiled from
  sources. An alternative way of flashing firmwares using SWD is to use `openocd`, this
  is not in the current Makefile and is left as a side work for those who prefer to use
  this tool.

## Compilation

Compilation is as simple as:

```sh
FLAGS=... make
```

where `FLAGS` are the ones previusly specified.

If you want to build a production firmware for the LEIA board using the bitbang
version:

```sh
BITBANG=1 make
```

And if you want to add the debug interface:

```sh
DEBUG=1 BITBANG=1 make
```

The end of the compilation should grant you with a summary of the firmware chosen
options, for example with a `make` with no specific option:

```sh
$ make
...
   text	   data	    bss	    dec	    hex	filename
  30808	    840	  46828	  78476	  1328c	build/leia-halst.elf
   [+] **LEIA** board selected (default) 
      ---------------------------------
   [+] Protocol on USB USART abstraction
       (this is the default protocol and debug consoles channel)
   [+] ISO7816 using USART (default)
```

When compiling with `DISCO407=1 USB=0 BITBANG=1 DEBUG=1` options:
```sh
$ DISCO407=1 USB=0 BITBANG=1 DEBUG=1 make
...
   text	   data	    bss	    dec	    hex	filename
  36992	    680	  48332	  86004	  14ff4	build/leia-halst.elf
   [+] **DISCO** board selected
          (DISCO F407 board)
      ---------------------------------
   [+] DEBUG activated
       (debug on physical UART)
   [+] Protocol on physical UART
   [+] ISO7816 using BITBANG
```

## Programming the boards

On LEIA boards, you can use the DFU mode of the STM32F4 bootrom. To activate it,
you need to:

    1. Connect LEIA to one of your USB ports
    2. Hold the reset button
    3. Press *also* the DFU button
    4. Release the reset button while keeping the DFU button pressed for 1 second
    5. Run `make dfu` from your favorite shell

On the WooKey board, you will have to use SWD (Single Wire Debug) protocol to flash
the board, using e.g. an ST-Link V2 connector connected to the proper pins. Once 
connected, you can use the `make burn` target to burn the firmware in the board.

On the DISCO boards (STM32 Discovery F407 and F429), an ST-Link V2 chip is already
present on the board and you should be able to directly run `make burn` to flash
the boards. Beware that the mini-USB port is used for flashing, and the micro-USB
OTG port is used for LEIA protocol and debug /dev/ttyACMx interfaces.

# Boards specific pinouts

The supported boards are LEIA, WOOKEY (WooKey board), DISCO407 (STM32 Discovery F407) and
DISCO429 (STM32 Discovery F429). Other boards can be added by modifying the source code,
mainly by adapting the configuration files. The pinout is not the same depending on the
board, here is a summary:

| Board   | ISO7816 USART   | ISO7816 Bitbang  | Protocol USART | Debug USART    |
|---------|-----------------|------------------|----------------|----------------|
| LEIA    | Reset:    PC12  | Reset:    PC12   |  TX: PA0       | TX: PB6        |
|         | Vcc  :    PD7   | Vcc  :    PD7    |  RX: PA1       | RX:  -         |
|         | I/O  :    PA2   | I/O  :    PA2    |                |                |
|         | I/O dir:  PE3   | I/O dir:  PE3    |                | NOTE: no RX pin|
|         | Clock:    *PA4* | Clock:    *PA3*  |                | since debug is |
|         | Contact:  PC11  | Contact:  PC11   |                | monodirectional|
|         |                 |                  |                |                |
|         | (optional pins) | (optional pins)  |                |                |
|         | Vpp  :    PC10  | Vpp  :    PC10   |                |                |
|         | Aux1 :    PE5   | Aux1 :    PE5    |                |                |
|         | Aux2 :    PE6   | Aux2 :    PE6    |                |                |
|         |                 |                  |                |                |
|         | NOTE: I/O dir is| NOTE: I/O dir is |                |                |
|         | used for opto-  | used for opto-   |                |                |
|         | coupling.       | coupling.        |                |                |
|         |                 |                  |                |                |
|---------|-----------------|------------------|----------------|----------------|
| WOOKEY  | Reset:    PE3   |      -           |  TX: PA0       | TX: PB6        |
|         | Vcc  :    PD7   |                  |  RX: PA1       | RX:  -         |
|         | I/O  :    PA2   | NOTE: WooKey does|                |                |
|         | I/O dir:   -    | not support      |                | NOTE: no RX pin|
|         | Clock:    PA4   | Bitbang as is.   |                | since debug is |
|         | Contact:  PE2   |                  |                | monodirectional|
|         |                 |                  |                |                |
|         | (optional pins) |                  |                |                |
|         | Vpp  :    PE7   |                  |                |                |
|         | Aux1 :    PE5   |                  |                |                |
|         | Aux2 :    PE6   |                  |                |                |
|         |                 |                  |                |                |
|---------|-----------------|------------------|----------------|----------------|
| DICO:   | Reset:    PE3   | Reset:    PE3    |  TX: PC10      | TX: PB6        |
|         | Vcc  :    PD7   | Vcc  :    PD7    |  RX: PC11      | RX:  -         |
|DISCO407 | I/O  :    PA2   | I/O  :    PA2    |                |                |
|  and    | I/O dir:   -    | I/O dir:   -     |                | NOTE: no RX pin|
|DISCO429 | Clock:    *PA4* | Clock:    *PA3*  |                | since debug is |
|         | Contact:  PE2   | Contact:  PE2    |                | monodirectional|
|         |                 |                  |                |                |
|         | (optional pins) | (optional pins)  |                |                |
|         | Vpp  :    PE7   | Vpp  :    PE7    |                |                |
|         | Aux1 :    PE5   | Aux1 :    PE5    |                |                |
|         | Aux2 :    PE6   | Aux2 :    PE6    |                |                |
|         |                 |                  |                |                |
|---------|-----------------|------------------|----------------|----------------|

NOTE1: reggarding UARTs, TX and RX are board centric, meaning that TX means data
from board to PC host, and RX means data from PC host to board.

NOTE2: "Protocol USART" and "Debug USART" are only available when selecting
the `USB=0` toggle (and `DEBUG=1` specifically for the "Debug USART"). By
default (`USB=1`), protocol and debug communication channels use the USB FS
(Full Speed) connector and show up as /dev/ttyACMx and /dev/ttyACMy devices
on the PC host side. Usually, /dev/ttyACMx is the protocol console and 
/dev/ttyACMy is the debug console when x < y.

