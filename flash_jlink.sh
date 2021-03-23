#!/bin/sh

openocd -f stm32f4disco_jlink.cfg -c "init" -c "reset halt" -c "flash write_image erase build/leia-halst.hex" -c "reset" -c "shutdown"
