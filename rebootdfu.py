#!/usr/bin/env python
# Part of this file is an extract of the OpenMV project.

import usb
import struct

idProduct = 0xDF11
idVendor = 0x0483

# DFU status
__DFU_STATE_DFU_DOWNLOAD_BUSY = 0x04
__DFU_STATE_DFU_DOWNLOAD_IDLE = 0x05
__DFU_STATE_DFU_MANIFEST = 0x07

__dev = None

def get_status():
    return __dev.ctrl_transfer(0xA1, 3, 0, 0, 6, 20000)


def open():
    global __dev

    l = list(usb.core.find(find_all=True))

    for d in l:
        if d.idVendor == idVendor and d.idProduct == idProduct:
            print("Device found.")
            __dev = d

    __dev.set_configuration()
    usb.util.claim_interface(__dev, 0)

open()

buf = struct.pack("<BI", 0x21, 0x08000000)


__dev.ctrl_transfer(0x21, 1, 0, 0, buf, 4000)

get_status() 
get_status()

__dev.ctrl_transfer(0xA1, 1, 2, 0, None, 4000)

get_status() 
