#!/bin/bash

boards=("" "LEIA=1" "WOOKEY=1" "DISCO=1" "DISCO407=1")
boards_label=("leia" "leia" "wookey" "disco" "disco407")

debug=("" "DEBUG=1")
debug_label=("nodebug" "debug")

forcetty=("" "FORCE_TTY_DEBUG=1")
forcetty_label=("noforcetty" "forcetty")

usb=("" "USB=0")
usb_label=("consoleusart" "consoleusb")

# This script performs the build matrix for the 'LEIA' firmware flavors
rm -f leia-artifacts.tar.gz
mkdir -p artifacts
for b in "${!boards[@]}"
do
    for d in "${!debug[@]}"
    do
        for ft in "${!forcetty[@]}"
        do
            for u in "${!usb[@]}"
            do
                COMMAND="${boards[b]} ${debug[d]} ${forcetty[ft]} ${usb[u]} make -j4"
                echo "================= $COMMAND"
                make clean && eval $COMMAND || exit 1
                # Save artifact
                if test -f leia-artifacts.tar; then
                    tar -cvf artifacts
                fi
                cp build/leia-halst.bin artifacts/leia-halst-"${boards_label[b]}"-"${debug_label[d]}"-"${forcetty_label[ft]}"-"${usb_label[u]}".bin
            done
        done
    done
done
tar acvf leia-artifacts.tar.gz artifacts
rm -rf artifacts
