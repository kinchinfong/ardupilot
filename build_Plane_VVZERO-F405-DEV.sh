#! /bin/bash

set -e

./waf configure --board VVZERO-F405-DEV --bootloader
./waf clean
./waf bootloader
cp build/VVZERO-F405-DEV/bin/AP_Bootloader.bin Tools/bootloaders/VVZERO-F405-DEV_bl.bin
./waf configure --board VVZERO-F405-DEV
./waf clean
./waf plane
cp build/VVZERO-F405-DEV/bin/arduplane_with_bl.hex /mnt/c/Users/villivateur/Desktop/arduplane_with_bl_VVZERO-F405-DEV.hex
#cp build/VVZERO-F405-DEV/bin/arduplane.apj /mnt/c/Users/villivateur/Desktop/arduplane_VVZERO-F405-DEV.apj
