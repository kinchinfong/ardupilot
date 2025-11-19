#! /bin/bash

set -e

./waf configure --board VVZERO-H743-DEV --bootloader
./waf clean
./waf bootloader
cp build/VVZERO-H743-DEV/bin/AP_Bootloader.bin Tools/bootloaders/VVZERO-H743-DEV_bl.bin
./waf configure --board VVZERO-H743-DEV
./waf clean
./waf plane
cp build/VVZERO-H743-DEV/bin/arduplane_with_bl.hex /mnt/c/Users/villivateur/Desktop/arduplane_with_bl_VVZERO-H743-DEV.hex
#cp build/VVZERO-H743-DEV/bin/arduplane.apj /mnt/c/Users/villivateur/Desktop/arduplane_VVZERO-H743-DEV.apj
