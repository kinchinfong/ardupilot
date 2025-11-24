#! /bin/bash

set -e

./waf configure --board AET-H743-Basic --bootloader
./waf clean
./waf bootloader
cp build/AET-H743-Basic/bin/AP_Bootloader.bin Tools/bootloaders/AET-H743-Basic_bl.bin
./waf configure --board AET-H743-Basic
./waf clean
./waf plane
cp build/AET-H743-Basic/bin/arduplane_with_bl.hex /mnt/c/Users/villivateur/Desktop/arduplane_with_bl_AET-H743-Basic.hex
cp build/AET-H743-Basic/bin/arduplane.apj /mnt/c/Users/villivateur/Desktop/arduplane_AET-H743-Basic.apj
