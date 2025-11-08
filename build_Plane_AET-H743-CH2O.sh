#! /bin/bash

set -e

./waf configure --board AET-H743-CH2O --bootloader
./waf clean
./waf bootloader
cp build/AET-H743-CH2O/bin/AP_Bootloader.bin Tools/bootloaders/AET-H743-CH2O_bl.bin
./waf configure --board AET-H743-CH2O
./waf clean
./waf plane
cp build/AET-H743-CH2O/bin/arduplane_with_bl.hex /mnt/c/Users/villivateur/Desktop/arduplane_with_bl_AET-H743-CH2O.hex
cp build/AET-H743-CH2O/bin/arduplane.apj /mnt/c/Users/villivateur/Desktop/arduplane_AET-H743-CH2O.apj
