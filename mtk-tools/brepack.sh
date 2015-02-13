#!/bin/bash
#Stop script if something is broken
set -e

#./mkimage kernel_lenovo89_wet_td.bin KERNEL > zImage

./mkbootfs ./boot.img-ramdisk/ | gzip >ramdisk.gz
./mkimage ramdisk.gz ROOTFS > ramdisk.img

#./mkbootimg --kernel zImage --ramdisk ramdisk.img -o boot.img
./mkbootimg --kernel kernel_lenovo89_wet_td.bin --ramdisk ramdisk.img -o boot.img

