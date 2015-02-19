#!/bin/bash
#Stop script if something is broken
set -e

./mkbootfs ./boot.img-ramdisk/ | gzip > ramdisk.gz
./mkimage ramdisk.gz ROOTFS > ramdisk.img
./mkbootimg --kernel kernel_lenovo_a820.bin --ramdisk ramdisk.img -o boot.img

