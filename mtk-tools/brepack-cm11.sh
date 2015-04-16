#!/bin/bash
#Stop script if something is broken
set -e

DEVICE_TREE="`cat ../DEVICE_TREE`"

./mkbootfs ./cm11/boot.img-ramdisk/ | gzip > ramdisk.gz
./mkimage ramdisk.gz ROOTFS > ramdisk.img
./mkbootimg --kernel ../out/target/product/"$DEVICE_TREE"/kernel_"$DEVICE_TREE".bin --ramdisk ramdisk.img -o boot.img

rm ramdisk.gz
rm ramdisk.img

