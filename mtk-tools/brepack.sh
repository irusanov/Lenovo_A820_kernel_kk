#!/bin/bash
#Stop script if something is broken
set -e

if [ -f kernel.conf ]
  then
    source "kernel.conf"
  else
	echo "Kernel configuration file (kernel.conf) does not exist!"
	echo "Using default configuration..."
	DEVICE=lenovo_a820
fi

./mkbootfs ./"$DEVICE"/boot.img-ramdisk/ | gzip > ramdisk.gz
./mkimage ramdisk.gz ROOTFS > ramdisk.img
./mkbootimg --kernel ../out/target/product/"$DEVICE"/kernel_"$DEVICE".bin --ramdisk ramdisk.img -o boot.img

rm ramdisk.gz
rm ramdisk.img

