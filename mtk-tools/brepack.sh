#!/bin/bash
#Stop script if something is broken
set -e

if [ -f ../proton_kernel.conf ]
  then
    source "../proton_kernel.conf"
  else
	echo "Kernel configuration file (kernel.conf) does not exist!"
	echo "Using default configuration..."
	DEVICE=lenovo_a820
fi

../mediatek/build/tools/images/mkbootfs ./"$DEVICE"/boot.img-ramdisk/ | ../mediatek/build/tools/images/minigzip > ramdisk.gz
../mediatek/build/tools/images/mkimage ramdisk.gz ROOTFS > ramdisk.img
../mediatek/build/tools/images/mkbootimg --kernel ../out/target/product/"$DEVICE"/kernel_"$DEVICE".bin --ramdisk ramdisk.img -o boot.img

rm ramdisk.gz
rm ramdisk.img
