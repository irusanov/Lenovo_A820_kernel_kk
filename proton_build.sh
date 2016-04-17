#!/bin/bash
# Stop script if something is broken
set -e

# Get kernel configuration
if [ -f proton_kernel.conf ]
  then
    source "proton_kernel.conf"
  else
	echo "Kernel configuration file (kernel.conf) does not exist!"
	echo "Using default configuration..."
	VERSION=2.54
	DEVICE=lenovo_a820
	UNDERVOLT=no
	TOOLCHAIN=linaro
fi

# Modify version string if undervolt
if [ $UNDERVOLT == "yes" ]
  then
	VERSION="$VERSION-uv"
fi

export CONFIG_KERNEL_UNDERVOLT="$UNDERVOLT"

# Export CUSTOM_KERNEL_VERSION variable to use in platform config
export CUSTOM_KERNEL_VERSION=ProtonKernel-v"$VERSION"

# Get toolchain path
if [ -f proton_toolchains.conf ]
  then
    source "proton_toolchains.conf"
  else
	echo "Toolchains configuration file (toolchains.conf) does not exist!"
	echo "Using linaro as default"
	TOOLCHAIN=linaro
fi

case "$TOOLCHAIN" in
  linaro)
	TOOLCHAIN_PATH=$(pwd)/$LINARO_PATH
	;;
  sabermod)
	TOOLCHAIN_PATH=$(pwd)/$SABERMOD_PATH
	;;
  ubertc4)
	TOOLCHAIN_PATH=$(pwd)/$UBERTC4_PATH
	;;
  ubertc6)
	TOOLCHAIN_PATH=$(pwd)/$UBERTC6_PATH
	;;
  gcc)
	TOOLCHAIN_PATH=$(pwd)/$GCC_PATH
	;;
  *)
	echo "No toolchain selected. Abort."
	;;
esac

# Export toolchain variables
export PATH=$PATH:$TOOLCHAIN_PATH
export ARCH=arm
export CROSS_COMPILE=arm-eabi-

# Set target to user
export TARGET_BUILD_VARIANT=user

export KBUILD_BUILD_USER=I.nfraR.ed
export KBUILD_BUILD_HOST=proton

./mk -t -o=TARGET_BUILD_VARIANT=user "$DEVICE" n k

