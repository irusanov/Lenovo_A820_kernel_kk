export PATH=$PATH:$(pwd)/../arm-eabi-4.8/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

export TARGET_BUILD_VARIANT=user

export KBUILD_BUILD_USER=I.nfraR.ed
export KBUILD_BUILD_HOST=proton

DEVICE_TREE="`cat DEVICE_TREE`"

./mk -t -o=TARGET_BUILD_VARIANT=user "$DEVICE_TREE" n k

