##### https://gitlab.com/SaberMod/arm-eabi-4.9
##### http://packages.ubuntu.com/vivid/libisl13

export PATH=$PATH:$(pwd)/../arm-eabi-4.9.sabermod/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

# this is essential to build a working kernel!
export TARGET_BUILD_VARIANT=user

export KBUILD_BUILD_USER=I.nfraR.ed
export KBUILD_BUILD_HOST=proton

DEVICE_TREE="`cat DEVICE_TREE`"

./mk -t -o=TARGET_BUILD_VARIANT=user "$DEVICE_TREE" n k

