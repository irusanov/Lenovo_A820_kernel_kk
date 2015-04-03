export PATH=$PATH:$(pwd)/../arm-eabi-4.8/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

DEVICE_TREE="`cat DEVICE_TREE`"

./makeMtk -t "$DEVICE_TREE" mrproper k
./makeMtk -t "$DEVICE_TREE" c k

