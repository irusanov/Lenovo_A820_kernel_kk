export PATH=$PATH:$(pwd)/../arm-eabi-4.8/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

./makeMtk -t lenovo_a820 mrproper k
./makeMtk -t lenovo_a820 c k

