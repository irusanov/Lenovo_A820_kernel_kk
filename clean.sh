export PATH=$PATH:$(pwd)/../arm-eabi-4.8/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

#./makeMtk -t lenovo89_cu_jb mrproper k
./makeMtk -t lenovo89_wet_td mrproper k
#./makeMtk -t lenovo89_cu_jb c k
./makeMtk -t lenovo89_wet_td c k

