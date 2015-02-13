export PATH=$PATH:$(pwd)/../arm-eabi-4.8/bin

export ARCH=arm
export CROSS_COMPILE=arm-eabi-

#Workaround for + appended on kernelrelease, may not be required
export LOCALVERSION=-Proton-Kernel-2.0-alpha

# this is essential to build a working kernel!
export TARGET_BUILD_VARIANT=user

export KBUILD_BUILD_USER=I.nfraR.ed
export KBUILD_BUILD_HOST=proton

./mk -t lenovo89_wet_td n k

