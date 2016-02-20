# Proton Kernel KK
This is a custom kitkat kernel for Mediatek MT6589 platform.

## Supported devices
- Lenovo A820
- Lenovo A830

## Getting Started

#### 1. Checkout the project
master branch (3.4.67)
```
git clone https://github.com/infraredbg/Lenovo_A820_kernel_kk
```
upstream branch (3.4.110)
```
git clone https://github.com/infraredbg/Lenovo_A820_kernel_kk -b upstream
```
#### 2. Change the project name in DEVICE_TREE

Supported devices: lenovo_a820 and lenovo_a830

#### 3. Build
```
./build-linaro.sh
```

#### 4. Repack
To repack new kernel, go to mtk-tools and execute

```
./brepack.sh
```
Your new boot.img is ready in mtk-tools directory 
