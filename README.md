# Proton Kernel KK
Proton kernel for KitKat

Supported devices:
- A820
- A830 (TBA)

# HOWTO

1. Change value in DEVICE_TREE.
   Available devices:
   - lenovo_a820
   - lenovo_a830
   
2. Execute build.sh from terminal or build-linaro.sh from terminal
3. To repack new kernel, go to mtk-tools and execute brepack.sh
4. Your new boot.img is ready in mtk-tools directory 
