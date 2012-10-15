#!/bin/bash

> Test_Compile.log

export CROSS_COMPILE=/btopt/sources/NEXTGEN/M/toolc/arm-2009q1/bin/arm-none-linux-gnueabi-
export ARCH=arm

make mrproper
make basi_defconfig
make -j16 uImage && echo "Basi OK" >> Test_Compile.log 
make mrproper
make dingo_defconfig
make -j16 uImage && echo "Dingo OK" >> Test_Compile.log 
make mrproper
make amico-i_defconfig
make -j16 uImage && echo "Amico-i OK" >> Test_Compile.log 
make mrproper
make lago_defconfig
make -j16 uImage && echo "Lago OK" >> Test_Compile.log 
make mrproper
make jumbo-i_defconfig
make -j16 uImage && echo "Jumbo-i OK" >> Test_Compile.log 
make mrproper
make owl_defconfig
make -j16 uImage && echo "Owl-i OK" >> Test_Compile.log 
make mrproper
make jumbo-d_defconfig
make -j16 uImage && echo "Jumbo-d OK" >> Test_Compile.log 

