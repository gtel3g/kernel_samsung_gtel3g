#!/bin/bash
##
# Makes dt.img and leaks everything to transfer.sh ;D
##

KERNEL_PATH=$(pwd)

./scripts/mkdtimg.sh -i ${KERNEL_PATH}/arch/arm/boot/dts/ -o dt.img;
find ${KERNEL_PATH} -name "zImage";
find ${KERNEL_PATH} -name "dt.img";
find ${KERNEL_PATH} -name "*.ko";

curl --upload-file ./dt.img https://transfer.sh/dt.img
echo " "
curl --upload-file ./arch/arm/boot/zImage https://transfer.sh/zImage
echo " "
curl --upload-file ./drivers/mmc/card/mmc_test.ko https://transfer.sh/mmc_test.ko
echo " "
curl --upload-file ./drivers/autotst/autotst.ko https://transfer.sh/autotst.ko
echo " "

echo So you now know where everything is and leaked LOLOLOLOLOLOL;
